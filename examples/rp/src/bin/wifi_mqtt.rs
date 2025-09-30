// src/bin/wifi_mqtt.rs
//! RP Pico W + Embassy：連 Wi-Fi、DHCP 取 IPv4，之後每 60 秒送一次 MQTT（QoS0）。
//! 初始化流程、PIO IRQ 寫法與韌體路徑完全比照你現有的 wifi_webrequest.rs。
//! MQTT 實作最小 3.1.1（CONNECT / PUBLISH / PINGREQ），無外部 MQTT crate。

#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::fmt;

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use defmt::Debug2Format;
use embassy_executor::Spawner;
use embassy_net::{Config, IpAddress, StackResources};
use embassy_net::tcp::TcpSocket;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pio::{Pio, InterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::bind_interrupts;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;
// 提供 RTT 與 panic handler（和你既有 bin 相同）
use {defmt_rtt as _, panic_probe as _};

use embedded_io_async::{ErrorType, Read, Write};

// ===== 依你習慣：Wi-Fi / MQTT 參數 =====
const WIFI_NETWORK: &str   = "WAX2617";
const WIFI_PASSWORD: &str  = "7499363II5495264";

// 你指定的 broker 固定 IP
const MQTT_BROKER_IP: (u8, u8, u8, u8) = (192, 168, 188, 182);
const MQTT_PORT: u16       = 2883;
const MQTT_TOPIC: &str     = "lab/picoW/telemetry";
const MQTT_CLIENT_PREFIX: &str = "picoW";

// === 用與你專案相同的方式宣告 PIO IRQ 綁定 ===
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// ===== 背景任務（具體型別，對齊你現有專案） =====
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Pico W MQTT (raw) booting...");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    // 韌體檔（相對路徑比照你的專案）
    let fw  = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // === CYW43 / PIO / SPI：照你現有寫法（bind_interrupts! 版）===
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs  = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // === 網路堆疊（DHCPv4）：照你現有寫法 ===
    let config = Config::dhcpv4(Default::default());
    let seed = rng.next_u64();
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    unwrap!(spawner.spawn(net_task(net_runner)));

    // === 連 Wi-Fi ===
    while let Err(err) = control.join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
        info!("join failed with status={}", err.status);
    }
    info!("waiting for link...");
    stack.wait_link_up().await;

    info!("waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(cfg) = stack.config_v4() {
        info!("IPv4: {}  gw: {}", Debug2Format(&cfg.address), Debug2Format(&cfg.gateway));
    }

    // client_id（簡化：固定前綴）
    let client_id: &str = MQTT_CLIENT_PREFIX;
    info!("MQTT client_id={}", client_id);

    // 直接用固定 IP，不走 DNS
    let (a, b, c, d) = MQTT_BROKER_IP;
    let broker_ip: IpAddress = IpAddress::v4(a, b, c, d);
    info!("Broker {}.{}.{}.{} -> {}", a, b, c, d, Debug2Format(&broker_ip));

    // TCP socket buffers
    let mut rx_buf = [0u8; 1536];
    let mut tx_buf = [0u8; 1536];

    'reconnect: loop {
        let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        sock.set_timeout(Some(Duration::from_secs(10)));

        info!("Connecting TCP to MQTT...");
        if let Err(e) = sock.connect((broker_ip, MQTT_PORT)).await {
            warn!("TCP connect failed: {:?}", Debug2Format(&e));
            Timer::after(Duration::from_secs(3)).await;
            continue 'reconnect;
        }
        info!("TCP connected.");

        if let Err(e) = mqtt_send_connect(&mut sock, client_id, 75).await {
            warn!("CONNECT send failed: {:?}", Debug2Format(&e));
            let _ = sock.close();
            Timer::after(Duration::from_secs(3)).await;
            continue 'reconnect;
        }

        match mqtt_expect_connack(&mut sock).await {
            Ok(()) => info!("MQTT CONNACK ok."),
            Err(_) => {
                warn!("Bad CONNACK, reconnecting...");
                let _ = sock.close();
                Timer::after(Duration::from_secs(3)).await;
                continue 'reconnect;
            }
        }

        let mut last_ping = Instant::now();

        loop {
            // —— 每 60 秒送一次（寫死資料） ——
            let payload = build_json_payload();
            match mqtt_publish_qos0(&mut sock, MQTT_TOPIC, payload.as_bytes()).await {
                Ok(_) => info!("PUBLISH ok: {}", payload.as_str()),
                Err(e) => {
                    warn!("PUBLISH failed: {:?}", Debug2Format(&e));
                    break;
                }
            }

            // 30 秒保活：PINGREQ
            while Instant::now() - last_ping < Duration::from_secs(30) {
                // 吞掉可能的回應（非阻塞）
                let _ = sock.read(&mut [0u8; 64]).await;
                Timer::after(Duration::from_millis(200)).await;
            }
            last_ping = Instant::now();
            if let Err(e) = mqtt_pingreq(&mut sock).await {
                warn!("PINGREQ failed: {:?}", Debug2Format(&e));
                break;
            }

            // 等滿 60 秒再送下一筆
            let start = Instant::now();
            while Instant::now() - start < Duration::from_secs(60) {
                let _ = sock.read(&mut [0u8; 64]).await;
                Timer::after(Duration::from_millis(200)).await;
            }
        }

        let _ = sock.close();
        info!("Reconnecting...");
        Timer::after(Duration::from_secs(2)).await;
    }
}

// ===== 固定欄位（先寫死）→ JSON =====
fn build_json_payload() -> heapless::String<256> {
    let light_on = true;            // 燈開關
    let heater_coil_ma = 182.0_f32; // 暖風機交流線圈電流 (mA)
    let temp_c = 26.5_f32;          // 溫度
    let rh = 64.0_f32;              // 相對溼度

    let mut s = heapless::String::<256>::new();
    let _ = fmt::write(
        &mut s,
        format_args!(
            "{{\"light\":{},\"heater_coil_mA\":{:.1},\"temp_c\":{:.1},\"rh\":{:.1}}}",
            if light_on { "true" } else { "false" }, heater_coil_ma, temp_c, rh
        ),
    );
    s
}

// ===== 超小型 MQTT 3.1.1 封包 =====

async fn mqtt_send_connect<S: Write + Read + ErrorType>(
    sock: &mut S, client_id: &str, keep_alive_s: u16
) -> Result<(), S::Error> {
    let protocol_name = "MQTT";
    let protocol_level = 0x04u8; // 3.1.1
    let connect_flags = 0b0000_0010u8; // Clean Session
    let keep_alive = keep_alive_s.to_be_bytes();

    // 固定標頭：CONNECT
    let mut hdr = heapless::Vec::<u8, 128>::new();
    hdr.push(0x10).ok();
    encode_rem_len((2 + protocol_name.len() + 1 + 1 + 2 + 2 + client_id.len()) as u32, &mut hdr);

    // 可變標頭
    push_str(&mut hdr, protocol_name);
    hdr.push(protocol_level).ok();
    hdr.push(connect_flags).ok();
    hdr.extend_from_slice(&keep_alive).ok();

    // 載荷：Client ID
    push_str(&mut hdr, client_id);

    sock.write_all(&hdr).await
}

async fn mqtt_expect_connack<S: Read + ErrorType>(sock: &mut S) -> Result<(), S::Error> {
    // 手動讀到 4 bytes（避免 ReadExactError 的 From bound 相容性問題）
    let mut buf = [0u8; 4];
    let mut got = 0usize;
    while got < 4 {
        match sock.read(&mut buf[got..]).await {
            Ok(0) => break,
            Ok(n) => got += n,
            Err(e) => return Err(e),
        }
    }
    if got == 4 && buf[0] == 0x20 && buf[1] == 0x02 && buf[2] == 0x00 && buf[3] == 0x00 {
        Ok(())
    } else {
        // 回傳個不可達型別，讓上層走重連路徑
        Err(unsafe { core::mem::MaybeUninit::uninit().assume_init() })
    }
}

async fn mqtt_publish_qos0<S: Write + ErrorType>(
    sock: &mut S, topic: &str, payload: &[u8]
) -> Result<(), S::Error> {
    let mut hdr = heapless::Vec::<u8, 256>::new();
    hdr.push(0x30).ok(); // PUBLISH QoS0
    encode_rem_len((2 + topic.len() + payload.len()) as u32, &mut hdr);
    push_str(&mut hdr, topic);
    sock.write_all(&hdr).await?;
    sock.write_all(payload).await
}

async fn mqtt_pingreq<S: Write + ErrorType>(sock: &mut S) -> Result<(), S::Error> {
    sock.write_all(&[0xC0, 0x00]).await
}

fn push_str<const N: usize>(v: &mut heapless::Vec<u8, N>, s: &str) {
    let len = s.len();
    v.extend_from_slice(&(len as u16).to_be_bytes()).ok();
    v.extend_from_slice(s.as_bytes()).ok();
}

fn encode_rem_len<const N: usize>(mut x: u32, out: &mut heapless::Vec<u8, N>) {
    loop {
        let mut byte = (x % 128) as u8;
        x /= 128;
        if x > 0 { byte |= 0x80; }
        out.push(byte).ok();
        if x == 0 { break; }
    }
}
