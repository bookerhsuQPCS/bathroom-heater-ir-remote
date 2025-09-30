// src/bin/wifi_mqtt.rs
//! RP Pico W + Embassy：Wi-Fi + DHCP，固定 IP broker，每 60 秒 PUBLISH。
//! 強化：KeepAlive=30s，PINGREQ 每 15s 並等待 PINGRESP，避免中間鏈路因 idle 收線。
//! Pin22 LED：連上（收 CONNACK）閃兩下；每次 PUBLISH 成功閃一下。

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
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use embedded_io_async::{ErrorType, Read, Write};
use embassy_rp::bind_interrupts;

// ===== 你的環境參數 =====
const WIFI_NETWORK: &str = "WAX2617";
const WIFI_PASSWORD: &str = "7499363II5495264";

// Broker 固定在 192.168.1.x（你環境限制在這段）
const MQTT_BROKER_IP: (u8, u8, u8, u8) = (192, 168, 188, 182);
// 依你的 NanoMQ 設定調整：你貼的 conf 是 2883（如改回 1883 請同步改數值）
const MQTT_PORT: u16 = 2883;

const MQTT_TOPIC: &str = "lab/picoW/telemetry";
const MQTT_CLIENT_PREFIX: &str = "picoW";

// 週期設定
const PUBLISH_EVERY: Duration = Duration::from_secs(60);
// MQTT KeepAlive（秒）：中繼/路由常對 idle 嚴格；我們 15 秒 ping 一次就穩了
const KEEP_ALIVE_S: u16 = 30;
const PING_EVERY: Duration = Duration::from_secs(15);
const PINGRESP_WAIT: Duration = Duration::from_secs(10);

// PIO IRQ 綁定（比照你既有檔）
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// ---- 背景任務（具體型別） ----
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
    info!("Pico W MQTT booting...");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    // LED on PIN22（高亮低滅）
    let mut led = Output::new(p.PIN_22, Level::Low);

    // CYW43 韌體檔（相對路徑比照你的專案）
    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // === CYW43 / PIO / SPI：照你現有寫法（bind_interrupts! 版）===
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
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

    // 關省電（Performance），避免握手/保活被睡死
    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    // === 網路堆疊（DHCPv4）===
    let config = Config::dhcpv4(Default::default());
    let seed = rng.next_u64();
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, net_runner) =
        embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), seed);
    unwrap!(spawner.spawn(net_task(net_runner)));

    // === 連 Wi-Fi ===
    while let Err(err) = control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        info!("join failed with status={}", err.status);
    }
    info!("waiting for link...");
    stack.wait_link_up().await;

    info!("waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(cfg) = stack.config_v4() {
        info!(
            "IPv4: {}  gw: {}",
            Debug2Format(&cfg.address),
            Debug2Format(&cfg.gateway)
        );
    }

    let client_id: &str = MQTT_CLIENT_PREFIX;
    let (a, b, c, d) = MQTT_BROKER_IP;
    let broker_ip: IpAddress = IpAddress::v4(a, b, c, d);
    info!(
        "Broker {}.{}.{}.{}:{}  keepalive={}s ping_every={}s",
        a,
        b,
        c,
        d,
        MQTT_PORT,
        KEEP_ALIVE_S,
        PING_EVERY.as_millis() / 1000
    );

    // TCP socket buffers
    let mut rx_buf = [0u8; 1536];
    let mut tx_buf = [0u8; 1536];

    'reconnect: loop {
        let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        // 適度逾時，避免永遠卡住
        sock.set_timeout(Some(Duration::from_secs(15)));

        info!("Connecting TCP to MQTT...");
        if let Err(e) = sock.connect((broker_ip, MQTT_PORT)).await {
            warn!("TCP connect failed: {:?}", Debug2Format(&e));
            Timer::after(Duration::from_secs(3)).await;
            continue 'reconnect;
        }
        info!("TCP connected.");

        if let Err(e) = mqtt_send_connect(&mut sock, client_id, KEEP_ALIVE_S).await {
            warn!("CONNECT send failed: {:?}", Debug2Format(&e));
            let _ = sock.close();
            Timer::after(Duration::from_secs(3)).await;
            continue 'reconnect;
        }

        match mqtt_expect_connack(&mut sock).await {
            Ok(()) => {
                info!("MQTT CONNACK ok.");
                // 連線成功：Pin22 閃兩下
                for _ in 0..2 {
                    led.set_high();
                    Timer::after(Duration::from_millis(80)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(80)).await;
                }
            }
            Err(_) => {
                warn!("Bad CONNACK, reconnecting...");
                let _ = sock.close();
                Timer::after(Duration::from_secs(3)).await;
                continue 'reconnect;
            }
        }

        // ===== 主循環：每 60s 發一次；中間每 15s ping 並等待 PINGRESP =====
        loop {
            // 先發一筆固定 JSON
            let payload = build_json_payload();
            match mqtt_publish_qos0(&mut sock, MQTT_TOPIC, payload.as_bytes()).await {
                Ok(_) => {
                    info!("PUBLISH ok: {}", payload.as_str());
                    // 發送成功：Pin22 閃一下
                    led.set_high();
                    Timer::after(Duration::from_millis(50)).await;
                    led.set_low();
                }
                Err(e) => {
                    warn!("PUBLISH failed: {:?}", Debug2Format(&e));
                    break;
                }
            }

            let start = Instant::now();
            while Instant::now() - start < PUBLISH_EVERY {
                Timer::after(PING_EVERY).await;

                if let Err(e) = mqtt_pingreq(&mut sock).await {
                    warn!("PINGREQ failed (send): {:?}", Debug2Format(&e));
                    break;
                }

                match mqtt_expect_pingresp(&mut sock, PINGRESP_WAIT).await {
                    Ok(true) => { /* 收到 PINGRESP，持續存活 */ }
                    Ok(false) => {
                        warn!("PINGRESP timeout/mismatch, reconnecting...");
                        break;
                    }
                    Err(e) => {
                        warn!("PINGRESP read error: {:?}", Debug2Format(&e));
                        break;
                    }
                }
            }
        }

        let _ = sock.close();
        info!("Reconnecting...");
        Timer::after(Duration::from_secs(2)).await;
    }
}

// ===== 固定欄位（先寫死）→ JSON =====
fn build_json_payload() -> heapless::String<256> {
    let light_on = true; // 燈開關
    let heater_coil_ma = 182.0_f32; // 暖風機交流線圈電流 (mA)
    let temp_c = 26.5_f32; // 溫度
    let rh = 64.0_f32; // 相對溼度

    let mut s = heapless::String::<256>::new();
    let _ = fmt::write(
        &mut s,
        format_args!(
            "{{\"light\":{},\"heater_coil_mA\":{:.1},\"temp_c\":{:.1},\"rh\":{:.1}}}",
            if light_on { "true" } else { "false" },
            heater_coil_ma,
            temp_c,
            rh
        ),
    );
    s
}

// ===== 超小型 MQTT 3.1.1 封包 =====

async fn mqtt_send_connect<S: Write + Read + ErrorType>(
    sock: &mut S,
    client_id: &str,
    keep_alive_s: u16,
) -> Result<(), S::Error> {
    let protocol_name = "MQTT";
    let protocol_level = 0x04u8; // 3.1.1
    let connect_flags = 0b0000_0010u8; // Clean Session
    let keep_alive = keep_alive_s.to_be_bytes();

    let mut hdr = heapless::Vec::<u8, 128>::new();
    hdr.push(0x10).ok(); // CONNECT
    encode_rem_len(
        (2 + protocol_name.len() + 1 + 1 + 2 + 2 + client_id.len()) as u32,
        &mut hdr,
    );
    push_str(&mut hdr, protocol_name);
    hdr.push(protocol_level).ok();
    hdr.push(connect_flags).ok();
    hdr.extend_from_slice(&keep_alive).ok();
    push_str(&mut hdr, client_id);

    sock.write_all(&hdr).await
}

async fn mqtt_expect_connack<S: Read + ErrorType>(sock: &mut S) -> Result<(), S::Error> {
    // 讀 4 bytes：0x20 0x02 0x00 0x00
    let mut buf = [0u8; 4];
    let mut got = 0usize;
    while got < 4 {
        match sock.read(&mut buf[got..]).await {
            Ok(0) => break, // 連線關閉
            Ok(n) => got += n,
            Err(e) => return Err(e),
        }
    }
    if got == 4 && buf[0] == 0x20 && buf[1] == 0x02 && buf[2] == 0x00 && buf[3] == 0x00 {
        Ok(())
    } else {
        // 回傳不可達型別，讓上層走重連路徑
        Err(unsafe { core::mem::MaybeUninit::uninit().assume_init() })
    }
}

async fn mqtt_publish_qos0<S: Write + ErrorType>(
    sock: &mut S,
    topic: &str,
    payload: &[u8],
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

// 等待 PINGRESP（0xD0, 0x00），限時 timeout
async fn mqtt_expect_pingresp<S: Read + ErrorType>(
    sock: &mut S,
    timeout: Duration,
) -> Result<bool, S::Error> {
    let deadline = Instant::now() + timeout;
    let mut buf = [0u8; 2];
    let mut got = 0usize;
    while got < 2 && Instant::now() < deadline {
        match sock.read(&mut buf[got..]).await {
            Ok(0) => break, // 連線關閉
            Ok(n) => got += n,
            Err(e) => return Err(e),
        }
    }
    Ok(got == 2 && buf[0] == 0xD0 && buf[1] == 0x00)
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
        if x > 0 {
            byte |= 0x80;
        }
        out.push(byte).ok();
        if x == 0 {
            break;
        }
    }
}
