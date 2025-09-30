// src/bin/wifi_mqtt_btn_led_ds18b20.rs
#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::fmt;

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use defmt::Debug2Format;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Config, IpAddress, StackResources};
use embassy_net::tcp::TcpSocket;
use embassy_rp::gpio::{Flex, Input, Level, Output, Pull};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_io_async::{ErrorType, Read, Write};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_rp::bind_interrupts;
use embedded_hal_1::delay::DelayNs as _; // 以 ns 為單位的延遲

// ===== 你的環境參數 =====
const WIFI_NETWORK: &str = "WAX2617";
const WIFI_PASSWORD: &str = "7499363II5495264";

// Broker 在 192.168.188.x 網段（依你的現況）
const MQTT_BROKER_IP: (u8, u8, u8, u8) = (192, 168, 188, 182);
const MQTT_PORT: u16 = 2883;

const MQTT_TOPIC: &str = "lab/picoW/telemetry";
const MQTT_CLIENT_PREFIX: &str = "picoW";

// 週期設定
const PUBLISH_EVERY: Duration = Duration::from_secs(60);
const KEEP_ALIVE_S: u16 = 45;
const PING_EVERY: Duration = Duration::from_secs(15);
const PINGRESP_WAIT: Duration = Duration::from_secs(8);

// 腳位：按鈕 GP15（低為按下）、LED GP22（高亮）、DS18B20 DQ 在 GP13（外掛 4.7k 上拉）
const BUTTON_IS_ACTIVE_LOW: bool = true;

// DS18B20：12-bit 轉換 750ms，抓 800ms 保守（僅作為最長等待上限的參考）
const DS18B20_CONVERT_DEADLINE_MS: u64 = 900;
// 平均樣本數（可調成 10）
const DS18B20_AVG_SAMPLES: usize = 3;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

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
    info!("Pico W: BTN GP15 → LED GP22 → DS18B20@GP13 → MQTT");

    let p = embassy_rp::init(Default::default());

    // CYW43 韌體（依你專案路徑調整）
    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // === CYW43 / PIO / SPI ===
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24, // CLK
        p.PIN_29, // DIO
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut cyw, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    cyw.init(clm).await;
    cyw.set_power_management(cyw43::PowerManagementMode::Performance).await;

    // === 網路堆疊（DHCP）===
    let config = Config::dhcpv4(Default::default());
    static RES: StaticCell<StackResources<5>> = StaticCell::new();
    let seed: u64 = 0x0123_4567_89ab_cdef;
    let (stack, net_runner) = embassy_net::new(net_device, config, RES.init(StackResources::new()), seed);
    unwrap!(spawner.spawn(net_task(net_runner)));

    // === I/O ===
    let btn = Input::new(p.PIN_15, Pull::Up);            // 低為按下
    let mut usr_led = Output::new(p.PIN_22, Level::Low); // 高亮
    let mut ow = Flex::new(p.PIN_13);                    // DS18B20 DQ
    ow.set_pull(Pull::Up); // 仍需外掛 4.7k 上拉到 3V3

    // === Wi-Fi join ===
    while let Err(err) = cyw.join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
        info!("WiFi join failed: {}", err.status);
        Timer::after(Duration::from_secs(2)).await;
    }
    info!("Waiting link...");
    stack.wait_link_up().await;

    info!("Waiting DHCP...");
    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        info!("IPv4: {} gw: {}", Debug2Format(&cfg.address), Debug2Format(&cfg.gateway));
    }

    let client_id = MQTT_CLIENT_PREFIX;
    let (a, b, c, d) = MQTT_BROKER_IP;
    let broker_ip: IpAddress = IpAddress::v4(a, b, c, d);
    info!("Broker {}.{}.{}.{}:{} keepalive={}s", a, b, c, d, MQTT_PORT, KEEP_ALIVE_S);

    // TCP buffers
    let mut rx_buf = [0u8; 1536];
    let mut tx_buf = [0u8; 1536];

    'reconnect: loop {
        let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
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
        if mqtt_expect_connack(&mut sock).await.is_ok() {
            info!("MQTT CONNACK ok.");
            // 連線成功：CYW43 板載 LED 雙閃
            for _ in 0..2 {
                let _ = cyw.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(80)).await;
                let _ = cyw.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(80)).await;
            }

            // 一次性：Dump Scratchpad 幫你確認 1-Wire 狀態
            if let Ok(sp) = ds18b20_dump_scratchpad_once(&mut ow).await {
                let crc = maxim_crc8(&sp[0..8]);
                info!("Scratchpad = {:?}  calc_crc={}  ok={}", Debug2Format(&sp), crc, crc == sp[8]);
            } else {
                warn!("READ SCRATCHPAD failed");
            }
        } else {
            warn!("Bad CONNACK, reconnecting...");
            let _ = sock.close();
            Timer::after(Duration::from_secs(2)).await;
            continue 'reconnect;
        }

        // ===== 主循環：定時 + 按鈕即時 =====
        let mut btn_latched = false;
        let mut next_ping_at = Instant::now() + PING_EVERY;

        loop {
            // ---- 例行：每 60 秒發一筆 ----
            {
                if !stack.is_link_up() {
                    break; // 交給外側重連
                }
                let temp_res = ds18b20_read_avg(&mut ow, &mut usr_led).await;
                let payload = build_json_payload(temp_res);
                if mqtt_publish_qos0(&mut sock, MQTT_TOPIC, payload.as_bytes()).await.is_ok() {
                    info!("PUBLISH ok: {}", payload.as_str());
                } else {
                    warn!("PUBLISH failed");
                    break;
                }
            }

            // 在下一次例行上傳前：輪詢按鈕 + ping
            let period_start = Instant::now();
            while Instant::now() - period_start < PUBLISH_EVERY {
                // ----- 按鈕（去抖 20ms，一次觸發）-----
                let pressed_now = if BUTTON_IS_ACTIVE_LOW { !btn.is_high() } else { btn.is_high() };
                if pressed_now && !btn_latched {
                    Timer::after(Duration::from_millis(20)).await;
                    let still = if BUTTON_IS_ACTIVE_LOW { !btn.is_high() } else { btn.is_high() };
                    if still {
                        btn_latched = true;
                        if !stack.is_link_up() { break; }
                        let t_res = ds18b20_read_avg(&mut ow, &mut usr_led).await;
                        let payload2 = build_json_payload(t_res);
                        match mqtt_publish_qos0(&mut sock, MQTT_TOPIC, payload2.as_bytes()).await {
                            Ok(_) => info!("PUBLISH (button) ok: {}", payload2.as_str()),
                            Err(e) => { warn!("PUBLISH (button) failed: {:?}", Debug2Format(&e)); break; }
                        }
                    }
                } else if !pressed_now {
                    btn_latched = false;
                }

                // ----- MQTT KeepAlive -----
                if Instant::now() >= next_ping_at {
                    next_ping_at += PING_EVERY;
                    if let Err(e) = mqtt_pingreq(&mut sock).await {
                        warn!("PINGREQ send failed: {:?}", Debug2Format(&e));
                        break;
                    }
                    match mqtt_expect_pingresp(&mut sock, PINGRESP_WAIT).await {
                        Ok(true) => { /* ok */ }
                        _ => { warn!("PINGRESP timeout/mismatch"); break; }
                    }
                }

                Timer::after(Duration::from_millis(20)).await;
            }
        }

        let _ = sock.close();
        info!("Reconnecting...");
        Timer::after(Duration::from_secs(2)).await;
    }
}

// ===== JSON （四欄位，沒有 sensor_ok；失敗時 temp_c=null）=====
fn build_json_payload(temp_res: Result<f32, ()>) -> heapless::String<256> {
    let light_on = true;
    let heater_coil_ma = 182.0;
    let rh = 64.0;

    let mut s = heapless::String::<256>::new();
    let _ = fmt::write(
        &mut s,
        format_args!(
            "{{\"light\":{},\"heater_coil_mA\":{:.1},",
            if light_on { "true" } else { "false" },
            heater_coil_ma
        ),
    );

    match temp_res {
        Ok(t) if t.is_finite() => {
            let _ = fmt::write(&mut s, format_args!("\"temp_c\":{:.2},", t));
        }
        _ => {
            let _ = fmt::write(&mut s, format_args!("\"temp_c\":null,"));
        }
    }

    let _ = fmt::write(&mut s, format_args!("\"rh\":{:.1}}}", rh));
    s
}

// ===== DS18B20：avg + 完成位輪詢，避免 85°C 舊值 =====
async fn ds18b20_read_avg(pin: &mut Flex<'_>, led: &mut Output<'_>) -> Result<f32, ()> {
    led.set_high();

    let mut sum = 0.0f32;
    let mut ok = 0usize;

    for _ in 0..DS18B20_AVG_SAMPLES {
        match ds18b20_read_once(pin).await {
            Ok(t) => { sum += t; ok += 1; }
            Err(_) => {}
        }
        Timer::after(Duration::from_millis(5)).await;
        yield_now().await; // 讓網路任務喘口氣
    }

    led.set_low();

    if ok == 0 { Err(()) } else { Ok(sum / (ok as f32)) }
}

async fn ds18b20_dump_scratchpad_once(pin: &mut Flex<'_>) -> Result<[u8; 9], ()> {
    let mut d = Delay;

    if !ow_reset(pin, &mut d) { return Err(()); }
    ow_write_byte(pin, 0xCC, &mut d); // SKIP ROM
    ow_write_byte(pin, 0x44, &mut d); // CONVERT T

    // 等轉換完成（輪詢完成位 + 最長期限）
    let deadline = Instant::now() + Duration::from_millis(DS18B20_CONVERT_DEADLINE_MS);
    loop {
        if ow_read_bit(pin, &mut d) { break; } // 完成回 1
        Timer::after(Duration::from_millis(2)).await;
        yield_now().await;
        if Instant::now() >= deadline { return Err(()); }
    }

    if !ow_reset(pin, &mut d) { return Err(()); }
    ow_write_byte(pin, 0xCC, &mut d); // SKIP ROM
    ow_write_byte(pin, 0xBE, &mut d); // READ SCRATCHPAD

    let mut data = [0u8; 9];
    for i in 0..9 { data[i] = ow_read_byte(pin, &mut d); }
    yield_now().await;
    Ok(data)
}

async fn ds18b20_read_once(pin: &mut Flex<'_>) -> Result<f32, ()> {
    let mut d = Delay;

    if !ow_reset(pin, &mut d) { return Err(()); }

    // SKIP ROM, CONVERT T
    ow_write_byte(pin, 0xCC, &mut d);
    ow_write_byte(pin, 0x44, &mut d);

    // 輪詢完成位 + 最長期限，避免 85°C 舊值
    let deadline = Instant::now() + Duration::from_millis(DS18B20_CONVERT_DEADLINE_MS);
    loop {
        if ow_read_bit(pin, &mut d) { break; } // 完成回 1
        Timer::after(Duration::from_millis(2)).await;
        yield_now().await;
        if Instant::now() >= deadline { return Err(()); }
    }

    if !ow_reset(pin, &mut d) { return Err(()); }
    // SKIP ROM + READ SCRATCHPAD
    ow_write_byte(pin, 0xCC, &mut d);
    ow_write_byte(pin, 0xBE, &mut d);

    let mut data = [0u8; 9];
    for i in 0..9 { data[i] = ow_read_byte(pin, &mut d); }
    yield_now().await;

    if maxim_crc8(&data[0..8]) != data[8] { return Err(()); }

    let raw = ((data[1] as i16) << 8) | (data[0] as i16);
    let temp_c = (raw as f32) * 0.0625;

    // 85°C = 未完成 / 舊值，視為錯誤
    if (temp_c - 85.0).abs() < 0.01 {
        return Err(());
    }

    Ok(temp_c)
}

// --- 1-Wire primitives（改用 ns，presence 取樣更保守，讀位元 12µs） ---
fn ow_reset(pin: &mut Flex<'_>, d: &mut Delay) -> bool {
    pin.set_as_output();
    pin.set_low();
    d.delay_ns(560_000); // 480–960us
    pin.set_as_input();
    d.delay_ns(80_000); // presence 取樣點（原 70us → 80us，更保守）
    let presence = !pin.is_high();
    d.delay_ns(420_000); // 補滿時槽
    presence
}

fn ow_write_bit(pin: &mut Flex<'_>, bit: bool, d: &mut Delay) {
    pin.set_as_output();
    pin.set_low();
    if bit {
        d.delay_ns(6_000);
        pin.set_as_input();
        d.delay_ns(64_000);
    } else {
        d.delay_ns(60_000);
        pin.set_as_input();
        d.delay_ns(10_000);
    }
}

fn ow_read_bit(pin: &mut Flex<'_>, d: &mut Delay) -> bool {
    pin.set_as_output();
    pin.set_low();
    d.delay_ns(6_000);
    pin.set_as_input();
    d.delay_ns(12_000); // 提早取樣（可視情況改 11_000）
    let high = pin.is_high();
    d.delay_ns(60_000); // 補滿時槽
    high
}

fn ow_write_byte(pin: &mut Flex<'_>, mut b: u8, d: &mut Delay) {
    for _ in 0..8 {
        ow_write_bit(pin, (b & 1) != 0, d);
        b >>= 1;
    }
}

fn ow_read_byte(pin: &mut Flex<'_>, d: &mut Delay) -> u8 {
    let mut v = 0u8;
    for i in 0..8 {
        if ow_read_bit(pin, d) { v |= 1 << i; }
    }
    v
}

// Maxim/DS18B20 CRC8（反射多項式 0x8C）
fn maxim_crc8(bytes: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in bytes {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 1) != 0 { crc = (crc >> 1) ^ 0x8C; } else { crc >>= 1; }
        }
    }
    crc
}

// ===== 極簡 MQTT =====
async fn mqtt_send_connect<S: Write + Read + ErrorType>(
    sock: &mut S,
    client_id: &str,
    keep_alive_s: u16,
) -> Result<(), S::Error> {
    let protocol_name = "MQTT";
    let protocol_level = 0x04u8;
    let connect_flags = 0b0000_0010u8; // Clean Session
    let keep_alive = keep_alive_s.to_be_bytes();

    let mut hdr = heapless::Vec::<u8, 128>::new();
    hdr.push(0x10).ok(); // CONNECT
    encode_rem_len((2 + protocol_name.len() + 1 + 1 + 2 + 2 + client_id.len()) as u32, &mut hdr);
    push_str(&mut hdr, protocol_name);
    hdr.push(protocol_level).ok();
    hdr.push(connect_flags).ok();
    hdr.extend_from_slice(&keep_alive).ok();
    push_str(&mut hdr, client_id);

    sock.write_all(&hdr).await
}

async fn mqtt_expect_connack<S: Read + ErrorType>(sock: &mut S) -> Result<(), S::Error> {
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

async fn mqtt_expect_pingresp<S: Read + ErrorType>(
    sock: &mut S,
    timeout: Duration,
) -> Result<bool, S::Error> {
    let deadline = Instant::now() + timeout;
    let mut buf = [0u8; 2];
    let mut got = 0usize;
    while got < 2 && Instant::now() < deadline {
        match sock.read(&mut buf[got..]).await {
            Ok(0) => break,
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
        if x > 0 { byte |= 0x80; }
        out.push(byte).ok();
        if x == 0 { break; }
    }
}
