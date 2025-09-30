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
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{ErrorType, Read, Write};
use static_cell::StaticCell;

// RTT + panic
use {defmt_rtt as _, panic_probe as _};

// I2C
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
// 讓 I2c 有 async .write/.read 方法
use embedded_hal_async::i2c::I2c as _;

// ================== 可調參數 ==================
const WIFI_SSID: &str = "WAX2617";
const WIFI_PASS: &str = "7499363II5495264";

// Broker
const MQTT_BROKER_IP: (u8, u8, u8, u8) = (192, 168, 188, 182);
const MQTT_PORT: u16 = 2883;

// Topic 與版本號
const MQTT_TOPIC: &str = "lab/picoW/telemetry";
const FW_VER: &str = "htu21d-1.0.0";      // JSON 與 client_id 都會附帶
const MQTT_CLIENT_PREFIX: &str = "picoW"; // 會變成 picoW-htu21d-1.0.0

// 發佈週期與 keepalive
const PUBLISH_EVERY: Duration = Duration::from_secs(60);
const KEEP_ALIVE_S: u16 = 45;
const PING_EVERY: Duration = Duration::from_secs(15);
const PINGRESP_WAIT: Duration = Duration::from_secs(8);

// I/O：按鈕 GP15（低為按下）、LED GP22（高亮）
const BUTTON_IS_ACTIVE_LOW: bool = true;

// HTU21D/SHT21 I2C（I2C0：SCL=GP5、SDA=GP4）
const HTU21D_ADDR: u8 = 0x40;
const CMD_TEMP_NOHOLD: u8 = 0xF3;
const CMD_RH_NOHOLD: u8 = 0xF5;
const CMD_SOFT_RESET: u8 = 0xFE;
// 典型轉換時間（保守）
const HTU_T_WAIT_MS: u64 = 60;
const HTU_RH_WAIT_MS: u64 = 25;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
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
    info!("Pico W + HTU21D (fw={})  BTN=GP15  LED=GP22  I2C0(SCL=GP5,SDA=GP4)", FW_VER);

    let p = embassy_rp::init(Default::default());

    // CYW43 firmware 路徑請依你的專案調整
    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // === CYW43 / PIO SPI ===
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

    // === Button / LED ===
    let btn = Input::new(p.PIN_15, Pull::Up);            // 低為按下
    let mut usr_led = Output::new(p.PIN_22, Level::Low); // 高亮

    // === I2C0：SCL=GP5、SDA=GP4 ===
    let mut i2c = {
        let cfg = I2cConfig::default();
        // 若要 400kHz：let mut cfg = I2cConfig::default(); cfg.frequency = 400_000;
        I2c::new_async(p.I2C0, p.PIN_5 /* SCL */, p.PIN_4 /* SDA */, Irqs, cfg)
    };

    // 軟重置 HTU21D
    let _ = i2c.write(HTU21D_ADDR, &[CMD_SOFT_RESET]).await;
    Timer::after(Duration::from_millis(15)).await; // datasheet：≥15ms

    // === Wi-Fi join ===
    while let Err(err) = cyw.join(WIFI_SSID, JoinOptions::new(WIFI_PASS.as_bytes())).await {
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

    // client_id = prefix-版本號
    let mut client_id_buf = heapless::String::<64>::new();
    let _ = fmt::write(&mut client_id_buf, format_args!("{}-{}", MQTT_CLIENT_PREFIX, FW_VER));
    let client_id = client_id_buf.as_str();

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
            // 連線成功：CYW43 LED 雙閃
            for _ in 0..2 {
                let _ = cyw.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(80)).await;
                let _ = cyw.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(80)).await;
            }
        } else {
            warn!("Bad CONNACK, reconnecting...");
            let _ = sock.close();
            Timer::after(Duration::from_secs(2)).await;
            continue 'reconnect;
        }

        let mut btn_latched = false;
        let mut next_ping_at = Instant::now() + PING_EVERY;

        loop {
            // ---- 例行：每 PUBLISH_EVERY 秒量測並發佈 ----
            {
                if !stack.is_link_up() { break; }

                usr_led.set_high();
                let (t_res, rh_res) = htu21d_read_temp_rh(&mut i2c).await;
                usr_led.set_low();

                log_htu_reading(&t_res, &rh_res);

                let payload = build_json_payload(t_res, rh_res);
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

                        usr_led.set_high();
                        let (t2, rh2) = htu21d_read_temp_rh(&mut i2c).await;
                        usr_led.set_low();

                        log_htu_reading(&t2, &rh2);

                        let payload2 = build_json_payload(t2, rh2);
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

// ================== 顯示量測結果（避免 defmt 精度格式，先組字串） ==================
fn log_htu_reading(temp: &Result<f32, ()>, rh: &Result<f32, ()>) {
    let mut line = heapless::String::<64>::new();
    match (temp, rh) {
        (Ok(t), Ok(h)) => {
            let _ = fmt::write(&mut line, format_args!("HTU21D => temp={:.2} °C, rh={:.1} %", t, h));
            info!("{}", line.as_str());
        }
        (Ok(t), Err(_)) => {
            let _ = fmt::write(&mut line, format_args!("HTU21D => temp={:.2} °C, rh=FAIL", t));
            warn!("{}", line.as_str());
        }
        (Err(_), Ok(h)) => {
            let _ = fmt::write(&mut line, format_args!("HTU21D => temp=FAIL, rh={:.1} %", h));
            warn!("{}", line.as_str());
        }
        (Err(_), Err(_)) => {
            warn!("HTU21D => read FAIL");
        }
    }
}

// ================== JSON（含 fw_ver；失敗→ temp_c/rh=null） ==================
fn build_json_payload(temp_res: Result<f32, ()>, rh_res: Result<f32, ()>) -> heapless::String<256> {
    let light_on = true;
    let heater_coil_ma = 182.0;

    let mut s = heapless::String::<256>::new();
    let _ = fmt::write(
        &mut s,
        format_args!(
            "{{\"fw_ver\":\"{}\",\"light\":{},\"heater_coil_mA\":{:.1},",
            FW_VER,
            if light_on { "true" } else { "false" },
            heater_coil_ma
        ),
    );

    match temp_res {
        Ok(t) if t.is_finite() => { let _ = fmt::write(&mut s, format_args!("\"temp_c\":{:.2},", t)); }
        _ => { let _ = fmt::write(&mut s, format_args!("\"temp_c\":null,")); }
    }

    match rh_res {
        Ok(rh) if rh.is_finite() => { let _ = fmt::write(&mut s, format_args!("\"rh\":{:.1}}}", rh)); }
        _ => { let _ = fmt::write(&mut s, format_args!("\"rh\":null}}")); }
    }

    s
}

// ================== HTU21D 量測（no hold + 固定等待 + CRC 驗證） ==================
async fn htu21d_read_temp_rh(
    i2c: &mut I2c<'_, I2C0, embassy_rp::i2c::Async>
) -> (Result<f32, ()>, Result<f32, ()>) {
    let t = htu21d_read_temp(i2c).await;
    let rh = htu21d_read_rh(i2c).await;
    (t, rh)
}

async fn htu21d_read_temp(
    i2c: &mut I2c<'_, I2C0, embassy_rp::i2c::Async>
) -> Result<f32, ()> {
    if i2c.write(HTU21D_ADDR, &[CMD_TEMP_NOHOLD]).await.is_err() { return Err(()); }
    Timer::after(Duration::from_millis(HTU_T_WAIT_MS)).await;

    let mut buf = [0u8; 3];
    if i2c.read(HTU21D_ADDR, &mut buf).await.is_err() { return Err(()); }
    if !sensirion_crc_ok(&buf[0..2], buf[2]) {
        warn!("HTU21D temp CRC error: {:?}", Debug2Format(&buf));
        return Err(());
    }

    let raw = (((buf[0] as u16) << 8) | (buf[1] as u16)) & !0x0003;
    let t_c = -46.85 + 175.72 * (raw as f32) / 65536.0;
    Ok(t_c)
}

async fn htu21d_read_rh(
    i2c: &mut I2c<'_, I2C0, embassy_rp::i2c::Async>
) -> Result<f32, ()> {
    if i2c.write(HTU21D_ADDR, &[CMD_RH_NOHOLD]).await.is_err() { return Err(()); }
    Timer::after(Duration::from_millis(HTU_RH_WAIT_MS)).await;

    let mut buf = [0u8; 3];
    if i2c.read(HTU21D_ADDR, &mut buf).await.is_err() { return Err(()); }
    if !sensirion_crc_ok(&buf[0..2], buf[2]) {
        warn!("HTU21D RH CRC error: {:?}", Debug2Format(&buf));
        return Err(());
    }

    let raw = (((buf[0] as u16) << 8) | (buf[1] as u16)) & !0x0003;
    let mut rh = -6.0 + 125.0 * (raw as f32) / 65536.0;
    if rh < 0.0 { rh = 0.0; }
    if rh > 100.0 { rh = 100.0; }
    Ok(rh)
}

// Sensirion CRC-8, poly 0x31, init 0x00
fn sensirion_crc_ok(two_bytes: &[u8], crc_in: u8) -> bool {
    let mut crc: u8 = 0x00;
    for &b in two_bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 { (crc << 1) ^ 0x31 } else { crc << 1 };
        }
    }
    crc == crc_in
}

// ================== 極簡 MQTT ==================
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
