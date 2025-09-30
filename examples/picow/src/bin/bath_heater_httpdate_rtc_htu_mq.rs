//! bath_heater_httpdate_rtc_htu_mq.rs
//! 功能：
//! - Wi-Fi + HTTP Date 對時（每 15 分鐘執行一次；失敗閃 GP22 三下）
//! - 內建 RTC：每 5 秒讀出列印目前時間
//! - HTU21D（I2C0：SCL=GP5, SDA=GP4）：每 15 秒做 3 次取平均後列印（溫度°C與相對濕度%）
//! - MQTT：每 3 分鐘上報一次 {fw_ver, light, heater_coil_mA, temp_c, rh}；發佈前後印訊息與結果
//!
//! 依你的環境對齊 embassy examples：CYW43 via PIO-SPI、embassy-net DHCPv4、I2c::new_blocking。

#![no_std]
#![no_main]

use core::str::from_utf8;

use defmt::*;
use defmt::Debug2Format;
use defmt_rtt as _;       // defmt 後端
use panic_probe as _;     // 提供 #[panic_handler]

use embassy_executor::Spawner;
use embassy_time::{Timer, Duration, with_timeout, Ticker};

use embassy_rp::bind_interrupts;          // bind_interrupts! 巨集
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, DMA_CH0, I2C0};
use embassy_rp::pio::{Pio, InterruptHandler};
use embassy_rp::clocks::RoscRng;
use embassy_rp::i2c::{self, I2c};
use embassy_rp::rtc::{Rtc, DateTime, DayOfWeek};

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};

use static_cell::StaticCell;

use embassy_net as net;
use embassy_net::{Config, StackResources, IpAddress, Ipv4Address, IpEndpoint};
use embassy_net::tcp::TcpSocket;
use embedded_io_async::Write;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// === 全域通道：送 unix epoch 秒給 RTC 服務 ===
static RTC_EPOCH_CH: Channel<CriticalSectionRawMutex, u64, 1> = Channel::new();

// 送 HTU21D 平均值（centi）給 MQTT 任務
static HTU_AVG_CH: Channel<CriticalSectionRawMutex, (i32, i32), 4> = Channel::new();

// === 設定 ===
const USER_LED_ACTIVE_HIGH: bool = true;  // GP22 為使用者 LED，High 亮

// HTTP Date 來源（Cloudflare 1.1.1.1）
const HTTP_DATE_IP: Ipv4Address = Ipv4Address::new(1, 1, 1, 1);
const HTTP_DATE_PORT: u16 = 80;

// MQTT 設定（參照 wifi_mqtt.rs）
const MQTT_BROKER_IP: (u8,u8,u8,u8) = (192, 168, 188, 182);
const MQTT_PORT: u16 = 2883;
const MQTT_TOPIC: &str = "lab/picoW/telemetry";
const MQTT_CLIENT_PREFIX: &str = "picoW";
const FW_VER: &str = "htu21d-0.1.0";      // JSON 與 client_id 都會附帶

// HTU21D
const HTU21D_ADDR: u8 = 0x40;
const CMD_TRIG_TEMP_NOHOLD: u8 = 0xF3;
const CMD_TRIG_RH_NOHOLD: u8   = 0xF5;

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

// ——— RTC 服務：
// 擁有 Rtc 本體；每 5 秒列印一次 now()；有新 epoch 從通道送進來就 set_datetime。
#[embassy_executor::task]
async fn rtc_task(mut rtc: Rtc<'static, embassy_rp::peripherals::RTC>) -> ! {
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        // 非阻塞嘗試收一筆更新（用 0 超時）
        if let Ok(epoch) = with_timeout(Duration::from_millis(0), RTC_EPOCH_CH.receive()).await {
            if let Some(dt) = unix_to_datetime(epoch) {
                let _ = rtc.set_datetime(dt);
                info!("RTC set by HTTP Date: unix={}.", epoch);
            }
        }

        // 每 5 秒列印 RTC 時間
        ticker.next().await;
        match rtc.now() {
            Ok(dt) => info!(
                "RTC now: {}-{:02}-{:02} {:02}:{:02}:{:02}",
                dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second
            ),
            Err(_) => warn!("RTC not set yet"),
        }
    }
}

// ——— HTTP Date 更新任務：每 15 分鐘跑一次，成功送 epoch 到 RTC_EPOCH_CH；失敗閃 GP22 三下
#[embassy_executor::task]
async fn httpdate_task(stack: net::Stack<'static>, mut led: Output<'static>) -> ! {
    loop {
        match http_date_once(stack).await {
            Some(ts) => {
                RTC_EPOCH_CH.send(ts).await;
            }
            None => {
                blink_err(&mut led, 3, 80, 80).await; // 每次失敗閃 3 下
            }
        }
        Timer::after_secs(15 * 60).await;
    }
}

// ——— HTU21D 任務：每 15 秒，溫度/濕度各讀 3 次取平均
#[embassy_executor::task]
async fn htu21d_task(mut i2c: I2c<'static, I2C0, i2c::Blocking>) -> ! {
    info!("HTU21D: I2C0 SCL=GP5, SDA=GP4, 400kHz; 每 15 秒做 3 次平均");
    loop {
        let mut t_sum: i32 = 0;
        let mut rh_sum: i32 = 0;
        let mut n: i32 = 0;
        for _ in 0..3 {
            match read_measurement_blocking(&mut i2c, CMD_TRIG_TEMP_NOHOLD, 60).await {
                Ok(raw_t) => { t_sum += temp_centi(raw_t); }
                Err(e) => { warn!("HTU21D temp read err: {}", e); }
            }
            match read_measurement_blocking(&mut i2c, CMD_TRIG_RH_NOHOLD, 30).await {
                Ok(raw_rh) => { rh_sum += rh_centi(raw_rh); }
                Err(e) => { warn!("HTU21D RH read err: {}", e); }
            }
            n += 1;
        }
        if n > 0 {
            let t_avg = t_sum / n;
            let rh_avg = (rh_sum / n).clamp(0, 10000);
            log_centi("T(°C)", t_avg);
            log_centi("RH(%)", rh_avg);
            // 將最新平均值送給 MQTT 任務
            HTU_AVG_CH.send((t_avg, rh_avg)).await;
        }
        Timer::after_secs(15).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Pico W + HTU21D (fw={})  BTN=GP15  LED=GP22  I2C0(SCL=GP5,SDA=GP4)", FW_VER);
    info!("booting (wifi + http-date + rtc + htu21d)…");

    // --- RP2040 基本初始化
    let p = embassy_rp::init(Default::default());

    // 使用者 LED：GP22（僅用於 HTTP Date 失敗時閃燈）
    let led = Output::new(
        p.PIN_22,
        if USER_LED_ACTIVE_HIGH { Level::Low } else { Level::High },
    );

    // --- CYW43（Pico W）初始化：PWR 腳 + PIO SPI + 固件/CLM
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs  = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,  // CMD
        p.PIN_29,  // D0/CLK（依 embassy 範例腳位）
        p.DMA_CH0,
    );

    // 固件/CLM 路徑沿用 embassy 範例目錄結構
    let fw  = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // --- 網路堆疊（DHCPv4）
    let mut rng = RoscRng;
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

    // --- Wi‑Fi 連線（從環境變數讀，沒有就用預設）
    let wifi_ssid: &str = option_env!("WIFI_NETWORK").unwrap_or("WAX2617");
    let wifi_pass: &str = option_env!("WIFI_PASSWORD").unwrap_or("7499363II5495264");
    info!("Wi-Fi join: {}", wifi_ssid);
    control.join(wifi_ssid, JoinOptions::new(wifi_pass.as_bytes())).await.unwrap();

    stack.wait_link_up().await;
    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() { info!("net up: {:?}", Debug2Format(&cfg)); }

    // --- 啟動 RTC 服務（持有 p.RTC）
    let rtc = Rtc::new(p.RTC);
    unwrap!(spawner.spawn(rtc_task(rtc)));

    // --- 啟動 HTTP Date 更新（持有 LED 與 stack）
    unwrap!(spawner.spawn(httpdate_task(stack, led)));

    // --- 啟動 HTU21D 任務（I2C0: SCL=GP5, SDA=GP4）
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 400_000; // 400kHz
    let i2c = I2c::new_blocking(p.I2C0, p.PIN_5, p.PIN_4, i2c_cfg);
    unwrap!(spawner.spawn(htu21d_task(i2c)));

    // --- 啟動 MQTT 任務（每 3 分鐘上報最新 HTU 平均值）
    unwrap!(spawner.spawn(mqtt_task(stack)));

    // main 不再持有任何資源；任務們各司其職。
    loop { Timer::after_secs(3600).await; }
}

// === HTTP Date（一次）：連 1.1.1.1:80，送 HEAD 取 Date: 行 → 轉 Unix 秒 ===
async fn http_date_once(stack: net::Stack<'_>) -> Option<u64> {
    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 1024];
    let mut sock = TcpSocket::new(stack, &mut rx, &mut tx);

    sock.set_timeout(Some(Duration::from_secs(4)));

    let ep = IpEndpoint::new(IpAddress::from(HTTP_DATE_IP), HTTP_DATE_PORT);
    if let Err(e) = sock.connect(ep).await {
        warn!("http_date: connect fail: {:?}", Debug2Format(&e));
        return None;
    }

    let req = b"HEAD / HTTP/1.1\r\nHost: 1.1.1.1\r\nConnection: close\r\nUser-Agent: pico-w\r\n\r\n";
    if let Err(e) = sock.write_all(req).await {
        warn!("http_date: write fail: {:?}", Debug2Format(&e));
        return None;
    }

    let mut buf = [0u8; 1024];
    let n = match sock.read(&mut buf).await {
        Ok(n) => n,
        Err(e) => { warn!("http_date: read fail: {:?}", Debug2Format(&e)); return None; }
    };

    let s = from_utf8(&buf[..n]).ok()?;
    for line in s.lines() {
        if let Some(rest) = line.strip_prefix("Date: ") {
            if let Some(ts) = parse_rfc7231_gmt_to_unix(rest.trim()) { return Some(ts); }
            warn!("http_date: Date header parse fail: {}", rest.trim());
            return None;
        }
    }
    warn!("http_date: no Date header");
    None
}

// === LED ===
#[inline]
async fn blink_err(led: &mut Output<'_>, times: u8, on_ms: u64, off_ms: u64) {
    for _ in 0..times {
        set_led(led, true);
        Timer::after_millis(on_ms).await;
        set_led(led, false);
        Timer::after_millis(off_ms).await;
    }
}
#[inline]
fn set_led(led: &mut Output<'_>, on: bool) {
    let level = if USER_LED_ACTIVE_HIGH {
        if on { Level::High } else { Level::Low }
    } else {
        if on { Level::Low } else { Level::High }
    };
    led.set_level(level);
}

// === RFC7231 "Date: Tue, 15 Nov 1994 08:12:31 GMT" → Unix 秒 ===
pub fn parse_rfc7231_gmt_to_unix(s: &str) -> Option<u64> {
    let mut toks: [&str; 6] = ["", "", "", "", "", ""];
    let mut i = 0usize;
    for part in s.split(|c| c == ' ' || c == ',') {
        if part.is_empty() { continue; }
        if i < 6 { toks[i] = part; i += 1; } else { break; }
    }
    if i < 6 || toks[5] != "GMT" { return None; }

    let day: u32 = toks[1].parse().ok()?;
    let month: u32 = match toks[2] {
        "Jan" => 1, "Feb" => 2, "Mar" => 3, "Apr" => 4, "May" => 5, "Jun" => 6,
        "Jul" => 7, "Aug" => 8, "Sep" => 9, "Oct" => 10, "Nov" => 11, "Dec" => 12,
        _ => return None,
    };
    let year: i32 = toks[3].parse().ok()?;
    let (hh, mm, ss) = {
        let mut it = toks[4].split(':');
        let h: u32 = it.next()?.parse().ok()?;
        let m: u32 = it.next()?.parse().ok()?;
        let s: u32 = it.next()?.parse().ok()?;
        (h, m, s)
    };

    let days = days_since_unix_epoch(year, month, day)?;
    Some(days as u64 * 86400 + (hh as u64) * 3600 + (mm as u64) * 60 + ss as u64)
}

#[inline]
fn is_leap(y: i32) -> bool { (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0) }
fn days_before_year(y: i32) -> i64 {
    let mut days: i64 = 0; let mut yr = 1970;
    if y >= 1970 { while yr < y { days += if is_leap(yr) { 366 } else { 365 }; yr += 1; } }
    else { while yr > y { yr -= 1; days -= if is_leap(yr) { 366 } else { 365 }; } }
    days
}
fn days_since_unix_epoch(year: i32, month: u32, day: u32) -> Option<i64> {
    if !(1..=12).contains(&month) || !(1..=31).contains(&day) { return None; }
    const MDAYS: [u32; 12] = [31,28,31,30,31,30,31,31,30,31,30,31];
    let md = if month == 2 && is_leap(year) { 29 } else { MDAYS[(month-1) as usize] };
    if day > md { return None; }
    let mut days = days_before_year(year);
    for m in 1..month { days += if m == 2 && is_leap(year) { 29 } else { MDAYS[(m-1) as usize] } as i64; }
    days += (day - 1) as i64; Some(days)
}

// ——— Unix 秒 → DateTime（UTC）
fn unix_to_datetime(ts: u64) -> Option<DateTime> {
    // 1970-01-01 是 Thursday
    let sec = ts as i64;
    let days = sec / 86400; let mut rem = (sec % 86400) as i64;
    if rem < 0 { rem += 86400; }
    let hour = (rem / 3600) as u8; rem %= 3600;
    let minute = (rem / 60) as u8; let second = (rem % 60) as u8;

    // 計算年月日
    let mut y: i32 = 1970;
    let mut day_count = days as i64;
    loop {
        let ydays = if is_leap(y) { 366 } else { 365 } as i64;
        if day_count >= ydays { day_count -= ydays; y += 1; } else { break; }
    }
    let mut month: u32 = 1;
    loop {
        let mdays = if month == 2 && is_leap(y) { 29 } else { [31,28,31,30,31,30,31,31,30,31,30,31][(month-1) as usize] } as i64;
        if day_count >= mdays { day_count -= mdays; month += 1; } else { break; }
    }
    let day = (day_count + 1) as u8;

    // 1970-01-01 Thu → DayOfWeek 對應
    let dow_index = ((days + 4) % 7) as u8; // 0..6
    let dow = match dow_index { 0 => DayOfWeek::Sunday, 1 => DayOfWeek::Monday, 2 => DayOfWeek::Tuesday, 3 => DayOfWeek::Wednesday, 4 => DayOfWeek::Thursday, 5 => DayOfWeek::Friday, _ => DayOfWeek::Saturday };

    Some(DateTime{ year: y as u16, month: month as u8, day, day_of_week: dow, hour, minute, second })
}

// === HTU21D 工具（CRC、轉換、讀取） ===
fn htu21d_crc(data: [u8; 2]) -> u8 {
    let mut crc: u8 = 0x00;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 { crc = if (crc & 0x80) != 0 { (crc << 1) ^ 0x31 } else { crc << 1 }; }
    }
    crc
}
async fn read_measurement_blocking(
    i2c: &mut I2c<'_, I2C0, i2c::Blocking>, cmd: u8, wait_ms: u64,
) -> Result<u16, &'static str> {
    i2c.blocking_write(HTU21D_ADDR, &[cmd]).map_err(|_| "i2c write fail")?;
    Timer::after_millis(wait_ms).await;
    let mut buf = [0u8; 3];
    i2c.blocking_read(HTU21D_ADDR, &mut buf).map_err(|_| "i2c read fail")?;
    let crc = htu21d_crc([buf[0], buf[1]]);
    if crc != buf[2] { return Err("crc mismatch"); }
    let raw = ((((buf[0] as u16) << 8) | buf[1] as u16) & 0xFFFC) as u16;
    Ok(raw)
}
fn temp_centi(raw: u16) -> i32 { -4685 + ((17572i32 * raw as i32) >> 16) }
fn rh_centi(raw: u16) -> i32 { (-600 + ((12500i32 * raw as i32) >> 16)).clamp(0, 10000) }
fn log_centi(label: &str, centi: i32) {
    let sign = if centi < 0 { "-" } else { "" };
    let a = centi.abs() as u32; let intp = a / 100; let frac = a % 100; let d1 = frac / 10; let d0 = frac % 10;
    info!("{}={}{}.{}{}", label, sign, intp, d1, d0);
}

// === MQTT 任務：每 3 分鐘上報一次，採用 HTU_AVG_CH 最新值 ===
#[embassy_executor::task]
async fn mqtt_task(stack: net::Stack<'static>) -> ! {
    let client_id: &str = MQTT_CLIENT_PREFIX;
    let (a,b,c,d) = MQTT_BROKER_IP;
    let broker_ip: IpAddress = IpAddress::v4(a,b,c,d);

    let mut rx_buf = [0u8; 1536];
    let mut tx_buf = [0u8; 1536];

    'reconnect: loop {
        let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        sock.set_timeout(Some(Duration::from_secs(10)));

        info!("MQTT: TCP connecting to {}.{}.{}.{}:{} ...", a,b,c,d, MQTT_PORT);
        if let Err(e) = sock.connect((broker_ip, MQTT_PORT)).await {
            warn!("MQTT: TCP connect failed: {:?}", Debug2Format(&e));
            Timer::after(Duration::from_secs(3)).await;
            continue 'reconnect;
        }
        info!("MQTT: TCP connected.");

        if let Err(e) = mqtt_send_connect(&mut sock, client_id, 75).await {
            warn!("MQTT: CONNECT failed: {:?}", Debug2Format(&e));
            let _ = sock.close();
            Timer::after(Duration::from_secs(3)).await;
            continue 'reconnect;
        }
        info!("MQTT: CONNECT sent.");

        let mut last_t: i32 = 0;
        let mut last_rh: i32 = 0;

        loop {
            if let Ok((t, rh)) = with_timeout(Duration::from_millis(1), HTU_AVG_CH.receive()).await {
                last_t = t;
                last_rh = rh;
            }

            let temp_c = (last_t as f32) / 100.0;
            let rh = (last_rh as f32) / 100.0;
            let light_on = false;
            let heater_coil_ma = 0.0_f32;

            let mut payload = heapless::String::<256>::new();
            let _ = core::fmt::Write::write_fmt(
                &mut payload,
                core::format_args!(
                    "{{\"fw_ver\":\"{}\",\"light\":{},\"heater_coil_mA\":{:.1},\"temp_c\":{:.2},\"rh\":{:.1}}}",
                    FW_VER,
                    if light_on { "true" } else { "false" },
                    heater_coil_ma, temp_c, rh
                )
            );

            info!("MQTT -> topic={} payload={}", MQTT_TOPIC, payload.as_str());
            match mqtt_publish_qos0(&mut sock, MQTT_TOPIC, payload.as_bytes()).await {
                Ok(_) => info!("MQTT publish ok"),
                Err(e) => { warn!("MQTT publish failed: {:?}", Debug2Format(&e)); let _ = sock.close(); Timer::after(Duration::from_secs(3)).await; continue 'reconnect; }
            }

            // 保活 3 分鐘期間每 60 秒 PING 一次
            for _ in 0..2 {
                Timer::after(Duration::from_secs(60)).await;
                if let Err(e) = mqtt_send_pingreq(&mut sock).await {
                    warn!("MQTT ping failed: {:?}", Debug2Format(&e));
                    let _ = sock.close();
                    continue 'reconnect;
                }
                if let Ok((t, rh)) = with_timeout(Duration::from_millis(1), HTU_AVG_CH.receive()).await {
                    last_t = t; last_rh = rh;
                }
            }
        }
    }
}

// ===== 極簡 MQTT 3.1.1 =====
async fn mqtt_send_connect<S: embedded_io_async::Write + embedded_io_async::Read + embedded_io_async::ErrorType>(
    sock: &mut S, client_id: &str, keep_alive_s: u16
) -> Result<(), S::Error> {
    let protocol_name = "MQTT";
    let protocol_level = 0x04u8;
    let connect_flags = 0b0000_0010u8; // Clean Session
    let keep_alive = keep_alive_s.to_be_bytes();

    let mut hdr = heapless::Vec::<u8, 128>::new();
    hdr.push(0x10).ok();
    encode_rem_len((2 + protocol_name.len() + 1 + 1 + 2 + 2 + client_id.len()) as u32, &mut hdr);
    push_str(&mut hdr, protocol_name);
    hdr.push(protocol_level).ok();
    hdr.push(connect_flags).ok();
    hdr.extend_from_slice(&keep_alive).ok();
    push_str(&mut hdr, client_id);
    sock.write_all(&hdr).await
}

async fn mqtt_publish_qos0<S: embedded_io_async::Write + embedded_io_async::ErrorType>(
    sock: &mut S, topic: &str, payload: &[u8]
) -> Result<(), S::Error> {
    let mut hdr = heapless::Vec::<u8, 128>::new();
    hdr.push(0x30).ok(); // PUBLISH QoS0
    encode_rem_len((2 + topic.len() + payload.len()) as u32, &mut hdr);
    push_str(&mut hdr, topic);
    sock.write_all(&hdr).await?;
    sock.write_all(payload).await
}

async fn mqtt_send_pingreq<S: embedded_io_async::Write + embedded_io_async::ErrorType>(sock: &mut S) -> Result<(), S::Error> {
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