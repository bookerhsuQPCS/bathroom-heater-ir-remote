// src/bin/wifi_ntp_rtc_full_v2.rs
#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net as net;
use embassy_net::{
    tcp::TcpSocket,
    udp::{PacketMetadata, UdpSocket},
    Config as NetConfig, IpAddress, IpEndpoint, Ipv4Address,
};
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{PIO0, USB},
    pio::Pio,
    rtc::{DateTime, DayOfWeek, Rtc},
    usb,
};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use cortex_m::peripheral::SCB;

// ========================== 可調參數 ==========================
const HTTP_CONNECT_TIMEOUT_S: u64 = 3;
const HTTP_READ_TIMEOUT_S: u64 = 3;
const NTP_TIMEOUT_S: u64 = 3;           // 每個 NTP 嘗試的接收逾時
const NTP_TRIES_PER_SERVER: usize = 1;  // 每台伺服器只試一次
const RESYNC_INTERVAL_S: u32 = 30 * 60; // 30 分鐘自動重對時
const RTC_PRINT_INTERVAL_S: u32 = 15;   // 每 15 秒印一次 RTC
const ARP_WARMUP_S: u64 = 1;            // DHCP 後暖機，避免第一輪 ARP Pending
// ============================================================

// --- CYW43 韌體 ---
const FW: &[u8] = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
const CLM: &[u8] = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

// --- Wi-Fi 參數 ---
fn wifi_ssid() -> &'static str {
    option_env!("WIFI_SSID").unwrap_or("WAX2617")
}
fn wifi_pass() -> &'static str {
    option_env!("WIFI_PASS").unwrap_or("7499363II5495264")
}

// --- IRQ 綁定 ---
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0  => embassy_rp::pio::InterruptHandler<PIO0>;
});

// ===== NTP 伺服器清單（TW → JP）=====
fn ntp_servers() -> [Ipv4Address; 4] {
    [
        Ipv4Address::new(192, 168, 188, 1),   // routerOS
        Ipv4Address::new(118, 163, 81, 62),   // TANet
        Ipv4Address::new(211, 22, 103, 158),  // 中華
        Ipv4Address::new(133, 243, 238, 163), // NICT
    ]
}

// ======== 工具：UNIX 秒 ↔ RTC DateTime（UTC）========
fn unix_to_datetime(mut t: u64) -> DateTime {
    let sec = (t % 60) as u8;
    t /= 60;
    let min = (t % 60) as u8;
    t /= 60;
    let hour = (t % 24) as u8;
    let days = (t / 24) as i32;

    let z = days + 719_163; // 1970-01-01 的 Rata Die
    let era = (z - 1) / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let mon = if mp < 10 { mp + 3 } else { mp - 9 } as u8;
    let year = (y + if mon <= 2 { 1 } else { 0 }) as u16;
    let weekday = ((days + 4).rem_euclid(7)) as u8; // 1970-01-01 是星期四=4

    DateTime {
        year,
        month: mon,
        day: d as u8,
        day_of_week: match weekday {
            0 => DayOfWeek::Sunday,
            1 => DayOfWeek::Monday,
            2 => DayOfWeek::Tuesday,
            3 => DayOfWeek::Wednesday,
            4 => DayOfWeek::Thursday,
            5 => DayOfWeek::Friday,
            _ => DayOfWeek::Saturday,
        },
        hour,
        minute: min,
        second: sec,
    }
}

// ======== LED 工具 ========
async fn blink(pin: &mut Output<'_>, times: u8, on_ms: u64, off_ms: u64) {
    for _ in 0..times {
        pin.set_high();
        Timer::after_millis(on_ms).await;
        pin.set_low();
        Timer::after_millis(off_ms).await;
    }
}
fn all_off(g: &mut Output<'_>, b: &mut Output<'_>, r: &mut Output<'_>) {
    g.set_low();
    b.set_low();
    r.set_low();
}

// ======== NTP：單次 ========
#[allow(static_mut_refs)]
async fn ntp_once(stack: net::Stack<'static>, ip: Ipv4Address) -> Option<u64> {
    static mut RX_META: [PacketMetadata; 4] = [PacketMetadata::EMPTY; 4];
    static mut TX_META: [PacketMetadata; 4] = [PacketMetadata::EMPTY; 4];
    static mut RX_BUF:  [u8; 576] = [0; 576];
    static mut TX_BUF:  [u8; 576] = [0; 576];

    let rx_meta = unsafe { &mut RX_META };
    let tx_meta = unsafe { &mut TX_META };
    let rx_buf  = unsafe { &mut RX_BUF  };
    let tx_buf  = unsafe { &mut TX_BUF  };

    let mut sock = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    unwrap!(sock.bind(IpEndpoint::new(IpAddress::v4(0, 0, 0, 0), 0)));

    let mut pkt = [0u8; 48];
    pkt[0] = 0x1B;

    let server = IpEndpoint::new(IpAddress::Ipv4(ip), 123);
    if let Err(e) = sock.send_to(&pkt, server).await {
        warn!("NTP send fail: {:?}", e);
        return None;
    }

    let mut buf = [0u8; 96];
    let deadline = Instant::now() + Duration::from_secs(NTP_TIMEOUT_S);
    loop {
        let now = Instant::now();
        if now >= deadline {
            warn!("NTP recv timeout");
            return None;
        }
        match embassy_time::with_timeout(deadline - now, sock.recv_from(&mut buf)).await {
            Ok(Ok((len, _from))) if len >= 48 => {
                let secs_1900 = u32::from_be_bytes([buf[40], buf[41], buf[42], buf[43]]) as u64;
                const NTP_UNIX_DELTA: u64 = 2_208_988_800;
                let unix = secs_1900.saturating_sub(NTP_UNIX_DELTA);
                return Some(unix);
            }
            Ok(Ok(_)) => { /* 太短，繼續 */ }
            _ => {
                warn!("NTP recv error");
                return None;
            }
        }
    }
}

// ======== NTP：輪詢 ========
async fn ntp_poll(stack: net::Stack<'static>) -> Option<(u64, Ipv4Address)> {
    for ip in ntp_servers() {
        info!("NTP IPv4 = {}", defmt::Display2Format(&ip));
        for _ in 0..NTP_TRIES_PER_SERVER {
            if let Some(ts) = ntp_once(stack, ip).await {
                return Some((ts, ip));
            }
            Timer::after_millis(300).await;
        }
    }
    None
}

// ======== HTTP Date 取時（1.1.1.1:80）========
#[allow(static_mut_refs)]
async fn http_date_poll(stack: net::Stack<'static>) -> Option<u64> {
    static mut RX: [u8; 1024] = [0; 1024];
    static mut TX: [u8; 512]  = [0; 512];

    let rx = unsafe { &mut RX };
    let tx = unsafe { &mut TX };
    let mut sock = TcpSocket::new(stack, rx, tx);

    let ep = IpEndpoint::new(IpAddress::v4(1, 1, 1, 1), 80);

    if embassy_time::with_timeout(Duration::from_secs(HTTP_CONNECT_TIMEOUT_S), sock.connect(ep))
        .await
        .is_err()
    {
        warn!("HTTP connect timeout");
        return None;
    }

    let req = b"GET / HTTP/1.1\r\nHost: 1.1.1.1\r\nConnection: close\r\nUser-Agent: pico-w\r\n\r\n";
    if sock.write(req).await.is_err() {
        warn!("HTTP write error");
        let _ = sock.close();
        return None;
    }
    let _ = sock.flush().await;

    let deadline = Instant::now() + Duration::from_secs(HTTP_READ_TIMEOUT_S);
    let mut total = 0usize;
    let mut buf = [0u8; 1024];

    loop {
        let now = Instant::now();
        if now >= deadline {
            warn!("HTTP recv timeout");
            let _ = sock.close();
            return None;
        }
        match embassy_time::with_timeout(deadline - now, sock.read(&mut buf[total..])).await {
            Ok(Ok(0)) => break,
            Ok(Ok(n)) => {
                total += n;
                if total >= buf.len() { break; }
                if total >= 4 && buf[..total].windows(4).any(|w| w == b"\r\n\r\n") {
                    break; // 有 header 了
                }
            }
            _ => {
                warn!("HTTP recv error");
                let _ = sock.close();
                return None;
            }
        }
    }
    let _ = sock.close();

    parse_http_date(&buf[..total])
}

// ======== 解析 RFC7231 Date 標頭 → UNIX 秒 ========
fn parse_http_date(payload: &[u8]) -> Option<u64> {
    let s = core::str::from_utf8(payload).ok()?;
    let mut date_line: Option<&str> = None;
    for line in s.lines() {
        let b = line.as_bytes();
        if b.len() >= 5 && (b[0] == b'D' || b[0] == b'd') && (&b[1..5]).eq_ignore_ascii_case(b"ate:")
        {
            let mut i = 5;
            while i < b.len() && (b[i] == b' ' || b[i] == b'\t') { i += 1; }
            date_line = Some(&line[i..]);
            break;
        }
    }
    let dl = date_line?;
    // 可能是 "Sun, 06 Nov 1994 08:49:37 GMT"
    let dl = dl.trim();
    let dl = if let Some(idx) = dl.as_bytes().iter().position(|&c| c == b',') {
        dl.get(idx + 1..)?.trim()
    } else { dl };

    let (day_str, rest1) = next_token(dl)?;
    let (mon_str, rest2) = next_token(rest1)?;
    let (year_str, rest3) = next_token(rest2)?;
    let (hms_str, _rest4) = next_token(rest3)?;

    let day: u32 = day_str.parse().ok()?;
    let mon: u32 = match_mon(mon_str)?;
    let year: u32 = year_str.parse().ok()?;

    let mut it = hms_str.split(':');
    let hh: u32 = it.next()?.parse().ok()?;
    let mm: u32 = it.next()?.parse().ok()?;
    let ss: u32 = it.next()?.parse().ok()?;

    let days = ymd_to_days(year as i32, mon as i32, day as i32)? - 719_163;
    if days < 0 { return None; }
    let secs = (days as u64) * 86_400 + (hh as u64) * 3600 + (mm as u64) * 60 + (ss as u64);
    Some(secs)
}

fn next_token(s: &str) -> Option<(&str, &str)> {
    let s = s.trim_start();
    if s.is_empty() { return None; }
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() && !bytes[i].is_ascii_whitespace() { i += 1; }
    let token = &s[..i];
    let mut j = i;
    while j < bytes.len() && bytes[j].is_ascii_whitespace() { j += 1; }
    Some((token, &s[j..]))
}

fn match_mon(s: &str) -> Option<u32> {
    let b = s.as_bytes();
    let m = match &b[..] {
        b"Jan" | b"JAN" | b"jan" => 1,
        b"Feb" | b"FEB" | b"feb" => 2,
        b"Mar" | b"MAR" | b"mar" => 3,
        b"Apr" | b"APR" | b"apr" => 4,
        b"May" | b"MAY" | b"may" => 5,
        b"Jun" | b"JUN" | b"jun" => 6,
        b"Jul" | b"JUL" | b"jul" => 7,
        b"Aug" | b"AUG" | b"aug" => 8,
        b"Sep" | b"SEP" | b"sep" => 9,
        b"Oct" | b"OCT" | b"oct" => 10,
        b"Nov" | b"NOV" | b"nov" => 11,
        b"Dec" | b"DEC" | b"dec" => 12,
        _ => return None,
    };
    Some(m)
}

fn ymd_to_days(y: i32, m: i32, d: i32) -> Option<i32> {
    if !(1..=12).contains(&m) || !(1..=31).contains(&d) { return None; }
    let y_adj = y - if m <= 2 { 1 } else { 0 };
    let era = y_adj / 400;
    let yoe = y_adj - era * 400;
    let mp = m + if m > 2 { -3 } else { 9 };
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era * 146_097 + doe)
}

// ===== 背景任務 =====
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, embassy_rp::peripherals::DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

// ===== 成功方法記錄 =====
#[derive(Copy, Clone)]
enum SyncMethod {
    Http,
    Ntp(Ipv4Address),
}

fn print_rtc_now<T: embassy_rp::rtc::Instance>(rtc: &mut Rtc<'_, T>) {
    if let Ok(now) = rtc.now() {
        info!(
            "RTC: {:04}/{:02}/{:02} {:02}:{:02}:{:02}",
            now.year, now.month, now.day, now.hour, now.minute, now.second
        );
    } else {
        warn!("RTC read failed");
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Pico W V2: HTTP-first → NTP refine; 30m resync; 15s print; reset on failure + RGB LEDs");

    // RP2040 init
    let p = embassy_rp::init(Default::default());

    // GPIO 20(GREEN) 21(BLUE) 22(RED)
    let mut led_g = Output::new(p.PIN_20, Level::Low);
    let mut led_b = Output::new(p.PIN_21, Level::Low);
    let mut led_r = Output::new(p.PIN_22, Level::Low);
    all_off(&mut led_g, &mut led_b, &mut led_r);

    // CYW43 power / CS
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);

    // PIO SPI（PIO0.SM0, IRQ0, PIN_24, PIN_29, DMA_CH0）
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

    // CYW43 start
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, cyw_runner) = cyw43::new(state, pwr, spi, FW).await;
    unwrap!(spawner.spawn(cyw43_task(cyw_runner)));

    // 閃 BLUE 表示初始化無線模組中
    let _ = blink(&mut led_b, 2, 80, 120).await;

    // 關鍵：初始化 CLM
    control.init(CLM).await;

    // 網路 Stack（DHCPv4）
    static RESOURCES: StaticCell<net::StackResources<4>> = StaticCell::new();
    let resources = RESOURCES.init(net::StackResources::<4>::new());
    let seed: u64 = Instant::now().as_ticks() as u64 ^ 0x5EED_1234_5678_ABCDu64;

    let config = NetConfig::dhcpv4(Default::default());
    let (stack, _net_runner) = net::new(net_device, config, resources, seed);
    unwrap!(spawner.spawn(net_task(_net_runner)));

    // 連網
    loop {
        match control.join(wifi_ssid(), JoinOptions::new(wifi_pass().as_bytes())).await {
            Ok(()) => break,
            Err(e) => {
                warn!("join failed: status={}", e.status);
                let _ = blink(&mut led_r, 1, 60, 140).await; // 紅燈小閃表示 join 失敗
                Timer::after_millis(600).await;
            }
        }
    }
    info!("Wi-Fi joined: {}", wifi_ssid());
    Timer::after_secs(ARP_WARMUP_S).await;

    let mut rtc = Rtc::new(p.RTC);

    // ===== 開機：先 HTTP → 再短 NTP =====
    let mut last_method: Option<SyncMethod> = None;

    // 1) HTTP 優先（BLUE）
    led_b.set_high();
    if let Some(ts) = http_date_poll(stack).await {
        let dt = unix_to_datetime(ts);
        let (y, mo, d, hh, mi, ss) = (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
        let _ = rtc.set_datetime(dt);
        info!(
            "Time sync SUCCESS via HTTP (1.1.1.1). RTC set to {:04}/{:02}/{:02} {:02}:{:02}:{:02}",
            y, mo, d, hh, mi, ss
        );
        last_method = Some(SyncMethod::Http);
        let _ = blink(&mut led_b, 3, 80, 120).await; // BLUE×3 表 HTTP 成功
    } else {
        warn!("HTTP Date failed; will try NTP.");
        let _ = blink(&mut led_r, 1, 80, 120).await; // RED 小提醒
    }
    led_b.set_low();

    // 2) 短 NTP 微調（GREEN）
    led_g.set_high();
    if let Some((ts, ip)) = ntp_poll(stack).await {
        let dt = unix_to_datetime(ts);
        let (y, mo, d, hh, mi, ss) = (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
        let _ = rtc.set_datetime(dt);
        info!(
            "Time sync SUCCESS via NTP ({}). RTC set to {:04}/{:02}/{:02} {:02}:{:02}:{:02}",
            defmt::Display2Format(&ip), y, mo, d, hh, mi, ss
        );
        last_method = Some(SyncMethod::Ntp(ip));
        let _ = blink(&mut led_g, 3, 80, 120).await; // GREEN×3 表 NTP 成功
    } else if last_method.is_none() {
        // HTTP 也失敗、NTP 也失敗 → 紅燈連閃，重開機
        error!("Time sync FAILED (HTTP & NTP). RESETTING...");
        let _ = blink(&mut led_r, 5, 80, 100).await;
        reboot();
    }
    led_g.set_low();

    // ===== 進入週期性任務：每 15 秒印 RTC、每 30 分鐘重對時 =====
    let mut sec_counter: u32 = 0;
    let mut since_resync: u32 = 0;

    loop {
        Timer::after_secs(1).await;
        sec_counter = sec_counter.wrapping_add(1);
        since_resync = since_resync.wrapping_add(1);

        // 每 15 秒印一次 RTC（GREEN 心跳一下）
        if sec_counter % RTC_PRINT_INTERVAL_S == 0 {
            print_rtc_now(&mut rtc);
            let _ = blink(&mut led_g, 1, 30, 60).await;
        }

        // 每 30 分鐘重對時：優先用上回成功的方法
        if since_resync >= RESYNC_INTERVAL_S {
            since_resync = 0;

            match last_method {
                Some(SyncMethod::Ntp(ip)) => {
                    info!(
                        "Periodic resync using last successful method: NTP ({})",
                        defmt::Display2Format(&ip)
                    );
                    led_g.set_high();
                    if let Some(ts) = ntp_once(stack, ip).await {
                        let dt = unix_to_datetime(ts);
                        let (y, mo, d, hh, mi, ss) =
                            (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
                        let _ = rtc.set_datetime(dt);
                        info!(
                            "Resync SUCCESS via NTP ({}). RTC={:04}/{:02}/{:02} {:02}:{:02}:{:02}",
                            defmt::Display2Format(&ip),
                            y, mo, d, hh, mi, ss
                        );
                        let _ = blink(&mut led_g, 2, 60, 120).await;
                        last_method = Some(SyncMethod::Ntp(ip));
                    } else {
                        led_g.set_low();
                        warn!("Resync NTP failed; trying HTTP fallback...");
                        led_b.set_high();
                        if let Some(ts) = http_date_poll(stack).await {
                            let dt = unix_to_datetime(ts);
                            let (y, mo, d, hh, mi, ss) =
                                (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
                            let _ = rtc.set_datetime(dt);
                            info!(
                                "Resync SUCCESS via HTTP. RTC={:04}/{:02}/{:02} {:02}:{:02}:{:02}",
                                y, mo, d, hh, mi, ss
                            );
                            let _ = blink(&mut led_b, 2, 60, 120).await;
                            last_method = Some(SyncMethod::Http);
                        } else {
                            error!("Resync FAILED (NTP then HTTP). RESETTING...");
                            let _ = blink(&mut led_r, 5, 80, 100).await;
                            reboot();
                        }
                        led_b.set_low();
                    }
                    led_g.set_low();
                }
                _ => {
                    info!("Periodic resync using last successful method: HTTP");
                    led_b.set_high();
                    if let Some(ts) = http_date_poll(stack).await {
                        let dt = unix_to_datetime(ts);
                        let (y, mo, d, hh, mi, ss) =
                            (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
                        let _ = rtc.set_datetime(dt);
                        info!(
                            "Resync SUCCESS via HTTP. RTC={:04}/{:02}/{:02} {:02}:{:02}:{:02}",
                            y, mo, d, hh, mi, ss
                        );
                        let _ = blink(&mut led_b, 2, 60, 120).await;
                        last_method = Some(SyncMethod::Http);
                    } else {
                        led_b.set_low();
                        warn!("Resync HTTP failed; trying NTP fallback...");
                        led_g.set_high();
                        if let Some((ts, ip)) = ntp_poll(stack).await {
                            let dt = unix_to_datetime(ts);
                            let (y, mo, d, hh, mi, ss) =
                                (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
                            let _ = rtc.set_datetime(dt);
                            info!(
                                "Resync SUCCESS via NTP ({}). RTC={:04}/{:02}/{:02} {:02}:{:02}:{:02}",
                                defmt::Display2Format(&ip),
                                y, mo, d, hh, mi, ss
                            );
                            let _ = blink(&mut led_g, 2, 60, 120).await;
                            last_method = Some(SyncMethod::Ntp(ip));
                        } else {
                            error!("Resync FAILED (HTTP then NTP). RESETTING...");
                            let _ = blink(&mut led_r, 5, 80, 100).await;
                            reboot();
                        }
                        led_g.set_low();
                    }
                }
            }
        }
    }
}

// ---- 發散函式：重開機（消除 unreachable / unused_unsafe 警告）----
#[inline(never)]
fn reboot() -> ! {
    SCB::sys_reset();
}
