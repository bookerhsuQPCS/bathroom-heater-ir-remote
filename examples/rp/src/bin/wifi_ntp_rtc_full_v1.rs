// src/bin/wifi_ntp_rtc_full_v1.rs
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
fn ntp_servers() -> [Ipv4Address; 3] {
    [
        Ipv4Address::new(211, 22, 103, 158),  // 中華
        Ipv4Address::new(133, 243, 238, 163), // NICT
        Ipv4Address::new(192, 168, 188, 1),   // routerOS
    ]
}

// ===== NTP 單次查詢（5s 逾時）=====
#[allow(static_mut_refs)]
async fn ntp_once(stack: &net::Stack<'static>, server: Ipv4Address) -> Result<u64, ()> {
    // 建立 socket buffer 與 metadata (必須是 static mut)
    static mut RX_META: [PacketMetadata<UdpMetadata>; 4] = [PacketMetadata::EMPTY; 4];
    static mut RX_BUF: [u8; 512] = [0; 512];
    static mut TX_META: [PacketMetadata<UdpMetadata>; 4] = [PacketMetadata::EMPTY; 4];
    static mut TX_BUF: [u8; 512] = [0; 512];

    let mut socket = unsafe {
        UdpSocket::new(
            stack,
            &mut RX_META,
            &mut RX_BUF,
            &mut TX_META,
            &mut TX_BUF,
        )
    };

    // 綁定任意本地 port
    unwrap!(socket.bind(IpEndpoint::new(IpAddress::v4(0, 0, 0, 0), 0)));

    let endpoint = IpEndpoint::new(IpAddress::Ipv4(server), 123);

    // NTP 請求封包 (48 bytes，全填 0，只設定第一個 byte = 0x1B)
    let mut buf = [0u8; 48];
    buf[0] = 0x1B;

    unwrap!(socket.send_to(&buf, endpoint).await);

    let mut recv_buf = [0u8; 48];
    let (n, _from) = match socket.recv_from(&mut recv_buf).await {
        Ok(x) => x,
        Err(_) => return Err(()),
    };

    if n < 48 {
        return Err(());
    }

    // 取出 transmit timestamp (第 40~47 bytes)
    let secs = u32::from_be_bytes([recv_buf[40], recv_buf[41], recv_buf[42], recv_buf[43]]);
    let frac = u32::from_be_bytes([recv_buf[44], recv_buf[45], recv_buf[46], recv_buf[47]]);

    // NTP epoch (1900) -> Unix epoch (1970)
    let unix_time = secs as u64 - 2_208_988_800u64;
    let micros = ((frac as u64) * 1_000_000) >> 32;

    Ok(unix_time * 1_000_000 + micros)
}

// ===== NTP 輪詢（每台只嘗試一次）=====
async fn ntp_poll(stack: &Stack, servers: &[Ipv4Address]) -> Option<u64> {
    for &server in servers {
        defmt::info!("NTP try server = {:?}", server);
        match ntp_once(stack, server).await {
            Ok(t) => {
                defmt::info!("NTP success from {:?}", server);
                return Some(t);
            }
            Err(_) => defmt::warn!("NTP failed for {:?}", server),
        }
    }
    defmt::error!("All NTP servers failed");
    None
}

// ===== HTTP Date 後備（1.1.1.1:80）=====
#[allow(static_mut_refs)]
async fn http_date_poll(stack: net::Stack<'static>) -> Option<u64> {
    static mut RX: [u8; 1024] = [0; 1024];
    static mut TX: [u8; 512]  = [0; 512];

    let rx = unsafe { &mut RX };
    let tx = unsafe { &mut TX };
    let mut sock = TcpSocket::new(stack, rx, tx);

    // 5s 逾時的 connect
    let ep = IpEndpoint::new(IpAddress::v4(1, 1, 1, 1), 80);
    if embassy_time::with_timeout(Duration::from_secs(5), sock.connect(ep)).await.is_err() {
        warn!("HTTP connect timeout");
        return None;
    } else {
        info!("HTTP connect success to {}", ep);
    }

    // 簡單 GET（Connection: close）
    let req = b"GET / HTTP/1.1\r\nHost: 1.1.1.1\r\nConnection: close\r\nUser-Agent: pico-w\r\n\r\n";
    if sock.write(req).await.is_err() {
        warn!("HTTP write error");
        let _ = sock.close();
        return None;
    }
    let _ = sock.flush().await;

    // 讀到 header 結束或 EOF（最多 5 秒）
    let deadline = Instant::now() + Duration::from_secs(5);
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
            Ok(Ok(0)) => break, // EOF
            Ok(Ok(n)) => {
                total += n;
                // 其他處理
            }
            Ok(Err(e)) => {
                warn!("HTTP read error: {:?}", e);
                let _ = sock.close();
                return None;
            }
            Err(_) => {
                warn!("HTTP recv timeout");
                let _ = sock.close();
                return None;
            }
        }
    }
    let _ = sock.close();

    parse_http_date(&buf[..total])
}

// 解析 RFC7231 Date 標頭 → UNIX 秒（UTC）
fn parse_http_date(payload: &[u8]) -> Option<u64> {
    let s = core::str::from_utf8(payload).ok()?;
    let mut date_line: Option<&str> = None;
    for line in s.lines() {
        // 簡單大小寫處理：常見為 "Date:"，少數可能 "date:"；這裡只檢查 D/d
        let bytes = line.as_bytes();
        if bytes.len() >= 5
            && (bytes[0] == b'D' || bytes[0] == b'd')
            && (&bytes[1..5]).eq_ignore_ascii_case(b"ate:")
        {
            // 跳過 "Date:" 之後的空白
            let mut i = 5;
            while i < bytes.len() && (bytes[i] == b' ' || bytes[i] == b'\t') { i += 1; }
            date_line = Some(&line[i..]);
            break;
        }
    }
    let dl = date_line?;

    // 典型格式：Sun, 06 Nov 1994 08:49:37 GMT
    // 先找逗號，如果有，就從逗號後開始
    let dl = dl.trim();
    let dl = if let Some(idx) = dl.as_bytes().iter().position(|&c| c == b',') {
        dl.get(idx+1..)?.trim()
    } else {
        dl
    };

    // 以空白切 5 個 token：DD Mon YYYY HH:MM:SS GMT
    let (day_str, rest1) = next_token(dl)?;
    let (mon_str, rest2) = next_token(rest1)?;
    let (year_str, rest3) = next_token(rest2)?;
    let (hms_str, _rest4) = next_token(rest3)?;

    let day: u32 = day_str.parse().ok()?;
    let mon: u32 = match_mon(mon_str)?;
    let year: u32 = year_str.parse().ok()?;

    // HH:MM:SS
    let mut it = hms_str.split(':');
    let hh: u32 = it.next()?.parse().ok()?;
    let mm: u32 = it.next()?.parse().ok()?;
    let ss: u32 = it.next()?.parse().ok()?;

    // -> UNIX 秒
    let days = ymd_to_days(year as i32, mon as i32, day as i32)? - 719_163; // 1970-01-01
    if days < 0 { return None; }
    let secs = (days as u64) * 86_400 + (hh as u64) * 3600 + (mm as u64) * 60 + (ss as u64);
    Some(secs)
}

// 從字串開頭取下一個以空白分隔的 token
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

// ===== 公曆換算（Rata Die）=====
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

// ===== UNIX 秒 → RP2040 RTC DateTime（UTC）=====
fn unix_to_datetime(mut t: u64) -> DateTime {
    let sec = (t % 60) as u8;
    t /= 60;
    let min = (t % 60) as u8;
    t /= 60;
    let hour = (t % 24) as u8;
    let days = (t / 24) as i32;

    // 1970-01-01 的 Rata Die 是 719163
    let z = days + 719_163;
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

// ===== main =====
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Pico W NTP→RTC with HTTP fallback (5s, 1 try/server, human-readable)");

    let p = embassy_rp::init(Default::default());

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
    let (net_device, mut control, cyw_runner) = cyw43::new(state, pwr, spi, FW).await;
    unwrap!(spawner.spawn(cyw43_task(cyw_runner)));

    // 關鍵：初始化 CLM，避免 MAC=00 導致 DHCP 不理你
    control.init(CLM).await;

    static RESOURCES: StaticCell<net::StackResources<4>> = StaticCell::new();
    let resources = RESOURCES.init(net::StackResources::<4>::new());
    let seed: u64 = Instant::now().as_ticks() as u64 ^ 0x5EED_1234_5678_ABCDu64;

    let config = NetConfig::dhcpv4(Default::default());
    let (stack, _net_runner) = net::new(net_device, config, resources, seed);
    unwrap!(spawner.spawn(net_task(_net_runner)));

    // 連網
    loop {
        match control
            .join(wifi_ssid(), JoinOptions::new(wifi_pass().as_bytes()))
            .await
        {
            Ok(()) => break,
            Err(e) => {
                warn!("join failed: status={}", e.status);
                Timer::after_millis(800).await;
            }
        }
    }
    info!("Wi-Fi joined: {}", wifi_ssid());
    stack.wait_link_up().await;
    stack.wait_config_up().await;

    // ARP/路由暖機一下，避免第一輪 NTP 就遇到 NeighborPending
    Timer::after_secs(1).await;

    let mut rtc = Rtc::new(p.RTC);

    // 先 NTP → 失敗才 HTTP Date
    let ts_opt = match ntp_poll(stack).await {
        Some(ts) => Some(ts),
        None => {
            warn!("NTP poll failed, fallback to HTTP Date poll");
            http_date_poll(stack).await
        }
    };

    if let Some(ts) = ts_opt {
        // 寫進 RTC（UTC）
        let dt = unix_to_datetime(ts);
        // 先取出欄位，避免 set_datetime 移動 dt 後無法再使用
        let (y, mo, d, hh, mi, ss) = (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
        let _ = rtc.set_datetime(dt);
        info!("RTC set: {:04}/{:02}/{:02} {:02}:{:02}:{:02}", y, mo, d, hh, mi, ss);
    } else {
        warn!("Time sync failed (NTP & HTTP).");
    }

    // 保持運行；若要週期性重對時，之後在這裡加即可
    loop {
        Timer::after_secs(5).await;
    }
}
