#![no_std]
#![no_main]

// pico_ds18b20_button_autorescan.rs
//
// Pico W + DS18B20 + Button + LED (GPIO13/15/22)
// - 短按：每次掃描→量測（沿用舊程式 measure_temperature / read_temperature）
// - 長按：自動模式（每 15 秒量測），進入時只掃描一次建立快取；
//         逾時或連續失敗達閾值則自動重掃
// - 量測成功 LED22 閃 100ms

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, OutputOpenDrain, Pull};
use embassy_time::{Delay, Instant, Timer};
use {embassy_rp as rp, panic_probe as _};

use heapless::Vec;
use onewire::{ds18b20, ds18b20::DS18B20, Device, DeviceSearch, OneWire};

const LONG_PRESS_MS: u32 = 800;
const PERIOD_MS: u32 = 15_000;
const RESCAN_TIMEOUT_MS: u64 = 300_000; // 5 分鐘
const MAX_DEVICES: usize = 8;

// 連續失敗重掃設定
const FAIL_RESCAN_THRESHOLD: u8 = 3;

macro_rules! DQ_PIN { ($p:ident) => { $p.PIN_13 }; } // DS18B20 DQ
macro_rules! BTN_PIN { ($p:ident) => { $p.PIN_15 }; } // Button（Active-Low）
macro_rules! LED_PIN { ($p:ident) => { $p.PIN_22 }; } // LED

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("pico_ds18b20_button_autorescan | DQ=GPIO13, BTN=GPIO15, LED=GPIO22");

    let p = rp::init(Default::default());

    // Button & LED
    let btn = Input::new(BTN_PIN!(p), Pull::Up);
    let mut led = Output::new(LED_PIN!(p), Level::Low);

    // OneWire：舊程式風格
    let dq = OutputOpenDrain::new(DQ_PIN!(p), Level::High);
    let mut bus: OneWire<OutputOpenDrain<'static>> = OneWire::new(dq, /* parasite = */ false);

    let mut delay = Delay;

    // 自動模式狀態
    let mut auto = false;
    let mut cached: Vec<Device, MAX_DEVICES> = Vec::new();
    let mut last_scan: Instant = Instant::now();

    // 連續失敗計數（自動模式）
    let mut consecutive_failures: u8 = 0;

    loop {
        if !auto {
            // 等待按下
            while btn.is_high() { Timer::after_millis(5).await; }
            Timer::after_millis(30).await; // 去抖
            if btn.is_high() { continue; }

            // 判斷長按/短按
            let mut held = 0u32;
            while btn.is_low() && held < LONG_PRESS_MS {
                Timer::after_millis(20).await;
                held += 20;
            }

            if held >= LONG_PRESS_MS {
                // 進入自動模式：掃描一次並快取、清零失敗計數
                auto = true;
                rescan_and_cache(&mut bus, &mut delay, &mut cached).await;
                last_scan = Instant::now();
                consecutive_failures = 0;
                wait_button_release(&btn).await;
            } else {
                // 短按：每次都掃描→量測
                single_scan_and_measure(&mut bus, &mut delay, &mut led).await;
                wait_button_release(&btn).await;
            }
        } else {
            // 逾時重掃
            if Instant::now().duration_since(last_scan).as_millis() as u64 >= RESCAN_TIMEOUT_MS {
                info!("逾時重掃");
                rescan_and_cache(&mut bus, &mut delay, &mut cached).await;
                last_scan = Instant::now();
                consecutive_failures = 0;
            }

            // 使用快取量測
            let (ok_cnt, fail_cnt) = measure_cached_devices(
                &cached, &mut bus, &mut delay, &mut led
            ).await;

            // 連續失敗判斷
            if ok_cnt == 0 && fail_cnt > 0 {
                consecutive_failures = consecutive_failures.saturating_add(1);
                warn!("自動模式：本輪全部失敗，連續失敗次數={}", consecutive_failures);
            } else if ok_cnt > 0 {
                consecutive_failures = 0;
            }

            if consecutive_failures >= FAIL_RESCAN_THRESHOLD {
                warn!("連續失敗達到閾值 {}，立即重掃。", FAIL_RESCAN_THRESHOLD);
                rescan_and_cache(&mut bus, &mut delay, &mut cached).await;
                last_scan = Instant::now();
                consecutive_failures = 0;
            }

            // 等待期間允許長按退出
            if wait_with_longpress_cancel(&btn, PERIOD_MS).await {
                auto = false;
                wait_button_release(&btn).await;
            }
        }
    }
}

// ──────────────── 短按：掃描＋量測 ────────────────
async fn single_scan_and_measure(
    bus: &mut OneWire<OutputOpenDrain<'static>>,
    delay: &mut Delay,
    led: &mut Output<'_>,
) {
    match bus.reset(delay) {
        Ok(presence) => {
            info!("reset presence={}", presence);
            if !presence {
                warn!("匯流排無裝置回應。");
            }
            Timer::after_millis(2).await;
        }
        Err(e) => {
            error!("reset 失敗: {:?}", e);
            return;
        }
    }

    let mut search = DeviceSearch::new_for_family(ds18b20::FAMILY_CODE);
    let mut found = 0u8;

    loop {
        Timer::after_millis(2).await;
        match bus.search_next(&mut search, delay) {
            Ok(Some(dev)) => {
                found += 1;
                let _ = measure_one_with_old_path(dev, bus, delay, led).await;
            }
            Ok(None) => {
                info!("搜尋結束，共 {} 顆", found);
                break;
            }
            Err(e) => {
                warn!("search_next 錯誤: {:?}", e);
                break;
            }
        }
    }
}

// ──────────────── 自動模式：掃描建立快取 ────────────────
async fn rescan_and_cache(
    bus: &mut OneWire<OutputOpenDrain<'static>>,
    delay: &mut Delay,
    cached: &mut Vec<Device, MAX_DEVICES>,
) {
    cached.clear();

    match bus.reset(delay) {
        Ok(_) => Timer::after_millis(2).await,
        Err(e) => { error!("reset 失敗: {:?}", e); return; }
    }

    let mut search = DeviceSearch::new_for_family(ds18b20::FAMILY_CODE);
    while let Ok(Some(dev)) = bus.search_next(&mut search, delay) {
        let _ = cached.push(dev);
        Timer::after_millis(2).await;
    }
    info!("快取裝置數：{}", cached.len());
}

// 回傳（成功次數, 失敗次數）
async fn measure_cached_devices(
    cached: &Vec<Device, MAX_DEVICES>,
    bus: &mut OneWire<OutputOpenDrain<'static>>,
    delay: &mut Delay,
    led: &mut Output<'_>,
) -> (u8, u8) {
    if cached.is_empty() {
        warn!("快取為空，無可量測裝置。");
        return (0, 1);
    }

    let mut ok_cnt: u8 = 0;
    let mut fail_cnt: u8 = 0;

    for dev in cached.iter().cloned() {
        match bus.reset(delay) {
            Ok(_) => Timer::after_millis(2).await,
            Err(e) => {
                warn!("reset 失敗（略過）: {:?}", e);
                fail_cnt = fail_cnt.saturating_add(1);
                continue;
            }
        }

        if measure_one_with_old_path(dev, bus, delay, led).await {
            ok_cnt = ok_cnt.saturating_add(1);
        } else {
            fail_cnt = fail_cnt.saturating_add(1);
        }
    }

    (ok_cnt, fail_cnt)
}

// ──────────────── 舊路徑：measure_temperature → read_temperature ────────────────
async fn measure_one_with_old_path(
    dev: Device,
    bus: &mut OneWire<OutputOpenDrain<'static>>,
    delay: &mut Delay,
    led: &mut Output<'_>,
) -> bool {
    let addr = dev.address;

    let sensor = match DS18B20::new(dev) {
        Ok(s) => s,
        Err(e) => {
            warn!(
                "非 DS18B20? addr={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, err={:?}",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], e
            );
            return false;
        }
    };

    let wait_ms = match sensor.measure_temperature(bus, delay) {
        Ok(t) => t.time_ms(),
        Err(e) => {
            warn!("measure_temperature 失敗: {:?}", e);
            return false;
        }
    };
    Timer::after_millis(wait_ms as u64).await;

    match sensor.read_temperature(bus, delay) {
        Ok(temp_u16) => {
            let (intc, frac) = ds18b20::split_temp(temp_u16);
            info!("raw=0x{:04x} → 溫度 = {}.{} °C", temp_u16, intc, frac);

            led.set_high();
            Timer::after_millis(100).await;
            led.set_low();

            Timer::after_millis(2).await;
            true
        }
        Err(e) => {
            warn!("read_temperature 失敗: {:?}", e);
            Timer::after_millis(2).await;
            false
        }
    }
}

// ──────────────── 按鈕工具 ────────────────
async fn wait_button_release(btn: &Input<'_>) {
    while btn.is_low() { Timer::after_millis(5).await; }
    Timer::after_millis(30).await;
}

async fn wait_with_longpress_cancel(btn: &Input<'_>, total_ms: u32) -> bool {
    let mut elapsed = 0u32;
    let mut hold = 0u32;
    while elapsed < total_ms {
        Timer::after_millis(20).await;
        elapsed += 20;
        if btn.is_low() {
            hold += 20;
            if hold >= LONG_PRESS_MS { return true; }
        } else {
            hold = 0;
        }
    }
    false
}
