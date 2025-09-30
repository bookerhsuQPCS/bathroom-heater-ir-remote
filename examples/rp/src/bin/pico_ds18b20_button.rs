#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, OutputOpenDrain, Pull};
use embassy_time::{Delay, Timer};
use {embassy_rp as rp, panic_probe as _};

use onewire::{ds18b20, ds18b20::DS18B20, DeviceSearch, OneWire};

// 換腳就改這裡，線也要一起移
macro_rules! DQ_PIN {
    ($p:ident) => {
        $p.PIN_13 // DS18B20 的 DQ 腳
    };
}
macro_rules! BTN_PIN {
    ($p:ident) => {
        $p.PIN_15 // 按鈕腳位（按下接地）
    };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Pico W + DS18B20 (button-triggered) | DQ=GPIO14, BTN=GPIO15");

    let p = rp::init(Default::default());

    // 按鈕：上拉，按下接地 => is_low() 代表「按下」
    let btn = Input::new(BTN_PIN!(p), Pull::Up);

    // DQ：開漏輸出、釋放為高（Hi-Z），交由外部 2.2–4.7kΩ 上拉到 3V3(OUT)
    let dq = OutputOpenDrain::new(DQ_PIN!(p), Level::High);

    // 啟動 Preflight：等待匯流排真的被拉到高（最多 1000ms）
    let mut waited_ms: u32 = 0;
    while !dq.is_high() && waited_ms < 1000 {
        Timer::after_millis(5).await;
        waited_ms += 5;
    }
    info!("Preflight: DQ idle-high = {} (waited {} ms)", dq.is_high(), waited_ms);
    if !dq.is_high() {
        error!("DQ 未被拉高：請確認 GPIO14↔3V3(OUT) 有 2.2–4.7kΩ 上拉、腳位/3V3 無誤、沒有短路。");
        loop { Timer::after_millis(1000).await; }
    }

    // 三線供電（VDD 有接）。寄生模式請先別用。
    let mut bus = OneWire::new(dq, false);
    let mut delay = Delay;

    info!("按下 GPIO15 按鈕開始量測…");

    loop {
        // 等待按下（低電位），簡單輪詢 + 去彈跳
        while btn.is_high() {
            Timer::after_millis(5).await;
        }
        Timer::after_millis(30).await; // debounce
        if btn.is_high() {
            continue; // 抖動誤觸
        }
        info!("按鈕按下 → reset / 掃描 / 量測");

        // 每次觸發都先 reset 一次
        match bus.reset(&mut delay) {
            Ok(presence) => {
                info!("reset presence={}", presence);
                if !presence {
                    warn!("總線正常但沒有裝置回應，請檢查 DQ 接線與裝置電源。");
                }
                // 小空檔讓匯流排回高
                Timer::after_millis(2).await;
            }
            Err(e) => {
                error!("reset FAILED: {:?}", e);
                wait_button_release(&btn).await;
                continue;
            }
        }

        // 用 DeviceSearch 尋找 DS18B20 家族（0x28）
        let mut search = DeviceSearch::new_for_family(ds18b20::FAMILY_CODE);
        let mut found = 0u8;

        loop {
            // 給匯流排一點時間回到高
            Timer::after_millis(2).await;

            match bus.search_next(&mut search, &mut delay) {
                Ok(Some(dev)) => {
                    found += 1;
                    handle_one_sensor(dev, &mut bus, &mut delay).await;
                }
                Ok(None) => {
                    info!("搜尋結束，共 {} 顆。", found);
                    break;
                }
                Err(e) => {
                    warn!("search_next error: {:?}", e);
                    break;
                }
            }
        }

        // 等待按鈕放開（避免重複觸發）
        wait_button_release(&btn).await;
        info!("放開按鈕，可再次按下重新量測。");
    }
}

// 量測一顆感測器：觸發轉換→等待→讀溫度
async fn handle_one_sensor(
    dev: onewire::Device,
    bus: &mut OneWire<OutputOpenDrain<'static>>,
    delay: &mut Delay,
) {
    let addr = dev.address;
    let sensor = match DS18B20::new(dev) {
        Ok(s) => s,
        Err(e) => {
            warn!(
                "非 DS18B20? addr={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, err={:?}",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], e
            );
            return;
        }
    };

    // 觸發量測並取得建議等待時間（解析度 9–12bit 約 94–750ms）
    let ms = match sensor.measure_temperature(bus, delay) {
        Ok(t) => t.time_ms(),
        Err(e) => {
            warn!("measure_temperature 失敗: {:?}", e);
            return;
        }
    };
    Timer::after_millis(ms as u64).await;

    // 讀出溫度
    match sensor.read_temperature(bus, delay) {
        Ok(raw) => {
            let (intc, frac) = ds18b20::split_temp(raw);
            info!(
                "ROM={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}  T={}.{:04}°C",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], intc, frac
            );
        }
        Err(e) => {
            warn!("read_temperature 失敗: {:?}", e);
        }
    }

    // 留一點空檔，讓匯流排回高
    Timer::after_millis(2).await;
}

// 等待按鈕放開 + 去彈跳
async fn wait_button_release(btn: &Input<'_>) {
    while btn.is_low() {
        Timer::after_millis(5).await;
    }
    Timer::after_millis(30).await; // debounce
}
