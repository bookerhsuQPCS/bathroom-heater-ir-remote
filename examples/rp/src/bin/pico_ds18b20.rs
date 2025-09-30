#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, OutputOpenDrain};
use embassy_time::{Delay, Timer};
use {embassy_rp as rp, panic_probe as _};

use onewire::{ds18b20, ds18b20::DS18B20, DeviceSearch, OneWire};

// 想換腳驗證，改這裡就好，線跟著移
macro_rules! DQ_PIN {
    ($p:ident) => {
        $p.PIN_13
    };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("DS18B20 on GPIO13 (Embassy + onewire eh1.0, defmt enabled)");

    let p = rp::init(Default::default());

    // 讓 DQ 處於「開漏且釋放（Hi-Z）」狀態，理論上外部 4.7kΩ 會把它拉到 3V3
    let dq = OutputOpenDrain::new(DQ_PIN!(p), Level::High);

    // 給匯流排更長的上拉穩定時間：最多等 500ms，10ms 檢查一次
    let mut waited_ms: u32 = 0;
    while !dq.is_high() && waited_ms < 500 {
        Timer::after_millis(10).await;
        waited_ms += 10;
    }
    let idle_high = dq.is_high();
    info!("Preflight: DQ idle-high = {} (waited {} ms)", idle_high, waited_ms);
    if !idle_high {
        error!("DQ 仍未被拉高：請重新檢查 3V3(OUT)↔GPIO 的 4.7kΩ、腳位是否正確、是否短路。");
        loop { Timer::after_millis(1000).await; }
    }

    // 非寄生供電（VDD 有接）。若兩線寄生需要 true（不建議一開始就用）
    let mut bus = OneWire::new(dq, false);
    let mut delay = Delay;

    // 匯流排 reset
    match bus.reset(&mut delay) {
        Ok(presence) => {
            if presence {
                info!("1-Wire reset OK: device PRESENT");
            } else {
                warn!("1-Wire reset OK: NO device responded（匯流排正常但找不到裝置；再檢查 DQ 是否接到那顆）");
            }
        }
        Err(e) => {
            error!("1-Wire reset FAILED: {:?}", e);
            loop { Timer::after_millis(1000).await; }
        }
    }

    // 只搜尋 DS18B20 (0x28)
    let mut search = DeviceSearch::new_for_family(ds18b20::FAMILY_CODE);

    info!("Scanning devices on DQ...");
    loop {
        match bus.search_next(&mut search, &mut delay) {
            Ok(Some(dev)) => {
                let addr = dev.address; // 備份 ROM
                let sensor = match DS18B20::new(dev) {
                    Ok(s) => s,
                    Err(e) => {
                        warn!(
                            "Not a DS18B20? addr={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, err={:?}",
                            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7], e
                        );
                        continue;
                    }
                };

                let ms = match sensor.measure_temperature(&mut bus, &mut delay) {
                    Ok(t) => t.time_ms(),
                    Err(e) => {
                        warn!("measure_temperature failed: {:?}", e);
                        continue;
                    }
                };
                Timer::after_millis(ms as u64).await;

                match sensor.read_temperature(&mut bus, &mut delay) {
                    Ok(raw) => {
                        let (intc, frac) = ds18b20::split_temp(raw);
                        info!(
                            "ROM={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}  T={}.{:04}°C",
                            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7],
                            intc, frac
                        );
                    }
                    Err(e) => {
                        warn!("read_temperature failed: {:?}", e);
                    }
                }
            }
            Ok(None) => {
                info!("Scan done.");
                break;
            }
            Err(e) => {
                warn!("search_next error: {:?}", e);
                break;
            }
        }
    }

    loop { Timer::after_secs(2).await; }
}
