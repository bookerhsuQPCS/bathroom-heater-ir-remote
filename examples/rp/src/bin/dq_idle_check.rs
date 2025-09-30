#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Flex, Pull};
use embassy_time::{Duration, Timer};
use {embassy_rp as rp, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("DQ idle-level checker on GPIO13");

    let p = rp::init(Default::default());

    // 用 Flex 才能動態切 Pull
    let mut dq = Flex::new(p.PIN_14);
    dq.set_as_input();

    // 1) 不開內建上拉，讀真實線路狀態（應該要靠「外部 4.7kΩ → 3V3」被拉高）
    dq.set_pull(Pull::None);
    for _ in 0..10 {
        let high = dq.is_high();
        info!("Pull=None -> DQ={}", if high { "HIGH" } else { "LOW" });
        Timer::after(Duration::from_millis(100)).await;
    }

    // 2) 開 RP2040 內建上拉（很弱，只能輔助）
    dq.set_pull(Pull::Up);
    for _ in 0..10 {
        let high = dq.is_high();
        info!("Pull=Up   -> DQ={}", if high { "HIGH" } else { "LOW" });
        Timer::after(Duration::from_millis(100)).await;
    }

    info!("結論提示：Pull=None 若仍 LOW，多半是缺 4.7kΩ 上拉或 DQ 短路/接錯；Pull=Up 才 HIGH 代表外部上拉未接。");
    loop { Timer::after_secs(1).await; }
}
