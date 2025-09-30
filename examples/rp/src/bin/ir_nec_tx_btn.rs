#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Delay, Instant};
use {defmt_rtt as _, panic_probe as _};

use infrared::protocol::nec::{Nec, NecCommand};
use infrared::tx::fixed::{send_encoded_blocking, FixedFreqCarrier}; 
use infrared::Protocol;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("IR (NEC) TX by button. BTN=GP15, LED=GP22, IR=GP16. Repeat immediately.");
    let p = embassy_rp::init(Default::default());

    // 板載 LED 當狀態指示
    let mut led = Output::new(p.PIN_22, Level::Low);

    // 按鈕：GP15，上拉，按下接地（active-low）
    let mut btn = Input::new(p.PIN_15, Pull::Up);

    // 紅外線輸出腳位（經過三極體驅動 IR LED）
    let mut ir_out = Output::new(p.PIN_16, Level::Low);

    // --- 固定 NEC Command 值 ---
    // 地址：0x00FF，命令：0x12
    let nec_cmd = NecCommand::new(0x00FF, 0x12);

    // 去抖時間
    let mut last = Instant::now();

    loop {
        // 等待按下（低電位）
        btn.wait_for_low().await;

        // 簡單去抖：50ms 內忽略
        let now = Instant::now();
        if (now - last).as_millis() < 50 {
            btn.wait_for_high().await;
            continue;
        }
        last = now;

        led.set_high();

        // 建立 NEC 編碼（完整幀）
        let nec = Nec;
        let frame = nec.encode(nec_cmd).unwrap();

        // 38 kHz 載波，占空比 ~33%，用 Delay 做 busy-wait
        let mut delay = Delay;
        let mut carrier = FixedFreqCarrier::new(&mut ir_out, &mut delay, 38_000, 3);

        // **無間隔發射兩次**
        match send_encoded_blocking(&mut carrier, &frame) {
            Ok(_) => info!("NEC sent once."),
            Err(_) => warn!("NEC send error (1st)."),
        }
        match send_encoded_blocking(&mut carrier, &frame) {
            Ok(_) => info!("NEC sent twice (immediate repeat)."),
            Err(_) => warn!("NEC send error (2nd)."),
        }

        led.set_low();

        // 等待放開，並做一點點釋放去抖
        btn.wait_for_high().await;
        embassy_time::Timer::after_millis(30).await;
    }
}
