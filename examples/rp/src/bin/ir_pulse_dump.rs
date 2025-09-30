#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Instant, Timer};
use {embassy_rp as rp, rp::bind_interrupts};

bind_interrupts!(struct Irqs {});

// ========= 可調參數 =========
const IR_PIN_IS_PULLUP: bool = true;     // 大多 IR 接收頭要上拉
const INVERT_LEVEL: bool = false;        // 若抓不到/極性相反，改成 true 再試（多數接收頭：低=有載波=MARK）
const MIN_US: u32 = 180;                 // 忽略過短毛邊（180~240 視環境微調）
const ACTIVE_WINDOW_US: u32 = 12_000;    // 只用來標示 frame 邊界（長靜默），不做解碼
const LED_ON_MS: u64 = 25;
const LED_OFF_MS: u64 = 25;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rp::init(Default::default());

    let mut ir_in = if IR_PIN_IS_PULLUP {
        Input::new(p.PIN_14, Pull::Up)   // 你的線路：IN=GP14
    } else {
        Input::new(p.PIN_14, Pull::None)
    };
    let mut led = Output::new(p.PIN_22, Level::Low); // LED=GP22

    let mut last_edge = Instant::now();
    let mut last_level = ir_in.get_level();
    let mut edges_in_frame: u32 = 0;

    info!("IR pulse dump start: pin=GP14, base=1MHz(us), LED=GP22");
    loop {
        ir_in.wait_for_any_edge().await;

        let now = Instant::now();
        let mut dt_us64 = (now - last_edge).as_micros();
        if dt_us64 > core::u32::MAX as u64 { dt_us64 = core::u32::MAX as u64; }
        let dt_us = dt_us64 as u32;

        // 過短毛邊：完全忽略，不更新 last_edge/level（避免切碎）
        if dt_us < MIN_US {
            continue;
        }

        // 長靜默 → 標示 frame 邊界（方便你閱讀），不影響後續量測
        if dt_us > ACTIVE_WINDOW_US {
            if edges_in_frame > 0 {
                info!("=== FRAME END (gap {=u32}us, edges={=u32}) ===", dt_us, edges_in_frame);
                edges_in_frame = 0;
                // 提示一下
                led.set_high();
                Timer::after_millis(LED_ON_MS).await;
                led.set_low();
                Timer::after_millis(LED_OFF_MS).await;
            }
        }

        // 這段脈衝對應的是「邊緣之前」的電位
        let prev_level = last_level;
        last_level = ir_in.get_level();
        last_edge = now;

        // 以「低=有載波」為 MARK；可由 INVERT_LEVEL 翻轉
        let mut is_mark = prev_level == Level::Low;
        if INVERT_LEVEL { is_mark = !is_mark; }
        let dt_signed: i32 = if is_mark { dt_us as i32 } else { -(dt_us as i32) };

        // 直接印「on/off（MARK/SPACE）長度」
        let label = if is_mark { "MARK" } else { "SPACE" };
        info!("[RAW] {=i32}us ({})", dt_signed, label);
        edges_in_frame = edges_in_frame.saturating_add(1);
    }
}
