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
// 硬體/時序
const IR_PIN_IS_PULLUP: bool = true;     // 多數 IR 接收頭需上拉
const INVERT_LEVEL: bool = false;        // 若極性相反/抓不到，改 true（常見：低=有載波=MARK）
const MIN_US: u32 = 180;                 // 忽略過短毛邊（180~240 視環境微調）
const ACTIVE_WINDOW_US: u32 = 12_000;    // frame 之間的長靜默門檻（NEC 典型 ~40~50ms，這裡只用來切段）
const LED_ON_MS: u64 = 25;
const LED_OFF_MS: u64 = 25;

const MAX_FRAME_SEGS: usize = 256;       // 每 frame 收集的段數上限（NEC 約 68 段）

// 是否輸出為「全正值」（更像 IRremote 的 array）
// true  => 全部轉正值
// false => 保留正負號（MARK 正、SPACE 負）
const ARRAY_ALL_POSITIVE: bool = false;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rp::init(Default::default());

    let mut ir_in = if IR_PIN_IS_PULLUP {
        Input::new(p.PIN_14, Pull::Up)   // IN = GP14
    } else {
        Input::new(p.PIN_14, Pull::None)
    };
    let mut led = Output::new(p.PIN_22, Level::Low); // LED = GP22

    let mut last_edge = Instant::now();
    let mut last_level = ir_in.get_level();

    // 當前 frame 的緩衝
    let mut segs: [i32; MAX_FRAME_SEGS] = [0; MAX_FRAME_SEGS];
    let mut n: usize = 0;
    let mut edges_in_frame: u32 = 0;

    info!("IR pulse dump v4 (single-line): pin=GP14, 1MHz(us), LED=GP22");

    loop {
        ir_in.wait_for_any_edge().await;

        let now = Instant::now();
        let mut dt_us64 = (now - last_edge).as_micros();
        if dt_us64 > core::u32::MAX as u64 { dt_us64 = core::u32::MAX as u64; }
        let dt_us = dt_us64 as u32;

        // 忽略過短毛邊：不更新 last_edge/level（避免把真脈衝切碎）
        if dt_us < MIN_US {
            continue;
        }

        // 若長時間靜默，視為 frame 邊界。
        if dt_us > ACTIVE_WINDOW_US {
            if edges_in_frame > 0 && n > 0 {
                // 只印「單行 frame 陣列」
                print_frame_array(&segs[..n], dt_us);
                // 亮個燈提示
                led.set_high();
                Timer::after_millis(LED_ON_MS).await;
                led.set_low();
                Timer::after_millis(LED_OFF_MS).await;
            }
            // 重置 frame 緩衝，並把這個長 gap 丟掉（不將它記為一段）
            n = 0;
            edges_in_frame = 0;
            last_edge = now;
            last_level = ir_in.get_level();
            continue; // 關鍵：避免把長 gap 再當成一段印出
        }

        // 這段脈衝對應的是「邊緣之前」的電位
        let prev_level = last_level;
        last_level = ir_in.get_level();
        last_edge = now;

        // 以「低=有載波」為 MARK；INVERT_LEVEL 可翻轉
        let mut is_mark = prev_level == Level::Low;
        if INVERT_LEVEL { is_mark = !is_mark; }

        let dt_signed: i32 = if is_mark { dt_us as i32 } else { -(dt_us as i32) };

        // 存入當前 frame 緩衝
        if n < MAX_FRAME_SEGS {
            segs[n] = dt_signed;
            n += 1;
            edges_in_frame = edges_in_frame.saturating_add(1);
        } else {
            // 緩衝滿了：直接輸出並重置，避免丟資料
            print_frame_array(&segs[..n], 0);
            n = 0;
            edges_in_frame = 0;
        }
    }
}

fn print_frame_array(segs: &[i32], gap_us: u32) {
    use heapless::Vec;

    // 轉換為需要的正負號型態
    let mut out: Vec<i32, {MAX_FRAME_SEGS as usize}> = Vec::new();
    for &v in segs {
        let x = if ARRAY_ALL_POSITIVE { v.unsigned_abs() as i32 } else { v };
        let _ = out.push(x);
    }

    // 單行輸出（defmt 會一次印在同一行）
    info!("FRAME n={=usize} gap={=u32}us  RAW: {:?}", segs.len(), gap_us, defmt::Debug2Format(out.as_slice()));
}
