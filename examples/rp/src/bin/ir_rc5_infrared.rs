// src/bin/ir_rc5_infrared.rs
#![no_std]
#![no_main]

use defmt::*;
use defmt::Debug2Format;                // ★ 重要：讓非 defmt::Format 的型別可用 {} 輸出（走 core::fmt::Debug）
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Instant, Timer};
use {embassy_rp as rp, rp::bind_interrupts};

// --- infrared ---
use infrared::cmd::AddressCommand;      // Rc5Command 的 address()/command()
use infrared::protocol::rc5::Rc5;
use infrared::receiver::{NoPin, Receiver}; // ★ 單協定用 Receiver，不是 MultiReceiver

bind_interrupts!(struct Irqs {});

// ===== 可調參數 =====
// 對齊你 NEC 版：IN=GP14，LED=GP22
const IR_PIN_IS_PULLUP: bool = true;    // 多數 TSOP 需上拉
const INVERT_LEVEL: bool = false;       // 若解不到，改 true 測試極性
const MIN_US: u32 = 150;                // 抗毛邊下限（RC5 半位元 ~889us）
const FAIL_GAP_US: u32 = 25_000;        // 活動期間 25ms 都無成功解碼 → 記一次 FAIL
const ACTIVE_WINDOW_US: u32 = 8_000;    // 只有 8ms 內確有邊緣活動才允許觸發 FAIL
const LED_ON_MS_OK: u64 = 60;
const LED_OFF_MS_OK: u64 = 120;
const LED_ON_MS_FAIL: u64 = 80;
const LED_OFF_MS_FAIL: u64 = 160;
const DEBUG_EDGES: bool = false;        // 診斷時改 true 看 dt 與電平節奏
// ===================

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rp::init(Default::default());

    // ★ IR 輸入腳位：GP14
    let mut ir_in = if IR_PIN_IS_PULLUP {
        Input::new(p.PIN_14, Pull::Up)
    } else {
        Input::new(p.PIN_14, Pull::None)
    };

    // ★ 指示 LED：GPIO22
    let mut led = Output::new(p.PIN_22, Level::Low);

    // ★ RC5 單協定事件式解碼（時間基準 1MHz -> 微秒）
    let mut rx: Receiver<Rc5, NoPin, u32> = Receiver::new(1_000_000);

    // 時戳：邊緣、最近一次成功解碼、最近一次有效活動
    let mut last_edge = Instant::now();
    let mut last_decode = last_edge;
    let mut last_activity = last_edge;

    info!(
        "IR (RC5) ready: IN=GP14, LED=GP22. ACTIVE_LOW={}",
        !INVERT_LEVEL
    );

    let mut edge_count: u32 = 0;

    loop {
        // 等任一邊緣
        ir_in.wait_for_any_edge().await;

        // 計算與上次邊緣的間隔（us）
        let now = Instant::now();
        let mut dt_us = (now - last_edge).as_micros();
        if dt_us > core::u32::MAX as u64 {
            dt_us = core::u32::MAX as u64;
        }
        let dt_us = dt_us as u32;
        last_edge = now;

        // 去毛邊
        if dt_us < MIN_US {
            continue;
        }
        last_activity = now;

        // Receiver::event_edge 需要目前「高電位？」布林
        let mut level_high = ir_in.get_level() == Level::High;
        if INVERT_LEVEL {
            level_high = !level_high;
        }

        edge_count += 1;
        if DEBUG_EDGES {
            info!("edge #{} dt={}us level_high={}", edge_count, dt_us, level_high);
        }

        // 餵事件（回傳 Result<Option<Cmd>>）
        match rx.event_edge(dt_us, level_high) {
            Ok(Some(c)) => {
                // 成功解碼：印 addr/cmd/toggle，閃 3 下
                let addr: u8 = c.address() as u8;
                let code: u8 = c.command() as u8;
                let tog = c.toggle;
                info!("[RC5 OK] addr={=u8} cmd={=u8} toggle={}", addr, code, tog);
                blink(&mut led, 3, LED_ON_MS_OK, LED_OFF_MS_OK).await;
                last_decode = now;
            }
            Ok(None) => {
                // 尚未湊齊，不動
            }
            Err(e) => {
                // ★ 修正點：用 Debug2Format 包裝，才能用 defmt 輸出
                warn!("[RC5 ERR] {}", Debug2Format(&e));
            }
        }

        // 失敗判定：最近有活動，但距上次成功解碼已超過 FAIL_GAP
        let active_recently = (now - last_activity).as_micros() as u64 <= ACTIVE_WINDOW_US as u64;
        let gap_us = (now - last_decode).as_micros() as u64;
        if active_recently && gap_us >= FAIL_GAP_US as u64 {
            warn!("[RC5 FAIL] gap {}us with recent activity", gap_us);
            blink(&mut led, 1, LED_ON_MS_FAIL, LED_OFF_MS_FAIL).await;
            last_decode = now; // 冷卻，避免馬上又觸發
        }
    }
}

async fn blink(led: &mut Output<'static>, times: u8, on_ms: u64, off_ms: u64) {
    for _ in 0..times {
        led.set_high();
        Timer::after_millis(on_ms).await;
        led.set_low();
        Timer::after_millis(off_ms).await;
    }
}
