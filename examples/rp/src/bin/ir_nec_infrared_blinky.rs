//! src/bin/ir_nec_infrared_blinky.rs
//! Pico W / RP2040 + Embassy：NEC 紅外線接收 + RAW 記錄 + 嘗試解碼 + LED 閃爍
//! IN=GP14 (PullUp), LED=GP22
//! 可調：ACTIVE_LOW / MIN_US / FAIL_GAP_US / ACTIVE_WINDOW_US / BLINK_* 參數

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

// ===== 參數 =====
const ACTIVE_LOW: bool = true;         // #invert-level：低電平=MARK(有載波) → true
const MIN_US: u32 = 150;               // #min-us：丟毛邊門檻（建議 120~200）
const FAIL_GAP_US: u32 = 40_000;       // #fail-gap-us：超過此靜默仍未完成 → FAIL
const ACTIVE_WINDOW_US: u32 = 8_000;   // #active-window-us：一幀內的活動視窗

const BLINK_OK: u32 = 3;               // #blink-ok
const BLINK_FAIL: u32 = 1;             // #blink-fail
const BLINK_ON_MS: u64 = 60;           // #blink-timing on
const BLINK_OFF_MS: u64 = 80;          // #blink-timing off

// ===== RAW 緩衝 =====
//
// 注意：每個 Edge 的 dt_us 是「上一段穩定電平維持多久」；level_before 是那段的電平。
#[derive(Copy, Clone)]
struct Edge {
    dt_us: u32,
    level_before: bool, // true=High, false=Low
}
const RAW_CAP: usize = 128;

// ===== 公用：LED 閃爍 =====
async fn blink(led: &mut Output<'_>, times: u32) {
    for _ in 0..times {
        led.set_low();
        Timer::after(Duration::from_millis(BLINK_ON_MS)).await;
        led.set_high();
        Timer::after(Duration::from_millis(BLINK_OFF_MS)).await;
    }
}

// ===== RAW 輸出（人眼檢查）=====
// active_low：Low=MARK, High=SPACE；否則相反。
fn dump_edges(tag: &str, buf: &[Edge], active_low: bool) {
    info!("[NEC RAW {}] n={}, active_low={}", tag, buf.len(), active_low);
    for e in buf {
        let is_mark = if active_low { !e.level_before } else { e.level_before };
        info!("  + {}us {}", e.dt_us, if is_mark { "MARK" } else { "SPACE" });
    }
}

// ===== NEC 嘗試解碼 =====
//
// 將 RAW 轉為 pulse 序列後，檢查：
//   leader ≈ 9ms MARK + 4.5ms SPACE
//   bit0 ≈ 560 MARK + 560 SPACE
//   bit1 ≈ 560 MARK + 1680 SPACE
// repeat：9ms MARK + 2.25ms SPACE + 560 MARK（*本例：只印 REPEAT，不改 LED*）
struct NecFrame {
    addr: u8,
    addr_inv: u8,
    cmd: u8,
    cmd_inv: u8,
}

fn within(x: u32, target: u32, tol: u32) -> bool {
    let lo = target.saturating_sub(tol);
    let hi = target + tol;
    x >= lo && x <= hi
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum PulseType {
    Mark,
    Space,
}

#[derive(Copy, Clone)]
struct Pulse {
    kind: PulseType,
    us: u32,
}

// 將 Edge[] 轉為 Pulse[]：Edge 表示「上一段電平持續 dt_us」，電平→MARK/SPACE 取決於 ACTIVE_LOW
fn edges_to_pulses(edges: &[Edge], active_low: bool) -> heapless::Vec<Pulse, RAW_CAP> {
    let mut out: heapless::Vec<Pulse, RAW_CAP> = heapless::Vec::new();
    for e in edges {
        if e.dt_us < MIN_US {
            continue; // 丟毛邊
        }
        let is_mark = if active_low { !e.level_before } else { e.level_before };
        let kind = if is_mark { PulseType::Mark } else { PulseType::Space };
        // 盡量合併相鄰同類脈衝（理論上不會，但保守處理）
        if let Some(last) = out.last_mut() {
            if last.kind == kind {
                last.us = last.us.saturating_add(e.dt_us);
                continue;
            }
        }
        let _ = out.push(Pulse { kind, us: e.dt_us });
    }
    out
}

fn try_decode_nec(edges: &[Edge], active_low: bool) -> Result<NecFrame, &'static str> {
    let pulses = edges_to_pulses(edges, active_low);
    if pulses.len() < 4 {
        return Err("too_few_pulses");
    }

    // NEC repeat：9ms MARK + 2.25ms SPACE + 560 MARK
    // 若符合 repeat，直接回報（不當作完整 32bit 幀）
    if pulses.len() >= 3
        && pulses[0].kind == PulseType::Mark
        && pulses[1].kind == PulseType::Space
        && pulses[2].kind == PulseType::Mark
        && within(pulses[0].us, 9000, 1200)
        && within(pulses[1].us, 2250, 700)
        && within(pulses[2].us, 560, 300)
    {
        info!("[NEC REPEAT]");
        return Err("repeat");
    }

    // Leader：~9ms MARK + ~4.5ms SPACE
    if !(pulses[0].kind == PulseType::Mark
        && pulses[1].kind == PulseType::Space
        && within(pulses[0].us, 9000, 1500)
        && within(pulses[1].us, 4500, 1200))
    {
        return Err("no_leader");
    }

    // 從 pulses[2] 開始，每個 bit 預期是：MARK(≈560) + SPACE(≈560/1680)
    // 收 32 bit
    let mut bits: u32 = 0;
    let mut p = 2usize;
    for i in 0..32 {
        // 需要兩個 pulse（MARK + SPACE）
        if p + 1 >= pulses.len() {
            return Err("truncated_bits");
        }
        let pm = pulses[p];
        let ps = pulses[p + 1];
        if pm.kind != PulseType::Mark || !within(pm.us, 560, 350) {
            return Err("bad_mark");
        }
        let bit_is_one = if within(ps.us, 560, 350) {
            false
        } else if within(ps.us, 1680, 600) {
            true
        } else {
            return Err("bad_space");
        };
        if bit_is_one {
            bits |= 1 << i; // LSB-first
        }
        p += 2;
    }

    let addr = (bits & 0xFF) as u8;
    let addr_inv = ((bits >> 8) & 0xFF) as u8;
    let cmd = ((bits >> 16) & 0xFF) as u8;
    let cmd_inv = ((bits >> 24) & 0xFF) as u8;

    // 基本健全性檢查（互反）
    if addr ^ addr_inv != 0xFF || cmd ^ cmd_inv != 0xFF {
        return Err("xor_mismatch");
    }

    Ok(NecFrame {
        addr,
        addr_inv,
        cmd,
        cmd_inv,
    })
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // LED：高電平熄、低電平亮（常見接法，視你的板子而定）
    let mut led = Output::new(p.PIN_22, Level::High);

    // IN：GP14，內建上拉
    let ir_in = Input::new(p.PIN_14, Pull::Up);

    info!(
        "IR (NEC) ready: IN=GP14, LED=GP22. ACTIVE_LOW={}",
        ACTIVE_LOW
    );

    // ===== 邊緣收集所需狀態 =====
    let mut last_level = ir_in.get_level(); // Level::High/Low
    let mut last_ts = Instant::now();
    let mut last_edge_ts = last_ts;

    let mut raw: [Edge; RAW_CAP] = [Edge {
        dt_us: 0,
        level_before: true,
    }; RAW_CAP];
    let mut raw_len: usize = 0;

    loop {
        // 簡單 polling（你可改用中斷/await 版本）
        let now = Instant::now();
        let lvl = ir_in.get_level();

        if lvl != last_level {
            let dt = now.duration_since(last_ts).as_micros() as u32;
            last_ts = now;
            last_edge_ts = now;

            if dt >= MIN_US {
                if raw_len < RAW_CAP {
                    // 此段的「前一電平」是 last_level
                    raw[raw_len] = Edge {
                        dt_us: dt,
                        level_before: last_level == Level::High,
                    };
                    raw_len += 1;
                }
            }
            last_level = lvl;
        }

        // 觀測到的最後一個邊緣距今的 gap
        let gap_us = now.duration_since(last_edge_ts).as_micros() as u32;

        // === 一幀結束：嘗試解碼 ===
        if raw_len > 0 && gap_us > ACTIVE_WINDOW_US {
            match try_decode_nec(&raw[..raw_len], ACTIVE_LOW) {
                Ok(f) => {
                    info!(
                        "[NEC OK] addr=0x{:02X} (~0x{:02X}), cmd=0x{:02X} (~0x{:02X}), edges={}",
                        f.addr, f.addr_inv, f.cmd, f.cmd_inv, raw_len
                    );
                    // 也 dump 一份 RAW 方便比對
                    dump_edges("OK", &raw[..raw_len], ACTIVE_LOW);
                    blink(&mut led, BLINK_OK).await;
                }
                Err(e) => {
                    warn!("[NEC FAIL] reason={}, edges={}", e, raw_len);
                    dump_edges("FAIL", &raw[..raw_len], ACTIVE_LOW);
                    blink(&mut led, BLINK_FAIL).await;
                }
            }
            raw_len = 0;
        }

        // === 長時間無後續：清掉殘留並視為 FAIL ===
        if gap_us > FAIL_GAP_US {
            if raw_len > 0 {
                warn!(
                    "[NEC FAIL] gap {}us with recent activity (edges={})",
                    gap_us, raw_len
                );
                dump_edges("FAIL", &raw[..raw_len], ACTIVE_LOW);
                blink(&mut led, BLINK_FAIL).await;
                raw_len = 0;
            }
        }

        // 節流
        Timer::after(Duration::from_millis(1)).await;
    }
}
