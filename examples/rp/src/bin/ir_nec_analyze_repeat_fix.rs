#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Instant, Timer};
use {embassy_rp as rp, rp::bind_interrupts};

use infrared::protocol::nec::Nec;
use infrared::receiver::{NoPin, Receiver};

bind_interrupts!(struct Irqs {});

// ===== 參數 =====
const IR_PIN_IS_PULLUP: bool = true;
const INVERT_LEVEL: bool = false;       // 解不到就翻轉看看
const MIN_US: u32 = 200;                // 180~240 視雜訊調
const ACTIVE_WINDOW_US: u32 = 15_000;   // 12~15ms
const MIN_EDGES_BEFORE_MISS: u32 = 8;   // 少於這個就不算一整包
const REPEAT_WINDOW_MS: u64 = 150;      // repeat 對回上一幀的時間窗
const LED_ON_MS_OK: u64 = 60;
const LED_OFF_MS_OK: u64 = 120;

const MAX_PULSES: usize = 512;          // raw frame 緩衝大小

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rp::init(Default::default());

    let mut ir_in = if IR_PIN_IS_PULLUP {
        Input::new(p.PIN_14, Pull::Up)
    } else {
        Input::new(p.PIN_14, Pull::None)
    };
    let mut led = Output::new(p.PIN_22, Level::Low);

    // 單協定 NEC（最穩路徑）
    let mut rx: Receiver<Nec, NoPin> = Receiver::new(1_000_000);

    // 邊緣/電位
    let mut last_edge = Instant::now();
    let mut last_level = ir_in.get_level();

    // frame 狀態
    let mut frame_edges: u32 = 0;
    let mut frame_decoded: bool = false;

    // raw 緩衝
    let mut pulses: [i32; MAX_PULSES] = [0; MAX_PULSES];
    let mut np: usize = 0;

    // 記住上一個完整 NEC 幀（非 repeat），把之後的 repeat 對回來
    let mut last_nec_full: Option<(u8, u8, Instant)> = None;

    info!("IR NEC analyze (solo) start: pin=GP14, 1MHz(us), LED=GP22");

    loop {
        ir_in.wait_for_any_edge().await;

        let now = Instant::now();
        let mut dt_us64 = (now - last_edge).as_micros();
        if dt_us64 > core::u32::MAX as u64 {
            dt_us64 = core::u32::MAX as u64;
        }
        let dt_us = dt_us64 as u32;

        // 過短毛邊直接忽略，不更新任何狀態
        if dt_us < MIN_US {
            continue;
        }

        // frame 切割：長靜默 → 結束上一 frame
        if dt_us > ACTIVE_WINDOW_US {
            // debug: frame boundary reached
            if frame_edges >= MIN_EDGES_BEFORE_MISS && !frame_decoded {
                if let Some(bytes) = try_lenient_nec_guess(&pulses[..np]) {
                    let (a,b,c,d) = (bytes.get(0).copied().unwrap_or(0),
                                     bytes.get(1).copied().unwrap_or(0),
                                     bytes.get(2).copied().unwrap_or(0),
                                     bytes.get(3).copied().unwrap_or(0));
                    info!("NEC-ish guess: {=[u8]:x} (len={=usize})  addr=0x{=u8:x} ~addr=0x{=u8:x} cmd=0x{=u8:x} ~cmd=0x{=u8:x}",
                          bytes.as_slice(), bytes.len(), a,b,c,d);
                } else {
                    info!("IR decode miss (frame gap {}us, edges={})", dt_us, frame_edges);
                }
            }
            // 重置 frame 狀態
            frame_edges = 0;
            frame_decoded = false;
            np = 0;
        }

        // 這段 dt 的電位（= 邊緣前）
        let prev_level = last_level;
        last_level = ir_in.get_level();
        last_edge = now;

        let mut is_low_before = prev_level == Level::Low;
        if INVERT_LEVEL { is_low_before = !is_low_before; }

        // 記 raw 脈衝（正=MARK/有載波，負=SPACE/無載波）；多數接收頭低=有載波
        let mut is_mark = prev_level == Level::Low;
        if INVERT_LEVEL { is_mark = !is_mark; }
        if np < MAX_PULSES {
            pulses[np] = if is_mark { dt_us as i32 } else { -(dt_us as i32) };
            np += 1;
            frame_edges = frame_edges.saturating_add(1);
        }

        // 餵單協定 NEC
        if let Ok(Some(c)) = rx.event(dt_us, is_low_before) {
            frame_decoded = true;
            if c.repeat {
                if let Some((a,k,t)) = last_nec_full {
                    if (now - t).as_millis() as u64 <= REPEAT_WINDOW_MS {
                        info!("[NEC] repeat of addr=0x{:02X} cmd=0x{:02X}", a, k);
                    } else {
                        info!("[NEC] repeat (no recent base)");
                    }
                } else {
                    info!("[NEC] repeat (no base)");
                }
            } else {
                info!("[NEC] addr=0x{:02X} cmd=0x{:02X} repeat=false", c.addr, c.cmd);
                last_nec_full = Some((c.addr, c.cmd, now));
            }
            blink(&mut led, 2, LED_ON_MS_OK, LED_OFF_MS_OK).await;
            np = 0; // 成功就清 raw
        }
    }
}

// --- NEC-ish 寬鬆猜測（放寬門檻版） ---
fn try_lenient_nec_guess(pulses: &[i32]) -> Option<heapless::Vec<u8, 12>> {
    use heapless::Vec;

    if pulses.len() < 10 { return None; }

    // 找 leader（放寬版）：mark > 5ms + space > 1.5ms
    let mut i = 0usize;
    while i + 1 < pulses.len() {
        let mark = pulses[i];
        let space = pulses[i + 1];
        if (mark.abs() as u32) > 5000 && space < 0 && (-space as u32) > 1500 {
            i += 2;
            break;
        }
        i += 1;
    }
    if i + 4 >= pulses.len() { return None; }

    // 收集短/長 space（放寬版）
    let mut short_spaces: [u16; 24] = [0; 24];
    let mut long_spaces:  [u16; 24] = [0; 24];
    let mut ns = 0usize;
    let mut nl = 0usize;

    let mut j = i;
    while j + 1 < pulses.len() && (ns < short_spaces.len() || nl < long_spaces.len()) {
        let _mark = pulses[j];
        let space = if j + 1 < pulses.len() { pulses[j + 1] } else { 0 };
        if space < 0 {
            let s = (-space) as u32;
            if (250..=1000).contains(&s) && ns < short_spaces.len() {
                short_spaces[ns] = s as u16; ns += 1;
            } else if (900..=4000).contains(&s) && nl < long_spaces.len() {
                long_spaces[nl] = s as u16; nl += 1;
            }
        }
        j += 2;
    }
    if ns == 0 || nl == 0 { return None; }
    short_spaces[..ns].sort_unstable();
    long_spaces[..nl].sort_unstable();
    let short = short_spaces[ns/2] as u32;
    let long  = long_spaces[nl/2] as u32;
    let thresh = (short + long) / 2;

    // 依照 space 長短判 0/1，最多 64 位（8 bytes）
    let mut bits: u64 = 0;
    let mut nbits = 0u8;
    j = i;
    while j + 1 < pulses.len() && nbits < 64 {
        let _m = pulses[j];
        let s = if j + 1 < pulses.len() { pulses[j + 1] } else { 0 };
        if s >= 0 { break; }
        let sp = (-s) as u32;
        let bit = if sp > thresh { 1u64 } else { 0u64 };
        bits |= bit << nbits;
        nbits += 1;
        j += 2;
    }
    if nbits < 16 { return None; }

    // 打包成 bytes（低位先出），最多 12 bytes
    let mut out: Vec<u8, 12> = Vec::new();
    let nbytes = core::cmp::min(12, (nbits as usize + 7) / 8);
    for k in 0..nbytes {
        out.push(((bits >> (k * 8)) & 0xFF) as u8).ok()?;
    }
    Some(out)
}

// 非阻塞閃燈
async fn blink(led: &mut Output<'static>, times: u8, on_ms: u64, off_ms: u64) {
    for _ in 0..times {
        led.set_high();
        Timer::after_millis(on_ms).await;
        led.set_low();
        Timer::after_millis(off_ms).await;
    }
}
