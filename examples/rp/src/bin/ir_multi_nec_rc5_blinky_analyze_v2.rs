#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Instant, Timer};
use {embassy_rp as rp, rp::bind_interrupts};

// --- infrared ---
use infrared::cmd::AddressCommand; // for Rc5Command::address()/command()
use infrared::protocol::{nec::Nec, rc5::Rc5};
use infrared::receiver::{MultiReceiver, MultiReceiverCommand, NoPin};

bind_interrupts!(struct Irqs {});

// ===== 調整參數 =====
const IR_PIN_IS_PULLUP: bool = true;       // 大多數 IR 接收頭使用上拉
const INVERT_LEVEL: bool = false;          // 若解不到，改 true 試試（極性反轉）
const MIN_US: u32 = 200;                   // 毛邊下限（200–240µs 視環境）
const ACTIVE_WINDOW_US: u32 = 12_000;      // frame 切割窗（> 協定內最長 space）
const MIN_EDGES_BEFORE_MISS: u32 = 8;      // frame 至少有這麼多邊緣才會記 miss
const LED_ON_MS_OK: u64 = 60;
const LED_OFF_MS_OK: u64 = 120;

// --- raw frame 緩衝（拿來做未知協定分析） ---
const MAX_PULSES: usize = 512; // 每 frame 最多記 512 個脈衝（mark/space 以一段為單位）

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rp::init(Default::default());

    // ★ IR 接收腳位（示範 GP14）
    let mut ir_in = if IR_PIN_IS_PULLUP {
        Input::new(p.PIN_14, Pull::Up)
    } else {
        Input::new(p.PIN_14, Pull::None)
    };

    // ★ 指示 LED（示範 GP22）
    let mut led = Output::new(p.PIN_22, Level::Low);

    // ★ 同時支援 NEC 與 RC5；時間基準 1_000_000 Hz（微秒）
    let mut rx: MultiReceiver<2, (Nec, Rc5), NoPin, u32> =
        MultiReceiver::new(1_000_000, NoPin);

    // 邊緣/電位
    let mut last_edge = Instant::now();
    let mut last_level = ir_in.get_level();

    // frame 狀態
    let mut frame_edges: u32 = 0;
    let mut frame_decoded: bool = false;

    // raw 緩衝
    let mut pulses: [i32; MAX_PULSES] = [0; MAX_PULSES];
    let mut np: usize = 0;

    info!("IR Multi+Analyze start: NEC+RC5, pin=GP14, base=1MHz(us), LED=GP22");

    loop {
        // 等待任一邊緣
        ir_in.wait_for_any_edge().await;

        // dt(us) = 上一段穩定電位維持時間
        let now = Instant::now();
        let mut dt_us64 = (now - last_edge).as_micros();
        if dt_us64 > core::u32::MAX as u64 {
            dt_us64 = core::u32::MAX as u64;
        }
        let dt_us = dt_us64 as u32;

        // 太短毛邊：完全忽略（不更新 last_edge/last_level，不記入 frame）
        if dt_us < MIN_US {
            continue;
        }

        // frame 切割：出現長靜默 → 判定上一 frame 結束
        if dt_us > ACTIVE_WINDOW_US {
            if frame_edges >= MIN_EDGES_BEFORE_MISS && !frame_decoded {
                // 先做一次「寬鬆 NEC 猜測」幫忙定位（不保證成功）
                if let Some(bytes) = try_lenient_nec_guess(&pulses[..np]) {
                    info!("NEC-ish guess: {=[u8]:x} (len={=usize})", bytes.as_slice(), bytes.len());
                } else {
                    // 再嘗試一次：把極性反過來（mark/space 取負號）
                    let mut flipped: [i32; MAX_PULSES] = [0; MAX_PULSES];
                    for k in 0..np { flipped[k] = -pulses[k]; }
                    if let Some(bytes) = try_lenient_nec_guess(&flipped[..np]) {
                        info!("NEC-ish guess (flipped): {=[u8]:x} (len={=usize})", bytes.as_slice(), bytes.len());
                    } else {
                        info!("IR decode miss (frame gap {}us, edges={})", dt_us, frame_edges);
                    }
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

        // 累計 frame 邊緣數
        frame_edges = frame_edges.saturating_add(1);

        // 記 raw 脈衝（正=mark/on，負=space/off，單位 µs）
        // 正號代表 MARK（有載波）；多數接收頭為「低=有載波」，若 INVERT_LEVEL 則相反
        let mut is_mark = (prev_level == Level::Low);
        if INVERT_LEVEL { is_mark = !is_mark; }
        if np < MAX_PULSES {
            pulses[np] = if is_mark { dt_us as i32 } else { -(dt_us as i32) };
            np += 1;
        }

        // 餵多協定解碼器
        let mut decoded_any = false;
        for cmd in rx.event_generic_iter(dt_us, is_low_before) {
            decoded_any = true;
            frame_decoded = true;
            match cmd {
                MultiReceiverCommand::Nec(c) => {
                    info!("[NEC] addr=0x{:02X} cmd=0x{:02X} repeat={}", c.addr, c.cmd, c.repeat);
                    blink(&mut led, 3, LED_ON_MS_OK, LED_OFF_MS_OK).await;
                }
                MultiReceiverCommand::Rc5(c) => {
                    let addr: u8 = (c.address() & 0x1F) as u8;  // 5 bits
                    let cmd:  u8 = (c.command() & 0x7F) as u8;   // 6~7 bits
                    info!("[RC5] addr={=u8} cmd={=u8} toggle={}", addr, cmd, c.toggle);
                    blink(&mut led, 3, LED_ON_MS_OK, LED_OFF_MS_OK).await;
                }
                _ => {}
            }
        }

        // 若成功解碼，清掉 raw（避免重複分析同一包）
        if decoded_any {
            np = 0;
        }
    }
}

// --- 簡易「寬鬆 NEC 猜測」：
// 目標：協助觀察「非標準 NEC」或 NEC-like 的碼，提供一串 bytes 參考。
// 方法：
//  - 尋找領頭：正>6ms + 負>2ms 的前導
//  - 統計「短 space」(300..900us) 與「長 space」(1100..3000us) 的中位數
//  - 用閾值 = (short+long)/2 區分 bit0 與 bit1
//  - 讀取 32 或 48 位（能讀多少讀多少），組成 bytes 輸出
fn try_lenient_nec_guess(pulses: &[i32]) -> Option<heapless::Vec<u8, 12>> {
    use heapless::Vec;

    if pulses.len() < 10 { return None; }

    // 找前導（leader）：正>6ms、負>2ms
    let mut i = 0usize;
    while i + 1 < pulses.len() {
        let mark = pulses[i];
        let space = pulses[i + 1];
        if mark.abs() as u32 > 5000 && space < 0 && (-space as u32) > 1500 {
            i += 2;
            break;
        }
        i += 1;
    }
    if i + 4 >= pulses.len() { return None; }

    // 掃描一段，找短/長 space 的代表值
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

    // 依據 space 長度判 0/1，讀最多 64 位（8 bytes）
    let mut bits: u64 = 0;
    let mut nbits = 0u8;
    j = i;
    while j + 1 < pulses.len() && nbits < 64 {
        let _m = pulses[j];
        let s = if j + 1 < pulses.len() { pulses[j + 1] } else { 0 };
        if s >= 0 { break; } // 不合預期
        let sp = (-s) as u32;
        let bit = if sp > thresh { 1u64 } else { 0u64 };
        bits |= bit << nbits;
        nbits += 1;
        j += 2;
    }
    if nbits < 16 { return None; }

    // 打包成 bytes（低位在前），最多 12 bytes
    let mut out: Vec<u8, 12> = Vec::new();
    let nbytes = core::cmp::min(12, (nbits as usize + 7) / 8);
    for k in 0..nbytes {
        out.push(((bits >> (k * 8)) & 0xFF) as u8).ok()?;
    }
    Some(out)
}

// 非阻塞閃燈：times 次，每次 on_ms / off_ms
async fn blink(led: &mut Output<'static>, times: u8, on_ms: u64, off_ms: u64) {
    for _ in 0..times {
        led.set_high();
        Timer::after_millis(on_ms).await;
        led.set_low();
        Timer::after_millis(off_ms).await;
    }
}
