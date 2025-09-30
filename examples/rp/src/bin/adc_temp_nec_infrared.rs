#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer, with_timeout};

use embassy_rp::bind_interrupts;
use embassy_rp::adc::{Adc, Async, Channel, Config};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::rtc::{Rtc, DateTime, DayOfWeek};

use infrared::{protocol::nec::Nec, Receiver};

// ===== 綁 ADC IRQ（Embassy ADC 需要）
bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

// ===== 內建溫度換算常數 =====
const VREF_MV: u32 = 3300;
const ADC_FULL_SCALE: u32 = 4095;
const DUMMY_READS: usize = 3;
const SAMPLES: usize = 15;

// ===== IR 參數 =====
const SENSOR_ACTIVE_LOW: bool = true; // 常見 TSOP/VS1838B：有載波→LOW
const FRAME_IDLE_TIMEOUT_MS: u64 = 200;
const IR_RES_HZ: u32 = 1_000_000; // 以微秒為刻度（1 MHz）
const POLL_MS: u64 = 10;          // 輕量輪詢間隔（同時服務 RTC 與 IR）
const TEMP_PERIOD_SECS: u8 = 30;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // ---- ADC（內建溫度）----
    let mut adc: Adc<Async> = Adc::new(p.ADC, Irqs, Config::default());
    let mut ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // ---- RTC（用作每秒節拍器：以「秒位變化」判斷）----
    let mut rtc = Rtc::new(p.RTC);
    let _ = rtc.set_datetime(DateTime {
        year: 2025, month: 8, day: 28, day_of_week: DayOfWeek::Thursday,
        hour: 0, minute: 0, second: 0,
    });
    let mut last_sec: u8 = rtc.now().ok().map(|n| n.second).unwrap_or(0);

    // ---- IR 硬體 ----
    let mut led = Output::new(p.PIN_22, Level::Low);
    let mut ir_in = Input::new(p.PIN_14, Pull::Up);
    let mut rx: Receiver<Nec> = Receiver::new(IR_RES_HZ);

    // 開機快閃確認韌體跑起來
    for _ in 0..3 {
        led.set_high(); Timer::after_millis(100).await;
        led.set_low();  Timer::after_millis(100).await;
    }

    info!("Start: RTC tick + internal temp, and NEC IR on GP14 (ACTIVE_LOW={}).", SENSOR_ACTIVE_LOW);

    // IR frame 狀態
    let mut edges: u32 = 0;
    let mut last_edge = Instant::now();
    let idle_timeout = Duration::from_millis(FRAME_IDLE_TIMEOUT_MS);

    loop {
        // 以 10ms 輪詢 IR 邊緣（有邊緣就回傳，否則 10ms 超時）
        let edge_result = with_timeout(Duration::from_millis(POLL_MS), ir_in.wait_for_any_edge()).await;

        // ===== 每秒節拍：只要秒位改變就量一次內建溫度 =====
        if let Ok(now) = rtc.now() {
            if now.second != last_sec {
                last_sec = now.second;

                if now.second % TEMP_PERIOD_SECS == 0 {
                    let raw = read_stable_raw(&mut adc, &mut ts).await;
                    let mv = (raw as u32) * VREF_MV / ADC_FULL_SCALE;
                    let v  = mv as f32 / 1000.0;
                    let t_c = 27.0 - (v - 0.706) / 0.001721;

                    info!(
                        "{:04}-{:02}-{:02} {:02}:{:02}:{:02} | temp: raw={}  ~{} mV  -> {} °C",
                        now.year, now.month, now.day, now.hour, now.minute, now.second,
                        raw, mv, t_c
                    );
                }
            }
        }

        // ===== IR 解碼：有邊緣就餵事件，沒邊緣就檢查 frame 超時 =====
        match edge_result {
            Ok(()) => {
                let nowi = Instant::now();
                let dt_us: u32 = (nowi - last_edge).as_micros() as u32;
                last_edge = nowi;

                let is_high = ir_in.is_high();
                // 傳「剛剛結束的那段是否為 mark（有載波）」：
                // active-low：邊緣後為 High → 剛結束 Low 段（mark=true）
                // active-high：邊緣後為 Low  → 剛結束 High 段（mark=true）
                let just_finished_is_mark = if SENSOR_ACTIVE_LOW { is_high } else { !is_high };

                edges += 1;
                // debug!("edge #{}, dt={}us, level_high={}, mark(prev)={}", edges, dt_us, is_high, just_finished_is_mark);

                if let Ok(Some(cmd)) = rx.event(dt_us, just_finished_is_mark) {
                    info!("NEC: addr=0x{:02X} cmd=0x{:02X}", cmd.addr, cmd.cmd);
                    led.set_high(); Timer::after_millis(60).await; led.set_low();
                }
            }
            Err(_) => {
                // 10ms 內沒有新邊緣；若距上次邊緣已超過 frame idle，視為一包結束
                if edges > 0 && (Instant::now() - last_edge) >= idle_timeout {
                    info!("IR frame end, edges={}", edges);
                    rx = Receiver::new(IR_RES_HZ);
                    edges = 0;
                }
                // 沒邊緣也沒超時 → 繼續下一輪
            }
        }
    }
}

// 丟首樣 + 多筆中位數（顯式標註 Adc 的生命週期）
async fn read_stable_raw(adc: &mut Adc<'_, Async>, ch: &mut Channel<'_>) -> u16 {
    for _ in 0..DUMMY_READS {
        let _ = adc.read(ch).await;
    }
    let mut buf = [0u16; SAMPLES];
    let mut i = 0;
    while i < SAMPLES {
        if let Ok(v) = adc.read(ch).await {
            buf[i] = v;
            i += 1;
        }
    }
    buf.sort_unstable();
    buf[SAMPLES / 2]
}
