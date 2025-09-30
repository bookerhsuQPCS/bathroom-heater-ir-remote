#![no_std]
#![no_main]

// 連上 defmt-rtt 與 panic handler
use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Instant, Timer};

use infrared::{protocol::nec::Nec, Receiver};

/// 市售接收頭（TSOP/VS1838B 類）通常為 active-low：有載波→輸出為 Low。
const SENSOR_ACTIVE_LOW: bool = true;

/// 一包資料的閒置超時（毫秒）
const FRAME_IDLE_TIMEOUT_MS: u64 = 200;

/// 事件解析度（Hz）。我們用「微秒」→ 1_000_000。
const RESOLUTION_HZ: u32 = 1_000_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // 外接 LED：GP22（注意：這不是 CYW43 的板載 LED）
    let mut led = Output::new(p.PIN_22, Level::High);

    // 開機快閃 3 次，確認韌體有跑
    for _ in 0..3 {
        led.set_low();
        Timer::after_millis(120).await;
        led.set_high();
        Timer::after_millis(120).await;
    }

    //LED to off
    led.set_low();

    // IR 輸入腳：GP14。常見接收頭 idle=High → Pull::Up
    let mut ir_in = Input::new(p.PIN_14, Pull::Up);

    // NEC 事件式接收器
    let mut rx: Receiver<Nec> = Receiver::new(RESOLUTION_HZ);

    info!("IR (NEC) ready: IN=GP14, LED=GP22. ACTIVE_LOW={}", SENSOR_ACTIVE_LOW);

    let idle_timeout = Duration::from_millis(FRAME_IDLE_TIMEOUT_MS);

    loop {
        // 先等到第一個邊緣
        ir_in.wait_for_any_edge().await;

        let mut last = Instant::now();
        let mut edges_in_frame: u32 = 0;

        loop {
            // 等下一個邊緣或超時
            let res = embassy_time::with_timeout(idle_timeout, ir_in.wait_for_any_edge()).await;

            let now = Instant::now();
            let dt_us: u32 = (now - last).as_micros() as u32; // 與 RESOLUTION_HZ 對齊（微秒）
            last = now;

            // 邊緣「之後」的腳位邏輯電平
            let is_high = ir_in.is_high();

            // ⚠️ 關鍵：傳「剛剛結束的那一段是否為 mark（有載波）」給 infrared
            // active-low：若邊緣後是 High → 剛結束的是 Low（mark=true）
            // active-high：若邊緣後是 Low  → 剛結束的是 High（mark=true）
            let just_finished_is_mark = if SENSOR_ACTIVE_LOW { is_high } else { !is_high };

            edges_in_frame += 1;
            debug!(
                "edge #{}, dt={}us, level_high={}, mark(prev)={}",
                edges_in_frame, dt_us, is_high, just_finished_is_mark
            );

            // 丟進 infrared 解碼（事件式）：(Δt, 剛結束是否為 mark)
            if let Ok(Some(cmd)) = rx.event(dt_us, just_finished_is_mark) {
                info!("NEC decoded: addr=0x{:02X} cmd=0x{:02X}", cmd.addr, cmd.cmd);
                // 成功解到就閃一下
                led.set_low();
                Timer::after_millis(60).await;
                led.set_high();
            }

            if res.is_err() {
                // 超時 → 視為一包結束；重置解碼器狀態以免殘留
                info!("frame end, edges={}", edges_in_frame);
                rx = Receiver::new(RESOLUTION_HZ);
                break;
            }
        }
    }
}
