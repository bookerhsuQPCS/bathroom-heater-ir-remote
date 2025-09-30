#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_rp::adc::{Adc, Config, Channel, Async};
use embassy_rp::bind_interrupts;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

/// T(°C) = 27 - (V_sense - 0.706) / 0.001721
/// 以 3.3V 當 Vref；若實測不同可調整 VREF_MV。
const VREF_MV: u32 = 3300;
const ADC_FULL_SCALE: u32 = 4095; // 12-bit
const DUMMY_READS: usize = 3;     // 丟首樣
const SAMPLES: usize = 15;        // 中位數取樣

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Embassy ADC（Async 模式）
    let mut adc: Adc<Async> = Adc::new(p.ADC, Irqs, Config::default());

    // 內建溫度感測器通道（你的驅動：Channel::new_temp_sensor）
    let mut ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    info!("RP2040 internal temperature (Embassy ADC, temp sensor)");

    loop {
        let raw = read_stable_raw(&mut adc, &mut ts).await;

        // raw -> mV -> °C
        let mv = (raw as u32) * VREF_MV / ADC_FULL_SCALE;
        let v  = mv as f32 / 1000.0;
        let t_c = 27.0 - (v - 0.706) / 0.001721;

        info!("temp: raw={}  ~{} mV  -> {} °C", raw, mv, t_c);

        Timer::after(Duration::from_secs(30)).await;
    }
}

// 丟首樣 + 多筆中位數（顯式標註 Adc 的生命週期）
async fn read_stable_raw(adc: &mut Adc<'_, Async>, ch: &mut Channel<'_>) -> u16 {
    // 丟掉首樣，讓取樣電容穩定
    for _ in 0..DUMMY_READS {
        let _ = adc.read(ch).await;
    }

    // 多筆採樣（忽略偶發錯誤）
    let mut buf = [0u16; SAMPLES];
    let mut i = 0;
    while i < SAMPLES {
        match adc.read(ch).await {
            Ok(v) => { buf[i] = v; i += 1; }
            Err(_) => { /* 重試即可 */ }
        }
    }

    buf.sort_unstable();
    buf[SAMPLES / 2]
}
