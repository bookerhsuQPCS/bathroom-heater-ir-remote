// src/bin/pio_ws2812.rs
//! RP2040 PIO 驅動 WS2812（Neopixel）。
//! 本版固定用 GP23 當資料腳（適用你那塊把 WS2812 拉到 GP23 的第三方板）。
//!
//! 硬體提醒：WS2812 供電建議 5V（VBUS），GND 一定要與板子共地；DIN 與 GP23 之間串 330Ω。
//! 建議在 5V 與 GND 之間並 ≥470µF 電容，避免上電突波。

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_time::{Duration, Ticker};
use smart_leds::RGB8;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// 改成你實際串的顆數
const NUM_LEDS: usize = 1;

/// 產生彩虹色（GRB/RGB 都可用的常見輪盤）
fn wheel(mut pos: u8) -> RGB8 {
    pos = 255 - pos;
    if pos < 85 {
        // R->G
        RGB8 {
            r: 255 - pos * 3,
            g: pos * 3,
            b: 0,
        }
    } else if pos < 170 {
        // G->B
        pos -= 85;
        RGB8 {
            r: 0,
            g: 255 - pos * 3,
            b: pos * 3,
        }
    } else {
        // B->R
        pos -= 170;
        RGB8 {
            r: pos * 3,
            g: 0,
            b: 255 - pos * 3,
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("WS2812 via PIO on GP23");

    // RP2040 初始化 + PIO0 取用
    let p = embassy_rp::init(Default::default());
    let mut pio = Pio::new(p.PIO0, Irqs);

    // 載入 WS2812 PIO 程式並建立驅動（資料腳：GP23）
    let program = PioWs2812Program::new(&mut pio.common);
    let mut ws2812 = PioWs2812::new(&mut pio.common, pio.sm0, p.DMA_CH0, p.PIN_23, &program);

    // LED 緩衝
    let mut data = [RGB8::default(); NUM_LEDS];

    // 10ms 更新一次，做彩虹掃描
    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        for j in 0..(256 * 5) {
            for i in 0..NUM_LEDS {
                // 基於索引與時間做位移，形成流動彩虹
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
            }
            ws2812.write(&data).await; // 送出到燈條
            ticker.next().await;
        }
    }
}
