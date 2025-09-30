//! pio_ws2812_brightness.rs — Embassy RP2040 + WS2812，加入「全域亮度縮放」
//! 只改程式檔，不動 Cargo.toml，不新增套件；維持 embassy 風格。

#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

use smart_leds::RGB8;

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0; // 只需 PIO0 做為 InterruptHandler 泛型
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};

// ---- 設定 ----
const NUM_LEDS: usize = 1;          // 你的 LED 顆數
const FRAME_MS: u64 = 10;            // 幀間隔（動畫速度）
const BRIGHTNESS_DEFAULT: u8 = 30;  // 0..=255；0 全暗、255 不變

// 綁定 PIO 中斷（注意要加上泛型 <PIO0>）
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("WS2812 brightness demo (embassy-rp)");

    // 初始化 SoC 外設（embassy-rp 提供）
    let p = embassy_rp::init(Default::default());

    // 取得 PIO 包裝，拿 state machine 0；新版 API 兩個參數：PIO 周邊 + 中斷
    let embassy_rp::pio::Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);

    // 準備 WS2812 的 PIO 程式
    let program = PioWs2812Program::new(&mut common);

    // 建立 WS2812 驅動；此 API 會因版本有小差異（此為常見簽名）
    // 需要：&mut common、state machine、DMA channel、輸出 PIN、&program
    let mut ws = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_13, &program);

    // 畫面緩衝
    let mut frame: [RGB8; NUM_LEDS] = [RGB8 { r: 0, g: 0, b: 0 }; NUM_LEDS];
    let mut scaled: [RGB8; NUM_LEDS] = [RGB8 { r: 0, g: 0, b: 0 }; NUM_LEDS];

    let mut t = Ticker::every(Duration::from_millis(FRAME_MS));

    // 主迴圈：彩虹 + 全域亮度縮放
    loop {
        for j in 0..(256 * 5) {
            // 產生彩虹
            for i in 0..NUM_LEDS {
                let idx = (((i as u32 * 256) / NUM_LEDS as u32) + j as u32) & 255;
                frame[i] = wheel(idx as u8);
            }
            // 全域亮度縮放（線性）
            for i in 0..NUM_LEDS {
                scaled[i] = scale_pixel_linear(frame[i], BRIGHTNESS_DEFAULT);
            }
            // 送出（不同版本 write 回傳型別不同；忽略回傳值可跨版本）
            let _ = ws.write(&scaled).await;

            t.next().await;
        }
    }
}

/// 線性縮放單一像素（0..=255），避免 overflow 用 u16 暫存
#[inline]
fn scale_pixel_linear(p: RGB8, brightness: u8) -> RGB8 {
    if brightness >= 255 { return p; }
    let b = brightness as u16;
    RGB8 {
        r: ((p.r as u16 * b as u16 + 127) / 255) as u8, // +127 做簡單四捨五入
        g: ((p.g as u16 * b as u16 + 127) / 255) as u8,
        b: ((p.b as u16 * b as u16 + 127) / 255) as u8,
    }
}

/// 彩虹色盤（類似 FastLED 的 wheel）
#[inline]
fn wheel(mut pos: u8) -> RGB8 {
    pos = 255 - pos;
    if pos < 85 {
        RGB8 { r: 255 - pos * 3, g: 0, b: pos * 3 }
    } else if pos < 170 {
        pos -= 85;
        RGB8 { r: 0, g: pos * 3, b: 255 - pos * 3 }
    } else {
        pos -= 170;
        RGB8 { r: pos * 3, g: 255 - pos * 3, b: 0 }
    }
}
