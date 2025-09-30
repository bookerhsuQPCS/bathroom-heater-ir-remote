//! embassy/examples/rp/src/bin/pico_oled_demo.rs
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, I2c};
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::ascii::{FONT_4X6, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use panic_probe as _;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

// RP2040 I2C0 需要中斷綁定
bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

// ====== 螢幕尺寸切換 ======
// 0.66" 常見解析度：64x48
// 若你是 128x64：把下面兩處 `DisplaySize64x48` 改成 `DisplaySize128x64`
// 若你是 128x32：改成 `DisplaySize128x32`
type DispSize = DisplaySize64x48;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // I2C0: SDA=GP4, SCL=GP5，400kHz
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, cfg);
    info!("I2C ready.");

    // 顯示介面（ssd1306 v0.10 用 new()；init/flush 為阻塞版）
    let iface = I2CDisplayInterface::new(i2c);
    let mut disp: Ssd1306<_, DispSize, BufferedGraphicsMode<_>> =
        Ssd1306::new(iface, DisplaySize64x48, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

    // 阻塞初始化與清屏
    disp.init().unwrap();
    disp.clear(BinaryColor::Off).unwrap();

    // 文字樣式
    let style_mid = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let style_small = MonoTextStyleBuilder::new()
        .font(&FONT_4X6)
        .text_color(BinaryColor::On)
        .build();

    // 1) Hello, world
    Text::with_baseline("Hello, world!", Point::new(0, 8), style_mid, Baseline::Top)
        .draw(&mut disp)
        .unwrap();
    Text::with_baseline("Rust + Embassy", Point::new(0, 22), style_small, Baseline::Top)
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();

    Timer::after(Duration::from_secs(2)).await;

    // 清屏
    disp.clear(BinaryColor::Off).unwrap();
    disp.flush().unwrap();

    // 2) 畫線與圖形（線、矩形、圓）
    let white_stroke: PrimitiveStyle<BinaryColor> = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let filled_white = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::On)
        .build();

    // 對角線
    Line::new(Point::new(0, 0), Point::new(63, 47))
        .into_styled(white_stroke)
        .draw(&mut disp)
        .unwrap();
    // 反對角
    Line::new(Point::new(63, 0), Point::new(0, 47))
        .into_styled(white_stroke)
        .draw(&mut disp)
        .unwrap();
    // 外框
    Rectangle::new(Point::new(1, 1), Size::new(62, 46))
        .into_styled(white_stroke)
        .draw(&mut disp)
        .unwrap();
    // 實心方塊
    Rectangle::new(Point::new(24, 18), Size::new(16, 12))
        .into_styled(filled_white)
        .draw(&mut disp)
        .unwrap();
    // 圓
    Circle::new(Point::new(50, 10), 10)
        .into_styled(white_stroke)
        .draw(&mut disp)
        .unwrap();

    disp.flush().unwrap();
    Timer::after(Duration::from_secs(2)).await;

    // 3) 反白（硬體反相）
    disp.set_invert(true).unwrap();
    disp.flush().unwrap();
    Timer::after(Duration::from_secs(1)).await;

    // 再切回正常
    disp.set_invert(false).unwrap();
    disp.flush().unwrap();

    // 4) 橫向捲動（軟體方式：重繪移動中的內容）
    let text = "Scrolling demo >>";
    // 粗估字寬：FONT_4X6 約 6px/字
    let text_px_w = (text.len() as i32) * 6;

    for _round in 0..2 {
        // 從畫面左外側一路捲到最右
        for x in (-text_px_w)..64 {
            disp.clear(BinaryColor::Off).unwrap();

            // 參考外框
            Rectangle::new(Point::new(0, 0), Size::new(63, 47))
                .into_styled(white_stroke)
                .draw(&mut disp)
                .unwrap();

            // 跑馬燈文字
            Text::with_baseline(text, Point::new(x, 18), style_small, Baseline::Top)
                .draw(&mut disp)
                .unwrap();

            // 右上角小圖示
            Circle::new(Point::new(56, 6), 5)
                .into_styled(white_stroke)
                .draw(&mut disp)
                .unwrap();

            disp.flush().unwrap();
            Timer::after(Duration::from_millis(40)).await;
        }
    }

    // 結尾畫面
    disp.clear(BinaryColor::Off).unwrap();
    Text::with_baseline("Done.", Point::new(0, 16), style_mid, Baseline::Top)
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
