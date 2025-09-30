#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_time::{Duration, Timer};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use panic_probe as _;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

use u8g2_fonts::fonts::{
    u8g2_font_10x20_tr, u8g2_font_6x10_tr, u8g2_font_7x13_tr, u8g2_font_9x15_tr,
};
use u8g2_fonts::types::{HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{FontRenderer, U8g2TextStyle};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

// ===== 面板尺寸：預設 64x48 =====
type DispSize = DisplaySize64x48; // 若是 128x64：改為 DisplaySize128x64，並把 WIDTH/HEIGHT 改掉
const WIDTH: i32 = 64;
const HEIGHT: i32 = 48;
// =================================

// 底部被吃列時整體上移（負值=往上）
const Y_SHIFT: i32 = -4;

// 固定清的一條“字幕帶”高度（64x48 建議 24；128x64 可用 32）
const BAND_H: u32 = 24;

#[derive(Clone, Copy)]
enum FontSel {
    F6x10,
    F7x13,
    F9x15,
    F10x20,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("centered u8g2 fonts cycler on GP15");

    let p = embassy_rp::init(Default::default());

    // I2C：穩定優先
    let mut cfg = i2c::Config::default();
    cfg.frequency = 100_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, cfg);

    // SSD1306
    let iface = I2CDisplayInterface::new(i2c);
    let mut disp: Ssd1306<_, DispSize, BufferedGraphicsMode<_>> =
        Ssd1306::new(iface, DisplaySize64x48, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

    disp.init().unwrap();

    // GP15 按鈕（內建上拉，按下接地）
    let btn = Input::new(p.PIN_15, Pull::Up);

    let mut sel = FontSel::F6x10;
    draw_centered(&mut disp, sel);

    // 簡單去彈跳：抓下降沿
    let mut prev_low = btn.is_low();
    loop {
        let now_low = btn.is_low();
        if now_low && !prev_low {
            Timer::after(Duration::from_millis(20)).await; // 去彈跳
            if btn.is_low() {
                sel = next(sel);
                info!("font -> {}", name_of(sel));
                draw_centered(&mut disp, sel);

                while btn.is_low() {
                    Timer::after(Duration::from_millis(10)).await;
                }
            }
        }
        prev_low = now_low;
        Timer::after(Duration::from_millis(10)).await;
    }
}

fn next(f: FontSel) -> FontSel {
    match f {
        FontSel::F6x10 => FontSel::F7x13,
        FontSel::F7x13 => FontSel::F9x15,
        FontSel::F9x15 => FontSel::F10x20,
        FontSel::F10x20 => FontSel::F6x10,
    }
}

fn name_of(f: FontSel) -> &'static str {
    match f {
        FontSel::F6x10 => "6x10_tr",
        FontSel::F7x13 => "7x13_tr",
        FontSel::F9x15 => "9x15_tr",
        FontSel::F10x20 => "10x20_tr",
    }
}

// 置中顯示：FontRenderer 量測，U8g2TextStyle<BinaryColor> 繪製，並用「字幕帶」消除殘影
fn draw_centered<DI, SIZE>(disp: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>, sel: FontSel)
where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::prelude::DisplaySize,
{
    // 整屏清除（避免前一頁有殘留圖形）
    disp.clear(BinaryColor::Off).unwrap();

    let text = name_of(sel);

    match sel {
        FontSel::F6x10 => {
            let meas = FontRenderer::new::<u8g2_font_6x10_tr>();
            let style: U8g2TextStyle<BinaryColor> =
                U8g2TextStyle::new(u8g2_font_6x10_tr, BinaryColor::On);
            place_and_draw(disp, &meas, style, text);
        }
        FontSel::F7x13 => {
            let meas = FontRenderer::new::<u8g2_font_7x13_tr>();
            let style: U8g2TextStyle<BinaryColor> =
                U8g2TextStyle::new(u8g2_font_7x13_tr, BinaryColor::On);
            place_and_draw(disp, &meas, style, text);
        }
        FontSel::F9x15 => {
            let meas = FontRenderer::new::<u8g2_font_9x15_tr>();
            let style: U8g2TextStyle<BinaryColor> =
                U8g2TextStyle::new(u8g2_font_9x15_tr, BinaryColor::On);
            place_and_draw(disp, &meas, style, text);
        }
        FontSel::F10x20 => {
            let meas = FontRenderer::new::<u8g2_font_10x20_tr>();
            let style: U8g2TextStyle<BinaryColor> =
                U8g2TextStyle::new(u8g2_font_10x20_tr, BinaryColor::On);
            place_and_draw(disp, &meas, style, text);
        }
    }

    disp.flush().unwrap();
}

// 核心：清一條對齊到 8 的字幕帶，真置中量測，並把文字 Y 對齊到偶數列避免“糊”
fn place_and_draw<DI, SIZE>(
    disp: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    meas: &FontRenderer,
    style: U8g2TextStyle<BinaryColor>,
    text: &str,
) where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::prelude::DisplaySize,
{
    // 1) 算字幕帶頂端，對齊到 8 的倍數（SSD1306 page 對齊）
    let mut band_top = ((HEIGHT - BAND_H as i32) / 2) + Y_SHIFT;
    band_top &= !7; // 對齊 8 的倍數，避免跨 page

    // 2) 清整條 band（完整覆蓋之前的殘影/格線）
    let fill = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .build();
    Rectangle::new(Point::new(0, band_top), Size::new(WIDTH as u32, BAND_H))
        .into_styled(fill)
        .draw(disp)
        .ok();

    // 3) 以 band 幾何中心為置中點，量測文字 bbox
    let center = Point::new(WIDTH / 2, band_top + (BAND_H as i32) / 2);
    if let Some(bbox) = meas
        .get_rendered_dimensions_aligned(
            format_args!("{}", text),
            center,
            VerticalPosition::Center,
            HorizontalAlignment::Center,
        )
        .unwrap()
    {
        // 4) 將 Y snap 成偶數，避免奇數高度落在頁界造成視覺“糊”
        let snapped_y = (bbox.top_left.y & !1).max(band_top);

        // 5) Top 基線從左上角畫字（透明字，背景已清）
        Text::with_baseline(text, Point::new(bbox.top_left.x, snapped_y), style, Baseline::Top)
            .draw(disp)
            .unwrap();
    }
}
