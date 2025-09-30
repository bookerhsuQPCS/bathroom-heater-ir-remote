#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, OutputOpenDrain};
use embassy_rp::i2c::{self, I2c};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::mono_font::ascii::{FONT_7X13, FONT_9X15};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use panic_probe as _;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

use onewire::{ds18b20, ds18b20::DS18B20, DeviceSearch, OneWire};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

// === 面板尺寸：預設 64x48；若 128x64，兩處一起改 ===
type DispSize = DisplaySize64x48;
// ===============================================

// 是否用 9x15 當主要溫度字體；false 則用 7x13
const USE_FONT_9X15: bool = true;
// 是否為寄生供電（沒接 Vdd）
const PARASITE_POWER: bool = false;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // I2C：穩定優先
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 100_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_cfg);
    let iface = I2CDisplayInterface::new(i2c);
    let mut disp: Ssd1306<_, DispSize, BufferedGraphicsMode<_>> =
        Ssd1306::new(iface, DisplaySize64x48, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
    // 換 128x64 時改為：Ssd1306::new(iface, DisplaySize128x64, ...)

    disp.init().unwrap();
    disp.clear(BinaryColor::Off).unwrap();

    // 樣式：標題用 7x13；溫度字體依開關選 9x15 或 7x13，且有背景避免殘影
    let style_title: MonoTextStyle<BinaryColor> = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(BinaryColor::On)
        .build();

    let font_temp = if USE_FONT_9X15 { &FONT_9X15 } else { &FONT_7X13 };
    let style_temp_bg: MonoTextStyle<BinaryColor> = MonoTextStyleBuilder::new()
        .font(font_temp)
        .text_color(BinaryColor::On)
        .background_color(BinaryColor::Off)
        .build();

    // 標題一次
    Text::with_baseline("DS18B20", Point::new(0, 0), style_title, Baseline::Top)
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();
    Timer::after(Duration::from_millis(400)).await;

    // 1-Wire on GP13
    let mut dq = OutputOpenDrain::new(p.PIN_13, Level::High);
    let mut bus = OneWire::new(&mut dq, PARASITE_POWER);
    let mut delay = Delay;

    match bus.reset(&mut delay) {
        Ok(true) => info!("1-Wire presence detected."),
        Ok(false) => { warn!("No devices on 1-Wire bus."); show_err(&mut disp, "No device"); loop { Timer::after(Duration::from_secs(1)).await; } }
        Err(e) => { error!("1-Wire reset err: {:?}", e); show_err(&mut disp, "1W reset err"); loop { Timer::after(Duration::from_secs(1)).await; } }
    }

    // 搜第一顆
    let mut search = DeviceSearch::new_for_family(ds18b20::FAMILY_CODE);
    let device = match bus.search_next(&mut search, &mut delay) {
        Ok(Some(dev)) => dev,
        Ok(None) => { show_err(&mut disp, "No sensor"); loop { Timer::after(Duration::from_secs(1)).await; } }
        Err(e) => { error!("search_next: {:?}", e); show_err(&mut disp, "Search err"); loop { Timer::after(Duration::from_secs(1)).await; } }
    };

    // defmt：把 ROM 轉 u64 印十六進位
    let rom_u64 = u64::from_le_bytes(device.address);
    info!("ROM={=u64:X}", rom_u64);

    let sensor = match DS18B20::new(device) {
        Ok(s) => s,
        Err(e) => { error!("Not DS18B20? {:?}", e); show_err(&mut disp, "Bad device"); loop { Timer::after(Duration::from_secs(1)).await; } }
    };

    loop {
        // 確認 bus 還活著
        match bus.reset(&mut delay) {
            Ok(true) => {}
            _ => { warn!("1-Wire lost"); show_err(&mut disp, "Bus lost"); Timer::after(Duration::from_millis(500)).await; continue; }
        }

        if let Err(e) = sensor.measure_temperature(&mut bus, &mut delay) {
            warn!("measure fail: {:?}", e);
            draw_status(&mut disp, "Meas fail", style_title, style_temp_bg);
            Timer::after(Duration::from_millis(700)).await;
            continue;
        }
        Timer::after(Duration::from_millis(750)).await; // 12-bit 典型

        match sensor.read_temperature(&mut bus, &mut delay) {
            Ok(raw) => {
                let (intc, frac) = ds18b20::split_temp(raw);

                // 組 "T=xx.xxC"
                let neg = intc < 0;
                let mut ii: i32 = intc as i32;
                if neg { ii = -ii; }
                let mut two = (frac as u32 + 50) / 100;
                let mut i_part = ii as u32;
                if two >= 100 { two = 0; i_part += 1; }

                let mut num = [0u8; 8];
                let nlen = u32_to_ascii(i_part, &mut num);
                let istr = core::str::from_utf8(&num[..nlen]).unwrap();

                let dd_tens = (two / 10) as u8 + b'0';
                let dd_ones = (two % 10) as u8 + b'0';

                let mut line = [0u8; 16];
                let mut idx = 0usize;
                line[idx]=b'T'; idx+=1;
                line[idx]=b'='; idx+=1;
                if neg { line[idx]=b'-'; idx+=1; }
                for &b in istr.as_bytes() { line[idx]=b; idx+=1; }
                line[idx]=b'.'; idx+=1;
                line[idx]=dd_tens; idx+=1;
                line[idx]=dd_ones; idx+=1;
                line[idx]=b'C'; idx+=1;

                let text = core::str::from_utf8(&line[..idx]).unwrap();

                // 整屏清除 + 有背景字，杜絕糊字
                disp.clear(BinaryColor::Off).unwrap();
                Text::with_baseline("DS18B20", Point::new(0, 0), style_title, Baseline::Top)
                    .draw(&mut disp).unwrap();

                // 64x48 用 y=18；若 128x64 建議 y=24
                let y = if core::mem::size_of::<DispSize>() == core::mem::size_of::<DisplaySize64x48>() { 18 } else { 24 };
                Text::with_baseline(text, Point::new(0, y), style_temp_bg, Baseline::Top)
                    .draw(&mut disp).unwrap();

                disp.flush().unwrap();
                info!("temp={}", text);
            }
            Err(e) => {
                warn!("read fail: {:?}", e);
                draw_status(&mut disp, "Read fail", style_title, style_temp_bg);
            }
        }

        Timer::after(Duration::from_millis(800)).await;
    }
}

// 狀態/錯誤顯示：整屏清後再畫
fn draw_status<DI, SIZE>(
    disp: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    msg: &str,
    style_title: MonoTextStyle<BinaryColor>,
    style_val_bg: MonoTextStyle<BinaryColor>,
) where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::prelude::DisplaySize,
{
    disp.clear(BinaryColor::Off).unwrap();
    Text::with_baseline("DS18B20", Point::new(0, 0), style_title, Baseline::Top)
        .draw(disp).unwrap();
    Text::with_baseline(msg, Point::new(0, 18), style_val_bg, Baseline::Top)
        .draw(disp).unwrap();
    Rectangle::new(Point::new(0, 0), Size::new(63, 47))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(disp).unwrap();
    disp.flush().unwrap();
}

// 無 heap 的 u32→ascii
fn u32_to_ascii(mut n: u32, buf: &mut [u8]) -> usize {
    if n == 0 { if !buf.is_empty() { buf[0]=b'0'; return 1; } return 0; }
    let mut tmp = [0u8; 10];
    let mut i = 0;
    while n > 0 && i < 10 { tmp[i]=b'0'+(n%10)as u8; n/=10; i+=1; }
    let mut j = 0;
    while i > 0 && j < buf.len() { i-=1; buf[j]=tmp[i]; j+=1; }
    j
}

// 簡易錯誤畫面：整屏清除、顯示訊息（避免殘影）
fn show_err<DI, SIZE>(
    disp: &mut ssd1306::Ssd1306<DI, SIZE, ssd1306::mode::BufferedGraphicsMode<SIZE>>,
    msg: &str,
) where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::prelude::DisplaySize,
{
    use embedded_graphics::mono_font::ascii::FONT_7X13;
    use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
    use embedded_graphics::pixelcolor::BinaryColor;
    use embedded_graphics::prelude::*;
    use embedded_graphics::text::{Baseline, Text};

    let style: MonoTextStyle<BinaryColor> = MonoTextStyleBuilder::new()
        .font(&FONT_7X13)
        .text_color(BinaryColor::On)
        .background_color(BinaryColor::Off) // 有背景，杜絕殘影
        .build();

    disp.clear(BinaryColor::Off).unwrap();
    Text::with_baseline("DS18B20", Point::new(0, 0), style, Baseline::Top)
        .draw(disp)
        .unwrap();
    Text::with_baseline(msg, Point::new(0, 18), style, Baseline::Top)
        .draw(disp)
        .unwrap();
    disp.flush().unwrap();
}
