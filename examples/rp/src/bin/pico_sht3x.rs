#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{main, Spawner};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// pico_htu21d.rs (repurposed) - SHT30 single-shot measurement demo
// - I2C0: SCL=GP5, SDA=GP4 at 400kHz
// - Every 15s: trigger high-repeatability single-shot, read T/RH, CRC check (poly 0x31, init 0xFF)
// - Print temperature/humidity (centi + formatted), LED: ok=1 blink, error=2 blinks

const SHT30_ADDR: u8 = 0x44; // 0x45 if ADDR pulled high
const CMD_SINGLE_SHOT_HIGHREP: [u8; 2] = [0x2C, 0x06];

#[main]
async fn main(_spawner: Spawner) {
    info!("SHT30 demo start");

    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_22, Level::Low);

    // I2C0, 400kHz
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    let mut i2c = I2c::new_blocking(p.I2C0, p.PIN_5, p.PIN_4, cfg);

    loop {
        match sht30_measure(&mut i2c).await {
            Ok((t_centi, rh_centi)) => {
                let ti = t_centi / 100; let tf = (t_centi % 100).abs();
                let ri = rh_centi / 100; let rf = (rh_centi % 100).abs();
                info!("SHT30: T={} ({}.{:02}°C), RH={} ({}.{:02}%)", t_centi, ti, tf, rh_centi, ri, rf);
                led_blink(&mut led, 1, 150).await;
            }
            Err(e) => {
                warn!("SHT30 read err: {}", e);
                led_blink(&mut led, 2, 120).await;
            }
        }

        Timer::after_millis(15_000).await;
    }
}

async fn sht30_measure(i2c: &mut I2c<'static, I2C0, i2c::Blocking>) -> Result<(i32, i32), &'static str> {
    // trigger single-shot
    i2c.blocking_write(SHT30_ADDR, &CMD_SINGLE_SHOT_HIGHREP).map_err(|_| "i2c write")?;
    // wait conversion
    Timer::after_millis(20).await;

    // read 6 bytes
    let mut buf = [0u8; 6];
    i2c.blocking_read(SHT30_ADDR, &mut buf).map_err(|_| "i2c read")?;

    // CRC check (poly=0x31, init=0xFF)
    if !sht_crc_ok(&buf[0..2], buf[2]) { return Err("temp crc"); }
    if !sht_crc_ok(&buf[3..5], buf[5]) { return Err("rh crc"); }

    let st = u16::from_be_bytes([buf[0], buf[1]]) as u32;
    let srh = u16::from_be_bytes([buf[3], buf[4]]) as u32;

    // formulas
    let t_c = -45.0f32 + 175.0f32 * (st as f32) / 65535.0;
    let rh = 100.0f32 * (srh as f32) / 65535.0;

    let t_centi = ((t_c * 100.0) + if (t_c * 100.0) >= 0.0 { 0.5 } else { -0.5 }) as i32;
    let rh_centi = ((rh * 100.0) + if (rh * 100.0) >= 0.0 { 0.5 } else { -0.5 }) as i32;
    Ok((t_centi, rh_centi))
}

fn sht_crc_ok(data: &[u8], crc: u8) -> bool {
    let mut x: u8 = 0xFF;
    for &b in data {
        x ^= b;
        for _ in 0..8 {
            let msb = (x & 0x80) != 0;
            x <<= 1;
            if msb { x ^= 0x31; }
        }
    }
    x == crc
}

async fn led_blink(led: &mut Output<'static>, times: usize, ms: u64) {
    for _ in 0..times {
        led.set_high();
        Timer::after_millis(ms).await;
        led.set_low();
        Timer::after_millis(ms).await;
    }
}
