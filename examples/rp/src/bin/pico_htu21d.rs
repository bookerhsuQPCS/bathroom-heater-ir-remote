#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{main, Spawner};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{I2C0};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const HTU21D_ADDR: u8 = 0x40;
const CMD_TRIG_TEMP_NOHOLD: u8 = 0xF3;
const CMD_TRIG_RH_NOHOLD: u8   = 0xF5;

// CRC-8 (poly 0x31, init 0x00), MSB-first
fn htu21d_crc(data: [u8; 2]) -> u8 {
    let mut crc: u8 = 0x00;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 { (crc << 1) ^ 0x31 } else { crc << 1 };
        }
    }
    crc
}

// 讀一次 No-Hold 測量：寫命令 -> 等待 -> 讀3 bytes -> 驗證CRC -> 去除狀態位
async fn read_measurement_blocking(
    i2c: &mut I2c<'_, I2C0, i2c::Blocking>,
    cmd: u8,
    wait_ms: u64,
) -> Result<u16, &'static str> {
    i2c.blocking_write(HTU21D_ADDR, &[cmd]).map_err(|_| "i2c write fail")?;
    Timer::after_millis(wait_ms).await;

    let mut buf = [0u8; 3];
    i2c.blocking_read(HTU21D_ADDR, &mut buf).map_err(|_| "i2c read fail")?;

    let crc = htu21d_crc([buf[0], buf[1]]);
    if crc != buf[2] {
        return Err("crc mismatch");
    }

    let raw = ((((buf[0] as u16) << 8) | buf[1] as u16) & 0xFFFC) as u16;
    Ok(raw)
}

// 轉為「百分位」整數（centi-units），避免浮點四捨五入/連結問題
fn temp_centi(raw: u16) -> i32 {
    // T*100 = -4685 + (17572 * raw) / 65536
    -4685 + ((17572i32 * raw as i32) >> 16)
}
fn rh_centi(raw: u16) -> i32 {
    // RH*100 = -600 + (12500 * raw) / 65536，並夾在 0..10000
    let v = -600 + ((12500i32 * raw as i32) >> 16);
    v.clamp(0, 10000)
}

// 手動格式化兩位小數：1234 -> "12.34"
fn log_centi(label: &str, centi: i32) {
    let sign = if centi < 0 { "-" } else { "" };
    let a = centi.abs() as u32;
    let intp = a / 100;
    let frac = a % 100;
    let d1 = frac / 10;
    let d0 = frac % 10;
    info!("{}={}{}.{}{}", label, sign, intp, d1, d0);
}

// LED 閃爍：times 次、每次亮/滅 duration_ms
async fn led_blink(led: &mut Output<'_>, times: u8, duration_ms: u64) {
    for _ in 0..times {
        led.set_high();
        Timer::after_millis(duration_ms).await;
        led.set_low();
        Timer::after_millis(duration_ms).await;
    }
}

#[main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // I2C0: SCL=GPIO5, SDA=GPIO4；100kHz；模組自帶上拉
    let mut i2c = {
        let mut cfg = i2c::Config::default();
        cfg.frequency = 100_000;
        I2c::new_blocking(p.I2C0, p.PIN_5, p.PIN_4, cfg)
    };

    // LED: GPIO22
    let mut led = Output::new(p.PIN_22, Level::Low);

    info!("HTU21D demo: I2C0 SCL=GP5, SDA=GP4; LED=GP22; No-Hold + CRC");

    loop {
        // 溫度
        match read_measurement_blocking(&mut i2c, CMD_TRIG_TEMP_NOHOLD, 60).await {
            Ok(raw_t) => {
                let t = temp_centi(raw_t);
                log_centi("T(°C)", t);
            }
            Err(e) => {
                warn!("temp read err: {}", e);
                led_blink(&mut led, 2, 100).await; // 錯誤：閃兩下
                continue;
            }
        }

        // 濕度
        match read_measurement_blocking(&mut i2c, CMD_TRIG_RH_NOHOLD, 30).await {
            Ok(raw_rh) => {
                let rh = rh_centi(raw_rh);
                // RH(%)=xx.xx %
                let a = rh.abs() as u32;
                let intp = a / 100;
                let frac = a % 100;
                let d1 = frac / 10;
                let d0 = frac % 10;
                info!("RH(%)={}.{}{} %", intp, d1, d0);
            }
            Err(e) => {
                warn!("rh read err: {}", e);
                led_blink(&mut led, 2, 100).await; // 錯誤：閃兩下
                continue;
            }
        }

        // 成功：閃一下
        led_blink(&mut led, 1, 200).await;

        Timer::after_millis(15000).await;
    }
}
