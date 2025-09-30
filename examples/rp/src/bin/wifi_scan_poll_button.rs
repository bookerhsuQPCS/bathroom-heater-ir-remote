//! Pico W: 按下 GPIO15 觸發 Wi-Fi 掃描（每按一次就掃一次）
//! 放在 src/bin/wifi_scan_poll_button.rs

#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::str;

use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello Pico W Wi-Fi scanner (button-triggered)");

    let p = embassy_rp::init(Default::default());

    // 按鈕：GPIO15，上拉，按下接地
    let mut button = Input::new(p.PIN_15, Pull::Up);

    // === Firmware 路徑沿用你的原寫法（從 src/bin/ 往上四層到 repo 根） ===
    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // === CYW43 初始化（同你原始流程） ===
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    info!("準備就緒：按下 GPIO15 來開始掃描。");

    // === 主循環：每按一次就掃一次 ===
    loop {
        // 等待按下（高->低），並簡單消抖
        button.wait_for_low().await;
        Timer::after(Duration::from_millis(30)).await;
        if !button.is_low() {
            continue;
        }

        info!("按鈕確認：開始 Wi-Fi 掃描…");

        // 執行一次掃描（不做去重、不切換省電模式）
        let mut count = 0u32;

        {
            let mut scanner = control.scan(Default::default()).await;
            while let Some(bss) = scanner.next().await {
                // SSID 可能是空字串（隱藏網路）或非 UTF-8
                match str::from_utf8(&bss.ssid) {
                    Ok(ssid) if !ssid.is_empty() => {
                        info!("AP {}: {}  BSSID={:x}  RSSI={}", count, ssid, bss.bssid, bss.rssi);
                    }
                    Ok(_) => {
                        info!("AP {}: <hidden>  BSSID={:x}  RSSI={}", count, bss.bssid, bss.rssi);
                    }
                    Err(_) => {
                        info!("AP {}: <non-utf8 SSID>  BSSID={:x}  RSSI={}", count, bss.bssid, bss.rssi);
                    }
                }
                count += 1;
            }
        } // scanner 在這裡被 drop，釋放對 control 的借用

        info!("本次掃描完成，共 {} 筆。放開按鈕以便下一次掃描。", count);

        // 等待放開（低->高），避免長按重觸發；再給 80ms 緩衝
        button.wait_for_high().await;
        Timer::after(Duration::from_millis(80)).await;
    }
}
