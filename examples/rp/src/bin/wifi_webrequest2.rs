// src/bin/wifi_webrequest.rs
//! RP Pico W: 連 Wi-Fi，DHCP 完成即印本機 IPv4；先測試兩個 HTTP 站，請求期間切 Performance 並亮 CYW43 LED。
//! Firmware 路徑使用已確認可行的 ../../../../cyw43-firmware/...

#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::str::from_utf8;

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use defmt::Debug2Format;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Config, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use reqwless::client::HttpClient; // 非 TLS
use reqwless::request::Method;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// 換成你的 Wi-Fi
const WIFI_NETWORK: &str = "WAX2617";
const WIFI_PASSWORD: &str = "7499363II5495264";

// 先測的 HTTP 站（寬容、好除錯）
const TEST_URLS: &[&str] = &[
    "http://neverssl.com/",
    "http://httpbin.org/get",
];

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    // 固件檔（從 src/bin/ 回到 repo 根）
    let fw = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

    // CYW43 / PIO / SPI
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
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // 網路堆疊（DHCPv4）
    let config = Config::dhcpv4(Default::default());
    let seed = rng.next_u64();
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), seed);
    unwrap!(spawner.spawn(net_task(runner)));

    // 連 Wi-Fi
    while let Err(err) = control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        info!("join failed with status={}", err.status);
    }

    info!("waiting for link...");
    stack.wait_link_up().await;

    info!("waiting for DHCP...");
    stack.wait_config_up().await;

    // ★ DHCP 完成後顯示本機 IPv4（以及 gateway）
    if let Some(cfg) = stack.config_v4() {
        // cfg.address 為 Ipv4Cidr（會印成 192.168.x.y/24）
        info!("IPv4 addr: {}  gw: {}", cfg.address, cfg.gateway);
    }
    info!("Stack is up!");

    loop {
        // 非 TLS 客戶端 + DNS（每次請求各自獨立）
        let client_state = TcpClientState::<1, 1024, 1024>::new();
        let tcp_client   = TcpClient::new(stack, &client_state);
        let dns_client   = DnsSocket::new(stack);
        let mut http_client = HttpClient::new(&tcp_client, &dns_client);

        for &url in TEST_URLS {
            // 每個請求前：切 Performance + 亮 CYW43 LED（index 0）
            control
                .set_power_management(cyw43::PowerManagementMode::Performance)
                .await;
            control.gpio_set(0, true).await;

            info!("connecting to {}", url);

            // 建請求
            let mut request = match http_client.request(Method::GET, url).await {
                Ok(req) => req,
                Err(e) => {
                    error!("Failed to make HTTP request: {}", Debug2Format(&e));
                    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;
                    control.gpio_set(0, false).await;
                    Timer::after(Duration::from_secs(3)).await;
                    continue;
                }
            };

            // 送出
            let mut rx_buffer = [0u8; 8192];
            let response = match request.send(&mut rx_buffer).await {
                Ok(resp) => resp,
                Err(e) => {
                    error!("Failed to send HTTP request: {}", Debug2Format(&e));
                    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;
                    control.gpio_set(0, false).await;
                    Timer::after(Duration::from_secs(3)).await;
                    continue;
                }
            };

            // 讀 body（只印前 200 字，避免刷屏）
            let body_str = match from_utf8(response.body().read_to_end().await.unwrap()) {
                Ok(b) => b,
                Err(e) => {
                    error!("Failed to read response body: {}", Debug2Format(&e));
                    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;
                    control.gpio_set(0, false).await;
                    Timer::after(Duration::from_secs(3)).await;
                    continue;
                }
            };
            let preview_len = body_str.len().min(200);
            info!("{} ok, {} bytes; preview:\n{:?}", url, body_str.len(), &body_str[..preview_len]);

            // 收尾：省電 & LED off
            control
                .set_power_management(cyw43::PowerManagementMode::PowerSave)
                .await;
            control.gpio_set(0, false).await;

            // 稍等再測下一個
            Timer::after(Duration::from_secs(2)).await;
        }

        // 迴圈休息一下再重新測
        Timer::after(Duration::from_secs(10)).await;
    }
}
