// src/bin/wifi_webrequest.rs
//! RP Pico W: 連 Wi-Fi，DHCP 完成印 IPv4；依 URL 自動測 HTTP/HTTPS，請求期間切 Performance + 亮 CYW43 LED。
//! 固件路徑維持 ../../../../cyw43-firmware/...

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
use reqwless::client::{HttpClient, TlsConfig, TlsVerify};
use reqwless::request::Method;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// 換成你的 Wi-Fi
const WIFI_NETWORK: &str = "WAX2617";
const WIFI_PASSWORD: &str = "7499363II5495264";

// 測試清單：先 HTTP，再 HTTPS
const TEST_URLS: &[&str] = &[
    "http://neverssl.com/",
    "http://httpbin.org/get",
    "https://example.com/",
    "https://httpbin.org/get",
    // "https://worldtimeapi.org/api/timezone/Europe/Berlin",
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

    // DHCP 完成：印本機 IPv4 / gateway
    if let Some(cfg) = stack.config_v4() {
        info!("IPv4 addr: {}  gw: {}", cfg.address, cfg.gateway);
    }
    info!("Stack is up!");

    loop {
        // 每輪都新建 client，避免狀態殘留
        let client_state = TcpClientState::<1, 1024, 1024>::new();
        let tcp_client   = TcpClient::new(stack, &client_state);
        let dns_client   = DnsSocket::new(stack);

        for &url in TEST_URLS {
            let is_https = url.as_bytes().starts_with(b"https://");

            // 請求期間：切 Performance + 亮 CYW43 LED
            control
                .set_power_management(cyw43::PowerManagementMode::Performance)
                .await;
            control.gpio_set(0, true).await;

            info!("connecting to {}", url);

            if is_https {
                // —— HTTPS 分支：把 TLS buffer / client / request / response 全部包在更小作用域 —— //
                {
                    let mut rx_buffer        = [0u8; 8192];
                    let mut tls_read_buffer  = [0u8; 16_640];
                    let mut tls_write_buffer = [0u8; 16_640];

                    let tls_config = TlsConfig::new(
                        seed,
                        &mut tls_read_buffer,
                        &mut tls_write_buffer,
                        TlsVerify::None, // 若要驗證憑證，之後可改
                    );

                    {
                        let mut https_client = HttpClient::new_with_tls(&tcp_client, &dns_client, tls_config);

                        // 最後的 match 都加分號，確保成「語句」→ 其暫時值會立刻 drop
                        match https_client.request(Method::GET, url).await {
                            Ok(mut req) => {
                                match req.send(&mut rx_buffer).await {
                                    Ok(resp) => {
                                        match from_utf8(resp.body().read_to_end().await.unwrap()) {
                                            Ok(b) => {
                                                let preview_len = b.len().min(200);
                                                info!("{} ok, {} bytes; preview:\n{:?}", url, b.len(), &b[..preview_len]);
                                            }
                                            Err(e) => error!("Failed to read HTTPS body: {}", Debug2Format(&e)),
                                        }
                                    }
                                    Err(e) => error!("Failed to send HTTPS request: {}", Debug2Format(&e)),
                                }
                            }
                            Err(e) => error!("Failed to make HTTPS request: {}", Debug2Format(&e)),
                        };
                    } // ← https_client 在這裡 drop
                } // ← tls_* 與 rx_buffer 在這裡 drop
            } else {
                // —— HTTP 分支：同理全部包起來 —— //
                {
                    let mut rx_buffer = [0u8; 8192];
                    let mut http_client = HttpClient::new(&tcp_client, &dns_client);

                    match http_client.request(Method::GET, url).await {
                        Ok(mut req) => {
                            match req.send(&mut rx_buffer).await {
                                Ok(resp) => {
                                    match from_utf8(resp.body().read_to_end().await.unwrap()) {
                                        Ok(b) => {
                                            let preview_len = b.len().min(200);
                                            info!("{} ok, {} bytes; preview:\n{:?}", url, b.len(), &b[..preview_len]);
                                        }
                                        Err(e) => error!("Failed to read HTTP body: {}", Debug2Format(&e)),
                                    }
                                }
                                Err(e) => error!("Failed to send HTTP request: {}", Debug2Format(&e)),
                            }
                        }
                        Err(e) => error!("Failed to make HTTP request: {}", Debug2Format(&e)),
                    };
                } // ← http_client / rx_buffer 在這裡 drop
            }

            // 收尾：省電 & LED off
            control
                .set_power_management(cyw43::PowerManagementMode::PowerSave)
                .await;
            control.gpio_set(0, false).await;

            Timer::after(Duration::from_secs(2)).await;
        }

        // 休息一下再跑下一輪
        Timer::after(Duration::from_secs(10)).await;
    }
}
