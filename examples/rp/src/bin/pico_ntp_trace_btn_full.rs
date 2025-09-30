#![no_std]
#![no_main]

// NTP one-shot over UDP with detailed trace logs.
// This version initializes CYW43 + embassy-net locally (no external symbol).

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::{info, warn, error};
use defmt::unwrap;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net as net;
use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    Config as NetConfig, IpAddress, IpEndpoint, Ipv4Address,
};
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, PIO0},
    pio::Pio,
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use static_cell::StaticCell;
use {panic_probe as _};

// Firmware blobs (paths match your working project)
const FW: &[u8]  = include_bytes!("../../../../cyw43-firmware/43439A0.bin");
const CLM: &[u8] = include_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");

// Button/LED pins (same as your habit)
//const LED_IS_GP22: bool = true;
//const BTN_IS_GP15: bool = true;

// Target NTP server
const NTP_SERVER: Ipv4Address = Ipv4Address::new(192, 168, 188, 100);

// Wi-Fi creds (reuse your helper style)
fn wifi_ssid() -> &'static str {
    option_env!("WIFI_SSID").unwrap_or("WAX2617")
}
fn wifi_pass() -> &'static str {
    option_env!("WIFI_PASS").unwrap_or("7499363II5495264")
}

// ===== Interrupt bindings =====
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
//use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    //DMA_IRQ_0  => DmaInterruptHandler;
});

// ===== Background tasks (runner-based) =====
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

// ===== UDP NTP one-shot with detailed trace =====
async fn ntp_once(stack: &net::Stack<'static>, server: Ipv4Address) -> Result<(u64, u32), ()> {
    // Embassy-net socket requires meta+buf, owned via StaticCell (no static mut refs)
    static RX_BUF_CELL:  StaticCell<[u8; 512]> = StaticCell::new();
    static TX_BUF_CELL:  StaticCell<[u8; 128]> = StaticCell::new();
    static RX_META_CELL: StaticCell<[PacketMetadata; 4]> = StaticCell::new();
    static TX_META_CELL: StaticCell<[PacketMetadata; 4]> = StaticCell::new();

    // Initialize once; returns &'static T. Convert to &mut [..] slices for the socket.
    let rx_buf:  &mut [u8]             = RX_BUF_CELL.init([0; 512]);
    let tx_buf:  &mut [u8]             = TX_BUF_CELL.init([0; 128]);
    let rx_meta: &mut [PacketMetadata] = RX_META_CELL.init([PacketMetadata::EMPTY; 4]);
    let tx_meta: &mut [PacketMetadata] = TX_META_CELL.init([PacketMetadata::EMPTY; 4]);

    let mut socket = UdpSocket::new(
        *stack,
        rx_meta,
        rx_buf,
        tx_meta,
        tx_buf,
    );

    // Bind to any local port
    unwrap!(socket.bind(IpEndpoint::new(IpAddress::v4(0,0,0,0), 0)));

    let endpoint = IpEndpoint::new(IpAddress::Ipv4(server), 8123);

    // NTP 48B request (LI=0, VN=4, Mode=3 → 0x23)
    let mut buf = [0u8; 48];
    buf[0] = 0x23;

    
    info!("NTP -> sending 48B to {}:{} ...", server, endpoint.port);
    // Log first byte (LI|VN|Mode) and buffer len
    info!("TX first byte (LI|VN|Mode) = 0x{:x}, len={}", buf[0], buf.len());

    let mut rbuf = [0u8; 128];
    let mut last_err = 0u8;
    // Try up to 3 times, 1s timeout each
    for attempt in 1..=3u8 {
        info!("attempt {}: send_to + await reply (1s timeout)", attempt);
        if let Err(e) = socket.send_to(&buf, endpoint).await {
            error!("udp send_to failed: {:?}", e);
            last_err = 1;
            // short backoff before next try
            Timer::after_millis(150).await;
            continue;
        }

        // Wait up to 1s for response
        match with_timeout(Duration::from_millis(1000), socket.recv_from(&mut rbuf)).await {
            Err(_) => {
                warn!("attempt {}: timeout", attempt);
                last_err = 2;
                Timer::after_millis(150).await;
                continue;
            }
            Ok(Err(e)) => {
                error!("attempt {}: recv_from error: {:?}", attempt, e);
                last_err = 3;
                Timer::after_millis(150).await;
                continue;
            }
            Ok(Ok((n, from))) => {
                info!("NTP <- received {}B from {:?}", n, from);
                if n < 48 {
                    warn!("NTP packet too short: {}B", n);
                    return Err(());
                }
                // Basic header check
                let li_vn_mode = rbuf[0];
                let mode = li_vn_mode & 0b111;
                if mode != 4 && mode != 5 {
                    warn!("NTP invalid mode (expect 4/5), got {}", mode);
                    return Err(());
                }

                // Transmit Timestamp (40..48)
                let sec_be  = u32::from_be_bytes([rbuf[40], rbuf[41], rbuf[42], rbuf[43]]);
                let frac_be = u32::from_be_bytes([rbuf[44], rbuf[45], rbuf[46], rbuf[47]]);

                const NTP_TO_UNIX: u64 = 2_208_988_800;
                let unix_sec = (sec_be as u64).saturating_sub(NTP_TO_UNIX);
                let micros   = ((frac_be as u64) * 1_000_000) >> 32;

                info!("NTP parsed: ntp_sec={}, ntp_frac={}, unix={}s + {}us",
                    sec_be, frac_be, unix_sec, micros as u64);

                return Ok((unix_sec, micros as u32));
            }
        }
    }

    // If we got here, all attempts failed
    warn!("NTP timeout: no response after 3 attempts");
    return Err(());


    let li_vn_mode = rbuf[0];
    let mode = li_vn_mode & 0b111;
    if mode != 4 && mode != 5 {
        warn!("NTP invalid mode (expect 4/5), got {}", mode);
        return Err(());
    }

    // Transmit Timestamp (40..48)
    let sec_be  = u32::from_be_bytes([rbuf[40], rbuf[41], rbuf[42], rbuf[43]]);
    let frac_be = u32::from_be_bytes([rbuf[44], rbuf[45], rbuf[46], rbuf[47]]);

    const NTP_TO_UNIX: u64 = 2_208_988_800;
    let unix_sec = (sec_be as u64).saturating_sub(NTP_TO_UNIX);
    let micros   = ((frac_be as u64) * 1_000_000) >> 32;

    info!("NTP parsed: ntp_sec={}, ntp_frac={}, unix={}s + {}us",
        sec_be, frac_be, unix_sec, micros as u64);

    Ok((unix_sec, micros as u32))
}

// ===== main =====
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // LED / BTN
    let mut led = Output::new(p.PIN_22, Level::Low);
    let btn = Input::new(p.PIN_15, Pull::Up); // Active-Low

    // CYW43 power and SPI via PIO (exactly like your working file)
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
    let (net_device, mut control, cyw_runner) = cyw43::new(state, pwr, spi, FW).await;
    unwrap!(spawner.spawn(cyw43_task(cyw_runner)));

    // CLM init (fixes MAC=00 issues with DHCP)
    control.init(CLM).await;

    static RESOURCES: StaticCell<net::StackResources<4>> = StaticCell::new();
    let resources = RESOURCES.init(net::StackResources::<4>::new());
    let seed: u64 = Instant::now().as_ticks() as u64 ^ 0x5EED_1234_5678_ABCDu64;

    let config = NetConfig::dhcpv4(Default::default());
    let (stack, _net_runner) = net::new(net_device, config, resources, seed);
    unwrap!(spawner.spawn(net_task(_net_runner)));

    // Join Wi-Fi
    loop {
        match control
            .join(wifi_ssid(), JoinOptions::new(wifi_pass().as_bytes()))
            .await
        {
            Ok(()) => break,
            Err(e) => {
                warn!("join failed: status={}", e.status);
                Timer::after_millis(800).await;
            }
        }
    }
    info!("Wi-Fi joined: {}", wifi_ssid());
    stack.wait_link_up().await;
    stack.wait_config_up().await;

    info!("Ready. Press the button to send one NTP request to {}.", NTP_SERVER);

    // Button loop
    let mut last = btn.is_low(); // Active-Low
    loop {
        let now = btn.is_low();
        if now && !last {
            for _ in 0..3 {
                led.set_high();
                Timer::after_millis(60).await;
                led.set_low();
                Timer::after_millis(60).await;
            }

            info!("==== ntp_once() START ====");
            match ntp_once(&stack, NTP_SERVER).await {
                Ok((unix, micros)) => {
                    info!("NTP OK: unix={}s + {}us", unix, micros);
                    led.set_high();
                    Timer::after_millis(300).await;
                    led.set_low();
                }
                Err(_) => {
                    warn!("NTP FAIL (timeout or parse error)");
                    for _ in 0..3 {
                        led.set_high();
                        Timer::after_millis(80).await;
                        led.set_low();
                        Timer::after_millis(80).await;
                    }
                }
            }
            info!("==== ntp_once() END ====");
        }
        last = now;
        Timer::after_millis(20).await;
    }
}
