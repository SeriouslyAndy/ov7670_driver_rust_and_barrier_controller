
/* 
#![no_std]
#![no_main]

use cyw43::JoinOptions;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, StackResources};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use embassy_lab_utils::{init_wifi, init_network_stack};
use core::str;

use defmt::*;

mod irqs;

const SOCK: usize = 4;
static RESOURCES: StaticCell<StackResources<SOCK>> = StaticCell::new();


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Get a handle to the RP's peripherals.
    let peripherals = embassy_rp::init(Default::default());

    // Init WiFi driver
    let (net_device, mut control) = init_wifi!(&spawner, peripherals).await;
    // Default config for dynamic IP address
   //let config = embassy_net::Config::dhcpv4(Default::default());
    // Init network stack

    let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(192, 168, 4, 1), 24),
        dns_servers: Vec::new(),
        gateway: None,
    });

    let stack = init_network_stack(&spawner, net_device, &RESOURCES, config);

    control.start_ap_wpa2("Nambani2", "andy123", 6).await;

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
 /*    control.start_ap_wpa2("Nambani2", "andy123", 5).await;
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);*/
    // If we want to keep the connection open regardless of inactivity, we can set the timeout

    info!("Listening on TCP:1234...");
    if let Err(e) = socket.accept(1234).await {
        warn!("accept error: {:?}", e);
        return;
    }

    info!("Received connection");

    let mut buf = [0; 4096];
    loop {
        let n = match socket.read(&mut buf).await {
            Ok(0) => {
                warn!("read EOF");
                break;
            }
            Ok(n) => n,
            Err(e) => {
                warn!("read error: {:?}", e);
                break;
            }
        };

        info!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());
    }

    info!("High Five scan");
    {
        let mut scanner = control.scan(Default::default()).await;
        while let Some(bss) = scanner.next().await {
            if let Ok(ssid) = str::from_utf8(&bss.ssid) {
                info!("Found network: {}", ssid);
            }
        }
    }
    let delay = Duration::from_secs(60);

    const WIFI_NETWORK: &str = "Nambani";
    const WIFI_PASSWORD: &str = "123andy123";
    loop {

        match control.join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
        Timer::after(delay).await;
    }
    info!("waiting for DHCP...");
    while !stack.is_config_up() {
     Timer::after_millis(100).await;
        }
info!("DHCP is now up!");
}


/*
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, StackResources};
use embassy_time::Duration;
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use embassy_lab_utils::{init_wifi, init_network_stack};

use defmt::*;

mod irqs;

const SOCK: usize = 4;
static RESOURCES: StaticCell<StackResources<SOCK>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());

    let (net_device, mut control) = init_wifi!(&spawner, peripherals).await;

    let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(192, 168, 4, 1), 24),
        dns_servers: Vec::new(),
        gateway: None,
    });

    let stack = init_network_stack(&spawner, net_device, &RESOURCES, config);

    control.start_ap_wpa2("Nambani2", "andy12345", 6).await;
    info!("Access Point running: Nambani2, IP: 192.168.4.1");

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    info!("socket:6000.");
    if let Err(e) = socket.accept(6000).await {
        warn!("accept error: {:?}", e);
        return;
    }

    info!("Received connection");

    let mut buf = [0; 4096];
    loop {
        let n = match socket.read(&mut buf).await {
            Ok(0) => {
                warn!("read EOF");
                break;
            }
            Ok(n) => n,
            Err(e) => {
                warn!("read error: {:?}", e);
                break;
            }
        };

        info!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());
    }
}*/
  *//* 
    #![no_std]
  #![no_main]

  use defmt::*;
  use embassy_executor::Spawner;
  use embassy_rp::gpio::{Input, Level, Output, Pin, Pull, AnyPin};
  use embassy_rp::peripherals::*;
  use embassy_time::{Delay, Timer};
  use embedded_hal_async::delay::DelayNs;
  use static_cell::StaticCell;
  use {defmt_rtt as _, panic_probe as _};
  use core::cell::RefCell;
  use embassy_rp::bind_interrupts;
  use embedded_hal_1::i2c::I2c as HalI2c;
  use embassy_rp::i2c::{self, I2c, InterruptHandler as I2CInterruptHandler, Config as I2cConfig};
use embedded_hal_async::i2c::{Error, I2c as _};
use embassy_rp::peripherals::I2C0;

  use embassy_rp::uart::{self, Uart};

  const WIDTH: usize = 320;
  const HEIGHT: usize = 240;
  const PIXEL_BYTES: usize = 2;

  const FRAMEBUFFER_SIZE: usize = WIDTH * HEIGHT * PIXEL_BYTES / 10;
  static FRAMEBUFFER: StaticCell<[u8; FRAMEBUFFER_SIZE]> = StaticCell::new();

  #[embassy_executor::main]
  async fn main(_spawner: Spawner) {
      info!("Initializing peripherals...");
      let p = embassy_rp::init(Default::default());
      let mut delay = Delay;
      let framebuffer = FRAMEBUFFER.init([0u8; FRAMEBUFFER_SIZE]);

      let mut i2c = I2c::new_blocking(p.I2C0, p.PIN_9, p.PIN_8, i2c::Config::default());
  //clock config -pclk pixel clock
  //max 400 khz

  //to do config i2c
  //figure out what to do withcamera
  //rmv uart and replace with i2c reader.

      info!("Resetting OV7670...");
      let mut reset = Output::new(p.PIN_10.degrade(), Level::Low);
      let mut pwnn = Output::new(p.PIN_22.degrade(), Level::Low);
      // pow-off
      reset.set_low();
      pwnn.set_low();
      delay.delay_ms(10);

      // pwnn
      pwnn.set_high();
      delay.delay_ms(10);

      // reset aici
      reset.set_high();
      delay.delay_ms(10);

      info!("Configuring OV7670...");
      if let Err(_) = ov7670_config(&mut i2c).await {
          info!("OV7670 config failed!");
          return;
      }
      info!("OV7670 configured successfully");


      let vsync = Input::new(p.PIN_7.degrade(), Pull::None);
      let href = Input::new(p.PIN_21.degrade(), Pull::None);
      let pclk = Input::new(p.PIN_11.degrade(), Pull::None);

      let d_pins = [
          Input::new(p.PIN_12.degrade(), Pull::None),
          Input::new(p.PIN_13.degrade(), Pull::None),
          Input::new(p.PIN_14.degrade(), Pull::None),
          Input::new(p.PIN_15.degrade(), Pull::None),
          Input::new(p.PIN_16.degrade(), Pull::None),
          Input::new(p.PIN_17.degrade(), Pull::None),
          Input::new(p.PIN_18.degrade(), Pull::None),
          Input::new(p.PIN_19.degrade(), Pull::None),
      ];

      info!("Waiting for VSYNC (frame start)...");
      while vsync.is_high() {}
      while vsync.is_low() {}
      info!("VSYNC received. Capturing image...");

      let mut i = 0;
      for _ in 0..HEIGHT {
          while href.is_low() {}
          for _ in 0..WIDTH {
              // read 1st byte
              while pclk.is_low() {}
              let high_byte = read_byte(&d_pins);
              while pclk.is_high() {}

              // 2nd byte
              while pclk.is_low() {}
              let low_byte = read_byte(&d_pins);
              while pclk.is_high() {}

              unsafe {
                  if i + 1 < framebuffer.len() {
                      framebuffer[i] = high_byte;
                      framebuffer[i + 1] = low_byte;
                      i += 2;
                  } else {
                      info!("Framebuffer overflow");
                      break;
                  }
              }
          }
          while href.is_high() {}
      }

      info!("Image captured. Sending to PC over UART...");

      let mut uart = Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, uart::Config::default());

      unsafe {
          for (idx, b) in framebuffer.iter().enumerate() {
              if let Err(_) = uart.blocking_write(&[*b]) {
                  info!("UART write failed at byte {}", idx);
                  break;
              }
          }
      }

      info!("Image sent");
  }

  fn read_byte(d_pins: &[Input<'_>; 8]) -> u8 {
      let mut byte = 0u8;
      for (i, pin) in d_pins.iter().enumerate() {
          if pin.is_high() {
              byte |= 1 << i;
          }
      }
      byte
  }

  async fn ov7670_config<I2C, E>(i2c: &mut I2C) -> Result<(), ()>
  where
      I2C: HalI2c<Error = E> + core::marker::Unpin,
      E: core::fmt::Debug,
  {
      let addr = 0x21; // OV7670 I2C address

      // Reset all registers
      if i2c.write(addr, &[0x12, 0x80]).is_err() { //issue here?????????????????????
          info!("❗ Failed to reset camera");
          return Err(());
      }
      Timer::after_millis(100).await;

      // Configure for QVGA RGB565 output
      let init_sequence = &[
          // Set QVGA and RGB output
          (0x12, 0x14),  // COM7: QVGA + RGB
          (0x40, 0xD0),  // COM15: Full range RGB565
          (0x8C, 0x00),  // RGB444: Disable

          // Clock settings
          (0x11, 0x01),  // CLKRC: Internal clock
          (0x6B, 0x0A),  // DBLV: PLL control

          // Timing and format
          (0x3A, 0x04),  // TSLB: Set output sequence
          (0x14, 0x38),  // COM9: AGC ceiling
          (0x0C, 0x08),  // COM3: Enable scaling

          // Color matrix coefficients (example values)
          (0x4F, 0xBB),
          (0x50, 0x9C),
          (0x51, 0x2D),
          (0x52, 0x00),
          (0x53, 0x00),
          (0x54, 0x00),

          // AWB and AEC
          (0x13, 0xFF),  // COM8: Enable AWB, AEC
          (0x01, 0x80),  // BLUE: Blue gain
          (0x02, 0x80),  // RED: Red gain
      ];

      for (reg, val) in init_sequence.iter() {
          if i2c.write(addr, &[*reg, *val]).is_err() {
              info!("❗ Failed to write reg {:02X}", reg);
              return Err(());
          }
          Timer::after_millis(10).await;
      }

      Ok(())
  }
  */

/*
// -------PROJECT AUTOMATED BARRIER------------
// -------MADE BY MITRAN ANDREI----------------
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Output, Pull, Level},
    pwm::{Config as PwmConfig, Pwm},
    init,
};
use embassy_time::Timer;
use fixed::traits::ToFixed;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let peripherals = init(Default::default());

    //servo motor config
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = 0xB71A;
    servo_config.divider = 64_i32.to_fixed();
    const PERIOD_US: usize = 20_000;
    const BARRIER_DOWN_US: usize = 500; //0 deg
    const BARRIER_UP_US: usize = 1500; //90 deg

    let barrier_up = (BARRIER_DOWN_US * servo_config.top as usize) / PERIOD_US;
    let barrier_down = (BARRIER_UP_US * servo_config.top as usize) / PERIOD_US;

    //pwm
    let mut servo = Pwm::new_output_a(
        peripherals.PWM_SLICE6,
        peripherals.PIN_28,
        servo_config.clone()
    );

    let ir_sensor = Input::new(peripherals.PIN_27, Pull::Up);
    let mut green_led = Output::new(peripherals.PIN_26, Level::Low);

//stagiul initial al barierei
    servo_config.compare_a = barrier_down as u16;
    servo.set_config(&servo_config);
    defmt::info!("JavaKet's Barrier system - Barrier closed.");

//switch in order to allow or deny
    let mut allow_access = true;
    loop {
        if allow_access {
        if ir_sensor.is_low() {
            if servo_config.compare_a != barrier_down as u16 {
                servo_config.compare_a = barrier_down as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier closing (DOWN position)");
            green_led.set_high();
            }
        } else {
            if servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier opening (UP position)");
                green_led.set_low();
            }
        }
    }
    else   {
        green_led.set_low();
        if ir_sensor.is_low() {
            if servo_config.compare_a != barrier_down as u16 {
                servo.set_config(&servo_config); //no change ca sa nu se deschida
                defmt::info!("Acces Denied");

            }
        } else {

            if servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16; // daca era deschis sa isi revina
                servo.set_config(&servo_config);
                defmt::info!("Barrier closing (DOWN position)");
             }
        }
    }
        Timer::after_millis(100).await;
    }
}*/
// dimensiuni h*w*3
// spi header
// jpeg header
// store in an array
//
/*
 #![no_std]
#![no_main]
 use defmt::*;
  use embassy_executor::Spawner;
  use embassy_rp::gpio::{Input, Level, Output, Pin, Pull, AnyPin};
  use embassy_rp::peripherals::*;
  use embassy_time::{Delay, Timer};
  use embedded_hal_async::delay::DelayNs;
  use static_cell::StaticCell;
  use {defmt_rtt as _, panic_probe as _};
  use core::cell::RefCell;
  use embassy_rp::bind_interrupts;
  use embedded_hal_1::i2c::I2c as HalI2c;
  use embassy_rp::i2c::{self, I2c, InterruptHandler as I2CInterruptHandler, Config as I2cConfig};
  use embedded_hal_async::i2c::{Error, I2c as _};
  use embassy_rp::peripherals::I2C0;

  use embassy_rp::spi::{self, Spi, Config as SpiConfig, InterruptHandler as SPIInterruptHandler};

  const WIDTH: usize = 1632;
  const HEIGHT: usize = 1232;
  const PIXEL_BYTES: usize = 3;
  const FRAMEBUFFER_SIZE: usize = WIDTH * HEIGHT * PIXEL_BYTES;
  static FRAMEBUFFER: StaticCell<[u8; FRAMEBUFFER_SIZE]> = StaticCell::new();// dimensiuni h*w*3

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
      info!("Initializing peripherals...");
      let p = embassy_rp::init(Default::default());
      let mut delay = Delay;
      let framebuffer = FRAMEBUFFER.init([0u8; FRAMEBUFFER_SIZE]);
      let mut config = spi::Config::default();
      config.frequency = 1_000_000;
      config.phase = spi::Phase::CaptureOnFirstTransition;
      config.polarity = spi::Polarity::IdleLow;


      let miso = p.PIN_X; //must fix
      let mosi = p.PIN_Y;
      let clk = p.PIN_Z;

      let mut spi = Spi::new(p.SPI0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, config);
      let mut cs = Output::new(p.PIN_N, Level::High);
    loop {

    }
}

*//*

//

*//* 
// -------PROJECT AUTOMATED BARRIER------------
// -------MADE BY MITRAN ANDREI----------------
#![no_std]
#![no_main]

mod irqs; // <-- must exist as irqs.rs and match bind_interrupts! macro

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Output, Pull, Level},
    pwm::{Config as PwmConfig, Pwm},
    init,
};
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use core::sync::atomic::{AtomicBool, Ordering};

// Networking
use embassy_net::{
    tcp::TcpSocket, Config as NetConfig, IpAddress, IpEndpoint,
    StaticConfigV4, StackResources,
};
use embassy_lab_utils::{init_wifi, init_network_stack};
use cyw43::JoinOptions;
use static_cell::StaticCell;

// Change these if you use different pins or need different address/port
const PICO_IP:   (u8,u8,u8,u8) = (198,162,0,4);
const PICO_PORT: u16           = 1989;

static ALLOW_ACCESS: AtomicBool = AtomicBool::new(true);
static CONTROL_RES: StaticCell<StackResources<1>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
  let (net_device, mut wifi_ctrl) = init_wifi!(&spawner, p).await;

const SSID: &str = "nambani";
const PASS: &str = "andy1234";
loop {
    match wifi_ctrl.join(SSID, JoinOptions::new(PASS.as_bytes())).await {
        Ok(_) => { info!("✔ joined {}", SSID); break; }
        Err(_) => {
            warn!("Wi-Fi join failed; retrying in 2s");
            Timer::after(Duration::from_secs(2)).await;
        }
    }
}

  
    let net_cfg = NetConfig::dhcpv4(Default::default());
    let stack   = init_network_stack(&spawner, net_device, &CONTROL_RES, net_cfg);
    info!("waiting for DHCP…");
    while !stack.is_config_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    if let Some(StaticConfigV4 { address: cidr, .. }) = stack.config_v4() {
        info!("IP = {}", cidr.address());
    }

    // --- 4. Spawn control task ---
    let pico_ep = IpEndpoint::new(
        IpAddress::v4(PICO_IP.0, PICO_IP.1, PICO_IP.2, PICO_IP.3),
        PICO_PORT,
    );
    static CTRL_RX: StaticCell<[u8; 32]> = StaticCell::new();
    static CTRL_TX: StaticCell<[u8; 32]> = StaticCell::new();
    let rx = CTRL_RX.init([0u8; 32]);
    let tx = CTRL_TX.init([0u8; 32]);
    let ctrl_socket = TcpSocket::new(stack, rx, tx);
    spawner.spawn(control_task(ctrl_socket, pico_ep)).unwrap();

    // --- 5. Hardware setup ---
    let mut servo_cfg: PwmConfig = Default::default();
    servo_cfg.top     = 0xB71A;
    servo_cfg.divider = 64_i32.to_fixed();
    const PERIOD_US: usize       = 20_000;
    const BARRIER_DOWN_US: usize = 500;   // 0°
    const BARRIER_UP_US:   usize = 1500;  // 90°

    let barrier_up   = (BARRIER_UP_US   * servo_cfg.top as usize) / PERIOD_US;
    let barrier_down = (BARRIER_DOWN_US * servo_cfg.top as usize) / PERIOD_US;

    // Pin assignments: adjust for your board!
    let mut servo = Pwm::new_output_a(
        p.PWM_SLICE6,   // <-- Adjust if using a different slice
        p.PIN_28,       // <-- Servo pin
        servo_cfg.clone(),
    );
    let ir_sensor = Input::new(p.PIN_27, Pull::Up);
    let mut green_led = Output::new(p.PIN_26, Level::Low);

    // Start with barrier closed
    servo_cfg.compare_a = barrier_down as u16;
    servo.set_config(&servo_cfg);
    info!("Barrier closed (INIT)");

    // --- 6. Main control loop ---
    loop {
        let allow = ALLOW_ACCESS.load(Ordering::Relaxed);
        if allow {
            if ir_sensor.is_low() {
                // object detected → close
                if servo_cfg.compare_a != barrier_down as u16 {
                    servo_cfg.compare_a = barrier_down as u16;
                    servo.set_config(&servo_cfg);
                    info!("Barrier closing (DOWN)");
                    green_led.set_high();
                }
            } else {
                // no object → open
                if servo_cfg.compare_a != barrier_up as u16 {
                    servo_cfg.compare_a = barrier_up as u16;
                    servo.set_config(&servo_cfg);
                    info!("Barrier opening (UP)");
                    green_led.set_low();
                }
            }
        } else {
            // access denied mode
            green_led.set_low();
            info!("ACCESS DENIED — IR input ignored");
            // Optional: force barrier closed?
            if servo_cfg.compare_a != barrier_down as u16 {
                servo_cfg.compare_a = barrier_down as u16;
                servo.set_config(&servo_cfg);
                info!("Barrier forced closed (DENY)");
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Control‐task: connects to server and watches for "ALLOW"/"DENY"
#[embassy_executor::task]
async fn control_task(
    mut socket: TcpSocket<'static>, 
    server: IpEndpoint
) {
    loop {
        // reconnect loop
        if socket.connect(server).await.is_err() {
            warn!("Control socket connect failed; retrying in 5s…");
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        info!("Control socket connected");

        let mut buf = [0u8; 32];
        loop {
            match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("Control socket closed by remote");
                    break;
                }
                Ok(n) => {
                    let line = core::str::from_utf8(&buf[..n])
                        .unwrap_or("")
                        .trim();
                    match line {
                        "ALLOW" => {
                            ALLOW_ACCESS.store(true, Ordering::Relaxed);
                            info!("Control → ALLOW");
                        }
                        "DENY" => {
                            ALLOW_ACCESS.store(false, Ordering::Relaxed);
                            info!("Control → DENY");
                        }
                        other => warn!("Unknown control msg `{}`", other),
                    }
                }
                Err(e) => {
                    warn!("Control read err {:?}, reconnecting…", e);
                    break;
                }
            }
        }
    }
}
*/
/* 
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_net::{Config, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use heapless::Vec;
use {defmt_rtt as _, panic_probe as _};

mod irqs;
use cyw43::JoinOptions;

// --- WiFi config ---
const SSID: &str = "nambani";
const PASS: &[u8] = b"andy1234";

const SOCKS: usize = 4;
static RESOURCES: StaticCell<StackResources<SOCKS>> = StaticCell::new();

// --- WebSocket server info (change as needed) ---
const WS_SERVER_IP: [u8; 4] = [192, 168, 94, 255]; // your laptop/server IP
const WS_SERVER_PORT: u16 = 8080;
const WS_SERVER_PATH: &str = "/"; // path part of ws://ip:port/

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let (net_device, mut control) = embassy_lab_utils::init_wifi!(&_spawner, p).await;
    let config = Config::dhcpv4(Default::default());
    let stack = embassy_lab_utils::init_network_stack(&_spawner, net_device, &RESOURCES, config);

    // --- Connect to WiFi ---
    loop {
        if control.join(SSID, JoinOptions::new(PASS)).await.is_ok() {
            info!("Joined WiFi '{}'", SSID);
            break;
        }
        warn!("Join failed, retrying...");
        Timer::after(Duration::from_secs(2)).await;
    }
    stack.wait_config_up().await;
    if let Some(cfg) = stack.config_v4() {
        let ip = cfg.address.address();
        info!("Assigned IP: {}", ip);
    } else {
        warn!("No IPv4 config!");
    }

    // --- Connect to WebSocket server via TCP ---
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    let ws_endpoint = embassy_net::IpEndpoint::new(
        embassy_net::IpAddress::v4(WS_SERVER_IP[0], WS_SERVER_IP[1], WS_SERVER_IP[2], WS_SERVER_IP[3]),
        WS_SERVER_PORT,
    );
    info!("Connecting to WebSocket server...");
    if let Err(e) = socket.connect(ws_endpoint).await {
        warn!("TCP connect failed");
        loop { Timer::after(Duration::from_secs(10)).await; }
    }
    info!("Connected to WebSocket server!");

    // --- WebSocket handshake ---
    let ws_key = "dGhlIHNhbXBsZSBub25jZQ=="; // "the sample nonce" base64 (static, for demo)
    let handshake = {
        // You can use heapless::String if you want this to be dynamic!
        let mut buf: heapless::String<256> = heapless::String::new();
        use core::fmt::Write;
       let _ = core::fmt::Write::write_fmt(&mut buf, format_args!(         
            "GET {} HTTP/1.1\r\n\
             Host: {}.{}.{}.{}:{}\r\n\
             Upgrade: websocket\r\n\
             Connection: Upgrade\r\n\
             Sec-WebSocket-Key: {}\r\n\
             Sec-WebSocket-Version: 13\r\n\
             \r\n",
            WS_SERVER_PATH,
            WS_SERVER_IP[0], WS_SERVER_IP[1], WS_SERVER_IP[2], WS_SERVER_IP[3], WS_SERVER_PORT,
            ws_key ));
    
        
        buf
    };
    let _ = socket.write(handshake.as_bytes()).await;
    let _ = socket.flush().await;

    // --- Read handshake response ---
    let mut resp_buf = [0u8; 256];
    let n = match socket.read(&mut resp_buf).await {
        Ok(n) if n > 0 => n,
        _ => {
            warn!("Handshake read failed");
            loop { Timer::after(Duration::from_secs(10)).await; }
        }
    };
    info!("Handshake response: {}", defmt::Debug2Format(&core::str::from_utf8(&resp_buf[..n])));

    // --- Send WebSocket text frame ("Hello from Pico") ---
    let msg = b"Hello from Pico";
    // WebSocket text frame: 0x81 [len] [payload]
    let mut frame = [0u8; 32];
    frame[0] = 0x81; // FIN=1, opcode=1 (text)
    frame[1] = msg.len() as u8; // (no mask, small payload)
    frame[2..2+msg.len()].copy_from_slice(msg);

    let _ = socket.write(&frame[..2+msg.len()]).await;
    let _ = socket.flush().await;
    info!("Sent WebSocket message!");

    // --- Read server reply frame (echo or other) ---
    let mut recv_buf = [0u8; 32];
    let n = match socket.read(&mut recv_buf).await {
        Ok(n) if n > 0 => n,
        _ => {
            warn!("No reply from server");
            loop { Timer::after(Duration::from_secs(10)).await; }
        }
    };
    // Frame parsing: skip first two bytes for short frames
    if n > 2 {
        info!("Got WebSocket frame: {}", defmt::Debug2Format(&core::str::from_utf8(&recv_buf[2..n])));
    } else {
        warn!("Short WebSocket frame received");
    }

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}*//*
// -------PROJECT AUTOMATED BARRIER------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// -------MADE BY MITRAN ANDREI----------------
#![no_std]
#![no_main]

mod irqs; 

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Output, Pull, Level},
    pwm::{Config as PwmConfig, Pwm},
    init,
};
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use core::sync::atomic::{AtomicBool, Ordering};

// networking
use embassy_net::{
    tcp::TcpSocket, Config as NetConfig, IpAddress, IpEndpoint,
    StaticConfigV4, StackResources,
};
use embassy_lab_utils::{init_wifi, init_network_stack};
use cyw43::JoinOptions;
use static_cell::StaticCell;


const PICO_IP:   (u8,u8,u8,u8) = (198,162,7,255);
const PICO_PORT: u16           = 1989;

static ALLOW_ACCESS: AtomicBool = AtomicBool::new(true);
static CONTROL_RES: StaticCell<StackResources<1>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
  let (net_device, mut wifi_ctrl) = init_wifi!(&spawner, p).await;

const SSID: &str = "nambani";
const PASS: &str = "andy1234";
loop {
    match wifi_ctrl.join(SSID, JoinOptions::new(PASS.as_bytes())).await {
        Ok(_) => { info!("✔ joined {}", SSID); break; }
        Err(_) => {
            warn!("Wi-Fi join failed; retrying in 2s");
            Timer::after(Duration::from_secs(2)).await;
        }
    }
}

  
    let net_cfg = NetConfig::dhcpv4(Default::default());
    let stack   = init_network_stack(&spawner, net_device, &CONTROL_RES, net_cfg);
    info!("waiting for DHCP…");
    while !stack.is_config_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    if let Some(StaticConfigV4 { address: cidr, .. }) = stack.config_v4() {
        info!("IP = {}", cidr.address());
    }
    // spawn control task
    let pico_ep = IpEndpoint::new(
        IpAddress::v4(PICO_IP.0, PICO_IP.1, PICO_IP.2, PICO_IP.3),
        PICO_PORT,
    );
    static CTRL_RX: StaticCell<[u8; 32]> = StaticCell::new();
    static CTRL_TX: StaticCell<[u8; 32]> = StaticCell::new();
    let rx = CTRL_RX.init([0u8; 32]);
    let tx = CTRL_TX.init([0u8; 32]);
    let ctrl_socket = TcpSocket::new(stack, rx, tx);
    spawner.spawn(control_task(ctrl_socket, pico_ep)).unwrap();

    let mut servo_cfg: PwmConfig = Default::default();
    servo_cfg.top     = 0xB71A;
    servo_cfg.divider = 64_i32.to_fixed();
    const PERIOD_US: usize       = 20_000;
    const BARRIER_DOWN_US: usize = 500;   // 0DEG
    const BARRIER_UP_US:   usize = 1500;  // 90DEG

    let barrier_up   = (BARRIER_UP_US   * servo_cfg.top as usize) / PERIOD_US;
    let barrier_down = (BARRIER_DOWN_US * servo_cfg.top as usize) / PERIOD_US;

   
    let mut servo = Pwm::new_output_a(
        p.PWM_SLICE6,   
        p.PIN_28,     
        servo_cfg.clone(),
    );
    let ir_sensor = Input::new(p.PIN_27, Pull::Up);
    let mut green_led = Output::new(p.PIN_26, Level::Low);

    servo_cfg.compare_a = barrier_down as u16;
    servo.set_config(&servo_cfg);
    info!("Barrier closed (INIT)");

    loop {
        let allow = ALLOW_ACCESS.load(Ordering::Relaxed);
        if allow {
            if ir_sensor.is_low() {
                if servo_cfg.compare_a != barrier_down as u16 {
                    servo_cfg.compare_a = barrier_down as u16;
                    servo.set_config(&servo_cfg);
                    info!("Barrier closing (DOWN)");
                    green_led.set_high();
                }
            } else {
                if servo_cfg.compare_a != barrier_up as u16 {
                    servo_cfg.compare_a = barrier_up as u16;
                    servo.set_config(&servo_cfg);
                    info!("Barrier opening (UP)");
                    green_led.set_low();
                }
            }
        } else {
            green_led.set_low();
            info!("ACCESS DENIED — IR input ignored");
            if servo_cfg.compare_a != barrier_down as u16 {
                servo_cfg.compare_a = barrier_down as u16;
                servo.set_config(&servo_cfg);
                info!("Barrier forced closed (DENY)");
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn control_task(
    mut socket: TcpSocket<'static>, 
    server: IpEndpoint
) {
    loop {
        // reconnect loop
        if socket.connect(server).await.is_err() {
            warn!("Control socket connect failed; retrying in 5s…");
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        info!("Control socket connected");

        let mut buf = [0u8; 32];
        loop {
            match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("Control socket closed by remote");
                    break;
                }
                Ok(n) => {
                    let line = core::str::from_utf8(&buf[..n])
                        .unwrap_or("")
                        .trim();
                    match line {
                        "ALLOW" => {
                            ALLOW_ACCESS.store(true, Ordering::Relaxed);
                            info!("Control → ALLOW");
                        }
                        "DENY" => {
                            ALLOW_ACCESS.store(false, Ordering::Relaxed);
                            info!("Control → DENY");
                        }
                        other => warn!("Unknown control msg `{}`", other),
                    }
                }
                Err(e) => {
                    warn!("Control read err {:?}, reconnecting…", e);
                    break;
                }
            }
        }
    }
}*/
/* 
// -------PROJECT AUTOMATED BARRIER------------
// -------MADE BY MITRAN ANDREI----------------
//! main.rs
#![no_std]
#![no_main]

use core::str;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Ipv4Cidr, StackResources};
use embassy_time::{Duration, Timer};
use embassy_lab_utils::{init_wifi, init_network_stack};
use static_cell::StaticCell;
use heapless::Vec;
use embassy_rp::{
    init,
    gpio::{Input, Output, Pull, Level},
    pwm::{Config as PwmConfig, Pwm},
    peripherals::{PWM_SLICE6, PIN_16, PIN_5, PIN_15},
};
use fixed::traits::ToFixed;
mod irqs;

const SOCK_COUNT: usize = 4;
static NET_RESOURCES: StaticCell<StackResources<SOCK_COUNT>> = StaticCell::new();
static ALLOW_ACCESS: AtomicBool = AtomicBool::new(true);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    
    let mut p = init(Default::default());
    // 2) Extract only the four items for the barrier
    let pwm_slice = p.PWM_SLICE6;
    let servo_pin = p.PIN_28;
    let ir_pin    = p.PIN_27;
    let led_pin   = p.PIN_26;
    // 3) Hand the rest of `p` into the Wi-Fi initializer
    let (net_device, mut wifi_control) = init_wifi!(&_spawner, p).await;
    // 4) Configure static IP and network stack
    let config = NetConfig::ipv4_static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(embassy_net::Ipv4Address::new(192,168,4,1), 24),
        dns_servers: Vec::new(),
        gateway: None,
    });
    let stack = init_network_stack(&_spawner, net_device, &NET_RESOURCES, config);
    // 5) Start WPA2 Access Point
    wifi_control
        .start_ap_wpa2("BarrierNet", "embassyRP", 6)
        .await;
    defmt::info!("Wi-Fi AP started: SSID=BarrierNet");
    // 6) Spawn a TCP server task to toggle ALLOW_ACCESS
    _spawner.spawn(tcp_server(stack)).unwrap();
    // 7) Run the barrier loop with the extracted peripherals
    run_barrier_loop(pwm_slice, servo_pin, ir_pin, led_pin).await;
}

#[embassy_executor::task]
async fn tcp_server(stack: embassy_net::Stack<'static>) {
    let mut rx = [0u8; 128];
    let mut tx = [0u8; 128];
    let mut sock = TcpSocket::new(stack, &mut rx, &mut tx);
    // Use embassy_time::Duration for the timeout
    sock.set_timeout(Some(Duration::from_secs(10)));

    loop {
      info!("socket:3000.");
      if let Err(e) = sock.accept(3000).await {
        warn!("accept error: {:?}", e);
        return;
    }
        defmt::info!("TCP client connected");
        let mut buf = [0u8; 128];
        while let Ok(n) = sock.read(&mut buf).await {
            if n == 0 { break; }
            if let Ok(cmd) = str::from_utf8(&buf[..n]) {
                match cmd.trim() {
                    "ALLOW" => {
                        ALLOW_ACCESS.store(true, Ordering::Relaxed);
                        let _ = sock.write(b"OK: ALLOW\n").await;
                    }
                    "DENY" => {
                        ALLOW_ACCESS.store(false, Ordering::Relaxed);
                        let _ = sock.write(b"OK: DENY\n").await;
                    }
                    _ => {
                        let _ = sock.write(b"ERR: Unknown command\n").await;
                    }
                }
            }
        }
        let _ = sock.close();
    }
}

async fn run_barrier_loop(
    pwm_slice: embassy_rp::peripherals::PWM_SLICE6,
    servo_pin: embassy_rp::peripherals::PIN_28,
    ir_pin: embassy_rp::peripherals::PIN_27,
    led_pin: embassy_rp::peripherals::PIN_26,
) {
    // Servo configuration
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = 0xB71A;
    servo_config.divider = 64_i32.to_fixed();
    const PERIOD_US: usize        = 20_000;
    const BARRIER_DOWN_US: usize  = 500;  // 0°
    const BARRIER_UP_US: usize    = 1500; // 90°

    let barrier_up   = (BARRIER_DOWN_US * servo_config.top as usize) / PERIOD_US;
    let barrier_down = (BARRIER_UP_US   * servo_config.top as usize) / PERIOD_US;

    // Initialize PWM, sensor, and LED
    let mut servo     = Pwm::new_output_a(pwm_slice, servo_pin, servo_config.clone());
    let ir_sensor     = Input::new(ir_pin, Pull::Up);
    let mut green_led = Output::new(led_pin, Level::Low);

    // Start with barrier closed
    servo_config.compare_a = barrier_down as u16;
    servo.set_config(&servo_config);
    defmt::info!("Barrier system: closed");

    loop {
        let sensor_low = ir_sensor.is_low();
        let allowed    = ALLOW_ACCESS.load(Ordering::Relaxed);

        if allowed {
            if sensor_low && servo_config.compare_a != barrier_down as u16 {
                servo_config.compare_a = barrier_down as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier closing (DOWN)");
                green_led.set_high();
            } else if !sensor_low && servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier opening (UP)");
                green_led.set_low();
            }
        } else {
            // DENY: always stay closed
            if servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16;
                servo.set_config(&servo_config);
            }
            defmt::info!("Access Denied: barrier remains closed");
            green_led.set_low();
        }

        // Use embassy_time::Timer with embassy_time::Duration
        Timer::after(Duration::from_millis(100)).await;
    }
}    
*/



/* 
// -------PROJECT AUTOMATED BARRIER------------
// -------MADE BY MITRAN ANDREI----------------

#![no_std]
#![no_main]

use core::str;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{info, warn};
use defmt_rtt as _;
use panic_probe as _;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Ipv4Cidr, StackResources};
use embassy_time::{Duration, Timer};
use embassy_lab_utils::{init_wifi, init_network_stack};
use static_cell::StaticCell;
use heapless::Vec;
use embassy_rp::{
    init,
    gpio::{Input, Output, Pull, Level},
    pwm::{Config as PwmConfig, Pwm},
    peripherals::{PWM_SLICE6, PIN_28, PIN_27, PIN_26},
};
use fixed::traits::ToFixed;
mod irqs;

const SOCK_COUNT: usize = 4;
static NET_RESOURCES: StaticCell<StackResources<SOCK_COUNT>> = StaticCell::new();
static ALLOW_ACCESS: AtomicBool = AtomicBool::new(true);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    
    let p = init(Default::default());
    
    let pwm_slice = p.PWM_SLICE6;
    let servo_pin = p.PIN_28;
    let ir_pin    = p.PIN_27;
    let led_pin   = p.PIN_26;
    
    let (net_device, mut wifi_control) = init_wifi!(&_spawner, p).await;
    
    let config = NetConfig::ipv4_static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(embassy_net::Ipv4Address::new(192,168,94,255), 24),
        dns_servers: Vec::new(),
        gateway: None,
    });
    let stack = init_network_stack(&_spawner, net_device, &NET_RESOURCES, config);
    
    wifi_control
        .start_ap_wpa2("BarrierNet", "embassyRP", 6)
        .await;
    defmt::info!("Wi-Fi AP started: SSID=BarrierNet");
    //allow acces
        _spawner.spawn(tcp_server(stack)).unwrap();
    
    run_barrier_loop(pwm_slice, servo_pin, ir_pin, led_pin).await;
}

#[embassy_executor::task]
async fn tcp_server(stack: embassy_net::Stack<'static>) {
    let mut rx = [0u8; 128];
    let mut tx = [0u8; 128];
    let mut sock = TcpSocket::new(stack, &mut rx, &mut tx);
    sock.set_timeout(Some(Duration::from_secs(10)));

    loop {
      info!("socket:3000.");
      if let Err(e) = sock.accept(3000).await {
        warn!("accept error: {:?}", e);
        return;
    }
        defmt::info!("TCP client connected");
        let mut buf = [0u8; 128];
        while let Ok(n) = sock.read(&mut buf).await {
            if n == 0 { break; }
            if let Ok(cmd) = str::from_utf8(&buf[..n]) {
                match cmd.trim() {
                    "ALLOW" => {
                        ALLOW_ACCESS.store(true, Ordering::Relaxed);
                        let _ = sock.write(b"OK: ALLOW\n").await;
                    }
                    "DENY" => {
                        ALLOW_ACCESS.store(false, Ordering::Relaxed);
                        let _ = sock.write(b"OK: DENY\n").await;
                    }
                    _ => {
                        let _ = sock.write(b"ERR: Unknown command\n").await;
                    }
                }
            }
        }
        let _ = sock.close();
    }
}

async fn run_barrier_loop(
    pwm_slice: embassy_rp::peripherals::PWM_SLICE6,
    servo_pin: embassy_rp::peripherals::PIN_28,
    ir_pin: embassy_rp::peripherals::PIN_27,
    led_pin: embassy_rp::peripherals::PIN_26,
) {
    // servo
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = 0xB71A;
    servo_config.divider = 64_i32.to_fixed();
    const PERIOD_US: usize        = 20_000;
    const BARRIER_DOWN_US: usize  = 500;  // 0deg
    const BARRIER_UP_US: usize    = 1500; // 90deg

    let barrier_up   = (BARRIER_DOWN_US * servo_config.top as usize) / PERIOD_US;
    let barrier_down = (BARRIER_UP_US   * servo_config.top as usize) / PERIOD_US;

    let mut servo     = Pwm::new_output_a(pwm_slice, servo_pin, servo_config.clone());
    let ir_sensor     = Input::new(ir_pin, Pull::Up);
    let mut green_led = Output::new(led_pin, Level::Low);

//init
    servo_config.compare_a = barrier_down as u16;
    servo.set_config(&servo_config);
    defmt::info!("Barrier system: closed");

    loop {
        let sensor_low = ir_sensor.is_low();
        let allowed    = ALLOW_ACCESS.load(Ordering::Relaxed);

        if allowed {
            if sensor_low && servo_config.compare_a != barrier_down as u16 {
                servo_config.compare_a = barrier_down as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier closing (DOWN)");
                green_led.set_high();
            } else if !sensor_low && servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier opening (UP)");
                green_led.set_low();
            }
        } else {
         
            if servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16;
                servo.set_config(&servo_config);
            }
            defmt::info!("Access Denied: barrier remains closed");
            green_led.set_low();
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}    
*/
/* // -------PROJECT AUTOMATED BARRIER------------
// -------MADE BY MITRAN ANDREI----------------
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Output, Pull, Level},
    pwm::{Config as PwmConfig, Pwm},
    init,
};
use embassy_time::Timer;
use fixed::traits::ToFixed;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let peripherals = init(Default::default());

    //servo motor config
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = 0xB71A;
    servo_config.divider = 64_i32.to_fixed();
    const PERIOD_US: usize = 20_000;
    const BARRIER_DOWN_US: usize = 500; //0 deg
    const BARRIER_UP_US: usize = 1500; //90 deg

    let barrier_up = (BARRIER_DOWN_US * servo_config.top as usize) / PERIOD_US;
    let barrier_down = (BARRIER_UP_US * servo_config.top as usize) / PERIOD_US;

    //pwm
    let mut servo = Pwm::new_output_a(
        peripherals.PWM_SLICE6,
        peripherals.PIN_28,
        servo_config.clone()
    );

    let ir_sensor = Input::new(peripherals.PIN_27, Pull::Up);
    let mut green_led = Output::new(peripherals.PIN_26, Level::Low);

//stagiul initial al barierei
    servo_config.compare_a = barrier_down as u16;
    servo.set_config(&servo_config);
    defmt::info!("JavaKet's Barrier system - Barrier closed.");

//switch in order to allow or deny
    let mut allow_access = true;
    loop {
        if allow_access {
        if ir_sensor.is_low() {
            if servo_config.compare_a != barrier_down as u16 {
                servo_config.compare_a = barrier_down as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier closing (DOWN position)");
            green_led.set_high();
            }
        } else {
            if servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16;
                servo.set_config(&servo_config);
                defmt::info!("Barrier opening (UP position)");
                green_led.set_low();
            }
        }
    }
    else   {
        green_led.set_low();
        if ir_sensor.is_low() {
            if servo_config.compare_a != barrier_down as u16 {
                servo.set_config(&servo_config); //no change ca sa nu se deschida
                defmt::info!("Acces Denied");

            }
        } else {

            if servo_config.compare_a != barrier_up as u16 {
                servo_config.compare_a = barrier_up as u16; // daca era deschis sa isi revina
                servo.set_config(&servo_config);
                defmt::info!("Barrier closing (DOWN position)");
             }
        }
    }
        Timer::after_millis(100).await;
    }
}*/


 #![no_std]
#![no_main]
 use defmt::*;
  use embassy_executor::Spawner;
  use embassy_rp::gpio::{Input, Level, Output, Pin, Pull, AnyPin};
  use embassy_rp::peripherals::*;
  use embassy_time::{Delay, Timer};
  use embedded_hal_async::delay::DelayNs;
  use static_cell::StaticCell;
  use {defmt_rtt as _, panic_probe as _};
  use core::cell::RefCell;
  use embassy_rp::bind_interrupts;
  use embedded_hal_1::i2c::I2c as HalI2c;
  use embassy_rp::i2c::{self, I2c, InterruptHandler as I2CInterruptHandler, Config as I2cConfig};
  use embedded_hal_async::i2c::{Error, I2c as _};
  use embassy_rp::peripherals::I2C0;

  use embassy_rp::spi::{self, Spi, Config as SpiConfig, InterruptHandler as SPIInterruptHandler};

  const WIDTH: usize = 1632;
  const HEIGHT: usize = 1232;
  const PIXEL_BYTES: usize = 3;
  const FRAMEBUFFER_SIZE: usize = WIDTH * HEIGHT * PIXEL_BYTES;
  static FRAMEBUFFER: StaticCell<[u8; FRAMEBUFFER_SIZE]> = StaticCell::new();// dimensiuni h*w*3

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
      info!("Initializing peripherals...");
      let p = embassy_rp::init(Default::default());
      let mut delay = Delay;
      let framebuffer = FRAMEBUFFER.init([0u8; FRAMEBUFFER_SIZE]);
      let mut config = spi::Config::default();
      config.frequency = 1_000_000;
      config.phase = spi::Phase::CaptureOnFirstTransition;
      config.polarity = spi::Polarity::IdleLow;


      let miso = p.PIN_X; //must fix
      let mosi = p.PIN_Y;
      let clk = p.PIN_Z;

      let mut spi = Spi::new(p.SPI0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, config);
      let mut cs = Output::new(p.PIN_N, Level::High);
    loop {

    }
}
