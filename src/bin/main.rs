// Todos:
// [] turn display orientation according to inertia sensor

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    geometry::AnchorPoint,
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, StrokeAlignment},
    text::{Alignment, Text},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use panic_rtt_target as _;

use mipidsi::{
    Builder,
    interface::SpiInterface,
    models::ST7789,
    options::{ColorOrder, Orientation},
};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 0.6.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // init display (1.47inch capacitive touch LCD display, 172×320 resolution, 262K color with display chip Jadard JD9853 (compatible with ST7789? ) and touch chip AXS5106L )
    // LCD_CLK GPIO1
    // LCD_DIN GPIO2
    // LCD_CS GPIO14
    // LCD_DC GPIO15
    // LCD_RST GPIO22
    // LCD_BL GPIO23
    let mut lcd_bl = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let lcd_cs = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let lcd_dc = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default()); // level low according to example from https://github.com/almindor/mipidsi/blob/master/examples/spi-ili9486-esp32-c3/src/main.rs
    let mut lcd_rst = Output::new(peripherals.GPIO22, Level::Low, OutputConfig::default());
    lcd_rst.set_high(); // todo why not directly high? (taken from above example)

    // according to spec 16ns is min write cycle duration (<62.5MHz), lets try 40Mhz
    let display_spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    );
    if display_spi.is_err() {
        warn!(
            "Failed to initialize SPI2 for display with error {:?}",
            display_spi.err()
        );
        Timer::after(Duration::from_secs(2)).await;
        panic!("Failed to initialize SPI2 for display");
    }
    
    let display_spi = display_spi.unwrap();
    let display_spi = display_spi
        .with_sck(peripherals.GPIO1)
        .with_mosi(peripherals.GPIO2);

    let spi_device = ExclusiveDevice::new(display_spi, lcd_cs, embassy_time::Delay).unwrap();

    let mut buffer = [0_u8; 512];

    let di = SpiInterface::new(spi_device, lcd_dc, &mut buffer);

    let mut delay = Delay::new();
    let mut display = Builder::new(ST7789, di)
        .display_size(172, 320)
        .display_offset(34, 0)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::new().flip_horizontal()) // with this the display is higher than wide, upper left is origin, usb connecctor is on the lower side
        .reset_pin(lcd_rst)
        .init(&mut delay)
        .unwrap();
    info!(
        "Display initialized. Size: {}Wx{}H",
        display.size().width,
        display.size().height
    );
    lcd_bl.set_high(); // turn on backlight

    display.clear(Rgb565::BLACK).unwrap();
    info!("Display cleared");
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLUE)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    display
        .bounding_box()
        .into_styled(border_stroke)
        .draw(&mut display)
        .unwrap();

    let text = "Roonstr.23 - Hzg";
    let character_style = MonoTextStyle::new(&FONT_8X13 /*FONT_6X10*/, Rgb565::WHITE);
    Text::with_alignment(
        text,
        display.bounding_box().anchor_point(AnchorPoint::TopCenter) + Point::new(0, 15),
        character_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // TODO: Spawn some tasks
    let _ = spawner;

    let mut cnt = 0u32;
    let mut angle_bg = None;
    loop {
        //info!("Hello world #{}", cnt);
        Timer::after(Duration::from_millis(40)).await;
        cnt += 20; // 1 turn in 2s

        // progress alike circle/arc
        let angle_start = ((cnt % 360) as f32).deg();
        //let angle_end = angle_start + 30.0.deg();
        let angle_sweep = 100.0.deg();

        if let Some(angle_bg) = angle_bg {
            embedded_graphics::primitives::Arc::new(
                Point::new(25, 32),
                40,
                angle_bg,
                if angle_bg - angle_start < 0.0.deg() {
                    angle_bg - angle_start + 360.0.deg()
                } else {
                    angle_bg - angle_start
                },
            )
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 2))
            .draw(&mut display)
            .unwrap();
        }

        embedded_graphics::primitives::Arc::new(Point::new(25, 32), 40, angle_start, angle_sweep)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 2))
            .draw(&mut display)
            .unwrap();
        angle_bg = Some(angle_start);
    }
}
