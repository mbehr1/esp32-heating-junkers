/// Module for heating related functions
/// and task.
///
/// Controls a junkers heating system via CAN bus (TWAI).
///
use defmt::{debug, info, warn};
use embassy_time::{Duration, Timer};

pub fn calc_heating_needs() {}

#[embassy_executor::task]
pub async fn heating_task(twai: esp_hal::peripherals::TWAI0<'static>) {
    info!("task 'heating_task' running...");

    // init CAN interface

    loop {
        debug!("heating_task tick");

        Timer::after(Duration::from_secs(5)).await;
    }
    warn!("end heating_task");
}
