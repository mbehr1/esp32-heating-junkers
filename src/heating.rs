/// Module for heating related functions
/// and task.
///
/// Controls a junkers heating system via CAN bus (TWAI).
///
/// TODOs:
/// [x] - send mandatory frames periodically (0xf9, 0x258, 0x252, 0x253)
/// [] - ignore the regular/weekly avoid valve stuck movements
/// [] - ignore devices with valve position < 5%?
/// [] - compare outside temperature as well
/// [] - detect stuck/old devices with open valve and ignore them
/// [x] - consider HT being close to target temp and reduce heating need accordingly?
/// [] - disable CAN controller to allow CONFIG_PM_ENABLE / sleep modes to work? (e.g. if we dont expect to send anything for a while)
///
use core::{cell::RefCell, fmt::Write};
use defmt::{debug, info, warn};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant, Timer};
use embedded_can::{Frame, Id};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};
use esp_hal::{
    Blocking,
    twai::{self, StandardId},
};
use heapless::{String, Vec};
// use embedded_can::Frame;

use crate::homematic::DEVICES;

// CAN settings:
// we want a weird 10k baudrate
// need just TX (GPIO8) /RX (GPIO7) to the transceiver
// PINs:
// GPIO8 -> strapping pin chip boot mode und ROM message printing / joint download mode -> if GPIO9 is low at boot
// GPIO9 -> strapping pin chip boot mode -> can be used as output/TX -> no, 10k Pullup, and 100nF to GND -> filters too much?

// GPIO header pins:
// 1 SPI LCD CLK / SCK
// 2 SPI LDC DIN / MOSI
// 3 #SD MISO
// 4 #SD CARD CS
// 5 #IMU INT1 <- by default in HighZ
// 6 #IMU INT2 <- by default in HighZ

// +7 <- TWAI RX ## JTAG MTDO
// +8 <- TWAI TX

// 9 BOOT MODE PIN
// 12 USB_D-
// 13 USB_D+
// 16 ## UART0 TX
// 17 ## UART0 RX
// 18 I2C IMU SDA
// 19 I2C IMU SCL

// no need for BUS_OFF (we'll be always on)
// no need for clk_out

/// CAN identifiers for Junkers/Bosch heating system (BM1 based interface)
///
/// request data (to heating controller/Hzg):
const CF_REQUEST_STATUS: u16 = 0xF9; // request data, 0 data bytes

/// max. Vorlauftemperatur Hzg (send from heating controller)
const CF_HEAT_MAX_SUPPLY_TEMP: u16 = 0x200; // max allowed supply temp, 1 data byte (0.5 °C steps)
/// aktuelle Vorlauftemperatur Hzg
const CF_HEAT_CUR_SUPPLY_TEMP: u16 = 0x201; // current allowed supply temp, 1 data byte (0.5 °C steps)

/*
202H 1 Byte   max. Warmwassertemperatur in Halbgradschritten
203H 1 Byte   aktuelle Warmwassertemperatur in Halbgradschritten

Die weiter oben genannten 204 und 205 sind wohl für den Speicherbetrieb.
Meine ZWR-18-6 bereitet das WW im Durchlaufprinzip, in diesem Fall
werden dann dann die IDs 202 und 203 benutzt.
*/

/// max. Vorlauftemperatur Warmwasser (send from heating controller)
const CF_WW_MAX_SUPPLY_TEMP: u16 = 0x204; // max allowed supply temp, 1 data byte (0.5 °C steps)
/// aktuelle Vorlauftemperatur Warmwasser
const CF_WW_CUR_SUPPLY_TEMP: u16 = 0x205; // current allowed supply temp, 1 data byte (0.5 °C steps)

/// Störungsmeldungen Hex Wert siehe ( Bedienungsanleitung )  00=Heizung ok
const CF_ERROR_MESSAGES: u16 = 0x206; // error messages, 1 data byte, hex value see manual, 00=heating ok

/// aktuell Außentemperatur
// const CF_CUR_OUTSIDE_TEMP: u16 = 0x207; // current outside temp, 2 data bytes in 0.01 °C steps (signed), eg 0x07E2=2018=20,18°C

/// 0x208 -> unknown... (im Log nur 0x01)

/// aktueller Brennerstatus
const CF_CUR_BURNER_STATUS: u16 = 0x209; // current burner status, 1 data byte, 00: off, 01: on

/// aktueller Status Heizungspumpe
const CF_HEAT_CUR_PUMP_STATUS: u16 = 0x20A; // current pump status, 1 data byte, 00: off, 01: on

/// 20BH  1 Byte   Speicherladung  EIN / AUS ?

/// 20CH  1 Byte   Sommer / Winterbetrieb Wert=00 Sommer Wert 01=Winter

/// 0x20D 1 byte unknown...  max Heizleistung in KW?

/// 250H  1 Byte  Sollwert von TR: Heizbetrieb 00=Aus 01=Ein
/// 251H  1 Byte  Sollwert von TR: Heizleistung gleitend (00-FF)

/// Vorlauftemperatur Hzg Sollwert (we need to send it)
const CF_HEAT_TARGET_SUPPLY_TEMP: u16 = 0x252; // target value for supply temp, 1 data byte (0.5 °C steps)

// 253H  1 Byte  Sollwert von TA/TR: Speichersollwert in Halbgradschritten oder Pumpe AUS(1)/EIN(0)  oder Sparbetrieb ?
const CF_HEAT_PUMP_ECO_MODE: u16 = 0x253; // heating pump control, 1 data byte, 00=pump on, 01=pump off (economy mode)
// 254H  1 Byte  Sollwert von TR: WW-Sofort 00=Aus 01=Ein
// 255H  1 Byte  Sollwert von TR: WW-Sollwert in Halbgradschritten--> bei einer ZWx Therme mit WW-Bereitung im Durchlaufprinzip, im Normalfall identisch mit 202H

/// Vorlauftemperatur Warmwasser Sollwert (we need to send it)
///
/// 10°C -> treated as off
// const CF_WW_TARGET_SUPPLY_TEMP: u16 = 0x255; // target value for supply temp, 1 data byte (0.5 °C steps)

/// current time frame (we can send it periodically, but no need)
// const CF_CUR_TIME: u16 = 0x256; // current time frame, 4 data bytes DOW (N = ISO-8601 numeric representation of the day of the week. (1 = Monday, 7 = Sunday)) HH MM 4:  6 9 27 4 = Sat 09:27

const CF_TR_OUTSIDE_TEMP_CTRL: u16 = 0x258; // TR outside temperature control, 1 data byte, 00=room thermostat, 01=weather compensated control

/*
Snapshots from https://www.mikrocontroller.net/topic/81265?page=single

Die Ist-VLT sowie Brenner-An und Heizung/Pumpe-An
werden von meiner ZWR 18-6 genau alle 6 Sekunden übermittelt. Die
maximale VLT sowie die WW-Ist-Temperatur alle 30 Sekunden.

VLT Soll vom Regler kommt im Heizbetrieb alle 5 Sekunden, im
"Schlafmodus", also wenn die Heizleistung abgeschaltet ist, nur noch
alle 30 Sekunden.

Interessant ist, dass manchmal die Brenner und Pumpe an/aus Meldungen
(Can-ID 209 und 20A von der Heizung) sporadisch ausbleiben, während die
anderen genannten Werte weiter von der Therme gesendet werden.

Dies - und da es ja nur sporadisch auftritt kann ich nur orakeln -
meiner Beobachtung nach dann, wenn ebendieser Schalter am Regler NICHT
auf Info steht.

Schaue ich dann am TR ob der denn die entsprechende Info anzeigen kann
und drehe dazu den Schalter auf Info, dann sendet (wohl der Regler) eine
Nachricht mit ID 0F9 Wert 00 auf den Bus, direkt und reproduzierbar von
der Anforderung 250 (Heizung/Pumpe an/aus).

Genau ab dann kommen von der Heizung wieder die fehlenden 209/20A. So
als ob genau die Meldungen, welche wirklich nur rein zu Infozwecken für
den Benutzer übermittelt werden, durch den Info-Schalter aktiviert
werden.
...
Ich vermute, dass CAN-ID 0x258 anzeigt, ob es sich um einen
Raumthermostat oder eine witterungsgeführte Steuerung handelt. Send ich
[0x258, 1], so sendet die Therme die Außentemperatur (0x207), wirft dann
aber auch den Fehler, dass kein Temperaturfühler anschlossen ist. Mit
[0x258, 0] wird diese Fehlermeldung unterdrückt.
...
Der Regler schaltet auch die Pumpe aus, z.B. wenn die Heizung auf "Aus"
steht. In diesem Fall wird [0x253, 0x1] gesendet. Ist die Heizung in
Betrieb, so sendet der Regler [0x253, 0x0]. Der Befehl war zuvor hier
als Speichertemperatur dekodiert worden, was an meiner Durchlauf-Therme
natürlich nicht sinnvoll war und ich daher natürlich nicht versucht
hatte. Das Kommando 0x253 ist also eher ein "Interne Pumpe deaktivieren"
Kommando.
...
Nachtabsenkung klappt wie folgt:
- 0x252 auf beispielsweise 10° stellen
- 0x253 auf 1 stellen

https://github.com/Neuroquila-n8fall/JunkersControl/blob/master/src/configuration.cpp
sends the following frames:
- targets sending every frame each 30s, with 5s distance (min 1s dist)
  +- 0x253 "heating economy mode" (1=on / pump off,0=off / pump on)
  - 0x252 "heating target supply temp" (e.g. 100 -> 50°C)
  +- 0x258 "heating mode" (1=weather comp.,0=room thermostat)
  - 0x254 "hot water now" with fixed value 1
  - 0x255 (?) 0x203 "hot water setpointtemp" (????) (e.g. 100 -> 50°C)
  +- 0xf9 (request data, no data bytes)

  and each min:
   - 0x256 "current time" (DOW HH MM SS)
  */

/// state from the junkers heating controller
/// only data we did receive

#[derive(Debug, Default)]
struct HeaterState {
    cur_heat_supply_temp_celsius: f32,
    max_heat_supply_temp_celsius: f32,
    cur_ww_supply_temp_celsius: f32,
    max_ww_supply_temp_celsius: f32,
    //cur_outside_temp_celsius: f32,
    cur_burner_on: bool,
    cur_heat_pump_on: bool,

    err_msg: u8, // only the last one for now, 0x00 = ok
}

/// target control values to send to the heating controller
#[derive(Debug, Default, Clone)]
pub struct HeaterControl {
    target_supply_temp_raw: u8, // in 0.5°C steps
    heat_pump_eco_mode: u8,
}

impl HeaterControl {
    pub fn target_supply_temp_celsius(&self) -> f32 {
        (self.target_supply_temp_raw as f32) * 0.5
    }
}

// we use a BinarySortedHeap for the CAN TX queue.
// sort by next transmit time (earliest first).
// if we do update an existing entry, we remove and re-insert it with "now" as next transmit time to get fast update of changes.

const MIN_CAN_TX_DISTANCE: Duration = Duration::from_millis(1000);

#[derive(Debug)]
struct CanTxEntry {
    id: u16,
    data: heapless::Vec<u8, 4>, // max 4 bytes for our frames
    next_tx: embassy_time::Instant,
    cycle_time: Duration, // 0 (or even < MIN_CAN_TX_DISTANCE) = one-shot, else relative to last tx
}
// implement ordering for the BinarySortedHeap
impl PartialEq for CanTxEntry {
    fn eq(&self, other: &Self) -> bool {
        self.next_tx == other.next_tx
    }
}
impl Eq for CanTxEntry {}
impl PartialOrd for CanTxEntry {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.next_tx.cmp(&other.next_tx))
    }
}
impl Ord for CanTxEntry {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.next_tx.cmp(&other.next_tx)
    }
}

// data for display: text, text color, background color
pub static HEATING_TEXT_TO_DISPLAY: Mutex<
    CriticalSectionRawMutex,
    RefCell<(String<16>, Rgb565, Rgb565)>,
> = Mutex::new(RefCell::new((String::new(), Rgb565::GREEN, Rgb565::BLACK)));

pub static HEATER_CONTROL: Mutex<CriticalSectionRawMutex, RefCell<HeaterControl>> =
    Mutex::new(RefCell::new(HeaterControl {
        target_supply_temp_raw: 20, // 10°C as default
        heat_pump_eco_mode: 1,      // pump off as default
    }));

pub struct HeaterOverwrite {
    pub heating_need_perc: u16, // 0..100%
    pub active_till: Instant,
}
pub static HEATER_OVERWRITE: Mutex<CriticalSectionRawMutex, RefCell<HeaterOverwrite>> =
    Mutex::new(RefCell::new(HeaterOverwrite {
        heating_need_perc: 70,     // 70% as default
        active_till: Instant::MIN, // not active
    }));

#[embassy_executor::task]
pub async fn heating_task(can_config: esp_hal::twai::TwaiConfiguration<'static, Blocking>) {
    info!("task 'heating_task' running...");

    let mut tx_queue: heapless::BinaryHeap<CanTxEntry, heapless::binary_heap::Min, 8> =
        heapless::BinaryHeap::new();

    // when are we allowed to send the next frame?
    // we do wait some time between frames to not flood the bus/BM1
    // after each frame sent, we update this to now + MIN_CAN_TX_DISTANCE
    let mut next_earliest_tx_time = Instant::now();

    // can_config.set_error_warning_limit(limit);
    // not needed can_config.set_filter(filter);
    let mut can = can_config.start();
    can.clear_receive_fifo();

    // state of the heater
    let mut heater_state = HeaterState::default();

    // target values towards the heating controller:
    // we use a local variable here and update the global only when something changes
    let mut heater_control = HeaterControl {
        target_supply_temp_raw: 20, // 10°C as default
        heat_pump_eco_mode: 1,      // pump off as default
    };

    // put the initial periodic frames into the tx queue:

    tx_queue
        .push(CanTxEntry {
            id: CF_REQUEST_STATUS,
            data: Vec::default(),
            next_tx: embassy_time::Instant::now() + Duration::from_secs(5),
            cycle_time: Duration::from_secs(30),
        })
        .unwrap();
    tx_queue
        .push(CanTxEntry {
            id: CF_TR_OUTSIDE_TEMP_CTRL,
            data: Vec::from_slice(&[0u8]).unwrap(), // no outside temp control
            next_tx: embassy_time::Instant::now() + Duration::from_secs(6),
            cycle_time: Duration::from_secs(30),
        })
        .unwrap();
    tx_queue
        .push(CanTxEntry {
            id: CF_HEAT_PUMP_ECO_MODE,
            data: Vec::from_slice(&[heater_control.heat_pump_eco_mode]).unwrap(), // TODO default off??? (depend on e.g. outside temp? What if no data is available?)
            next_tx: embassy_time::Instant::now() + Duration::from_secs(7),
            cycle_time: Duration::from_secs(30),
        })
        .unwrap();
    tx_queue
        .push(CanTxEntry {
            id: CF_HEAT_TARGET_SUPPLY_TEMP,
            data: Vec::from_slice(&[heater_control.target_supply_temp_raw]).unwrap(), // 10°C as default -> off (first eval below will update)
            next_tx: embassy_time::Instant::now() + Duration::from_secs(8),
            cycle_time: Duration::from_secs(30),
        })
        .unwrap();

    loop {
        debug!(
            "heating_task tick, can.bus_off(): {},.receive_error_count(): {}, .num_avail_msgs: {}",
            can.is_bus_off(),
            can.receive_error_count(),
            can.num_available_messages()
        );
        let heating_need_perc = calc_heating_needs(); // TODO no need to do this so often. Every 2s is enough...
        debug!("Calculated heating need: {}%", heating_need_perc);

        // map it to two values:
        // heat needed? (heating_need_perc > 0)
        let heat_pump_eco_mode = if heating_need_perc > 0 { 0 } else { 1 };
        // target supply temp: map 0..100% to 30..60°C
        let target_supply_temp_celsius = if heat_pump_eco_mode == 0 {
            30.0 + (heating_need_perc as f32 * 0.3) // 30..60°C
        } else {
            10.0 // treated as off
        };

        let target_supply_temp_raw = (target_supply_temp_celsius * 2.0) as u8; // in 0.5°C steps
        if target_supply_temp_raw != heater_control.target_supply_temp_raw
            || heat_pump_eco_mode != heater_control.heat_pump_eco_mode
        {
            info!(
                "Heating need: {}%, heat_needed: {}, target_supply_temp: {} °C (raw: {})",
                heating_need_perc,
                heat_pump_eco_mode,
                target_supply_temp_celsius,
                target_supply_temp_raw
            );
            heater_control.target_supply_temp_raw = target_supply_temp_raw;
            heater_control.heat_pump_eco_mode = heat_pump_eco_mode;

            HEATER_CONTROL.lock(|hc| {
                *hc.borrow_mut() = heater_control.clone();
            });

            update_target_values_in_tx_queue(&mut tx_queue, &heater_control);
        }

        update_display(&mut heater_state, &heater_control);

        // first receive any available frames:
        while can.num_available_messages() > 0 {
            match can.receive() {
                Ok(frame) => {
                    // debug!("Received CAN frame: ID: {}, Data: {:?}", frame.id(), frame.data());
                    info!("Received CAN frame: {}", defmt::Debug2Format(&frame));
                    let data = frame.data();
                    match frame.id() {
                        Id::Standard(id) if matches!(id.as_raw(), CF_ERROR_MESSAGES) => {
                            if data.len() >= 1 {
                                let err_msg = data[0];
                                info!("rcvd: Störungsmeldung: 0x{:02X}", err_msg);
                                heater_state.err_msg = err_msg;
                                // store time or e.g. history as well?
                            }
                        }
                        Id::Standard(id)
                            if matches!(
                                id.as_raw(),
                                CF_CUR_BURNER_STATUS | CF_HEAT_CUR_PUMP_STATUS
                            ) =>
                        {
                            if data.len() >= 1 {
                                let status = data[0] != 0;
                                match id.as_raw() {
                                    CF_CUR_BURNER_STATUS => {
                                        info!(
                                            "rcvd: Aktueller Brennerstatus: {}",
                                            if status { "AN" } else { "AUS" }
                                        );
                                        heater_state.cur_burner_on = status;
                                    }
                                    CF_HEAT_CUR_PUMP_STATUS => {
                                        info!(
                                            "rcvd: Aktueller Heizungspumpenstatus: {}",
                                            if status { "AN" } else { "AUS" }
                                        );
                                        heater_state.cur_heat_pump_on = status;
                                    }
                                    _ => (),
                                }
                            }
                        }
                        Id::Standard(id)
                            if matches!(
                                id.as_raw(),
                                CF_HEAT_MAX_SUPPLY_TEMP
                                    | CF_HEAT_CUR_SUPPLY_TEMP
                                    | CF_WW_MAX_SUPPLY_TEMP
                                    | CF_WW_CUR_SUPPLY_TEMP
                            ) =>
                        {
                            if data.len() >= 1 {
                                let cur_temp_raw = data[0];
                                let temp_celsius = (cur_temp_raw as f32) / 2.0;
                                match id.as_raw() {
                                    CF_HEAT_MAX_SUPPLY_TEMP => {
                                        info!("rcvd: Max Vorlauftemp Hzg: {} °C", temp_celsius);
                                        heater_state.max_heat_supply_temp_celsius = temp_celsius;
                                    }
                                    CF_HEAT_CUR_SUPPLY_TEMP => {
                                        info!("rcvd: Akt Vorlauftemp Hzg: {} °C", temp_celsius);
                                        heater_state.cur_heat_supply_temp_celsius = temp_celsius;
                                    }
                                    CF_WW_MAX_SUPPLY_TEMP => {
                                        info!("rcvd: Max Vorlauftemp WW: {} °C", temp_celsius);
                                        heater_state.max_ww_supply_temp_celsius = temp_celsius;
                                    }
                                    CF_WW_CUR_SUPPLY_TEMP => {
                                        info!("rcvd: Akt Vorlauftemp WW: {} °C", temp_celsius);
                                        heater_state.cur_ww_supply_temp_celsius = temp_celsius;
                                    }
                                    _ => (),
                                }
                            } else {
                                warn!(
                                    "Invalid data length for current 1byte temp frame: {}",
                                    defmt::Debug2Format(&frame)
                                );
                            }
                        }
                        Id::Standard(id) => {
                            debug!("Standard ID: {}", defmt::Debug2Format(&id.as_raw()))
                        }
                        Id::Extended(id) => {
                            debug!("Extended ID: {}", defmt::Debug2Format(&id.as_raw()));
                        }
                    }
                }
                Err(e) => {
                    warn!("Error receiving CAN frame: {}", defmt::Debug2Format(&e));
                }
            }
        }
        if !can.is_bus_off() {
            // now send any frames that are due:
            let now = embassy_time::Instant::now();
            if now >= next_earliest_tx_time {
                if let Some(entry) = tx_queue.peek() {
                    if entry.next_tx <= now {
                        let mut entry = tx_queue.pop().unwrap();
                        // send it:
                        if let Err(e) = send_std_can_frame(&mut can, entry.id, &entry.data, 3).await
                        {
                            warn!("Error sending CAN frame: {}", defmt::Debug2Format(&e));
                        }
                        let now = Instant::now();
                        next_earliest_tx_time = now + MIN_CAN_TX_DISTANCE;
                        // if periodic, re-insert with updated next_tx
                        if entry.cycle_time >= MIN_CAN_TX_DISTANCE {
                            entry.next_tx = now + entry.cycle_time;
                            tx_queue.push(entry).unwrap();
                        }
                    }
                }
            }
        // send simulated can frames: (only for testing without real heating controller)
        /*
                    if let Err(e) = send_std_can_frame(&mut can, CF_HEAT_MAX_SUPPLY_TEMP, &[150u8], 3).await {
                        warn!("Error sending CAN frame: {}", defmt::Debug2Format(&e));
                    }
                    if let Err(e) = send_std_can_frame(&mut can, CF_HEAT_CUR_SUPPLY_TEMP, &[68u8], 3).await {
                        warn!("Error sending CAN frame: {}", defmt::Debug2Format(&e));
                    }
                    if let Err(e) = send_std_can_frame(&mut can, CF_WW_MAX_SUPPLY_TEMP, &[120u8], 3).await {
                        warn!("Error sending CAN frame: {}", defmt::Debug2Format(&e));
                    }
                    if let Err(e) = send_std_can_frame(&mut can, CF_WW_CUR_SUPPLY_TEMP, &[99u8], 3).await {
                        warn!("Error sending CAN frame: {}", defmt::Debug2Format(&e));
                    }
        */
        } else {
            warn!("CAN bus is off, skipping transmit");
            can.clear_receive_fifo();
        }
        // TODO handle receive error count!

        // wait a bit, there is so little traffic on the bus that we can wait a lot longer...
        Timer::after(Duration::from_millis(250)).await;
    }
    // let _ = can.stop();
    // warn!("end heating_task");
}

/// Update the display data with current target values
///
/// We toggle the following infos:
/// - if there is an error, show "E xx" every 2s for 1s
/// - If the pump is off (eco mode), show "H. AUS", cyclically
/// - If the pump is on, show the target supply temp, cyclically
/// - Show the burner status "B. ON" and pump status ("P. ON"), cyclically
/// - Show the current water temp as well "W xx.x°C", cyclically

// the logic is realized by a simple state machine via:
// a list that gets cycled through every 1s
// list items: ERROR, PUMP_STATE, ERROR, BURNER_STATE, ERROR, TARGET_TEMP, ERROR, WATER_TEMP
// error is skipped if no error

enum DisplayState {
    Error,
    PumpState,
    BurnerState,
    TargetTemp,
    WaterTemp,
}

const DISPLAY_LIST: [DisplayState; 8] = [
    DisplayState::Error,
    DisplayState::PumpState,
    DisplayState::Error,
    DisplayState::BurnerState,
    DisplayState::Error,
    DisplayState::TargetTemp,
    DisplayState::Error,
    DisplayState::WaterTemp,
];

fn update_display(heater_state: &mut HeaterState, heater_control: &HeaterControl) {
    static mut NEXT_DISPLAY_STATE_CHANGE: Instant = Instant::from_ticks(0);
    static mut CURRENT_DISPLAY_LIST_IDX: usize = 0;

    // next update time?
    let now = Instant::now();
    if now < unsafe { NEXT_DISPLAY_STATE_CHANGE } {
        return;
    }
    // update next change time
    unsafe {
        // let old_idx = CURRENT_DISPLAY_LIST_IDX;
        NEXT_DISPLAY_STATE_CHANGE = now + Duration::from_secs(2);

        CURRENT_DISPLAY_LIST_IDX = (CURRENT_DISPLAY_LIST_IDX + 1) % DISPLAY_LIST.len();
        // skip next error state if no error
        let is_err = matches!(DISPLAY_LIST[CURRENT_DISPLAY_LIST_IDX], DisplayState::Error);
        let has_err = heater_state.err_msg != 0;
        if is_err && !has_err {
            CURRENT_DISPLAY_LIST_IDX = (CURRENT_DISPLAY_LIST_IDX + 1) % DISPLAY_LIST.len();
        }
        // info!("Display state changed: {} -> {}, is_err: {}, has err: {}", old_idx, CURRENT_DISPLAY_LIST_IDX, is_err && !has_err, has_err);
    }

    HEATING_TEXT_TO_DISPLAY.lock(|htd| {
        let mut htd = htd.borrow_mut();
        htd.0.clear();
        match DISPLAY_LIST[unsafe { CURRENT_DISPLAY_LIST_IDX }] {
            DisplayState::Error => {
                if heater_state.err_msg != 0 {
                    let _ = core::write!(htd.0, "E. {:02X}", heater_state.err_msg);
                    htd.1 = Rgb565::YELLOW;
                    htd.2 = Rgb565::BLACK;
                }
            }
            DisplayState::PumpState => {
                let _ = core::write!(
                    htd.0,
                    "H. {}",
                    if heater_state.cur_heat_pump_on {
                        "AN"
                    } else {
                        "AUS"
                    }
                );
                htd.1 = if heater_state.cur_heat_pump_on {
                    Rgb565::GREEN
                } else {
                    Rgb565::BLUE
                };
                htd.2 = Rgb565::BLACK;
            }
            DisplayState::BurnerState => {
                let _ = core::write!(
                    htd.0,
                    "F. {}",
                    if heater_state.cur_burner_on {
                        "AN"
                    } else {
                        "AUS"
                    }
                );
                htd.1 = if heater_state.cur_burner_on {
                    Rgb565::RED
                } else {
                    Rgb565::BLUE
                };
                htd.2 = Rgb565::BLACK;
            }
            DisplayState::TargetTemp => {
                let _ = core::write!(
                    htd.0,
                    "{:.1}°C",
                    heater_control.target_supply_temp_raw as f32 * 0.5
                );
                htd.1 = if heater_control.target_supply_temp_raw as f32 * 0.5 < 20.0 {
                    Rgb565::BLUE
                } else {
                    Rgb565::RED
                };
                htd.2 = Rgb565::BLACK;
            }
            DisplayState::WaterTemp => {
                let _ = core::write!(
                    htd.0,
                    "L. {:>2.0}°C",
                    heater_state.cur_ww_supply_temp_celsius
                );
                htd.1 = Rgb565::BLUE;
                htd.2 = Rgb565::BLACK;
            }
        }
    });
}

fn update_target_values_in_tx_queue(
    tx_queue: &mut heapless::BinaryHeap<CanTxEntry, heapless::binary_heap::Min, 8>,
    heater_control: &HeaterControl,
) {
    // remove existing entries and re-insert with updated values and next_tx = now
    let mut new_queue: heapless::BinaryHeap<CanTxEntry, heapless::binary_heap::Min, 8> =
        heapless::BinaryHeap::new();
    while let Some(mut entry) = tx_queue.pop() {
        if entry.id == CF_HEAT_TARGET_SUPPLY_TEMP {
            entry.data = Vec::from_slice(&[heater_control.target_supply_temp_raw]).unwrap();
            entry.next_tx = embassy_time::Instant::now();
            info!(
                "Updated target supply temp in tx queue to raw value: {}",
                heater_control.target_supply_temp_raw
            );
        } else if entry.id == CF_HEAT_PUMP_ECO_MODE {
            entry.data = Vec::from_slice(&[heater_control.heat_pump_eco_mode]).unwrap();
            entry.next_tx = embassy_time::Instant::now();
            info!(
                "Updated heat pump eco mode in tx queue to: {}",
                heater_control.heat_pump_eco_mode
            );
        }
        new_queue.push(entry).unwrap();
    }
    *tx_queue = new_queue;
}

/// Send a standard CAN frame with self-reception
///
/// # Parameters
/// * `can` - Mutable reference to the TWAI controller
/// * `id` - Standard CAN identifier
/// * `data` - Data bytes to send
/// * `auto_retries` - Number of automatic retries in case of WouldBlock. Retries will be done with a delay of 10ms.
async fn send_std_can_frame(
    can: &mut esp_hal::twai::Twai<'static, Blocking>,
    id: u16,
    data: &[u8],
    auto_retries: u8,
) -> Result<(), nb::Error<twai::EspTwaiError>> {
    let frame = twai::EspTwaiFrame::new_self_reception(
        StandardId::new(id).ok_or(twai::EspTwaiError::NonCompliantDlc(0xde))?,
        data,
    )
    .ok_or(twai::EspTwaiError::NonCompliantDlc(0xdf))?;

    let mut auto_retries = auto_retries;
    loop {
        match can.transmit(&frame) {
            Ok(()) => {
                break;
            }
            Err(nb::Error::WouldBlock) if auto_retries > 0 => {
                warn!(
                    "CAN transmit would block, retrying... ({} retries left)",
                    auto_retries
                );
                auto_retries -= 1;
                Timer::after(Duration::from_millis(10)).await;
            }
            Err(e) => {
                return Err(e);
            }
        }
    }
    info!("Sent CAN frame: ID: {}", defmt::Debug2Format(&frame.id()));
    Ok(())
}

fn calc_heating_needs() -> u16 {
    // manual override active?
    let now = Instant::now();
    let heating_need_manual = HEATER_OVERWRITE.lock(|ho| {
        let ho = ho.borrow();
        if now < ho.active_till {
            Some(ho.heating_need_perc)
        } else {
            None
        }
    });

    // lock the DEVICES mutex
    let heating_need = if let Some(manual_need) = heating_need_manual {
        debug!("Using manual override heating need: {}%", manual_need);
        manual_need
    } else {
        DEVICES.lock(|devices| {
            let devices = devices.borrow();

            // for now a very simple calculation: sum of all valve positions / number of devices
            let mut total_valve_position: u16 = 0;
            for (id, device) in devices.iter().enumerate() {
                debug!(
                    "Device ID: {}, Valve Position: {}",
                    id, device.valve_position
                );
                // ignore all devices with valve position <5%?
                total_valve_position += (device.valve_position * 100.0) as u16;
            }
            if devices.is_empty() {
                // warn!("No devices found for heating calculation!");
                0
            } else {
                total_valve_position / devices.len() as u16
            }
        })
    };

    heating_need
}
