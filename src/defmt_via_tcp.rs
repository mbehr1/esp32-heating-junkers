// copied from https://github.com/probe-rs/rtt-target/blob/master/rtt-target/Cargo.toml
// MIT License
//
// extended to use fring as ringbuffer for defmt data

use portable_atomic::{AtomicBool, Ordering};
use rtt_target::{UpChannel, rtt_init}; // TODO use core::sync::atomic::...

// ringbuffer used for tcp transfer
const LOG_RINGBUF_SIZE: usize = 1024;
static mut LOG_RINGBUF: fring::Buffer<LOG_RINGBUF_SIZE> = fring::Buffer::new();

const RTT_BUFFER_SIZE: usize = 1024;
static mut CHANNEL: Option<UpChannel> = None;

#[defmt::global_logger]
struct Logger;

/// Sets the channel to use for [`defmt`] macros.
fn set_defmt_channel(channel: UpChannel) {
    unsafe { CHANNEL = Some(channel) }
}

/// Global logger lock.
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();
static mut ENCODER: defmt::Encoder = defmt::Encoder::new(); // TODO ensure this is encoding-rzcobs

static mut PRODUCER: Option<fring::Producer<'static, LOG_RINGBUF_SIZE>> = None;

static mut WRITE_REGION: Option<
    fring::Region<'static, fring::Producer<'static, LOG_RINGBUF_SIZE>>,
> = None;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // safety: Must be paired with corresponding call to release(), see below
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // no need for CAS because interrupts are disabled
        TAKEN.store(true, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe { CS_RESTORE = restore };

        unsafe {
            if let Some(producer) = &mut *core::ptr::addr_of_mut!(PRODUCER) {
                WRITE_REGION = Some(producer.write(usize::MAX));
            }
        };

        // safety: accessing the `static mut` is OK because we have disabled interrupts / are mutex protected.
        unsafe {
            let encoder = &mut *core::ptr::addr_of_mut!(ENCODER);
            encoder.start_frame(do_write)
        }
    }

    unsafe fn flush() {}

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let encoder = unsafe { &mut *core::ptr::addr_of_mut!(ENCODER) };
        encoder.end_frame(do_write);

        // drop the write region to commit the data
        unsafe {
            let cur_region = (*core::ptr::addr_of_mut!(WRITE_REGION)).take();
            if let Some(region) = cur_region {
                region.partial_drop(0); // we dont need the rest
            }
            WRITE_REGION = None;
        };

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        TAKEN.store(false, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let restore = unsafe { CS_RESTORE };

        // safety: Must be paired with corresponding call to acquire(), see above
        unsafe { critical_section::release(restore) };
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        let encoder = unsafe { &mut *core::ptr::addr_of_mut!(ENCODER) };
        encoder.write(bytes, do_write);
    }
}

fn do_write(bytes: &[u8]) {
    unsafe {
        //let channel = core::ptr::addr_of_mut!(CHANNEL);
        let channel = &raw mut CHANNEL;
        if let Some(Some(c)) = channel.as_mut() {
            c.write(bytes);
        }
        if let Some(region) = &mut *core::ptr::addr_of_mut!(WRITE_REGION) {
            // write to region:
            let to_copy = bytes.len().min(region.len());
            if to_copy == 0 {
                return;
            }
            // TODO might be easier to copy into a local buf and on release copy that into the ringbuffer
            // would avoid partial frames as well!
            region[..to_copy].copy_from_slice(&bytes[..to_copy]);
            region.consume(to_copy);
        }
    }
}

/// our init function to be called once at program start
///
/// inits an RTT channel and sets it as defmt channel
///
pub fn init() -> fring::Consumer<'static, LOG_RINGBUF_SIZE> {
    let channels = rtt_init! {
        up: {
            0: {
                size: RTT_BUFFER_SIZE,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "defmt"
            }
        }
    };
    set_defmt_channel(channels.up.0);
    // UNSAFE: need to ensure init can only be called once!
    let log_ringbuffer = unsafe { &mut *core::ptr::addr_of_mut!(LOG_RINGBUF) };
    let (producer, consumer) = log_ringbuffer.split();
    unsafe {
        PRODUCER = Some(producer);
    }
    consumer
}
