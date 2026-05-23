#![no_std]
#![no_main]

extern crate alloc;

#[cfg(feature = "serial")]
use rp_keyer::cdc_serial_state::{CdcWithSerialState, State as CdcState};
#[cfg(feature = "midi")]
use rp_keyer::midi_interrupt::MidiInterruptClass;

use rp_keyer::buttons::{ButtonInput, ButtonPanel};
use rp_keyer::buzzer::Buzzer;
use rp_keyer::radio_key::RadioKey;
use rp_keyer::shared::{ConfigDirtySignal, SharedConfig, SharedDecoder, UsbStateSignal};
use rp_keyer::storage::{self, AsyncFlash, FLASH_TOTAL_SIZE};
use rp_keyer::ui::{OledI2c, Ui};
use rp_keyer::usb_emit_task::UsbEmitApp;

use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig, I2c};
use embassy_rp::peripherals::{FLASH, I2C0, USB};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Timer};
use embassy_usb::Builder;
use embedded_alloc::LlffHeap as Heap;
use radio_utils_keyer::KeyerConfig;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

#[defmt::panic_handler]
fn panic() -> ! {
    rp_keyer::safe_stop();
    cortex_m::asm::udf()
}

// ── Global allocator ─────────────────────────────────────────────────────
// The keyer engine uses Vec/VecDeque internally for text macros and the
// text_queue.  64 KiB is comfortable: macros peak at a few hundred
// elements; the transition log is a fixed-size heapless ring (see
// `MAX_RECORDED_TRANSITIONS` in radio_utils_keyer::engine) so it doesn't
// touch the heap at all; menu strings are heapless::String.
#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 65_536;
static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

// ── Shared state ─────────────────────────────────────────────────────────
static SHARED_CONFIG: StaticCell<SharedConfig> = StaticCell::new();
static KEY_INDICATOR: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
/// `true` whenever the keyer engine is mid-element / has a held paddle
/// or pending PTT hang; updated each engine tick.  The persistence
/// task waits for this to fall to `false` before committing a save so
/// the ~50 ms flash-erase pause doesn't interrupt active keying.
static ENGINE_ACTIVE: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
/// Handshake: the persistence task sets this to `true` after observing
/// `ENGINE_ACTIVE == false` and before calling the (~50 ms,
/// IRQs-disabled) flash erase.  The keyer task checks it at the top
/// of every iteration and parks if set — closes the race where the
/// engine could start keying between the persistence task's load and
/// the erase entry.  See `keyer_task::run`'s prologue and the
/// `config_persist_task` handshake comment.
static SAVE_LOCK: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
/// Keyer → USB emit task channel.  Lossy by design — only the latest
/// snapshot matters; the consumer caches `prev_*` so a write failure
/// can't drop an edge.
static USB_SIGNAL: UsbStateSignal = embassy_sync::signal::Signal::new();
/// UI → persistence task — set on every config-mutating button press.
/// The persistence task debounces and waits for engine idle before
/// committing.
static CONFIG_DIRTY: ConfigDirtySignal = embassy_sync::signal::Signal::new();
/// Backing storage for the per-device USB serial number derived from
/// the flash unique-ID; lives in `.bss` so the descriptor table can
/// borrow it `'static`.
static SERIAL_BUF: StaticCell<[u8; 16]> = StaticCell::new();
/// CW decoder, shared between the keyer task (transition feeder) and
/// the UI task (poll + snapshot for the main-screen text row).  See
/// `shared::SharedDecoder` for the locking model.
static DECODER: SharedDecoder = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(
    radio_utils_cw_decoder::Decoder::new(),
));

// ── Tasks ────────────────────────────────────────────────────────────────
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn keyer_task(
    dit_pin: Input<'static>,
    dah_pin: Input<'static>,
    radio_key: RadioKey,
    buzzer: Buzzer,
    shared_config: &'static SharedConfig,
) {
    rp_keyer::keyer_task::run(
        dit_pin,
        dah_pin,
        radio_key,
        buzzer,
        shared_config,
        &KEY_INDICATOR,
        &USB_SIGNAL,
        &ENGINE_ACTIVE,
        &SAVE_LOCK,
        &DECODER,
    )
    .await;
}

#[embassy_executor::task]
async fn usb_emit_task(mut app: UsbEmitApp<'static, Driver<'static, USB>>) {
    app.run(&USB_SIGNAL).await;
}

#[embassy_executor::task]
async fn ui_task(
    i2c: OledI2c,
    mut buttons: ButtonPanel,
    shared_config: &'static SharedConfig,
) -> ! {
    let mut ui = Ui::new(i2c).await;

    let mut btn_buf = [rp_keyer::buttons::ButtonEvent::Up; 4];
    // Cached WPM + enabled flag for the decoder poll. Both refreshed
    // every render below; seeded from the persisted config so the very
    // first poll uses the right threshold and (when disabled) skips
    // the lock entirely.
    let (mut decoder_wpm, mut decoder_enabled) = {
        let cfg = shared_config.lock().await;
        (cfg.speed_wpm, cfg.decoder_enabled)
    };
    let mut decoded_buf = [0u8; 32];
    loop {
        ui.note_key_state(KEY_INDICATOR.load(portable_atomic::Ordering::Relaxed));

        let events = buttons.poll(&mut btn_buf);
        if !events.is_empty() {
            let mut guard = shared_config.lock().await;
            for &ev in events {
                ui.on_button(ev, &mut guard);
            }
            // Drop the guard before render so we don't hold it during
            // the (blocking) I2C flush.
            drop(guard);
            // Signal the persistence task on any edit — it debounces +
            // waits for idle before actually writing flash.
            if ui.take_config_changed() {
                CONFIG_DIRTY.signal(());
            }
        }

        // Poll + snapshot the decoder when it's enabled.  Closure runs
        // under a critical section — keep it strictly to the decoder
        // calls and a small memcpy.  Both keyer task (writer) and UI
        // task (reader) take this lock; lock duration ≪ 1 µs so the
        // keyer task on SWI_IRQ_1 is never delayed appreciably.
        if decoder_enabled {
            let now_us = embassy_time::Instant::now().as_micros();
            let n = DECODER.lock(|d| {
                let mut d = d.borrow_mut();
                d.poll(now_us, decoder_wpm);
                d.snapshot(&mut decoded_buf)
            });
            ui.note_decoded(&decoded_buf[..n]);
        }

        // Only snapshot the config when there's actually something to
        // draw — avoids the [String; 8] clone + critical section every
        // 25 ms when nothing on screen changed.
        if ui.is_dirty() {
            let cfg_snap: KeyerConfig = shared_config.lock().await.clone();
            decoder_wpm = cfg_snap.speed_wpm;
            decoder_enabled = cfg_snap.decoder_enabled;
            ui.render(&cfg_snap);
        }

        Timer::after(Duration::from_millis(25)).await;
    }
}

/// Debounce-and-commit task for settings persistence.
///
/// State machine: wait → debounce → acquire-save-window → save.
///
/// * **wait** — block on `CONFIG_DIRTY` until the UI signals an edit.
/// * **debounce** — each further edit within 3 s extends the window.
///   Avoids 8 flash writes when the user scrolls a slider up by 8.
/// * **acquire-save-window** — flash erase disables interrupts for
///   ~50 ms, so we need the keyer engine to be quiescent for the
///   duration.  Two-step handshake:
///
///     1. wait for `ENGINE_ACTIVE == false`,
///     2. set `SAVE_LOCK = true` and wait one millisecond (4 keyer
///        ticks at the 250 µs cadence) for the keyer to observe it,
///     3. recheck `ENGINE_ACTIVE`.  If it raced (paddle press landed
///        in step 1's gap before the keyer parked), release the lock
///        and retry.  Otherwise the keyer is parked at its prologue
///        and won't drive the key line until we clear `SAVE_LOCK`.
///
/// * **save** — snapshot the shared config and append to the ring.
///   IRQs go off for ~50 ms during the erase; the keyer task is
///   already parked, so this is safe.  Clear `SAVE_LOCK` when done.
#[embassy_executor::task]
async fn config_persist_task(mut flash: AsyncFlash, shared_config: &'static SharedConfig) {
    const DEBOUNCE: Duration = Duration::from_secs(3);
    const IDLE_RECHECK: Duration = Duration::from_millis(250);
    /// Long enough for the keyer task (250 µs cadence) to observe
    /// SAVE_LOCK and have updated ENGINE_ACTIVE one more time so the
    /// recheck reflects post-park reality.
    const PARK_OBSERVE: Duration = Duration::from_millis(1);

    loop {
        // Wait for the first dirty signal of this batch.
        CONFIG_DIRTY.wait().await;

        // Debounce: each new dirty signal restarts the 3-second window.
        // The loop exits as soon as `Timer::after(DEBOUNCE)` wins, i.e.
        // the user has been quiet for the full window.
        while let Either::First(_) = select(CONFIG_DIRTY.wait(), Timer::after(DEBOUNCE)).await {}

        // Acquire-save-window handshake — see task docstring.
        loop {
            while ENGINE_ACTIVE.load(portable_atomic::Ordering::Relaxed) {
                Timer::after(IDLE_RECHECK).await;
            }
            SAVE_LOCK.store(true, portable_atomic::Ordering::Relaxed);
            Timer::after(PARK_OBSERVE).await;
            if !ENGINE_ACTIVE.load(portable_atomic::Ordering::Relaxed) {
                break;
            }
            // Raced — a paddle press landed between the engine-active
            // check and the lock store.  Release and retry.
            SAVE_LOCK.store(false, portable_atomic::Ordering::Relaxed);
            Timer::after(IDLE_RECHECK).await;
        }

        let cfg = shared_config.lock().await.clone();
        match storage::save_config(&mut flash, &cfg).await {
            Ok(()) => info!("settings saved"),
            Err(()) => warn!("settings save failed"),
        }
        SAVE_LOCK.store(false, portable_atomic::Ordering::Relaxed);
    }
}

// ── Interrupt executor for the keyer task ────────────────────────────────
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
unsafe extern "C" fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

/// Read the RP2040 flash unique ID and format it as 16 hex chars into
/// `buf`.  Falls back to a fixed string if the flash read errors so
/// the USB enumeration still has *some* serial.
fn read_flash_serial_into(
    flash: &mut Flash<'static, FLASH, Blocking, { FLASH_TOTAL_SIZE }>,
    buf: &'static mut [u8; 16],
) -> &'static str {
    let mut uid = [0u8; 8];
    if flash.blocking_unique_id(&mut uid).is_ok() {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        for (i, byte) in uid.iter().enumerate() {
            buf[2 * i] = HEX[(byte >> 4) as usize];
            buf[2 * i + 1] = HEX[(byte & 0x0F) as usize];
        }
    } else {
        // Deterministic 16-byte fallback so two enumeration attempts
        // don't churn the OS's USB device cache.
        buf.copy_from_slice(b"rpk-rp2040-0000\0");
    }
    // Safety: every byte above is ASCII (hex digits or the printable
    // fallback string), so it's valid UTF-8.
    unsafe { core::str::from_utf8_unchecked(buf.as_slice()) }
}

// ── Main ─────────────────────────────────────────────────────────────────
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialise the heap before anything that touches the keyer crate
    // (KeyerEngine::new allocates a Vec).
    unsafe {
        HEAP.init(core::ptr::addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE);
    }

    let p = embassy_rp::init(Default::default());

    info!("radio-utils-firmware booting (RP2040)");

    // ── Flash driver: unique-ID read for USB serial + settings ring ──
    let mut flash: Flash<'static, FLASH, Blocking, { FLASH_TOTAL_SIZE }> =
        Flash::new_blocking(p.FLASH);
    let serial_number = read_flash_serial_into(&mut flash, SERIAL_BUF.init([0u8; 16]));
    let mut async_flash = AsyncFlash::new(flash);

    // ── Try to load saved settings; fall back to defaults ────────────
    let initial_cfg: KeyerConfig = match storage::load_config(&mut async_flash).await {
        Some(cfg) => {
            info!("settings restored from flash");
            cfg
        }
        None => {
            info!("no saved settings — using defaults");
            KeyerConfig::default()
        }
    };

    // ── Shared keyer config ──────────────────────────────────────────
    let shared_config: &'static SharedConfig =
        SHARED_CONFIG.init(embassy_sync::mutex::Mutex::new(initial_cfg));

    // ── USB ──────────────────────────────────────────────────────────
    let driver = Driver::new(p.USB, Irqs);
    let mut usb_config = embassy_usb::Config::new(0x16c0, 0x27dc);
    usb_config.manufacturer = Some("radio-utils");
    usb_config.product = Some("CW Keyer + Interface");
    usb_config.serial_number = Some(serial_number);
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 512]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESCRIPTOR.init([0; 512]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    #[cfg(feature = "serial")]
    let serial = {
        static CDC_STATE: StaticCell<CdcState> = StaticCell::new();
        Some(CdcWithSerialState::new(
            &mut builder,
            CDC_STATE.init(CdcState::new()),
            64,
        ))
    };

    #[cfg(feature = "midi")]
    let midi = Some(MidiInterruptClass::new(&mut builder, 1, 0, 64, 1));

    let usb = builder.build();
    spawner.spawn(usb_task(usb).unwrap());

    // ── Paddles ──────────────────────────────────────────────────────
    let dit_pin = Input::new(p.PIN_14, Pull::Up);
    let dah_pin = Input::new(p.PIN_15, Pull::Up);

    // ── Radio key (2N2222 base) ──────────────────────────────────────
    let radio_key = RadioKey::new(Output::new(p.PIN_18, Level::Low));

    // ── Buzzer on PWM1 channel B (PIN_19) ────────────────────────────
    let buzzer = {
        let pwm_cfg = PwmConfig::default();
        let pwm = Pwm::new_output_b(p.PWM_SLICE1, p.PIN_19, pwm_cfg);
        Buzzer::new(pwm)
    };

    // ── OLED I2C0: SDA=GP20, SCL=GP21 ───────────────────────────────
    let oled_i2c: OledI2c = {
        let mut cfg = I2cConfig::default();
        cfg.frequency = 400_000;
        I2c::new_blocking(p.I2C0, p.PIN_21, p.PIN_20, cfg)
    };

    // ── Buttons: UP=GP10, DOWN=GP11, OK=GP12, BACK=GP13 ──────────────
    let buttons = ButtonPanel {
        up: ButtonInput::new(Input::new(p.PIN_10, Pull::Up)),
        down: ButtonInput::new(Input::new(p.PIN_11, Pull::Up)),
        ok: ButtonInput::new(Input::new(p.PIN_12, Pull::Up)),
        back: ButtonInput::new(Input::new(p.PIN_13, Pull::Up)),
    };

    // ── Spawn UI + USB emit + persistence on the thread executor ─────
    spawner.spawn(ui_task(oled_i2c, buttons, shared_config).unwrap());
    spawner.spawn(config_persist_task(async_flash, shared_config).unwrap());

    let usb_emit_app = UsbEmitApp {
        #[cfg(feature = "serial")]
        serial,
        #[cfg(feature = "midi")]
        midi,
        #[cfg(not(any(feature = "serial", feature = "midi")))]
        _marker: core::marker::PhantomData,
    };
    spawner.spawn(usb_emit_task(usb_emit_app).unwrap());

    // ── Interrupt priorities ─────────────────────────────────────────
    // All NVIC priorities default to P0; the InterruptExecutor running
    // on SWI_IRQ_1 then can't preempt USBCTRL_IRQ / I2C0_IRQ even
    // though it's the highest-latency-sensitive code on the chip.
    // Lower USB + I2C to P3 (lowest) and keep SWI at P0 so the keyer
    // task can actually meet its 250 µs cadence.
    use embassy_rp::interrupt::{InterruptExt, Priority};
    embassy_rp::interrupt::USBCTRL_IRQ.set_priority(Priority::P3);
    embassy_rp::interrupt::I2C0_IRQ.set_priority(Priority::P3);
    embassy_rp::interrupt::SWI_IRQ_1.set_priority(Priority::P0);

    // ── Spawn keyer task on the elevated-priority interrupt executor ─
    let hi_spawner = EXECUTOR_HIGH.start(embassy_rp::pac::Interrupt::SWI_IRQ_1);

    hi_spawner.spawn(keyer_task(dit_pin, dah_pin, radio_key, buzzer, shared_config).unwrap());

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(3600)).await;
    }
}
