#![no_std]
#![no_main]

// RP2350A (Pico 2) firmware.  See ../bin/rp2040.rs for the full
// commentary — this file is a near-mirror, differing only in:
//   * memory layout (set in build.rs by target triple),
//   * `embassy_rp::flash::blocking_unique_id` is RP2040-only, so the
//     RP2350 build presents a static serial number,
//   * no second-stage boot2 (BOOTROM jumps straight to flash).

extern crate alloc;

#[cfg(feature = "serial")]
use rp_keyer::cdc_serial_state::{CdcWithSerialState, State as CdcState};
#[cfg(feature = "midi")]
use rp_keyer::midi_interrupt::MidiInterruptClass;

use rp_keyer::buttons::{ButtonInput, ButtonPanel};
use rp_keyer::buzzer::Buzzer;
use rp_keyer::radio_key::RadioKey;
use rp_keyer::shared::{
    ConfigDirtySignal, SharedConfig, SharedDecoder, UsbStateSignal,
};
use rp_keyer::storage::{self, AsyncFlash};
use rp_keyer::ui::{OledI2c, Ui};
use rp_keyer::usb_emit_task::UsbEmitApp;

use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::flash::Flash;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig, I2c};
use embassy_rp::peripherals::{I2C0, USB};
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

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 65_536;
static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

static SHARED_CONFIG: StaticCell<SharedConfig> = StaticCell::new();
static KEY_INDICATOR: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
static ENGINE_ACTIVE: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
/// Handshake flag set by the persistence task before flash-erase to
/// park the keyer task — see rp2040.rs for the full rationale.
static SAVE_LOCK: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
/// CW decoder shared between keyer (writer) and UI (reader).
static DECODER: SharedDecoder = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(
    radio_utils_cw_decoder::Decoder::new(),
));
/// Keyer → USB emit task channel.  Lossy by design — see rp2040.rs.
static USB_SIGNAL: UsbStateSignal = embassy_sync::signal::Signal::new();
static CONFIG_DIRTY: ConfigDirtySignal = embassy_sync::signal::Signal::new();

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
    // See rp2040.rs for the seeding rationale.
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
            drop(guard);
            if ui.take_config_changed() {
                CONFIG_DIRTY.signal(());
            }
        }

        if decoder_enabled {
            let now_us = embassy_time::Instant::now().as_micros();
            let n = DECODER.lock(|d| {
                let mut d = d.borrow_mut();
                d.poll(now_us, decoder_wpm);
                d.snapshot(&mut decoded_buf)
            });
            ui.note_decoded(&decoded_buf[..n]);
        }

        if ui.is_dirty() {
            let cfg_snap: KeyerConfig = shared_config.lock().await.clone();
            decoder_wpm = cfg_snap.speed_wpm;
            decoder_enabled = cfg_snap.decoder_enabled;
            ui.render(&cfg_snap);
        }

        Timer::after(Duration::from_millis(25)).await;
    }
}

// See rp2040.rs::config_persist_task for the handshake rationale.
#[embassy_executor::task]
async fn config_persist_task(mut flash: AsyncFlash, shared_config: &'static SharedConfig) {
    const DEBOUNCE: Duration = Duration::from_secs(3);
    const IDLE_RECHECK: Duration = Duration::from_millis(250);
    const PARK_OBSERVE: Duration = Duration::from_millis(1);

    loop {
        CONFIG_DIRTY.wait().await;

        // Debounce: keep restarting the window each time the user
        // edits again; commit when the timer wins.
        while let Either::First(_) = select(CONFIG_DIRTY.wait(), Timer::after(DEBOUNCE)).await {}

        // Acquire-save-window handshake.
        loop {
            while ENGINE_ACTIVE.load(portable_atomic::Ordering::Relaxed) {
                Timer::after(IDLE_RECHECK).await;
            }
            SAVE_LOCK.store(true, portable_atomic::Ordering::Relaxed);
            Timer::after(PARK_OBSERVE).await;
            if !ENGINE_ACTIVE.load(portable_atomic::Ordering::Relaxed) {
                break;
            }
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

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
unsafe extern "C" fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        HEAP.init(core::ptr::addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE);
    }

    let p = embassy_rp::init(Default::default());

    info!("radio-utils-firmware booting (RP2350)");

    // ── Flash driver for the settings ring ───────────────────────────
    let flash = Flash::new_blocking(p.FLASH);
    let mut async_flash = AsyncFlash::new(flash);

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

    let shared_config: &'static SharedConfig =
        SHARED_CONFIG.init(embassy_sync::mutex::Mutex::new(initial_cfg));

    let driver = Driver::new(p.USB, Irqs);
    let mut usb_config = embassy_usb::Config::new(0x16c0, 0x27dd);
    usb_config.manufacturer = Some("radio-utils");
    usb_config.product = Some("CW Keyer + Interface");
    usb_config.serial_number = Some("rpk-rp2350-0001");
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

    let dit_pin = Input::new(p.PIN_14, Pull::Up);
    let dah_pin = Input::new(p.PIN_15, Pull::Up);

    let radio_key = RadioKey::new(Output::new(p.PIN_18, Level::Low));

    let buzzer = {
        let pwm_cfg = PwmConfig::default();
        let pwm = Pwm::new_output_b(p.PWM_SLICE1, p.PIN_19, pwm_cfg);
        Buzzer::new(pwm)
    };

    let oled_i2c: OledI2c = {
        let mut cfg = I2cConfig::default();
        cfg.frequency = 400_000;
        I2c::new_blocking(p.I2C0, p.PIN_21, p.PIN_20, cfg)
    };

    let buttons = ButtonPanel {
        up: ButtonInput::new(Input::new(p.PIN_10, Pull::Up)),
        down: ButtonInput::new(Input::new(p.PIN_11, Pull::Up)),
        ok: ButtonInput::new(Input::new(p.PIN_12, Pull::Up)),
        back: ButtonInput::new(Input::new(p.PIN_13, Pull::Up)),
    };

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
    // Lower USB + I2C so the keyer's interrupt executor (SWI_IRQ_1)
    // can actually preempt them and meet its 250 µs cadence.
    use embassy_rp::interrupt::{InterruptExt, Priority};
    embassy_rp::interrupt::USBCTRL_IRQ.set_priority(Priority::P3);
    embassy_rp::interrupt::I2C0_IRQ.set_priority(Priority::P3);
    embassy_rp::interrupt::SWI_IRQ_1.set_priority(Priority::P0);

    let hi_spawner = EXECUTOR_HIGH.start(embassy_rp::pac::Interrupt::SWI_IRQ_1);

    hi_spawner.spawn(keyer_task(dit_pin, dah_pin, radio_key, buzzer, shared_config).unwrap());

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(3600)).await;
    }
}
