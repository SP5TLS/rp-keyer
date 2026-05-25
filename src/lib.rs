#![no_std]

extern crate alloc;

pub mod cdc_serial_state;
pub mod midi_interrupt;

pub mod buttons;
pub mod buzzer;
pub mod keyer_task;
pub mod paddle;
pub mod radio_key;
pub mod shared;
pub mod storage;
pub mod ui;
pub mod usb_emit_task;

/// GPIO pin the firmware drives the 2N2222 keying base from.
pub const RADIO_KEY_PIN: u8 = 11;

/// PWM slice driving the piezo buzzer (channel A / PIN_10).
pub const BUZZER_PWM_SLICE: usize = 5;

/// Deassert the radio key and silence the buzzer using only direct
/// register writes — safe to call from a panic handler (no allocator,
/// no async, no driver state).
///
/// Without this the default panic handler runs `udf` → HardFault loop
/// with the radio-key pin latched high, leaving the radio transmitting
/// an unmodulated carrier until power-cycled.
pub fn safe_stop() {
    use embassy_rp::pac;

    cortex_m::interrupt::disable();

    // GPIO_OUT bank-0 atomic-clear: bits set in the written value clear
    // the corresponding output bits. The radio-key pin's pad function
    // was set to SIO output in main(); we don't change it here.
    pac::SIO
        .gpio_out(0)
        .value_clr()
        .write_value(1u32 << RADIO_KEY_PIN);

    // Force the buzzer slice's compare values to 0 so the buzzer
    // output stays low for the rest of the current period and every
    // period thereafter.
    pac::PWM
        .ch(BUZZER_PWM_SLICE)
        .cc()
        .write_value(pac::pwm::regs::ChCc(0));
}
