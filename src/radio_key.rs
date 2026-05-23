//! GPIO drive of the 2N2222 NPN that keys the radio.
//!
//! Wiring: MCU pin -> 1k base resistor -> 2N2222 base.  Emitter -> GND.
//! Collector -> radio key tip (or PTT line) through a current-limiting
//! resistor.  Pull-up to the radio's keying supply is provided by the
//! radio itself.
//!
//! Active-high: pin HIGH saturates the transistor, sinking the radio's
//! key line to GND and keying the carrier.

use embassy_rp::gpio::Output;

pub struct RadioKey {
    pin: Output<'static>,
}

impl RadioKey {
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }

    pub fn set(&mut self, on: bool) {
        if on {
            self.pin.set_high();
        } else {
            self.pin.set_low();
        }
    }
}
