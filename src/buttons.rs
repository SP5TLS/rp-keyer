//! Push-button GPIOs.
//!
//! Four buttons (UP, DOWN, OK, BACK) wired between an MCU pin and GND,
//! with the MCU's internal pull-up enabled — active-low.  Scanned by the
//! UI task at ~20 ms which is comfortably above mechanical switch
//! bounce; we add a one-shot edge detector so a single press generates a
//! single event regardless of how long it's held.

use embassy_rp::gpio::Input;

use crate::paddle::Debouncer;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButtonEvent {
    Up,
    Down,
    Ok,
    Back,
}

pub struct ButtonInput {
    pin: Input<'static>,
    debouncer: Debouncer,
    last_pressed: bool,
}

impl ButtonInput {
    pub fn new(pin: Input<'static>) -> Self {
        Self {
            pin,
            // 2-sample threshold is plenty when called every 20 ms.
            debouncer: Debouncer::new(false, 2),
            last_pressed: false,
        }
    }

    /// Returns `true` once on the rising edge of a press (released → pressed),
    /// otherwise `false`.
    pub fn poll_rising_edge(&mut self) -> bool {
        let raw = self.pin.is_low();
        let now_pressed = self.debouncer.update(raw);
        let edge = now_pressed && !self.last_pressed;
        self.last_pressed = now_pressed;
        edge
    }
}

pub struct ButtonPanel {
    pub up: ButtonInput,
    pub down: ButtonInput,
    pub ok: ButtonInput,
    pub back: ButtonInput,
}

impl ButtonPanel {
    /// Drain all rising edges from this poll into the provided buffer.
    /// Returns the slice of events actually written.
    pub fn poll<'a>(&mut self, buf: &'a mut [ButtonEvent; 4]) -> &'a [ButtonEvent] {
        let mut n = 0;
        let push = |b: &mut [ButtonEvent; 4], n: &mut usize, e: ButtonEvent| {
            if *n < b.len() {
                b[*n] = e;
                *n += 1;
            }
        };
        if self.up.poll_rising_edge() {
            push(buf, &mut n, ButtonEvent::Up);
        }
        if self.down.poll_rising_edge() {
            push(buf, &mut n, ButtonEvent::Down);
        }
        if self.ok.poll_rising_edge() {
            push(buf, &mut n, ButtonEvent::Ok);
        }
        if self.back.poll_rising_edge() {
            push(buf, &mut n, ButtonEvent::Back);
        }
        &buf[..n]
    }
}
