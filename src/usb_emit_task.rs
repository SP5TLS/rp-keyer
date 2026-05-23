//! Low-priority USB emit task.
//!
//! Consumes [`UsbSnapshot`]s produced by the keyer task and writes
//! MIDI / CDC SERIAL_STATE transitions to the host.  Runs on the thread
//! executor (not the interrupt executor that owns the keyer loop), so a
//! USB endpoint poll wait (≤ 1 ms per write at full-speed interrupt
//! cadence) cannot stretch the keyer's 250 µs poll interval.
//!
//! Signals from the keyer are latest-wins (lossy); `prev_*` here caches
//! the last successfully-written state so a write failure (host
//! disconnect, endpoint stall) doesn't lose the edge — the next signal
//! re-emits the diff.
//!
//! Wire contract (same as the legacy cw-adapter path):
//!
//! * **Paddle style** — two MIDI notes (60 = dit, 62 = dah) and two
//!   serial bits (DCD = dit, DSR = dah) track paddle press/release
//!   directly.
//! * **Keyed style** — note 60 and DCD both carry the engine's keyed
//!   line. Note 62 / DSR are unused; Paddle-mode hosts still see a
//!   single bit toggle per element on DCD.

#[cfg(feature = "serial")]
use crate::cdc_serial_state::CdcWithSerialState;
#[cfg(feature = "midi")]
use crate::midi_interrupt::MidiInterruptClass;

use embassy_usb::driver::Driver;
use radio_utils_keyer::UsbEmitStyle;

use crate::shared::{UsbSnapshot, UsbStateSignal};

#[cfg(feature = "midi")]
const MIDI_NOTE_DIT: u8 = 60;
#[cfg(feature = "midi")]
const MIDI_NOTE_DAH: u8 = 62;

#[cfg(feature = "midi")]
const MIDI_NOTE_ON: u8 = 0x90;
#[cfg(feature = "midi")]
const MIDI_NOTE_OFF: u8 = 0x80;
#[cfg(feature = "midi")]
const MIDI_CIN_NOTE_ON: u8 = 0x09;
#[cfg(feature = "midi")]
const MIDI_CIN_NOTE_OFF: u8 = 0x08;

/// Carrier for the USB transports the emit task owns.  Fields are
/// cfg-gated so the bin file constructs whichever subset of
/// `serial` / `midi` was compiled in.
pub struct UsbEmitApp<'a, D: Driver<'a>> {
    #[cfg(feature = "serial")]
    pub serial: Option<CdcWithSerialState<'a, D>>,
    #[cfg(feature = "midi")]
    pub midi: Option<MidiInterruptClass<'a, D>>,
    /// Keep `'a` and `D` referenced when neither transport is enabled
    /// so the struct still compiles in that (degenerate) configuration.
    #[cfg(not(any(feature = "serial", feature = "midi")))]
    pub _marker: core::marker::PhantomData<&'a D>,
}

impl<'a, D: Driver<'a>> UsbEmitApp<'a, D> {
    /// Drain `signal` forever, writing edges to the available USB
    /// transports.  Never returns.
    pub async fn run(&mut self, signal: &'static UsbStateSignal) -> ! {
        #[cfg(feature = "serial")]
        let (mut prev_dit_ser, mut prev_dah_ser) = (false, false);
        #[cfg(feature = "serial")]
        let mut prev_key_ser: bool = false;
        #[cfg(feature = "midi")]
        let (mut prev_dit_midi, mut prev_dah_midi) = (false, false);
        #[cfg(feature = "midi")]
        let mut prev_key_midi: bool = false;
        let mut prev_style: Option<UsbEmitStyle> = None;

        loop {
            let snap: UsbSnapshot = signal.wait().await;

            // Style switch — release every dit/dah note and DCD/DSR bit
            // so the host doesn't see anything stuck from the previous
            // style.  Best-effort: on Err the next edge in the new
            // style will eventually re-sync host state on its own.
            if prev_style != Some(snap.emit_style) {
                #[cfg(feature = "serial")]
                if let Some(ref mut ser) = self.serial {
                    let _ = ser.send_serial_state(false, false).await;
                    prev_dit_ser = false;
                    prev_dah_ser = false;
                    prev_key_ser = false;
                }
                #[cfg(feature = "midi")]
                if let Some(ref mut m) = self.midi {
                    let _ = m
                        .write_packet(&[
                            MIDI_CIN_NOTE_OFF,
                            MIDI_NOTE_OFF,
                            MIDI_NOTE_DIT,
                            0x00, //
                            MIDI_CIN_NOTE_OFF,
                            MIDI_NOTE_OFF,
                            MIDI_NOTE_DAH,
                            0x00,
                        ])
                        .await;
                    prev_dit_midi = false;
                    prev_dah_midi = false;
                    prev_key_midi = false;
                }
                prev_style = Some(snap.emit_style);
            }

            // Run the CDC and MIDI writes concurrently via `join` so
            // MIDI latency is independent of the CDC interrupt-IN poll
            // schedule.  At 1 ms poll each on both endpoints, the total
            // per-signal latency is `max(CDC, MIDI) ≈ 1 ms` instead of
            // `CDC + MIDI ≈ 2 ms` when serialised.
            //
            // The two futures borrow disjoint fields (`self.serial`,
            // `self.midi`) so the borrow checker accepts the parallel
            // mutable borrow.  `prev_*_ser` and `prev_*_midi` are local
            // so they're also disjoint.
            #[cfg(all(feature = "serial", feature = "midi"))]
            let ser_slot = &mut self.serial;
            #[cfg(all(feature = "serial", feature = "midi"))]
            let midi_slot = &mut self.midi;

            #[cfg(feature = "serial")]
            #[cfg(not(feature = "midi"))]
            let ser_slot = &mut self.serial;
            #[cfg(feature = "midi")]
            #[cfg(not(feature = "serial"))]
            let midi_slot = &mut self.midi;

            #[cfg(feature = "serial")]
            let ser_fut = async {
                match snap.emit_style {
                    UsbEmitStyle::Paddle => {
                        if let Some(ref mut ser) = *ser_slot
                            && (snap.dit_pressed != prev_dit_ser
                                || snap.dah_pressed != prev_dah_ser)
                            && ser
                                .send_serial_state(snap.dit_pressed, snap.dah_pressed)
                                .await
                                .is_ok()
                        {
                            prev_dit_ser = snap.dit_pressed;
                            prev_dah_ser = snap.dah_pressed;
                        }
                    }
                    UsbEmitStyle::Keyed => {
                        if let Some(ref mut ser) = *ser_slot
                            && snap.live_key != prev_key_ser
                            && ser.send_serial_state(snap.live_key, false).await.is_ok()
                        {
                            prev_key_ser = snap.live_key;
                        }
                    }
                }
            };

            #[cfg(feature = "midi")]
            let midi_fut = async {
                match snap.emit_style {
                    UsbEmitStyle::Paddle => {
                        if let Some(ref mut m) = *midi_slot {
                            let dit_changed = snap.dit_pressed != prev_dit_midi;
                            let dah_changed = snap.dah_pressed != prev_dah_midi;
                            if dit_changed || dah_changed {
                                let mut buf = [0u8; 8];
                                let mut len = 0;
                                if dit_changed {
                                    buf[len..len + 4].copy_from_slice(if snap.dit_pressed {
                                        &[MIDI_CIN_NOTE_ON, MIDI_NOTE_ON, MIDI_NOTE_DIT, 0x7F]
                                    } else {
                                        &[MIDI_CIN_NOTE_OFF, MIDI_NOTE_OFF, MIDI_NOTE_DIT, 0x00]
                                    });
                                    len += 4;
                                }
                                if dah_changed {
                                    buf[len..len + 4].copy_from_slice(if snap.dah_pressed {
                                        &[MIDI_CIN_NOTE_ON, MIDI_NOTE_ON, MIDI_NOTE_DAH, 0x7F]
                                    } else {
                                        &[MIDI_CIN_NOTE_OFF, MIDI_NOTE_OFF, MIDI_NOTE_DAH, 0x00]
                                    });
                                    len += 4;
                                }
                                if m.write_packet(&buf[..len]).await.is_ok() {
                                    if dit_changed {
                                        prev_dit_midi = snap.dit_pressed;
                                    }
                                    if dah_changed {
                                        prev_dah_midi = snap.dah_pressed;
                                    }
                                }
                            }
                        }
                    }
                    UsbEmitStyle::Keyed => {
                        // One MIDI note (dit) carries the engine's
                        // keyed line — note 62 stays silent so a
                        // single-note Keyed listener doesn't pick up
                        // two simultaneous notes per element.
                        if let Some(ref mut m) = *midi_slot
                            && snap.live_key != prev_key_midi
                        {
                            let buf: [u8; 4] = if snap.live_key {
                                [MIDI_CIN_NOTE_ON, MIDI_NOTE_ON, MIDI_NOTE_DIT, 0x7F]
                            } else {
                                [MIDI_CIN_NOTE_OFF, MIDI_NOTE_OFF, MIDI_NOTE_DIT, 0x00]
                            };
                            if m.write_packet(&buf).await.is_ok() {
                                prev_key_midi = snap.live_key;
                            }
                        }
                    }
                }
            };

            #[cfg(all(feature = "serial", feature = "midi"))]
            embassy_futures::join::join(ser_fut, midi_fut).await;
            #[cfg(all(feature = "serial", not(feature = "midi")))]
            ser_fut.await;
            #[cfg(all(feature = "midi", not(feature = "serial")))]
            midi_fut.await;
        }
    }
}
