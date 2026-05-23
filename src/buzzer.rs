//! PWM-driven passive piezo buzzer for sidetone.
//!
//! Drives the buzzer pin (PWM channel B) at the configured sidetone
//! frequency with a 50 % square wave while keyed, and parks duty at 0 %
//! when idle so the buzzer is silent.
//!
//! Frequency is set via the (divider, top) pair so a single PWM slice
//! covers the full sidetone range (~300–1200 Hz) without straining the
//! 16-bit counter at low frequencies.

use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use fixed::FixedU16;
use fixed::types::extra::U4;

pub struct Buzzer {
    pwm: Pwm<'static>,
    cfg: PwmConfig,
    /// System clock frequency the PWM slice runs against.  Read once
    /// from `embassy_rp::clocks::clk_sys_freq()` in `new` rather than
    /// hard-coded per chip, so a custom `embassy_rp::config::Config`
    /// (or a future embassy-rp version that changes the default
    /// `clk_sys`) can't desync the sidetone pitch.
    sys_clk_hz: u32,
    /// Sidetone frequency currently programmed into the PWM hardware.
    /// Tracked so that a config change can be detected without
    /// reprogramming the slice every tick.
    current_freq_hz: u32,
    /// Volume programmed into the PWM. Implemented as duty cycle from
    /// 0 (silent) up to 50 % (loudest square wave).
    current_volume: f32,
    /// Whether the carrier is currently being driven.
    keying: bool,
}

impl Buzzer {
    /// Construct from a configured PWM slice. Must be called *after*
    /// `embassy_rp::init()` so `clk_sys_freq()` returns the live sys
    /// clock; otherwise the slice atomic still reads zero.
    pub fn new(pwm: Pwm<'static>) -> Self {
        Self {
            pwm,
            cfg: PwmConfig::default(),
            sys_clk_hz: embassy_rp::clocks::clk_sys_freq(),
            current_freq_hz: 0,
            current_volume: 0.0,
            keying: false,
        }
    }

    /// Update programmed frequency / volume if either differs from the
    /// current setting.  Cheap to call every keyer tick — the hardware
    /// is only reprogrammed on actual change.
    pub fn update_settings(&mut self, freq_hz: u32, volume: f32) {
        // Reject NaN; clamp finite values into [0, 1].
        let v = if volume.is_nan() {
            0.0
        } else {
            volume.clamp(0.0, 1.0)
        };
        let freq = freq_hz.max(50);
        if freq == self.current_freq_hz && (v - self.current_volume).abs() < f32::EPSILON {
            return;
        }
        self.current_freq_hz = freq;
        self.current_volume = v;
        self.program();
    }

    /// Gate the PWM output: `true` engages the configured tone, `false`
    /// silences it (compare_b = 0).
    pub fn set_keyed(&mut self, on: bool) {
        if self.keying == on {
            return;
        }
        self.keying = on;
        self.program();
    }

    fn program(&mut self) {
        // PWM slice clock = sys_clk_hz divided by `divider`.  Pick a
        // divider that yields a `top` in [1000, 50000] for the requested
        // frequency, keeping resolution high without overflowing the
        // 16-bit counter.
        //
        // We aim for slice_clk ≈ freq * 4096:
        //     divider ≈ sys_clk / (freq * 4096)
        // Clamped to [1, 255].
        let target_slice_clk = self.current_freq_hz.saturating_mul(4096).max(1);
        let div_int = (self.sys_clk_hz / target_slice_clk).clamp(1, 255) as u8;
        let slice_clk = self.sys_clk_hz / div_int as u32;
        let top = (slice_clk / self.current_freq_hz.max(1)).clamp(2, u16::MAX as u32) as u16;

        // 50 % duty = top / 2 → square wave (loudest tone from a piezo).
        // Scale by volume; floor at 0 when silenced.
        //
        // `compare_b == 0` plus channel-B output enabled & non-inverted
        // (the embassy-rp `Pwm::new_output_b` default) parks the pin
        // low for the entire period — that's the silent state we
        // want.  If anyone ever flips the channel polarity, audit
        // `set_keyed(false)` and `safe_stop` together.
        let max_duty = (top / 2) as u32;
        let compare_b = if self.keying {
            (max_duty as f32 * self.current_volume) as u16
        } else {
            0
        };

        let top_shrunk = top < self.cfg.top;
        self.cfg.divider = FixedU16::<U4>::from_num(div_int);
        self.cfg.top = top;
        self.cfg.compare_b = compare_b;
        self.pwm.set_config(&self.cfg);

        if top_shrunk {
            // If TOP shrinks below the live CTR, the slice has to count
            // up to 0xFFFF before wrapping — that can be tens of ms of
            // stuck-low output mid-element.  Reset the counter so the
            // new period takes effect immediately.
            embassy_rp::pac::PWM
                .ch(crate::BUZZER_PWM_SLICE)
                .ctr()
                .write_value(embassy_rp::pac::pwm::regs::ChCtr(0));
        }
    }
}
