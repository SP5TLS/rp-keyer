//! Paddle GPIO reads with software debouncing.
//!
//! Copied verbatim from `cw-adapter-rp::common::Debouncer` — same integrator
//! algorithm with an 8-tick threshold against the 1 ms keyer loop, which is
//! the value that's been working in the cw-adapter on real hardware.

/// Integration-style debouncer. A change is accepted only after the raw
/// input has read the *new* state for `threshold` consecutive samples.
pub struct Debouncer {
    state: bool,
    integration_counter: u8,
    threshold: u8,
}

impl Debouncer {
    pub fn new(initial_state: bool, threshold: u8) -> Self {
        debug_assert!(threshold > 0, "Debouncer threshold must be > 0");
        Self {
            state: initial_state,
            integration_counter: 0,
            threshold,
        }
    }

    pub fn update(&mut self, raw_state: bool) -> bool {
        if raw_state != self.state {
            self.integration_counter += 1;
            if self.integration_counter >= self.threshold {
                self.state = raw_state;
                self.integration_counter = 0;
            }
        } else {
            self.integration_counter = 0;
        }
        self.state
    }
}
