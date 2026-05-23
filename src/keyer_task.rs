//! High-priority keyer loop.
//!
//! Runs on the interrupt executor at a 250 µs cadence.  Paddle reads,
//! debouncing, USB-snapshot signalling and engine ticks all happen
//! every 250 µs — the engine is constructed via `new_with_tick(_, 250)`
//! so each `tick()` advances `elapsed_us` by 250 µs and the
//! schedule-phase residual handling is matched to the same quantum.
//!
//! At the previous 1 ms engine quantum, 28 WPM produced a repeating
//! 43-43-43-43-43-43-42 ms dit pattern because `1200 / 28 = 42.857 ms`
//! doesn't fall on a 1 ms grid; the residual built up and the engine
//! emitted a short dit every 7 elements.  At 250 µs that swing shrinks
//! to a single tick (250 µs) — below the audibility threshold and
//! within the bound the schedule-phase carry was designed for.
//!
//! USB I/O does **not** run in this task.  A `Signal<UsbSnapshot>`
//! carries the latest paddle / engine-key state to a separate
//! low-priority `usb_emit_task`, so a USB endpoint poll (≤ 1 ms per
//! interrupt-IN transfer) can never stretch the keyer's 250 µs cadence
//! and warp the engine's perception of wall-clock time.
//!
//! Paddle debounce: `debounce_threshold × 250 µs` — currently ~2 ms
//! (8 samples) to stay above real-world paddle contact bounce.  Tighter
//! thresholds let release-bounce sneak through and the iambic memory
//! turns one tap into two elements (see `dit_debounce` in `run`).

use embassy_rp::gpio::Input;
use embassy_time::{Duration, Instant, Ticker};
use radio_utils_keyer::{KeyerConfig, KeyerEngine, KeyerOutput, UsbEmitStyle};

use crate::buzzer::Buzzer;
use crate::paddle::Debouncer;
use crate::radio_key::RadioKey;
use crate::shared::{SharedConfig, SharedDecoder, UsbSnapshot, UsbStateSignal};

/// Keyer poll + engine tick interval.  Drives both the paddle debounce
/// cadence and the engine's time advance — both updated at this rate.
const POLL_INTERVAL_US: u32 = 250;

/// Config refresh throttle, expressed in ticks.  ~50 ms between
/// re-locks of the shared mutex (200 × 250 µs).
const CONFIG_REFRESH_TICKS: u32 = (50 * 1000) / POLL_INTERVAL_US;

/// Feed a key transition to the CW decoder. Held under the decoder's
/// critical-section lock for just the duration of `on_transition` —
/// O(1) work, well under a microsecond, so it never extends the
/// keyer task's 250 µs cadence.
///
/// Called twice in close succession for each key-down: once from
/// `PttRequest(true)` (one engine tick before the actual element
/// edge) and once from the following `KeyDown`. That matches the
/// on-air key line — `PttRequest(true)` already drives the GPIO low —
/// so the decoder is correctly observing the keyed signal. The second
/// call lands with `state == Down`, so `process_gap` short-circuits;
/// only `last_transition_us` advances by the ~250 µs between the two
/// events, which biases the next gap by the same amount (≈ 0.4% of a
/// dit at 18 WPM — well below decode-threshold sensitivity).
#[inline]
fn feed_decoder(decoder: &'static SharedDecoder, key_down: bool, wpm: u8) {
    let now_us = Instant::now().as_micros();
    decoder.lock(|d| d.borrow_mut().on_transition(now_us, key_down, wpm));
}

/// Main keyer loop.  Never returns.
#[allow(clippy::too_many_arguments)] // each parameter is a distinct piece of wiring with no natural grouping
pub async fn run(
    dit_pin: Input<'static>,
    dah_pin: Input<'static>,
    mut radio_key: RadioKey,
    mut buzzer: Buzzer,
    shared_config: &'static SharedConfig,
    key_indicator: &'static portable_atomic::AtomicBool,
    usb_signal: &'static UsbStateSignal,
    engine_active: &'static portable_atomic::AtomicBool,
    save_lock: &'static portable_atomic::AtomicBool,
    decoder: &'static SharedDecoder,
) -> ! {
    // Seed the engine with the current config snapshot.
    let mut keys_reversed: bool;
    let mut emit_style: UsbEmitStyle;
    let mut wpm: u8;
    let mut decoder_enabled: bool;
    let initial_cfg: KeyerConfig = {
        let guard = shared_config.lock().await;
        keys_reversed = guard.keys_reversed;
        emit_style = guard.usb_emit_style;
        wpm = guard.speed_wpm;
        decoder_enabled = guard.decoder_enabled;
        guard.clone()
    };
    buzzer.update_settings(initial_cfg.sidetone_freq, initial_cfg.sidetone_volume);
    let mut engine = KeyerEngine::new_with_tick(initial_cfg, POLL_INTERVAL_US);

    // 8 consecutive samples ≈ 2 ms debounce.  Mechanical paddles
    // can bounce for 1–3 ms on release; anything shorter lets the
    // bounce-back-to-pressed rising edge through and the iambic
    // engine latches it as a fresh dit_memory during the next
    // inter-element gap, producing a phantom second dit (the
    // classic "double-dit on one tap" failure mode at 18 WPM).
    // 2 ms is far below the shortest intentional dit at 60 WPM
    // (≈ 20 ms) so it costs nothing in element fidelity.
    let mut dit_debounce = Debouncer::new(false, 8);
    let mut dah_debounce = Debouncer::new(false, 8);

    // Live engine key state — flipped by KeyDown/KeyUp/PttRequest(true)
    // and shipped to usb_emit_task via the Signal so the host sees
    // engine-keyed transitions even in Keyed emit mode.
    let mut live_key: bool = false;
    let mut last_signaled: Option<UsbSnapshot> = None;

    // Counts engine ticks for the config-refresh throttle.  Resets to
    // 0 on each fire rather than wrapping a u32 — avoids a spurious
    // refresh at u32::MAX → 0 (the previous `is_multiple_of` check
    // matched 0 too, so the wrap moment triggered an extra refresh).
    let mut refresh_counter: u32 = 0;

    // Fixed-interval ticker rather than `Timer::after(POLL_INTERVAL_US)`
    // — the latter measures from end-of-body, so any per-iteration jitter
    // (engine state branch, periodic config clone, etc.) accumulated as
    // wall-clock drift while the engine's `elapsed_us` advanced by a
    // fixed quantum per tick.  `Ticker` fires on absolute interval
    // boundaries so engine ticks track wall clock.
    let mut ticker = Ticker::every(Duration::from_micros(POLL_INTERVAL_US as u64));

    loop {
        // ── 0. Handshake with the settings-persistence task. ──
        //      `save_lock` is set by `config_persist_task` while it's
        //      about to call `blocking_erase`, which disables IRQs for
        //      ~50 ms.  Skip this iteration entirely (don't touch
        //      paddles, engine, or the key line) so an in-flight
        //      element can't tear when IRQs resume.  Re-storing
        //      `engine_active = false` is what unblocks the persist
        //      task's recheck: it polls this atomic to confirm the
        //      keyer has parked before going into the erase.
        //
        //      Decoder side effect: if the engine had produced a
        //      KeyDown right before the lock fired, the matching KeyUp
        //      is delivered ~50 ms late from the decoder's wall-clock
        //      perspective, which inflates the measured pulse into a
        //      spurious dah. Mis-decode of one character is the
        //      accepted cost — saves are infrequent and the persist
        //      task already waits for engine idle whenever it can.
        if save_lock.load(portable_atomic::Ordering::Relaxed) {
            engine_active.store(false, portable_atomic::Ordering::Relaxed);
            ticker.next().await;
            continue;
        }

        // ── 1. Paddle read (every 250 µs) ─────────────────────
        // Active-low: is_low() == pressed.  Apply keys_reversed
        // immediately so every consumer (engine, USB transports,
        // key indicator) sees the same logical dit/dah.
        let raw_dit = dit_pin.is_low();
        let raw_dah = dah_pin.is_low();
        let dit_phys = dit_debounce.update(raw_dit);
        let dah_phys = dah_debounce.update(raw_dah);
        let (dit_pressed, dah_pressed) = if keys_reversed {
            (dah_phys, dit_phys)
        } else {
            (dit_phys, dah_phys)
        };

        // ── 2. Push the latest USB-facing state to the emit task.
        //      `Signal` is lossy by design — the usb_emit_task caches
        //      `prev_*` so a transient miss doesn't drop an edge; we
        //      only signal on actual change to avoid waking it for
        //      identical snapshots.
        let snapshot = UsbSnapshot {
            dit_pressed,
            dah_pressed,
            live_key,
            emit_style,
        };
        if Some(snapshot) != last_signaled {
            usb_signal.signal(snapshot);
            last_signaled = Some(snapshot);
        }

        // ── 3. Push paddle state to engine every poll so that
        //      iambic memory latches within ~500 µs of the rising
        //      edge.  `set_paddle` itself doesn't advance time — it
        //      only updates the held/memory bits — so calling it
        //      every poll instead of every tick is safe.
        //
        //      The engine swaps internally on keys_reversed; we
        //      pass the *physical* state so that swap remains the
        //      single source of truth for the engine itself even
        //      though the USB path above uses the post-reversal
        //      state explicitly.
        engine.set_paddle(dit_phys, dah_phys);

        // ── 4. Refresh config snapshot every CONFIG_REFRESH_TICKS
        //      (~ 50 ms).  `lock().await` rather than try_lock so we
        //      never silently drop a config update under UI contention.
        refresh_counter += 1;
        if refresh_counter >= CONFIG_REFRESH_TICKS {
            refresh_counter = 0;
            let cfg = shared_config.lock().await.clone();
            keys_reversed = cfg.keys_reversed;
            emit_style = cfg.usb_emit_style;
            wpm = cfg.speed_wpm;
            // Reset the decoder on the off→on edge so a long-frozen
            // `last_transition_us` doesn't fabricate a giant gap (and
            // a leading space) on the first transition after re-enable.
            // `clear` parks state back to Idle, which the next
            // `on_transition` treats as a fresh start.
            if cfg.decoder_enabled && !decoder_enabled {
                decoder.lock(|d| d.borrow_mut().clear());
            }
            decoder_enabled = cfg.decoder_enabled;
            let freq = cfg.sidetone_freq;
            let vol = cfg.sidetone_volume;
            engine.update_config(cfg);
            buzzer.update_settings(freq, vol);
            // No drain_transitions call: the engine's transition log is
            // a fixed-size heapless ring (see radio_utils_keyer::engine)
            // and self-trims at O(1).
        }

        // ── 5. Tick the engine every iteration.  At POLL_INTERVAL_US =
        //      250 µs each tick advances elapsed_us by 250 µs, matching
        //      the engine's `new_with_tick(_, 250)` quantum.
        if let Some(out) = engine.tick() {
            match out {
                // Treat PttRequest(true) as a key-down too — the
                // radio_key GPIO doubles as PTT for full-break-in CW,
                // so we'd otherwise be one engine tick late (~ 250 µs)
                // keying the leading edge of every fresh element.
                // KeyDown follows on the next tick and is idempotent.
                KeyerOutput::KeyDown | KeyerOutput::PttRequest(true) => {
                    radio_key.set(true);
                    buzzer.set_keyed(true);
                    key_indicator.store(true, portable_atomic::Ordering::Relaxed);
                    live_key = true;
                    if decoder_enabled {
                        feed_decoder(decoder, true, wpm);
                    }
                }
                KeyerOutput::KeyUp => {
                    radio_key.set(false);
                    buzzer.set_keyed(false);
                    key_indicator.store(false, portable_atomic::Ordering::Relaxed);
                    live_key = false;
                    if decoder_enabled {
                        feed_decoder(decoder, false, wpm);
                    }
                }
                KeyerOutput::PttRequest(false) => {
                    // Key was already driven low by the matching
                    // KeyUp earlier in the element. No-op here.
                }
            }
        }

        // Publish engine-idle vs active so the settings-persistence
        // task can skip flash writes (each takes ~50 ms with interrupts
        // disabled) while keying is in flight.
        engine_active.store(engine.is_active(), portable_atomic::Ordering::Relaxed);

        ticker.next().await;
    }
}
