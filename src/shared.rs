//! Shared state between the high-priority keyer task, the
//! low-priority USB emit / UI tasks.
//!
//! Two channels of cross-task state:
//!
//! * [`SharedConfig`] — `Mutex<KeyerConfig>` mutated by the UI on button
//!   presses, snapshotted by the keyer task every ~50 ms.  Uses
//!   `CriticalSectionRawMutex` so the brief locks taken by the UI never
//!   block IRQ-mode reads from the keyer task long-term.
//! * [`UsbStateSignal`] — `Signal<UsbSnapshot>` driven by the keyer task
//!   on every state change and consumed by the USB emit task.  Signals
//!   are lossy by design (latest wins); the consumer caches `prev_*` so
//!   a write failure doesn't drop an edge.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use radio_utils_cw_decoder::Decoder as CwDecoder;
use radio_utils_keyer::{KeyerConfig, UsbEmitStyle};

/// Global `KeyerConfig` shared between UI and keyer task.
///
/// Initialised once at boot in `main`.  Both readers (keyer task) and
/// writers (UI) lock briefly: the UI to mutate a single field after a
/// button press, the keyer task to clone the struct ~once per 50 ms.
/// The clone is heap-free at boot since `KeyerConfig::default()`'s
/// `String` fields are empty and the firmware never populates them.
pub type SharedConfig = Mutex<CriticalSectionRawMutex, KeyerConfig>;

/// Per-iteration snapshot of the keyer's USB-facing state.
///
/// Produced by the high-priority keyer task and consumed by the
/// low-priority `usb_emit_task`, so that the keyer loop never blocks
/// on a USB endpoint poll (≤ 1 ms) and the engine's tick cadence stays
/// faithful to wall-clock time even under USB host stalls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbSnapshot {
    /// Logical dit paddle state (post `keys_reversed` swap).
    pub dit_pressed: bool,
    /// Logical dah paddle state (post `keys_reversed` swap).
    pub dah_pressed: bool,
    /// Engine's current keyed-line state — toggles per Morse element.
    pub live_key: bool,
    /// Which emit style the keyer is currently configured for.
    pub emit_style: UsbEmitStyle,
}

/// Latest-wins channel from keyer task → USB emit task.  See module
/// docstring for the rationale; `CriticalSectionRawMutex` because the
/// producer runs in an interrupt-executor and the consumer in the
/// thread executor.
pub type UsbStateSignal = Signal<CriticalSectionRawMutex, UsbSnapshot>;

/// "Config has been edited and is waiting to be persisted to flash."
/// Signalled by the UI task on every value change, consumed by the
/// `config_persist_task` which then debounces + waits for engine idle
/// before committing.
pub type ConfigDirtySignal = Signal<CriticalSectionRawMutex, ()>;

/// CW decoder shared between the IRQ-priority keyer task (writer:
/// feeds key transitions on every KeyDown/KeyUp emitted by the engine)
/// and the thread-executor UI task (reader: polls + snapshots into the
/// OLED main-screen text row at the UI's 25 ms cadence).
///
/// A `blocking_mutex` with `CriticalSectionRawMutex` is the right
/// primitive here — every access is a tiny closure (a single
/// `on_transition` or `poll` + `snapshot`), well under a microsecond,
/// and the critical-section guard keeps the lock interrupt-safe so the
/// keyer task on SWI_IRQ_1 can take it without deadlocking against the
/// UI task on the thread executor.
pub type SharedDecoder = BlockingMutex<CriticalSectionRawMutex, RefCell<CwDecoder>>;
