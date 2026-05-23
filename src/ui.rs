//! SSD1306 OLED + menu state machine.
//!
//! Display layout (128 × 64, 4 rows of 8x16 font / 8 rows of 6x8 font):
//!
//!   Main screen:
//!     ┌──────────────────────┐
//!     │  CW Keyer            │  ← title row
//!     │  Mode: IambicB       │
//!     │  WPM:  18            │
//!     │  CQ DE SP5...        │  ← decoder (last ~20 chars; hidden
//!     └──────────────────────┘     when `KeyerConfig::decoder_enabled
//!                                  == false`)
//!
//!   Menu screen (scrollable):
//!     ┌──────────────────────┐
//!     │  > Mode      IambicB │  ← selected
//!     │    WPM       18      │
//!     │    Weight    50      │
//!     │    Sidetone  600     │
//!     └──────────────────────┘

use core::fmt::Write as _;

use embassy_rp::i2c::{Blocking, I2c};
use embassy_rp::peripherals::I2C0;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use heapless::String;
use ssd1306::I2CDisplayInterface;
use ssd1306::Ssd1306;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::*;

use radio_utils_keyer::{KeyerConfig, KeyerMode, UsbEmitStyle};

use crate::buttons::ButtonEvent;

pub type OledI2c = I2c<'static, I2C0, Blocking>;

type OledDisplay =
    Ssd1306<I2CInterface<OledI2c>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

/// Cursor position inside the settings menu.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MenuItem {
    Mode,
    Wpm,
    Weight,
    SidetoneFreq,
    SidetoneVolume,
    HangTimeMs,
    KeysReversed,
    IambicBPercent,
    FarnsworthWpm,
    KeyingCompMs,
    UsbEmit,
    Decoder,
}

impl MenuItem {
    /// Display order, mirroring the on-screen list.
    pub const ALL: &'static [MenuItem] = &[
        MenuItem::Mode,
        MenuItem::Wpm,
        MenuItem::Weight,
        MenuItem::SidetoneFreq,
        MenuItem::SidetoneVolume,
        MenuItem::HangTimeMs,
        MenuItem::KeysReversed,
        MenuItem::IambicBPercent,
        MenuItem::FarnsworthWpm,
        MenuItem::KeyingCompMs,
        MenuItem::UsbEmit,
        MenuItem::Decoder,
    ];

    pub fn label(self) -> &'static str {
        match self {
            MenuItem::Mode => "Mode",
            MenuItem::Wpm => "WPM",
            MenuItem::Weight => "Weight",
            MenuItem::SidetoneFreq => "Tone Hz",
            MenuItem::SidetoneVolume => "Volume",
            MenuItem::HangTimeMs => "Hang ms",
            MenuItem::KeysReversed => "Reverse",
            MenuItem::IambicBPercent => "B Time %",
            MenuItem::FarnsworthWpm => "Farns WPM",
            MenuItem::KeyingCompMs => "Comp ms",
            MenuItem::UsbEmit => "USB emit",
            MenuItem::Decoder => "Decoder",
        }
    }

    /// `true` for items whose value space is short enough that
    /// cycling on every OK press is friendlier than entering a
    /// dedicated edit screen.  OK on these items advances to the next
    /// option in-place; numeric/ranged items still open the Edit screen.
    pub fn cycles_on_ok(self) -> bool {
        matches!(
            self,
            MenuItem::Mode | MenuItem::KeysReversed | MenuItem::UsbEmit | MenuItem::Decoder
        )
    }
}

/// Top-level screen the UI is currently showing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Screen {
    /// Live status: mode, WPM, sidetone freq.
    Main,
    /// Settings list, cursor at `selected` index of `MenuItem::ALL`.
    Menu { selected: u8 },
    /// Editing the value of `MenuItem::ALL[selected]`.
    Edit { selected: u8 },
}

pub struct Ui {
    display: OledDisplay,
    screen: Screen,
    /// Latched dirty flag so the UI only redraws when something changed.
    /// `true` immediately after construction so we paint the first frame.
    dirty: bool,
    /// Set whenever a button press mutates `KeyerConfig` (via `adjust`).
    /// Consumed by `take_config_changed()` so the UI task can fire the
    /// "settings dirty" signal exactly once per edit batch.
    config_changed: bool,
    /// Drawn periodically as a small "*" indicator in the top-right so
    /// the user can tell at a glance whether the keyer is actively
    /// keying without staring at the LED.
    last_key_indicator: bool,
    /// Cached decoded text from the CW decoder. Updated by
    /// `note_decoded`; rendered on the main screen's bottom row when
    /// `KeyerConfig::decoder_enabled` is true.
    decoded: heapless::Vec<u8, 32>,
}

impl Ui {
    pub async fn new(i2c: OledI2c) -> Self {
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        // `init` is sync (blocking I2C transactions) in ssd1306 0.10 —
        // we don't have an async builder.  Fine: it runs once at boot.
        //
        // Log on failure so a missing / misaddressed OLED is obvious
        // from RTT; the keyer engine still runs without an attached
        // display, but every subsequent draw call no-ops silently.
        if display.init().is_err() {
            defmt::warn!("SSD1306 init failed — OLED not detected on I2C0?");
        }
        let _ = display.clear(BinaryColor::Off);
        let _ = display.flush();

        Self {
            display,
            screen: Screen::Main,
            dirty: true,
            config_changed: false,
            last_key_indicator: false,
            decoded: heapless::Vec::new(),
        }
    }

    /// Update the cached decoded text. Flags the UI as dirty (so the
    /// next `render` call repaints) only when the main screen is
    /// active and the text actually changed — menu / edit screens
    /// don't show the decoder.
    pub fn note_decoded(&mut self, text: &[u8]) {
        if self.decoded.as_slice() == text {
            return;
        }
        // Callers pass at most HISTORY_CAPACITY bytes (the decoder's
        // snapshot is sized to that), matching this Vec's capacity.
        // The `min` clamp keeps the slice in bounds defensively;
        // `debug_assert!` surfaces a future caller mismatch in dev
        // without panicking embedded production firmware.
        debug_assert!(text.len() <= self.decoded.capacity());
        self.decoded.clear();
        let _ = self
            .decoded
            .extend_from_slice(&text[..text.len().min(self.decoded.capacity())]);
        if matches!(self.screen, Screen::Main) {
            self.dirty = true;
        }
    }

    /// Handle a button event by mutating the screen state and (where
    /// appropriate) the shared `KeyerConfig`.  Returns `true` if the
    /// display should be redrawn.
    pub fn on_button(&mut self, ev: ButtonEvent, config: &mut KeyerConfig) -> bool {
        match self.screen {
            Screen::Main => match ev {
                ButtonEvent::Ok => {
                    self.screen = Screen::Menu { selected: 0 };
                    self.dirty = true;
                }
                ButtonEvent::Up | ButtonEvent::Down => {
                    // Quick WPM adjustment from the main screen — same
                    // direction sense as inside the menu.
                    adjust(config, MenuItem::Wpm, ev == ButtonEvent::Up);
                    self.dirty = true;
                    self.config_changed = true;
                }
                ButtonEvent::Back => {}
            },
            Screen::Menu { selected } => match ev {
                ButtonEvent::Up => {
                    let n = MenuItem::ALL.len() as u8;
                    self.screen = Screen::Menu {
                        selected: (selected + n - 1) % n,
                    };
                    self.dirty = true;
                }
                ButtonEvent::Down => {
                    let n = MenuItem::ALL.len() as u8;
                    self.screen = Screen::Menu {
                        selected: (selected + 1) % n,
                    };
                    self.dirty = true;
                }
                ButtonEvent::Ok => {
                    // Short-list items cycle in-place on OK so the
                    // user doesn't have to open an Edit screen just
                    // to flip a 2- or 6-way enum.  Numeric items
                    // (WPM, Tone Hz, etc.) still open Edit because
                    // cycling them with OK presses would be tedious.
                    let item = MenuItem::ALL[selected as usize];
                    if item.cycles_on_ok() {
                        adjust(config, item, true);
                        self.config_changed = true;
                    } else {
                        self.screen = Screen::Edit { selected };
                    }
                    self.dirty = true;
                }
                ButtonEvent::Back => {
                    self.screen = Screen::Main;
                    self.dirty = true;
                }
            },
            Screen::Edit { selected } => match ev {
                ButtonEvent::Up => {
                    adjust(config, MenuItem::ALL[selected as usize], true);
                    self.dirty = true;
                    self.config_changed = true;
                }
                ButtonEvent::Down => {
                    adjust(config, MenuItem::ALL[selected as usize], false);
                    self.dirty = true;
                    self.config_changed = true;
                }
                ButtonEvent::Ok | ButtonEvent::Back => {
                    self.screen = Screen::Menu { selected };
                    self.dirty = true;
                }
            },
        }
        self.dirty
    }

    /// Returns `true` once per edit batch (then clears).  The UI task
    /// pulls this each tick to decide whether to fire the persistence
    /// dirty-signal.
    pub fn take_config_changed(&mut self) -> bool {
        let v = self.config_changed;
        self.config_changed = false;
        v
    }

    /// True iff the next `render` call has work to do; lets the UI task
    /// skip the (heap-allocating) config snapshot when nothing on
    /// screen has changed.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Force a redraw on the next render (used when the keyer task
    /// changes the live key indicator).
    pub fn note_key_state(&mut self, keyed: bool) {
        if keyed != self.last_key_indicator {
            self.last_key_indicator = keyed;
            // Only the main screen shows the indicator — no point in
            // forcing a menu redraw for it.
            if matches!(self.screen, Screen::Main) {
                self.dirty = true;
            }
        }
    }

    /// Redraw if state changed since the previous frame.  Cheap to call
    /// at 20 Hz from the UI task — does nothing when the buffer is
    /// already in sync with the on-screen image.
    pub fn render(&mut self, config: &KeyerConfig) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let _ = self.display.clear(BinaryColor::Off);
        match self.screen {
            Screen::Main => self.draw_main(config),
            Screen::Menu { selected } => self.draw_menu(config, selected, false),
            Screen::Edit { selected } => self.draw_menu(config, selected, true),
        }
        let _ = self.display.flush();
    }

    fn draw_main(&mut self, cfg: &KeyerConfig) {
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        let title: String<24> = if self.last_key_indicator {
            String::try_from("CW Keyer    *").unwrap_or_default()
        } else {
            String::try_from("CW Keyer").unwrap_or_default()
        };
        let _ = Text::with_baseline(&title, Point::new(0, 0), style, Baseline::Top)
            .draw(&mut self.display);

        let mut line: String<24> = String::new();
        let _ = write!(&mut line, "Mode: {}", mode_label(cfg.mode));
        let _ = Text::with_baseline(&line, Point::new(0, 16), style, Baseline::Top)
            .draw(&mut self.display);

        line.clear();
        let _ = write!(&mut line, "WPM:  {}", cfg.speed_wpm);
        let _ = Text::with_baseline(&line, Point::new(0, 30), style, Baseline::Top)
            .draw(&mut self.display);

        // Decoded CW text on the bottom row, when enabled. We show the
        // last MAX_VISIBLE bytes — at 6 px/glyph the 128 px display
        // fits ~21 columns, so a 32-char history is always rendered as
        // its trailing slice (most recently decoded characters).
        if cfg.decoder_enabled && !self.decoded.is_empty() {
            const MAX_VISIBLE: usize = 21;
            let slice = &self.decoded[self.decoded.len().saturating_sub(MAX_VISIBLE)..];
            // ASCII-only (decoder emits letters / digits / space + a
            // few punctuation marks), so byte → char cast is safe.
            let mut s: String<MAX_VISIBLE> = String::new();
            for &b in slice {
                let _ = s.push(b as char);
            }
            let _ = Text::with_baseline(&s, Point::new(0, 44), style, Baseline::Top)
                .draw(&mut self.display);
        }
    }

    fn draw_menu(&mut self, cfg: &KeyerConfig, selected: u8, editing: bool) {
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        // Render a viewport of 5 items around `selected`.
        const VIEWPORT: usize = 5;
        let total = MenuItem::ALL.len();
        let sel = selected as usize;
        let start = sel
            .saturating_sub(VIEWPORT / 2)
            .min(total.saturating_sub(VIEWPORT));
        let end = (start + VIEWPORT).min(total);

        for (row, idx) in (start..end).enumerate() {
            let item = MenuItem::ALL[idx];
            let mut line: String<32> = String::new();
            let marker = if idx == sel {
                if editing { "*" } else { ">" }
            } else {
                " "
            };
            let _ = write!(
                &mut line,
                "{}{:<9}{}",
                marker,
                item.label(),
                value_string::<16>(cfg, item).as_str()
            );
            let y = (row as i32) * 12;
            let _ = Text::with_baseline(&line, Point::new(0, y), style, Baseline::Top)
                .draw(&mut self.display);
        }
    }
}

fn mode_label(m: KeyerMode) -> &'static str {
    match m {
        KeyerMode::Straight => "Straight",
        KeyerMode::IambicA => "IambicA",
        KeyerMode::IambicB => "IambicB",
        KeyerMode::Bug => "Bug",
        KeyerMode::Ultimatic => "Ultimatic",
        KeyerMode::SinglePaddle => "Single",
    }
}

fn value_string<const N: usize>(cfg: &KeyerConfig, item: MenuItem) -> String<N> {
    let mut s: String<N> = String::new();
    match item {
        MenuItem::Mode => {
            let _ = s.push_str(mode_label(cfg.mode));
        }
        MenuItem::Wpm => {
            let _ = write!(&mut s, "{}", cfg.speed_wpm);
        }
        MenuItem::Weight => {
            let _ = write!(&mut s, "{}", cfg.weight);
        }
        MenuItem::SidetoneFreq => {
            let _ = write!(&mut s, "{}", cfg.sidetone_freq);
        }
        MenuItem::SidetoneVolume => {
            let _ = write!(&mut s, "{}", (cfg.sidetone_volume * 100.0) as u32);
        }
        MenuItem::HangTimeMs => {
            let _ = write!(&mut s, "{}", cfg.hang_time_ms);
        }
        MenuItem::KeysReversed => {
            let _ = s.push_str(if cfg.keys_reversed { "Y" } else { "N" });
        }
        MenuItem::IambicBPercent => {
            let _ = write!(&mut s, "{}", cfg.iambic_b_timing_percent);
        }
        MenuItem::FarnsworthWpm => {
            let _ = write!(&mut s, "{}", cfg.farnsworth_wpm);
        }
        MenuItem::KeyingCompMs => {
            let _ = write!(&mut s, "{}", cfg.keying_compensation_ms);
        }
        MenuItem::UsbEmit => {
            let _ = s.push_str(match cfg.usb_emit_style {
                UsbEmitStyle::Paddle => "Paddle",
                UsbEmitStyle::Keyed => "Keyed",
            });
        }
        MenuItem::Decoder => {
            let _ = s.push_str(if cfg.decoder_enabled { "On" } else { "Off" });
        }
    }
    s
}

/// Apply a single increment / decrement step to the indicated config
/// field.  Steps + bounds match what the menu's existing default range
/// implies in the keyer crate.
fn adjust(cfg: &mut KeyerConfig, item: MenuItem, up: bool) {
    match item {
        MenuItem::Mode => {
            cfg.mode = next_mode(cfg.mode, up);
        }
        MenuItem::Wpm => {
            cfg.speed_wpm = step_u8(cfg.speed_wpm, 1, 5, 60, up);
        }
        MenuItem::Weight => {
            cfg.weight = step_u8(cfg.weight, 1, 25, 75, up);
        }
        MenuItem::SidetoneFreq => {
            cfg.sidetone_freq = step_u32(cfg.sidetone_freq, 25, 300, 1200, up);
        }
        MenuItem::SidetoneVolume => {
            let pct = (cfg.sidetone_volume * 100.0) as i32;
            let next = (pct + if up { 5 } else { -5 }).clamp(0, 100) as f32 / 100.0;
            cfg.sidetone_volume = next;
        }
        MenuItem::HangTimeMs => {
            cfg.hang_time_ms = step_u32(cfg.hang_time_ms, 50, 0, 2000, up);
        }
        MenuItem::KeysReversed => {
            cfg.keys_reversed = !cfg.keys_reversed;
        }
        MenuItem::IambicBPercent => {
            cfg.iambic_b_timing_percent = step_u8(cfg.iambic_b_timing_percent, 5, 0, 100, up);
        }
        MenuItem::FarnsworthWpm => {
            cfg.farnsworth_wpm = step_u8(cfg.farnsworth_wpm, 1, 0, 60, up);
        }
        MenuItem::KeyingCompMs => {
            cfg.keying_compensation_ms = step_u8(cfg.keying_compensation_ms, 1, 0, 50, up);
        }
        MenuItem::UsbEmit => {
            // Toggle — Up/Down both flip; only two values exist.
            let _ = up;
            cfg.usb_emit_style = match cfg.usb_emit_style {
                UsbEmitStyle::Paddle => UsbEmitStyle::Keyed,
                UsbEmitStyle::Keyed => UsbEmitStyle::Paddle,
            };
        }
        MenuItem::Decoder => {
            let _ = up;
            cfg.decoder_enabled = !cfg.decoder_enabled;
        }
    }
}

fn next_mode(m: KeyerMode, up: bool) -> KeyerMode {
    const ORDER: [KeyerMode; 6] = [
        KeyerMode::Straight,
        KeyerMode::IambicA,
        KeyerMode::IambicB,
        KeyerMode::Bug,
        KeyerMode::Ultimatic,
        KeyerMode::SinglePaddle,
    ];
    let idx = ORDER.iter().position(|&x| x == m).unwrap_or(2);
    let n = ORDER.len();
    let next = if up { (idx + 1) % n } else { (idx + n - 1) % n };
    ORDER[next]
}

fn step_u8(v: u8, step: u8, lo: u8, hi: u8, up: bool) -> u8 {
    if up {
        v.saturating_add(step).min(hi)
    } else {
        v.saturating_sub(step).max(lo)
    }
}

fn step_u32(v: u32, step: u32, lo: u32, hi: u32, up: bool) -> u32 {
    if up {
        v.saturating_add(step).min(hi)
    } else {
        v.saturating_sub(step).max(lo)
    }
}
