//! Persistent settings storage in the on-board SPI flash.
//!
//! The last 16 KiB of flash (four 4 KiB sectors, reserved by `build.rs`
//! shrinking the FLASH region) holds a `sequential-storage` map of
//! `KeyerConfig` records.  Sequential-storage's append-then-erase
//! discipline gives us wear-leveling across the four sectors:
//! ~hundreds of saves per erase × 100k erase cycles per sector ≈ tens
//! of millions of saves before the chip wears out.
//!
//! The map carries exactly one key (`KEYER_CONFIG_KEY = 0`); the
//! value is `(SCHEMA_VERSION: u16, KeyerConfig)` encoded with postcard
//! and written through the `&[u8]` Value impl sequential-storage already
//! provides (so we avoid adding `serde` as a direct firmware dep — the
//! version field is a plain u16, which postcard varint-encodes natively).
//!
//! The version prefix lets the loader detect blobs written by a previous
//! firmware version that had a different `KeyerConfig` schema and fall
//! back to defaults instead of silently deserialising into garbage.
//! Bump `SCHEMA_VERSION` whenever `KeyerConfig`'s postcard wire format
//! changes (adding/removing fields, reordering, type changes).
//!
//! Why an async wrapper around the *Blocking* embassy-rp flash driver:
//! sequential-storage 7's traits are async-only.  Embassy-rp does have
//! an async Flash mode but it needs a DMA channel + DMA IRQ binding,
//! and the actual erase/write are blocking under the hood anyway
//! (RP2040 SPI flash IO can't proceed while XIP is fetching code from
//! the same flash).  Wrapping the Blocking driver in an async-fronted
//! shim keeps the binary setup simpler with no behavioural penalty.

use embassy_rp::flash::{Blocking, ERASE_SIZE, Flash, WRITE_SIZE};
use embassy_rp::peripherals::FLASH;
use embedded_storage_async::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use radio_utils_keyer::KeyerConfig;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};

/// Pico-class RP2040 module on-board flash size.  The whole 2 MiB lives
/// at `0x1000_0000`; we reserve the top 16 KiB for settings.
pub const FLASH_TOTAL_SIZE: usize = 2 * 1024 * 1024;

/// Bytes reserved for the settings ring.  4 × 4 KiB sectors — see
/// `build.rs`, which shrinks the FLASH region by the same amount so
/// linker output can't collide with the storage region.
pub const STORAGE_BYTES: u32 = 16 * 1024;

/// Offset (within flash) of the first storage sector.
pub const STORAGE_OFFSET: u32 = FLASH_TOTAL_SIZE as u32 - STORAGE_BYTES;

/// Sequential-storage map key for the single `KeyerConfig` record.
const KEYER_CONFIG_KEY: u8 = 0;

/// Schema version for the stored blob.  Bumped on any breaking
/// `KeyerConfig` wire-format change; `load_config` rejects blobs whose
/// stored version doesn't match this and the caller falls back to
/// `KeyerConfig::default()`.
///
/// History:
/// * v1 — initial release.
/// * v2 — added `KeyerConfig::decoder_enabled` (bool field; postcard's
///   positional wire format breaks on any field addition).
const SCHEMA_VERSION: u16 = 2;

/// Sequential-storage scratch buffer size.  Must comfortably fit
/// `key + postcard(KeyerConfig)`; the firmware never populates the
/// `[String; 8]` macros, optional `serial_port`, `midi_device`, or
/// `Option<char>` paddle keys, so today's serialized form is well under
/// 100 bytes.  512 bytes gives headroom for a future change that
/// populates them (e.g. user-editable macro text).
pub const DATA_BUFFER_SIZE: usize = 512;

/// Wrap the Blocking Flash driver in the async NorFlash traits
/// sequential-storage expects.  All operations forward to the blocking
/// underlying calls; the `async` is purely shape, not concurrency.
pub struct AsyncFlash {
    inner: Flash<'static, FLASH, Blocking, { FLASH_TOTAL_SIZE }>,
}

impl AsyncFlash {
    pub fn new(inner: Flash<'static, FLASH, Blocking, { FLASH_TOTAL_SIZE }>) -> Self {
        Self { inner }
    }
}

impl ErrorType for AsyncFlash {
    type Error = embassy_rp::flash::Error;
}

impl ReadNorFlash for AsyncFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.inner.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.inner.capacity()
    }
}

impl NorFlash for AsyncFlash {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.inner.blocking_erase(from, to)
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.inner.blocking_write(offset, bytes)
    }
}

// ── FlashRef: a thin pass-through that lets MapStorage borrow our
//    AsyncFlash rather than take ownership.  Sequential-storage's
//    constructor takes the flash by value — we want load/save to be
//    reentrant across calls without repeatedly consuming the flash,
//    so a `&mut AsyncFlash`-backed wrapper does the trick.

struct FlashRef<'a> {
    inner: &'a mut AsyncFlash,
}

impl ErrorType for FlashRef<'_> {
    type Error = embassy_rp::flash::Error;
}

impl ReadNorFlash for FlashRef<'_> {
    const READ_SIZE: usize = <AsyncFlash as ReadNorFlash>::READ_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.inner.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.inner.capacity()
    }
}

impl NorFlash for FlashRef<'_> {
    const WRITE_SIZE: usize = <AsyncFlash as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <AsyncFlash as NorFlash>::ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.inner.erase(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.inner.write(offset, bytes).await
    }
}

fn map_config_for_ref<'a>() -> MapConfig<FlashRef<'a>> {
    MapConfig::new(STORAGE_OFFSET..(STORAGE_OFFSET + STORAGE_BYTES))
}

/// Load the most recently persisted `KeyerConfig`.  Returns `None` if
/// no record exists yet (first boot), the flash read errored, the
/// stored bytes can't be deserialized, or the stored schema version
/// doesn't match `SCHEMA_VERSION`.  Caller falls back to
/// `KeyerConfig::default()` in those cases.
pub async fn load_config(flash: &mut AsyncFlash) -> Option<KeyerConfig> {
    let mut buf = [0u8; DATA_BUFFER_SIZE];
    let mut storage = MapStorage::<u8, _, _>::new(
        FlashRef { inner: flash },
        map_config_for_ref(),
        NoCache::new(),
    );
    let bytes: &[u8] = match storage
        .fetch_item::<&[u8]>(&mut buf, &KEYER_CONFIG_KEY)
        .await
    {
        Ok(Some(b)) => b,
        Ok(None) => return None,
        Err(_) => {
            defmt::warn!("settings load: flash read failed");
            return None;
        }
    };
    match postcard::from_bytes::<(u16, KeyerConfig)>(bytes) {
        Ok((version, cfg)) if version == SCHEMA_VERSION => Some(cfg),
        Ok((version, _)) => {
            defmt::warn!(
                "settings load: stored schema {} != current {} — using defaults",
                version,
                SCHEMA_VERSION
            );
            None
        }
        Err(_) => {
            defmt::warn!(
                "settings load: postcard deserialize failed ({} bytes) — using defaults",
                bytes.len()
            );
            None
        }
    }
}

/// Persist `config` to the next slot in the settings ring.  Sleeps the
/// running task for the duration of the flash erase/program (~50 ms in
/// the worst case — one sector erase plus one page write).
pub async fn save_config(flash: &mut AsyncFlash, config: &KeyerConfig) -> Result<(), ()> {
    // Serialize first into a scratch buffer so the actual store_item
    // call doesn't have to interleave its own postcard work with flash
    // I/O.  256 bytes is plenty for the current `KeyerConfig` plus the
    // varint-encoded schema version (1–3 bytes for any practical value).
    let mut serialized = [0u8; 256];
    let payload: (u16, &KeyerConfig) = (SCHEMA_VERSION, config);
    let bytes: &[u8] = match postcard::to_slice(&payload, &mut serialized) {
        Ok(b) => b,
        Err(_) => {
            defmt::warn!("settings save: postcard serialize overflowed scratch buffer");
            return Err(());
        }
    };

    let mut buf = [0u8; DATA_BUFFER_SIZE];
    let mut storage = MapStorage::<u8, _, _>::new(
        FlashRef { inner: flash },
        map_config_for_ref(),
        NoCache::new(),
    );
    storage
        .store_item(&mut buf, &KEYER_CONFIG_KEY, &bytes)
        .await
        .map_err(|_| {
            defmt::warn!("settings save: flash write failed");
        })
}
