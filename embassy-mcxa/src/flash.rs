//! Flash driver using the ROM API.
//!
//! This module provides safe access to the internal flash memory through
//! the ROM-resident flash driver API. The ROM API lives at a fixed address and exposes
//! flash operations (init, erase, program, verify, read) via a function pointer table.
//!
//! # Safety
//!
//! All flash-modifying operations (erase, program) run inside a critical section
//! to prevent interrupts from executing code in flash while it is being modified.
//! After each modifying operation, speculation buffers and the LPCAC are cleared
//! to ensure subsequent reads return fresh data.
//!
//! # Example
//!
//! ```rust,no_run
//! #![no_std]
//! #![no_main]
//!
//! # use panic_halt as _;
//! use embassy_executor::Spawner;
//! use embassy_mcxa::clocks::config::Div8;
//! use embassy_mcxa::config::Config;
//! use embassy_mcxa::flash::Flash;
//! use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     let mut config = Config::default();
//!     config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);
//!
//!     let p = embassy_mcxa::init(config);
//!
//!     let mut flash = Flash::new().unwrap();
//!
//!     // Erase a sector at offset 0xFE000 (near the end of 1 MB flash)
//!     flash.erase(0xFE000, 0x10_0000).unwrap();
//!
//!     // Program 128 bytes at that offset (must be a multiple of 16-byte phrase size)
//!     let data = [0xABu8; 128];
//!     flash.write(0xFE000, &data).unwrap();
//!
//!     // Read back
//!     let mut buf = [0u8; 128];
//!     flash.read(0xFE000, &mut buf).unwrap();
//! }
//!
//! ```

use core::slice;

use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

use crate::pac;
use crate::pac::syscon::vals::{ClrLpcac, DisDataSpec, DisFlashSpec, DisLpcac, DisMbeccErrData, DisMbeccErrInst};
use crate::rom;

// ---------------------------------------------------------------------------
// Flash geometry constants
// ---------------------------------------------------------------------------

/// Base address of the internal program flash.
pub const FLASH_BASE: u32 = 0x0000_0000;

/// Total size of the internal program flash in bytes.
#[cfg(feature = "mcxa2xx")]
pub const FLASH_SIZE: usize = 0x10_0000;

/// Total size of the internal program flash in bytes.
#[cfg(feature = "mcxa5xx")]
pub const FLASH_SIZE: usize = 0x20_0000;

/// Sector size in bytes (8 KB) — erase granularity.
pub const SECTOR_SIZE: usize = 0x2000;

/// Page size in bytes (128) — program-page granularity.
pub const PAGE_SIZE: usize = 128;

/// Phrase size in bytes (16) — minimum program unit.
pub const PHRASE_SIZE: usize = 16;

// ---------------------------------------------------------------------------
// ROM API constants
// ---------------------------------------------------------------------------

/// Flash erase key: `FOUR_CHAR_CODE('l','f','e','k')` in little-endian.
const FLASH_ERASE_KEY: u32 = 0x6B65_666C;

/// Returns a reference to the ROM API flash driver interface.
#[inline(always)]
fn flash_api() -> &'static rom::FlashDriverInterface {
    unsafe { &*rom::BootloaderTree::get().flash_driver }
}

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

/// Errors that can occur during flash operations.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Address or range is outside the flash region.
    OutOfBounds,
    /// Address or length is not properly aligned.
    Unaligned,
    /// The ROM API returned a non-zero status code.
    RomApi(i32),
}

/// Flash property identifiers (ROM API `flash_get_property`).
#[repr(u32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashProperty {
    /// Pflash sector size property.
    PflashSectorSize = 0x00,
    /// Pflash total size property.
    PflashTotalSize = 0x01,
    /// Pflash block size property.
    PflashBlockSize = 0x02,
    /// Pflash block count property.
    PflashBlockCount = 0x03,
    /// Pflash block base address property.
    PflashBlockBaseAddr = 0x04,
    /// Pflash page size property.
    PflashPageSize = 0x30,
    /// Pflash system frequency property.
    PflashSystemFreq = 0x31,
    /// FFR sector size property.
    FfrSectorSize = 0x40,
    /// FFR total size property.
    FfrTotalSize = 0x41,
    /// FFR block base address property.
    FfrBlockBaseAddr = 0x42,
    /// FFR page size property.
    FfrPageSize = 0x43,
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::Unaligned => NorFlashErrorKind::NotAligned,
            Self::RomApi(_) => NorFlashErrorKind::Other,
        }
    }
}

/// Convert a ROM API status code to a `Result`.
#[inline]
fn check_status(status: i32) -> Result<(), Error> {
    if status == 0 {
        Ok(())
    } else {
        Err(Error::RomApi(status))
    }
}

// ---------------------------------------------------------------------------
// Cache clearing helpers (must be called after erase/program)
// ---------------------------------------------------------------------------

/// Clear flash and data speculation buffers by toggling the disable bits
/// in SYSCON->NVM_CTRL, matching the C `speculation_buffer_clear()`.
#[inline]
fn speculation_buffer_clear() {
    let nvm = pac::SYSCON.nvm_ctrl();
    let val = nvm.read();

    // Only proceed if MBECC error reporting is enabled for both inst and data
    if val.dis_mbecc_err_inst() == DisMbeccErrInst::ENABLE && val.dis_mbecc_err_data() == DisMbeccErrData::ENABLE {
        // Toggle flash speculation disable
        if val.dis_flash_spec() == DisFlashSpec::ENABLE {
            nvm.modify(|w| w.set_dis_flash_spec(DisFlashSpec::DISABLE));
            nvm.modify(|w| w.set_dis_flash_spec(DisFlashSpec::ENABLE));
        }
        // Toggle data speculation disable
        if nvm.read().dis_data_spec() == DisDataSpec::ENABLE {
            nvm.modify(|w| w.set_dis_data_spec(DisDataSpec::DISABLE));
            nvm.modify(|w| w.set_dis_data_spec(DisDataSpec::ENABLE));
        }
    }
}

/// Clear LPCAC by setting the CLR_LPCAC bit in SYSCON->LPCAC_CTRL,
/// matching the C `lpcac_clear()`.
#[inline]
fn lpcac_clear() {
    let lpcac = pac::SYSCON.lpcac_ctrl();
    if lpcac.read().dis_lpcac() == DisLpcac::ENABLE {
        lpcac.modify(|w| w.set_clr_lpcac(ClrLpcac::DISABLE));
    }
}

/// Combined cache clearing: speculation buffers + LPCAC.
#[inline]
fn clear_caches() {
    speculation_buffer_clear();
    lpcac_clear();
}

// ---------------------------------------------------------------------------
// Flash driver
// ---------------------------------------------------------------------------

/// Flash driver providing safe access to the MCXA276 internal flash via ROM API.
///
/// The driver holds an internal `FlashConfig` that is initialised by the ROM
/// API's `flash_init` function during construction. All subsequent operations
/// pass this config to the ROM API.
pub struct Flash {
    config: rom::FlashConfig,
}

impl Flash {
    /// Create and initialise a new flash driver.
    ///
    /// This calls the ROM API `flash_init` to populate the internal flash
    /// configuration. Returns an error if the ROM API reports failure.
    pub fn new() -> Result<Self, Error> {
        let mut config = rom::FlashConfig::default();
        let status = unsafe { (flash_api().flash_init)(&mut config) };
        check_status(status)?;
        Ok(Self { config })
    }

    /// Erase flash sectors encompassing the given absolute address range.
    ///
    /// - `address`: absolute start address (must be sector-aligned).
    /// - `len`: number of bytes to erase (must be a multiple of sector size).
    ///
    /// Runs inside a critical section and clears caches afterwards.
    pub fn blocking_erase(&mut self, address: u32, len: u32) -> Result<(), Error> {
        let status = cortex_m::interrupt::free(|_| unsafe {
            (flash_api().flash_erase_sector)(&self.config, address, len, FLASH_ERASE_KEY)
        });
        clear_caches();
        check_status(status)
    }

    /// Program a phrase (16 bytes) of data at the given absolute address.
    ///
    /// - `address`: absolute start address (must be phrase-aligned).
    /// - `data`: source buffer whose length must be a multiple of `PHRASE_SIZE`.
    ///
    /// Runs inside a critical section and clears caches afterwards.
    pub fn blocking_program_phrase(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        let status = cortex_m::interrupt::free(|_| unsafe {
            (flash_api().flash_program_phrase)(&self.config, address, data.as_ptr(), data.len() as u32)
        });
        clear_caches();
        check_status(status)
    }

    /// Program a page of data at the given absolute address.
    ///
    /// - `address`: absolute start address (must be page-aligned).
    /// - `data`: source buffer whose length must be a multiple of `PAGE_SIZE`.
    ///
    /// Runs inside a critical section and clears caches afterwards.
    pub fn blocking_program(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        let status = cortex_m::interrupt::free(|_| unsafe {
            (flash_api().flash_program_page)(&self.config, address, data.as_ptr(), data.len() as u32)
        });
        clear_caches();
        check_status(status)
    }

    /// Verify that the programmed data at `address` matches `expected`.
    ///
    /// On mismatch, returns `Error::RomApi` with the ROM status code.
    pub fn verify_program(&mut self, address: u32, expected: &[u8]) -> Result<(), Error> {
        let mut failed_address: u32 = 0;
        let mut failed_data: u32 = 0;
        let status = unsafe {
            (flash_api().flash_verify_program)(
                &self.config,
                address,
                expected.len() as u32,
                expected.as_ptr(),
                &mut failed_address,
                &mut failed_data,
            )
        };
        check_status(status)
    }

    /// Verify that the sector(s) starting at `address` are erased.
    pub fn verify_erase_sector(&mut self, address: u32, len: u32) -> Result<(), Error> {
        let status = unsafe { (flash_api().flash_verify_erase_sector)(&self.config, address, len) };
        check_status(status)
    }

    /// Read flash data using the ROM API.
    ///
    /// - `address`: absolute start address.
    /// - `dest`: destination buffer.
    pub fn blocking_read_rom(&mut self, address: u32, dest: &mut [u8]) -> Result<(), Error> {
        let status = unsafe { (flash_api().flash_read)(&self.config, address, dest.as_mut_ptr(), dest.len() as u32) };
        check_status(status)
    }

    /// Read flash data by direct memory-mapped access (no ROM API call).
    ///
    /// - `offset`: byte offset from flash base (`0x0000_0000`).
    /// - `dest`: destination buffer.
    pub fn blocking_read(&self, offset: u32, dest: &mut [u8]) -> Result<(), Error> {
        if offset as usize + dest.len() > FLASH_SIZE {
            return Err(Error::OutOfBounds);
        }
        let src = unsafe { slice::from_raw_parts((FLASH_BASE + offset) as *const u8, dest.len()) };
        dest.copy_from_slice(src);
        Ok(())
    }

    /// Return the ROM API version.
    pub fn rom_api_version(&self) -> u32 {
        flash_api().version
    }

    /// Get a ROM API flash property value.
    pub fn get_property(&mut self, property: FlashProperty) -> Result<u32, Error> {
        let mut value: u32 = 0;
        let status = unsafe { (flash_api().flash_get_property)(&self.config, property as u32, &mut value) };
        check_status(status)?;
        Ok(value)
    }
}

// ---------------------------------------------------------------------------
// embedded-storage trait implementations
// ---------------------------------------------------------------------------

impl ErrorType for Flash {
    type Error = Error;
}

impl ReadNorFlash for Flash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl NorFlash for Flash {
    const WRITE_SIZE: usize = PHRASE_SIZE;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to < from || to as usize > FLASH_SIZE {
            return Err(Error::OutOfBounds);
        }
        if !(from as usize).is_multiple_of(Self::ERASE_SIZE) || !(to as usize).is_multiple_of(Self::ERASE_SIZE) {
            return Err(Error::Unaligned);
        }

        // Erase one sector at a time
        for sector_addr in (from..to).step_by(Self::ERASE_SIZE) {
            self.blocking_erase(FLASH_BASE + sector_addr, SECTOR_SIZE as u32)?;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if offset as usize + bytes.len() > FLASH_SIZE {
            return Err(Error::OutOfBounds);
        }
        if !(offset as usize).is_multiple_of(Self::WRITE_SIZE) || !bytes.len().is_multiple_of(Self::WRITE_SIZE) {
            return Err(Error::Unaligned);
        }

        // Program one phrase at a time (16 bytes — the smallest write unit)
        for (i, chunk) in bytes.chunks(PHRASE_SIZE).enumerate() {
            let addr = FLASH_BASE + offset + (i * PHRASE_SIZE) as u32;
            self.blocking_program_phrase(addr, chunk)?;
        }
        Ok(())
    }
}
