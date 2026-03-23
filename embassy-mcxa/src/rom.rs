/// Flash FFR (Factory Failure Records) configuration, populated by `flash_init`.
#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct FlashFfrConfig {
    pub ffr_block_base: u32,
    pub ffr_total_size: u32,
    pub ffr_page_size: u32,
    pub sector_size: u32,
    pub cfpa_page_version: u32,
    pub cfpa_page_offset: u32,
}

/// Flash driver configuration, populated by `flash_init`.
#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct FlashConfig {
    pub pflash_block_base: u32,
    pub pflash_total_size: u32,
    pub pflash_block_count: u32,
    pub pflash_page_size: u32,
    pub pflash_sector_size: u32,
    pub ffr_config: FlashFfrConfig,
}

// Type aliases for ROM API function pointer signatures (C ABI).
// Only `flash_init` takes `*mut FlashConfig`; all other calls use `*const FlashConfig`.
pub type FnFlashInit = unsafe extern "C" fn(config: *mut FlashConfig) -> i32;
pub type FnFlashEraseSector = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32, key: u32) -> i32;
pub type FnFlashProgramPhrase =
    unsafe extern "C" fn(config: *const FlashConfig, start: u32, src: *const u8, len: u32) -> i32;
pub type FnFlashProgramPage =
    unsafe extern "C" fn(config: *const FlashConfig, start: u32, src: *const u8, len: u32) -> i32;
pub type FnFlashVerifyProgram = unsafe extern "C" fn(
    config: *const FlashConfig,
    start: u32,
    len: u32,
    expected: *const u8,
    failed_addr: *mut u32,
    failed_data: *mut u32,
) -> i32;
pub type FnFlashVerifyErasePhrase = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32) -> i32;
pub type FnFlashVerifyErasePage = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32) -> i32;
pub type FnFlashVerifyEraseSector = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32) -> i32;
pub type FnFlashGetProperty = unsafe extern "C" fn(config: *const FlashConfig, property: u32, value: *mut u32) -> i32;
pub type FnFlashRead = unsafe extern "C" fn(config: *const FlashConfig, start: u32, dest: *mut u8, len: u32) -> i32;

pub type FnIfrVerifyErasePhrase = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32) -> i32;
pub type FnIfrVerifyErasePage = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32) -> i32;
pub type FnIfrVerifyEraseSector = unsafe extern "C" fn(config: *const FlashConfig, start: u32, len: u32) -> i32;

/// Optional ifr flash driver methods for the same ROM API version, depending on whether
/// it has been compiled with FSL_FEATURE_ROMAPI_IFR.
#[repr(C)]
pub struct IfrDriverInterface {
    pub ifr_verify_erase_phrase: FnIfrVerifyErasePhrase,
    pub ifr_verify_erase_page: FnIfrVerifyErasePage,
    pub ifr_verify_erase_sector: FnIfrVerifyEraseSector,
}

/// ROM API flash driver interface vtable.
#[repr(C)]
pub struct FlashDriverInterface {
    pub flash_init: FnFlashInit,
    pub flash_erase_sector: FnFlashEraseSector,
    pub flash_program_phrase: FnFlashProgramPhrase,
    pub flash_program_page: FnFlashProgramPage,
    pub flash_verify_program: FnFlashVerifyProgram,
    pub flash_verify_erase_phrase: FnFlashVerifyErasePhrase,
    pub flash_verify_erase_page: FnFlashVerifyErasePage,
    pub flash_verify_erase_sector: FnFlashVerifyEraseSector,
    pub flash_get_property: FnFlashGetProperty,
    #[cfg(feature = "mcxa5xx")]
    pub flash_ifr: IfrDriverInterface,
    pub flash_read: FnFlashRead,
    pub version: u32,
}

/// Root of the ROM bootloader API tree.
#[repr(C)]
pub struct BootloaderTree {
    pub run_bootloader: unsafe extern "C" fn(arg: *mut core::ffi::c_void),
    pub flash_driver: *const FlashDriverInterface,
    pub jump: unsafe extern "C" fn(arg: *mut core::ffi::c_void),
}

impl BootloaderTree {
    #[cfg(feature = "mcxa2xx")]
    const INSTANCE: *const BootloaderTree = 0x0300_5FE0 as *const BootloaderTree;

    #[cfg(feature = "mcxa5xx")]
    const INSTANCE: *const BootloaderTree = 0x1303_D800 as *const BootloaderTree;

    pub const fn get() -> &'static BootloaderTree {
        unsafe { &*Self::INSTANCE }
    }
}
