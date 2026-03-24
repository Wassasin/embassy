#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::{
    config::Config,
    flash::FlashProperty,
    pac::{self, mbc},
};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[derive(Debug)]
#[repr(C)]
struct Cfpa {
    _buf: [u8; 0x200],
}

#[derive(Debug)]
#[repr(C)]
struct Cmpa {
    _buf: [u8; 0x1600],
}

#[derive(Debug)]
#[repr(C)]
struct Nfpa {
    _buf: [u8; 0x800],
}

#[derive(Debug)]
#[repr(C)]
struct Devcfg {
    cfpa: Cfpa,
    // cmpa: Cmpa,
    // nfpa: Nfpa,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = hal::init(Config::default());

    defmt::info!("Hello, world!");
    cortex_m::asm::delay(10_000_000);

    let mut flash = defmt::unwrap!(hal::flash::Flash::new());

    defmt::info!("Flash version: 0x{:x}", flash.rom_api_version());

    let pflash_block_base = defmt::unwrap!(flash.get_property(FlashProperty::PflashBlockBaseAddr));
    let pflash_sector_size = defmt::unwrap!(flash.get_property(FlashProperty::PflashSectorSize));
    let pflash_total_size = defmt::unwrap!(flash.get_property(FlashProperty::PflashTotalSize));
    let pflash_page_size = defmt::unwrap!(flash.get_property(FlashProperty::PflashPageSize));

    defmt::info!("PFlash Information:");
    defmt::info!("kFLASH_PropertyPflashBlockBaseAddr = 0x{:X}", pflash_block_base);
    defmt::info!("kFLASH_PropertyPflashSectorSize = {}", pflash_sector_size);
    defmt::info!("kFLASH_PropertyPflashTotalSize = {}", pflash_total_size);
    defmt::info!("kFLASH_PropertyPflashPageSize = 0x{:X}", pflash_page_size);

    defmt::info!("mbc0_mem0_glbcfg {:?}", pac::MBC0.mbc0_mem0_glbcfg().read());
    defmt::info!("mbc0_mem1_glbcfg {:?}", pac::MBC0.mbc0_mem1_glbcfg().read());
    defmt::info!("mbc0_mem2_glbcfg {:?}", pac::MBC0.mbc0_mem2_glbcfg().read());
    defmt::info!("mbc0_mem3_glbcfg {:?}", pac::MBC0.mbc0_mem3_glbcfg().read());

    defmt::info!("mbc0_memn_glbac0 {:?}", pac::MBC0.mbc0_memn_glbac0().read());
    defmt::info!("mbc0_memn_glbac1 {:?}", pac::MBC0.mbc0_memn_glbac1().read());
    defmt::info!("mbc0_memn_glbac2 {:?}", pac::MBC0.mbc0_memn_glbac2().read());
    defmt::info!("mbc0_memn_glbac3 {:?}", pac::MBC0.mbc0_memn_glbac3().read());
    defmt::info!("mbc0_memn_glbac4 {:?}", pac::MBC0.mbc0_memn_glbac4().read());
    defmt::info!("mbc0_memn_glbac5 {:?}", pac::MBC0.mbc0_memn_glbac5().read());
    defmt::info!("mbc0_memn_glbac6 {:?}", pac::MBC0.mbc0_memn_glbac6().read());
    defmt::info!("mbc0_memn_glbac7 {:?}", pac::MBC0.mbc0_memn_glbac7().read());

    for i in 0..=15 {
        defmt::info!(
            "mbc0_dom0_mem0_blk_cfg_w{}: {:?}",
            i,
            pac::MBC0.mbc0_dom0_mem0_blk_cfg_wn(i).read()
        );
    }

    defmt::info!(
        "mbc0_dom0_mem1_blk_cfg_w0 {:?}",
        pac::MBC0.mbc0_dom0_mem1_blk_cfg_w0().read()
    );
    defmt::info!(
        "mbc0_dom0_mem1_blk_cfg_w1 {:?}",
        pac::MBC0.mbc0_dom0_mem1_blk_cfg_w1().read()
    );
    defmt::info!(
        "mbc0_dom0_mem1_blk_cfg_w2 {:?}",
        pac::MBC0.mbc0_dom0_mem1_blk_cfg_w2().read()
    );
    defmt::info!(
        "mbc0_dom0_mem1_blk_cfg_w3 {:?}",
        pac::MBC0.mbc0_dom0_mem1_blk_cfg_w3().read()
    );

    defmt::info!(
        "mbc0_dom0_mem2_blk_cfg_w0 {:?}",
        pac::MBC0.mbc0_dom0_mem2_blk_cfg_w0().read()
    );

    // First word in SCRATCH determines the command.
    // Followed by a warm reset.

    // In IFR0
    // let ptr = 0x01000000 as *const Devcfg;
    // let ptr = 0x01002000 as *const Devcfg;

    let mut buf = [0u8; 0x200];
    cortex_m::asm::delay(10_000_000);

    let addr = 0x11000000;

    defmt::unwrap!(flash.blocking_read(addr, &mut buf));
    defmt::info!("Devcfg: {:?}", buf);

    // let devcfg = unsafe { &*ptr };
    // let devcfg: &Devcfg = unsafe { core::mem::transmute(&buf) };

    // defmt::info!("Devcfg: {:?}", defmt::Debug2Format(devcfg));
    // defmt::info!("Devcfg: {:?}", devcfg);
    cortex_m::asm::delay(10_000_000);

    cortex_m::asm::bkpt();
}
