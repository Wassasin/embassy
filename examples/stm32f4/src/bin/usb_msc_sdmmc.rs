#![no_std]
#![no_main]

use core::cell::RefCell;

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::peripherals::{self, DMA2_CH6, SDIO};
use embassy_stm32::sdmmc::{self, Sdmmc};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, Config};
use embassy_usb::class::msc::subclass::scsi::block_device::{BlockDevice, BlockDeviceError};
use embassy_usb::class::msc::subclass::scsi::Scsi;
use embassy_usb::class::msc::transport::bulk_only::BulkOnlyTransport;
use embassy_usb::Builder;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// SDMMC driver only supports 512 byte blocks for now
const BLOCK_SIZE: usize = 512;

type MySdmmc<'d> = Sdmmc<'d, SDIO, DMA2_CH6>;
const SDIO_FREQ: Hertz = Hertz(12_000_000);

struct SdmmcBlockDevice<'d> {
    sdmmc: RefCell<MySdmmc<'d>>,
}

impl<'d> BlockDevice for SdmmcBlockDevice<'d> {
    fn status(&self) -> Result<(), BlockDeviceError> {
        Ok(())
    }

    fn block_size(&self) -> Result<usize, BlockDeviceError> {
        Ok(BLOCK_SIZE)
    }

    fn num_blocks(&self) -> Result<u32, BlockDeviceError> {
        Ok((self.sdmmc.borrow().card().unwrap().csd.card_size() / BLOCK_SIZE as u64) as u32)
    }

    async fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError> {
        if block.as_ptr() as usize % 4 != 0 {
            panic!("Invalid block alignment for SDMMC");
        }

        let block: &mut [u8; BLOCK_SIZE] = block.try_into().unwrap();
        let block = unsafe { core::mem::transmute(block) };

        self.sdmmc.borrow_mut().read_block(lba, block).await.map_err(|e| {
            error!("SDMMC read error: {:?}", e);
            BlockDeviceError::ReadError
        })
    }

    async fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError> {
        if block.as_ptr() as usize % 4 != 0 {
            panic!("Invalid block alignment for SDMMC");
        }

        let block: &[u8; BLOCK_SIZE] = block.try_into().unwrap();
        let block = unsafe { core::mem::transmute(block) };

        self.sdmmc.borrow_mut().write_block(lba, block).await.map_err(|e| {
            error!("SDMMC write error: {:?}", e);
            BlockDeviceError::WriteError
        })
    }
}

#[repr(align(4))]
struct AlignedBuffer([u8; BLOCK_SIZE]);

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    SDIO => sdmmc::InterruptHandler<peripherals::SDIO>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    // Create the driver, from the HAL.
    static OUTPUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    let ep_out_buffer = &mut OUTPUT_BUFFER.init([0; 256])[..];
    let mut config = embassy_stm32::usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = false;

    let driver = Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("MSC example");
    config.serial_number = Some("12345678");

    // Required for windows compatiblity.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = Default::default();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut control_buf,
    );

    let mut sdmmc = Sdmmc::new_4bit(
        p.SDIO,
        Irqs,
        p.DMA2_CH6,
        p.PC12,
        p.PD2,
        p.PC8,
        p.PC9,
        p.PC10,
        p.PC11,
        Default::default(),
    );

    sdmmc.init_card(SDIO_FREQ).await.expect("SD card init failed");
    info!("Initialized SD card: {:#?}", Debug2Format(sdmmc.card().unwrap()));

    // Create SCSI target for our block device
    let mut scsi_buffer = AlignedBuffer([0u8; BLOCK_SIZE]);
    let scsi = Scsi::new(
        SdmmcBlockDevice {
            sdmmc: RefCell::new(sdmmc),
        },
        &mut scsi_buffer.0,
        "Embassy",
        "MSC",
        "1234",
    );

    // Use bulk-only transport for our SCSI target
    let mut msc_transport = BulkOnlyTransport::new(&mut builder, &mut state, 64, scsi);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Run mass storage transport
    let msc_fut = msc_transport.run();

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, msc_fut).await;
}
