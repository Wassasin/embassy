#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::Range;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::usb::Driver;
use embassy_nrf::{
    bind_interrupts,
    nvmc::Nvmc,
    pac, peripherals,
    usb::{self, vbus_detect::HardwareVbusDetect},
};
use embassy_usb::class::msc::subclass::scsi::block_device::{BlockDevice, BlockDeviceError};
use embassy_usb::class::msc::subclass::scsi::Scsi;
use embassy_usb::class::msc::transport::bulk_only::BulkOnlyTransport;
use embassy_usb::Builder;
use embedded_storage::nor_flash::NorFlash;
use {defmt_rtt as _, panic_probe as _};

// WARNING: this example is way too slow to

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
});

// We use a 4k block size, which is the flash sector size of NRF52840.
const BLOCK_SIZE: usize = 4096;

struct FlashBlockDevice<FLASH: NorFlash> {
    flash: RefCell<FLASH>,
    range: Range<usize>,
}

impl<FLASH: NorFlash> BlockDevice for FlashBlockDevice<FLASH> {
    fn block_size(&self) -> Result<usize, BlockDeviceError> {
        Ok(BLOCK_SIZE)
    }

    fn num_blocks(&self) -> Result<u32, BlockDeviceError> {
        Ok((self.range.len() / BLOCK_SIZE) as u32)
    }

    async fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError> {
        self.flash
            .borrow_mut()
            .read(self.range.start as u32 + (lba * BLOCK_SIZE as u32), block)
            .map_err(|_| BlockDeviceError::ReadError)?;
        Ok(())
    }

    async fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError> {
        let mut flash = self.flash.borrow_mut();
        let start = self.range.start as u32 + (lba * BLOCK_SIZE as u32);
        flash
            .erase(start, start + block.len() as u32)
            .map_err(|_| BlockDeviceError::WriteError)?;
        flash.write(start, block).map_err(|_| BlockDeviceError::WriteError)?;
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let clock: pac::CLOCK = unsafe { core::mem::transmute(()) };

    info!("Enabling ext hfosc...");
    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    while clock.events_hfclkstarted.read().bits() != 1 {}

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

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

    let flash = RefCell::new(Nvmc::new(p.NVMC));

    // Use upper 512KB of the 1KB flash
    let range = (512 * 1024)..(1024 * 1024);

    let mut scsi_buffer = [0u8; BLOCK_SIZE];
    // Create SCSI target for our block device
    let scsi = Scsi::new(
        FlashBlockDevice { flash, range },
        &mut scsi_buffer,
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
