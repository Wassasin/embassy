#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::Range;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::usb::Driver;
use embassy_nrf::{
    bind_interrupts, pac, peripherals, qspi,
    usb::{self, vbus_detect::HardwareVbusDetect},
};
use embassy_usb::class::msc::subclass::scsi::block_device::{BlockDevice, BlockDeviceError};
use embassy_usb::class::msc::subclass::scsi::Scsi;
use embassy_usb::class::msc::transport::bulk_only::BulkOnlyTransport;
use embassy_usb::Builder;
use embedded_storage_async::nor_flash::NorFlash;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
    QSPI => qspi::InterruptHandler<peripherals::QSPI>;
});

const BLOCK_SIZE: usize = 4096;
const BLOCK_COUNT: usize = 2 * 1024;

struct FlashBlockDevice<FLASH: NorFlash> {
    flash: RefCell<FLASH>,
    range: Range<usize>,
}

impl<FLASH: NorFlash> BlockDevice for FlashBlockDevice<FLASH> {
    fn status(&self) -> Result<(), BlockDeviceError> {
        Ok(())
    }

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
            .await
            .map_err(|_| BlockDeviceError::ReadError)?;

        Ok(())
    }

    async fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError> {
        let mut flash = self.flash.borrow_mut();
        let start = self.range.start as u32 + (lba * BLOCK_SIZE as u32);
        flash
            .erase(start, start + BLOCK_SIZE as u32)
            .await
            .map_err(|_| BlockDeviceError::WriteError)?;

        flash
            .write(start, block)
            .await
            .map_err(|_| BlockDeviceError::WriteError)?;
        Ok(())
    }

    async fn flush(&self) -> Result<(), BlockDeviceError> {
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

    // Config for the MX25R64 present in the nRF52840 DK
    let mut qspi_config = qspi::Config::default();
    qspi_config.capacity = (BLOCK_SIZE * BLOCK_COUNT) as u32;
    qspi_config.frequency = qspi::Frequency::M32;
    qspi_config.read_opcode = qspi::ReadOpcode::READ4IO;
    qspi_config.write_opcode = qspi::WriteOpcode::PP4IO;
    qspi_config.write_page_size = qspi::WritePageSize::_256BYTES;

    let mut q = qspi::Qspi::new(
        p.QSPI,
        Irqs,
        p.P0_19,
        p.P0_17,
        p.P0_20,
        p.P0_21,
        p.P0_22,
        p.P0_23,
        qspi_config,
    );

    let mut id = [1; 3];
    unwrap!(q.custom_instruction(0x9F, &[], &mut id).await);
    info!("id: {}", id);

    // Read status register
    let mut status = [4; 1];
    unwrap!(q.custom_instruction(0x05, &[], &mut status).await);

    info!("status: {:?}", status[0]);

    if status[0] & 0x40 == 0 {
        status[0] |= 0x40;

        unwrap!(q.custom_instruction(0x01, &status, &mut []).await);

        info!("enabled quad in status");
    }

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

    let flash = RefCell::new(q);

    let range = 0..(BLOCK_COUNT * BLOCK_SIZE);

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
