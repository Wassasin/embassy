#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, pac, peripherals, usb::{self, vbus_detect::HardwareVbusDetect}};
use embassy_usb::class::msc::subclass::scsi::block_device::{BlockDevice, BlockDeviceError};
use embassy_usb::class::msc::subclass::scsi::Scsi;
use embassy_usb::class::msc::transport::bulk_only::BulkOnlyTransport;
use embassy_usb::Builder;
use embassy_futures::join::join;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
});

// 512 is a standard block size supported by most systems
const BLOCK_SIZE: usize = 512;
const BLOCK_COUNT: usize = 128;

struct RamBlockDevice {
    data: [u8; BLOCK_SIZE * BLOCK_COUNT],
}

impl BlockDevice for &mut RamBlockDevice {
    fn status(&self) -> Result<(), BlockDeviceError> {
        Ok(())
    }

    fn block_size(&self) -> Result<usize, BlockDeviceError> {
        Ok(BLOCK_SIZE)
    }

    fn num_blocks(&self) -> Result<u32, BlockDeviceError> {
        Ok((self.data.len() / self.block_size().unwrap()) as u32)
    }

    async fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError> {
        block.copy_from_slice(&self.data[lba as usize * BLOCK_SIZE..(lba as usize + 1) * BLOCK_SIZE]);
        Ok(())
    }

    async fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError> {
        self.data[lba as usize * BLOCK_SIZE..(lba as usize + 1) * BLOCK_SIZE].copy_from_slice(block);
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

    static RAM_BLOCK_DEVICE: StaticCell<RamBlockDevice> = StaticCell::new();
    let ram_block_device = RAM_BLOCK_DEVICE.init(RamBlockDevice {
        data: [0u8; BLOCK_SIZE * BLOCK_COUNT],
    });

    // Create SCSI target for our block device
    let mut scsi_buffer = [0u8; BLOCK_SIZE];
    let scsi = Scsi::new(
        ram_block_device,
        &mut scsi_buffer,
        "Embassy",
        "MSC",
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
