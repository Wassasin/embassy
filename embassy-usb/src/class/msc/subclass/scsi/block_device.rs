//! Block Storage device to be implemented by the embedded device.

/// Errors and 'normal' status codes that can be emitted by a [BlockDevice].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BlockDeviceError {
    /// Block device is not present and cannot be accessed.
    ///
    /// SCSI NOT READY 3Ah/00h MEDIUM NOT PRESENT
    MediumNotPresent,
    /// Logical Block Address is out of range
    ///
    /// SCSI ILLEGAL REQUEST 21h/00h LOGICAL BLOCK ADDRESS OUT OF RANGE
    LbaOutOfRange,
    /// Unrecoverable hardware error
    ///
    /// SCSI HARDWARE ERROR 00h/00h NO ADDITIONAL SENSE INFORMATION
    HardwareError,
    /// SCSI MEDIUM ERROR 11h/00h UNRECOVERED READ ERROR
    ReadError,
    /// SCSI MEDIUM ERROR 0Ch/00h WRITE ERROR
    WriteError,
    /// SCSI MEDIUM ERROR 51h/00h ERASE FAILURE
    EraseError,
}

/// Block Storage device to be implemented by the embedded device.
///
/// Examples include:
/// * In-memory RAM block.
/// * On-die Flash memory.
/// * External serial NOR or NAND flash.
/// * External SD (or eMMC) card.
pub trait BlockDevice {
    /// The number of bytes per block. This determines the size of the buffer passed
    /// to read/write functions.
    fn block_size(&self) -> Result<usize, BlockDeviceError>;

    /// Number of blocks in device.
    fn num_blocks(&self) -> Result<u32, BlockDeviceError>;

    /// Read the block indicated by `lba` into the provided buffer.
    ///
    /// The `block` is guaranteed to be of `self.num_blocks()` length.
    async fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError>;

    /// Write the `block` buffer to the block indicated by `lba`.
    ///
    /// The `block` is guaranteed to be of `self.num_blocks()` length.
    async fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError>;

    /// Called for periodic `TEST UNIT READY` SCSI requests.
    ///
    /// Should return error if device is not ready (i.e. [BlockDeviceError::MediumNotPresent] if SD card is not present).
    fn status(&self) -> Result<(), BlockDeviceError> {
        Ok(())
    }

    /// If any operations are pending, finalize them and only return when that has occurred.
    ///
    /// Examples where this might be relevant:
    /// * when the BlockDevice has some internal buffer that is asynchronously flushed.
    /// * when the backing storage is a SD card that can asynchronously perform writes.
    async fn flush(&mut self) -> Result<(), BlockDeviceError> {
        Ok(())
    }
}
