use super::control::Control;
use crate::packed::BE;
use crate::packed_struct;

packed_struct! {
    pub struct Write10Command<10> {
        #[offset = 0, size = 8]
        op_code: u8,
        /// A Force unit Access (FUA) bit set to zero specifies that the device server may read the logical blocks from the volatile cache (if any), the
        /// specified data pattern for that LBA (e.g., the data pattern for unmapped data), the non-volatile cache, or the medium.
        ///
        /// An FUA bit set to one specifies that the device server shall read the logical blocks from the specified data pattern for that LBA, the
        /// non-volatile cache (if any), or the medium. If a volatile cache contains a more recent version of a logical block, then the device server
        /// shall write that logical block to non-volatile cache or the medium before reading the logical block.
        #[offset = 1*8+3, size = 1]
        force_unit_access: bool,
        /// A Disable Page Out (DPO) bit set to zero specifies that the retention priority shall be determined by the RETENTION PRIORITY fields in the
        /// Caching mode page (see 5.3.9).
        ///
        /// A DPO bit set to one specifies that the device server shall assign the logical blocks accessed by this command the lowest retention priority
        /// for being fetched into or retained by the cache. A DPO bit set to one overrides any retention priority specified in the Caching mode
        /// page. All other aspects of the algorithm implementing the cache replacement strategy are not defined by this manual.
        #[offset = 1*8+4, size = 1]
        disable_page_out: bool,
        #[offset = 1*8+5, size = 3]
        write_protect: u8,
        #[offset = 2*8+0, size = 32]
        lba: BE<u32>,
        #[offset = 6*8+0, size = 5]
        group_number: u8,
        #[offset = 7*8+0, size = 16]
        transfer_length: BE<u16>,
        #[offset = 9*8+0, size = 8]
        control: Control<[u8; Control::SIZE]>,
    }
}

impl Write10Command<[u8; Write10Command::SIZE]> {
    pub const OPCODE: u8 = 0x2A;
}