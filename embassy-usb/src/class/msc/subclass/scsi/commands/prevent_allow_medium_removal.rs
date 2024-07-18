use super::{control::Control, Command};
use crate::packed::packed_struct;

packed_struct! {
    pub struct PreventAllowMediumRemoval<6> {
        #[offset = 0, size = 8]
        op_code: u8,
        #[offset = 4*8+0, size = 1]
        prevent: bool,
        #[offset = 5*8, size = 8]
        control: Control<[u8; Control::SIZE]>,
    }
}

impl Command for PreventAllowMediumRemoval<[u8; PreventAllowMediumRemoval::SIZE]> {
    const OPCODE: u8 = 0x1E;
}
