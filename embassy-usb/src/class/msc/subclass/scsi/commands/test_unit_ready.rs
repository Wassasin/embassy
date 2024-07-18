use super::{control::Control, Command};
use crate::packed::packed_struct;

packed_struct! {
    pub struct TestUnitReadyCommand<6> {
        #[offset = 0, size = 8]
        op_code: u8,
        #[offset = 5*8, size = 8]
        control: Control<[u8; Control::SIZE]>,
    }
}

impl Command for TestUnitReadyCommand<[u8; TestUnitReadyCommand::SIZE]> {
    const OPCODE: u8 = 0x00;
}
