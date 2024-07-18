use super::control::Control;
use crate::packed_struct;

packed_struct! {
    pub struct StartStopUnitCommand<6> {
        #[offset = 0, size = 8]
        op_code: u8,
        #[offset = 1*8+0, size = 1]
        immed: bool,
        #[offset = 3*8+0, size = 4]
        power_condition_modifier: u8,
        #[offset = 4*8+0, size = 1]
        start: bool,
        #[offset = 4*8+1, size = 1]
        loej: bool,
        #[offset = 4*8+2, size = 1]
        no_flush: bool,
        #[offset = 4*8+4, size = 4]
        power_condition: u8,
        #[offset = 5*8, size = 8]
        control: Control<[u8; Control::SIZE]>,
    }
}

impl StartStopUnitCommand<[u8; StartStopUnitCommand::SIZE]> {
    pub const OPCODE: u8 = 0x1B;
}
