// Collection of all OPCodes for the NES

use crate::cpu::AddressingMode;
use std::collections::HashMap;

pub struct OPCode {
    pub code: u8,
    pub name: &'static str,
    pub len: u8,
    pub cycles: u8,
    pub mode: AddressingMode,
}

impl OPCode {
    fn new(code: u8, name: &'static str, len: u8, cycles: u8, mode: AddressingMode) -> Self {
        OPCode {
            code: code,
            name: name,
            len: len,
            cycles: cycles,
            mode: mode,
        }
    }
}

lazy_static! {
    pub static ref CPU_OPCodeS: Vec<OPCode> = vec! [
        OPCode::new(0x00, "brk", 1, 7, AddressingMode::NoneAddressing),
        OPCode::new(0xaa, "TAX", 1, 2, AddressingMode::NoneAddressing),
        OPCode::new(0xe8, "INX", 1, 2, AddressingMode::NoneAddressing),

        OPCode::new(0xa9, "LDA", 2, 2, AddressingMode::Immediate),
        OPCode::new(0xa5, "LDA", 2, 3, AddressingMode::ZeroPage),
        OPCode::new(0xb5, "LDA", 2, 4, AddressingMode::ZeroPageX),
        OPCode::new(0xad, "LDA", 3, 4, AddressingMode::Absolute),
        OPCode::new(0xbd, "LDA", 3, 4, AddressingMode::AbsoluteX),
        OPCode::new(0xb9, "LDA", 3, 4, AddressingMode::AbsoluteY),
        OPCode::new(0xa1, "LDA", 2, 6, AddressingMode::IndirectX),
        OPCode::new(0xb1, "LDA", 2, 5, AddressingMode::IndirectY),

        OPCode::new(0x85, "STA", 2, 3, AddressingMode::ZeroPage),
        OPCode::new(0x95, "STA", 2, 4, AddressingMode::ZeroPageX),
        OPCode::new(0x8d, "STA", 3, 4, AddressingMode::Absolute),
        OPCode::new(0x9d, "STA", 3, 5, AddressingMode::AbsoluteX),
        OPCode::new(0x99, "STA", 3, 5, AddressingMode::AbsoluteY),
        OPCode::new(0x81, "STA", 2, 6, AddressingMode::IndirectX),
        OPCode::new(0x91, "STA", 2, 6, AddressingMode::IndirectY),
    ];

    pub static ref OPCodes_MAP: HashMap<u8, &'static OPCode> = {
        let mut map = HashMap::new();
        for cpuop in &*CPU_OPCodeS {
            map.insert(cpuop.code, cpuop);
        }
        return map;
    };
}