use crate::cpu::AddressingMode;
use std::collections::{HashMap, hash_map};

#[derive(Clone, Copy)]
pub struct OpCode {
    pub opcode: u8,             // 0xa9
    pub mnemonic: &'static str, // LDA
    pub bytes: u8,              // 2
    pub cycles: u8,             // 2
    pub mode: AddressingMode,   // AddressingMode::Immediate
}

impl OpCode {
    fn new(
        opcode: u8,
        mnemonic: &'static str,
        bytes: u8,
        cycles: u8,
        mode: AddressingMode,
    ) -> Self {
        OpCode {
            opcode,
            mnemonic,
            bytes,
            cycles,
            mode,
        }
    }
}

lazy_static! {
    pub static ref CPU_OPCODES: Vec<OpCode> = vec![
        OpCode::new(0x00, "BRK", 1, 7, AddressingMode::None),
        OpCode::new(0xAA, "TAX", 1, 2, AddressingMode::None),
        OpCode::new(0xE8, "INX", 1, 2, AddressingMode::None),

        OpCode::new(0xA9, "LDA", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA5, "LDA", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB5, "LDA", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0xAD, "LDA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBD, "LDA", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),
        OpCode::new(0xB9, "LDA", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),
        OpCode::new(0xA1, "LDA", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0xB1, "LDA", 2, 5 /* 6 cycles if page crossed */, AddressingMode::IndirectY),

        OpCode::new(0x85, "STA", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x95, "STA", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0x8D, "STA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x9D, "STA", 3, 5, AddressingMode::AbsoluteX),
        OpCode::new(0x99, "STA", 3, 5, AddressingMode::AbsoluteY),
        OpCode::new(0x81, "STA", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0x91, "STA", 2, 6, AddressingMode::IndirectY),
    ];

    pub static ref OPCODES_MAP: HashMap<u8, &'static OpCode> = {
        let mut map = HashMap::new();
        for cpu_op in &*CPU_OPCODES {
            map.insert(cpu_op.opcode, cpu_op);
        }
        map
    };
}
