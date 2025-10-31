use crate::cpu::AddressingMode;
use std::collections::{HashMap, hash_map};

#[derive(Debug, Clone, Copy)]
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
        OpCode::new(0x69, "ADC", 2, 2, AddressingMode::Immediate),
        OpCode::new(0x65, "ADC", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x75, "ADC", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0x6D, "ADC", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x7D, "ADC", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),
        OpCode::new(0x79, "ADC", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),
        OpCode::new(0x61, "ADC", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0x71, "ADC", 2, 5 /* 6 cycles if page crossed */, AddressingMode::IndirectY),

        OpCode::new(0x29, "AND", 2, 2, AddressingMode::Immediate),
        OpCode::new(0x25, "AND", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x35, "AND", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0x2D, "AND", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x3D, "AND", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),
        OpCode::new(0x39, "AND", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),
        OpCode::new(0x21, "AND", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0x31, "AND", 2, 5 /* 6 cycles if page crossed */, AddressingMode::IndirectY),

        OpCode::new(0x0A, "ASL", 1, 2, AddressingMode::Accumulator),
        OpCode::new(0x06, "ASL", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0x16, "ASL", 2, 6, AddressingMode::ZeroPageX),
        OpCode::new(0x0E, "ASL", 3, 6, AddressingMode::Absolute),
        OpCode::new(0x1E, "ASL", 3, 7, AddressingMode::AbsoluteX),

        OpCode::new(0x90, "BCC", 2, 2 /* 3 cycles if branch taken, 4 if page crossed */, AddressingMode::Relative),

        OpCode::new(0xB0, "BCS", 2, 2 /* 3 cycles if branch taken, 4 if page crossed */, AddressingMode::Relative),

        OpCode::new(0xF0, "BEQ", 2, 2 /* 3 cycles if branch taken, 4 if page crossed */, AddressingMode::Relative),

        OpCode::new(0x24, "BIT", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x2C, "BIT", 3, 4, AddressingMode::Absolute),

        OpCode::new(0xD0, "BNE", 2, 3 /* 3 cycles if branch taken, 4 if page crossed */, AddressingMode::Relative),

        OpCode::new(0x10, "BPL", 2, 2 /* 3 cycles if branch taken, 4 if page crossed */, AddressingMode::Relative),

        OpCode::new(0x00, "BRK", 1, 7, AddressingMode::None),

        OpCode::new(0x18, "CLC", 1, 2, AddressingMode::None),

        OpCode::new(0xC9, "CMP", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xC5, "CMP", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xD5, "CMP", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0xCD, "CMP", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xDD, "CMP", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),
        OpCode::new(0xD9, "CMP", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),
        OpCode::new(0xC1, "CMP", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0xD1, "CMP", 2, 5 /* 6 cycles if page crossed */, AddressingMode::IndirectY),

        OpCode::new(0xE0, "CPX", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xE4, "CPX", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xEC, "CPX", 3, 4, AddressingMode::Absolute),

        OpCode::new(0xC0, "CPY", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xC4, "CPY", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xCC, "CPY", 3, 4, AddressingMode::Absolute),

        OpCode::new(0xC6, "DEC", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0xD6, "DEC", 2, 6, AddressingMode::ZeroPageX),
        OpCode::new(0xCE, "DEC", 3, 6, AddressingMode::Absolute),
        OpCode::new(0xDE, "DEC", 3, 7, AddressingMode::AbsoluteX),

        OpCode::new(0xCA, "DEX", 1, 2, AddressingMode::None),

        OpCode::new(0x88, "DEY", 1, 2, AddressingMode::None),

        OpCode::new(0x38, "SEC", 1, 2, AddressingMode::None),

        OpCode::new(0xAA, "TAX", 1, 2, AddressingMode::None),

        OpCode::new(0x8A, "TXA", 1, 2, AddressingMode::None),

        OpCode::new(0xE6, "INC", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0xF6, "INC", 2, 6, AddressingMode::ZeroPageX),
        OpCode::new(0xEE, "INC", 3, 6, AddressingMode::Absolute),
        OpCode::new(0xFE, "INC", 3, 7, AddressingMode::AbsoluteX),

        OpCode::new(0xE8, "INX", 1, 2, AddressingMode::None),

        OpCode::new(0x4C, "JMP", 3, 3, AddressingMode::Absolute),
        OpCode::new(0x6C, "JMP", 3, 5, AddressingMode::Indirect),

        OpCode::new(0x20, "JSR", 3, 6, AddressingMode::Absolute),

        OpCode::new(0xA9, "LDA", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA5, "LDA", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB5, "LDA", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0xAD, "LDA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBD, "LDA", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),
        OpCode::new(0xB9, "LDA", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),
        OpCode::new(0xA1, "LDA", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0xB1, "LDA", 2, 5 /* 6 cycles if page crossed */, AddressingMode::IndirectY),

        OpCode::new(0xA2, "LDX", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA6, "LDX", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB6, "LDX", 2, 4, AddressingMode::ZeroPageY),
        OpCode::new(0xAE, "LDX", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBE, "LDX", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),

        OpCode::new(0xA0, "LDY", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA4, "LDY", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB4, "LDY", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0xAC, "LDY", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBC, "LDY", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),

        OpCode::new(0x4A, "LSR", 1, 2, AddressingMode::Accumulator),
        OpCode::new(0x46, "LSR", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0x56, "LSR", 2, 6, AddressingMode::ZeroPageX),
        OpCode::new(0x4E, "LSR", 3, 6, AddressingMode::Absolute),
        OpCode::new(0x5E, "LSR", 3, 7, AddressingMode::AbsoluteX),

        OpCode::new(0xEA, "NOP", 1, 2, AddressingMode::None),

        OpCode::new(0x60, "RTS", 1, 6, AddressingMode::None),

        OpCode::new(0xE9, "SBC", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xE5, "SBC", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xF5, "SBC", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0xED, "SBC", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xFD, "SBC", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteX),
        OpCode::new(0xF9, "SBC", 3, 4 /* 5 cycles if page crossed */, AddressingMode::AbsoluteY),
        OpCode::new(0xE1, "SBC", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0xF1, "SBC", 2, 5 /* 6 cycles if page crossed */, AddressingMode::IndirectY),

        OpCode::new(0x85, "STA", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x95, "STA", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0x8D, "STA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x9D, "STA", 3, 5, AddressingMode::AbsoluteX),
        OpCode::new(0x99, "STA", 3, 5, AddressingMode::AbsoluteY),
        OpCode::new(0x81, "STA", 2, 6, AddressingMode::IndirectX),
        OpCode::new(0x91, "STA", 2, 6, AddressingMode::IndirectY),

        OpCode::new(0x86, "STX", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x96, "STX", 2, 4, AddressingMode::ZeroPageY),
        OpCode::new(0x8E, "STX", 3, 4, AddressingMode::Absolute),

        OpCode::new(0x84, "STY", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x94, "STY", 2, 4, AddressingMode::ZeroPageX),
        OpCode::new(0x8C, "STY", 3, 4, AddressingMode::Absolute),
    ];

    pub static ref OPCODES_MAP: HashMap<u8, &'static OpCode> = {
        let mut map = HashMap::new();
        for cpu_op in &*CPU_OPCODES {
            map.insert(cpu_op.opcode, cpu_op);
        }
        map
    };
}
