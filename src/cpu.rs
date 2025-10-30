use crate::opcodes;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect, // JMP only
    IndirectX,
    IndirectY,
    Accumulator,
    None,
}

const CARRY_FLAG: u8 = 0b00000001;
const ZERO_FLAG: u8 = 0b00000010;
const IRQ_DISABLE_FLAG: u8 = 0b00000100;
const DECIMAL_FLAG: u8 = 0b00001000;
const INDEX_REGISTER_SELECT_FLAG: u8 = 0b00010000;
const ACCUMULATOR_SELECT_FLAG: u8 = 0b00100000;
const OVERFLOW_FLAG: u8 = 0b01000000;
const NEGATIVE_FLAG: u8 = 0b10000000;

#[derive(Debug)]
pub struct CPU {
    pub reg_a: u8,
    pub reg_x: u8,
    pub reg_y: u8,
    pub status: u8, // FLAGS
    pub program_counter: u16,
    pub stack_pointer: u8,

    pub memory: [u8; 0xFFFF],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            reg_a: 0,
            reg_x: 0,
            reg_y: 0,
            status: 0,
            program_counter: 0xFFC,
            stack_pointer: 0xFD,
            memory: [0x00; 0xFFFF],
        }
    }

    pub fn reset(&mut self) {
        self.reg_a = 0x00;
        self.reg_x = 0x00;
        self.reg_y = 0x00;
        self.status = 0x00;

        self.program_counter = self.read_mem_u16(0xFFFC);
        self.stack_pointer = 0xFF;
    }

    #[inline]
    fn set_flag(&mut self, flag: u8) {
        self.status |= flag;
    }

    #[inline]
    fn clear_flag(&mut self, flag: u8) {
        self.status &= !flag;
    }

    #[inline]
    fn get_flag(&self, flag: u8) -> bool {
        (self.status & flag) != 0
    }

    #[inline]
    fn read_mem(&mut self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    #[inline]
    fn read_mem_u16(&mut self, address: u16) -> u16 {
        let low = self.memory[address as usize];
        let high = self.memory[address.wrapping_add(1) as usize];
        u16::from_le_bytes([low, high])
    }

    #[inline]
    fn write_mem(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }

    #[inline]
    fn write_mem_u16(&mut self, address: u16, value: u16) {
        let [low, high] = value.to_le_bytes();
        self.memory[address as usize] = low;
        self.memory[address.wrapping_add(1) as usize] = high;
    }

    // INFO: The stack goes from $0100 to $01FF
    // It is a descending empty stack: it grows downwards and first writes and
    // the stack pointer is moved after pushing and before pulling

    #[inline]
    fn stack_push(&mut self, value: u8) {
        let address = 0x0100 + (self.stack_pointer as u16);
        self.write_mem(address, value);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    #[inline]
    fn stack_pull(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        let address = 0x0100 + (self.stack_pointer as u16);
        self.read_mem(address)
    }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter, // Will read the immediate value
            AddressingMode::ZeroPage => self.read_mem(self.program_counter) as u16,
            AddressingMode::ZeroPageX => {
                self.read_mem(self.program_counter).wrapping_add(self.reg_x) as u16
            }
            AddressingMode::ZeroPageY => {
                self.read_mem(self.program_counter).wrapping_add(self.reg_y) as u16
            }
            AddressingMode::Absolute => self.read_mem_u16(self.program_counter),
            AddressingMode::AbsoluteX => self
                .read_mem_u16(self.program_counter)
                .wrapping_add(self.reg_x as u16),
            AddressingMode::AbsoluteY => self
                .read_mem_u16(self.program_counter)
                .wrapping_add(self.reg_y as u16),
            AddressingMode::Indirect => {
                let ptr = self.read_mem_u16(self.program_counter);

                self.read_mem_u16(ptr)
            }
            AddressingMode::IndirectX => {
                let address = self.read_mem(self.program_counter).wrapping_add(self.reg_x);

                let low = self.read_mem(address as u16);
                let high = self.read_mem(address.wrapping_add(1) as u16);
                (high as u16) << 8 | (low as u16)
            }
            AddressingMode::IndirectY => {
                let address = self.read_mem(self.program_counter);

                let low = self.read_mem(address as u16);
                let high = self.read_mem(address.wrapping_add(1) as u16);
                let built_address = (high as u16) << 8 | (low as u16);
                built_address.wrapping_add(self.reg_y as u16)
            }
            AddressingMode::Accumulator => 0, // We work with the accumulator register
            _ => {
                panic!("mode {:?} is not implemented yet!", mode);
            }
        }
    }

    fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let sum: u16 = self.reg_a as u16 + value as u16 + (self.status & CARRY_FLAG) as u16;

        self.debug_state();
        println!("Value to add: {:x}", value);
        println!("Sum: {:x}", sum);

        if sum > 0xff {
            self.set_flag(CARRY_FLAG);
        } else {
            self.clear_flag(CARRY_FLAG);
        }

        let result = sum as u8;

        if (result ^ self.reg_a) & (result ^ value) & 0x80 != 0 {
            self.set_flag(OVERFLOW_FLAG);
        } else {
            self.clear_flag(OVERFLOW_FLAG);
        }

        self.reg_a = result;
        self.set_zero_and_negative_flags(result);
    }

    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = self.reg_a & value;

        self.reg_a = result;
        self.set_zero_and_negative_flags(result);
    }

    fn asl(&mut self, mode: &AddressingMode) {
        let value: u8;
        let result: u8;

        if *mode == AddressingMode::Accumulator {
            value = self.reg_a;
            result = value << 1;
            self.reg_a = result;
        } else {
            let address = self.get_operand_address(mode);
            value = self.read_mem(address);
            result = value << 1;
            self.write_mem(address, result);
        }

        if value & 0x80 != 0 {
            self.set_flag(CARRY_FLAG);
        } else {
            self.clear_flag(CARRY_FLAG);
        }

        self.set_zero_and_negative_flags(result);
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = self.reg_a & value;

        if result == 0x00 {
            self.set_flag(ZERO_FLAG);
        } else {
            self.clear_flag(ZERO_FLAG);
        }

        if value & 0x40 != 0 {
            self.set_flag(OVERFLOW_FLAG);
        } else {
            self.clear_flag(OVERFLOW_FLAG);
        }

        if value & 0x80 != 0 {
            self.set_flag(NEGATIVE_FLAG);
        } else {
            self.clear_flag(NEGATIVE_FLAG);
        }
    }

    fn clc(&mut self) {
        self.clear_flag(CARRY_FLAG);
    }

    fn cmp(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = self.reg_a - value;

        if self.reg_a >= value {
            self.set_flag(CARRY_FLAG);
        } else {
            self.clear_flag(CARRY_FLAG);
        }

        self.set_zero_and_negative_flags(result);
    }

    fn cpx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = self.reg_x - value;

        if self.reg_x >= value {
            self.set_flag(CARRY_FLAG);
        } else {
            self.clear_flag(CARRY_FLAG);
        }

        self.set_zero_and_negative_flags(result);
    }

    fn cpy(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = self.reg_y - value;

        if self.reg_y >= value {
            self.set_flag(CARRY_FLAG);
        } else {
            self.clear_flag(CARRY_FLAG);
        }

        self.set_zero_and_negative_flags(result);
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.program_counter = address;
    }

    fn jsr(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);

        self.program_counter += 1;
        // RTS adds one to the PC
        let return_address = ((self.program_counter >> 8) | (self.program_counter & 0xFF));

        self.stack_push(((self.program_counter >> 8) as u8));
        self.stack_push(((self.program_counter & 0xFF) as u8));

        self.program_counter = address;
    }

    fn dec(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = value.wrapping_sub(1);

        self.write_mem(address, result);
        self.set_zero_and_negative_flags(result);
    }

    fn dex(&mut self) {
        let value = self.reg_x.wrapping_sub(1);
        self.reg_x = value;
        self.set_zero_and_negative_flags(value);
    }

    fn dey(&mut self) {
        let value = self.reg_y.wrapping_sub(1);
        self.reg_y = value;
        self.set_zero_and_negative_flags(value);
    }

    fn sec(&mut self) {
        self.set_flag(CARRY_FLAG);
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);

        self.reg_a = value;
        self.set_zero_and_negative_flags(value);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);

        self.reg_x = value;
        self.set_zero_and_negative_flags(value);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);

        self.reg_y = value;
        self.set_zero_and_negative_flags(value);
    }

    fn rts(&mut self) {
        let low = self.stack_pull();
        let high = self.stack_pull();
        let return_address = (high as u16) << 8 | (low as u16);

        self.program_counter = return_address + 1;
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.write_mem(address, self.reg_a);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.write_mem(address, self.reg_x);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.write_mem(address, self.reg_y);
    }

    // 0xAA
    fn tax(&mut self) {
        let value: u8 = self.reg_a;
        self.reg_x = value;
        self.set_zero_and_negative_flags(value);
    }

    fn txa(&mut self) {
        let value = self.reg_x;
        self.reg_a = value;
        self.set_zero_and_negative_flags(value);
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);
        let result = value.wrapping_add(1);

        self.write_mem(address, result);
        self.set_zero_and_negative_flags(result);
    }

    fn inx(&mut self) {
        let value = self.reg_x + 1;
        self.reg_x = value;
        self.set_zero_and_negative_flags(value);
    }

    fn set_zero_and_negative_flags(&mut self, value: u8) {
        if value == 0x00 {
            self.set_flag(ZERO_FLAG);
        } else {
            self.clear_flag(ZERO_FLAG);
        }

        if (value & 0x80) != 0 {
            self.set_flag(NEGATIVE_FLAG);
        } else {
            self.clear_flag(NEGATIVE_FLAG);
        }
    }

    fn load_program(&mut self, program: Vec<u8>) {
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program);
        self.write_mem_u16(0xFFFC, 0x8000);
    }

    fn run(&mut self) {
        let ref opcodes = *opcodes::OPCODES_MAP;

        loop {
            let code = self.read_mem(self.program_counter);
            self.program_counter += 1;
            let program_counter_snapshot = self.program_counter;

            let opcode = opcodes
                .get(&code)
                .expect(&format!("OpCode {:x} is not implemented yet!", code));

            println!("{:?} at {:x}", opcode, self.program_counter - 1);

            match code {
                0x69 | 0x65 | 0x75 | 0x6d | 0x7d | 0x79 | 0x61 | 0x71 => self.adc(&opcode.mode),
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => self.and(&opcode.mode),
                0x0a | 0x06 | 0x16 | 0x0e | 0x1e => self.asl(&opcode.mode),
                0x24 | 0x2c => self.bit(&opcode.mode),
                0x18 => self.clc(),
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 => self.cmp(&opcode.mode),
                0xe0 | 0xe4 | 0xec => self.cpx(&opcode.mode),
                0xc0 | 0xc4 | 0xcc => self.cpy(&opcode.mode),
                0x4c | 0x6c => self.jmp(&opcode.mode),
                0x20 => self.jsr(&opcode.mode),
                0xc6 | 0xd6 | 0xce | 0xde => self.dec(&opcode.mode),
                0xca => self.dex(),
                0x88 => self.dey(),
                0x38 => self.sec(),
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => self.lda(&opcode.mode),
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => self.ldx(&opcode.mode),
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => self.ldy(&opcode.mode),
                0x60 => self.rts(),
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => self.sta(&opcode.mode),
                0x86 | 0x96 | 0x8e => self.stx(&opcode.mode),
                0x84 | 0x94 | 0x8c => self.sty(&opcode.mode),
                0xe6 | 0xf6 | 0xee | 0xfe => self.inc(&opcode.mode),
                0xe8 => self.inx(),
                0x8a => self.txa(),
                0xaa => self.tax(),
                0xea => continue,
                0x00 => return,
                _ => todo!(),
            }

            if program_counter_snapshot == self.program_counter {
                self.program_counter += (opcode.bytes - 1) as u16;
            }
        }
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load_program(program);
        self.reset();
        self.run();
    }

    pub fn debug_state(&self) {
        println!("================ CPU STATE ================");
        println!("A  = 0x{:02X} ({})", self.reg_a, self.reg_a);
        println!("X  = 0x{:02X} ({})", self.reg_x, self.reg_x);
        println!("Y  = 0x{:02X} ({})", self.reg_y, self.reg_y);
        println!(
            "PC = 0x{:04X} ({})",
            self.program_counter, self.program_counter
        );
        println!("SP = 0x{:02X} ({})", self.stack_pointer, self.stack_pointer);
        println!("STATUS = 0b{:08b}", self.status);
        println!("FLAGS: {}", self.flags_string());
        println!("==========================================");
    }

    pub fn flags_string(&self) -> String {
        format!(
            "N:{} V:{} D:{} I:{} Z:{} C:{}",
            if self.get_flag(NEGATIVE_FLAG) { 1 } else { 0 },
            if self.get_flag(OVERFLOW_FLAG) { 1 } else { 0 },
            if self.get_flag(DECIMAL_FLAG) { 1 } else { 0 },
            if self.get_flag(IRQ_DISABLE_FLAG) {
                1
            } else {
                0
            },
            if self.get_flag(ZERO_FLAG) { 1 } else { 0 },
            if self.get_flag(CARRY_FLAG) { 1 } else { 0 },
        )
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0x69_adc_immediate_basic() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x10, 0x69, 0x20, 0x00]);
        assert_eq!(cpu.reg_a, 0x30);
        assert!(cpu.status & CARRY_FLAG == 0);
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & NEGATIVE_FLAG == 0);
        assert!(cpu.status & OVERFLOW_FLAG == 0);
    }

    #[test]
    fn test_0x69_adc_immediate_carry_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xff, 0x69, 0x02, 0x00]);
        assert_eq!(cpu.reg_a, 0x01);
        assert!(cpu.status & CARRY_FLAG != 0); // Overflow en unsigned
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0x69_adc_immediate_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xff, 0x69, 0x01, 0x00]);
        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & CARRY_FLAG != 0);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0x69_adc_immediate_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x7f, 0x69, 0x02, 0x00]);
        assert_eq!(cpu.reg_a, 0x81);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0x69_adc_immediate_overflow_positive_to_negative() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x7f, 0x69, 0x01, 0x00]);
        assert_eq!(cpu.reg_a, 0x80);
        assert!(cpu.status & OVERFLOW_FLAG != 0);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
        assert!(cpu.status & CARRY_FLAG == 0);
    }

    #[test]
    fn test_0x69_adc_immediate_overflow_negative_to_positive() {
        let mut cpu = CPU::new();
        // -128 + -1 = -129, pero en u8 wrap around (overflow en signed)
        cpu.load_and_run(vec![0xa9, 0x80, 0x69, 0x80, 0x00]);
        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & OVERFLOW_FLAG != 0);
        assert!(cpu.status & CARRY_FLAG != 0);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0x69_adc_immediate_with_carry() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0x38, 0xa9, 0x10, 0x69, 0x20, 0x00]); // SEC, LDA #$10, ADC #$20
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.reg_a, 0x31); // 0x10 + 0x20 + 1 (carry)
    }

    #[test]
    fn test_0x65_adc_zero_page() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x20);
        cpu.load_and_run(vec![0xa9, 0x10, 0x65, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x30);
    }

    #[test]
    fn test_0x75_adc_zero_page_x() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x13, 0x20);
        cpu.load_program(vec![0xa9, 0x10, 0x75, 0x10, 0x00]);
        cpu.reset();
        cpu.reg_x = 0x03;
        cpu.run();
        assert_eq!(cpu.reg_a, 0x30);
    }

    #[test]
    fn test_0x6d_adc_absolute() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x2050, 0x20);
        cpu.load_and_run(vec![0xa9, 0x10, 0x6d, 0x50, 0x20, 0x00]);
        assert_eq!(cpu.reg_a, 0x30);
    }

    #[test]
    fn test_adc_chain() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![
            0xa9, 0x01, // LDA #$01
            0x69, 0x01, // ADC #$01
            0x69, 0x01, // ADC #$01
            0x69, 0x01, // ADC #$01
            0x00, // BRK
        ]);
        assert_eq!(cpu.reg_a, 0x04);
    }

    #[test]
    fn test_0x29_and_immediate() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b1111_0000, 0x29, 0b1010_1010, 0x00]);
        assert_eq!(cpu.reg_a, 0b1010_0000);
    }

    #[test]
    fn test_0x29_and_immediate_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b1111_0000, 0x29, 0b0000_1111, 0x00]);
        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0x29_and_immediate_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b1111_0000, 0x29, 0b1010_1010, 0x00]);
        assert_eq!(cpu.reg_a, 0b1010_0000);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0x25_and_zero_page() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0b1010_1010);
        cpu.load_and_run(vec![0xa9, 0b1111_0000, 0x25, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0b1010_0000);
    }

    #[test]
    fn test_0x35_and_zero_page_x() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x13, 0b1010_1010);
        cpu.load_program(vec![0xa9, 0b1111_0000, 0x35, 0x10, 0x00]);
        cpu.reset();
        cpu.reg_x = 0x03;
        cpu.run();
        assert_eq!(cpu.reg_a, 0b1010_0000);
    }

    #[test]
    fn test_0x0a_asl_accumulator() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b0000_0101, 0x0a, 0x00]);
        assert_eq!(cpu.reg_a, 0b0000_1010);
        assert!(cpu.status & CARRY_FLAG == 0);
    }

    #[test]
    fn test_0x0a_asl_accumulator_carry_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b1000_0001, 0x0a, 0x00]);
        assert_eq!(cpu.reg_a, 0b0000_0010);
        assert!(cpu.status & CARRY_FLAG != 0);
    }

    #[test]
    fn test_0x0a_asl_accumulator_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b1000_0000, 0x0a, 0x00]);
        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
        assert!(cpu.status & CARRY_FLAG != 0);
    }

    #[test]
    fn test_0x0a_asl_accumulator_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b0100_0000, 0x0a, 0x00]);
        assert_eq!(cpu.reg_a, 0b1000_0000);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0x06_asl_zero_page() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0b0000_0101);
        cpu.load_and_run(vec![0x06, 0x10, 0x00]);
        assert_eq!(cpu.read_mem(0x10), 0b0000_1010);
    }

    #[test]
    fn test_0x24_bit_zero_page_zero_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0b0000_1111);
        cpu.load_and_run(vec![0xa9, 0b1111_0000, 0x24, 0x10, 0x00]);
        // A & memory = 0b1111_0000 & 0b0000_1111 = 0
        assert!(cpu.status & ZERO_FLAG != 0);
        assert!(cpu.status & OVERFLOW_FLAG == 0); // bit 6 of memory is 0
        assert!(cpu.status & NEGATIVE_FLAG == 0); // bit 7 of memory is 0
    }

    #[test]
    fn test_0x24_bit_zero_page_negative_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0b1000_0000);
        cpu.load_and_run(vec![0xa9, 0b1111_1111, 0x24, 0x10, 0x00]);
        // A & memory = 0b1111_1111 & 0b1000_0000 = 0b1000_0000 (not zero)
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & OVERFLOW_FLAG == 0); // bit 6 of memory is 0
        assert!(cpu.status & NEGATIVE_FLAG != 0); // bit 7 of memory is 1
    }

    #[test]
    fn test_0x24_bit_zero_page_overflow_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0b0100_0000);
        cpu.load_and_run(vec![0xa9, 0b1111_1111, 0x24, 0x10, 0x00]);
        // A & memory = 0b1111_1111 & 0b0100_0000 = 0b0100_0000 (not zero)
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & OVERFLOW_FLAG != 0); // bit 6 of memory is 1
        assert!(cpu.status & NEGATIVE_FLAG == 0); // bit 7 of memory is 0
    }

    #[test]
    fn test_0xc9_cmp_immediate_equal() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x50, 0xc9, 0x50, 0x00]);
        assert!(cpu.status & ZERO_FLAG != 0); // A == memory
        assert!(cpu.status & CARRY_FLAG != 0); // A >= memory
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0xc9_cmp_immediate_greater() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x50, 0xc9, 0x30, 0x00]);
        assert!(cpu.status & ZERO_FLAG == 0); // A != memory
        assert!(cpu.status & CARRY_FLAG != 0); // A >= memory
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0xc9_cmp_immediate_less() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x30, 0xc9, 0x50, 0x00]);
        assert!(cpu.status & ZERO_FLAG == 0); // A != memory
        assert!(cpu.status & CARRY_FLAG == 0); // A < memory
        assert!(cpu.status & NEGATIVE_FLAG != 0); // Result is negative
    }

    #[test]
    fn test_0xe0_cpx_immediate_less() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xe0, 0x50, 0x00]);
        cpu.reset();
        cpu.reg_x = 0x30;
        cpu.run();
        assert!(cpu.status & CARRY_FLAG == 0);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0xc0_cpy_immediate_greater() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xc0, 0x30, 0x00]);
        cpu.reset();
        cpu.reg_y = 0x50;
        cpu.run();
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & CARRY_FLAG != 0);
    }

    #[test]
    fn test_0xc6_dec_zero_page_wraparound() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x00);
        cpu.load_and_run(vec![0xc6, 0x10, 0x00]);
        assert_eq!(cpu.read_mem(0x10), 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0xc6_dec_zero_page_zero_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x01);
        cpu.load_and_run(vec![0xc6, 0x10, 0x00]);
        assert_eq!(cpu.read_mem(0x10), 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0xca_dex_decrement_x() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xca, 0x00]);
        cpu.reset();
        cpu.reg_x = 10;
        cpu.run();
        assert_eq!(cpu.reg_x, 9);
    }

    #[test]
    fn test_0xca_dex_wraparound() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xca, 0x00]);
        cpu.reset();
        cpu.reg_x = 0x00;
        cpu.run();
        assert_eq!(cpu.reg_x, 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0xca_dex_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xca, 0x00]);
        cpu.reset();
        cpu.reg_x = 0x01;
        cpu.run();
        assert_eq!(cpu.reg_x, 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0x88_dey_wraparound() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0x88, 0x00]);
        cpu.reset();
        cpu.reg_y = 0x00;
        cpu.run();
        assert_eq!(cpu.reg_y, 0xFF);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0xa2_ldx_immediate() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa2, 0x55, 0x00]);
        assert_eq!(cpu.reg_x, 0x55);
    }

    #[test]
    fn test_0xa6_ldx_zero_page() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x55);
        cpu.load_and_run(vec![0xa6, 0x10, 0x00]);
        assert_eq!(cpu.reg_x, 0x55);
    }

    #[test]
    fn test_0xb6_ldx_zero_page_y() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x13, 0x55);
        cpu.load_program(vec![0xb6, 0x10, 0x00]);
        cpu.reset();
        cpu.reg_y = 0x03;
        cpu.run();
        assert_eq!(cpu.reg_x, 0x55);
    }

    #[test]
    fn test_0xa0_ldy_immediate() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa0, 0x55, 0x00]);
        assert_eq!(cpu.reg_y, 0x55);
    }

    #[test]
    fn test_0xa4_ldy_zero_page() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x55);
        cpu.load_and_run(vec![0xa4, 0x10, 0x00]);
        assert_eq!(cpu.reg_y, 0x55);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x55);
        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x55);
    }

    #[test]
    fn test_lda_from_memory_zero_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x00);
        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_lda_from_memory_negative_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x85);
        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x85);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }

    #[test]
    fn test_0x20_jsr_and_0x60_rts() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![
            0xa9, 0x00, // $8000: LDA #$00
            0x20, 0x0A, 0x80, // $8002: JSR $8009
            0xa9, 0x33, // $8005: LDA #$33
            0x00, // $8007: BRK
            0xa9, 0xff, // $8008: padding
            0xa9, 0x11, // $800A: LDA #$11
            0xe8, // $800C: INX
            0x60, // $800D: RTS
        ]);
        cpu.reset();
        cpu.run();

        assert_eq!(cpu.reg_a, 0x33);
        assert_eq!(cpu.reg_x, 1);
    }

    #[test]
    fn test_0x20_jsr_pushes_correct_return_address() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![
            0x20, 0x06, 0x80, // $8000: JSR $8006
            0x00, // $8003: BRK
            0xff, 0xff, // padding
            0xa9, 0x42, // $8006: LDA #$42
            0x60, // $8008: RTS
        ]);
        cpu.reset();
        cpu.run();

        // Debe haber vuelto y ejecutado BRK
        assert_eq!(cpu.reg_a, 0x42);
    }

    #[test]
    fn test_stack_operations() {
        let mut cpu = CPU::new();
        cpu.reset();

        assert_eq!(cpu.stack_pointer, 0xFF);

        cpu.stack_push(0x12);
        assert_eq!(cpu.stack_pointer, 0xFE);
        cpu.stack_push(0x34);
        assert_eq!(cpu.stack_pointer, 0xFD);

        assert_eq!(cpu.stack_pull(), 0x34);
        assert_eq!(cpu.stack_pointer, 0xFE);
        assert_eq!(cpu.stack_pull(), 0x12);
        assert_eq!(cpu.stack_pointer, 0xFF);
    }

    #[test]
    fn test_nested_jsr() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![
            0x20, 0x09, 0x80, // $8000: JSR 8009
            0xa9, 0x99, // $8003: LDA #$99
            0x00, // $8005: BRK
            0xff, 0xff, 0xff, // padding
            0xa9, 0x11, // $8009: LDA #$11
            0x20, 0x0f, 0x80, // $800B: JSR 800F
            0xe8, // $800E: INX
            0x60, // $800F: RTS
            0xa9, 0x22, // $8010: LDA #$22
            0xe8, // $8012: INX
            0x60, // $8013: RTS
        ]);
        cpu.reset();
        cpu.run();

        assert_eq!(cpu.reg_a, 0x99);
        assert_eq!(cpu.reg_x, 1);
    }

    #[test]
    fn test_0x85_sta_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0x85, 0xBA, 0x00]);
        cpu.reset();
        cpu.reg_a = 0x4F;
        cpu.run();
        assert_eq!(cpu.read_mem(0xBA), 0x4F);
    }

    #[test]
    fn test_0x86_stx_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0x86, 0x10, 0x00]);
        cpu.reset();
        cpu.reg_x = 0x55;
        cpu.run();
        assert_eq!(cpu.read_mem(0x10), 0x55);
    }

    #[test]
    fn test_0x84_sty_zero_page() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0x84, 0x10, 0x00]);
        cpu.reset();
        cpu.reg_y = 0x55;
        cpu.run();
        assert_eq!(cpu.read_mem(0x10), 0x55);
    }

    #[test]
    fn test_0xe6_inc_zero_page_wraparound() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0xFF);
        cpu.load_and_run(vec![0xe6, 0x10, 0x00]);
        assert_eq!(cpu.read_mem(0x10), 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0xe6_inc_zero_page_negative_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x10, 0x7F);
        cpu.load_and_run(vec![0xe6, 0x10, 0x00]);
        assert_eq!(cpu.read_mem(0x10), 0x80);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }
    #[test]
    fn test_0xe8_inx_inc_x() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xe8, 0x00]);
        cpu.reset();
        cpu.reg_x = 10;
        cpu.run();
        assert_eq!(cpu.reg_x, 11);
    }

    #[test]
    fn test_0xe8_inx_inc_x_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xe8, 0x00]);
        cpu.reset();
        cpu.reg_x = 0xFF;
        cpu.run();
        assert_eq!(cpu.reg_x, 0);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0x4c_jmp_absolute() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![
            0x4c, 0x05, 0x80, // JMP $8005
            0xa9, 0xff, // LDA #$FF
            0xa9, 0x42, // LDA #$42
            0x00, // BRK
        ]);
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.reg_a, 0x42);
    }

    #[test]
    fn test_0x6c_jmp_indirect() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x0120, 0x05);
        cpu.write_mem(0x0121, 0x80);

        cpu.load_program(vec![
            0x6c, 0x20, 0x01, // JMP ($0120)
            0xa9, 0xff, // LDA #$FF
            0xa9, 0x42, // LDA #$42
            0x00, // BRK
        ]);
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.reg_a, 0x42);
    }

    #[test]
    fn test_0x4c_jmp_absolute_forward() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![
            0xa9, 0x10, // LDA #$10
            0x4c, 0x09, 0x80, // JMP $8009
            0xa9, 0x20, // LDA #$20
            0xa9, 0x30, // LDA #$30
            0xe8, // INX
            0x00, // BRK
        ]);
        cpu.reset();
        cpu.run();
        assert_eq!(cpu.reg_a, 0x10);
        assert_eq!(cpu.reg_x, 1);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.reg_a = 10;
        cpu.run();
        assert_eq!(cpu.reg_x, cpu.reg_a);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.reg_a = 0;
        cpu.run();
        assert_eq!(cpu.reg_x, cpu.reg_a);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0x8a_txa_move_x_to_a() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0x8a, 0x00]);
        cpu.reset();
        cpu.reg_x = 10;
        cpu.run();
        assert_eq!(cpu.reg_a, 10);
    }
}
