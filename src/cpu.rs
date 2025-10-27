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

#[derive(Debug)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    IndirectX,
    IndirectY,
}

const CARRY_FLAG: u8 = 0b00000001;
const ZERO_FLAG: u8 = 0b00000010;
const IRQ_DISABLE_FLAG: u8 = 0b00000100;
const DECIMAL_FLAG: u8 = 0b00001000;
const INDEX_REGISTER_SELECT_FLAG: u8 = 0b00010000;
const ACCUMULATOR_SELECT_FLAG: u8 = 0b00100000;
const OVERFLOW_FLAG: u8 = 0b01000000;
const NEGATIVE_FLAG: u8 = 0b10000000;

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

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,
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
            _ => {
                panic!("mode {:?} is not implemented yet!", mode);
            }
        }
    }

    // 0xA9
    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.read_mem(address);

        self.reg_a = value;
        self.set_zero_and_negative_flags(value);
    }

    // 0xAA
    fn tax(&mut self) {
        let value: u8 = self.reg_a;
        self.reg_x = value;
        self.set_zero_and_negative_flags(value);
    }

    // 0xAA
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
        loop {
            let opcode = self.memory[self.program_counter as usize];
            self.program_counter += 1;

            match opcode {
                // LDA
                0xa9 => {
                    self.lda(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xa5 => {
                    self.lda(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0xb5 => {
                    self.lda(&AddressingMode::ZeroPageX);
                    self.program_counter += 1;
                }
                0xad => {
                    self.lda(&AddressingMode::Absolute);
                    self.program_counter += 2;
                }
                0xbd => {
                    self.lda(&AddressingMode::AbsoluteX);
                    self.program_counter += 2;
                }
                0xb9 => {
                    self.lda(&AddressingMode::AbsoluteY);
                    self.program_counter += 2;
                }
                0xa1 => {
                    self.lda(&AddressingMode::IndirectX);
                    self.program_counter += 1;
                }
                0xb1 => {
                    self.lda(&AddressingMode::IndirectY);
                    self.program_counter += 1;
                }

                0xaa => self.tax(),
                0xe8 => self.inx(),
                0x00 => return,
                _ => todo!(),
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
    fn test_lda_from_memory_absolute() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x2050, 0x55);

        cpu.load_and_run(vec![0xad, 0x50, 0x20, 0x00]);

        assert_eq!(cpu.reg_a, 0x55);
    }

    #[test]
    fn test_lda_from_memory_absolute_zero_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x2050, 0x00);

        cpu.load_and_run(vec![0xad, 0x50, 0x20, 0x00]);

        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_lda_from_memory_absolute_negative_flag() {
        let mut cpu = CPU::new();
        cpu.write_mem(0x2050, 0x85);

        cpu.load_and_run(vec![0xad, 0x50, 0x20, 0x00]);

        assert_eq!(cpu.reg_a, 0x85);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
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
    fn test_0xaa_tax_move_a_to_x_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.reg_a = 10 | 0x80;
        cpu.run();
        assert_eq!(cpu.reg_x, cpu.reg_a);
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
    fn test_0xe8_inx_inc_x_negative_flag() {
        let mut cpu = CPU::new();
        cpu.load_program(vec![0xe8, 0x00]);
        cpu.reset();
        cpu.reg_x = 10 | 0x80;
        cpu.run();
        assert_eq!(cpu.reg_x, (10 | 0x80) + 1);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }
}
