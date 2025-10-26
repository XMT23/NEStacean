#[derive(Debug)]
pub struct CPU {
    pub reg_a: u8,
    pub reg_x: u8,
    pub reg_y: u8,
    pub status: u8, // FLAGS
    pub program_counter: u16,
    pub stack_pointer: u8,
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

            program_counter: 0,
            stack_pointer: 0,
        }
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

    pub fn execute_program(&mut self, program: Vec<u8>) {
        self.program_counter = 0;

        loop {
            let opcode = program[self.program_counter as usize];
            self.program_counter += 1;

            match opcode {
                // LDA
                0xA9 => {
                    let value = program[self.program_counter as usize];
                    self.program_counter += 1;

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

                    self.reg_a = value;
                }

                // BRK
                0x00 => {
                    return;
                }
                _ => todo!(),
            }
        }
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
    fn test_0xa9_lda_immediate_no_flags() {
        let mut cpu = CPU::new();
        cpu.execute_program(vec![0xA9, 0x03, 0x00]);
        assert_eq!(cpu.reg_a, 0x03);
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0xa9_lda_immediate_zero_flag() {
        let mut cpu = CPU::new();
        cpu.execute_program(vec![0xA9, 0x00, 0x00]);
        assert_eq!(cpu.reg_a, 0x00);
        assert!(cpu.status & ZERO_FLAG != 0);
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0xa9_lda_immediate_negative_flag() {
        let mut cpu = CPU::new();
        cpu.execute_program(vec![0xA9, 0x9C, 0x00]);
        assert_eq!(cpu.reg_a, 0x9C);
        assert!(cpu.status & ZERO_FLAG == 0);
        assert!(cpu.status & NEGATIVE_FLAG != 0);
    }
}
