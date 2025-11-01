use crate::{memory::Memory, rom::Rom};

#[derive(Debug)]
pub struct Bus {
    pub cpu_ram: [u8; 0xFFFF], // 2048
    pub rom: Rom,
}

// Ram goes from 0x0000 - 0x2000
const RAM_START: u16 = 0x0000;
const RAM_END: u16 = 0x1FFF;
const PPU_REGISTERS_START: u16 = 0x2000;
const PPU_REGISTERS_END: u16 = 0x3FFF;

impl Memory for Bus {
    // NOTE: In the future we will have to make a distintion depending on if we are
    // at a RAM address, a PPU register, etc...

    #[inline]
    fn read_mem(&self, address: u16) -> u8 {
        match address {
            RAM_START..=RAM_END => self.cpu_ram[(address & 0b00000111_11111111) as usize],
            PPU_REGISTERS_START..=PPU_REGISTERS_END => todo!("PPU registers not implemented!"),
            0x8000..=0xFFFF => self.read_prg_rom(address),
            _ => {
                println!("Ignoring mem access to {address}");
                return 0;
            }
        }
    }

    #[inline]
    fn write_mem(&mut self, address: u16, value: u8) {
        match address {
            RAM_START..=RAM_END => self.cpu_ram[(address & 0b111_11111111) as usize] = value,
            PPU_REGISTERS_START..=PPU_REGISTERS_END => todo!("PPU registers not implemented!"),
            0x8000..=0xFFFF => panic!("Attempt to write to ROM PRG space"),
            _ => {
                println!("Ignoring mem access to {address}");
            }
        }
    }
}

impl Bus {
    pub fn new(rom: Rom) -> Self {
        Bus {
            cpu_ram: [0; 0xFFFF],
            rom: rom,
        }
    }

    fn read_prg_rom(&self, mut address: u16) -> u8 {
        address -= 0x8000;
        if self.rom.prg.len() == 0x4000 && address >= 0x4000 {
            // Mirror
            address = address % 0x4000;
        }
        self.rom.prg[address as usize]
    }
}
