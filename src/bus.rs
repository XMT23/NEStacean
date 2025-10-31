use crate::memory::Memory;

#[derive(Debug)]
pub struct Bus {
    pub cpu_ram: [u8; 0xFFFF], // 2048
}

// Ram goes from 0x0000 - 0x2000
const RAM_ADDR: u16 = 0x0000;
const MIRROR_ENDING: u16 = 0x1FFF;

impl Memory for Bus {
    // NOTE: In the future we will have to make a distintion depending on if we are
    // at a RAM address, a PPU register, etc...

    #[inline]
    fn read_mem(&self, address: u16) -> u8 {
        self.cpu_ram[address as usize]
    }

    #[inline]
    fn write_mem(&mut self, address: u16, value: u8) {
        self.cpu_ram[address as usize] = value;
    }
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            cpu_ram: [0; 0xFFFF],
        }
    }
}
