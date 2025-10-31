use crate::CPU;

pub trait Memory {
    fn read_mem(&self, address: u16) -> u8;

    #[inline]
    fn read_mem_u16(&self, address: u16) -> u16 {
        let low = self.read_mem(address) as u16;
        let high = self.read_mem(address + 1) as u16;

        return (high << 8) | low;
    }

    fn write_mem(&mut self, address: u16, value: u8);

    #[inline]
    fn write_mem_u16(&mut self, address: u16, value: u16) {
        let low = (value & 0x00FF) as u8;
        let high = (value >> 8) as u8;

        self.write_mem(address, low);
        self.write_mem(address + 1, high);
    }
}
