// The NES's 2A03 is a modified version of the 6502 chip. 
// As with any CPU, the goal of this module is to execute the main program instructions.

// Memory Map:
// RAM:             [0x0000 ... 0x2000]
// IO Registers:    [0x2000 ... 0x4020]
// Expansion ROM:   [0x4020 ... 0x6000]
// Save RAM:        [0x6000 ... 0x8000]
// Program ROM:     [0x8000 ... 0xFFFF]

// Registers:
// Program Counter:     Next Instruction Address
// Stack Pointer:       [0x100 ... 0x1FF] Stack Address, top to bottom
// Accumulator:         Stores the result of arithmetic, logic, and memory operations
// Index X:             General Register
// Index Y:             General Register
// Processor Status:    Represents 7 status flags
// NV1B DIZC:           Negative, Overflow, True, Special Use, Decimal, Interrupt Disable, Zero, Carry

use std::collections::HashMap;
use crate::opcodes::{self, OPCode};

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub register_sp: u8,
    pub register_pc: u16,
    pub flags: Flags,
    memory: [u8; 0xFFFF],
}

pub struct Flags {
    pub bits: u8
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
    NoneAddressing,
}

trait Mem {
    // Read the data byte at a spectific adddress
    fn mem_read(&self, addr: u16) -> u8;

    // Write a data byte a specific memory address
    fn mem_write(&mut self, addr: u16, data: u8);

    // Read two data bytes in little endian format at address
    fn mem_read_16(&self, addr: u16) -> u16 {
        let lo = self.mem_read(addr) as u16;
        let hi = self.mem_read(addr + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    // Write two data bytes in little endian format at address
    fn mem_write_16(&mut self, addr: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xFF) as u8;
        self.mem_write(addr, lo);
        self.mem_write(addr + 1, hi);
    }
}

impl Mem for CPU {
    
    fn mem_read(&self, addr: u16) -> u8 { 
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) { 
        self.memory[addr as usize] = data;
    }
}

impl CPU {
    // Initiate the CPU
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            register_sp: 0,
            register_pc: 0,
            flags: Flags::new(),
            memory: [0; 0xFFFF],
        }
    }

    // Set Flags from result
    fn update_flags(&mut self, result: u8) {
        // Set Zero
        self.flags.set_zero(result == 0);

        // Set Negative
        self.flags.set_negative(result & 0b1000_0000 != 0);
    }

    // Get the the address of operands
    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.register_pc,
            AddressingMode::ZeroPage => self.mem_read(self.register_pc) as u16,
            AddressingMode::Absolute => self.mem_read_16(self.register_pc),
            AddressingMode::ZeroPageX => {
                let pos = self.mem_read(self.register_pc);
                let addr = pos.wrapping_add(self.register_x) as u16;
                return addr;
            }
            AddressingMode::ZeroPageY => {
                let pos = self.mem_read(self.register_pc);
                let addr = pos.wrapping_add(self.register_y) as u16;
                return addr;
            }
            AddressingMode::AbsoluteX => {
                let base = self.mem_read_16(self.register_pc);
                let addr = base.wrapping_add(self.register_x as u16);
                return addr;
            }
            AddressingMode::AbsoluteY => {
                let base = self.mem_read_16(self.register_pc);
                let addr = base.wrapping_add(self.register_y as u16);
                return addr;
            }
            AddressingMode::IndirectX => {
                let base = self.mem_read(self.register_pc);
                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                return (hi as u16) << 8 | (lo as u16);
            }
            AddressingMode::IndirectY => {
                let base = self.mem_read(self.register_pc);
                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                return deref;
            }
            AddressingMode::NoneAddressing => {
                panic!("Mode {:?} is not supported", mode);
            }
        }
    }

    // Reset the Emulator to initial state and reset address
    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.flags.bits = 0;

        self.register_pc = self.mem_read_16(0xFFFC)
    }

    // Load program into memory starting at PROM location 0x8000
    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_16(0xFFFC, 0x8000);
    }

    // Decode and execute program file
    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, &'static opcodes::OPCode> = *opcodes::OPCodes_MAP;

        loop {
            let code = self.mem_read(self.register_pc);
            self.register_pc += 1;
            let program_counter_state = self.register_pc;

            let opcode = opcodes.get(&code).expect(&format!("OPCode {:x} is not recognized", code));
            
            // Check the opcode with each opcode case
            match code {
                // LDA
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                // STA
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }
                
                0xAA => self.tax(),
                0xe8 => self.inx(),
                0x00 => return,
                _ => todo!(),
            }

            if program_counter_state == self.register_pc {
                self.register_pc += (opcode.len - 1) as u16;
            }
        }
    }

    // Load a specific program and run from there
    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }


    /*  Opcodes  */

    fn adc(&mut self) {
        todo!();
    }

    fn and(&mut self) {
        todo!();
    }

    fn asl(&mut self) {
        todo!();
    }

    fn bcc(&mut self) {
        todo!();
    }

    fn bcs(&mut self) {
        todo!();
    }

    fn beq(&mut self) {
        todo!();
    }

    fn bit(&mut self) {
        todo!();
    }

    fn bmi(&mut self) {
        todo!();
    }

    fn bne(&mut self) {
        todo!();
    }

    fn bpl(&mut self) {
        todo!();
    }

    fn brk(&mut self) {
        todo!();
    }

    fn bvc(&mut self) {
        todo!();
    }

    fn bvs(&mut self) {
        todo!();
    }

    fn clc(&mut self) {
        todo!();
    }

    fn cld(&mut self) {
        todo!();
    }

    fn cli(&mut self) {
        todo!();
    }

    fn clv(&mut self) {
        todo!();
    }

    fn cmp(&mut self) {
        todo!();
    }

    fn cpx(&mut self) {
        todo!();
    }

    fn cpy(&mut self) {
        todo!();
    }

    fn dec(&mut self) {
        todo!();
    }

    fn dex(&mut self) {
        todo!();
    }

    fn dey(&mut self) {
        todo!();
    }

    fn eor(&mut self) {
        todo!();
    }

    fn inc(&mut self) {
        todo!();
    }

    // Increment X Register
    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_flags(self.register_x);
    }

    fn iny(&mut self) {
        todo!();
    }

    fn jmp(&mut self) {
        todo!();
    }

    fn jsr(&mut self) {
        todo!();
    }
    
    // Load value into the A register
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = value;

        self.update_flags(self.register_a);
    }

    fn ldx(&mut self) {
        todo!();
    }

    fn ldy(&mut self) {
        todo!();
    }

    fn lsr(&mut self) {
        todo!();
    }

    fn nop(&mut self) {
        todo!();
    }

    fn ora(&mut self) {
        todo!();
    }

    fn pha(&mut self) {
        todo!();
    }

    fn php(&mut self) {
        todo!();
    }

    fn pla(&mut self) {
        todo!();
    }

    fn plp(&mut self) {
        todo!();
    }

    fn rol(&mut self) {
        todo!();
    }

    fn ror(&mut self) {
        todo!();
    }

    fn rti(&mut self) {
        todo!();
    }

    fn rts(&mut self) {
        todo!();
    }

    fn sbc(&mut self) {
        todo!();
    }

    fn sec(&mut self) {
        todo!();
    }

    fn sed(&mut self) {
        todo!();
    }

    fn sei(&mut self) {
        todo!();
    }

    // Copy value from A to memory
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stx(&mut self) {
        todo!();
    }

    fn sty(&mut self) {
        todo!();
    }

    // Copy value from A to X
    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_flags(self.register_x);
    }

    fn tay(&mut self) {
        todo!();
    }

    fn tsx(&mut self) {
        todo!();
    }

    fn txa(&mut self) {
        todo!();
    }

    fn txs(&mut self) {
        todo!();
    }

    fn tya(&mut self) {
        todo!();
    }
}

impl Flags {
    fn new() -> Self {
        Flags { bits: 0 }
    }

    fn set_bit(&mut self, bit: u8, value: bool) {
        if value { self.bits |= 1 << bit; }
        else { self.bits &= !(1 << bit); }
    }

    fn get_bit(&self, bit: u8) -> bool {
        (self.bits & (1 << bit)) != 0
    }

    fn carry(&self) -> bool     { self.get_bit(0) }
    fn zero(&self) -> bool      { self.get_bit(1) }
    fn int(&self) -> bool       { self.get_bit(2) }
    fn decimal(&self) -> bool   { self.get_bit(3) }
    fn bflag(&self) -> bool     { self.get_bit(4) }
    fn overflow(&self) -> bool  { self.get_bit(6) }
    fn negative(&self) -> bool  { self.get_bit(7) }

    fn set_carry(&mut self, value: bool)        { self.set_bit(0, value); }
    fn set_zero(&mut self, value: bool)         { self.set_bit(1, value); }
    fn set_int(&mut self, value: bool)          { self.set_bit(2, value); }
    fn set_decimal(&mut self, value: bool)      { self.set_bit(3, value); }
    fn set_bflag(&mut self, value: bool)        { self.set_bit(4, value); }
    fn set_overflow(&mut self, value: bool)     { self.set_bit(6, value); }
    fn set_negative(&mut self, value: bool)     { self.set_bit(7, value); }
}

// Testing Functions, Not needed
#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_lda() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 0x05);
        assert!(cpu.flags.bits & 0b0000_0010 == 0b00);
        assert!((cpu.flags.bits & 0b1000_0000 == 0));
    }
    #[test]
    fn test_lda_zero() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.flags.bits & 0b0000_0010 == 0b10);
    }
    #[test]
    fn test_tax() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x0A,0xaa, 0x00]);

        assert_eq!(cpu.register_x, 10)
    }
    #[test]
    fn test_inx() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xff, 0xaa,0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1)
    }
    #[test]
    fn test_5_ops() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }
    #[test]
   fn test_lda_from_memory() {
       let mut cpu = CPU::new();
       cpu.mem_write(0x10, 0x55);

       cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

       assert_eq!(cpu.register_a, 0x55);
   }
}