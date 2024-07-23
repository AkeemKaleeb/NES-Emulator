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

use std::collections::HashMap;
use crate::opcodes::{self, OPCode};

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xFD;

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub register_sp: u8,
    pub register_pc: u16,
    pub flags: Flags,
    memory: [u8; 0xFFFF],
}

#[derive(Clone)]
pub struct Flags {
    pub bits: u8
    /* 
    N V _ B D I Z C
    | |   | | | | +---- Carry
    | |   | | | +------ Zero
    | |   | | +-------- Interrupt Disable
    | |   | +---------- Decimal (Not Used)
    | |   +------------ Break
    | +---------------- Overflow
    +------------------ Negative
    */
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
            register_sp: STACK_RESET,
            register_pc: 0,
            flags: Flags::new(),
            memory: [0; 0xFFFF],
        }
    }

    // Load program into memory starting at PROM location 0x8000
    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_16(0xFFFC, 0x8000);
    }

    // Reset the Emulator to initial state and reset address
    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.flags.bits = 0;

        self.register_pc = self.mem_read_16(0xFFFC)
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
                /* RET */ 0x00 =>                                                   return,
                /* ADC */ 0x69 | 0x65 | 0x75 | 0x6d | 0x7d | 0x79 | 0x61 | 0x71 =>  self.adc(&opcode.mode),
                /* AND */ 0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 =>  self.and(&opcode.mode),
                /* ASL */ 0x0a =>                                                   self.asl_a(),
                /* ASL */ 0x06 | 0x16 | 0x0e | 0x1e =>                              self.asl(&opcode.mode),
                /* BCC */ 0x90 =>                                                   self.bcc(),
                /* BCS */ 0xb0 =>                                                   self.bcs(),
                /* BEQ */ 0xf0 =>                                                   self.beq(),
                /* BIT */ 0x24 | 0x2c =>                                            self.bit(&opcode.mode),
                /* BMI */ 0x30 =>                                                   self.bmi(),
                /* BNE */ 0xd0 =>                                                   self.bne(),
                /* BPL */ 0x10 =>                                                   self.bpl(),
                /* BVC */ 0x50 =>                                                   self.bvc(),
                /* BVS */ 0x70 =>                                                   self.bvs(),
                /* CLC */ 0x18 =>                                                   self.clc(),
                /* CLD */ 0xd8 =>                                                   self.cld(),
                /* CLI */ 0x58 =>                                                   self.cli(),
                /* CLV */ 0xb8 =>                                                   self.clv(),
                /* CMP */ 0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 =>  self.cmp(&opcode.mode),
                /* CPX */ 0xe0 | 0xe4 | 0xec =>                                     self.cpx(&opcode.mode),
                /* CPY */ 0xc0 | 0xc4 | 0xcc =>                                     self.cpy(&opcode.mode),
                /* DEC */ 0xc6 | 0xd6 | 0xce | 0xde =>                              self.dec(&opcode.mode),
                /* DEX */ 0xca =>                                                   self.dex(),
                /* DEY */ 0x88 =>                                                   self.dey(),
                /* EOR */ 0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 =>  self.eor(&opcode.mode),
                /* INC */ 0xe6 | 0xf6 | 0xee | 0xfe =>                              self.inc(&opcode.mode),
                /* INX */ 0xe8 =>                                                   self.inx(),
                /* INY */ 0xc8 =>                                                   self.iny(),
                /* LDA */ 0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 =>  self.lda(&opcode.mode),
                /* LDX */ 0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe =>                       self.ldx(&opcode.mode),
                /* LDY */ 0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc =>                       self.ldy(&opcode.mode),
                /* LSR */ 0x4a =>                                                   self.lsr_a(),
                /* LSR */ 0x46 | 0x56 | 0x4e | 0x5e =>                              self.lsr(&opcode.mode),
                /* NOP */ 0xea =>                                                   self.nop(),
                /* ORA */ 0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 =>  self.ora(&opcode.mode),
                /* PHA */ 0x48 =>                                                   self.pha(),
                /* PHP */ 0x08 =>                                                   self.php(),
                /* PLA */ 0x68 =>                                                   self.pla(),
                /* PLP */ 0x28 =>                                                   self.plp(),
                /* ROL */ 0x2a =>                                                   self.rol_a(),
                /* ROL */ 0x26 | 0x36 | 0x2e | 0x3e =>                              self.rol(&opcode.mode),
                /* ROR */ 0x6a =>                                                   self.ror_a(),
                /* ROR */ 0x66 | 0x76 | 0x6e | 0x7e =>                              self.ror(&opcode.mode),
                /* RTI */ 0x40 =>                                                   self.rti(),
                /* RTS */ 0x60 =>                                                   self.rts(),
                /* SBC */ 0xe9 | 0xe5 | 0xf5 | 0xed | 0xfd | 0xf9 | 0xe1 | 0xf1 =>  self.sbc(&opcode.mode),
                /* SEC */ 0x38 =>                                                   self.sec(),
                /* SED */ 0xf8 =>                                                   self.sed(),
                /* SEI */ 0x78 =>                                                   self.sei(),
                /* STA */ 0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 =>         self.sta(&opcode.mode),
                /* STX */ 0x86 | 0x96 | 0x8e =>                                     self.stx(&opcode.mode),
                /* STY */ 0x84 | 0x94 | 0x8c =>                                     self.sty(&opcode.mode),
                /* TAX */ 0xAA =>                                                   self.tax(),
                /* TAY */ 0xa8 =>                                                   self.tay(),
                /* TSX */ 0xba =>                                                   self.tsx(),
                /* TXA */ 0x8a =>                                                   self.txa(),
                /* TXS */ 0x9a =>                                                   self.txs(),
                /* TYA */ 0x98 =>                                                   self.tya(),
                _ => todo!("Implement Jump Instructions"),
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

    /*                       */
    /* Opcode Helper Methods */
    /*                       */

    // Add Register A a value and set flags
    // Helper Method for ADC and SBC
    fn add_to_reg_a(&mut self, data: u8) {
        let sum = self.register_a as u16
            + data as u16 
            + self.flags.carry() as u16;
        
        let carry = sum > 0xFF;
        self.flags.set_carry(carry);

        let result = sum as u8;
        self.flags.set_overflow((data ^ result) & (result ^ self.register_a) & 0x80 != 0);

        self.register_a = result;
        self.update_flags(self.register_a);
    }

    // Branch function to change program counter based on conditions
    fn branch(&mut self, condition: bool) {
        if condition {
            let displacement: i8 = self.mem_read(self.register_pc) as i8;
            let addr = self.register_pc.wrapping_add(1).wrapping_add(displacement as u16);

            self.register_pc = addr;
        }
    }

    // Compare register with a byte of memory
    fn compare(&mut self, mode: &AddressingMode, compare_with: u8) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);

        self.flags.set_carry(compare_with >= data);
        self.update_flags(compare_with.wrapping_sub(data));
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

    // Push Value to Stack
    fn stack_push(&mut self, data: u8) {
        self.mem_write((STACK as u16) + self.register_sp as u16, data);
        self.register_sp = self.register_sp.wrapping_sub(1);
    }

    // Pop Value from the Stack
    fn stack_pop(&mut self) -> u8 {
        self.register_sp = self.register_sp.wrapping_add(1);

        return self.mem_read((STACK as u16) + self.register_sp as u16)
    }

    // Push 2 Byte Value to the Stack
    fn stack_push_16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xFF) as u8;
        self.stack_push(hi);
        self.stack_push(lo);
    }

    // Pop 2 Byte Value from the Stack
    fn stack_pop_16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;

        return hi << 8 | lo
    }

    // Set Zero and Negative Flags from result
    fn update_flags(&mut self, result: u8) {
        // Set Zero
        self.flags.set_zero(result == 0);

        // Set Negative
        self.flags.set_negative(result & 0b1000_0000 != 0);
    }

    /*           */
    /*  Opcodes  */
    /*           */

    // Add value to register A with the carry bit
    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.add_to_reg_a(value);
    }

    // Logical AND performed bit by bit on the A Register using a byte of memory
    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_a = self.register_a & value;

        self.update_flags(self.register_a);
    }

    // Shift all bits of the A Register one bit left
    fn asl_a(&mut self) {
        let mut data = self.register_a;
        self.flags.set_carry(data >> 7 == 1);

        data = data << 1;
        self.register_a = data;
    }

    // Shift all bits of the Memory contents one bit left
    fn asl(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        self.flags.set_carry(data >> 7 == 1);

        data = data << 1;
        self.mem_write(addr, data);
        self.update_flags(data);
    }

    // Branch if the carry flag is not set
    fn bcc(&mut self) {
        self.branch(!self.flags.carry());
    }

    // Branch if the carry flag is set
    fn bcs(&mut self) {
        self.branch(self.flags.carry());
    }

    // Branch if the result is Equal
    fn beq(&mut self) {
        self.branch(self.flags.zero());
    }

    // Test if one or more bits are set at a memory location
    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);
        let and = self.register_a & data;

        self.flags.set_zero(and == 0);
        self.flags.set_negative(data & 0b1000_0000 > 0);
        self.flags.set_overflow(data & 0b0100_0000 > 0);
    }

    // Branch if the result is negative
    fn bmi(&mut self) {
        self.branch(self.flags.negative());
    }

    // Branch if the result is not equal
    fn bne(&mut self) {
        self.branch(!self.flags.zero());
    }

    // Branch if the result is positve
    fn bpl(&mut self) {
        self.branch(!self.flags.negative());
    }

    // Force the generation of an interrupt request, pushing status to the stack and loading IRQ interrupt vector at $FFFE/F in the PC
    fn brk(&mut self) {
        self.stack_push_16(self.register_pc);
        self.stack_push(self.flags.bits);
        self.register_pc = self.mem_read_16(0xFFFE);
        self.flags.set_bflag(true);
    }

    // Branch if the overflow is not set adding a displacement to the program counter
    fn bvc(&mut self) {
        self.branch(!self.flags.overflow());
    }

    // Branch if the overflow is set adding a displacement to the program counter
    fn bvs(&mut self) {
        self.branch(self.flags.overflow());
    }

    // Set Carry Flag to False
    fn clc(&mut self) {
        self.flags.set_carry(false);
    }

    // Set Decimal Mode to False
    fn cld(&mut self) {
        self.flags.set_decimal(false);
    }

    // Set Interrupt Disable to False
    fn cli(&mut self) {
        self.flags.set_int(false);
    }

    // Clear the Overflow Flag
    fn clv(&mut self) {
        self.flags.set_overflow(false);
    }

    // Compare the A Register with another byte of memory
    fn cmp(&mut self, mode: &AddressingMode) {
        self.compare(mode, self.register_a);
    }

    // Compare the X Register with another byte of memory
    fn cpx(&mut self, mode: &AddressingMode) {
        self.compare(mode, self.register_x);
    }

    // Compare the Y Register with another byte of memory
    fn cpy(&mut self, mode: &AddressingMode) {
        self.compare(mode, self.register_y);
    }

    // Decrement the value of a byte in memory
    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr).wrapping_sub(1);
        self.mem_write(addr, data);
        self.update_flags(data)
    }

    // Decrement the X Register
    fn dex(&mut self) {
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_flags(self.register_x)
    }

    // Decrement the Y Register
    fn dey(&mut self) {
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_flags(self.register_y)
    }

    // Exclusive OR performed bit by bit on the A register using a byte of memory
    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);

        self.register_a = self.register_a ^ data;
        self.update_flags(self.register_a);
    }

    // Increment the value stored at a specific memory location
    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr).wrapping_add(1);
        self.mem_write(addr, data);
        self.update_flags(data)
    }

    // Increment X Register
    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_flags(self.register_x);
    }

    // Increment Y Register
    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_sub(1);
        self.update_flags(self.register_y)
    }

    // Jump to a specific program counter address
    fn jmp_abs(&mut self) {
        let addr = self.mem_read_16(self.register_pc);
        self.register_pc = addr;
    }

    // Jump to a specific program counter address
    fn jmp_ind(&mut self) {
        let addr = self.mem_read_16(self.register_pc);

        // Fixes a bug on older CPUs
        let indirect_ref = if addr & 0x00FF == 0x00FF {
            let lo = self.mem_read(addr);
            let hi = self.mem_read(addr & 0xFF00);
            (hi as u16) << 8 | (lo as u16)
        } else {self.mem_read_16(addr)};

        self.register_pc = indirect_ref;
    }

    // Jump to the subroutine and store current address on the stack
    fn jsr(&mut self) {
        self.stack_push_16(self.register_pc + 1);
        let addr = self.mem_read_16(self.register_pc);
        self.register_pc = addr;
    }
    
    // Load the A register using a byte of memory
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_a = value;

        self.update_flags(self.register_a);
    }

    // Load the X Register using a byte of memory
    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_x = value;

        self.update_flags(self.register_x);
    }

    // Load the Y Register using a byte of memory
    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        self.register_y = value;

        self.update_flags(self.register_y);
    }

    // Logical Shift A Register bits right one place
    fn lsr_a(&mut self) {
        let data = self.register_a;
        self.flags.set_carry(data & 1 == 1);

        self.register_a = data >> 1;
        self.update_flags(self.register_a);

    }

    // Logical Shift bits right one place
    fn lsr(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        self.flags.set_carry(data & 1 == 1);

        data = data >> 1;
        self.mem_write(addr, data);
        self.update_flags(data);
    }

    // No Operation, do nothing
    fn nop(&self) {
        // Do Nothing
    }

    // Logical OR performed bit by bit on the A Register using a byte of memory
    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read(addr);

        self.register_a = self.register_a | data;
        self.update_flags(self.register_a);
    }

    // Push A Register to the stack
    fn pha(&mut self) {
        self.stack_push(self.register_a);
    }

    // Push a copy of the status flags onto the stack
    fn php(&mut self) {
        let mut flags = self.flags.clone();
        flags.set_bflag(true);
        flags.set_b2flag(true);
        self.stack_push(flags.bits);
    }

    // Pull an 8 bit value from the stack into the A register
    fn pla(&mut self) {
        let data = self.stack_pop();
        self.register_a = data;
    }

    // Pull an 8 bit value from the stack into the processor flags
    fn plp(&mut self) {
        self.flags.bits = self.stack_pop();
        self.flags.set_bflag(false);
        self.flags.set_b2flag(false);
    }

    // Rotate A Register bits to the left
    fn rol_a(&mut self) {
        let mut data = self.register_a;
        let old_carry = self.flags.carry() as u8;

        self.flags.set_carry(data >> 7 == 1);
        data = data << 1;
        data = data | old_carry;

        self.register_a = data;
        self.update_flags(self.register_a);
    }

    // Rotate bits to the left
    fn rol(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        let old_carry = self.flags.carry() as u8;

        self.flags.set_carry(data >> 7 == 1);
        data = data << 1;
        data = data | old_carry;

        self.mem_write(addr, data);
        self.update_flags(data);
    }

    // Rotate A Register bits to the Right
    fn ror_a(&mut self) {
        let mut data = self.register_a;
        let old_carry = self.flags.carry();

        self.flags.set_carry(data & 1 == 1);
        data = data >> 1;
        if old_carry {
            data = data | 0b1000_0000;
        }

        self.register_a = data;
        self.update_flags(self.register_a);
    }

    // Rotate bits to the right
    fn ror(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        let old_carry = self.flags.carry();

        self.flags.set_carry(data & 1 == 1);
        data = data >> 1;
        if old_carry {
            data = data | 0b1000_0000;
        }

        self.mem_write(addr, data);
        self.update_flags(data);
    }

    // Return from an Interrupt processing routine to the address stored on the stack
    fn rti(&mut self) {
        self.flags.bits = self.stack_pop();
        self.flags.set_bflag(false);
        self.flags.set_b2flag(true);

        self.register_pc = self.stack_pop_16()
    }

    // Return from a subroutine to the pointer stored on the stack
    fn rts(&mut self) {
        self.register_pc = self.stack_pop_16() + 1;
    }

    // Add value to register A with the carry bit
    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let data = self.mem_read(addr);
        
        self.add_to_reg_a(((data as i8).wrapping_neg().wrapping_sub(1)) as u8);
    }

    // Set Carry Flag to True
    fn sec(&mut self) {
        self.flags.set_carry(true);
    }

    // Set Decimal Mode to True
    fn sed(&mut self) {
        self.flags.set_decimal(true);
    }

    // Set Interrupt Disable to True
    fn sei(&mut self) {
        self.flags.set_int(true);
    }

    // Copy value from A to memory
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    // Store X Register at address
    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    // Store Y Register at address
    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    // Transfer the contents of the A register to the X register
    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_flags(self.register_x);
    }

    // Transfer the contents of the A register to the Y register
    fn tay(&mut self) {
        self.register_y = self.register_a;
        self.update_flags(self.register_y);
    }

    // Transfer the contents of the Stack Pointer to the X register
    fn tsx(&mut self) {
        self.register_x = self.register_sp;
        self.update_flags(self.register_x);
    }

    // Transfer the contents of the X register to the A register
    fn txa(&mut self) {
        self.register_a = self.register_x;
        self.update_flags(self.register_a);
    }

    // Transfer the contents of the X register to the Stack Pointer
    fn txs(&mut self) {
        self.register_sp = self.register_x;
    }

    // Transfer the contents of the Y register to the A register
    fn tya(&mut self) {
        self.register_a = self.register_y;
        self.update_flags(self.register_a);
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
    fn b2flag(&self) -> bool    { self.get_bit(5) }
    fn overflow(&self) -> bool  { self.get_bit(6) }
    fn negative(&self) -> bool  { self.get_bit(7) }

    fn set_carry(&mut self, value: bool)        { self.set_bit(0, value); }
    fn set_zero(&mut self, value: bool)         { self.set_bit(1, value); }
    fn set_int(&mut self, value: bool)          { self.set_bit(2, value); }
    fn set_decimal(&mut self, value: bool)      { self.set_bit(3, value); }
    fn set_bflag(&mut self, value: bool)        { self.set_bit(4, value); }
    fn set_b2flag(&mut self, value: bool)       { self.set_bit(5, value); }
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
        cpu.load_and_run(vec![0xa9, 0x0A, 0xaa, 0x00]);

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

   #[test]
    fn test_adc() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0x38, 0xa9, 0x05, 0x69, 0x05, 0x00]);

        assert_eq!(cpu.register_a, 0x0B);
        assert_eq!(cpu.flags.carry(), false);
    }
}