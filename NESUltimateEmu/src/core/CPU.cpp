#include "CPU.hpp"
#include "Bus.hpp"

CPU::CPU(Bus& bus) : m_bus(bus)
{
    buildTable();
}

void CPU::reset()
{
    uint8_t lo = read(0xFFFC);
    uint8_t hi = read(0xFFFD);

    uint16_t resetVector = 0xC000;
    if (lo != 0 || hi != 0) {
        resetVector = static_cast<uint16_t>(lo) | ((static_cast<uint16_t>(hi) << 8));
    }

    m_pc = resetVector;
    m_sp = 0xFD;
    m_a = m_x = m_y = 0;
    m_status = 0x24;
    m_cycles = 0;
}

void CPU::nmi()
{
    // Push PC and status (B flag clear, bit 5 set), set I flag, jump to NMI vector
    push16(m_pc);
    push((m_status & ~0x10) | 0x20);  // clear B, set unused bit
    setFlag(0x04, true);              // set I
    uint8_t lo = read(0xFFFA);
    uint8_t hi = read(0xFFFB);
    m_pc = (uint16_t)lo | ((uint16_t)hi << 8);
    m_cycles = 8;                     // NMI takes 7-8 cycles
}

void CPU::irq()
{
    if (getFlag(0x04))  // I flag set → ignore
        return;
    push16(m_pc);
    push((m_status & ~0x10) | 0x20);
    setFlag(0x04, true);
    uint8_t lo = read(0xFFFE);
    uint8_t hi = read(0xFFFF);
    m_pc = (uint16_t)lo | ((uint16_t)hi << 8);
    m_cycles = 7;
}

void CPU::clock()
{
    if (m_cycles == 0) {
        uint8_t opcode = fetch();
        execute(opcode);
    }

    m_cycles--;
}

uint8_t CPU::fetch()
{
    uint8_t data = read(m_pc);
    m_pc++;
    return data;
}

void CPU::execute(uint8_t opcode)
{
    Instruction& inst = m_table[opcode];

    if (inst.operate) {
        (this->*inst.operate)();
        m_cycles = inst.cycles;
    }
    else {
        m_cycles = 2;
    }
}

uint16_t CPU::addrImmediate() { return m_pc++; }

uint16_t CPU::addrAbsolute()
{
    uint8_t lo = read(m_pc++);
    uint8_t hi = read(m_pc++);
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

uint16_t CPU::addrIndirect()
{
    uint8_t loPtr = read(m_pc++);
    uint8_t hiPtr = read(m_pc++);
    uint16_t ptr = (uint16_t)loPtr | ((uint16_t)hiPtr << 8);

    uint8_t lo = read(ptr);
    uint8_t hi = read(ptr + 1);

    return (uint16_t)lo | ((uint16_t)hi << 8);
}

uint16_t CPU::addrZeroPage()
{
    return read(m_pc++);
}

uint16_t CPU::addrZeroPageX()
{
    uint8_t zp = read(m_pc++);
    return (uint16_t)((zp + m_x) & 0xFF);
}

uint16_t CPU::addrZeroPageY()
{
    uint8_t zp = read(m_pc++);
    return (uint16_t)((zp + m_y) & 0xFF);
}

uint16_t CPU::addrAbsoluteX()
{
    uint16_t base = addrAbsolute();
    return base + m_x;
}

uint16_t CPU::addrAbsoluteY()
{
    uint16_t base = addrAbsolute();
    return base + m_y;
}

uint16_t CPU::addrIndirectX()
{
    uint8_t zp = read(m_pc++);
    uint8_t ptrLo = read((uint8_t)(zp + m_x));
    uint8_t ptrHi = read((uint8_t)(zp + m_x + 1));
    return (uint16_t)ptrLo | ((uint16_t)ptrHi << 8);
}

uint16_t CPU::addrIndirectY()
{
    uint8_t zp = read(m_pc++);
    uint8_t ptrLo = read(zp);
    uint8_t ptrHi = read((uint8_t)(zp + 1));
    uint16_t base = (uint16_t)ptrLo | ((uint16_t)ptrHi << 8);
    return base + m_y;
}

// Read variants with page-cross penalty (for loads/ALU/logic/compare)

uint16_t CPU::addrAbsoluteXRead()
{
    uint16_t base = addrAbsolute();
    uint16_t addr = base + m_x;
    if ((base & 0xFF00) != (addr & 0xFF00))
        m_cycles++;
    return addr;
}

uint16_t CPU::addrAbsoluteYRead()
{
    uint16_t base = addrAbsolute();
    uint16_t addr = base + m_y;
    if ((base & 0xFF00) != (addr & 0xFF00))
        m_cycles++;
    return addr;
}

uint16_t CPU::addrIndirectYRead()
{
    uint8_t zp = read(m_pc++);
    uint8_t ptrLo = read(zp);
    uint8_t ptrHi = read((uint8_t)(zp + 1));
    uint16_t base = (uint16_t)ptrLo | ((uint16_t)ptrHi << 8);
    uint16_t addr = base + m_y;
    if ((base & 0xFF00) != (addr & 0xFF00))
        m_cycles++;
    return addr;
}

void CPU::setZeroNeg(uint8_t value)
{
    setFlag(0x02, value == 0);
    setFlag(0x80, value & 0x80);
}

void CPU::setFlag(uint8_t flag, bool value)
{
    if (value)
        m_status |= flag;
    else
        m_status &= ~flag;
}

bool CPU::getFlag(uint8_t flag) const
{
    return (m_status & flag) != 0;
}

void CPU::branchIf(bool condition)
{
    int8_t offset = (int8_t)read(m_pc++);
    if (condition) {
        uint16_t oldPC = m_pc;
        m_pc += offset;
        m_cycles++; // branch taken
        if ((oldPC & 0xFF00) != (m_pc & 0xFF00))
            m_cycles++; // page cross
    }
}

uint8_t CPU::read(uint16_t addr) const { return m_bus.read(addr); }
void CPU::write(uint16_t addr, uint8_t data) { m_bus.write(addr, data); }

// ---------------------------------------------------------
// STACK
// ---------------------------------------------------------

void CPU::push(uint8_t value)
{
    write(0x0100 + m_sp, value);
    m_sp--;
}

uint8_t CPU::pull()
{
    m_sp++;
    return read(0x0100 + m_sp);
}

void CPU::push16(uint16_t value)
{
    push((value >> 8) & 0xFF);
    push(value & 0xFF);
}

uint16_t CPU::pull16()
{
    uint8_t lo = pull();
    uint8_t hi = pull();
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

// ---------------------------------------------------------
// INSTRUCTIONS
// ---------------------------------------------------------

// Loads (existing)
void CPU::opLDA_imm()
{
    m_a = read(addrImmediate());
    setZeroNeg(m_a);
}

void CPU::opLDX_imm()
{
    m_x = read(addrImmediate());
    setZeroNeg(m_x);
}

void CPU::opLDY_imm()
{
    m_y = read(addrImmediate());
    setZeroNeg(m_y);
}

// Loads (zp/abs you already had)
void CPU::opLDA_zp()
{
    m_a = read(addrZeroPage());
    setZeroNeg(m_a);
}

void CPU::opLDA_abs()
{
    m_a = read(addrAbsolute());
    setZeroNeg(m_a);
}

void CPU::opLDX_zp()
{
    m_x = read(addrZeroPage());
    setZeroNeg(m_x);
}

void CPU::opLDX_abs()
{
    m_x = read(addrAbsolute());
    setZeroNeg(m_x);
}

void CPU::opLDY_zp()
{
    m_y = read(addrZeroPage());
    setZeroNeg(m_y);
}

void CPU::opLDY_abs()
{
    m_y = read(addrAbsolute());
    setZeroNeg(m_y);
}

// NEW Loads: indexed & indirect

void CPU::opLDA_zpx()
{
    m_a = read(addrZeroPageX());
    setZeroNeg(m_a);
}

void CPU::opLDA_absx()
{
    m_a = read(addrAbsoluteXRead());
    setZeroNeg(m_a);
}

void CPU::opLDA_absy()
{
    m_a = read(addrAbsoluteYRead());
    setZeroNeg(m_a);
}

void CPU::opLDA_indx()
{
    m_a = read(addrIndirectX());
    setZeroNeg(m_a);
}

void CPU::opLDA_indy()
{
    m_a = read(addrIndirectYRead());
    setZeroNeg(m_a);
}

void CPU::opLDX_zpy()
{
    m_x = read(addrZeroPageY());
    setZeroNeg(m_x);
}

void CPU::opLDX_absy()
{
    m_x = read(addrAbsoluteYRead());
    setZeroNeg(m_x);
}

void CPU::opLDY_zpx()
{
    m_y = read(addrZeroPageX());
    setZeroNeg(m_y);
}

void CPU::opLDY_absx()
{
    m_y = read(addrAbsoluteXRead());
    setZeroNeg(m_y);
}

// INC/DEC registers
void CPU::opINX() { m_x++; setZeroNeg(m_x); }
void CPU::opDEX() { m_x--; setZeroNeg(m_x); }
void CPU::opINY() { m_y++; setZeroNeg(m_y); }
void CPU::opDEY() { m_y--; setZeroNeg(m_y); }

// Flags
void CPU::opSEI() { setFlag(0x04, true); }
void CPU::opCLI() { setFlag(0x04, false); }

void CPU::opCLC() { setFlag(0x01, false); }
void CPU::opSEC() { setFlag(0x01, true); }
void CPU::opCLD() { setFlag(0x08, false); }
void CPU::opSED() { setFlag(0x08, true); }
void CPU::opCLV() { setFlag(0x40, false); }

// Jumps / interrupts
void CPU::opJMP_abs() { m_pc = addrAbsolute(); }
void CPU::opJMP_ind() { m_pc = addrIndirect(); }

void CPU::opBRK()
{
    m_pc++; // BRK is treated as a 2-byte instruction
    push16(m_pc);
    push(m_status | 0x10); // B flag set on stack
    setFlag(0x04, true);   // Set I
    uint8_t lo = read(0xFFFE);
    uint8_t hi = read(0xFFFF);
    m_pc = (uint16_t)lo | ((uint16_t)hi << 8);
}

void CPU::opRTI()
{
    m_status = pull() & ~0x10;
    m_pc = pull16();
}

// Stores
void CPU::opSTA_zp() { write(addrZeroPage(), m_a); }
void CPU::opSTX_zp() { write(addrZeroPage(), m_x); }
void CPU::opSTY_zp() { write(addrZeroPage(), m_y); }

void CPU::opSTA_zpx() { write(addrZeroPageX(), m_a); }
void CPU::opSTX_zpy() { write(addrZeroPageY(), m_x); }
void CPU::opSTY_zpx() { write(addrZeroPageX(), m_y); }

void CPU::opSTA_abs() { write(addrAbsolute(), m_a); }
void CPU::opSTA_absx() { write(addrAbsoluteX(), m_a); }
void CPU::opSTA_absy() { write(addrAbsoluteY(), m_a); }

void CPU::opSTA_indx()
{
    uint16_t addr = addrIndirectX();
    write(addr, m_a);
}

void CPU::opSTA_indy()
{
    uint16_t addr = addrIndirectY();
    write(addr, m_a);
}

void CPU::opSTX_abs()
{
    uint16_t addr = addrAbsolute();
    write(addr, m_x);
}

void CPU::opSTY_abs()
{
    uint16_t addr = addrAbsolute();
    write(addr, m_y);
}

// Logic (existing imm)
void CPU::opAND_imm()
{
    m_a &= read(addrImmediate());
    setZeroNeg(m_a);
}

void CPU::opORA_imm()
{
    m_a |= read(addrImmediate());
    setZeroNeg(m_a);
}

void CPU::opEOR_imm()
{
    m_a ^= read(addrImmediate());
    setZeroNeg(m_a);
}

// Logic (zp/abs)
void CPU::opAND_zp()
{
    m_a &= read(addrZeroPage());
    setZeroNeg(m_a);
}

void CPU::opAND_abs()
{
    m_a &= read(addrAbsolute());
    setZeroNeg(m_a);
}

void CPU::opORA_zp()
{
    m_a |= read(addrZeroPage());
    setZeroNeg(m_a);
}

void CPU::opORA_abs()
{
    m_a |= read(addrAbsolute());
    setZeroNeg(m_a);
}

void CPU::opEOR_zp()
{
    m_a ^= read(addrZeroPage());
    setZeroNeg(m_a);
}

void CPU::opEOR_abs()
{
    m_a ^= read(addrAbsolute());
    setZeroNeg(m_a);
}

// NEW Logic (indexed / indirect)

void CPU::opAND_zpx()
{
    m_a &= read(addrZeroPageX());
    setZeroNeg(m_a);
}

void CPU::opAND_absx()
{
    m_a &= read(addrAbsoluteXRead());
    setZeroNeg(m_a);
}

void CPU::opAND_absy()
{
    m_a &= read(addrAbsoluteYRead());
    setZeroNeg(m_a);
}

void CPU::opAND_indx()
{
    m_a &= read(addrIndirectX());
    setZeroNeg(m_a);
}

void CPU::opAND_indy()
{
    m_a &= read(addrIndirectYRead());
    setZeroNeg(m_a);
}

void CPU::opORA_zpx()
{
    m_a |= read(addrZeroPageX());
    setZeroNeg(m_a);
}

void CPU::opORA_absx()
{
    m_a |= read(addrAbsoluteXRead());
    setZeroNeg(m_a);
}

void CPU::opORA_absy()
{
    m_a |= read(addrAbsoluteYRead());
    setZeroNeg(m_a);
}

void CPU::opORA_indx()
{
    m_a |= read(addrIndirectX());
    setZeroNeg(m_a);
}

void CPU::opORA_indy()
{
    m_a |= read(addrIndirectYRead());
    setZeroNeg(m_a);
}

void CPU::opEOR_zpx()
{
    m_a ^= read(addrZeroPageX());
    setZeroNeg(m_a);
}

void CPU::opEOR_absx()
{
    m_a ^= read(addrAbsoluteXRead());
    setZeroNeg(m_a);
}

void CPU::opEOR_absy()
{
    m_a ^= read(addrAbsoluteYRead());
    setZeroNeg(m_a);
}

void CPU::opEOR_indx()
{
    m_a ^= read(addrIndirectX());
    setZeroNeg(m_a);
}

void CPU::opEOR_indy()
{
    m_a ^= read(addrIndirectYRead());
    setZeroNeg(m_a);
}

// BIT

void CPU::opBIT_zp()
{
    uint8_t value = read(addrZeroPage());
    uint8_t res = m_a & value;
    setFlag(0x02, res == 0);       // Z
    setFlag(0x40, value & 0x40);   // V
    setFlag(0x80, value & 0x80);   // N
}

void CPU::opBIT_abs()
{
    uint8_t value = read(addrAbsolute());
    uint8_t res = m_a & value;
    setFlag(0x02, res == 0);
    setFlag(0x40, value & 0x40);
    setFlag(0x80, value & 0x80);
}

// Shifts / rotates

void CPU::opASL_acc()
{
    uint8_t v = m_a;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    m_a = v;
    setZeroNeg(m_a);
}

void CPU::opASL_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opASL_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opASL_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opASL_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opLSR_acc()
{
    uint8_t v = m_a;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    m_a = v;
    setZeroNeg(m_a);
}

void CPU::opLSR_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opLSR_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opLSR_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opLSR_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROL_acc()
{
    uint8_t v = m_a;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    m_a = v;
    setZeroNeg(m_a);
}

void CPU::opROL_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROL_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROL_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROL_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROR_acc()
{
    uint8_t v = m_a;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    m_a = v;
    setZeroNeg(m_a);
}

void CPU::opROR_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROR_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROR_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

void CPU::opROR_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);
    setZeroNeg(v);
}

// Branches

void CPU::opBEQ() { branchIf(getFlag(0x02)); }
void CPU::opBNE() { branchIf(!getFlag(0x02)); }
void CPU::opBMI() { branchIf(getFlag(0x80)); }
void CPU::opBPL() { branchIf(!getFlag(0x80)); }
void CPU::opBCC() { branchIf(!getFlag(0x01)); }
void CPU::opBCS() { branchIf(getFlag(0x01)); }
void CPU::opBVC() { branchIf(!getFlag(0x40)); }
void CPU::opBVS() { branchIf(getFlag(0x40)); }

// Stack ops
void CPU::opPHA() { push(m_a); }
void CPU::opPHP() { push(m_status | 0x10); }
void CPU::opPLA() { m_a = pull(); setZeroNeg(m_a); }
void CPU::opPLP() { m_status = pull() & ~0x10; }

// Subroutines
void CPU::opJSR()
{
    uint16_t addr = addrAbsolute();
    push16(m_pc - 1);
    m_pc = addr;
}

void CPU::opRTS()
{
    uint16_t ret = pull16();
    m_pc = ret + 1;
}

// Transfers

void CPU::opTAX()
{
    m_x = m_a;
    setZeroNeg(m_x);
}

void CPU::opTXA()
{
    m_a = m_x;
    setZeroNeg(m_a);
}

void CPU::opTAY()
{
    m_y = m_a;
    setZeroNeg(m_y);
}

void CPU::opTYA()
{
    m_a = m_y;
    setZeroNeg(m_a);
}

void CPU::opTSX()
{
    m_x = m_sp;
    setZeroNeg(m_x);
}

void CPU::opTXS()
{
    m_sp = m_x;
}

// ---------------------------------------------------------
// ARITHMETIC
// ---------------------------------------------------------

void CPU::opADC_imm()
{
    uint8_t value = read(addrImmediate());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF); // Carry
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80)); // Overflow
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_zp()
{
    uint8_t value = read(addrZeroPage());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_zpx()
{
    uint8_t value = read(addrZeroPageX());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_abs()
{
    uint8_t value = read(addrAbsolute());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_absx()
{
    uint8_t value = read(addrAbsoluteXRead());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_absy()
{
    uint8_t value = read(addrAbsoluteYRead());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_indx()
{
    uint8_t value = read(addrIndirectX());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opADC_indy()
{
    uint8_t value = read(addrIndirectYRead());
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_imm()
{
    uint8_t value = read(addrImmediate()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_zp()
{
    uint8_t value = read(addrZeroPage()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_zpx()
{
    uint8_t value = read(addrZeroPageX()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_abs()
{
    uint8_t value = read(addrAbsolute()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_absx()
{
    uint8_t value = read(addrAbsoluteXRead()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_absy()
{
    uint8_t value = read(addrAbsoluteYRead()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_indx()
{
    uint8_t value = read(addrIndirectX()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opSBC_indy()
{
    uint8_t value = read(addrIndirectYRead()) ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);

    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);

    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80));
    m_a = result;
    setZeroNeg(m_a);
}

// ---------------------------------------------------------
// COMPARE
// ---------------------------------------------------------

void CPU::cmpHelper(uint8_t reg, uint8_t value)
{
    uint16_t diff = (uint16_t)reg - (uint16_t)value;
    setFlag(0x01, reg >= value); // Carry if reg >= value
    setZeroNeg((uint8_t)(diff & 0xFF));
}

void CPU::opCMP_imm()
{
    uint8_t value = read(addrImmediate());
    cmpHelper(m_a, value);
}

void CPU::opCMP_zp()
{
    uint8_t value = read(addrZeroPage());
    cmpHelper(m_a, value);
}

void CPU::opCMP_zpx()
{
    uint8_t value = read(addrZeroPageX());
    cmpHelper(m_a, value);
}

void CPU::opCMP_abs()
{
    uint8_t value = read(addrAbsolute());
    cmpHelper(m_a, value);
}

void CPU::opCMP_absx()
{
    uint8_t value = read(addrAbsoluteXRead());
    cmpHelper(m_a, value);
}

void CPU::opCMP_absy()
{
    uint8_t value = read(addrAbsoluteYRead());
    cmpHelper(m_a, value);
}

void CPU::opCMP_indx()
{
    uint8_t value = read(addrIndirectX());
    cmpHelper(m_a, value);
}

void CPU::opCMP_indy()
{
    uint8_t value = read(addrIndirectYRead());
    cmpHelper(m_a, value);
}

void CPU::opCPX_imm()
{
    uint8_t value = read(addrImmediate());
    cmpHelper(m_x, value);
}

void CPU::opCPX_zp()
{
    uint8_t value = read(addrZeroPage());
    cmpHelper(m_x, value);
}

void CPU::opCPX_abs()
{
    uint8_t value = read(addrAbsolute());
    cmpHelper(m_x, value);
}

void CPU::opCPY_imm()
{
    uint8_t value = read(addrImmediate());
    cmpHelper(m_y, value);
}

void CPU::opCPY_zp()
{
    uint8_t value = read(addrZeroPage());
    cmpHelper(m_y, value);
}

void CPU::opCPY_abs()
{
    uint8_t value = read(addrAbsolute());
    cmpHelper(m_y, value);
}

// ---------------------------------------------------------
// MEMORY INC/DEC
// ---------------------------------------------------------

void CPU::opINC_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t value = read(addr);
    value++;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opINC_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t value = read(addr);
    value++;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opINC_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t value = read(addr);
    value++;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opINC_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t value = read(addr);
    value++;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opDEC_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t value = read(addr);
    value--;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opDEC_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t value = read(addr);
    value--;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opDEC_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t value = read(addr);
    value--;
    write(addr, value);
    setZeroNeg(value);
}

void CPU::opDEC_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t value = read(addr);
    value--;
    write(addr, value);
    setZeroNeg(value);
}

// ---------------------------------------------------------
// MISC
// ---------------------------------------------------------

void CPU::opNOP()
{
    // Do nothing
}

// ---------------------------------------------------------
// UNOFFICIAL / ILLEGAL OPCODES
// ---------------------------------------------------------

// JAM / KIL – freezes the CPU (never returns from the instruction)
void CPU::opJAM()
{
    // Keep the PC on the JAM opcode and burn cycles forever.
    // Setting a huge cycle count makes the emulator hang here.
    m_pc--;          // undo the fetch advance so we stay on the JAM
    m_cycles = 0x7FFFFFFF;
}

// Unofficial NOPs that consume an operand
void CPU::opNOP_imm()
{
    (void)addrImmediate();
}

void CPU::opNOP_zp()
{
    (void)addrZeroPage();
}

void CPU::opNOP_zpx()
{
    (void)addrZeroPageX();
}

void CPU::opNOP_abs()
{
    (void)addrAbsolute();
}

void CPU::opNOP_absx()
{
    (void)addrAbsoluteXRead(); // still pays the page-cross penalty
}

// ---------- SLO = ASL mem then ORA A ----------
void CPU::opSLO_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_indy()
{
    uint16_t addr = addrIndirectY(); // no page-cross penalty on RMW
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x80);
    v <<= 1;
    write(addr, v);
    m_a |= v;
    setZeroNeg(m_a);
}

// ---------- RLA = ROL mem then AND A ----------
void CPU::opRLA_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    write(addr, v);
    m_a &= v;
    setZeroNeg(m_a);
}

// ---------- SRE = LSR mem then EOR A ----------
void CPU::opSRE_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    setFlag(0x01, v & 0x01);
    v >>= 1;
    write(addr, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

// ---------- RRA = ROR mem then ADC A ----------
void CPU::opRRA_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    // now ADC
    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opRRA_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opRRA_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opRRA_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opRRA_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opRRA_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opRRA_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    write(addr, v);

    uint16_t sum = (uint16_t)m_a + v + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, (~(m_a ^ v) & (m_a ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

// ---------- SAX = store (A & X) ----------
void CPU::opSAX_indx()
{
    uint16_t addr = addrIndirectX();
    write(addr, m_a & m_x);
}

void CPU::opSAX_zp()
{
    uint16_t addr = addrZeroPage();
    write(addr, m_a & m_x);
}

void CPU::opSAX_abs()
{
    uint16_t addr = addrAbsolute();
    write(addr, m_a & m_x);
}

void CPU::opSAX_zpy()
{
    uint16_t addr = addrZeroPageY();
    write(addr, m_a & m_x);
}

// ---------- LAX = LDA + LDX ----------
void CPU::opLAX_indx()
{
    uint8_t v = read(addrIndirectX());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_zp()
{
    uint8_t v = read(addrZeroPage());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_abs()
{
    uint8_t v = read(addrAbsolute());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_indy()
{
    uint8_t v = read(addrIndirectYRead());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_zpy()
{
    uint8_t v = read(addrZeroPageY());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_absy()
{
    uint8_t v = read(addrAbsoluteYRead());
    m_a = m_x = v;
    setZeroNeg(v);
}

// ---------- DCP = DEC mem then CMP A ----------
void CPU::opDCP_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    v--;
    write(addr, v);
    cmpHelper(m_a, v);
}

// ---------- ISC / ISB = INC mem then SBC A ----------
void CPU::opISC_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    // SBC
    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opISC_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opISC_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opISC_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opISC_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opISC_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

void CPU::opISC_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    v++;
    write(addr, v);

    uint8_t value = v ^ 0xFF;
    uint16_t sum = (uint16_t)m_a + value + (getFlag(0x01) ? 1 : 0);
    setFlag(0x01, sum > 0xFF);
    uint8_t result = (uint8_t)(sum & 0xFF);
    setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
    m_a = result;
    setZeroNeg(m_a);
}

// ---------- Immediate unofficial ----------
void CPU::opANC_imm()
{
    m_a &= read(addrImmediate());
    setZeroNeg(m_a);
    setFlag(0x01, m_a & 0x80); // C = N
}

void CPU::opALR_imm()
{
    m_a &= read(addrImmediate());
    setFlag(0x01, m_a & 0x01);
    m_a >>= 1;
    setZeroNeg(m_a);
}

void CPU::opARR_imm()
{
    m_a &= read(addrImmediate());
    // ROR
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, m_a & 0x01); // temporary, will be overwritten
    m_a = (uint8_t)((m_a >> 1) | carryIn);

    // Special ARR flags: C = bit 6, V = bit 6 xor bit 5
    setFlag(0x01, m_a & 0x40);
    setFlag(0x40, ((m_a >> 6) ^ (m_a >> 5)) & 1);
    setZeroNeg(m_a);
}

void CPU::opAXS_imm()
{
    uint8_t imm = read(addrImmediate());
    uint8_t tmp = m_a & m_x;
    uint16_t diff = (uint16_t)tmp - imm;
    setFlag(0x01, tmp >= imm);
    m_x = (uint8_t)(diff & 0xFF);
    setZeroNeg(m_x);
}

void CPU::opSBC_imm_unofficial()
{
    // Identical to official SBC immediate
    opSBC_imm();
}

// Unstable immediate approximations (common in accurate emulators)
void CPU::opLXA_imm()
{
    // Most common approximation: A = X = (A | 0xEE) & imm  (or simply A = X = imm)
    // We use the widely-used “A = X = imm” form that many test ROMs expect.
    uint8_t v = read(addrImmediate());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opANE_imm()
{
    // Highly unstable. Common approximation used by Mesen / Nestopia:
    // A = (A | CONST) & X & imm   (CONST often 0xEE or 0xFF)
    uint8_t imm = read(addrImmediate());
    m_a = (m_a | 0xEE) & m_x & imm;
    setZeroNeg(m_a);
}

// ---------- Highly unstable store / transfer opcodes ----------
// These use the classic “AND with (H+1)” approximation.

void CPU::opSHA_indy()
{
    uint8_t zp = read(m_pc++);
    uint8_t ptrLo = read(zp);
    uint8_t ptrHi = read((uint8_t)(zp + 1));
    uint16_t base = (uint16_t)ptrLo | ((uint16_t)ptrHi << 8);
    uint16_t addr = base + m_y;
    uint8_t h = (uint8_t)((base >> 8) + 1);
    write(addr, m_a & m_x & h);
}

void CPU::opSHA_absy()
{
    uint16_t base = addrAbsolute();
    uint16_t addr = base + m_y;
    uint8_t h = (uint8_t)((base >> 8) + 1);
    write(addr, m_a & m_x & h);
}

void CPU::opSHX_absy()
{
    uint16_t base = addrAbsolute();
    uint16_t addr = base + m_y;
    uint8_t h = (uint8_t)((base >> 8) + 1);
    write(addr, m_x & h);
}

void CPU::opSHY_absx()
{
    uint16_t base = addrAbsolute();
    uint16_t addr = base + m_x;
    uint8_t h = (uint8_t)((base >> 8) + 1);
    write(addr, m_y & h);
}

void CPU::opTAS_absy()
{
    uint16_t base = addrAbsolute();
    uint16_t addr = base + m_y;
    m_sp = m_a & m_x;
    uint8_t h = (uint8_t)((base >> 8) + 1);
    write(addr, m_sp & h);
}

void CPU::opLAS_absy()
{
    uint8_t v = read(addrAbsoluteYRead());
    v &= m_sp;
    m_a = m_x = m_sp = v;
    setZeroNeg(v);
}

// ---------------------------------------------------------
// OPCODE TABLE
// ---------------------------------------------------------

void CPU::buildTable()
{
    m_table.fill({ nullptr, 2 });

    // Loads (imm)
    m_table[0xA9] = { &CPU::opLDA_imm, 2 };
    m_table[0xA2] = { &CPU::opLDX_imm, 2 };
    m_table[0xA0] = { &CPU::opLDY_imm, 2 };

    // Loads (zp/abs)
    m_table[0xA5] = { &CPU::opLDA_zp, 3 };
    m_table[0xAD] = { &CPU::opLDA_abs, 4 };
    m_table[0xA6] = { &CPU::opLDX_zp, 3 };
    m_table[0xAE] = { &CPU::opLDX_abs, 4 };
    m_table[0xA4] = { &CPU::opLDY_zp, 3 };
    m_table[0xAC] = { &CPU::opLDY_abs, 4 };

    // NEW Loads (indexed / indirect)
    m_table[0xB5] = { &CPU::opLDA_zpx, 4 };
    m_table[0xBD] = { &CPU::opLDA_absx, 4 };
    m_table[0xB9] = { &CPU::opLDA_absy, 4 };
    m_table[0xA1] = { &CPU::opLDA_indx, 6 };
    m_table[0xB1] = { &CPU::opLDA_indy, 5 };

    m_table[0xB6] = { &CPU::opLDX_zpy, 4 };
    m_table[0xBE] = { &CPU::opLDX_absy, 4 };

    m_table[0xB4] = { &CPU::opLDY_zpx, 4 };
    m_table[0xBC] = { &CPU::opLDY_absx, 4 };

    // Stores
    m_table[0x85] = { &CPU::opSTA_zp, 3 };
    m_table[0x86] = { &CPU::opSTX_zp, 3 };
    m_table[0x84] = { &CPU::opSTY_zp, 3 };

    m_table[0x95] = { &CPU::opSTA_zpx, 4 };
    m_table[0x96] = { &CPU::opSTX_zpy, 4 };
    m_table[0x94] = { &CPU::opSTY_zpx, 4 };

    m_table[0x8D] = { &CPU::opSTA_abs, 4 };
    m_table[0x9D] = { &CPU::opSTA_absx, 5 };
    m_table[0x99] = { &CPU::opSTA_absy, 5 };

    m_table[0x8E] = { &CPU::opSTX_abs, 4 };
    m_table[0x8C] = { &CPU::opSTY_abs, 4 };

    m_table[0x81] = { &CPU::opSTA_indx, 6 };
    m_table[0x91] = { &CPU::opSTA_indy, 6 };

    // Logic (imm)
    m_table[0x29] = { &CPU::opAND_imm, 2 };
    m_table[0x09] = { &CPU::opORA_imm, 2 };
    m_table[0x49] = { &CPU::opEOR_imm, 2 };

    // Logic (zp/abs)
    m_table[0x25] = { &CPU::opAND_zp, 3 };
    m_table[0x2D] = { &CPU::opAND_abs, 4 };
    m_table[0x05] = { &CPU::opORA_zp, 3 };
    m_table[0x0D] = { &CPU::opORA_abs, 4 };
    m_table[0x45] = { &CPU::opEOR_zp, 3 };
    m_table[0x4D] = { &CPU::opEOR_abs, 4 };

    // NEW Logic (indexed / indirect)
    m_table[0x35] = { &CPU::opAND_zpx, 4 };
    m_table[0x3D] = { &CPU::opAND_absx, 4 };
    m_table[0x39] = { &CPU::opAND_absy, 4 };
    m_table[0x21] = { &CPU::opAND_indx, 6 };
    m_table[0x31] = { &CPU::opAND_indy, 5 };

    m_table[0x15] = { &CPU::opORA_zpx, 4 };
    m_table[0x1D] = { &CPU::opORA_absx, 4 };
    m_table[0x19] = { &CPU::opORA_absy, 4 };
    m_table[0x01] = { &CPU::opORA_indx, 6 };
    m_table[0x11] = { &CPU::opORA_indy, 5 };

    m_table[0x55] = { &CPU::opEOR_zpx, 4 };
    m_table[0x5D] = { &CPU::opEOR_absx, 4 };
    m_table[0x59] = { &CPU::opEOR_absy, 4 };
    m_table[0x41] = { &CPU::opEOR_indx, 6 };
    m_table[0x51] = { &CPU::opEOR_indy, 5 };

    // BIT
    m_table[0x24] = { &CPU::opBIT_zp, 3 };
    m_table[0x2C] = { &CPU::opBIT_abs, 4 };

    // Shifts / rotates
    m_table[0x0A] = { &CPU::opASL_acc, 2 };
    m_table[0x06] = { &CPU::opASL_zp, 5 };
    m_table[0x16] = { &CPU::opASL_zpx, 6 };
    m_table[0x0E] = { &CPU::opASL_abs, 6 };
    m_table[0x1E] = { &CPU::opASL_absx, 7 };

    m_table[0x4A] = { &CPU::opLSR_acc, 2 };
    m_table[0x46] = { &CPU::opLSR_zp, 5 };
    m_table[0x56] = { &CPU::opLSR_zpx, 6 };
    m_table[0x4E] = { &CPU::opLSR_abs, 6 };
    m_table[0x5E] = { &CPU::opLSR_absx, 7 };

    m_table[0x2A] = { &CPU::opROL_acc, 2 };
    m_table[0x26] = { &CPU::opROL_zp, 5 };
    m_table[0x36] = { &CPU::opROL_zpx, 6 };
    m_table[0x2E] = { &CPU::opROL_abs, 6 };
    m_table[0x3E] = { &CPU::opROL_absx, 7 };

    m_table[0x6A] = { &CPU::opROR_acc, 2 };
    m_table[0x66] = { &CPU::opROR_zp, 5 };
    m_table[0x76] = { &CPU::opROR_zpx, 6 };
    m_table[0x6E] = { &CPU::opROR_abs, 6 };
    m_table[0x7E] = { &CPU::opROR_absx, 7 };

    // INC/DEC registers
    m_table[0xE8] = { &CPU::opINX, 2 };
    m_table[0xCA] = { &CPU::opDEX, 2 };
    m_table[0xC8] = { &CPU::opINY, 2 };
    m_table[0x88] = { &CPU::opDEY, 2 };

    // Flags
    m_table[0x78] = { &CPU::opSEI, 2 };
    m_table[0x58] = { &CPU::opCLI, 2 };
    m_table[0x18] = { &CPU::opCLC, 2 };
    m_table[0x38] = { &CPU::opSEC, 2 };
    m_table[0xD8] = { &CPU::opCLD, 2 };
    m_table[0xF8] = { &CPU::opSED, 2 };
    m_table[0xB8] = { &CPU::opCLV, 2 };

    // Jumps / interrupts
    m_table[0x4C] = { &CPU::opJMP_abs, 3 };
    m_table[0x6C] = { &CPU::opJMP_ind, 5 };
    m_table[0x00] = { &CPU::opBRK, 7 };
    m_table[0x40] = { &CPU::opRTI, 6 };

    // Branches
    m_table[0xF0] = { &CPU::opBEQ, 2 };
    m_table[0xD0] = { &CPU::opBNE, 2 };
    m_table[0x30] = { &CPU::opBMI, 2 };
    m_table[0x10] = { &CPU::opBPL, 2 };
    m_table[0x90] = { &CPU::opBCC, 2 };
    m_table[0xB0] = { &CPU::opBCS, 2 };
    m_table[0x50] = { &CPU::opBVC, 2 };
    m_table[0x70] = { &CPU::opBVS, 2 };

    // Stack ops
    m_table[0x48] = { &CPU::opPHA, 3 };
    m_table[0x08] = { &CPU::opPHP, 3 };
    m_table[0x68] = { &CPU::opPLA, 4 };
    m_table[0x28] = { &CPU::opPLP, 4 };

    // Subroutines
    m_table[0x20] = { &CPU::opJSR, 6 };
    m_table[0x60] = { &CPU::opRTS, 6 };

    // Transfers
    m_table[0xAA] = { &CPU::opTAX, 2 };
    m_table[0x8A] = { &CPU::opTXA, 2 };
    m_table[0xA8] = { &CPU::opTAY, 2 };
    m_table[0x98] = { &CPU::opTYA, 2 };
    m_table[0xBA] = { &CPU::opTSX, 2 };
    m_table[0x9A] = { &CPU::opTXS, 2 };

    // Arithmetic
    m_table[0x69] = { &CPU::opADC_imm, 2 };
    m_table[0x65] = { &CPU::opADC_zp, 3 };
    m_table[0x75] = { &CPU::opADC_zpx, 4 };
    m_table[0x6D] = { &CPU::opADC_abs, 4 };
    m_table[0x7D] = { &CPU::opADC_absx, 4 };
    m_table[0x79] = { &CPU::opADC_absy, 4 };
    m_table[0x61] = { &CPU::opADC_indx, 6 };
    m_table[0x71] = { &CPU::opADC_indy, 5 };

    m_table[0xE9] = { &CPU::opSBC_imm, 2 };
    m_table[0xE5] = { &CPU::opSBC_zp, 3 };
    m_table[0xF5] = { &CPU::opSBC_zpx, 4 };
    m_table[0xED] = { &CPU::opSBC_abs, 4 };
    m_table[0xFD] = { &CPU::opSBC_absx, 4 };
    m_table[0xF9] = { &CPU::opSBC_absy, 4 };
    m_table[0xE1] = { &CPU::opSBC_indx, 6 };
    m_table[0xF1] = { &CPU::opSBC_indy, 5 };

    // Compare
    m_table[0xC9] = { &CPU::opCMP_imm, 2 };
    m_table[0xC5] = { &CPU::opCMP_zp, 3 };
    m_table[0xD5] = { &CPU::opCMP_zpx, 4 };
    m_table[0xCD] = { &CPU::opCMP_abs, 4 };
    m_table[0xDD] = { &CPU::opCMP_absx, 4 };
    m_table[0xD9] = { &CPU::opCMP_absy, 4 };
    m_table[0xC1] = { &CPU::opCMP_indx, 6 };
    m_table[0xD1] = { &CPU::opCMP_indy, 5 };

    m_table[0xE0] = { &CPU::opCPX_imm, 2 };
    m_table[0xE4] = { &CPU::opCPX_zp, 3 };
    m_table[0xEC] = { &CPU::opCPX_abs, 4 };

    m_table[0xC0] = { &CPU::opCPY_imm, 2 };
    m_table[0xC4] = { &CPU::opCPY_zp, 3 };
    m_table[0xCC] = { &CPU::opCPY_abs, 4 };

    // Memory INC/DEC
    m_table[0xE6] = { &CPU::opINC_zp, 5 };
    m_table[0xF6] = { &CPU::opINC_zpx, 6 };
    m_table[0xEE] = { &CPU::opINC_abs, 6 };
    m_table[0xFE] = { &CPU::opINC_absx, 7 };

    m_table[0xC6] = { &CPU::opDEC_zp, 5 };
    m_table[0xD6] = { &CPU::opDEC_zpx, 6 };
    m_table[0xCE] = { &CPU::opDEC_abs, 6 };
    m_table[0xDE] = { &CPU::opDEC_absx, 7 };

    // Misc
    m_table[0xEA] = { &CPU::opNOP, 2 };

    // =========================================================
    // UNOFFICIAL / ILLEGAL OPCODES – full coverage of all 256
    // =========================================================

    // --- JAM / KIL (halt the CPU) ---
    m_table[0x02] = { &CPU::opJAM, 2 };
    m_table[0x12] = { &CPU::opJAM, 2 };
    m_table[0x22] = { &CPU::opJAM, 2 };
    m_table[0x32] = { &CPU::opJAM, 2 };
    m_table[0x42] = { &CPU::opJAM, 2 };
    m_table[0x52] = { &CPU::opJAM, 2 };
    m_table[0x62] = { &CPU::opJAM, 2 };
    m_table[0x72] = { &CPU::opJAM, 2 };
    m_table[0x92] = { &CPU::opJAM, 2 };
    m_table[0xB2] = { &CPU::opJAM, 2 };
    m_table[0xD2] = { &CPU::opJAM, 2 };
    m_table[0xF2] = { &CPU::opJAM, 2 };

    // --- Unofficial NOPs ---
    // 1-byte implied
    m_table[0x1A] = { &CPU::opNOP, 2 };
    m_table[0x3A] = { &CPU::opNOP, 2 };
    m_table[0x5A] = { &CPU::opNOP, 2 };
    m_table[0x7A] = { &CPU::opNOP, 2 };
    m_table[0xDA] = { &CPU::opNOP, 2 };
    m_table[0xFA] = { &CPU::opNOP, 2 };

    // 2-byte immediate / SKB
    m_table[0x80] = { &CPU::opNOP_imm, 2 };
    m_table[0x82] = { &CPU::opNOP_imm, 2 };
    m_table[0x89] = { &CPU::opNOP_imm, 2 };
    m_table[0xC2] = { &CPU::opNOP_imm, 2 };
    m_table[0xE2] = { &CPU::opNOP_imm, 2 };

    // 2-byte zero-page
    m_table[0x04] = { &CPU::opNOP_zp, 3 };
    m_table[0x44] = { &CPU::opNOP_zp, 3 };
    m_table[0x64] = { &CPU::opNOP_zp, 3 };

    // 2-byte zero-page,X
    m_table[0x14] = { &CPU::opNOP_zpx, 4 };
    m_table[0x34] = { &CPU::opNOP_zpx, 4 };
    m_table[0x54] = { &CPU::opNOP_zpx, 4 };
    m_table[0x74] = { &CPU::opNOP_zpx, 4 };
    m_table[0xD4] = { &CPU::opNOP_zpx, 4 };
    m_table[0xF4] = { &CPU::opNOP_zpx, 4 };

    // 3-byte absolute
    m_table[0x0C] = { &CPU::opNOP_abs, 4 };

    // 3-byte absolute,X (page-cross penalty)
    m_table[0x1C] = { &CPU::opNOP_absx, 4 };
    m_table[0x3C] = { &CPU::opNOP_absx, 4 };
    m_table[0x5C] = { &CPU::opNOP_absx, 4 };
    m_table[0x7C] = { &CPU::opNOP_absx, 4 };
    m_table[0xDC] = { &CPU::opNOP_absx, 4 };
    m_table[0xFC] = { &CPU::opNOP_absx, 4 };

    // --- SLO (ASL + ORA) ---
    m_table[0x03] = { &CPU::opSLO_indx, 8 };
    m_table[0x07] = { &CPU::opSLO_zp, 5 };
    m_table[0x0F] = { &CPU::opSLO_abs, 6 };
    m_table[0x13] = { &CPU::opSLO_indy, 8 };
    m_table[0x17] = { &CPU::opSLO_zpx, 6 };
    m_table[0x1B] = { &CPU::opSLO_absy, 7 };
    m_table[0x1F] = { &CPU::opSLO_absx, 7 };

    // --- RLA (ROL + AND) ---
    m_table[0x23] = { &CPU::opRLA_indx, 8 };
    m_table[0x27] = { &CPU::opRLA_zp, 5 };
    m_table[0x2F] = { &CPU::opRLA_abs, 6 };
    m_table[0x33] = { &CPU::opRLA_indy, 8 };
    m_table[0x37] = { &CPU::opRLA_zpx, 6 };
    m_table[0x3B] = { &CPU::opRLA_absy, 7 };
    m_table[0x3F] = { &CPU::opRLA_absx, 7 };

    // --- SRE (LSR + EOR) ---
    m_table[0x43] = { &CPU::opSRE_indx, 8 };
    m_table[0x47] = { &CPU::opSRE_zp, 5 };
    m_table[0x4F] = { &CPU::opSRE_abs, 6 };
    m_table[0x53] = { &CPU::opSRE_indy, 8 };
    m_table[0x57] = { &CPU::opSRE_zpx, 6 };
    m_table[0x5B] = { &CPU::opSRE_absy, 7 };
    m_table[0x5F] = { &CPU::opSRE_absx, 7 };

    // --- RRA (ROR + ADC) ---
    m_table[0x63] = { &CPU::opRRA_indx, 8 };
    m_table[0x67] = { &CPU::opRRA_zp, 5 };
    m_table[0x6F] = { &CPU::opRRA_abs, 6 };
    m_table[0x73] = { &CPU::opRRA_indy, 8 };
    m_table[0x77] = { &CPU::opRRA_zpx, 6 };
    m_table[0x7B] = { &CPU::opRRA_absy, 7 };
    m_table[0x7F] = { &CPU::opRRA_absx, 7 };

    // --- SAX (store A & X) ---
    m_table[0x83] = { &CPU::opSAX_indx, 6 };
    m_table[0x87] = { &CPU::opSAX_zp, 3 };
    m_table[0x8F] = { &CPU::opSAX_abs, 4 };
    m_table[0x97] = { &CPU::opSAX_zpy, 4 };

    // --- LAX (LDA + LDX) ---
    m_table[0xA3] = { &CPU::opLAX_indx, 6 };
    m_table[0xA7] = { &CPU::opLAX_zp, 3 };
    m_table[0xAF] = { &CPU::opLAX_abs, 4 };
    m_table[0xB3] = { &CPU::opLAX_indy, 5 };
    m_table[0xB7] = { &CPU::opLAX_zpy, 4 };
    m_table[0xBF] = { &CPU::opLAX_absy, 4 };

    // --- DCP (DEC + CMP) ---
    m_table[0xC3] = { &CPU::opDCP_indx, 8 };
    m_table[0xC7] = { &CPU::opDCP_zp, 5 };
    m_table[0xCF] = { &CPU::opDCP_abs, 6 };
    m_table[0xD3] = { &CPU::opDCP_indy, 8 };
    m_table[0xD7] = { &CPU::opDCP_zpx, 6 };
    m_table[0xDB] = { &CPU::opDCP_absy, 7 };
    m_table[0xDF] = { &CPU::opDCP_absx, 7 };

    // --- ISC / ISB (INC + SBC) ---
    m_table[0xE3] = { &CPU::opISC_indx, 8 };
    m_table[0xE7] = { &CPU::opISC_zp, 5 };
    m_table[0xEF] = { &CPU::opISC_abs, 6 };
    m_table[0xF3] = { &CPU::opISC_indy, 8 };
    m_table[0xF7] = { &CPU::opISC_zpx, 6 };
    m_table[0xFB] = { &CPU::opISC_absy, 7 };
    m_table[0xFF] = { &CPU::opISC_absx, 7 };

    // --- Immediate unofficial ---
    m_table[0x0B] = { &CPU::opANC_imm, 2 };
    m_table[0x2B] = { &CPU::opANC_imm, 2 };
    m_table[0x4B] = { &CPU::opALR_imm, 2 };
    m_table[0x6B] = { &CPU::opARR_imm, 2 };
    m_table[0xCB] = { &CPU::opAXS_imm, 2 };
    m_table[0xEB] = { &CPU::opSBC_imm_unofficial, 2 };

    // Unstable immediate
    m_table[0xAB] = { &CPU::opLXA_imm, 2 }; // LAX immediate (approx)
    m_table[0x8B] = { &CPU::opANE_imm, 2 }; // ANE / XAA (approx)

    // Highly unstable (common approximations used by accurate emulators)
    m_table[0x93] = { &CPU::opSHA_indy, 6 };
    m_table[0x9F] = { &CPU::opSHA_absy, 5 };
    m_table[0x9E] = { &CPU::opSHX_absy, 5 };
    m_table[0x9C] = { &CPU::opSHY_absx, 5 };
    m_table[0x9B] = { &CPU::opTAS_absy, 5 };
    m_table[0xBB] = { &CPU::opLAS_absy, 4 };
}























