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
    m_pc = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    m_sp = 0xFD;
    m_a = m_x = m_y = 0;
    m_status = 0x24;
    m_nmiPending = false;
    m_nmiPolled = false;
    m_irqLine = false;
    m_irqPolled = false;
    m_pollInterruptsThisSequence = false;
    m_irqDisableBeforeInstruction = true;
    m_currentOpcode = 0;
    m_branchPageCrossed = false;
    m_pendingIoOp = PendingIoOp::None;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_instructionCount = 0;
    m_lastInstructionPc = m_pc;
    m_lastOpcode = 0;
    m_lastOperand1 = 0;
    m_lastOperand2 = 0;
    m_lastA = m_a;
    m_lastX = m_x;
    m_lastY = m_y;
    m_lastSp = m_sp;
    m_lastStatus = m_status;
    // Hardware reset consumes 7 CPU cycles before normal execution resumes.
    m_cycles = 7;
}

void CPU::nmi()
{
    // NMI is edge-triggered. Latch it and service it at the next
    // instruction boundary rather than interrupting an instruction that
    // is already in progress.
    m_nmiPending = true;
}

void CPU::cancelPendingNmi()
{
    // Once the CPU has sampled the edge for the current instruction it is
    // committed. Only the asynchronous, not-yet-polled edge can disappear
    // in the PPU's short VBlank/NMI suppression race.
    if (!m_nmiPolled)
        m_nmiPending = false;
}

void CPU::setIrqLine(bool asserted)
{
    m_irqLine = asserted;
}

void CPU::irq()
{
    // Kept for compatibility with any caller that still pulses IRQ directly.
    // Normal system wiring uses Bus::clock() to drive the shared level line.
    m_irqLine = true;
}

void CPU::serviceNmi()
{
    // Interrupt entry must remain visible on the CPU bus for all seven
    // cycles. In particular, an IRQ/BRK sequence can have its vector fetch
    // hijacked by an NMI that arrives after the sequence has begun.
    m_nmiPolled = false;
    m_irqPolled = false;
    m_pendingIoOp = PendingIoOp::InterruptNmi;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_cycles = 7;
    m_pollInterruptsThisSequence = false;
}

void CPU::serviceIrq()
{
    m_irqPolled = false;
    m_pendingIoOp = PendingIoOp::InterruptIrqIrq;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_cycles = 7;
    m_pollInterruptsThisSequence = false;
}

bool CPU::isInterruptEntry() const
{
    switch (m_pendingIoOp) {
    case PendingIoOp::InterruptBrkIrq:
    case PendingIoOp::InterruptBrkNmi:
    case PendingIoOp::InterruptIrqIrq:
    case PendingIoOp::InterruptIrqNmi:
    case PendingIoOp::InterruptNmi:
        return true;
    default:
        return false;
    }
}

void CPU::clockInterruptEntry()
{
    const bool brkSequence =
        m_pendingIoOp == PendingIoOp::InterruptBrkIrq ||
        m_pendingIoOp == PendingIoOp::InterruptBrkNmi;
    const bool irqSequence =
        m_pendingIoOp == PendingIoOp::InterruptIrqIrq ||
        m_pendingIoOp == PendingIoOp::InterruptIrqNmi;

    // NMI can hijack BRK/IRQ vectoring without changing what has already
    // been pushed. The edge must arrive no later than the low-vector fetch.
    // For BRK this gives the five-cycle hijack window exercised by Blargg's
    // nmi_and_brk test (cycles 2 through 6 of BRK).
    if ((brkSequence || irqSequence) &&
        (m_pendingIoOp == PendingIoOp::InterruptBrkIrq ||
         m_pendingIoOp == PendingIoOp::InterruptIrqIrq) &&
        m_nmiPending && m_cycles >= 2) {
        m_pendingIoOp = brkSequence
            ? PendingIoOp::InterruptBrkNmi
            : PendingIoOp::InterruptIrqNmi;
        m_nmiPending = false;
        m_nmiPolled = false;
    }

    const bool useNmiVector =
        m_pendingIoOp == PendingIoOp::InterruptBrkNmi ||
        m_pendingIoOp == PendingIoOp::InterruptIrqNmi ||
        m_pendingIoOp == PendingIoOp::InterruptNmi;
    const uint16_t vector = useNmiVector ? 0xFFFA : 0xFFFE;

    if (brkSequence) {
        // BRK cycle 1 (opcode fetch) was performed by fetch(). PC has already
        // been advanced by two so the stack receives the architectural BRK
        // return address; cycle 2 still reads the padding byte at PC-1.
        switch (m_cycles) {
        case 6:
            (void)read(static_cast<uint16_t>(m_pc - 1));
            break;
        case 5:
            push(static_cast<uint8_t>(m_pc >> 8));
            break;
        case 4:
            push(static_cast<uint8_t>(m_pc & 0xFF));
            break;
        case 3:
            push(static_cast<uint8_t>(m_status | 0x30));
            setFlag(0x04, true);
            break;
        case 2:
            m_pendingIoData = read(vector);
            break;
        case 1: {
            const uint8_t hi = read(static_cast<uint16_t>(vector + 1));
            m_pc = static_cast<uint16_t>(m_pendingIoData) |
                (static_cast<uint16_t>(hi) << 8);
            m_pendingIoOp = PendingIoOp::None;
            break;
        }
        default:
            break;
        }
        return;
    }

    // Hardware IRQ/NMI entry has two discarded reads before the three stack
    // writes. Keeping those cycles explicit is required for NMI-over-IRQ
    // vector hijacking and for interrupt/DMA timing tests.
    switch (m_cycles) {
    case 7:
    case 6:
        (void)read(m_pc);
        break;
    case 5:
        push(static_cast<uint8_t>(m_pc >> 8));
        break;
    case 4:
        push(static_cast<uint8_t>(m_pc & 0xFF));
        break;
    case 3:
        push(static_cast<uint8_t>((m_status & ~0x10) | 0x20));
        setFlag(0x04, true);
        break;
    case 2:
        m_pendingIoData = read(vector);
        break;
    case 1: {
        const uint8_t hi = read(static_cast<uint16_t>(vector + 1));
        m_pc = static_cast<uint16_t>(m_pendingIoData) |
            (static_cast<uint16_t>(hi) << 8);
        m_pendingIoOp = PendingIoOp::None;
        break;
    }
    default:
        break;
    }
}

void CPU::pollInterrupts()
{
    // NMOS 6502 interrupt inputs are not decided from the live pins at the
    // next opcode boundary.  For ordinary instructions they are polled near
    // the end of the current instruction.  If an APU IRQ becomes active
    // after this point, the following instruction still executes before the
    // IRQ sequence begins (the distinction measured by Blargg 08.irq_timing).
    if (m_nmiPending) {
        m_nmiPolled = true;
        m_nmiPending = false;
    }

    // CLI/SEI/PLP change I too late to affect their own interrupt poll.
    // RTI is intentionally excluded: its restored I flag is visible in time.
    bool irqDisabled = getFlag(0x04);
    if (m_currentOpcode == 0x58 || m_currentOpcode == 0x78 || m_currentOpcode == 0x28)
        irqDisabled = m_irqDisableBeforeInstruction;

    m_irqPolled = m_irqPolled || (m_irqLine && !irqDisabled);
}

bool CPU::isBranchOpcode(uint8_t opcode)
{
    switch (opcode) {
    case 0x10: case 0x30: case 0x50: case 0x70:
    case 0x90: case 0xB0: case 0xD0: case 0xF0:
        return true;
    default:
        return false;
    }
}

bool CPU::needsSecondCyclePcRead(uint8_t opcode)
{
    // NMOS 6502 one-byte instructions still fetch the following byte during
    // cycle 2. The value is discarded, but the bus access is observable when
    // PC points at memory-mapped I/O (notably $2002 in cpu_exec_space_ppuio).
    // JAM/KIL opcodes are intentionally excluded because this core models
    // their halted state separately.
    switch (opcode) {
    case 0x08: case 0x0A: case 0x18: case 0x1A:
    case 0x28: case 0x2A: case 0x38: case 0x3A:
    case 0x40: case 0x48: case 0x4A: case 0x58: case 0x5A:
    case 0x60: case 0x68: case 0x6A: case 0x78: case 0x7A:
    case 0x88: case 0x8A: case 0x98: case 0x9A:
    case 0xA8: case 0xAA: case 0xB8: case 0xBA:
    case 0xC8: case 0xCA: case 0xD8: case 0xDA:
    case 0xE8: case 0xEA: case 0xF8: case 0xFA:
        return true;
    default:
        return false;
    }
}

bool CPU::indexedDummyReadAddress(uint16_t& addr) const
{
    // Indexed 6502 addressing performs a provisional read before the final
    // access. For read instructions this occurs only when the low-byte add
    // carries into the high byte; indexed stores always perform it. The
    // provisional address uses the original high byte with the corrected
    // low byte. This bus access is observable through PPU/APU I/O mirrors.
    uint16_t base = 0;
    bool always = false;

    switch (m_currentOpcode) {
    // abs,X reads whose final access is kept pending by this core
    case 0xBD: // LDA abs,X
    case 0xBC: // LDY abs,X
        base = static_cast<uint16_t>(m_pendingIoAddr - m_x);
        break;

    // abs,Y reads whose final access is kept pending
    case 0xB9: // LDA abs,Y
    case 0xBE: // LDX abs,Y
    case 0xBF: // LAX abs,Y (unofficial)
        base = static_cast<uint16_t>(m_pendingIoAddr - m_y);
        break;

    // (zp),Y reads whose final access is kept pending
    case 0xB1: // LDA (zp),Y
    case 0xB3: // LAX (zp),Y (unofficial)
        base = static_cast<uint16_t>(m_pendingIoAddr - m_y);
        break;

    // Indexed stores always issue the provisional read, regardless of carry.
    case 0x9D: // STA abs,X
        base = static_cast<uint16_t>(m_pendingIoAddr - m_x);
        always = true;
        break;
    case 0x99: // STA abs,Y
    case 0x91: // STA (zp),Y
        base = static_cast<uint16_t>(m_pendingIoAddr - m_y);
        always = true;
        break;

    default:
        return false;
    }

    if (!always && (base & 0xFF00) == (m_pendingIoAddr & 0xFF00))
        return false;

    addr = static_cast<uint16_t>((base & 0xFF00) | (m_pendingIoAddr & 0x00FF));
    return true;
}

void CPU::clock()
{
    bool startedInstruction = false;

    if (m_cycles == 0) {
        // Service only interrupts committed by the previous instruction's
        // poll. NMI has priority if both were sampled.
        if (m_nmiPolled) {
            serviceNmi();
        }
        else if (m_irqPolled) {
            serviceIrq();
        }
        else {
            // Capture pre-instruction trace metadata only while tracing is
            // enabled, keeping normal emulation free of debugger memory peeks.
            if (m_traceCaptureEnabled) {
                m_lastInstructionPc = m_pc;
                m_lastA = m_a;
                m_lastX = m_x;
                m_lastY = m_y;
                m_lastSp = m_sp;
                m_lastStatus = m_status;
                m_lastOperand1 = m_bus.debugRead(static_cast<uint16_t>(m_pc + 1));
                m_lastOperand2 = m_bus.debugRead(static_cast<uint16_t>(m_pc + 2));
            }
            const bool irqDisableBefore = getFlag(0x04);
            uint8_t opcode = fetch();

            // The opcode fetch advances PC before the instruction body runs.
            // Capture that address now because RTS/RTI/BRK mutate PC
            // immediately in this instruction-oriented core, while hardware
            // still performs its discarded cycle-2 read from this location.
            if (needsSecondCyclePcRead(opcode)) {
                m_pendingIoOp = PendingIoOp::DummyRead;
                m_pendingIoAddr = m_pc;
                m_pendingIoData = 0;
            }

            m_currentOpcode = opcode;
            m_irqDisableBeforeInstruction = irqDisableBefore;
            m_pollInterruptsThisSequence = true;
            m_branchPageCrossed = false;
            startedInstruction = true;
            if (m_traceCaptureEnabled)
                m_lastOpcode = opcode;
            ++m_instructionCount;
            execute(opcode);
        }
    }

    if (m_cycles > 0) {
        // BRK/IRQ/NMI use an explicit seven-cycle bus sequence. BRK's first
        // cycle was the opcode fetch above, so its sequence begins on the next
        // CPU clock; hardware IRQ/NMI begin immediately at this boundary.
        if (isInterruptEntry()) {
            if (!startedInstruction)
                clockInterruptEntry();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        // Cycle 2 of every one-byte NMOS instruction reads the byte at the PC
        // following the opcode. The instruction body is still atomic, but
        // preserving this externally visible I/O read lets PPU/APU registers
        // observe the same side effects as hardware.
        if (!startedInstruction && m_pendingIoOp == PendingIoOp::DummyRead) {
            const uint16_t dummyAddr = m_pendingIoAddr;
            m_pendingIoOp = PendingIoOp::None;
            (void)read(dummyAddr);
        }

        // Indexed addressing has a provisional bus read one cycle before the
        // final data access. For page-crossing reads this uses the old high
        // byte; indexed stores perform it unconditionally. Keep it on the
        // penultimate CPU cycle so memory-mapped I/O sees the correct order.
        if (!startedInstruction && m_cycles == 2 &&
            m_pendingIoOp != PendingIoOp::None &&
            m_pendingIoOp != PendingIoOp::DummyRead) {
            uint16_t dummyAddr = 0;
            if (indexedDummyReadAddress(dummyAddr))
                (void)read(dummyAddr);
        }

        // Most instructions poll IRQ/NMI on their second-to-last cycle.
        // Branches are an NMOS 6502 exception: all branches poll during the
        // opcode/offset setup (before cycle 2), and a taken branch that crosses
        // a page polls a second time before the PCH fixup cycle.
        if (m_pollInterruptsThisSequence) {
            if (isBranchOpcode(m_currentOpcode)) {
                if (startedInstruction || (m_branchPageCrossed && m_cycles == 2))
                    pollInterrupts();
            }
            else if (m_cycles == 2) {
                pollInterrupts();
            }
        }

        // Memory-mapped APU I/O happens on the final CPU cycle of the
        // instruction. Bus::clock() advances the APU before calling here,
        // which also gives the correct ordering for same-cycle frame/length
        // clocks versus CPU register accesses.
        if (m_cycles == 1)
            completePendingIo();
        --m_cycles;
        if (m_cycles == 0)
            m_pollInterruptsThisSequence = false;
    }
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

    // Establish the base cycle count before executing the operation.
    // Addressing helpers and branches may add page-cross/taken-branch
    // penalties to m_cycles.  The old ordering overwrote those penalties.
    if (inst.operate) {
        m_cycles = inst.cycles;
        (this->*inst.operate)();
    }
    else {
        m_cycles = 2;
    }
}

void CPU::scheduleIoWrite(uint16_t addr, uint8_t data)
{
    // Stores put their value on the external bus on the final instruction
    // cycle. Keeping every scheduled store pending (not just PPU/APU writes)
    // also gives indexed stores a real penultimate dummy-read slot.
    m_pendingIoOp = PendingIoOp::Write;
    m_pendingIoAddr = addr;
    m_pendingIoData = data;
}

void CPU::scheduleIoRead(PendingIoOp op, uint16_t addr)
{
    // Loads/BIT/LAX that use this helper perform the data read on their final
    // cycle. This is required for indexed page-cross dummy reads: the
    // provisional read must happen first, then the corrected effective read.
    m_pendingIoOp = op;
    m_pendingIoAddr = addr;
    m_pendingIoData = 0;
}

void CPU::completePendingIo()
{
    if (m_pendingIoOp == PendingIoOp::None)
        return;

    const PendingIoOp op = m_pendingIoOp;
    const uint16_t addr = m_pendingIoAddr;
    const uint8_t data = m_pendingIoData;
    m_pendingIoOp = PendingIoOp::None;

    if (op == PendingIoOp::Write) {
        write(addr, data);
        return;
    }
    if (op == PendingIoOp::DummyRead) {
        (void)read(addr);
        return;
    }

    const uint8_t value = read(addr);
    switch (op) {
    case PendingIoOp::Lda:
        m_a = value; setZeroNeg(m_a); break;
    case PendingIoOp::Ldx:
        m_x = value; setZeroNeg(m_x); break;
    case PendingIoOp::Ldy:
        m_y = value; setZeroNeg(m_y); break;
    case PendingIoOp::Bit:
        setFlag(0x02, (m_a & value) == 0);
        setFlag(0x40, (value & 0x40) != 0);
        setFlag(0x80, (value & 0x80) != 0);
        break;
    case PendingIoOp::Lax:
        m_a = m_x = value; setZeroNeg(value); break;
    default:
        break;
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

    // NMOS 6502 JMP-indirect page-wrap bug: if the pointer ends in
    // $FF, the high byte is read from the beginning of the same page.
    uint16_t hiAddr = (uint16_t)((ptr & 0xFF00) | ((ptr + 1) & 0x00FF));
    uint8_t hi = read(hiAddr);

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
    m_branchPageCrossed = false;
    if (condition) {
        uint16_t oldPC = m_pc;
        m_pc += offset;
        m_cycles++; // branch taken
        if ((oldPC & 0xFF00) != (m_pc & 0xFF00)) {
            m_cycles++; // page cross
            m_branchPageCrossed = true;
        }
    }
}

uint8_t CPU::read(uint16_t addr) const { return m_bus.read(addr); }
void CPU::write(uint16_t addr, uint8_t data) { m_bus.write(addr, data); }

void CPU::writeRmw(uint16_t addr, uint8_t oldValue, uint8_t newValue)
{
    // NMOS 6502 memory read-modify-write instructions perform two writes:
    // the original value first, then the modified value.  The CPU core still
    // executes an instruction atomically, but preserving both bus writes fixes
    // mapper/I/O side effects and gives the cycle-stepped Bus a sound base for
    // later per-cycle scheduling.
    write(addr, oldValue);
    write(addr, newValue);
}

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
    scheduleIoRead(PendingIoOp::Lda, addrAbsolute());
}

void CPU::opLDX_zp()
{
    m_x = read(addrZeroPage());
    setZeroNeg(m_x);
}

void CPU::opLDX_abs()
{
    scheduleIoRead(PendingIoOp::Ldx, addrAbsolute());
}

void CPU::opLDY_zp()
{
    m_y = read(addrZeroPage());
    setZeroNeg(m_y);
}

void CPU::opLDY_abs()
{
    scheduleIoRead(PendingIoOp::Ldy, addrAbsolute());
}

// NEW Loads: indexed & indirect

void CPU::opLDA_zpx()
{
    m_a = read(addrZeroPageX());
    setZeroNeg(m_a);
}

void CPU::opLDA_absx()
{
    scheduleIoRead(PendingIoOp::Lda, addrAbsoluteXRead());
}

void CPU::opLDA_absy()
{
    scheduleIoRead(PendingIoOp::Lda, addrAbsoluteYRead());
}

void CPU::opLDA_indx()
{
    scheduleIoRead(PendingIoOp::Lda, addrIndirectX());
}

void CPU::opLDA_indy()
{
    scheduleIoRead(PendingIoOp::Lda, addrIndirectYRead());
}

void CPU::opLDX_zpy()
{
    m_x = read(addrZeroPageY());
    setZeroNeg(m_x);
}

void CPU::opLDX_absy()
{
    scheduleIoRead(PendingIoOp::Ldx, addrAbsoluteYRead());
}

void CPU::opLDY_zpx()
{
    m_y = read(addrZeroPageX());
    setZeroNeg(m_y);
}

void CPU::opLDY_absx()
{
    scheduleIoRead(PendingIoOp::Ldy, addrAbsoluteXRead());
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
    // BRK is architecturally a two-byte instruction, but its stack writes and
    // vector fetch happen over later bus cycles. Delaying those operations is
    // what allows a late NMI edge to hijack the BRK vector while preserving
    // the B flag in the already-pushed status byte.
    m_pc++;
    m_pendingIoOp = PendingIoOp::InterruptBrkIrq;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pollInterruptsThisSequence = false;
}

void CPU::opRTI()
{
    // B is not a persistent CPU flag; bit 5 is always set in the
    // internal representation.
    m_status = (pull() & 0xEF) | 0x20;
    m_pc = pull16();
}

// Stores
void CPU::opSTA_zp() { write(addrZeroPage(), m_a); }
void CPU::opSTX_zp() { write(addrZeroPage(), m_x); }
void CPU::opSTY_zp() { write(addrZeroPage(), m_y); }

void CPU::opSTA_zpx() { write(addrZeroPageX(), m_a); }
void CPU::opSTX_zpy() { write(addrZeroPageY(), m_x); }
void CPU::opSTY_zpx() { write(addrZeroPageX(), m_y); }

void CPU::opSTA_abs() { scheduleIoWrite(addrAbsolute(), m_a); }
void CPU::opSTA_absx() { scheduleIoWrite(addrAbsoluteX(), m_a); }
void CPU::opSTA_absy() { scheduleIoWrite(addrAbsoluteY(), m_a); }

void CPU::opSTA_indx()
{
    uint16_t addr = addrIndirectX();
    scheduleIoWrite(addr, m_a);
}

void CPU::opSTA_indy()
{
    uint16_t addr = addrIndirectY();
    scheduleIoWrite(addr, m_a);
}

void CPU::opSTX_abs()
{
    uint16_t addr = addrAbsolute();
    scheduleIoWrite(addr, m_x);
}

void CPU::opSTY_abs()
{
    uint16_t addr = addrAbsolute();
    scheduleIoWrite(addr, m_y);
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
    scheduleIoRead(PendingIoOp::Bit, addrAbsolute());
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
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opASL_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opASL_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opASL_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
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
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opLSR_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opLSR_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opLSR_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opROL_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opROL_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opROL_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opROR_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opROR_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);
    setZeroNeg(v);
}

void CPU::opROR_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);
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
void CPU::opPHP() { push(m_status | 0x30); }
void CPU::opPLA() { m_a = pull(); setZeroNeg(m_a); }
void CPU::opPLP() { m_status = (pull() & 0xEF) | 0x20; }

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
    uint8_t oldValue = value;
    value++;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opINC_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value++;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opINC_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value++;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opINC_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value++;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opDEC_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value--;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opDEC_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value--;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opDEC_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value--;
    writeRmw(addr, oldValue, value);
    setZeroNeg(value);
}

void CPU::opDEC_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t value = read(addr);
    uint8_t oldValue = value;
    value--;
    writeRmw(addr, oldValue, value);
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
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_indy()
{
    uint16_t addr = addrIndirectY(); // no page-cross penalty on RMW
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

void CPU::opSLO_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x80);
    v <<= 1;
    writeRmw(addr, oldValue, v);
    m_a |= v;
    setZeroNeg(m_a);
}

// ---------- RLA = ROL mem then AND A ----------
void CPU::opRLA_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

void CPU::opRLA_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 1 : 0;
    setFlag(0x01, v & 0x80);
    v = (uint8_t)((v << 1) | carryIn);
    writeRmw(addr, oldValue, v);
    m_a &= v;
    setZeroNeg(m_a);
}

// ---------- SRE = LSR mem then EOR A ----------
void CPU::opSRE_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

void CPU::opSRE_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    setFlag(0x01, v & 0x01);
    v >>= 1;
    writeRmw(addr, oldValue, v);
    m_a ^= v;
    setZeroNeg(m_a);
}

// ---------- RRA = ROR mem then ADC A ----------
void CPU::opRRA_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
    setFlag(0x01, v & 0x01);
    v = (uint8_t)((v >> 1) | carryIn);
    writeRmw(addr, oldValue, v);

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
    scheduleIoWrite(addr, m_a & m_x);
}

void CPU::opSAX_zpy()
{
    uint16_t addr = addrZeroPageY();
    write(addr, m_a & m_x);
}

// ---------- LAX = LDA + LDX ----------
void CPU::opLAX_indx()
{
    scheduleIoRead(PendingIoOp::Lax, addrIndirectX());
}

void CPU::opLAX_zp()
{
    uint8_t v = read(addrZeroPage());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_abs()
{
    scheduleIoRead(PendingIoOp::Lax, addrAbsolute());
}

void CPU::opLAX_indy()
{
    scheduleIoRead(PendingIoOp::Lax, addrIndirectYRead());
}

void CPU::opLAX_zpy()
{
    uint8_t v = read(addrZeroPageY());
    m_a = m_x = v;
    setZeroNeg(v);
}

void CPU::opLAX_absy()
{
    scheduleIoRead(PendingIoOp::Lax, addrAbsoluteYRead());
}

// ---------- DCP = DEC mem then CMP A ----------
void CPU::opDCP_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_zp()
{
    uint16_t addr = addrZeroPage();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_abs()
{
    uint16_t addr = addrAbsolute();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_indy()
{
    uint16_t addr = addrIndirectY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_zpx()
{
    uint16_t addr = addrZeroPageX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_absy()
{
    uint16_t addr = addrAbsoluteY();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

void CPU::opDCP_absx()
{
    uint16_t addr = addrAbsoluteX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v--;
    writeRmw(addr, oldValue, v);
    cmpHelper(m_a, v);
}

// ---------- ISC / ISB = INC mem then SBC A ----------
void CPU::opISC_indx()
{
    uint16_t addr = addrIndirectX();
    uint8_t v = read(addr);
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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
    uint8_t oldValue = v;
    v++;
    writeRmw(addr, oldValue, v);

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


CPU::DebugState CPU::debugState() const
{
    DebugState state;
    state.a = m_a;
    state.x = m_x;
    state.y = m_y;
    state.sp = m_sp;
    state.pc = m_pc;
    state.status = m_status;
    state.cyclesRemaining = m_cycles;
    state.nmiPending = m_nmiPending || m_nmiPolled;
    state.nmiPolled = m_nmiPolled;
    state.irqLine = m_irqLine;
    state.irqPolled = m_irqPolled;
    state.instructionCount = m_instructionCount;
    return state;
}

CPU::TraceState CPU::lastTraceState() const
{
    TraceState state;
    state.pc = m_lastInstructionPc;
    state.opcode = m_lastOpcode;
    state.operand1 = m_lastOperand1;
    state.operand2 = m_lastOperand2;
    state.a = m_lastA;
    state.x = m_lastX;
    state.y = m_lastY;
    state.sp = m_lastSp;
    state.status = m_lastStatus;
    state.instructionCount = m_instructionCount;
    return state;
}

void CPU::saveState(std::vector<uint8_t>& out) const
{
    auto put8 = [&](uint8_t v) { out.push_back(v); };
    auto put16 = [&](uint16_t v) { out.push_back(v & 0xFF); out.push_back((v >> 8) & 0xFF); };
    auto put32 = [&](uint32_t v) {
        for (int i = 0; i < 4; i++) out.push_back((v >> (i * 8)) & 0xFF);
        };
    put8(m_a); put8(m_x); put8(m_y); put8(m_sp);
    put16(m_pc); put8(m_status);
    put32((uint32_t)m_cycles);
    put8(m_nmiPending ? 1 : 0);
    put8(m_nmiPolled ? 1 : 0);
    put8(m_irqLine ? 1 : 0);
    put8(m_irqPolled ? 1 : 0);
    put8(m_pollInterruptsThisSequence ? 1 : 0);
    put8(m_irqDisableBeforeInstruction ? 1 : 0);
    put8(m_currentOpcode);
    put8(m_branchPageCrossed ? 1 : 0);
    put8(static_cast<uint8_t>(m_pendingIoOp));
    put16(m_pendingIoAddr);
    put8(m_pendingIoData);
}

bool CPU::loadState(const uint8_t*& p, const uint8_t* end)
{
    auto get8 = [&](uint8_t& v) -> bool {
        if (p >= end) return false; v = *p++; return true;
        };
    auto get16 = [&](uint16_t& v) -> bool {
        if (p + 2 > end) return false;
        v = p[0] | (uint16_t(p[1]) << 8); p += 2; return true;
        };
    auto get32 = [&](uint32_t& v) -> bool {
        if (p + 4 > end) return false;
        v = p[0] | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
        p += 4; return true;
        };
    uint32_t cycles = 0;
    uint8_t nmiPending = 0, nmiPolled = 0, irqLine = 0, irqPolled = 0;
    uint8_t pollSequence = 0, irqDisableBefore = 0, currentOpcode = 0, branchPageCrossed = 0, pendingIo = 0;
    uint16_t pendingAddr = 0;
    uint8_t pendingData = 0;
    if (!get8(m_a) || !get8(m_x) || !get8(m_y) || !get8(m_sp)) return false;
    if (!get16(m_pc) || !get8(m_status) || !get32(cycles)) return false;
    if (!get8(nmiPending) || !get8(nmiPolled) || !get8(irqLine) || !get8(irqPolled) ||
        !get8(pollSequence) || !get8(irqDisableBefore) || !get8(currentOpcode) ||
        !get8(branchPageCrossed) || !get8(pendingIo) ||
        !get16(pendingAddr) || !get8(pendingData)) return false;
    if (pendingIo > static_cast<uint8_t>(PendingIoOp::InterruptNmi)) return false;
    m_status = (m_status & 0xEF) | 0x20;
    m_cycles = (int)cycles;
    m_nmiPending = nmiPending != 0;
    m_nmiPolled = nmiPolled != 0;
    m_irqLine = irqLine != 0;
    m_irqPolled = irqPolled != 0;
    m_pollInterruptsThisSequence = pollSequence != 0;
    m_irqDisableBeforeInstruction = irqDisableBefore != 0;
    m_currentOpcode = currentOpcode;
    m_branchPageCrossed = branchPageCrossed != 0;
    m_pendingIoOp = static_cast<PendingIoOp>(pendingIo);
    m_pendingIoAddr = pendingAddr;
    m_pendingIoData = pendingData;
    return true;
}
























