#include "CPU.hpp"
#include "Bus.hpp"

CPU::CPU(Bus& bus) : m_bus(bus)
{
    buildTable();
}

void CPU::powerOn()
{
    // The RP2A03's externally observable power-up state used by the NES CPU
    // reset tests. Bit 5 is kept set internally; B is synthesized only when
    // status is pushed by PHP/BRK.
    m_a = 0;
    m_x = 0;
    m_y = 0;
    // Power-on enters the same externally visible RESET bus sequence. Start
    // S at $00 so the three reset stack-page reads/decrements leave it at
    // the NES-observed $FD value when opcode execution begins.
    m_sp = 0x00;
    m_status = 0x24;
    m_pc = 0x0000;

    m_nmiPending = false;
    m_nmiSampled = false;
    m_nmiPolled = false;
    m_irqLine = false;
    m_irqPolled = false;
    m_pollInterruptsThisSequence = false;
    m_irqDisableBeforeInstruction = true;
    m_currentOpcode = 0;
    m_branchPageCrossed = false;
    m_pendingIoOp = PendingIoOp::ResetSequence;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_instructionCount = 0;

    // RESET occupies seven externally visible CPU bus clocks.
    m_cycles = 7;
}

void CPU::reset()
{
    // The physical RESET input is not a second power-on. A, X and Y are not
    // modified. Its three would-be stack writes are reads on real hardware;
    // defer those accesses and S decrements to clockResetSequence().

    // RESET asserts interrupt-disable while otherwise preserving flags. The
    // B flag is not a physical status latch, so keep our internal copy clear.
    m_status = static_cast<uint8_t>((m_status | 0x24) & 0xEF);

    m_nmiPending = false;
    m_nmiSampled = false;
    m_nmiPolled = false;
    m_irqLine = false;
    m_irqPolled = false;
    m_pollInterruptsThisSequence = false;
    m_irqDisableBeforeInstruction = true;
    m_currentOpcode = 0;
    m_branchPageCrossed = false;
    m_pendingIoOp = PendingIoOp::ResetSequence;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;

    // The reset sequence consumes seven explicit CPU bus clocks.
    m_cycles = 7;
}

void CPU::nmi()
{
    // NMI is edge-triggered. Latch it and service it at the next
    // instruction boundary rather than interrupting an instruction that
    // is already in progress.
    m_nmiPending = true;
}

void CPU::sampleNmiInput()
{
    if (m_nmiPending) {
        m_nmiSampled = true;
        m_nmiPending = false;
    }
}

void CPU::cancelPendingNmi()
{
    // Once the CPU has sampled the edge for the current instruction it is
    // committed. Only the asynchronous, not-yet-polled edge can disappear
    // in the PPU's short VBlank/NMI suppression race.
    if (!m_nmiPolled && !m_nmiSampled)
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
    m_nmiSampled = false;
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

    // NMI takeover has a subtle BRK/IRQ distinction. BRK and hardware IRQ both commit the vector after the status-push
    // boundary.  BRK still retains its origin semantics (PC+2 and B=1 in the
    // pushed status) when an NMI has hijacked the vector before that point.
    // BRK and hardware IRQ select their vector at the boundary immediately
    // before the status-push tick (T4 -> T5 internally). An NMI edge that has
    // been recognized by then can hijack the sequence; an edge arriving once
    // the low-vector read is about to begin is too late and must remain
    // pending for the next interrupt opportunity. Keeping BRK hijackable at
    // m_cycles == 2 incorrectly allowed a late NMI to redirect only after
    // vector selection had already committed.
    const bool brkCanBeHijacked =
        brkSequence && m_pendingIoOp == PendingIoOp::InterruptBrkIrq &&
        m_nmiSampled && m_cycles >= 3;
    const bool irqCanBeHijacked =
        irqSequence && m_pendingIoOp == PendingIoOp::InterruptIrqIrq &&
        m_nmiSampled && m_cycles >= 3;

    if (brkCanBeHijacked || irqCanBeHijacked) {
        m_pendingIoOp = brkSequence
            ? PendingIoOp::InterruptBrkNmi
            : PendingIoOp::InterruptIrqNmi;
        m_nmiSampled = false;
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
        m_bus.applyCartridgeResetBootstrap(m_pc, m_sp);
        m_pendingIoOp = PendingIoOp::None;
        break;
    }
    default:
        break;
    }
}

bool CPU::isResetSequence() const
{
    return m_pendingIoOp == PendingIoOp::ResetSequence;
}

bool CPU::isStackSequence() const
{
    switch (m_pendingIoOp) {
    case PendingIoOp::StackPha:
    case PendingIoOp::StackPhp:
    case PendingIoOp::StackPla:
    case PendingIoOp::StackPlp:
    case PendingIoOp::StackJsr:
    case PendingIoOp::StackRts:
    case PendingIoOp::StackRti:
        return true;
    default:
        return false;
    }
}

bool CPU::isZpIndexedSequence() const
{
    switch (m_pendingIoOp) {
    case PendingIoOp::ZpXLda:
    case PendingIoOp::ZpYLdx:
    case PendingIoOp::ZpXLdy:
    case PendingIoOp::ZpXSta:
    case PendingIoOp::ZpYStx:
    case PendingIoOp::ZpXSty:
    case PendingIoOp::ZpXAnd:
    case PendingIoOp::ZpXOra:
    case PendingIoOp::ZpXEor:
    case PendingIoOp::ZpXAdc:
    case PendingIoOp::ZpXSbc:
    case PendingIoOp::ZpXCmp:
    case PendingIoOp::ZpXNop:
    case PendingIoOp::ZpYLax:
    case PendingIoOp::ZpYSax:
        return true;
    default:
        return false;
    }
}

void CPU::clockZpIndexedSequence()
{
    const PendingIoOp op = m_pendingIoOp;
    const uint8_t index = (op == PendingIoOp::ZpYLdx || op == PendingIoOp::ZpYStx ||
                           op == PendingIoOp::ZpYLax || op == PendingIoOp::ZpYSax)
        ? m_y : m_x;

    if (m_cycles == 3) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }

    const uint8_t base = static_cast<uint8_t>(m_pendingIoAddr);
    if (m_cycles == 2) {
        (void)read(base);
        return;
    }

    if (m_cycles != 1)
        return;

    const uint16_t addr = static_cast<uint8_t>(base + index);
    switch (op) {
    case PendingIoOp::ZpXLda:
        m_a = read(addr);
        setZeroNeg(m_a);
        break;
    case PendingIoOp::ZpYLdx:
        m_x = read(addr);
        setZeroNeg(m_x);
        break;
    case PendingIoOp::ZpXLdy:
        m_y = read(addr);
        setZeroNeg(m_y);
        break;
    case PendingIoOp::ZpXSta:
        write(addr, m_a);
        break;
    case PendingIoOp::ZpYStx:
        write(addr, m_x);
        break;
    case PendingIoOp::ZpXSty:
        write(addr, m_y);
        break;
    case PendingIoOp::ZpXAnd:
        m_a = static_cast<uint8_t>(m_a & read(addr));
        setZeroNeg(m_a);
        break;
    case PendingIoOp::ZpXOra:
        m_a = static_cast<uint8_t>(m_a | read(addr));
        setZeroNeg(m_a);
        break;
    case PendingIoOp::ZpXEor:
        m_a = static_cast<uint8_t>(m_a ^ read(addr));
        setZeroNeg(m_a);
        break;
    case PendingIoOp::ZpXAdc: {
        const uint8_t value = read(addr);
        const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80) != 0);
        m_a = result;
        setZeroNeg(m_a);
        break;
    }
    case PendingIoOp::ZpXSbc: {
        const uint8_t value = static_cast<uint8_t>(read(addr) ^ 0xFF);
        const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, ((m_a ^ result) & (value ^ result) & 0x80) != 0);
        m_a = result;
        setZeroNeg(m_a);
        break;
    }
    case PendingIoOp::ZpXCmp:
        cmpHelper(m_a, read(addr));
        break;
    case PendingIoOp::ZpXNop:
        (void)read(addr);
        break;
    case PendingIoOp::ZpYLax: {
        const uint8_t value = read(addr);
        m_a = m_x = value;
        setZeroNeg(value);
        break;
    }
    case PendingIoOp::ZpYSax:
        write(addr, static_cast<uint8_t>(m_a & m_x));
        break;
    default:
        break;
    }
    m_pendingIoOp = PendingIoOp::None;
}



bool CPU::isDirectMemorySequence() const
{
    return m_pendingIoOp == PendingIoOp::ZpRead ||
           m_pendingIoOp == PendingIoOp::ZpStore ||
           m_pendingIoOp == PendingIoOp::AbsRead ||
           m_pendingIoOp == PendingIoOp::AbsStore;
}

uint8_t CPU::directStoreValue() const
{
    switch (m_currentOpcode) {
    case 0x85: case 0x8D: return m_a; // STA
    case 0x86: case 0x8E: return m_x; // STX
    case 0x84: case 0x8C: return m_y; // STY
    case 0x87: case 0x8F: return static_cast<uint8_t>(m_a & m_x); // SAX
    default: return 0;
    }
}

void CPU::applyDirectMemoryRead(uint8_t value)
{
    switch (m_currentOpcode) {
    case 0xA5: case 0xAD: // LDA
        m_a = value; setZeroNeg(m_a); break;
    case 0xA6: case 0xAE: // LDX
        m_x = value; setZeroNeg(m_x); break;
    case 0xA4: case 0xAC: // LDY
        m_y = value; setZeroNeg(m_y); break;
    case 0x25: case 0x2D: // AND
        m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a); break;
    case 0x05: case 0x0D: // ORA
        m_a = static_cast<uint8_t>(m_a | value); setZeroNeg(m_a); break;
    case 0x45: case 0x4D: // EOR
        m_a = static_cast<uint8_t>(m_a ^ value); setZeroNeg(m_a); break;
    case 0x65: case 0x6D: { // ADC
        const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a);
        break;
    }
    case 0xE5: case 0xED: { // SBC
        const uint8_t inv = static_cast<uint8_t>(value ^ 0xFF);
        const uint16_t sum = static_cast<uint16_t>(m_a) + inv + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, ((m_a ^ result) & (inv ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a);
        break;
    }
    case 0xC5: case 0xCD: // CMP
        cmpHelper(m_a, value); break;
    case 0xE4: case 0xEC: // CPX
        cmpHelper(m_x, value); break;
    case 0xC4: case 0xCC: // CPY
        cmpHelper(m_y, value); break;
    case 0x24: case 0x2C: // BIT
        setFlag(0x02, (m_a & value) == 0);
        setFlag(0x40, (value & 0x40) != 0);
        setFlag(0x80, (value & 0x80) != 0);
        break;
    case 0xA7: case 0xAF: // LAX
        m_a = m_x = value; setZeroNeg(value); break;
    case 0x04: case 0x44: case 0x64: case 0x0C: // memory NOPs
        break;
    default:
        break;
    }
}

void CPU::clockDirectMemorySequence()
{
    const bool absolute = m_pendingIoOp == PendingIoOp::AbsRead ||
                          m_pendingIoOp == PendingIoOp::AbsStore;
    const bool store = m_pendingIoOp == PendingIoOp::ZpStore ||
                       m_pendingIoOp == PendingIoOp::AbsStore;

    if (!absolute) {
        if (m_cycles == 2) {
            m_pendingIoAddr = read(m_pc++);
            return;
        }
        if (m_cycles == 1) {
            const uint16_t addr = static_cast<uint8_t>(m_pendingIoAddr);
            if (store)
                write(addr, directStoreValue());
            else
                applyDirectMemoryRead(read(addr));
            m_pendingIoOp = PendingIoOp::None;
        }
        return;
    }

    if (m_cycles == 3) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }
    if (m_cycles == 2) {
        const uint8_t hi = read(m_pc++);
        m_pendingIoAddr = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) |
                                                (m_pendingIoAddr & 0x00FF));
        return;
    }
    if (m_cycles == 1) {
        if (store)
            write(m_pendingIoAddr, directStoreValue());
        else
            applyDirectMemoryRead(read(m_pendingIoAddr));
        m_pendingIoOp = PendingIoOp::None;
    }
}


bool CPU::isAbsIndexedReadSequence() const
{
    return m_pendingIoOp == PendingIoOp::AbsXRead ||
           m_pendingIoOp == PendingIoOp::AbsYRead;
}

void CPU::applyAbsIndexedRead(uint8_t value)
{
    switch (m_currentOpcode) {
    case 0xBD: case 0xB9: // LDA abs,X / abs,Y
        m_a = value; setZeroNeg(m_a); break;
    case 0xBE: // LDX abs,Y
        m_x = value; setZeroNeg(m_x); break;
    case 0xBC: // LDY abs,X
        m_y = value; setZeroNeg(m_y); break;
    case 0x3D: case 0x39: // AND
        m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a); break;
    case 0x1D: case 0x19: // ORA
        m_a = static_cast<uint8_t>(m_a | value); setZeroNeg(m_a); break;
    case 0x5D: case 0x59: // EOR
        m_a = static_cast<uint8_t>(m_a ^ value); setZeroNeg(m_a); break;
    case 0x7D: case 0x79: { // ADC
        const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a);
        break;
    }
    case 0xFD: case 0xF9: { // SBC
        const uint8_t inv = static_cast<uint8_t>(value ^ 0xFF);
        const uint16_t sum = static_cast<uint16_t>(m_a) + inv + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, ((m_a ^ result) & (inv ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a);
        break;
    }
    case 0xDD: case 0xD9: // CMP
        cmpHelper(m_a, value); break;
    case 0xBF: // LAX abs,Y
        m_a = m_x = value; setZeroNeg(value); break;
    case 0xBB: // LAS abs,Y
        value = static_cast<uint8_t>(value & m_sp);
        m_a = m_x = m_sp = value; setZeroNeg(value); break;
    case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
        // Unofficial absolute,X NOPs still perform the final memory read.
        break;
    default:
        break;
    }
}

void CPU::clockAbsIndexedReadSequence()
{
    const uint8_t index = m_pendingIoOp == PendingIoOp::AbsXRead ? m_x : m_y;

    if (m_cycles == 3) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }

    if (m_cycles == 2 && m_pendingIoData == 0) {
        const uint8_t hi = read(m_pc++);
        const uint16_t base = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) |
                                                    (m_pendingIoAddr & 0x00FF));
        m_pendingIoAddr = base;
        m_pendingIoData = 1;
        const uint16_t effective = static_cast<uint16_t>(base + index);
        if ((base & 0xFF00) != (effective & 0xFF00))
            ++m_cycles;
        return;
    }

    const uint16_t base = m_pendingIoAddr;
    const uint16_t effective = static_cast<uint16_t>(base + index);
    if (m_cycles == 2) {
        const uint16_t provisional = static_cast<uint16_t>((base & 0xFF00) |
                                                           (effective & 0x00FF));
        (void)read(provisional);
        return;
    }

    if (m_cycles == 1) {
        const uint8_t value = read(effective);
        applyAbsIndexedRead(value);
        m_pendingIoOp = PendingIoOp::None;
        m_pendingIoData = 0;
    }
}

void CPU::notifyRdyReadStall()
{
    // AccuracyCoin deliberately pulls RDY low on the indexed provisional read
    // two clocks before the SH*/SHA write. On NMOS silicon this suppresses the
    // H+1 term for the eventual write. Only latch the condition for that exact
    // bus phase; unrelated DMC stalls must not alter the instruction.
    const bool absHigh = m_pendingIoOp == PendingIoOp::AbsXHighStore ||
                         m_pendingIoOp == PendingIoOp::AbsYHighStore;
    const bool indHigh = m_pendingIoOp == PendingIoOp::IndYHighStore;
    if ((absHigh || indHigh) && m_cycles == 2)
        m_unstableHighStoreRdy = true;
}

bool CPU::isAbsIndexedStoreSequence() const
{
    switch (m_pendingIoOp) {
    case PendingIoOp::AbsXStore:
    case PendingIoOp::AbsYStore:
    case PendingIoOp::AbsXHighStore:
    case PendingIoOp::AbsYHighStore:
        return true;
    default:
        return false;
    }
}

void CPU::clockAbsIndexedStoreSequence()
{
    const bool xIndexed = m_pendingIoOp == PendingIoOp::AbsXStore ||
                          m_pendingIoOp == PendingIoOp::AbsXHighStore;
    const bool unstable = m_pendingIoOp == PendingIoOp::AbsXHighStore ||
                          m_pendingIoOp == PendingIoOp::AbsYHighStore;
    const uint8_t index = xIndexed ? m_x : m_y;

    if (m_cycles == 4) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }
    if (m_cycles == 3) {
        const uint8_t hi = read(m_pc++);
        m_pendingIoAddr = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) |
                                                (m_pendingIoAddr & 0x00FF));
        return;
    }

    const uint16_t base = m_pendingIoAddr;
    const uint16_t effective = static_cast<uint16_t>(base + index);
    if (m_cycles == 2) {
        const uint16_t provisional = static_cast<uint16_t>((base & 0xFF00) |
                                                           (effective & 0x00FF));
        (void)read(provisional);
        return;
    }

    if (m_cycles != 1)
        return;

    if (!unstable) {
        write(effective, m_a);
        m_pendingIoOp = PendingIoOp::None;
        return;
    }

    uint8_t source = 0;
    switch (m_currentOpcode) {
    case 0x9C: source = m_y; break; // SHY abs,X
    case 0x9E: source = m_x; break; // SHX abs,Y
    case 0x9F: source = static_cast<uint8_t>(m_a & m_x); break; // AHX abs,Y
    case 0x9B: source = m_sp; break; // TAS abs,Y
    default:
        m_pendingIoOp = PendingIoOp::None;
        return;
    }

    uint8_t stored = source;
    uint16_t target = effective;
    if (!m_unstableHighStoreRdy) {
        const uint8_t hPlusOne = static_cast<uint8_t>((base >> 8) + 1);
        stored = static_cast<uint8_t>(source & hPlusOne);
        if ((base & 0xFF00) != (effective & 0xFF00))
            target = static_cast<uint16_t>((static_cast<uint16_t>(stored) << 8) |
                                           (effective & 0x00FF));
    }
    write(target, stored);
    m_pendingIoOp = PendingIoOp::None;
    m_unstableHighStoreRdy = false;
}



bool CPU::isDirectRmwSequence() const
{
    return m_pendingIoOp == PendingIoOp::ZpRmw ||
           m_pendingIoOp == PendingIoOp::AbsRmw;
}

void CPU::clockDirectRmwSequence()
{
    const bool absolute = m_pendingIoOp == PendingIoOp::AbsRmw;

    if (!absolute) {
        if (m_cycles == 4) {
            m_pendingIoAddr = read(m_pc++);
            return;
        }
        const uint16_t addr = static_cast<uint8_t>(m_pendingIoAddr);
        if (m_cycles == 3) {
            const uint8_t oldValue = read(addr);
            uint8_t newValue = oldValue;
            applyIndexedRmw(oldValue, newValue);
            m_pendingIoData = oldValue;
            m_pendingIoData2 = newValue;
            return;
        }
        if (m_cycles == 2) { write(addr, m_pendingIoData); return; }
        if (m_cycles == 1) { write(addr, m_pendingIoData2); m_pendingIoOp = PendingIoOp::None; }
        return;
    }

    if (m_cycles == 5) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }
    if (m_cycles == 4) {
        const uint8_t hi = read(m_pc++);
        m_pendingIoAddr = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) |
                                                (m_pendingIoAddr & 0x00FF));
        return;
    }
    const uint16_t addr = m_pendingIoAddr;
    if (m_cycles == 3) {
        const uint8_t oldValue = read(addr);
        uint8_t newValue = oldValue;
        applyIndexedRmw(oldValue, newValue);
        m_pendingIoData = oldValue;
        m_pendingIoData2 = newValue;
        return;
    }
    if (m_cycles == 2) { write(addr, m_pendingIoData); return; }
    if (m_cycles == 1) { write(addr, m_pendingIoData2); m_pendingIoOp = PendingIoOp::None; }
}


bool CPU::isImmediateSequence() const
{
    return m_pendingIoOp == PendingIoOp::Immediate;
}

bool CPU::isImpliedSequence() const
{
    return m_pendingIoOp == PendingIoOp::Implied;
}

void CPU::applyImplied()
{
    switch (m_currentOpcode) {
    case 0xE8: ++m_x; setZeroNeg(m_x); break; // INX
    case 0xCA: --m_x; setZeroNeg(m_x); break; // DEX
    case 0xC8: ++m_y; setZeroNeg(m_y); break; // INY
    case 0x88: --m_y; setZeroNeg(m_y); break; // DEY

    case 0x78: setFlag(0x04, true); break;  // SEI
    case 0x58: setFlag(0x04, false); break; // CLI
    case 0x18: setFlag(0x01, false); break; // CLC
    case 0x38: setFlag(0x01, true); break;  // SEC
    case 0xD8: setFlag(0x08, false); break; // CLD
    case 0xF8: setFlag(0x08, true); break;  // SED (ignored by 2A03 ADC/SBC)
    case 0xB8: setFlag(0x40, false); break; // CLV

    case 0xAA: m_x = m_a; setZeroNeg(m_x); break; // TAX
    case 0x8A: m_a = m_x; setZeroNeg(m_a); break; // TXA
    case 0xA8: m_y = m_a; setZeroNeg(m_y); break; // TAY
    case 0x98: m_a = m_y; setZeroNeg(m_a); break; // TYA
    case 0xBA: m_x = m_sp; setZeroNeg(m_x); break; // TSX
    case 0x9A: m_sp = m_x; break;                    // TXS

    case 0x0A: { // ASL A
        setFlag(0x01, (m_a & 0x80) != 0);
        m_a = static_cast<uint8_t>(m_a << 1);
        setZeroNeg(m_a);
        break;
    }
    case 0x4A: { // LSR A
        setFlag(0x01, (m_a & 0x01) != 0);
        m_a = static_cast<uint8_t>(m_a >> 1);
        setZeroNeg(m_a);
        break;
    }
    case 0x2A: { // ROL A
        const uint8_t carryIn = getFlag(0x01) ? 1 : 0;
        setFlag(0x01, (m_a & 0x80) != 0);
        m_a = static_cast<uint8_t>((m_a << 1) | carryIn);
        setZeroNeg(m_a);
        break;
    }
    case 0x6A: { // ROR A
        const uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
        setFlag(0x01, (m_a & 0x01) != 0);
        m_a = static_cast<uint8_t>((m_a >> 1) | carryIn);
        setZeroNeg(m_a);
        break;
    }

    // Official and unofficial one-byte NOPs.
    case 0xEA: case 0x1A: case 0x3A: case 0x5A:
    case 0x7A: case 0xDA: case 0xFA:
        break;
    default:
        break;
    }
}

void CPU::clockImpliedSequence()
{
    // Cycle 1 was the opcode fetch. Cycle 2 re-reads PC, then the internal
    // register/flag operation becomes visible at the end of that cycle.
    if (m_cycles != 1)
        return;
    (void)read(m_pc);
    applyImplied();
    m_pendingIoOp = PendingIoOp::None;
}

bool CPU::isJmpSequence() const
{
    return m_pendingIoOp == PendingIoOp::JmpAbs ||
           m_pendingIoOp == PendingIoOp::JmpInd;
}

bool CPU::isBranchSequence() const
{
    return m_pendingIoOp == PendingIoOp::Branch;
}

void CPU::applyImmediate(uint8_t value)
{
    switch (m_currentOpcode) {
    case 0xA9: m_a = value; setZeroNeg(m_a); break;
    case 0xA2: m_x = value; setZeroNeg(m_x); break;
    case 0xA0: m_y = value; setZeroNeg(m_y); break;
    case 0x29: m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a); break;
    case 0x09: m_a = static_cast<uint8_t>(m_a | value); setZeroNeg(m_a); break;
    case 0x49: m_a = static_cast<uint8_t>(m_a ^ value); setZeroNeg(m_a); break;
    case 0x69: {
        const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x01, sum > 0xFF);
        setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a); break;
    }
    case 0xE9: case 0xEB: {
        const uint8_t inv = static_cast<uint8_t>(value ^ 0xFF);
        const uint16_t sum = static_cast<uint16_t>(m_a) + inv + (getFlag(0x01) ? 1 : 0);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x01, sum > 0xFF);
        setFlag(0x40, ((m_a ^ result) & (inv ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a); break;
    }
    case 0xC9: cmpHelper(m_a, value); break;
    case 0xE0: cmpHelper(m_x, value); break;
    case 0xC0: cmpHelper(m_y, value); break;
    case 0x0B: case 0x2B:
        m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a); setFlag(0x01, (m_a & 0x80) != 0); break;
    case 0x4B:
        m_a = static_cast<uint8_t>(m_a & value); setFlag(0x01, (m_a & 0x01) != 0); m_a >>= 1; setZeroNeg(m_a); break;
    case 0x6B: {
        m_a = static_cast<uint8_t>(m_a & value);
        const uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
        m_a = static_cast<uint8_t>((m_a >> 1) | carryIn);
        setFlag(0x01, (m_a & 0x40) != 0);
        setFlag(0x40, (((m_a >> 6) ^ (m_a >> 5)) & 1) != 0);
        setZeroNeg(m_a); break;
    }
    case 0xCB: {
        const uint8_t tmp = static_cast<uint8_t>(m_a & m_x);
        setFlag(0x01, tmp >= value); m_x = static_cast<uint8_t>(tmp - value); setZeroNeg(m_x); break;
    }
    case 0xAB: m_a = m_x = value; setZeroNeg(value); break;
    case 0x8B: m_a = static_cast<uint8_t>((m_a | 0xEE) & m_x & value); setZeroNeg(m_a); break;
    // Immediate NOPs $80/$82/$89/$C2/$E2 consume the byte only.
    default: break;
    }
}

void CPU::clockImmediateSequence()
{
    if (m_cycles != 1) return;
    const uint8_t value = read(m_pc++);
    applyImmediate(value);
    m_pendingIoOp = PendingIoOp::None;
}

void CPU::clockJmpSequence()
{
    if (m_pendingIoOp == PendingIoOp::JmpAbs) {
        if (m_cycles == 2) { m_pendingIoAddr = read(m_pc++); return; }
        if (m_cycles == 1) {
            const uint8_t hi = read(m_pc++);
            m_pc = static_cast<uint16_t>(m_pendingIoAddr & 0x00FF) | (static_cast<uint16_t>(hi) << 8);
            m_pendingIoOp = PendingIoOp::None;
        }
        return;
    }

    if (m_cycles == 4) { m_pendingIoAddr = read(m_pc++); return; }
    if (m_cycles == 3) {
        const uint8_t hi = read(m_pc++);
        m_pendingIoAddr = static_cast<uint16_t>(m_pendingIoAddr & 0x00FF) | (static_cast<uint16_t>(hi) << 8);
        return;
    }
    if (m_cycles == 2) { m_pendingIoData = read(m_pendingIoAddr); return; }
    if (m_cycles == 1) {
        const uint16_t hiAddr = static_cast<uint16_t>((m_pendingIoAddr & 0xFF00) | ((m_pendingIoAddr + 1) & 0x00FF));
        const uint8_t hi = read(hiAddr);
        m_pc = static_cast<uint16_t>(m_pendingIoData) | (static_cast<uint16_t>(hi) << 8);
        m_pendingIoOp = PendingIoOp::None;
    }
}

void CPU::clockBranchSequence()
{
    // Cycle 2 always fetches the signed relative offset. A not-taken branch
    // ends here. For a taken branch, keep the sequential PC until the extra
    // discarded bus cycle(s) have completed.
    if (m_cycles == 1 && m_pendingIoData2 == 0) {
        const int8_t offset = static_cast<int8_t>(read(m_pc++));
        m_pendingIoData = static_cast<uint8_t>(offset);
        const uint16_t sequentialPc = m_pc;
        const uint16_t target = static_cast<uint16_t>(sequentialPc + offset);

        bool take = false;
        switch (m_currentOpcode) {
        case 0xF0: take = getFlag(0x02); break;
        case 0xD0: take = !getFlag(0x02); break;
        case 0x30: take = getFlag(0x80); break;
        case 0x10: take = !getFlag(0x80); break;
        case 0x90: take = !getFlag(0x01); break;
        case 0xB0: take = getFlag(0x01); break;
        case 0x50: take = !getFlag(0x40); break;
        case 0x70: take = getFlag(0x40); break;
        default: break;
        }

        if (!take) {
            m_pendingIoOp = PendingIoOp::None;
            return;
        }

        m_pendingIoAddr = target;
        m_branchPageCrossed = (sequentialPc & 0xFF00) != (target & 0xFF00);
        m_pendingIoData2 = 1;
        // We are currently on cycle 2. After CPU::clock() decrements m_cycles,
        // leave one extra slot for an ordinary taken branch and two for a
        // page-crossing branch.
        m_cycles = m_branchPageCrossed ? 3 : 2;
        return;
    }

    // First taken-branch extra cycle: discarded read from the sequential PC.
    if ((!m_branchPageCrossed && m_cycles == 1) ||
        (m_branchPageCrossed && m_cycles == 2)) {
        (void)read(m_pc);
        if (!m_branchPageCrossed) {
            m_pc = m_pendingIoAddr;
            m_pendingIoOp = PendingIoOp::None;
        }
        return;
    }

    // Page-crossing final penalty cycle reads from the target low byte while
    // still driving the old page high byte, then commits the corrected PC.
    if (m_branchPageCrossed && m_cycles == 1) {
        const uint16_t provisional = static_cast<uint16_t>((m_pc & 0xFF00) |
                                                           (m_pendingIoAddr & 0x00FF));
        (void)read(provisional);
        m_pc = m_pendingIoAddr;
        m_pendingIoOp = PendingIoOp::None;
    }
}

bool CPU::isAbsIndexedRmwSequence() const
{
    return m_pendingIoOp == PendingIoOp::AbsXRmw ||
           m_pendingIoOp == PendingIoOp::AbsYRmw;
}

void CPU::applyIndexedRmw(uint8_t oldValue, uint8_t& value)
{
    value = oldValue;
    switch (m_currentOpcode) {
    // ASL / SLO
    case 0x06: case 0x0E: case 0x07: case 0x0F: case 0x1E: case 0x1F: case 0x1B: case 0x03: case 0x13:
        setFlag(0x01, value & 0x80);
        value = static_cast<uint8_t>(value << 1);
        if (m_currentOpcode == 0x07 || m_currentOpcode == 0x0F || m_currentOpcode == 0x1F || m_currentOpcode == 0x1B || m_currentOpcode == 0x03 || m_currentOpcode == 0x13) {
            m_a = static_cast<uint8_t>(m_a | value); setZeroNeg(m_a);
        } else setZeroNeg(value);
        break;

    // ROL / RLA
    case 0x26: case 0x2E: case 0x27: case 0x2F: case 0x3E: case 0x3F: case 0x3B: case 0x23: case 0x33: {
        const uint8_t carryIn = getFlag(0x01) ? 1 : 0;
        setFlag(0x01, value & 0x80);
        value = static_cast<uint8_t>((value << 1) | carryIn);
        if (m_currentOpcode == 0x27 || m_currentOpcode == 0x2F || m_currentOpcode == 0x3F || m_currentOpcode == 0x3B || m_currentOpcode == 0x23 || m_currentOpcode == 0x33) {
            m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a);
        } else setZeroNeg(value);
        break;
    }

    // LSR / SRE
    case 0x46: case 0x4E: case 0x47: case 0x4F: case 0x5E: case 0x5F: case 0x5B: case 0x43: case 0x53:
        setFlag(0x01, value & 0x01);
        value = static_cast<uint8_t>(value >> 1);
        if (m_currentOpcode == 0x47 || m_currentOpcode == 0x4F || m_currentOpcode == 0x5F || m_currentOpcode == 0x5B || m_currentOpcode == 0x43 || m_currentOpcode == 0x53) {
            m_a = static_cast<uint8_t>(m_a ^ value); setZeroNeg(m_a);
        } else setZeroNeg(value);
        break;

    // ROR / RRA
    case 0x66: case 0x6E: case 0x67: case 0x6F: case 0x7E: case 0x7F: case 0x7B: case 0x63: case 0x73: {
        const uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
        setFlag(0x01, value & 0x01);
        value = static_cast<uint8_t>((value >> 1) | carryIn);
        if (m_currentOpcode == 0x67 || m_currentOpcode == 0x6F || m_currentOpcode == 0x7F || m_currentOpcode == 0x7B || m_currentOpcode == 0x63 || m_currentOpcode == 0x73) {
            const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
            setFlag(0x01, sum > 0xFF);
            const uint8_t result = static_cast<uint8_t>(sum);
            setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80) != 0);
            m_a = result; setZeroNeg(m_a);
        } else setZeroNeg(value);
        break;
    }

    // INC / ISC
    case 0xE6: case 0xEE: case 0xE7: case 0xEF: case 0xFE: case 0xFF: case 0xFB: case 0xE3: case 0xF3:
        value = static_cast<uint8_t>(value + 1);
        if (m_currentOpcode == 0xE7 || m_currentOpcode == 0xEF || m_currentOpcode == 0xFF || m_currentOpcode == 0xFB || m_currentOpcode == 0xE3 || m_currentOpcode == 0xF3) {
            const uint8_t inv = static_cast<uint8_t>(value ^ 0xFF);
            const uint16_t sum = static_cast<uint16_t>(m_a) + inv + (getFlag(0x01) ? 1 : 0);
            setFlag(0x01, sum > 0xFF);
            const uint8_t result = static_cast<uint8_t>(sum);
            setFlag(0x40, ((m_a ^ result) & (inv ^ result) & 0x80) != 0);
            m_a = result; setZeroNeg(m_a);
        } else setZeroNeg(value);
        break;

    // DEC / DCP
    case 0xC6: case 0xCE: case 0xC7: case 0xCF: case 0xDE: case 0xDF: case 0xDB: case 0xC3: case 0xD3:
        value = static_cast<uint8_t>(value - 1);
        if (m_currentOpcode == 0xC7 || m_currentOpcode == 0xCF || m_currentOpcode == 0xDF || m_currentOpcode == 0xDB || m_currentOpcode == 0xC3 || m_currentOpcode == 0xD3)
            cmpHelper(m_a, value);
        else
            setZeroNeg(value);
        break;

    default:
        break;
    }
}

void CPU::clockAbsIndexedRmwSequence()
{
    const uint8_t index = m_pendingIoOp == PendingIoOp::AbsXRmw ? m_x : m_y;

    if (m_cycles == 6) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }
    if (m_cycles == 5) {
        const uint8_t hi = read(m_pc++);
        m_pendingIoAddr = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) |
                                                (m_pendingIoAddr & 0x00FF));
        return;
    }

    const uint16_t base = m_pendingIoAddr;
    const uint16_t effective = static_cast<uint16_t>(base + index);
    if (m_cycles == 4) {
        const uint16_t provisional = static_cast<uint16_t>((base & 0xFF00) |
                                                           (effective & 0x00FF));
        (void)read(provisional);
        return;
    }
    if (m_cycles == 3) {
        const uint8_t oldValue = read(effective);
        uint8_t newValue = oldValue;
        applyIndexedRmw(oldValue, newValue);
        m_pendingIoData = oldValue;
        m_pendingIoData2 = newValue;
        return;
    }
    if (m_cycles == 2) {
        write(effective, m_pendingIoData);
        return;
    }
    if (m_cycles == 1) {
        write(effective, m_pendingIoData2);
        m_pendingIoOp = PendingIoOp::None;
    }
}

bool CPU::isZpIndexedRmwSequence() const
{
    return m_pendingIoOp == PendingIoOp::ZpXRmw;
}

void CPU::clockZpIndexedRmwSequence()
{
    if (m_cycles == 5) {
        m_pendingIoAddr = read(m_pc++);
        return;
    }

    const uint8_t base = static_cast<uint8_t>(m_pendingIoAddr);
    if (m_cycles == 4) {
        (void)read(base);
        return;
    }

    const uint16_t addr = static_cast<uint8_t>(base + m_x);
    if (m_cycles == 3) {
        const uint8_t oldValue = read(addr);
        uint8_t value = oldValue;

        switch (m_currentOpcode) {
        case 0x16: // ASL zp,X
        case 0x17: // SLO zp,X
            setFlag(0x01, value & 0x80);
            value = static_cast<uint8_t>(value << 1);
            if (m_currentOpcode == 0x17) { m_a = static_cast<uint8_t>(m_a | value); setZeroNeg(m_a); }
            else setZeroNeg(value);
            break;
        case 0x36: // ROL zp,X
        case 0x37: { // RLA zp,X
            const uint8_t carryIn = getFlag(0x01) ? 1 : 0;
            setFlag(0x01, value & 0x80);
            value = static_cast<uint8_t>((value << 1) | carryIn);
            if (m_currentOpcode == 0x37) { m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a); }
            else setZeroNeg(value);
            break;
        }
        case 0x56: // LSR zp,X
        case 0x57: // SRE zp,X
            setFlag(0x01, value & 0x01);
            value = static_cast<uint8_t>(value >> 1);
            if (m_currentOpcode == 0x57) { m_a = static_cast<uint8_t>(m_a ^ value); setZeroNeg(m_a); }
            else setZeroNeg(value);
            break;
        case 0x76: // ROR zp,X
        case 0x77: { // RRA zp,X
            const uint8_t carryIn = getFlag(0x01) ? 0x80 : 0x00;
            setFlag(0x01, value & 0x01);
            value = static_cast<uint8_t>((value >> 1) | carryIn);
            if (m_currentOpcode == 0x77) {
                const uint8_t operand = value;
                const uint16_t sum = static_cast<uint16_t>(m_a) + operand + (getFlag(0x01) ? 1 : 0);
                setFlag(0x01, sum > 0xFF);
                const uint8_t result = static_cast<uint8_t>(sum);
                setFlag(0x40, (~(m_a ^ operand) & (m_a ^ result) & 0x80) != 0);
                m_a = result;
                setZeroNeg(m_a);
            } else setZeroNeg(value);
            break;
        }
        case 0xD6: // DEC zp,X
        case 0xD7: // DCP zp,X
            value = static_cast<uint8_t>(value - 1);
            if (m_currentOpcode == 0xD7) cmpHelper(m_a, value);
            else setZeroNeg(value);
            break;
        case 0xF6: // INC zp,X
        case 0xF7: // ISC zp,X
            value = static_cast<uint8_t>(value + 1);
            if (m_currentOpcode == 0xF7) {
                const uint8_t operand = static_cast<uint8_t>(value ^ 0xFF);
                const uint16_t sum = static_cast<uint16_t>(m_a) + operand + (getFlag(0x01) ? 1 : 0);
                setFlag(0x01, sum > 0xFF);
                const uint8_t result = static_cast<uint8_t>(sum);
                setFlag(0x40, ((m_a ^ result) & (operand ^ result) & 0x80) != 0);
                m_a = result;
                setZeroNeg(m_a);
            } else setZeroNeg(value);
            break;
        default:
            break;
        }

        m_pendingIoData = oldValue;
        m_pendingIoData2 = value;
        return;
    }

    if (m_cycles == 2) {
        write(addr, m_pendingIoData);
        return;
    }

    if (m_cycles == 1) {
        write(addr, m_pendingIoData2);
        m_pendingIoOp = PendingIoOp::None;
    }
}

bool CPU::isIndirectSequence() const
{
    switch (m_pendingIoOp) {
    case PendingIoOp::IndXRead:
    case PendingIoOp::IndXStore:
    case PendingIoOp::IndXRmw:
    case PendingIoOp::IndYRead:
    case PendingIoOp::IndYStore:
    case PendingIoOp::IndYHighStore:
    case PendingIoOp::IndYRmw:
        return true;
    default:
        return false;
    }
}

void CPU::applyIndirectRead(uint8_t value)
{
    switch (m_currentOpcode) {
    case 0xA1: case 0xB1: // LDA
        m_a = value; setZeroNeg(m_a); break;
    case 0x21: case 0x31: // AND
        m_a = static_cast<uint8_t>(m_a & value); setZeroNeg(m_a); break;
    case 0x01: case 0x11: // ORA
        m_a = static_cast<uint8_t>(m_a | value); setZeroNeg(m_a); break;
    case 0x41: case 0x51: // EOR
        m_a = static_cast<uint8_t>(m_a ^ value); setZeroNeg(m_a); break;
    case 0x61: case 0x71: { // ADC
        const uint16_t sum = static_cast<uint16_t>(m_a) + value + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, (~(m_a ^ value) & (m_a ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a);
        break;
    }
    case 0xE1: case 0xF1: { // SBC
        const uint8_t inv = static_cast<uint8_t>(value ^ 0xFF);
        const uint16_t sum = static_cast<uint16_t>(m_a) + inv + (getFlag(0x01) ? 1 : 0);
        setFlag(0x01, sum > 0xFF);
        const uint8_t result = static_cast<uint8_t>(sum);
        setFlag(0x40, ((m_a ^ result) & (inv ^ result) & 0x80) != 0);
        m_a = result; setZeroNeg(m_a);
        break;
    }
    case 0xC1: case 0xD1: // CMP
        cmpHelper(m_a, value); break;
    case 0xA3: case 0xB3: // LAX
        m_a = m_x = value; setZeroNeg(value); break;
    default:
        break;
    }
}

void CPU::clockIndirectSequence()
{
    const PendingIoOp op = m_pendingIoOp;
    const bool indX = op == PendingIoOp::IndXRead || op == PendingIoOp::IndXStore || op == PendingIoOp::IndXRmw;
    const bool rmw = op == PendingIoOp::IndXRmw || op == PendingIoOp::IndYRmw;
    const bool store = op == PendingIoOp::IndXStore || op == PendingIoOp::IndYStore || op == PendingIoOp::IndYHighStore;

    if (indX) {
        // ($nn,X): operand, unindexed dummy, indexed pointer low/high, data.
        if (m_cycles == (rmw ? 7 : 5)) {
            m_pendingIoAddr = read(m_pc++); // zero-page operand
            return;
        }
        if (m_cycles == (rmw ? 6 : 4)) {
            (void)read(static_cast<uint8_t>(m_pendingIoAddr));
            return;
        }
        if (m_cycles == (rmw ? 5 : 3)) {
            const uint8_t ptr = static_cast<uint8_t>(m_pendingIoAddr + m_x);
            m_pendingIoData = read(ptr); // pointer low
            return;
        }
        if (m_cycles == (rmw ? 4 : 2)) {
            const uint8_t ptr = static_cast<uint8_t>(m_pendingIoAddr + m_x);
            const uint8_t hi = read(static_cast<uint8_t>(ptr + 1));
            m_pendingIoAddr = static_cast<uint16_t>(m_pendingIoData) |
                              (static_cast<uint16_t>(hi) << 8);
            return;
        }
        if (rmw && m_cycles == 3) {
            const uint8_t oldValue = read(m_pendingIoAddr);
            uint8_t newValue = oldValue;
            applyIndexedRmw(oldValue, newValue);
            m_pendingIoData = oldValue;
            m_pendingIoData2 = newValue;
            return;
        }
        if (rmw && m_cycles == 2) {
            write(m_pendingIoAddr, m_pendingIoData);
            return;
        }
        if (rmw && m_cycles == 1) {
            write(m_pendingIoAddr, m_pendingIoData2);
            m_pendingIoOp = PendingIoOp::None;
            return;
        }
        if (m_cycles == 1) {
            if (store) {
                const uint8_t data = m_currentOpcode == 0x83
                    ? static_cast<uint8_t>(m_a & m_x) : m_a;
                write(m_pendingIoAddr, data);
            } else {
                applyIndirectRead(read(m_pendingIoAddr));
            }
            m_pendingIoOp = PendingIoOp::None;
        }
        return;
    }

    // ($nn),Y: operand, pointer low/high, provisional read, corrected access.
    if (m_cycles == (rmw ? 7 : (store ? 5 : 4))) {
        m_pendingIoAddr = read(m_pc++); // zero-page pointer address
        return;
    }
    if (m_cycles == (rmw ? 6 : (store ? 4 : 3))) {
        m_pendingIoData = read(static_cast<uint8_t>(m_pendingIoAddr));
        return;
    }
    if (m_cycles == (rmw ? 5 : (store ? 3 : 2)) && (store || rmw || m_pendingIoData2 == 0)) {
        const uint8_t zp = static_cast<uint8_t>(m_pendingIoAddr);
        const uint8_t hi = read(static_cast<uint8_t>(zp + 1));
        const uint16_t base = static_cast<uint16_t>(m_pendingIoData) |
                              (static_cast<uint16_t>(hi) << 8);
        m_pendingIoAddr = base;
        if (!store && !rmw) {
            m_pendingIoData2 = 1;
            const uint16_t effective = static_cast<uint16_t>(base + m_y);
            if ((base & 0xFF00) != (effective & 0xFF00))
                ++m_cycles;
        }
        return;
    }

    const uint16_t base = m_pendingIoAddr;
    const uint16_t effective = static_cast<uint16_t>(base + m_y);
    const uint16_t provisional = static_cast<uint16_t>((base & 0xFF00) | (effective & 0x00FF));

    if (rmw && m_cycles == 4) {
        (void)read(provisional);
        return;
    }
    if (rmw && m_cycles == 3) {
        const uint8_t oldValue = read(effective);
        uint8_t newValue = oldValue;
        applyIndexedRmw(oldValue, newValue);
        m_pendingIoData = oldValue;
        m_pendingIoData2 = newValue;
        return;
    }
    if (rmw && m_cycles == 2) {
        write(effective, m_pendingIoData);
        return;
    }
    if (rmw && m_cycles == 1) {
        write(effective, m_pendingIoData2);
        m_pendingIoOp = PendingIoOp::None;
        return;
    }

    if (store && m_cycles == 2) {
        (void)read(provisional);
        return;
    }
    if (store && m_cycles == 1) {
        if (op == PendingIoOp::IndYHighStore) {
            uint8_t stored = static_cast<uint8_t>(m_a & m_x);
            uint16_t target = effective;
            if (!m_unstableHighStoreRdy) {
                const uint8_t hPlusOne = static_cast<uint8_t>((base >> 8) + 1);
                stored = static_cast<uint8_t>(stored & hPlusOne);
                if ((base & 0xFF00) != (effective & 0xFF00))
                    target = static_cast<uint16_t>((static_cast<uint16_t>(stored) << 8) |
                                                   (effective & 0x00FF));
            }
            write(target, stored);
            m_unstableHighStoreRdy = false;
        } else {
            write(effective, m_a);
        }
        m_pendingIoOp = PendingIoOp::None;
        return;
    }

    if (!store && m_cycles == 2) {
        // Crossing read: provisional address is externally visible and discarded.
        (void)read(provisional);
        return;
    }
    if (!store && m_cycles == 1) {
        applyIndirectRead(read(effective));
        m_pendingIoOp = PendingIoOp::None;
    }
}

void CPU::clockResetSequence()
{
    switch (m_cycles) {
    case 7:
    case 6:
        (void)read(m_pc);
        break;
    case 5:
    case 4:
    case 3:
        (void)read(static_cast<uint16_t>(0x0100 | m_sp));
        --m_sp;
        break;
    case 2:
        m_pendingIoData = read(0xFFFC);
        break;
    case 1: {
        const uint8_t hi = read(0xFFFD);
        m_pc = static_cast<uint16_t>(m_pendingIoData) |
            (static_cast<uint16_t>(hi) << 8);
        m_pendingIoOp = PendingIoOp::None;
        break;
    }
    default:
        break;
    }
}

void CPU::clockStackSequence()
{
    const PendingIoOp op = m_pendingIoOp;

    switch (op) {
    case PendingIoOp::StackPha:
    case PendingIoOp::StackPhp:
        if (m_cycles == 2) {
            (void)read(m_pc);
        }
        else if (m_cycles == 1) {
            const uint8_t value = op == PendingIoOp::StackPha
                ? m_a : static_cast<uint8_t>(m_status | 0x30);
            push(value);
            m_pendingIoOp = PendingIoOp::None;
        }
        break;

    case PendingIoOp::StackPla:
    case PendingIoOp::StackPlp:
        if (m_cycles == 3) {
            (void)read(m_pc);
        }
        else if (m_cycles == 2) {
            (void)read(static_cast<uint16_t>(0x0100 | m_sp));
        }
        else if (m_cycles == 1) {
            const uint8_t value = pull();
            if (op == PendingIoOp::StackPla) {
                m_a = value;
                setZeroNeg(m_a);
            }
            else {
                m_status = static_cast<uint8_t>((value & 0xEF) | 0x20);
            }
            m_pendingIoOp = PendingIoOp::None;
        }
        break;

    case PendingIoOp::StackJsr:
        switch (m_cycles) {
        case 5:
            m_pendingIoData = read(m_pc++); // target low byte
            break;
        case 4:
            (void)read(static_cast<uint16_t>(0x0100 | m_sp));
            break;
        case 3:
            push(static_cast<uint8_t>(m_pc >> 8));
            break;
        case 2:
            push(static_cast<uint8_t>(m_pc & 0xFF));
            break;
        case 1: {
            const uint8_t hi = read(m_pc);
            m_pc = static_cast<uint16_t>(m_pendingIoData) |
                (static_cast<uint16_t>(hi) << 8);
            m_pendingIoOp = PendingIoOp::None;
            break;
        }
        default:
            break;
        }
        break;

    case PendingIoOp::StackRts:
        switch (m_cycles) {
        case 5:
            (void)read(m_pc);
            break;
        case 4:
            (void)read(static_cast<uint16_t>(0x0100 | m_sp));
            break;
        case 3:
            m_pendingIoData = pull();
            break;
        case 2: {
            const uint8_t hi = pull();
            m_pendingIoAddr = static_cast<uint16_t>(m_pendingIoData) |
                (static_cast<uint16_t>(hi) << 8);
            break;
        }
        case 1:
            (void)read(m_pendingIoAddr);
            m_pc = static_cast<uint16_t>(m_pendingIoAddr + 1);
            m_pendingIoOp = PendingIoOp::None;
            break;
        default:
            break;
        }
        break;

    case PendingIoOp::StackRti:
        switch (m_cycles) {
        case 5:
            (void)read(m_pc);
            break;
        case 4:
            (void)read(static_cast<uint16_t>(0x0100 | m_sp));
            break;
        case 3: {
            const uint8_t value = pull();
            m_status = static_cast<uint8_t>((value & 0xEF) | 0x20);
            break;
        }
        case 2:
            m_pendingIoData = pull();
            break;
        case 1: {
            const uint8_t hi = pull();
            m_pc = static_cast<uint16_t>(m_pendingIoData) |
                (static_cast<uint16_t>(hi) << 8);
            m_pendingIoOp = PendingIoOp::None;
            break;
        }
        default:
            break;
        }
        break;

    default:
        break;
    }
}

void CPU::pollNmi()
{
    if (m_nmiSampled) {
        m_nmiPolled = true;
        m_nmiSampled = false;
    }
}

void CPU::pollIrq()
{
    // CLI/SEI/PLP change I too late to affect their own IRQ poll. RTI is
    // intentionally excluded: its restored I flag is visible in time.
    bool irqDisabled = getFlag(0x04);
    if (m_currentOpcode == 0x58 || m_currentOpcode == 0x78 || m_currentOpcode == 0x28)
        irqDisabled = m_irqDisableBeforeInstruction;

    m_irqPolled = m_irqPolled || (m_irqLine && !irqDisabled);
}

void CPU::pollInterrupts()
{
    // Branches have their own polling points, where both asynchronous NMI and
    // the level-sensitive IRQ input are sampled together.
    pollNmi();
    pollIrq();
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

    // The unstable high-byte stores retain the unindexed base in the pending
    // I/O slot. They otherwise use the same store-addressing provisional read
    // as STA: old high byte plus the already-indexed low byte.
    case 0x9C: // SHY/SYA abs,X
    case 0x9E: // SHX/SXA abs,Y
    case 0x9F: // SHA/AHX abs,Y
    case 0x9B: // TAS/SHS abs,Y
    case 0x93: // SHA/AHX (zp),Y
        if (m_pendingIoOp != PendingIoOp::UnstableHighStore)
            return false;
        base = m_pendingIoAddr;
        always = true;
        break;

    default:
        return false;
    }

    if (!always && (base & 0xFF00) == (m_pendingIoAddr & 0xFF00))
        return false;

    uint16_t effective = m_pendingIoAddr;
    switch (m_currentOpcode) {
    case 0x9C:
        effective = static_cast<uint16_t>(base + m_x);
        break;
    case 0x9E:
    case 0x9F:
    case 0x9B:
    case 0x93:
        effective = static_cast<uint16_t>(base + m_y);
        break;
    default:
        break;
    }

    addr = static_cast<uint16_t>((base & 0xFF00) | (effective & 0x00FF));
    return true;
}

CPU::BusCycle CPU::nextBusCycle() const
{
    BusCycle cycle{};
    cycle.type = BusCycleType::Read;
    cycle.address = m_pc;
    cycle.data = 0;
    cycle.dummy = false;
    cycle.exact = false;

    // At an instruction boundary the CPU will either begin IRQ/NMI entry with
    // a discarded PC read or fetch the next opcode. Both place PC on the bus.
    if (m_cycles == 0) {
        cycle.address = m_pc;
        cycle.exact = true;
        return cycle;
    }

    if (isInterruptEntry()) {
        const bool brkSequence =
            m_pendingIoOp == PendingIoOp::InterruptBrkIrq ||
            m_pendingIoOp == PendingIoOp::InterruptBrkNmi;
        const bool useNmiVector =
            m_pendingIoOp == PendingIoOp::InterruptBrkNmi ||
            m_pendingIoOp == PendingIoOp::InterruptIrqNmi ||
            m_pendingIoOp == PendingIoOp::InterruptNmi;
        const uint16_t vector = useNmiVector ? 0xFFFA : 0xFFFE;

        if (brkSequence && m_cycles == 6) {
            cycle.address = static_cast<uint16_t>(m_pc - 1);
            cycle.dummy = true;
            cycle.exact = true;
            return cycle;
        }
        if (!brkSequence && (m_cycles == 7 || m_cycles == 6)) {
            cycle.address = m_pc;
            cycle.dummy = true;
            cycle.exact = true;
            return cycle;
        }
        if (m_cycles >= 3 && m_cycles <= 5) {
            cycle.type = BusCycleType::Write;
            cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
            if (m_cycles == 5)
                cycle.data = static_cast<uint8_t>(m_pc >> 8);
            else if (m_cycles == 4)
                cycle.data = static_cast<uint8_t>(m_pc & 0xFF);
            else if (brkSequence)
                cycle.data = static_cast<uint8_t>(m_status | 0x30);
            else
                cycle.data = static_cast<uint8_t>((m_status & ~0x10) | 0x20);
            cycle.exact = true;
            return cycle;
        }
        if (m_cycles == 2 || m_cycles == 1) {
            cycle.address = static_cast<uint16_t>(vector + (m_cycles == 1 ? 1 : 0));
            cycle.exact = true;
            return cycle;
        }
        return cycle;
    }

    if (isResetSequence()) {
        cycle.exact = true;
        cycle.dummy = true;
        if (m_cycles >= 6) {
            cycle.address = m_pc;
        }
        else if (m_cycles >= 3) {
            cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
        }
        else if (m_cycles == 2) {
            cycle.address = 0xFFFC;
            cycle.dummy = false;
        }
        else if (m_cycles == 1) {
            cycle.address = 0xFFFD;
            cycle.dummy = false;
        }
        return cycle;
    }

    if (isStackSequence()) {
        cycle.exact = true;
        switch (m_pendingIoOp) {
        case PendingIoOp::StackPha:
        case PendingIoOp::StackPhp:
            if (m_cycles == 2) {
                cycle.address = m_pc;
                cycle.dummy = true;
            }
            else if (m_cycles == 1) {
                cycle.type = BusCycleType::Write;
                cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
                cycle.data = m_pendingIoOp == PendingIoOp::StackPha
                    ? m_a : static_cast<uint8_t>(m_status | 0x30);
            }
            return cycle;

        case PendingIoOp::StackPla:
        case PendingIoOp::StackPlp:
            if (m_cycles == 3) {
                cycle.address = m_pc;
                cycle.dummy = true;
            }
            else if (m_cycles == 2) {
                cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
                cycle.dummy = true;
            }
            else if (m_cycles == 1) {
                cycle.address = static_cast<uint16_t>(0x0100 | static_cast<uint8_t>(m_sp + 1));
            }
            return cycle;

        case PendingIoOp::StackJsr:
            if (m_cycles == 5) {
                cycle.address = m_pc;
            }
            else if (m_cycles == 4) {
                cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
                cycle.dummy = true;
            }
            else if (m_cycles == 3 || m_cycles == 2) {
                cycle.type = BusCycleType::Write;
                cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
                cycle.data = m_cycles == 3
                    ? static_cast<uint8_t>(m_pc >> 8)
                    : static_cast<uint8_t>(m_pc & 0xFF);
            }
            else if (m_cycles == 1) {
                cycle.address = m_pc;
            }
            return cycle;

        case PendingIoOp::StackRts:
            if (m_cycles == 5) {
                cycle.address = m_pc;
                cycle.dummy = true;
            }
            else if (m_cycles == 4) {
                cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
                cycle.dummy = true;
            }
            else if (m_cycles == 3 || m_cycles == 2) {
                cycle.address = static_cast<uint16_t>(0x0100 | static_cast<uint8_t>(m_sp + 1));
            }
            else if (m_cycles == 1) {
                cycle.address = m_pendingIoAddr;
                cycle.dummy = true;
            }
            return cycle;

        case PendingIoOp::StackRti:
            if (m_cycles == 5) {
                cycle.address = m_pc;
                cycle.dummy = true;
            }
            else if (m_cycles == 4) {
                cycle.address = static_cast<uint16_t>(0x0100 | m_sp);
                cycle.dummy = true;
            }
            else if (m_cycles >= 1 && m_cycles <= 3) {
                cycle.address = static_cast<uint16_t>(0x0100 | static_cast<uint8_t>(m_sp + 1));
            }
            return cycle;

        default:
            break;
        }
    }

    if (isIndirectSequence()) {
        cycle.exact = true;
        const PendingIoOp op = m_pendingIoOp;
        const bool indX = op == PendingIoOp::IndXRead || op == PendingIoOp::IndXStore || op == PendingIoOp::IndXRmw;
        const bool rmw = op == PendingIoOp::IndXRmw || op == PendingIoOp::IndYRmw;
        const bool store = op == PendingIoOp::IndXStore || op == PendingIoOp::IndYStore || op == PendingIoOp::IndYHighStore;

        if (indX) {
            const int operandCycle = rmw ? 7 : 5;
            if (m_cycles == operandCycle) { cycle.address = m_pc; return cycle; }
            if (m_cycles == operandCycle - 1) {
                cycle.address = static_cast<uint8_t>(m_pendingIoAddr); cycle.dummy = true; return cycle;
            }
            if (m_cycles == operandCycle - 2) {
                cycle.address = static_cast<uint8_t>(m_pendingIoAddr + m_x); return cycle;
            }
            if (m_cycles == operandCycle - 3) {
                cycle.address = static_cast<uint8_t>(m_pendingIoAddr + m_x + 1); return cycle;
            }
            if (rmw && m_cycles == 3) { cycle.address = m_pendingIoAddr; return cycle; }
            if (rmw && (m_cycles == 2 || m_cycles == 1)) {
                cycle.type = BusCycleType::Write; cycle.address = m_pendingIoAddr;
                cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2; return cycle;
            }
            if (m_cycles == 1) {
                cycle.address = m_pendingIoAddr;
                if (store) {
                    cycle.type = BusCycleType::Write;
                    cycle.data = m_currentOpcode == 0x83 ? static_cast<uint8_t>(m_a & m_x) : m_a;
                }
                return cycle;
            }
            return cycle;
        }

        const int operandCycle = rmw ? 7 : (store ? 5 : 4);
        if (m_cycles == operandCycle) { cycle.address = m_pc; return cycle; }
        if (m_cycles == operandCycle - 1) { cycle.address = static_cast<uint8_t>(m_pendingIoAddr); return cycle; }
        if (m_cycles == operandCycle - 2 && (store || rmw || m_pendingIoData2 == 0)) {
            cycle.address = static_cast<uint8_t>(m_pendingIoAddr + 1); return cycle;
        }

        const uint16_t base = m_pendingIoAddr;
        const uint16_t effective = static_cast<uint16_t>(base + m_y);
        const uint16_t provisional = static_cast<uint16_t>((base & 0xFF00) | (effective & 0x00FF));
        if (rmw && m_cycles == 4) { cycle.address = provisional; cycle.dummy = true; return cycle; }
        if (rmw && m_cycles == 3) { cycle.address = effective; return cycle; }
        if (rmw && (m_cycles == 2 || m_cycles == 1)) {
            cycle.type = BusCycleType::Write; cycle.address = effective;
            cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2; return cycle;
        }
        if (store && m_cycles == 2) { cycle.address = provisional; cycle.dummy = true; return cycle; }
        if (store && m_cycles == 1) {
            cycle.type = BusCycleType::Write;
            if (op == PendingIoOp::IndYHighStore) {
                uint8_t stored = static_cast<uint8_t>(m_a & m_x);
                uint16_t target = effective;
                if (!m_unstableHighStoreRdy) {
                    const uint8_t hPlusOne = static_cast<uint8_t>((base >> 8) + 1);
                    stored = static_cast<uint8_t>(stored & hPlusOne);
                    if ((base & 0xFF00) != (effective & 0xFF00))
                        target = static_cast<uint16_t>((static_cast<uint16_t>(stored) << 8) | (effective & 0x00FF));
                }
                cycle.address = target; cycle.data = stored;
            } else { cycle.address = effective; cycle.data = m_a; }
            return cycle;
        }
        if (!store && m_cycles == 2) { cycle.address = provisional; cycle.dummy = true; return cycle; }
        if (!store && m_cycles == 1) { cycle.address = effective; return cycle; }
        return cycle;
    }

    if (isAbsIndexedReadSequence()) {
        cycle.exact = true;
        const uint8_t index = m_pendingIoOp == PendingIoOp::AbsXRead ? m_x : m_y;
        if (m_cycles == 3) {
            cycle.address = m_pc;
            return cycle;
        }
        if (m_cycles == 2 && m_pendingIoData == 0) {
            cycle.address = m_pc;
            return cycle;
        }
        const uint16_t base = m_pendingIoAddr;
        const uint16_t effective = static_cast<uint16_t>(base + index);
        if (m_cycles == 2) {
            cycle.address = static_cast<uint16_t>((base & 0xFF00) | (effective & 0x00FF));
            cycle.dummy = true;
            return cycle;
        }
        if (m_cycles == 1) {
            cycle.address = effective;
            return cycle;
        }
        return cycle;
    }

    if (isAbsIndexedStoreSequence()) {
        cycle.exact = true;
        const bool xIndexed = m_pendingIoOp == PendingIoOp::AbsXStore ||
                              m_pendingIoOp == PendingIoOp::AbsXHighStore;
        const bool unstable = m_pendingIoOp == PendingIoOp::AbsXHighStore ||
                              m_pendingIoOp == PendingIoOp::AbsYHighStore;
        const uint8_t index = xIndexed ? m_x : m_y;
        if (m_cycles == 4 || m_cycles == 3) {
            cycle.address = m_pc;
            return cycle;
        }
        const uint16_t base = m_pendingIoAddr;
        const uint16_t effective = static_cast<uint16_t>(base + index);
        if (m_cycles == 2) {
            cycle.address = static_cast<uint16_t>((base & 0xFF00) |
                                                  (effective & 0x00FF));
            cycle.dummy = true;
            return cycle;
        }
        if (m_cycles == 1) {
            cycle.type = BusCycleType::Write;
            cycle.address = effective;
            cycle.data = m_a;
            if (unstable) {
                uint8_t source = 0;
                switch (m_currentOpcode) {
                case 0x9C: source = m_y; break;
                case 0x9E: source = m_x; break;
                case 0x9F: source = static_cast<uint8_t>(m_a & m_x); break;
                case 0x9B: source = m_sp; break;
                default: break;
                }
                uint8_t stored = source;
                if (!m_unstableHighStoreRdy) {
                    stored = static_cast<uint8_t>(source &
                        static_cast<uint8_t>((base >> 8) + 1));
                    if ((base & 0xFF00) != (effective & 0xFF00))
                        cycle.address = static_cast<uint16_t>((static_cast<uint16_t>(stored) << 8) |
                                                              (effective & 0x00FF));
                }
                cycle.data = stored;
            }
            return cycle;
        }
        return cycle;
    }

    if (isAbsIndexedRmwSequence()) {
        cycle.exact = true;
        const uint8_t index = m_pendingIoOp == PendingIoOp::AbsXRmw ? m_x : m_y;
        if (m_cycles == 6 || m_cycles == 5) {
            cycle.address = m_pc;
            return cycle;
        }
        const uint16_t base = m_pendingIoAddr;
        const uint16_t effective = static_cast<uint16_t>(base + index);
        if (m_cycles == 4) {
            cycle.address = static_cast<uint16_t>((base & 0xFF00) |
                                                  (effective & 0x00FF));
            cycle.dummy = true;
            return cycle;
        }
        if (m_cycles == 3) {
            cycle.address = effective;
            return cycle;
        }
        if (m_cycles == 2 || m_cycles == 1) {
            cycle.type = BusCycleType::Write;
            cycle.address = effective;
            cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2;
            cycle.dummy = (m_cycles == 2);
            return cycle;
        }
        return cycle;
    }

    if (isZpIndexedRmwSequence()) {
        cycle.exact = true;
        if (m_cycles == 5) {
            cycle.address = m_pc;
            return cycle;
        }
        const uint8_t base = static_cast<uint8_t>(m_pendingIoAddr);
        if (m_cycles == 4) {
            cycle.address = base;
            cycle.dummy = true;
            return cycle;
        }
        const uint16_t addr = static_cast<uint8_t>(base + m_x);
        if (m_cycles == 3) {
            cycle.address = addr;
            return cycle;
        }
        if (m_cycles == 2 || m_cycles == 1) {
            cycle.type = BusCycleType::Write;
            cycle.address = addr;
            cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2;
            cycle.dummy = (m_cycles == 2);
            return cycle;
        }
        return cycle;
    }

    if (isZpIndexedSequence()) {
        cycle.exact = true;
        const PendingIoOp op = m_pendingIoOp;
        const uint8_t index = (op == PendingIoOp::ZpYLdx || op == PendingIoOp::ZpYStx ||
                               op == PendingIoOp::ZpYLax || op == PendingIoOp::ZpYSax)
            ? m_y : m_x;
        if (m_cycles == 3) {
            cycle.address = m_pc;
            return cycle;
        }
        const uint8_t base = static_cast<uint8_t>(m_pendingIoAddr);
        if (m_cycles == 2) {
            cycle.address = base;
            cycle.dummy = true;
            return cycle;
        }
        if (m_cycles == 1) {
            cycle.address = static_cast<uint8_t>(base + index);
            if (op == PendingIoOp::ZpXSta || op == PendingIoOp::ZpYStx ||
                op == PendingIoOp::ZpXSty || op == PendingIoOp::ZpYSax) {
                cycle.type = BusCycleType::Write;
                if (op == PendingIoOp::ZpXSta) cycle.data = m_a;
                else if (op == PendingIoOp::ZpYStx) cycle.data = m_x;
                else if (op == PendingIoOp::ZpYSax) cycle.data = static_cast<uint8_t>(m_a & m_x);
                else cycle.data = m_y;
            }
            return cycle;
        }
        return cycle;
    }

    if (isImmediateSequence()) {
        cycle.address = m_pc;
        cycle.exact = true;
        return cycle;
    }

    if (isJmpSequence()) {
        cycle.exact = true;
        if (m_pendingIoOp == PendingIoOp::JmpAbs) {
            cycle.address = m_pc;
            return cycle;
        }
        if (m_cycles == 4 || m_cycles == 3) { cycle.address = m_pc; return cycle; }
        if (m_cycles == 2) { cycle.address = m_pendingIoAddr; return cycle; }
        if (m_cycles == 1) {
            cycle.address = static_cast<uint16_t>((m_pendingIoAddr & 0xFF00) | ((m_pendingIoAddr + 1) & 0x00FF));
            return cycle;
        }
        return cycle;
    }


    if (isBranchSequence()) {
        cycle.exact = true;
        if (m_pendingIoData2 == 0) {
            // Cycle 2: relative offset fetch.
            cycle.address = m_pc;
            return cycle;
        }
        if ((!m_branchPageCrossed && m_cycles == 1) ||
            (m_branchPageCrossed && m_cycles == 2)) {
            // Taken branch discarded sequential-PC read.
            cycle.address = m_pc;
            cycle.dummy = true;
            return cycle;
        }
        if (m_branchPageCrossed && m_cycles == 1) {
            cycle.address = static_cast<uint16_t>((m_pc & 0xFF00) |
                                                  (m_pendingIoAddr & 0x00FF));
            cycle.dummy = true;
            return cycle;
        }
        return cycle;
    }

    // Phase 26G2 direct zero-page/absolute memory RMW sequences.
    if (isDirectRmwSequence()) {
        const bool absolute = m_pendingIoOp == PendingIoOp::AbsRmw;
        cycle.exact = true;
        if (!absolute) {
            if (m_cycles == 4) { cycle.address = m_pc; return cycle; }
            const uint16_t addr = static_cast<uint8_t>(m_pendingIoAddr);
            if (m_cycles == 3) { cycle.address = addr; return cycle; }
            if (m_cycles == 2 || m_cycles == 1) {
                cycle.type = BusCycleType::Write;
                cycle.address = addr;
                cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2;
                cycle.dummy = (m_cycles == 2);
                return cycle;
            }
            return cycle;
        }
        if (m_cycles == 5 || m_cycles == 4) { cycle.address = m_pc; return cycle; }
        const uint16_t addr = m_pendingIoAddr;
        if (m_cycles == 3) { cycle.address = addr; return cycle; }
        if (m_cycles == 2 || m_cycles == 1) {
            cycle.type = BusCycleType::Write;
            cycle.address = addr;
            cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2;
            cycle.dummy = (m_cycles == 2);
            return cycle;
        }
        return cycle;
    }

    // Phase 26G1 direct zero-page/absolute memory sequences.
    if (isDirectMemorySequence()) {
        const bool absolute = m_pendingIoOp == PendingIoOp::AbsRead ||
                              m_pendingIoOp == PendingIoOp::AbsStore;
        const bool store = m_pendingIoOp == PendingIoOp::ZpStore ||
                           m_pendingIoOp == PendingIoOp::AbsStore;
        cycle.exact = true;

        if (!absolute) {
            if (m_cycles == 2) {
                cycle.address = m_pc;
                return cycle;
            }
            if (m_cycles == 1) {
                cycle.address = static_cast<uint8_t>(m_pendingIoAddr);
                if (store) {
                    cycle.type = BusCycleType::Write;
                    cycle.data = directStoreValue();
                }
                return cycle;
            }
            return cycle;
        }

        if (m_cycles == 3 || m_cycles == 2) {
            cycle.address = m_pc;
            return cycle;
        }
        if (m_cycles == 1) {
            cycle.address = m_pendingIoAddr;
            if (store) {
                cycle.type = BusCycleType::Write;
                cycle.data = directStoreValue();
            }
            return cycle;
        }
        return cycle;
    }

    // One-byte instructions have an observable discarded PC read on cycle 2.
    if (m_pendingIoOp == PendingIoOp::DummyRead) {
        cycle.address = m_pendingIoAddr;
        cycle.dummy = true;
        cycle.exact = true;
        return cycle;
    }

    // NMOS memory RMW instructions use the final two CPU cycles for two
    // externally visible writes to the same address: first the unmodified
    // byte, then the modified byte. RDY cannot halt either write cycle.
    if (m_pendingIoOp == PendingIoOp::Rmw && (m_cycles == 2 || m_cycles == 1)) {
        cycle.type = BusCycleType::Write;
        cycle.address = m_pendingIoAddr;
        cycle.data = m_cycles == 2 ? m_pendingIoData : m_pendingIoData2;
        cycle.dummy = (m_cycles == 2);
        cycle.exact = true;
        return cycle;
    }

    // Indexed reads/stores expose their provisional address on the cycle
    // immediately before the final transfer.
    if (m_cycles == 2 && m_pendingIoOp != PendingIoOp::None) {
        uint16_t dummyAddr = 0;
        if (indexedDummyReadAddress(dummyAddr)) {
            cycle.address = dummyAddr;
            cycle.dummy = true;
            cycle.exact = true;
            return cycle;
        }
    }

    if (isImpliedSequence()) {
        cycle.address = m_pc;
        cycle.dummy = true;
        cycle.exact = true;
        return cycle;
    }

    // Final scheduled I/O is fully known, including store data. This is the
    // authoritative signal used by RDY/DMC arbitration instead of a separate
    // collection of opcode-specific write-cycle guesses.
    if (m_cycles == 1 && m_pendingIoOp != PendingIoOp::None) {
        if (m_pendingIoOp == PendingIoOp::Write) {
            cycle.type = BusCycleType::Write;
            cycle.address = m_pendingIoAddr;
            cycle.data = m_pendingIoData;
            cycle.exact = true;
            return cycle;
        }
        if (m_pendingIoOp == PendingIoOp::UnstableHighStore) {
            const uint16_t base = m_pendingIoAddr;
            uint8_t index = 0;
            uint8_t source = 0;
            switch (m_currentOpcode) {
            case 0x9C: index = m_x; source = m_y; break;
            case 0x9E: index = m_y; source = m_x; break;
            case 0x9F: case 0x93: index = m_y; source = static_cast<uint8_t>(m_a & m_x); break;
            case 0x9B: index = m_y; source = m_sp; break;
            default: break;
            }
            const uint8_t hPlusOne = static_cast<uint8_t>((base >> 8) + 1);
            const uint8_t stored = static_cast<uint8_t>(source & hPlusOne);
            uint16_t target = static_cast<uint16_t>(base + index);
            if ((base & 0xFF00) != (target & 0xFF00))
                target = static_cast<uint16_t>((static_cast<uint16_t>(stored) << 8) |
                                               (target & 0x00FF));
            cycle.type = BusCycleType::Write;
            cycle.address = target;
            cycle.data = stored;
            cycle.exact = true;
            return cycle;
        }

        cycle.address = m_pendingIoAddr;
        cycle.exact = true;
        return cycle;
    }

    // Defensive fallback. The all-opcode headless regression verifies that
    // every executable opcode reaches one of the exact finite sequences above.
    // In normal execution this path is therefore reserved for the deliberate
    // JAM/KIL halted representation (or a future unconverted sequence). Keep it
    // tagged !exact so DMA never mistakes a synthesized PC read for hardware.
    cycle.address = m_pc;
    cycle.dummy = true;
    return cycle;
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

        if (isResetSequence()) {
            clockResetSequence();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isStackSequence()) {
            // Stack/subroutine sequences now own every remaining bus slot.
            // Preserve the established interrupt poll timing while letting RTI
            // expose its restored I flag before the poll at m_cycles == 2.
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();

            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;

            clockStackSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();

            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isImpliedSequence()) {
            // Two-cycle implied/accumulator instructions poll during the
            // opcode-fetch cycle. The actual register/flag mutation and the
            // observable discarded PC read happen on cycle 2, so RDY/DMA can
            // stretch that final cycle without applying the operation early.
            if (m_pollInterruptsThisSequence && startedInstruction)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockImpliedSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isImmediateSequence()) {
            // Two-cycle immediate instructions poll IRQ/NMI during cycle 1
            // (the opcode fetch), not during the final operand-read cycle.
            // Polling on cycle 2 made a just-asserted IRQ suppress the next
            // instruction, which is visible in cpu_interrupts_v2/4-irq_and_dma
            // at the +7 boundary (LDA #$07 must finish and STA $4014 must run).
            if (m_pollInterruptsThisSequence && startedInstruction)
                pollInterrupts();

            // NMI is also sampled at the normal second-to-last-cycle point.
            // Keep the final-cycle late-edge path below for an edge that arrives
            // after this poll but before the instruction actually completes.
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockImmediateSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled) pollNmi();
            --m_cycles;
            if (m_cycles == 0) m_pollInterruptsThisSequence = false;
            return;
        }

        if (isJmpSequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2) pollInterrupts();
            const bool nmiPendingAtFinalCycleStart = m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockJmpSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled) pollNmi();
            --m_cycles;
            if (m_cycles == 0) m_pollInterruptsThisSequence = false;
            return;
        }

        if (isBranchSequence()) {
            // Branches poll once at opcode setup. A taken page-crossing branch
            // polls again immediately before its PCH-fixup/provisional cycle.
            const bool crossingPenaltyCycle = m_branchPageCrossed && m_cycles == 2;
            if (m_pollInterruptsThisSequence && (startedInstruction || crossingPenaltyCycle))
                pollInterrupts();
            clockBranchSequence();
            --m_cycles;
            if (m_cycles == 0) m_pollInterruptsThisSequence = false;
            return;
        }

        if (isDirectRmwSequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockDirectRmwSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isDirectMemorySequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockDirectMemorySequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isIndirectSequence()) {
            // ($nn),Y reads conditionally gain a provisional page-cross cycle.
            // Poll on the pointer-high cycle only when it remains the
            // second-to-last cycle; otherwise poll on the provisional cycle.
            const bool indYRead = m_pendingIoOp == PendingIoOp::IndYRead;
            const bool pointerHighCycle = indYRead && m_cycles == 2 && m_pendingIoData2 == 0;
            if (m_pollInterruptsThisSequence && m_cycles == 2 && !pointerHighCycle)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockIndirectSequence();
            if (m_pollInterruptsThisSequence && pointerHighCycle && m_cycles == 2)
                pollInterrupts();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isAbsIndexedStoreSequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockAbsIndexedStoreSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isAbsIndexedReadSequence()) {
            // A crossing read polls on the provisional cycle; a non-crossing
            // read polls on the high-byte operand cycle. The crossing status
            // is only known after that high byte has been fetched.
            if (m_pollInterruptsThisSequence && m_cycles == 2 && m_pendingIoData != 0)
                pollInterrupts();
            const bool highOperandCycle = (m_cycles == 2 && m_pendingIoData == 0);
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockAbsIndexedReadSequence();
            if (m_pollInterruptsThisSequence && highOperandCycle && m_cycles == 2)
                pollInterrupts();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isAbsIndexedRmwSequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockAbsIndexedRmwSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isZpIndexedRmwSequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockZpIndexedRmwSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
            --m_cycles;
            if (m_cycles == 0)
                m_pollInterruptsThisSequence = false;
            return;
        }

        if (isZpIndexedSequence()) {
            if (m_pollInterruptsThisSequence && m_cycles == 2)
                pollInterrupts();
            const bool nmiPendingAtFinalCycleStart =
                m_pollInterruptsThisSequence && m_cycles == 1 && m_nmiSampled;
            clockZpIndexedSequence();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
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

        // Memory RMW cycle immediately before the final write drives the
        // original byte back onto the bus. Keep the pending operation alive
        // so completePendingIo() can perform the modified write next cycle.
        if (!startedInstruction && m_cycles == 2 && m_pendingIoOp == PendingIoOp::Rmw) {
            write(m_pendingIoAddr, m_pendingIoData);
        }

        // Indexed addressing has a provisional bus read one cycle before the
        // final data access. For page-crossing reads this uses the old high
        // byte; indexed stores perform it unconditionally. Keep it on the
        // penultimate CPU cycle so memory-mapped I/O sees the correct order.
        if (!startedInstruction && m_cycles == 2 &&
            m_pendingIoOp != PendingIoOp::None &&
            m_pendingIoOp != PendingIoOp::DummyRead &&
            m_pendingIoOp != PendingIoOp::Rmw) {
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
                    // IRQ and NMI are normally recognized at the same
                // second-to-last-cycle polling point. NMI remains edge-latched
                // and may still be caught by the final-cycle late-edge path
                // below if it arrives after this poll.
                pollInterrupts();
            }
        }

        // Remember whether an external NMI edge was already present when the
        // final cycle began. A final-cycle PPU register access is allowed to
        // cancel that edge ($2002 read / $2000 disable). Conversely, an NMI
        // edge created by the final I/O access itself ($2000 enabling NMI
        // during VBlank) is deliberately not sampled until the next
        // instruction, preserving the documented one-instruction delay.
        const bool nmiPendingAtFinalCycleStart =
            m_pollInterruptsThisSequence && !isBranchOpcode(m_currentOpcode) &&
            m_cycles == 1 && m_nmiSampled;

        // Memory-mapped APU/PPU I/O happens on the final CPU cycle of the
        // instruction. Bus::clock() advances the APU before calling here,
        // which also gives the correct ordering for same-cycle frame/length
        // clocks versus CPU register accesses.
        if (m_cycles == 1) {
            completePendingIo();
            if (nmiPendingAtFinalCycleStart && m_nmiSampled)
                pollNmi();
        }
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

void CPU::scheduleUnstableHighStore(uint16_t base)
{
    // $93/$9B/$9C/$9E/$9F need the original, unindexed base until the final
    // bus cycle. On a page crossing the NMOS address-high computation collides
    // with the value being stored, so the final address high byte is corrupted
    // by the same AND that produces the data byte.
    m_pendingIoOp = PendingIoOp::UnstableHighStore;
    m_pendingIoAddr = base;
    m_pendingIoData = 0;
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
    if (op == PendingIoOp::Rmw) {
        write(addr, m_pendingIoData2);
        return;
    }
    if (op == PendingIoOp::DummyRead) {
        (void)read(addr);
        return;
    }
    if (op == PendingIoOp::UnstableHighStore) {
        const uint16_t base = addr;
        uint8_t index = 0;
        uint8_t source = 0;

        switch (m_currentOpcode) {
        case 0x9C: // SHY/SYA abs,X
            index = m_x;
            source = m_y;
            break;
        case 0x9E: // SHX/SXA abs,Y
            index = m_y;
            source = m_x;
            break;
        case 0x9F: // SHA/AHX abs,Y
        case 0x93: // SHA/AHX (zp),Y
            index = m_y;
            source = static_cast<uint8_t>(m_a & m_x);
            break;
        case 0x9B: // TAS/SHS abs,Y
            index = m_y;
            source = m_sp;
            break;
        default:
            return;
        }

        const uint8_t hPlusOne = static_cast<uint8_t>((base >> 8) + 1);
        const uint8_t stored = static_cast<uint8_t>(source & hPlusOne);
        uint16_t target = static_cast<uint16_t>(base + index);

        // Without a carry, the normal base high byte remains on the address
        // bus. With a carry, the incremented high byte is ANDed with the same
        // source that drives the data bus. Thus the high byte of the actual
        // write address becomes exactly the byte being stored.
        if ((base & 0xFF00) != (target & 0xFF00))
            target = static_cast<uint16_t>((static_cast<uint16_t>(stored) << 8) |
                                           (target & 0x00FF));

        write(target, stored);
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
    // Phase 26H2: branch condition is re-evaluated when the real cycle-2
    // offset fetch occurs. Do not consume the operand or change PC atomically
    // during opcode execution. `condition` is intentionally unused here; the
    // opcode selects the same flag test in clockBranchSequence().
    (void)condition;
    m_branchPageCrossed = false;
    m_pendingIoOp = PendingIoOp::Branch;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

uint8_t CPU::read(uint16_t addr) const { return m_bus.read(addr); }
void CPU::write(uint16_t addr, uint8_t data) { m_bus.write(addr, data); }

void CPU::writeRmw(uint16_t addr, uint8_t oldValue, uint8_t newValue)
{
    // Phase 26C: do not perform either NMOS RMW write atomically here. The
    // original value is driven on the penultimate CPU cycle and the modified
    // value on the final CPU cycle. This makes both write slots visible to
    // DMA/RDY arbitration and to memory-mapped mapper/I/O side effects.
    m_pendingIoOp = PendingIoOp::Rmw;
    m_pendingIoAddr = addr;
    m_pendingIoData = oldValue;
    m_pendingIoData2 = newValue;
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
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLDX_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLDY_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// Loads (zp/abs you already had)
void CPU::opLDA_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLDA_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLDX_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLDX_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLDY_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLDY_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

// NEW Loads: indexed & indirect

void CPU::opLDA_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXLda;
    m_pendingIoAddr = 0;
}

void CPU::opLDA_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opLDA_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opLDA_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLDA_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLDX_zpy()
{
    m_pendingIoOp = PendingIoOp::ZpYLdx;
    m_pendingIoAddr = 0;
}

void CPU::opLDX_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opLDY_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXLdy;
    m_pendingIoAddr = 0;
}

void CPU::opLDY_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

// INC/DEC registers
void CPU::opINX() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opDEX() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opINY() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opDEY() { m_pendingIoOp = PendingIoOp::Implied; }

// Flags
void CPU::opSEI() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opCLI() { m_pendingIoOp = PendingIoOp::Implied; }

void CPU::opCLC() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opSEC() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opCLD() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opSED() { m_pendingIoOp = PendingIoOp::Implied; }
void CPU::opCLV() { m_pendingIoOp = PendingIoOp::Implied; }

// Jumps / interrupts
void CPU::opJMP_abs() { m_pendingIoOp = PendingIoOp::JmpAbs; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }
void CPU::opJMP_ind() { m_pendingIoOp = PendingIoOp::JmpInd; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

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
    m_pendingIoOp = PendingIoOp::StackRti;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
}

// Stores
void CPU::opSTA_zp() { m_pendingIoOp = PendingIoOp::ZpStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }
void CPU::opSTX_zp() { m_pendingIoOp = PendingIoOp::ZpStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }
void CPU::opSTY_zp() { m_pendingIoOp = PendingIoOp::ZpStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSTA_zpx() { m_pendingIoOp = PendingIoOp::ZpXSta; m_pendingIoAddr = 0; }
void CPU::opSTX_zpy() { m_pendingIoOp = PendingIoOp::ZpYStx; m_pendingIoAddr = 0; }
void CPU::opSTY_zpx() { m_pendingIoOp = PendingIoOp::ZpXSty; m_pendingIoAddr = 0; }

void CPU::opSTA_abs() { m_pendingIoOp = PendingIoOp::AbsStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }
void CPU::opSTA_absx() { m_pendingIoOp = PendingIoOp::AbsXStore; m_pendingIoAddr = 0; m_pendingIoData = 0; }
void CPU::opSTA_absy() { m_pendingIoOp = PendingIoOp::AbsYStore; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opSTA_indx()
{
    m_pendingIoOp = PendingIoOp::IndXStore;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSTA_indy()
{
    m_pendingIoOp = PendingIoOp::IndYStore;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSTX_abs() { m_pendingIoOp = PendingIoOp::AbsStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSTY_abs() { m_pendingIoOp = PendingIoOp::AbsStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

// Logic (existing imm)
void CPU::opAND_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opORA_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opEOR_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// Logic (zp/abs)
void CPU::opAND_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opAND_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opORA_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opORA_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opEOR_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opEOR_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

// NEW Logic (indexed / indirect)

void CPU::opAND_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXAnd;
    m_pendingIoAddr = 0;
}

void CPU::opAND_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opAND_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opAND_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opAND_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opORA_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXOra;
    m_pendingIoAddr = 0;
}

void CPU::opORA_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opORA_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opORA_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opORA_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opEOR_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXEor;
    m_pendingIoAddr = 0;
}

void CPU::opEOR_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opEOR_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opEOR_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opEOR_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// BIT

void CPU::opBIT_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opBIT_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

// Shifts / rotates

void CPU::opASL_acc()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opASL_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opASL_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opASL_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opASL_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLSR_acc()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opLSR_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLSR_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLSR_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLSR_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opROL_acc()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opROL_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opROL_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opROL_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opROL_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opROR_acc()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opROR_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opROR_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opROR_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opROR_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
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
void CPU::opPHA() { m_pendingIoOp = PendingIoOp::StackPha; }
void CPU::opPHP() { m_pendingIoOp = PendingIoOp::StackPhp; }
void CPU::opPLA() { m_pendingIoOp = PendingIoOp::StackPla; }
void CPU::opPLP() { m_pendingIoOp = PendingIoOp::StackPlp; }

// Subroutines
void CPU::opJSR()
{
    m_pendingIoOp = PendingIoOp::StackJsr;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
}

void CPU::opRTS()
{
    m_pendingIoOp = PendingIoOp::StackRts;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
}

// Transfers

void CPU::opTAX()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opTXA()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opTAY()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opTYA()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opTSX()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

void CPU::opTXS()
{
    m_pendingIoOp = PendingIoOp::Implied;
}

// ---------------------------------------------------------
// ARITHMETIC
// ---------------------------------------------------------

void CPU::opADC_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opADC_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opADC_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXAdc;
    m_pendingIoAddr = 0;
}

void CPU::opADC_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opADC_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opADC_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opADC_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opADC_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSBC_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSBC_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSBC_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXSbc;
    m_pendingIoAddr = 0;
}

void CPU::opSBC_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSBC_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opSBC_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opSBC_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSBC_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
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
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opCMP_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opCMP_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXCmp;
    m_pendingIoAddr = 0;
}

void CPU::opCMP_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opCMP_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opCMP_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

void CPU::opCMP_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opCMP_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opCPX_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opCPX_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opCPX_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opCPY_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opCPY_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opCPY_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

// ---------------------------------------------------------
// MEMORY INC/DEC
// ---------------------------------------------------------

void CPU::opINC_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opINC_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opINC_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opINC_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opDEC_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opDEC_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opDEC_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opDEC_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------------------------------------------------------
// MISC
// ---------------------------------------------------------

void CPU::opNOP()
{
    m_pendingIoOp = PendingIoOp::Implied;
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
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opNOP_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opNOP_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXNop;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opNOP_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opNOP_absx() { m_pendingIoOp = PendingIoOp::AbsXRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

// ---------- SLO = ASL mem then ORA A ----------
void CPU::opSLO_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSLO_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSLO_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSLO_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSLO_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSLO_absy()
{
    m_pendingIoOp = PendingIoOp::AbsYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSLO_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- RLA = ROL mem then AND A ----------
void CPU::opRLA_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRLA_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opRLA_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opRLA_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRLA_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRLA_absy()
{
    m_pendingIoOp = PendingIoOp::AbsYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRLA_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- SRE = LSR mem then EOR A ----------
void CPU::opSRE_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSRE_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSRE_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSRE_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSRE_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSRE_absy()
{
    m_pendingIoOp = PendingIoOp::AbsYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSRE_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- RRA = ROR mem then ADC A ----------
void CPU::opRRA_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRRA_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opRRA_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opRRA_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRRA_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRRA_absy()
{
    m_pendingIoOp = PendingIoOp::AbsYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opRRA_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- SAX = store (A & X) ----------
void CPU::opSAX_indx()
{
    m_pendingIoOp = PendingIoOp::IndXStore;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSAX_zp() { m_pendingIoOp = PendingIoOp::ZpStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSAX_abs() { m_pendingIoOp = PendingIoOp::AbsStore; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opSAX_zpy()
{
    m_pendingIoOp = PendingIoOp::ZpYSax;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- LAX = LDA + LDX ----------
void CPU::opLAX_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLAX_zp() { m_pendingIoOp = PendingIoOp::ZpRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLAX_abs() { m_pendingIoOp = PendingIoOp::AbsRead; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opLAX_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRead;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLAX_zpy()
{
    m_pendingIoOp = PendingIoOp::ZpYLax;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opLAX_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

// ---------- DCP = DEC mem then CMP A ----------
void CPU::opDCP_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opDCP_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opDCP_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opDCP_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opDCP_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opDCP_absy()
{
    m_pendingIoOp = PendingIoOp::AbsYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opDCP_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- ISC / ISB = INC mem then SBC A ----------
void CPU::opISC_indx()
{
    m_pendingIoOp = PendingIoOp::IndXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opISC_zp() { m_pendingIoOp = PendingIoOp::ZpRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opISC_abs() { m_pendingIoOp = PendingIoOp::AbsRmw; m_pendingIoAddr = 0; m_pendingIoData = 0; m_pendingIoData2 = 0; }

void CPU::opISC_indy()
{
    m_pendingIoOp = PendingIoOp::IndYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opISC_zpx()
{
    m_pendingIoOp = PendingIoOp::ZpXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opISC_absy()
{
    m_pendingIoOp = PendingIoOp::AbsYRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opISC_absx()
{
    m_pendingIoOp = PendingIoOp::AbsXRmw;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- Immediate unofficial ----------
void CPU::opANC_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opALR_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opARR_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opAXS_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSBC_imm_unofficial()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// Unstable immediate approximations (common in accurate emulators)
void CPU::opLXA_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opANE_imm()
{
    m_pendingIoOp = PendingIoOp::Immediate;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

// ---------- Highly unstable store / transfer opcodes ----------
// These use the classic “AND with (H+1)” approximation.

void CPU::opSHA_indy()
{
    m_unstableHighStoreRdy = false;
    m_pendingIoOp = PendingIoOp::IndYHighStore;
    m_pendingIoAddr = 0;
    m_pendingIoData = 0;
    m_pendingIoData2 = 0;
}

void CPU::opSHA_absy()
{
    m_unstableHighStoreRdy = false;
    m_pendingIoOp = PendingIoOp::AbsYHighStore; m_pendingIoAddr = 0; m_pendingIoData = 0;
}

void CPU::opSHX_absy()
{
    m_unstableHighStoreRdy = false;
    m_pendingIoOp = PendingIoOp::AbsYHighStore; m_pendingIoAddr = 0; m_pendingIoData = 0;
}

void CPU::opSHY_absx()
{
    m_unstableHighStoreRdy = false;
    m_pendingIoOp = PendingIoOp::AbsXHighStore; m_pendingIoAddr = 0; m_pendingIoData = 0;
}

void CPU::opTAS_absy()
{
    m_unstableHighStoreRdy = false;
    m_sp = static_cast<uint8_t>(m_a & m_x);
    m_pendingIoOp = PendingIoOp::AbsYHighStore; m_pendingIoAddr = 0; m_pendingIoData = 0;
}

void CPU::opLAS_absy() { m_pendingIoOp = PendingIoOp::AbsYRead; m_pendingIoAddr = 0; m_pendingIoData = 0; }

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
    put8(m_nmiSampled ? 1 : 0);
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
    put8(m_pendingIoData2);
}

bool CPU::loadState(const uint8_t*& p, const uint8_t* end)
{
    auto get8 = [&](uint8_t& v) -> bool {
        if (p >= end) return false;
        v = *p++;
        return true;
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
    uint8_t nmiPending = 0, nmiSampled = 0, nmiPolled = 0, irqLine = 0, irqPolled = 0;
    uint8_t pollSequence = 0, irqDisableBefore = 0, currentOpcode = 0, branchPageCrossed = 0, pendingIo = 0;
    uint16_t pendingAddr = 0;
    uint8_t pendingData = 0, pendingData2 = 0;
    if (!get8(m_a) || !get8(m_x) || !get8(m_y) || !get8(m_sp)) return false;
    if (!get16(m_pc) || !get8(m_status) || !get32(cycles)) return false;
    if (!get8(nmiPending) || !get8(nmiSampled) || !get8(nmiPolled) || !get8(irqLine) || !get8(irqPolled) ||
        !get8(pollSequence) || !get8(irqDisableBefore) || !get8(currentOpcode) ||
        !get8(branchPageCrossed) || !get8(pendingIo) ||
        !get16(pendingAddr) || !get8(pendingData) || !get8(pendingData2)) return false;
    if (pendingIo > static_cast<uint8_t>(PendingIoOp::ZpYSax)) return false;
    m_status = (m_status & 0xEF) | 0x20;
    m_cycles = (int)cycles;
    m_nmiPending = nmiPending != 0;
    m_nmiSampled = nmiSampled != 0;
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
    m_pendingIoData2 = pendingData2;
    return true;
}
























