#pragma once
#include <cstdint>
#include <vector>
#include <array>

class Bus;

class CPU {
public:
    CPU(Bus& bus);

    // Cold CPU power-up: initializes architectural registers to the NES
    // power-on values, then performs the reset-vector sequence.
    void powerOn();

    // Console RESET input: preserves A/X/Y and RAM-visible state, sets I,
    // decrements S by three without stack writes, and reloads the vector.
    void reset();
    void clock();
    void nmi();   // Non-maskable interrupt (from PPU VBlank)
    // Cancel an asynchronous NMI edge that has not yet been sampled by the
    // CPU. The PPU uses this only for the very narrow VBlank race where a
    // $2002 read or $2000 NMI-disable immediately after VBlank suppresses an
    // otherwise newly-generated NMI edge.
    void cancelPendingNmi();
    // IRQ is a shared level-sensitive line. The Bus recomputes it from
    // all active IRQ sources before each CPU clock.
    void setIrqLine(bool asserted);
    void irq(); // Legacy helper: assert the line until the Bus recomputes it.

    bool atInstructionBoundary() const { return m_cycles == 0; }
    uint64_t instructionCount() const { return m_instructionCount; }

    // Canonical description of the CPU bus slot that would execute on the
    // next CPU clock. Every NMOS 6502 clock is externally a read or a write.
    // `exact` is false only for instruction families that are still executed
    // atomically by this core; those synthesized slots are deliberately
    // visible so later cycle-conversion phases can remove them without hiding
    // assumptions inside DMA code.
    enum class BusCycleType : uint8_t { Read = 0, Write = 1 };
    struct BusCycle {
        BusCycleType type = BusCycleType::Read;
        uint16_t address = 0;
        uint8_t data = 0;
        bool dummy = false;
        bool exact = false;
    };
    BusCycle nextBusCycle() const;

    // Save states
    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

private:
    Bus& m_bus;

    uint8_t  m_a = 0;
    uint8_t  m_x = 0;
    uint8_t  m_y = 0;
    uint8_t  m_sp = 0;
    uint16_t m_pc = 0;
    uint8_t  m_status = 0;

    int m_cycles = 0;
    // External interrupt state is sampled before the current instruction
    // ends.  m_nmiPending is the asynchronous edge latch; the *Polled
    // fields are what the CPU actually committed to service at the next
    // instruction boundary.  Looking at the live IRQ line only at the
    // boundary is too late and makes APU IRQs arrive one instruction early.
    bool m_nmiPending = false;
    bool m_nmiPolled = false;
    bool m_irqLine = false;
    bool m_irqPolled = false;
    bool m_pollInterruptsThisSequence = false;
    bool m_irqDisableBeforeInstruction = true;
    uint8_t m_currentOpcode = 0;
    bool m_branchPageCrossed = false;

    // The CPU core is instruction-oriented, but external bus accesses still
    // need the correct relative cycle ordering. Loads/stores that use this
    // queue complete on the final cycle, leaving the penultimate cycle free
    // for observable indexed dummy reads.
    enum class PendingIoOp : uint8_t {
        None = 0,
        Write,
        Lda,
        Ldx,
        Ldy,
        Bit,
        Lax,
        // Cycle-2 read of PC for one-byte NMOS 6502 instructions. Reuses
        // the existing pending-I/O state so mid-instruction save states keep
        // the address of the externally visible dummy bus access.
        DummyRead,

        // Interrupt-entry microsequences. These reuse the serialized pending
        // I/O byte so BRK/IRQ/NMI can expose their stack/vector bus cycles
        // without changing the save-state payload layout. The IRQ-vector
        // forms may be hijacked by a late NMI before the vector low fetch.
        InterruptBrkIrq,
        InterruptBrkNmi,
        InterruptIrqIrq,
        InterruptIrqNmi,
        InterruptNmi,

        // Unstable NMOS high-byte stores ($93/$9B/$9C/$9E/$9F). Keep this
        // after the existing interrupt values so old save-state enum values
        // remain binary-compatible. m_pendingIoAddr stores the unindexed
        // base address; the final write address/data are resolved on cycle 5/6.
        UnstableHighStore,

        // Phase 26B: explicit reset/stack/subroutine microsequences. Appended
        // so all pre-26B serialized enum values keep their numeric meaning.
        ResetSequence,
        StackPha,
        StackPhp,
        StackPla,
        StackPlp,
        StackJsr,
        StackRts,
        StackRti,

        // Phase 26C: scheduled NMOS memory read/modify/write bus sequence.
        // m_pendingIoData is the original value; m_pendingIoData2 is the
        // modified value. The penultimate CPU cycle writes the original byte
        // and the final cycle writes the modified byte.
        Rmw,

        // Phase 26D1: fully explicit indexed zero-page load/store sequences.
        // Cycle 2 fetches the zero-page operand into m_pendingIoAddr, cycle 3
        // performs the mandatory unindexed dummy read, and cycle 4 performs
        // the indexed data access.
        ZpXLda,
        ZpYLdx,
        ZpXLdy,
        ZpXSta,
        ZpYStx,
        ZpXSty,

        // Phase 26D2: indexed zero-page ALU/compare reads. These share the
        // same cycle-2 operand fetch / cycle-3 unindexed dummy read /
        // cycle-4 indexed data read sequence as the Phase 26D1 loads.
        ZpXAnd,
        ZpXOra,
        ZpXEor,
        ZpXAdc,
        ZpXSbc,
        ZpXCmp,

        // Phase 26D3: indexed zero-page memory RMW sequence. The current
        // opcode selects ASL/LSR/ROL/ROR/INC/DEC or the unofficial combined
        // operation. m_pendingIoAddr holds the unindexed zero-page operand;
        // m_pendingIoData/Data2 hold old/new values after the indexed read.
        ZpXRmw,

        // Phase 26E1: explicit absolute-indexed read sequences. m_pendingIoAddr
        // holds the low operand byte until cycle 3, then the unindexed base.
        // m_pendingIoData marks that the high operand byte has been fetched.
        AbsXRead,
        AbsYRead,

        // Phase 26E2: explicit absolute-indexed store sequences. The normal
        // stores and unstable high-byte stores fetch both operand bytes on
        // their real cycles, perform the mandatory provisional read, then
        // drive the final write on cycle 5.
        AbsXStore,
        AbsYStore,
        AbsXHighStore,
        AbsYHighStore,

        // Phase 26E3: explicit absolute-indexed memory RMW sequences. These
        // always perform the provisional read, corrected data read, old-value
        // write, then modified-value write. X is used by official abs,X and
        // most unofficial forms; Y is used by the unofficial abs,Y forms.
        AbsXRmw,
        AbsYRmw,

        // Phase 26F: explicit indexed-indirect / indirect-indexed sequences.
        // IndX* uses the six-cycle ($nn,X) pointer walk (eight for RMW).
        // IndYRead conditionally adds the corrected read on page crossing;
        // IndYStore/Rmw always perform the provisional read.
        IndXRead,
        IndXStore,
        IndXRmw,
        IndYRead,
        IndYStore,
        IndYHighStore,
        IndYRmw,

        // Phase 26G1: explicit non-indexed zero-page/absolute read/store
        // sequences. The current opcode determines the operation while this
        // state owns the real operand-fetch and final data-transfer cycles.
        ZpRead,
        ZpStore,
        AbsRead,
        AbsStore,

        // Phase 26G2: explicit non-indexed zero-page/absolute RMW sequences.
        // These expose operand/address fetches, data read, old-value write,
        // and modified-value write as separate exact CPU bus cycles.
        ZpRmw,
        AbsRmw,

        // Phase 26H1: exact two-cycle immediate operand fetches and JMP
        // control-flow sequences. Immediate applies the current opcode after
        // fetching PC on cycle 2. JMP absolute/indirect expose every operand
        // and pointer read, including the NMOS indirect page-wrap bug.
        Immediate,
        JmpAbs,
        JmpInd,

        // Phase 26H2: exact conditional-branch sequence. m_pendingIoData
        // stores the signed offset byte; m_pendingIoAddr stores the final
        // target once cycle 2 has evaluated a taken branch.
        Branch,

        // Phase 26I: close remaining normal indexed-zero-page gaps.
        ZpXNop,
        ZpYLax,
        ZpYSax
    };
    PendingIoOp m_pendingIoOp = PendingIoOp::None;
    uint16_t m_pendingIoAddr = 0;
    uint8_t m_pendingIoData = 0;
    uint8_t m_pendingIoData2 = 0;

    uint64_t m_instructionCount = 0;

    void serviceNmi();
    void serviceIrq();
    bool isInterruptEntry() const;
    void clockInterruptEntry();
    bool isResetSequence() const;
    bool isStackSequence() const;
    bool isZpIndexedSequence() const;
    bool isZpIndexedRmwSequence() const;
    bool isAbsIndexedRmwSequence() const;
    bool isAbsIndexedReadSequence() const;
    bool isAbsIndexedStoreSequence() const;
    bool isIndirectSequence() const;
    bool isDirectMemorySequence() const;
    bool isDirectRmwSequence() const;
    bool isImmediateSequence() const;
    bool isJmpSequence() const;
    bool isBranchSequence() const;
    void clockResetSequence();
    void clockStackSequence();
    void clockZpIndexedSequence();
    void clockZpIndexedRmwSequence();
    void clockAbsIndexedRmwSequence();
    void applyIndexedRmw(uint8_t oldValue, uint8_t& newValue);
    void clockAbsIndexedReadSequence();
    void clockAbsIndexedStoreSequence();
    void clockIndirectSequence();
    void clockDirectMemorySequence();
    void clockDirectRmwSequence();
    void clockImmediateSequence();
    void clockJmpSequence();
    void clockBranchSequence();
    void applyImmediate(uint8_t value);
    void applyDirectMemoryRead(uint8_t value);
    uint8_t directStoreValue() const;
    void applyAbsIndexedRead(uint8_t value);
    void applyIndirectRead(uint8_t value);
    void pollNmi();
    void pollIrq();
    void pollInterrupts();
    static bool isBranchOpcode(uint8_t opcode);
    static bool needsSecondCyclePcRead(uint8_t opcode);
    bool indexedDummyReadAddress(uint16_t& addr) const;

    // Status flag helpers
    void cmpHelper(uint8_t reg, uint8_t value);
    void setZeroNeg(uint8_t value);
    void setFlag(uint8_t flag, bool value);
    bool getFlag(uint8_t flag) const;

    // Branch helper (handles cycles + page cross)
    void branchIf(bool condition);

    // Memory helpers
    uint8_t read(uint16_t addr) const;
    void    write(uint16_t addr, uint8_t data);
    void    writeRmw(uint16_t addr, uint8_t oldValue, uint8_t newValue);
    void scheduleIoWrite(uint16_t addr, uint8_t data);
    void scheduleUnstableHighStore(uint16_t base);
    void scheduleIoRead(PendingIoOp op, uint16_t addr);
    void completePendingIo();

    // Stack helpers
    void push(uint8_t value);
    uint8_t pull();
    void push16(uint16_t value);
    uint16_t pull16();

    // Instruction helpers
    uint8_t fetch();
    void    execute(uint8_t opcode);

    // Addressing modes
    uint16_t addrImmediate();
    uint16_t addrAbsolute();
    uint16_t addrIndirect();
    uint16_t addrZeroPage();
    uint16_t addrZeroPageX();
    uint16_t addrZeroPageY();
    uint16_t addrAbsoluteX();
    uint16_t addrAbsoluteY();
    uint16_t addrIndirectX();
    uint16_t addrIndirectY();

    // Read variants with page-cross penalty (for loads/ALU/logic/compare)
    uint16_t addrAbsoluteXRead();
    uint16_t addrAbsoluteYRead();
    uint16_t addrIndirectYRead();

    struct Instruction {
        void (CPU::* operate)();
        int cycles;
    };

    std::array<Instruction, 256> m_table;

    // Loads
    void opLDA_imm();
    void opLDX_imm();
    void opLDY_imm();

    void opLDA_zp();
    void opLDA_zpx();
    void opLDA_abs();
    void opLDA_absx();
    void opLDA_absy();
    void opLDA_indx();
    void opLDA_indy();

    void opLDX_zp();
    void opLDX_zpy();
    void opLDX_abs();
    void opLDX_absy();

    void opLDY_zp();
    void opLDY_zpx();
    void opLDY_abs();
    void opLDY_absx();

    // INC/DEC registers
    void opINX();
    void opDEX();
    void opINY();
    void opDEY();

    // Flags
    void opSEI();
    void opCLI();

    // New flag ops
    void opCLC();
    void opSEC();
    void opCLD();
    void opSED();
    void opCLV();

    // Jumps / interrupts
    void opJMP_abs();
    void opJMP_ind();
    void opBRK();
    void opRTI();

    // Stores
    void opSTA_zp();
    void opSTX_zp();
    void opSTY_zp();

    void opSTA_zpx();
    void opSTX_zpy();
    void opSTY_zpx();

    void opSTA_abs();
    void opSTA_absx();
    void opSTA_absy();

    void opSTA_indx();
    void opSTA_indy();

    void opSTX_abs();
    void opSTY_abs();

    // Logic
    void opAND_imm();
    void opORA_imm();
    void opEOR_imm();

    void opAND_zp();
    void opAND_zpx();
    void opAND_abs();
    void opAND_absx();
    void opAND_absy();
    void opAND_indx();
    void opAND_indy();

    void opORA_zp();
    void opORA_zpx();
    void opORA_abs();
    void opORA_absx();
    void opORA_absy();
    void opORA_indx();
    void opORA_indy();

    void opEOR_zp();
    void opEOR_zpx();
    void opEOR_abs();
    void opEOR_absx();
    void opEOR_absy();
    void opEOR_indx();
    void opEOR_indy();

    // BIT
    void opBIT_zp();
    void opBIT_abs();

    // Shifts / rotates
    void opASL_acc();
    void opASL_zp();
    void opASL_zpx();
    void opASL_abs();
    void opASL_absx();

    void opLSR_acc();
    void opLSR_zp();
    void opLSR_zpx();
    void opLSR_abs();
    void opLSR_absx();

    void opROL_acc();
    void opROL_zp();
    void opROL_zpx();
    void opROL_abs();
    void opROL_absx();

    void opROR_acc();
    void opROR_zp();
    void opROR_zpx();
    void opROR_abs();
    void opROR_absx();

    // Branches
    void opBEQ();
    void opBNE();
    void opBMI();
    void opBPL();
    void opBCC();
    void opBCS();
    void opBVC();
    void opBVS();

    // Stack ops
    void opPHA();
    void opPHP();
    void opPLA();
    void opPLP();

    // Subroutines
    void opJSR();
    void opRTS();

    // Transfers
    void opTAX();
    void opTXA();
    void opTAY();
    void opTYA();
    void opTSX();
    void opTXS();

    // Arithmetic
    void opADC_imm();
    void opADC_zp();
    void opADC_zpx();
    void opADC_abs();
    void opADC_absx();
    void opADC_absy();
    void opADC_indx();
    void opADC_indy();

    void opSBC_imm();
    void opSBC_zp();
    void opSBC_zpx();
    void opSBC_abs();
    void opSBC_absx();
    void opSBC_absy();
    void opSBC_indx();
    void opSBC_indy();

    // Compare
    void opCMP_imm();
    void opCMP_zp();
    void opCMP_zpx();
    void opCMP_abs();
    void opCMP_absx();
    void opCMP_absy();
    void opCMP_indx();
    void opCMP_indy();

    void opCPX_imm();
    void opCPX_zp();
    void opCPX_abs();

    void opCPY_imm();
    void opCPY_zp();
    void opCPY_abs();

    // Memory INC/DEC
    void opINC_zp();
    void opINC_zpx();
    void opINC_abs();
    void opINC_absx();
    void opDEC_zp();
    void opDEC_zpx();
    void opDEC_abs();
    void opDEC_absx();

    // Misc
    void opNOP();

    // ---------------------------------------------------------
    // UNOFFICIAL / ILLEGAL OPCODES (all remaining 105)
    // ---------------------------------------------------------

    // JAM / KIL (halt)
    void opJAM();

    // Unofficial NOPs (various lengths / addressing)
    void opNOP_imm();
    void opNOP_zp();
    void opNOP_zpx();
    void opNOP_abs();
    void opNOP_absx();

    // SLO = ASL + ORA
    void opSLO_indx();
    void opSLO_zp();
    void opSLO_abs();
    void opSLO_indy();
    void opSLO_zpx();
    void opSLO_absy();
    void opSLO_absx();

    // RLA = ROL + AND
    void opRLA_indx();
    void opRLA_zp();
    void opRLA_abs();
    void opRLA_indy();
    void opRLA_zpx();
    void opRLA_absy();
    void opRLA_absx();

    // SRE = LSR + EOR
    void opSRE_indx();
    void opSRE_zp();
    void opSRE_abs();
    void opSRE_indy();
    void opSRE_zpx();
    void opSRE_absy();
    void opSRE_absx();

    // RRA = ROR + ADC
    void opRRA_indx();
    void opRRA_zp();
    void opRRA_abs();
    void opRRA_indy();
    void opRRA_zpx();
    void opRRA_absy();
    void opRRA_absx();

    // SAX = store A & X
    void opSAX_indx();
    void opSAX_zp();
    void opSAX_abs();
    void opSAX_zpy();

    // LAX = LDA + LDX
    void opLAX_indx();
    void opLAX_zp();
    void opLAX_abs();
    void opLAX_indy();
    void opLAX_zpy();
    void opLAX_absy();

    // DCP = DEC + CMP
    void opDCP_indx();
    void opDCP_zp();
    void opDCP_abs();
    void opDCP_indy();
    void opDCP_zpx();
    void opDCP_absy();
    void opDCP_absx();

    // ISC / ISB = INC + SBC
    void opISC_indx();
    void opISC_zp();
    void opISC_abs();
    void opISC_indy();
    void opISC_zpx();
    void opISC_absy();
    void opISC_absx();

    // Immediate unofficial
    void opANC_imm();   // AND + set C from bit 7
    void opALR_imm();   // AND + LSR
    void opARR_imm();   // AND + ROR (special flags)
    void opAXS_imm();   // X = (A & X) - imm
    void opSBC_imm_unofficial(); // $EB same as official SBC
    void opLXA_imm();   // unstable LAX immediate (approx)
    void opANE_imm();   // unstable ANE/XAA (approx)

    // Highly unstable (implemented with common approximations)
    void opSHA_indy();
    void opSHA_absy();
    void opSHX_absy();
    void opSHY_absx();
    void opTAS_absy();
    void opLAS_absy();

    void buildTable();
};




