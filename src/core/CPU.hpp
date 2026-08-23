#pragma once
#include <cstdint>
#include <vector>
#include <array>

class Bus;

class CPU {
public:
    CPU(Bus& bus);

    void powerOn();

    void reset();
    void clock();
    void nmi();

    void sampleNmiInput();

    void cancelPendingNmi();

    void setIrqLine(bool asserted);
    void irq();

    bool atInstructionBoundary() const { return m_cycles == 0; }
    uint64_t instructionCount() const { return m_instructionCount; }

    enum class BusCycleType : uint8_t { Read = 0, Write = 1 };
    struct BusCycle {
        BusCycleType type = BusCycleType::Read;
        uint16_t address = 0;
        uint8_t data = 0;
        bool dummy = false;
        bool exact = false;
    };
    BusCycle nextBusCycle() const;

    void notifyRdyReadStall();

#ifdef NES_HEADLESS

    bool testNmiPending() const { return m_nmiPending || m_nmiSampled; }
    bool testNmiPolled() const { return m_nmiPolled; }
    uint8_t testAccumulator() const { return m_a; }
    uint16_t testProgramCounter() const { return m_pc; }
#endif

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

    bool m_nmiPending = false;
    bool m_nmiSampled = false;
    bool m_nmiPolled = false;
    bool m_irqLine = false;
    bool m_irqPolled = false;
    bool m_pollInterruptsThisSequence = false;
    bool m_irqDisableBeforeInstruction = true;
    uint8_t m_currentOpcode = 0;
    bool m_branchPageCrossed = false;
    bool m_unstableHighStoreRdy = false;

    enum class PendingIoOp : uint8_t {
        None = 0,
        Write,
        Lda,
        Ldx,
        Ldy,
        Bit,
        Lax,

        DummyRead,

        InterruptBrkIrq,
        InterruptBrkNmi,
        InterruptIrqIrq,
        InterruptIrqNmi,
        InterruptNmi,

        UnstableHighStore,

        ResetSequence,
        StackPha,
        StackPhp,
        StackPla,
        StackPlp,
        StackJsr,
        StackRts,
        StackRti,

        Rmw,

        ZpXLda,
        ZpYLdx,
        ZpXLdy,
        ZpXSta,
        ZpYStx,
        ZpXSty,

        ZpXAnd,
        ZpXOra,
        ZpXEor,
        ZpXAdc,
        ZpXSbc,
        ZpXCmp,

        ZpXRmw,

        AbsXRead,
        AbsYRead,

        AbsXStore,
        AbsYStore,
        AbsXHighStore,
        AbsYHighStore,

        AbsXRmw,
        AbsYRmw,

        IndXRead,
        IndXStore,
        IndXRmw,
        IndYRead,
        IndYStore,
        IndYHighStore,
        IndYRmw,

        ZpRead,
        ZpStore,
        AbsRead,
        AbsStore,

        ZpRmw,
        AbsRmw,

        Immediate,
        JmpAbs,
        JmpInd,

        Branch,

        ZpXNop,
        ZpYLax,
        ZpYSax,

        Implied
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
    bool isImpliedSequence() const;
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
    void clockImpliedSequence();
    void clockJmpSequence();
    void clockBranchSequence();
    void applyImmediate(uint8_t value);
    void applyImplied();
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

    void cmpHelper(uint8_t reg, uint8_t value);
    void setZeroNeg(uint8_t value);
    void setFlag(uint8_t flag, bool value);
    bool getFlag(uint8_t flag) const;

    void branchIf(bool condition);

    uint8_t read(uint16_t addr) const;
    void    write(uint16_t addr, uint8_t data);
    void    writeRmw(uint16_t addr, uint8_t oldValue, uint8_t newValue);
    void scheduleIoWrite(uint16_t addr, uint8_t data);
    void scheduleUnstableHighStore(uint16_t base);
    void scheduleIoRead(PendingIoOp op, uint16_t addr);
    void completePendingIo();

    void push(uint8_t value);
    uint8_t pull();
    void push16(uint16_t value);
    uint16_t pull16();

    uint8_t fetch();
    void    execute(uint8_t opcode);

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

    uint16_t addrAbsoluteXRead();
    uint16_t addrAbsoluteYRead();
    uint16_t addrIndirectYRead();

    struct Instruction {
        void (CPU::* operate)();
        int cycles;
    };

    std::array<Instruction, 256> m_table;

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

    void opINX();
    void opDEX();
    void opINY();
    void opDEY();

    void opSEI();
    void opCLI();

    void opCLC();
    void opSEC();
    void opCLD();
    void opSED();
    void opCLV();

    void opJMP_abs();
    void opJMP_ind();
    void opBRK();
    void opRTI();

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

    void opBIT_zp();
    void opBIT_abs();

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

    void opBEQ();
    void opBNE();
    void opBMI();
    void opBPL();
    void opBCC();
    void opBCS();
    void opBVC();
    void opBVS();

    void opPHA();
    void opPHP();
    void opPLA();
    void opPLP();

    void opJSR();
    void opRTS();

    void opTAX();
    void opTXA();
    void opTAY();
    void opTYA();
    void opTSX();
    void opTXS();

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

    void opINC_zp();
    void opINC_zpx();
    void opINC_abs();
    void opINC_absx();
    void opDEC_zp();
    void opDEC_zpx();
    void opDEC_abs();
    void opDEC_absx();

    void opNOP();

    void opJAM();

    void opNOP_imm();
    void opNOP_zp();
    void opNOP_zpx();
    void opNOP_abs();
    void opNOP_absx();

    void opSLO_indx();
    void opSLO_zp();
    void opSLO_abs();
    void opSLO_indy();
    void opSLO_zpx();
    void opSLO_absy();
    void opSLO_absx();

    void opRLA_indx();
    void opRLA_zp();
    void opRLA_abs();
    void opRLA_indy();
    void opRLA_zpx();
    void opRLA_absy();
    void opRLA_absx();

    void opSRE_indx();
    void opSRE_zp();
    void opSRE_abs();
    void opSRE_indy();
    void opSRE_zpx();
    void opSRE_absy();
    void opSRE_absx();

    void opRRA_indx();
    void opRRA_zp();
    void opRRA_abs();
    void opRRA_indy();
    void opRRA_zpx();
    void opRRA_absy();
    void opRRA_absx();

    void opSAX_indx();
    void opSAX_zp();
    void opSAX_abs();
    void opSAX_zpy();

    void opLAX_indx();
    void opLAX_zp();
    void opLAX_abs();
    void opLAX_indy();
    void opLAX_zpy();
    void opLAX_absy();

    void opDCP_indx();
    void opDCP_zp();
    void opDCP_abs();
    void opDCP_indy();
    void opDCP_zpx();
    void opDCP_absy();
    void opDCP_absx();

    void opISC_indx();
    void opISC_zp();
    void opISC_abs();
    void opISC_indy();
    void opISC_zpx();
    void opISC_absy();
    void opISC_absx();

    void opANC_imm();
    void opALR_imm();
    void opARR_imm();
    void opAXS_imm();
    void opSBC_imm_unofficial();
    void opLXA_imm();
    void opANE_imm();

    void opSHA_indy();
    void opSHA_absy();
    void opSHX_absy();
    void opSHY_absx();
    void opTAS_absy();
    void opLAS_absy();

    void buildTable();
};
