#pragma once
#include <cstdint>
#include <array>

class Bus;

class CPU {
public:
    CPU(Bus& bus);

    void reset();
    void clock();
    void nmi();   // Non-maskable interrupt (from PPU VBlank)
    void irq();   // Maskable interrupt (MMC3, APU, etc.)

private:
    Bus& m_bus;

    uint8_t  m_a = 0;
    uint8_t  m_x = 0;
    uint8_t  m_y = 0;
    uint8_t  m_sp = 0;
    uint16_t m_pc = 0;
    uint8_t  m_status = 0;

    int m_cycles = 0;

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


