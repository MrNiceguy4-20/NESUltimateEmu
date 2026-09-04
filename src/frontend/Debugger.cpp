#include "Debugger.hpp"
#include "../core/Bus.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <iomanip>

namespace {

enum class Mode { IMP, ACC, IMM, ZP, ZPX, ZPY, ABS, ABSX, ABSY, IND, INDX, INDY, REL };
struct OpcodeInfo { const char* mnemonic; Mode mode; };

constexpr std::array<OpcodeInfo, 256> kOpcodes = {{
    { "BRK", Mode::IMP },
    { "ORA", Mode::INDX },
    { "JAM", Mode::IMP },
    { "SLO", Mode::INDX },
    { "NOP", Mode::ZP },
    { "ORA", Mode::ZP },
    { "ASL", Mode::ZP },
    { "SLO", Mode::ZP },
    { "PHP", Mode::IMP },
    { "ORA", Mode::IMM },
    { "ASL", Mode::ACC },
    { "ANC", Mode::IMM },
    { "NOP", Mode::ABS },
    { "ORA", Mode::ABS },
    { "ASL", Mode::ABS },
    { "SLO", Mode::ABS },
    { "BPL", Mode::REL },
    { "ORA", Mode::INDY },
    { "JAM", Mode::IMP },
    { "SLO", Mode::INDY },
    { "NOP", Mode::ZPX },
    { "ORA", Mode::ZPX },
    { "ASL", Mode::ZPX },
    { "SLO", Mode::ZPX },
    { "CLC", Mode::IMP },
    { "ORA", Mode::ABSY },
    { "NOP", Mode::IMP },
    { "SLO", Mode::ABSY },
    { "NOP", Mode::ABSX },
    { "ORA", Mode::ABSX },
    { "ASL", Mode::ABSX },
    { "SLO", Mode::ABSX },
    { "JSR", Mode::ABS },
    { "AND", Mode::INDX },
    { "JAM", Mode::IMP },
    { "RLA", Mode::INDX },
    { "BIT", Mode::ZP },
    { "AND", Mode::ZP },
    { "ROL", Mode::ZP },
    { "RLA", Mode::ZP },
    { "PLP", Mode::IMP },
    { "AND", Mode::IMM },
    { "ROL", Mode::ACC },
    { "ANC", Mode::IMM },
    { "BIT", Mode::ABS },
    { "AND", Mode::ABS },
    { "ROL", Mode::ABS },
    { "RLA", Mode::ABS },
    { "BMI", Mode::REL },
    { "AND", Mode::INDY },
    { "JAM", Mode::IMP },
    { "RLA", Mode::INDY },
    { "NOP", Mode::ZPX },
    { "AND", Mode::ZPX },
    { "ROL", Mode::ZPX },
    { "RLA", Mode::ZPX },
    { "SEC", Mode::IMP },
    { "AND", Mode::ABSY },
    { "NOP", Mode::IMP },
    { "RLA", Mode::ABSY },
    { "NOP", Mode::ABSX },
    { "AND", Mode::ABSX },
    { "ROL", Mode::ABSX },
    { "RLA", Mode::ABSX },
    { "RTI", Mode::IMP },
    { "EOR", Mode::INDX },
    { "JAM", Mode::IMP },
    { "SRE", Mode::INDX },
    { "NOP", Mode::ZP },
    { "EOR", Mode::ZP },
    { "LSR", Mode::ZP },
    { "SRE", Mode::ZP },
    { "PHA", Mode::IMP },
    { "EOR", Mode::IMM },
    { "LSR", Mode::ACC },
    { "ALR", Mode::IMM },
    { "JMP", Mode::ABS },
    { "EOR", Mode::ABS },
    { "LSR", Mode::ABS },
    { "SRE", Mode::ABS },
    { "BVC", Mode::REL },
    { "EOR", Mode::INDY },
    { "JAM", Mode::IMP },
    { "SRE", Mode::INDY },
    { "NOP", Mode::ZPX },
    { "EOR", Mode::ZPX },
    { "LSR", Mode::ZPX },
    { "SRE", Mode::ZPX },
    { "CLI", Mode::IMP },
    { "EOR", Mode::ABSY },
    { "NOP", Mode::IMP },
    { "SRE", Mode::ABSY },
    { "NOP", Mode::ABSX },
    { "EOR", Mode::ABSX },
    { "LSR", Mode::ABSX },
    { "SRE", Mode::ABSX },
    { "RTS", Mode::IMP },
    { "ADC", Mode::INDX },
    { "JAM", Mode::IMP },
    { "RRA", Mode::INDX },
    { "NOP", Mode::ZP },
    { "ADC", Mode::ZP },
    { "ROR", Mode::ZP },
    { "RRA", Mode::ZP },
    { "PLA", Mode::IMP },
    { "ADC", Mode::IMM },
    { "ROR", Mode::ACC },
    { "ARR", Mode::IMM },
    { "JMP", Mode::IND },
    { "ADC", Mode::ABS },
    { "ROR", Mode::ABS },
    { "RRA", Mode::ABS },
    { "BVS", Mode::REL },
    { "ADC", Mode::INDY },
    { "JAM", Mode::IMP },
    { "RRA", Mode::INDY },
    { "NOP", Mode::ZPX },
    { "ADC", Mode::ZPX },
    { "ROR", Mode::ZPX },
    { "RRA", Mode::ZPX },
    { "SEI", Mode::IMP },
    { "ADC", Mode::ABSY },
    { "NOP", Mode::IMP },
    { "RRA", Mode::ABSY },
    { "NOP", Mode::ABSX },
    { "ADC", Mode::ABSX },
    { "ROR", Mode::ABSX },
    { "RRA", Mode::ABSX },
    { "NOP", Mode::IMM },
    { "STA", Mode::INDX },
    { "NOP", Mode::IMM },
    { "SAX", Mode::INDX },
    { "STY", Mode::ZP },
    { "STA", Mode::ZP },
    { "STX", Mode::ZP },
    { "SAX", Mode::ZP },
    { "DEY", Mode::IMP },
    { "NOP", Mode::IMM },
    { "TXA", Mode::IMP },
    { "ANE", Mode::IMM },
    { "STY", Mode::ABS },
    { "STA", Mode::ABS },
    { "STX", Mode::ABS },
    { "SAX", Mode::ABS },
    { "BCC", Mode::REL },
    { "STA", Mode::INDY },
    { "JAM", Mode::IMP },
    { "SHA", Mode::INDY },
    { "STY", Mode::ZPX },
    { "STA", Mode::ZPX },
    { "STX", Mode::ZPY },
    { "SAX", Mode::ZPY },
    { "TYA", Mode::IMP },
    { "STA", Mode::ABSY },
    { "TXS", Mode::IMP },
    { "TAS", Mode::ABSY },
    { "SHY", Mode::ABSX },
    { "STA", Mode::ABSX },
    { "SHX", Mode::ABSY },
    { "SHA", Mode::ABSY },
    { "LDY", Mode::IMM },
    { "LDA", Mode::INDX },
    { "LDX", Mode::IMM },
    { "LAX", Mode::INDX },
    { "LDY", Mode::ZP },
    { "LDA", Mode::ZP },
    { "LDX", Mode::ZP },
    { "LAX", Mode::ZP },
    { "TAY", Mode::IMP },
    { "LDA", Mode::IMM },
    { "TAX", Mode::IMP },
    { "LXA", Mode::IMM },
    { "LDY", Mode::ABS },
    { "LDA", Mode::ABS },
    { "LDX", Mode::ABS },
    { "LAX", Mode::ABS },
    { "BCS", Mode::REL },
    { "LDA", Mode::INDY },
    { "JAM", Mode::IMP },
    { "LAX", Mode::INDY },
    { "LDY", Mode::ZPX },
    { "LDA", Mode::ZPX },
    { "LDX", Mode::ZPY },
    { "LAX", Mode::ZPY },
    { "CLV", Mode::IMP },
    { "LDA", Mode::ABSY },
    { "TSX", Mode::IMP },
    { "LAS", Mode::ABSY },
    { "LDY", Mode::ABSX },
    { "LDA", Mode::ABSX },
    { "LDX", Mode::ABSY },
    { "LAX", Mode::ABSY },
    { "CPY", Mode::IMM },
    { "CMP", Mode::INDX },
    { "NOP", Mode::IMM },
    { "DCP", Mode::INDX },
    { "CPY", Mode::ZP },
    { "CMP", Mode::ZP },
    { "DEC", Mode::ZP },
    { "DCP", Mode::ZP },
    { "INY", Mode::IMP },
    { "CMP", Mode::IMM },
    { "DEX", Mode::IMP },
    { "AXS", Mode::IMM },
    { "CPY", Mode::ABS },
    { "CMP", Mode::ABS },
    { "DEC", Mode::ABS },
    { "DCP", Mode::ABS },
    { "BNE", Mode::REL },
    { "CMP", Mode::INDY },
    { "JAM", Mode::IMP },
    { "DCP", Mode::INDY },
    { "NOP", Mode::ZPX },
    { "CMP", Mode::ZPX },
    { "DEC", Mode::ZPX },
    { "DCP", Mode::ZPX },
    { "CLD", Mode::IMP },
    { "CMP", Mode::ABSY },
    { "NOP", Mode::IMP },
    { "DCP", Mode::ABSY },
    { "NOP", Mode::ABSX },
    { "CMP", Mode::ABSX },
    { "DEC", Mode::ABSX },
    { "DCP", Mode::ABSX },
    { "CPX", Mode::IMM },
    { "SBC", Mode::INDX },
    { "NOP", Mode::IMM },
    { "ISC", Mode::INDX },
    { "CPX", Mode::ZP },
    { "SBC", Mode::ZP },
    { "INC", Mode::ZP },
    { "ISC", Mode::ZP },
    { "INX", Mode::IMP },
    { "SBC", Mode::IMM },
    { "NOP", Mode::IMP },
    { "SBC", Mode::IMM },
    { "CPX", Mode::ABS },
    { "SBC", Mode::ABS },
    { "INC", Mode::ABS },
    { "ISC", Mode::ABS },
    { "BEQ", Mode::REL },
    { "SBC", Mode::INDY },
    { "JAM", Mode::IMP },
    { "ISC", Mode::INDY },
    { "NOP", Mode::ZPX },
    { "SBC", Mode::ZPX },
    { "INC", Mode::ZPX },
    { "ISC", Mode::ZPX },
    { "SED", Mode::IMP },
    { "SBC", Mode::ABSY },
    { "NOP", Mode::IMP },
    { "ISC", Mode::ABSY },
    { "NOP", Mode::ABSX },
    { "SBC", Mode::ABSX },
    { "INC", Mode::ABSX },
    { "ISC", Mode::ABSX },
}};

uint8_t modeLength(Mode mode)
{
    switch (mode) {
    case Mode::IMP:
    case Mode::ACC: return 1;
    case Mode::IMM:
    case Mode::ZP:
    case Mode::ZPX:
    case Mode::ZPY:
    case Mode::INDX:
    case Mode::INDY:
    case Mode::REL: return 2;
    default: return 3;
    }
}

std::string hex2(uint8_t value)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(value));
    return buf;
}

std::string hex4(uint16_t value)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", static_cast<unsigned>(value));
    return buf;
}

}

namespace {

DisassembledInstruction decodeInstruction(uint16_t address, uint8_t opcode, uint8_t b1, uint8_t b2)
{
    DisassembledInstruction result;
    result.address = address;
    result.opcode = opcode;
    const OpcodeInfo& info = kOpcodes[result.opcode];
    result.length = modeLength(info.mode);
    const uint16_t word = static_cast<uint16_t>(b1 | (uint16_t(b2) << 8));

    result.bytes = hex2(result.opcode);
    if (result.length > 1) result.bytes += " " + hex2(b1);
    if (result.length > 2) result.bytes += " " + hex2(b2);

    std::string operand;
    switch (info.mode) {
    case Mode::IMP: break;
    case Mode::ACC: operand = "A"; break;
    case Mode::IMM: operand = "#$" + hex2(b1); break;
    case Mode::ZP: operand = "$" + hex2(b1); break;
    case Mode::ZPX: operand = "$" + hex2(b1) + ",X"; break;
    case Mode::ZPY: operand = "$" + hex2(b1) + ",Y"; break;
    case Mode::ABS: operand = "$" + hex4(word); break;
    case Mode::ABSX: operand = "$" + hex4(word) + ",X"; break;
    case Mode::ABSY: operand = "$" + hex4(word) + ",Y"; break;
    case Mode::IND: operand = "($" + hex4(word) + ")"; break;
    case Mode::INDX: operand = "($" + hex2(b1) + ",X)"; break;
    case Mode::INDY: operand = "($" + hex2(b1) + "),Y"; break;
    case Mode::REL: {
        const int8_t displacement = static_cast<int8_t>(b1);
        const uint16_t target = static_cast<uint16_t>(address + 2 + displacement);
        operand = "$" + hex4(target);
        break;
    }
    }

    result.text = info.mnemonic;
    if (!operand.empty()) result.text += " " + operand;
    return result;
}

}

DisassembledInstruction Debugger::disassemble(uint16_t address) const
{
    const uint8_t opcode = m_bus.debugRead(address);
    const uint8_t b1 = m_bus.debugRead(static_cast<uint16_t>(address + 1));
    const uint8_t b2 = m_bus.debugRead(static_cast<uint16_t>(address + 2));
    return decodeInstruction(address, opcode, b1, b2);
}

std::string Debugger::formatTrace(uint16_t pc, uint8_t opcode, uint8_t operand1, uint8_t operand2,
    uint8_t a, uint8_t x, uint8_t y, uint8_t sp, uint8_t status, uint64_t cpuCycle,
    int ppuScanline, int ppuCycle) const
{
    const DisassembledInstruction inst = decodeInstruction(pc, opcode, operand1, operand2);
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0')
        << std::setw(4) << static_cast<unsigned>(pc) << "  ";

    out << std::left << std::setfill(' ') << std::setw(9) << inst.bytes
        << std::setw(18) << inst.text << std::right << std::setfill('0')
        << " A:" << std::setw(2) << static_cast<unsigned>(a)
        << " X:" << std::setw(2) << static_cast<unsigned>(x)
        << " Y:" << std::setw(2) << static_cast<unsigned>(y)
        << " P:" << std::setw(2) << static_cast<unsigned>(status)
        << " SP:" << std::setw(2) << static_cast<unsigned>(sp)
        << std::dec << " CYC:" << cpuCycle
        << " PPU:" << ppuScanline << "," << ppuCycle;
    return out.str();
}
