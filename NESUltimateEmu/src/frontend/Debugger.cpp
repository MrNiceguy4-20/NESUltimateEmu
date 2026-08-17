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
    { "BRK", Mode::IMP }, // 00
    { "ORA", Mode::INDX }, // 01
    { "JAM", Mode::IMP }, // 02
    { "SLO", Mode::INDX }, // 03
    { "NOP", Mode::ZP }, // 04
    { "ORA", Mode::ZP }, // 05
    { "ASL", Mode::ZP }, // 06
    { "SLO", Mode::ZP }, // 07
    { "PHP", Mode::IMP }, // 08
    { "ORA", Mode::IMM }, // 09
    { "ASL", Mode::ACC }, // 0A
    { "ANC", Mode::IMM }, // 0B
    { "NOP", Mode::ABS }, // 0C
    { "ORA", Mode::ABS }, // 0D
    { "ASL", Mode::ABS }, // 0E
    { "SLO", Mode::ABS }, // 0F
    { "BPL", Mode::REL }, // 10
    { "ORA", Mode::INDY }, // 11
    { "JAM", Mode::IMP }, // 12
    { "SLO", Mode::INDY }, // 13
    { "NOP", Mode::ZPX }, // 14
    { "ORA", Mode::ZPX }, // 15
    { "ASL", Mode::ZPX }, // 16
    { "SLO", Mode::ZPX }, // 17
    { "CLC", Mode::IMP }, // 18
    { "ORA", Mode::ABSY }, // 19
    { "NOP", Mode::IMP }, // 1A
    { "SLO", Mode::ABSY }, // 1B
    { "NOP", Mode::ABSX }, // 1C
    { "ORA", Mode::ABSX }, // 1D
    { "ASL", Mode::ABSX }, // 1E
    { "SLO", Mode::ABSX }, // 1F
    { "JSR", Mode::ABS }, // 20
    { "AND", Mode::INDX }, // 21
    { "JAM", Mode::IMP }, // 22
    { "RLA", Mode::INDX }, // 23
    { "BIT", Mode::ZP }, // 24
    { "AND", Mode::ZP }, // 25
    { "ROL", Mode::ZP }, // 26
    { "RLA", Mode::ZP }, // 27
    { "PLP", Mode::IMP }, // 28
    { "AND", Mode::IMM }, // 29
    { "ROL", Mode::ACC }, // 2A
    { "ANC", Mode::IMM }, // 2B
    { "BIT", Mode::ABS }, // 2C
    { "AND", Mode::ABS }, // 2D
    { "ROL", Mode::ABS }, // 2E
    { "RLA", Mode::ABS }, // 2F
    { "BMI", Mode::REL }, // 30
    { "AND", Mode::INDY }, // 31
    { "JAM", Mode::IMP }, // 32
    { "RLA", Mode::INDY }, // 33
    { "NOP", Mode::ZPX }, // 34
    { "AND", Mode::ZPX }, // 35
    { "ROL", Mode::ZPX }, // 36
    { "RLA", Mode::ZPX }, // 37
    { "SEC", Mode::IMP }, // 38
    { "AND", Mode::ABSY }, // 39
    { "NOP", Mode::IMP }, // 3A
    { "RLA", Mode::ABSY }, // 3B
    { "NOP", Mode::ABSX }, // 3C
    { "AND", Mode::ABSX }, // 3D
    { "ROL", Mode::ABSX }, // 3E
    { "RLA", Mode::ABSX }, // 3F
    { "RTI", Mode::IMP }, // 40
    { "EOR", Mode::INDX }, // 41
    { "JAM", Mode::IMP }, // 42
    { "SRE", Mode::INDX }, // 43
    { "NOP", Mode::ZP }, // 44
    { "EOR", Mode::ZP }, // 45
    { "LSR", Mode::ZP }, // 46
    { "SRE", Mode::ZP }, // 47
    { "PHA", Mode::IMP }, // 48
    { "EOR", Mode::IMM }, // 49
    { "LSR", Mode::ACC }, // 4A
    { "ALR", Mode::IMM }, // 4B
    { "JMP", Mode::ABS }, // 4C
    { "EOR", Mode::ABS }, // 4D
    { "LSR", Mode::ABS }, // 4E
    { "SRE", Mode::ABS }, // 4F
    { "BVC", Mode::REL }, // 50
    { "EOR", Mode::INDY }, // 51
    { "JAM", Mode::IMP }, // 52
    { "SRE", Mode::INDY }, // 53
    { "NOP", Mode::ZPX }, // 54
    { "EOR", Mode::ZPX }, // 55
    { "LSR", Mode::ZPX }, // 56
    { "SRE", Mode::ZPX }, // 57
    { "CLI", Mode::IMP }, // 58
    { "EOR", Mode::ABSY }, // 59
    { "NOP", Mode::IMP }, // 5A
    { "SRE", Mode::ABSY }, // 5B
    { "NOP", Mode::ABSX }, // 5C
    { "EOR", Mode::ABSX }, // 5D
    { "LSR", Mode::ABSX }, // 5E
    { "SRE", Mode::ABSX }, // 5F
    { "RTS", Mode::IMP }, // 60
    { "ADC", Mode::INDX }, // 61
    { "JAM", Mode::IMP }, // 62
    { "RRA", Mode::INDX }, // 63
    { "NOP", Mode::ZP }, // 64
    { "ADC", Mode::ZP }, // 65
    { "ROR", Mode::ZP }, // 66
    { "RRA", Mode::ZP }, // 67
    { "PLA", Mode::IMP }, // 68
    { "ADC", Mode::IMM }, // 69
    { "ROR", Mode::ACC }, // 6A
    { "ARR", Mode::IMM }, // 6B
    { "JMP", Mode::IND }, // 6C
    { "ADC", Mode::ABS }, // 6D
    { "ROR", Mode::ABS }, // 6E
    { "RRA", Mode::ABS }, // 6F
    { "BVS", Mode::REL }, // 70
    { "ADC", Mode::INDY }, // 71
    { "JAM", Mode::IMP }, // 72
    { "RRA", Mode::INDY }, // 73
    { "NOP", Mode::ZPX }, // 74
    { "ADC", Mode::ZPX }, // 75
    { "ROR", Mode::ZPX }, // 76
    { "RRA", Mode::ZPX }, // 77
    { "SEI", Mode::IMP }, // 78
    { "ADC", Mode::ABSY }, // 79
    { "NOP", Mode::IMP }, // 7A
    { "RRA", Mode::ABSY }, // 7B
    { "NOP", Mode::ABSX }, // 7C
    { "ADC", Mode::ABSX }, // 7D
    { "ROR", Mode::ABSX }, // 7E
    { "RRA", Mode::ABSX }, // 7F
    { "NOP", Mode::IMM }, // 80
    { "STA", Mode::INDX }, // 81
    { "NOP", Mode::IMM }, // 82
    { "SAX", Mode::INDX }, // 83
    { "STY", Mode::ZP }, // 84
    { "STA", Mode::ZP }, // 85
    { "STX", Mode::ZP }, // 86
    { "SAX", Mode::ZP }, // 87
    { "DEY", Mode::IMP }, // 88
    { "NOP", Mode::IMM }, // 89
    { "TXA", Mode::IMP }, // 8A
    { "ANE", Mode::IMM }, // 8B
    { "STY", Mode::ABS }, // 8C
    { "STA", Mode::ABS }, // 8D
    { "STX", Mode::ABS }, // 8E
    { "SAX", Mode::ABS }, // 8F
    { "BCC", Mode::REL }, // 90
    { "STA", Mode::INDY }, // 91
    { "JAM", Mode::IMP }, // 92
    { "SHA", Mode::INDY }, // 93
    { "STY", Mode::ZPX }, // 94
    { "STA", Mode::ZPX }, // 95
    { "STX", Mode::ZPY }, // 96
    { "SAX", Mode::ZPY }, // 97
    { "TYA", Mode::IMP }, // 98
    { "STA", Mode::ABSY }, // 99
    { "TXS", Mode::IMP }, // 9A
    { "TAS", Mode::ABSY }, // 9B
    { "SHY", Mode::ABSX }, // 9C
    { "STA", Mode::ABSX }, // 9D
    { "SHX", Mode::ABSY }, // 9E
    { "SHA", Mode::ABSY }, // 9F
    { "LDY", Mode::IMM }, // A0
    { "LDA", Mode::INDX }, // A1
    { "LDX", Mode::IMM }, // A2
    { "LAX", Mode::INDX }, // A3
    { "LDY", Mode::ZP }, // A4
    { "LDA", Mode::ZP }, // A5
    { "LDX", Mode::ZP }, // A6
    { "LAX", Mode::ZP }, // A7
    { "TAY", Mode::IMP }, // A8
    { "LDA", Mode::IMM }, // A9
    { "TAX", Mode::IMP }, // AA
    { "LXA", Mode::IMM }, // AB
    { "LDY", Mode::ABS }, // AC
    { "LDA", Mode::ABS }, // AD
    { "LDX", Mode::ABS }, // AE
    { "LAX", Mode::ABS }, // AF
    { "BCS", Mode::REL }, // B0
    { "LDA", Mode::INDY }, // B1
    { "JAM", Mode::IMP }, // B2
    { "LAX", Mode::INDY }, // B3
    { "LDY", Mode::ZPX }, // B4
    { "LDA", Mode::ZPX }, // B5
    { "LDX", Mode::ZPY }, // B6
    { "LAX", Mode::ZPY }, // B7
    { "CLV", Mode::IMP }, // B8
    { "LDA", Mode::ABSY }, // B9
    { "TSX", Mode::IMP }, // BA
    { "LAS", Mode::ABSY }, // BB
    { "LDY", Mode::ABSX }, // BC
    { "LDA", Mode::ABSX }, // BD
    { "LDX", Mode::ABSY }, // BE
    { "LAX", Mode::ABSY }, // BF
    { "CPY", Mode::IMM }, // C0
    { "CMP", Mode::INDX }, // C1
    { "NOP", Mode::IMM }, // C2
    { "DCP", Mode::INDX }, // C3
    { "CPY", Mode::ZP }, // C4
    { "CMP", Mode::ZP }, // C5
    { "DEC", Mode::ZP }, // C6
    { "DCP", Mode::ZP }, // C7
    { "INY", Mode::IMP }, // C8
    { "CMP", Mode::IMM }, // C9
    { "DEX", Mode::IMP }, // CA
    { "AXS", Mode::IMM }, // CB
    { "CPY", Mode::ABS }, // CC
    { "CMP", Mode::ABS }, // CD
    { "DEC", Mode::ABS }, // CE
    { "DCP", Mode::ABS }, // CF
    { "BNE", Mode::REL }, // D0
    { "CMP", Mode::INDY }, // D1
    { "JAM", Mode::IMP }, // D2
    { "DCP", Mode::INDY }, // D3
    { "NOP", Mode::ZPX }, // D4
    { "CMP", Mode::ZPX }, // D5
    { "DEC", Mode::ZPX }, // D6
    { "DCP", Mode::ZPX }, // D7
    { "CLD", Mode::IMP }, // D8
    { "CMP", Mode::ABSY }, // D9
    { "NOP", Mode::IMP }, // DA
    { "DCP", Mode::ABSY }, // DB
    { "NOP", Mode::ABSX }, // DC
    { "CMP", Mode::ABSX }, // DD
    { "DEC", Mode::ABSX }, // DE
    { "DCP", Mode::ABSX }, // DF
    { "CPX", Mode::IMM }, // E0
    { "SBC", Mode::INDX }, // E1
    { "NOP", Mode::IMM }, // E2
    { "ISC", Mode::INDX }, // E3
    { "CPX", Mode::ZP }, // E4
    { "SBC", Mode::ZP }, // E5
    { "INC", Mode::ZP }, // E6
    { "ISC", Mode::ZP }, // E7
    { "INX", Mode::IMP }, // E8
    { "SBC", Mode::IMM }, // E9
    { "NOP", Mode::IMP }, // EA
    { "SBC", Mode::IMM }, // EB
    { "CPX", Mode::ABS }, // EC
    { "SBC", Mode::ABS }, // ED
    { "INC", Mode::ABS }, // EE
    { "ISC", Mode::ABS }, // EF
    { "BEQ", Mode::REL }, // F0
    { "SBC", Mode::INDY }, // F1
    { "JAM", Mode::IMP }, // F2
    { "ISC", Mode::INDY }, // F3
    { "NOP", Mode::ZPX }, // F4
    { "SBC", Mode::ZPX }, // F5
    { "INC", Mode::ZPX }, // F6
    { "ISC", Mode::ZPX }, // F7
    { "SED", Mode::IMP }, // F8
    { "SBC", Mode::ABSY }, // F9
    { "NOP", Mode::IMP }, // FA
    { "ISC", Mode::ABSY }, // FB
    { "NOP", Mode::ABSX }, // FC
    { "SBC", Mode::ABSX }, // FD
    { "INC", Mode::ABSX }, // FE
    { "ISC", Mode::ABSX }, // FF
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

} // namespace

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

} // namespace

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
