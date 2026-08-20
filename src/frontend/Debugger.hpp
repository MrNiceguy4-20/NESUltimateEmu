#pragma once
#include <cstdint>
#include <string>

class Bus;

struct DisassembledInstruction {
    uint16_t address = 0;
    uint8_t opcode = 0;
    uint8_t length = 1;
    std::string bytes;
    std::string text;
};

class Debugger {
public:
    explicit Debugger(const Bus& bus) : m_bus(bus) {}

    DisassembledInstruction disassemble(uint16_t address) const;
    std::string formatTrace(uint16_t pc, uint8_t opcode, uint8_t operand1, uint8_t operand2,
        uint8_t a, uint8_t x, uint8_t y, uint8_t sp, uint8_t status, uint64_t cpuCycle,
        int ppuScanline, int ppuCycle) const;

private:
    const Bus& m_bus;
};
