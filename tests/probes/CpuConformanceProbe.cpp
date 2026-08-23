#include "CPU.hpp"
#include "Bus.hpp"
#include "PPU.hpp"
#include "APU.hpp"
#include "Cartridge.hpp"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <string>
#include <vector>

namespace {
struct Machine {
    Bus bus;
    CPU cpu{bus};
    PPU ppu;
    APU apu;
    Cartridge cart;

    Machine() {
        bus.connectCPU(&cpu);
        bus.connectPPU(&ppu);
        bus.connectAPU(&apu);
        bus.connectCartridge(&cart);
        ppu.connectCartridge(&cart);
        ppu.connectCPU(&cpu);
        cart.connectCPU(&cpu);
        apu.connectBus(&bus);
        apu.connectCPU(&cpu);
        apu.connectCartridge(&cart);
    }
};

struct CpuState {
    uint8_t a = 0;
    uint8_t x = 0;
    uint8_t y = 0;
    uint8_t sp = 0;
    uint16_t pc = 0;
    uint8_t p = 0;
};

CpuState stateOf(const CPU& cpu)
{
    std::vector<uint8_t> data;
    cpu.saveState(data);
    CpuState s;
    if (data.size() >= 7) {
        s.a = data[0];
        s.x = data[1];
        s.y = data[2];
        s.sp = data[3];
        s.pc = static_cast<uint16_t>(data[4]) |
               (static_cast<uint16_t>(data[5]) << 8);
        s.p = data[6];
    }
    return s;
}

bool writeRom(const std::filesystem::path& path, uint16_t reset,
              std::initializer_list<std::pair<uint16_t, std::vector<uint8_t>>> chunks)
{
    std::vector<uint8_t> rom(16 + 0x4000, 0xEA);
    const uint8_t header[16] = {'N','E','S',0x1A,1,0,0,0,0,0,0,0,0,0,0,0};
    for (int i = 0; i < 16; ++i) rom[i] = header[i];
    for (const auto& chunk : chunks) {
        size_t off = 16 + ((chunk.first - 0x8000) & 0x3FFF);
        for (uint8_t byte : chunk.second)
            rom[off++] = byte;
    }
    const size_t vectors = 16 + 0x3FFA;
    rom[vectors + 0] = static_cast<uint8_t>(reset & 0xFF);
    rom[vectors + 1] = static_cast<uint8_t>(reset >> 8);
    rom[vectors + 2] = static_cast<uint8_t>(reset & 0xFF);
    rom[vectors + 3] = static_cast<uint8_t>(reset >> 8);
    rom[vectors + 4] = static_cast<uint8_t>(reset & 0xFF);
    rom[vectors + 5] = static_cast<uint8_t>(reset >> 8);
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(rom.data()), static_cast<std::streamsize>(rom.size()));
    return !!out;
}

bool loadAndReset(Machine& m, const std::filesystem::path& path)
{
    if (!m.cart.loadFromFile(path.string()) || !m.cart.mapperSupported())
        return false;
    m.bus.powerOn();
    for (int i = 0; i < 7; ++i)
        m.cpu.clock();
    return m.cpu.atInstructionBoundary();
}

int runInstruction(CPU& cpu, std::vector<CPU::BusCycle>* trace = nullptr)
{
    int cycles = 0;
    do {
        if (trace) trace->push_back(cpu.nextBusCycle());
        cpu.clock();
        ++cycles;
        if (cycles > 16) return cycles;
    } while (!cpu.atInstructionBoundary());
    return cycles;
}

bool testImpliedTiming()
{
    const auto path = std::filesystem::temp_directory_path() / "nesultimate_cpu_implied_probe.nes";
    if (!writeRom(path, 0x8000, {{0x8000, {0xA2,0x10,0xE8,0xA9,0x81,0x0A,0xEA}}}))
        return false;
    Machine m;
    if (!loadAndReset(m, path)) return false;

    if (runInstruction(m.cpu) != 2 || stateOf(m.cpu).x != 0x10)
        return false;

    m.cpu.clock();
    const CpuState afterFetch = stateOf(m.cpu);
    const CPU::BusCycle second = m.cpu.nextBusCycle();
    const bool delayedInx = afterFetch.x == 0x10 &&
        second.exact && second.dummy && second.type == CPU::BusCycleType::Read &&
        second.address == 0x8003;
    m.cpu.clock();
    const bool finishedInx = stateOf(m.cpu).x == 0x11 && m.cpu.atInstructionBoundary();

    if (runInstruction(m.cpu) != 2 || stateOf(m.cpu).a != 0x81)
        return false;

    m.cpu.clock();
    const CpuState aslFetch = stateOf(m.cpu);
    const CPU::BusCycle aslSecond = m.cpu.nextBusCycle();
    m.cpu.clock();
    const CpuState aslDone = stateOf(m.cpu);
    const bool delayedAsl = aslFetch.a == 0x81 && (aslFetch.p & 0x01) == 0;
    const bool aslBus = aslSecond.exact && aslSecond.dummy &&
        aslSecond.type == CPU::BusCycleType::Read && aslSecond.address == 0x8006;
    const bool finishedAsl = aslDone.a == 0x02 && (aslDone.p & 0x01) != 0;

    std::printf("implied_delay=%s asl_delay=%s asl_bus=%s x_fetch=%02X x_done=%02X second=%04X asl_second=%04X boundary=%d\n",
        (delayedInx && finishedInx) ? "PASS" : "FAIL",
        (delayedAsl && finishedAsl) ? "PASS" : "FAIL", aslBus ? "PASS" : "FAIL",
        afterFetch.x, stateOf(m.cpu).x, second.address, aslSecond.address, m.cpu.atInstructionBoundary() ? 1 : 0);
    return delayedInx && finishedInx && delayedAsl && aslBus && finishedAsl;
}

bool testUnstableStoreRdy()
{
    const auto path = std::filesystem::temp_directory_path() / "nesultimate_cpu_shx_rdy_probe.nes";

    if (!writeRom(path, 0x8000, {{0x8000, {0xA2,0xA5,0xA0,0x00,0x9E,0x00,0x05,0xEA}}}))
        return false;

    Machine normal;
    if (!loadAndReset(normal, path)) return false;
    if (runInstruction(normal.cpu) != 2 || runInstruction(normal.cpu) != 2) return false;
    normal.bus.write(0x0500, 0x5A);
    if (runInstruction(normal.cpu) != 5) return false;
    const uint8_t normalValue = normal.bus.read(0x0500);

    Machine stalled;
    if (!loadAndReset(stalled, path)) return false;
    if (runInstruction(stalled.cpu) != 2 || runInstruction(stalled.cpu) != 2) return false;
    stalled.bus.write(0x0500, 0x5A);

    bool dmaInjected = false;
    int clocks = 0;
    do {
        const CPU::BusCycle c = stalled.cpu.nextBusCycle();
        if (!dmaInjected && c.exact && c.dummy && c.type == CPU::BusCycleType::Read && c.address == 0x0500) {
            if (!stalled.bus.requestDmcDma(0x8000)) return false;
            dmaInjected = true;
        }

        if (dmaInjected) stalled.bus.clock();
        else stalled.cpu.clock();
        ++clocks;
        if (clocks > 16) return false;
    } while (!stalled.cpu.atInstructionBoundary());

    const uint8_t stalledValue = stalled.bus.read(0x0500);
    const bool ok = normalValue == 0x04 && dmaInjected && stalledValue == 0xA5;
    std::printf("unstable_store_rdy=%s normal=%02X stalled=%02X dma=%d clocks=%d\n",
        ok ? "PASS" : "FAIL", normalValue, stalledValue, dmaInjected ? 1 : 0, clocks);
    return ok;
}

bool testJmpIndirectWrap()
{
    const auto path = std::filesystem::temp_directory_path() / "nesultimate_cpu_jmp_wrap_probe.nes";
    if (!writeRom(path, 0x8000, {{0x8000, {0x6C,0xFF,0x02}}, {0x8134, {0xEA}}}))
        return false;
    Machine m;
    if (!loadAndReset(m, path)) return false;
    m.bus.write(0x02FF, 0x34);
    m.bus.write(0x0200, 0x81);
    m.bus.write(0x0300, 0x99);

    std::vector<CPU::BusCycle> trace;
    const int cycles = runInstruction(m.cpu, &trace);
    const CpuState s = stateOf(m.cpu);
    const uint16_t expected[] = {0x8000,0x8001,0x8002,0x02FF,0x0200};
    bool busOk = trace.size() == 5;
    for (size_t i = 0; busOk && i < 5; ++i)
        busOk = trace[i].exact && trace[i].address == expected[i];

    std::printf("jmp_indirect_cycles=%d pc=%04X wrap_bus=%s\n",
        cycles, s.pc, busOk ? "PASS" : "FAIL");
    return cycles == 5 && s.pc == 0x8134 && busOk;
}

bool testBranchCycles()
{
    const auto path = std::filesystem::temp_directory_path() / "nesultimate_cpu_branch_probe.nes";
    if (!writeRom(path, 0x8000, {{0x8000, {0xA9,0x00,0xF0,0x02,0xEA,0xEA,0xEA}}}))
        return false;
    Machine m;
    if (!loadAndReset(m, path)) return false;
    if (runInstruction(m.cpu) != 2) return false;

    std::vector<CPU::BusCycle> trace;
    const int cycles = runInstruction(m.cpu, &trace);
    const CpuState s = stateOf(m.cpu);
    const bool traceOk = trace.size() == 3 &&
        trace[0].address == 0x8002 && trace[1].address == 0x8003 &&
        trace[2].address == 0x8004 && trace[0].exact && trace[1].exact && trace[2].exact;
    std::printf("branch_taken_cycles=%d pc=%04X bus=%s\n",
        cycles, s.pc, traceOk ? "PASS" : "FAIL");
    return cycles == 3 && s.pc == 0x8006 && traceOk;
}
}

bool testAllOpcodeBusCyclesExact()
{
    const auto path = std::filesystem::temp_directory_path() / "nesultimate_cpu_exact_bus_probe.nes";
    if (!writeRom(path, 0x0200, {}))
        return false;

    static constexpr uint8_t jamOpcodes[] = {
        0x02,0x12,0x22,0x32,0x42,0x52,0x62,0x72,0x92,0xB2,0xD2,0xF2
    };
    auto isJam = [](uint8_t opcode) {
        for (uint8_t jam : jamOpcodes)
            if (opcode == jam) return true;
        return false;
    };

    Machine m;
    if (!m.cart.loadFromFile(path.string()) || !m.cart.mapperSupported())
        return false;

    std::vector<uint8_t> inexactOpcodes;
    std::vector<uint8_t> jamExactUnexpected;
    size_t checkedCycles = 0;

    for (int op = 0; op < 256; ++op) {
        const uint8_t opcode = static_cast<uint8_t>(op);
        m.bus.powerOn();
        for (int i = 0; i < 7; ++i)
            m.cpu.clock();
        if (!m.cpu.atInstructionBoundary())
            return false;

        m.bus.write(0x0200, opcode);

        m.bus.write(0x0201, 0x00);
        m.bus.write(0x0202, 0x00);
        m.bus.write(0x0000, 0x00);
        m.bus.write(0x0001, 0x00);

        if (isJam(opcode)) {
            const CPU::BusCycle fetch = m.cpu.nextBusCycle();
            if (!fetch.exact)
                jamExactUnexpected.push_back(opcode);
            m.cpu.clock();
            const CPU::BusCycle halted = m.cpu.nextBusCycle();
            if (halted.exact)
                jamExactUnexpected.push_back(opcode);
            continue;
        }

        bool exact = true;
        int cycles = 0;
        do {
            const CPU::BusCycle c = m.cpu.nextBusCycle();
            exact = exact && c.exact;
            ++checkedCycles;
            m.cpu.clock();
            ++cycles;
            if (cycles > 16) {
                exact = false;
                break;
            }
        } while (!m.cpu.atInstructionBoundary());

        if (!exact)
            inexactOpcodes.push_back(opcode);
    }

    std::printf("exact_bus_all_opcodes=%s executable=%zu cycles=%zu jam=%zu\n",
        (inexactOpcodes.empty() && jamExactUnexpected.empty()) ? "PASS" : "FAIL",
        static_cast<size_t>(256 - sizeof(jamOpcodes)), checkedCycles,
        sizeof(jamOpcodes));
    if (!inexactOpcodes.empty()) {
        std::printf("inexact executable opcodes:");
        for (uint8_t opcode : inexactOpcodes) std::printf(" %02X", opcode);
        std::puts("");
    }
    if (!jamExactUnexpected.empty()) {
        std::printf("unexpected JAM exactness:");
        for (uint8_t opcode : jamExactUnexpected) std::printf(" %02X", opcode);
        std::puts("");
    }
    return inexactOpcodes.empty() && jamExactUnexpected.empty();
}

int runCpuConformanceProbe()
{
    const bool implied = testImpliedTiming();
    const bool jmpWrap = testJmpIndirectWrap();
    const bool branch = testBranchCycles();
    const bool unstableRdy = testUnstableStoreRdy();
    const bool exactBus = testAllOpcodeBusCyclesExact();
    const bool ok = implied && jmpWrap && branch && unstableRdy && exactBus;
    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runCpuConformanceProbe(); }
#endif
