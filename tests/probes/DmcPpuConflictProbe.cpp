#include "../../src/core/CPU.hpp"
#include "../../src/core/Bus.hpp"
#include "../../src/core/PPU.hpp"
#include "../../src/core/APU.hpp"
#include "../../src/core/Cartridge.hpp"
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

struct Machine {
    Bus bus;
    CPU cpu{bus};
    PPU ppu;
    APU apu;
    Cartridge cart;
    Machine() {
        bus.connectCPU(&cpu); bus.connectPPU(&ppu); bus.connectAPU(&apu); bus.connectCartridge(&cart);
        ppu.connectCartridge(&cart); ppu.connectCPU(&cpu); cart.connectCPU(&cpu);
        apu.connectBus(&bus); apu.connectCPU(&cpu); apu.connectCartridge(&cart);
    }
};

static bool runUntilRead(Machine& m, uint16_t addr, uint64_t limit = 10000) {
    for (uint64_t i = 0; i < limit; ++i) {
        const CPU::BusCycle c = m.cpu.nextBusCycle();
        if (c.exact && c.type == CPU::BusCycleType::Read && c.address == addr)
            return true;
        m.bus.clock();
    }
    return false;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: DmcPpuConflictProbe <probe.nes>\n";
        return 2;
    }
    Machine m;
    if (!m.cart.loadFromFile(argv[1]) || !m.cart.mapperSupported()) return 2;
    m.bus.powerOn();

    if (!runUntilRead(m, 0x2007)) {
        std::cerr << "did not reach $2007 read\n";
        return 1;
    }
    const uint16_t v0 = m.ppu.testVramAddress();
    const bool align = !m.bus.dmaGetCycle();
    if (!m.bus.requestDmcDma(0xC000)) return 1;
    while (m.bus.dmcDmaActive()) m.bus.clock();
    // DMC GET completed; CPU is still held for that slot. Resume the original read.
    m.bus.clock();
    const uint16_t v1 = m.ppu.testVramAddress();
    const uint16_t expectedDelta = align ? 4 : 3; // halt + dummy + [align] + resumed CPU read
    const uint16_t actualDelta = static_cast<uint16_t>(v1 - v0);
    if (actualDelta != expectedDelta) {
        std::cerr << "$2007 v delta mismatch: got " << actualDelta
                  << " expected " << expectedDelta << "\n";
        return 1;
    }

    if (!runUntilRead(m, 0x2002)) {
        std::cerr << "did not reach $2002 read\n";
        return 1;
    }
    m.ppu.cpuWrite(0x2005, 0x12); // first scroll write sets w=1
    if (!m.ppu.testWriteToggle()) return 1;
    if (!m.bus.requestDmcDma(0xC000)) return 1;
    m.bus.clock(); // DMC halt repeats the stalled $2002 read
    if (m.ppu.testWriteToggle()) {
        std::cerr << "$2002 halt read did not clear PPU write toggle\n";
        return 1;
    }

    std::cout << "$2007_delta=" << actualDelta
              << " align=" << (align ? 1 : 0)
              << " $2002_w_clear=PASS\nPASS\n";
    return 0;
}
