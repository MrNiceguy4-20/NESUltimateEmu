#include "../../src/core/CPU.hpp"
#include "../../src/core/Bus.hpp"
#include "../../src/core/PPU.hpp"
#include "../../src/core/APU.hpp"
#include "../../src/core/Cartridge.hpp"
#include <cstdint>
#include <iostream>
#include <string>

struct Machine {
    Bus bus; CPU cpu{bus}; PPU ppu; APU apu; Cartridge cart;
    Machine() {
        bus.connectCPU(&cpu); bus.connectPPU(&ppu); bus.connectAPU(&apu); bus.connectCartridge(&cart);
        ppu.connectCartridge(&cart); ppu.connectCPU(&cpu); cart.connectCPU(&cpu);
        apu.connectBus(&bus); apu.connectCPU(&cpu); apu.connectCartridge(&cart);
    }
};

static bool reach4000(Machine& m, uint64_t limit=10000) {
    for (uint64_t i=0;i<limit;++i) {
        auto c=m.cpu.nextBusCycle();
        if (c.exact && c.type==CPU::BusCycleType::Read && c.address==0x4000) return true;
        m.bus.clock();
    }
    return false;
}

int runDmcApuConflictProbe(const std::string& romPath) {
    Machine m;
    if (!m.cart.loadFromFile(romPath) || !m.cart.mapperSupported()) return 2;
    m.bus.powerOn();

    // Low 5 DMA bits = $15 while CPU is halted in $4000-$401F must activate $4015.
    if (!reach4000(m)) return 1;
    m.apu.testSetFrameIrqFlag(true);
    m.bus.testSetInternalDataBus(0x00); // D5 must remain clear across DMC external GET
    if (!m.bus.requestDmcDma(0xC015)) return 1;
    while (m.bus.dmcDmaActive()) m.bus.clock();
    // A $4015 activation requests the acknowledge on the DMA GET itself; the
    // frame IRQ latch clears on the following APU GET transition rather than
    // combinationally inside the register read. Allow that transition here,
    // matching AccuracyCoin's double-read phase test.
    if (m.apu.testFrameIrqFlag()) m.bus.clock();
    if (m.apu.testFrameIrqFlag()) {
        std::cerr << "$4015 conflict did not clear frame IRQ\n";
        return 1;
    }
    // The DMA byte ($A5, D5=1) is visible externally, but a conflicting
    // internal $4015 activation must not copy that D5 into the 2A03 internal
    // data latch. Prime the internal latch with D5=0 before the DMA.
    const uint8_t extLatch = m.bus.testExternalDataBus();
    const uint8_t intLatch = m.bus.testInternalDataBus();
    if (extLatch != 0xA5 || (intLatch & 0x20) != 0) {
        std::cerr << "$4015 bus isolation mismatch ext=" << unsigned(extLatch)
                  << " int=" << unsigned(intLatch) << "\n";
        return 1;
    }
    const uint8_t extAfter4015 = m.bus.read(0x4000);
    if (extAfter4015 != 0xA5) {
        std::cerr << "external bus lost DMC sample: " << unsigned(extAfter4015) << "\n";
        return 1;
    }

    // Low 5 DMA bits = $16 activates controller 1 even though the CPU read is $4000.
    m.bus.setController1(0x02); // A=0, B=1
    m.bus.testLatchControllers();
    if (!reach4000(m)) return 1;
    if (!m.bus.requestDmcDma(0xC016)) return 1;
    while (m.bus.dmcDmaActive()) m.bus.clock();
    const uint8_t conflicted = m.bus.read(0x4000); // releases controller select
    const uint8_t next = m.bus.read(0x4016);
    if ((conflicted & 1) != 0 || (next & 1) != 1) {
        std::cerr << "$4016 activation mismatch conflict=" << unsigned(conflicted)
                  << " next=" << unsigned(next) << "\n";
        return 1;
    }

    std::cout << "$4015_irq_clear=PASS external=A5 $4016_activation=PASS\nPASS\n";
    return 0;
}

#ifndef NES_PROBE_SUITE
int main(int argc, char** argv) { return argc == 2 ? runDmcApuConflictProbe(argv[1]) : 2; }
#endif
