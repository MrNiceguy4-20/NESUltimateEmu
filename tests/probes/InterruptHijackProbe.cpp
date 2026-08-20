#include "CPU.hpp"
#include "Bus.hpp"
#include "PPU.hpp"
#include "APU.hpp"
#include "Cartridge.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
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

bool writeRom(const std::string& path) {
    std::vector<uint8_t> rom(16 + 0x4000, 0xEA);
    const uint8_t header[16] = {'N','E','S',0x1A,1,0,0,0,0,0,0,0,0,0,0,0};
    for (int i = 0; i < 16; ++i) rom[i] = header[i];
    auto put = [&](uint16_t cpuAddr, std::initializer_list<uint8_t> bytes) {
        size_t off = 16 + (cpuAddr - 0x8000);
        for (uint8_t b : bytes) rom[off++] = b;
    };
    // Main: SEI; BRK; padding; forever.
    put(0x8000, {0x78, 0x00, 0xEA, 0x4C, 0x03, 0x80});
    // NMI handler: LDA #$42; STA $6000; forever.
    put(0x8100, {0xA9,0x42,0x8D,0x00,0x60,0x4C,0x05,0x81});
    // IRQ/BRK handler: LDA #$99; STA $6000; forever.
    put(0x8200, {0xA9,0x99,0x8D,0x00,0x60,0x4C,0x05,0x82});
    // Vectors at mirrored top of 16K PRG.
    const size_t v = 16 + 0x3FFA;
    rom[v+0] = 0x00; rom[v+1] = 0x81; // NMI $8100
    rom[v+2] = 0x00; rom[v+3] = 0x80; // RESET $8000
    rom[v+4] = 0x00; rom[v+5] = 0x82; // IRQ/BRK $8200
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(rom.data()), static_cast<std::streamsize>(rom.size()));
    return !!out;
}
}

int main() {
    const std::string path = "/tmp/nesultimate_interrupt_hijack_probe.nes";
    if (!writeRom(path)) return 2;

    Machine m;
    if (!m.cart.loadFromFile(path) || !m.cart.mapperSupported()) return 2;
    m.bus.powerOn();

    bool injected = false;
    bool redirectedHighVector = false;
    for (uint64_t i = 0; i < 2000; ++i) {
        const CPU::BusCycle before = m.cpu.nextBusCycle();
        if (!injected && before.type == CPU::BusCycleType::Read && before.address == 0xFFFE) {
            // NMI becomes pending exactly as the BRK low-vector bus cycle is
            // about to start. Hardware must redirect this fetch to $FFFA.
            m.cpu.nmi();
            injected = true;
            m.bus.clock();
            const CPU::BusCycle after = m.cpu.nextBusCycle();
            redirectedHighVector = after.type == CPU::BusCycleType::Read && after.address == 0xFFFB;
            std::printf("after_low_vector=%04X redirected=%d\n", after.address, redirectedHighVector ? 1 : 0);
            break;
        }
        m.bus.clock();
    }

    const bool ok = injected && redirectedHighVector;
    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
