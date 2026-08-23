#include "CPU.hpp"
#include "Bus.hpp"
#include "PPU.hpp"
#include "APU.hpp"
#include "Cartridge.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <filesystem>
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

bool writeIrqRom(const std::string& path) {
    std::vector<uint8_t> rom(16 + 0x4000, 0xEA);
    const uint8_t header[16] = {'N','E','S',0x1A,1,0,0,0,0,0,0,0,0,0,0,0};
    for (int i = 0; i < 16; ++i) rom[i] = header[i];
    auto put = [&](uint16_t cpuAddr, std::initializer_list<uint8_t> bytes) {
        size_t off = 16 + (cpuAddr - 0x8000);
        for (uint8_t b : bytes) rom[off++] = b;
    };
    // Enable maskable IRQs, execute harmless instructions, then loop.
    put(0x8000, {0x58, 0xEA, 0xEA, 0x4C, 0x01, 0x80});
    put(0x8100, {0x40}); // NMI: RTI
    put(0x8200, {0x40}); // IRQ: RTI
    const size_t v = 16 + 0x3FFA;
    rom[v+0] = 0x00; rom[v+1] = 0x81;
    rom[v+2] = 0x00; rom[v+3] = 0x80;
    rom[v+4] = 0x00; rom[v+5] = 0x82;
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(rom.data()), static_cast<std::streamsize>(rom.size()));
    return !!out;
}
}

int runInterruptHijackProbe() {
    const std::string brkPath = (std::filesystem::temp_directory_path() / "nesultimate_interrupt_hijack_probe.nes").string();
    if (!writeRom(brkPath)) return 2;

    // Case 1a: NMI arriving before BRK's status-push/vector-selection
    // boundary hijacks the in-flight BRK. The origin is still BRK, so B=1
    // remains in the pushed status even though the NMI vector is selected.
    Machine brk;
    if (!brk.cart.loadFromFile(brkPath) || !brk.cart.mapperSupported()) return 2;
    brk.bus.powerOn();

    bool brkInjected = false;
    bool brkRedirectedVector = false;
    bool brkPushedBreak = false;
    for (uint64_t i = 0; i < 2000; ++i) {
        const CPU::BusCycle before = brk.cpu.nextBusCycle();
        // BRK cycle 5 is the status push. Inject before that cycle so vector
        // selection must resolve to NMI.
        if (!brkInjected && before.type == CPU::BusCycleType::Write &&
            before.address == 0x01FB && (before.data & 0x10) != 0) {
            brk.cpu.nmi();
            brk.cpu.sampleNmiInput();
            brkInjected = true;
            brk.bus.clock();
            brkPushedBreak = (brk.bus.read(0x01FB) & 0x10) != 0;
            const CPU::BusCycle after = brk.cpu.nextBusCycle();
            brkRedirectedVector =
                after.type == CPU::BusCycleType::Read && after.address == 0xFFFA;
            std::printf("brk_before_status_next=%04X redirected=%d B=%d\n",
                after.address, brkRedirectedVector ? 1 : 0, brkPushedBreak ? 1 : 0);
            break;
        }
        brk.bus.clock();
    }

    // Case 1b: once BRK reaches its low-vector fetch, vector selection is
    // committed. A new NMI edge here must not half/full-hijack this BRK.
    Machine brkLate;
    if (!brkLate.cart.loadFromFile(brkPath) || !brkLate.cart.mapperSupported()) return 2;
    brkLate.bus.powerOn();
    bool brkLateInjected = false;
    bool brkLateStayedCommitted = false;
    for (uint64_t i = 0; i < 2000; ++i) {
        const CPU::BusCycle before = brkLate.cpu.nextBusCycle();
        if (!brkLateInjected && before.type == CPU::BusCycleType::Read && before.address == 0xFFFE) {
            brkLate.cpu.nmi();
            brkLate.cpu.sampleNmiInput();
            brkLateInjected = true;
            brkLate.bus.clock();
            const CPU::BusCycle after = brkLate.cpu.nextBusCycle();
            brkLateStayedCommitted =
                after.type == CPU::BusCycleType::Read && after.address == 0xFFFF;
            std::printf("brk_after_low_vector=%04X committed=%d\n",
                after.address, brkLateStayedCommitted ? 1 : 0);
            break;
        }
        brkLate.bus.clock();
    }

    // Case 2: hardware IRQ has an earlier vector-commit point. An NMI edge
    // arriving at IRQ's $FFFE low-vector boundary must not redirect that
    // in-flight IRQ; it remains pending for the following instruction boundary.
    const std::string irqPath = (std::filesystem::temp_directory_path() / "nesultimate_irq_commit_probe.nes").string();
    if (!writeIrqRom(irqPath)) return 2;
    Machine irq;
    if (!irq.cart.loadFromFile(irqPath) || !irq.cart.mapperSupported()) return 2;
    irq.bus.powerOn();
    irq.cpu.setIrqLine(true);

    bool irqInjected = false;
    bool irqStayedCommitted = false;
    for (uint64_t i = 0; i < 4000; ++i) {
        const CPU::BusCycle before = irq.cpu.nextBusCycle();
        if (!irqInjected && before.type == CPU::BusCycleType::Read && before.address == 0xFFFE) {
            irq.cpu.nmi();
            irq.cpu.sampleNmiInput();
            irqInjected = true;
            irq.cpu.clock();
            const CPU::BusCycle after = irq.cpu.nextBusCycle();
            irqStayedCommitted =
                after.type == CPU::BusCycleType::Read && after.address == 0xFFFF;
            std::printf("irq_after_low_vector=%04X committed=%d\n",
                after.address, irqStayedCommitted ? 1 : 0);
            break;
        }
        irq.cpu.clock();
    }

    const bool ok = brkInjected && brkRedirectedVector && brkPushedBreak &&
                    brkLateInjected && brkLateStayedCommitted &&
                    irqInjected && irqStayedCommitted;
    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runInterruptHijackProbe(); }
#endif
