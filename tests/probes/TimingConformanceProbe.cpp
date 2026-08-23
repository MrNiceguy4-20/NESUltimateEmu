#include "ProbeSuite.hpp"
#include "../../src/core/APU.hpp"
#include "../../src/core/Bus.hpp"
#include "../../src/core/Cartridge.hpp"
#include "../../src/core/PPU.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace {
bool writeTimingRom(const std::filesystem::path& path, uint8_t timingBits)
{
    std::vector<uint8_t> image(16 + 0x4000, 0);
    image[0] = 'N'; image[1] = 'E'; image[2] = 'S'; image[3] = 0x1A;
    image[4] = 1;
    image[5] = 0;
    image[7] = 0x08;
    image[11] = 0x07;
    image[12] = timingBits & 3;

    image[16 + 0x3FFC] = 0x00;
    image[16 + 0x3FFD] = 0x80;
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
    return bool(f);
}

uint64_t measureFullFrame(PPU& ppu)
{

    while (!ppu.frameComplete()) ppu.clock();
    ppu.clearFrameComplete();
    uint64_t clocks = 0;
    do { ppu.clock(); ++clocks; } while (!ppu.frameComplete());
    ppu.clearFrameComplete();
    return clocks;
}
}

int runTimingConformanceProbe()
{
    bool pass = true;

    const auto base = std::filesystem::temp_directory_path();
    const auto ntscPath = base / "nesultimate_timing_ntsc.nes";
    const auto palPath = base / "nesultimate_timing_pal.nes";
    const auto multiPath = base / "nesultimate_timing_multi.nes";
    const auto dendyPath = base / "nesultimate_timing_dendy.nes";
    writeTimingRom(ntscPath, 0); writeTimingRom(palPath, 1);
    writeTimingRom(multiPath, 2); writeTimingRom(dendyPath, 3);

    Cartridge ntsc, pal, multi, dendy;
    const bool parsed = ntsc.loadFromFile(ntscPath.string()) && pal.loadFromFile(palPath.string()) &&
        multi.loadFromFile(multiPath.string()) && dendy.loadFromFile(dendyPath.string());
    const bool headerPass = parsed && ntsc.timing() == ConsoleTiming::NTSC &&
        pal.timing() == ConsoleTiming::PAL && dendy.timing() == ConsoleTiming::Dendy &&
        multi.timing() == ConsoleTiming::NTSC && multi.isMultiRegion();
    std::cout << "nes20_timing_parse=" << (headerPass ? "PASS" : "FAIL") << "\n";
    pass &= headerPass;

    PPU ppuNtsc, ppuPal, ppuDendy;
    ppuNtsc.setTiming(ConsoleTiming::NTSC);
    ppuPal.setTiming(ConsoleTiming::PAL);
    ppuDendy.setTiming(ConsoleTiming::Dendy);
    ppuNtsc.powerOn(); ppuPal.powerOn(); ppuDendy.powerOn();
    const uint64_t ntscFrame = measureFullFrame(ppuNtsc);
    const uint64_t palFrame = measureFullFrame(ppuPal);
    const uint64_t dendyFrame = measureFullFrame(ppuDendy);
    const bool framePass = ntscFrame == 341ull * 262ull &&
        palFrame == 341ull * 312ull && dendyFrame == 341ull * 312ull;
    std::cout << "ppu_frames ntsc=" << ntscFrame << " pal=" << palFrame
              << " dendy=" << dendyFrame << " " << (framePass ? "PASS" : "FAIL") << "\n";
    pass &= framePass;

    PPU dendyVblank;
    dendyVblank.setTiming(ConsoleTiming::Dendy);
    dendyVblank.powerOn();
    while (!(dendyVblank.scanline() == 241 && dendyVblank.cycle() == 2)) dendyVblank.clock();
    const bool noEarlyVblank = (dendyVblank.testStatus() & 0x80) == 0;
    while (!(dendyVblank.scanline() == 291 && dendyVblank.cycle() == 2)) dendyVblank.clock();
    const bool dendyVblankAt291 = (dendyVblank.testStatus() & 0x80) != 0;
    const bool dendyVblankPass = noEarlyVblank && dendyVblankAt291;
    std::cout << "dendy_vblank_291=" << (dendyVblankPass ? "PASS" : "FAIL") << "\n";
    pass &= dendyVblankPass;

    Bus bus;
    PPU ratioPpu;
    APU ratioApu;
    bus.connectPPU(&ratioPpu); bus.connectAPU(&ratioApu);
    bus.setTiming(ConsoleTiming::PAL);
    ratioPpu.powerOn(); ratioApu.powerOn();
    for (int i = 0; i < 5; ++i) bus.clock();
    const bool ratioPass = ratioPpu.scanline() == 0 && ratioPpu.cycle() == 16;
    std::cout << "pal_ppu_ratio_5cpu=" << ratioPpu.cycle() << " " << (ratioPass ? "PASS" : "FAIL") << "\n";
    pass &= ratioPass;

    APU apu;
    apu.setTiming(ConsoleTiming::PAL);
    apu.powerOn();
    apu.cpuWrite(0x400E, 0x02);
    apu.cpuWrite(0x4010, 0x02);
    const bool palTables = apu.cpuClockHz() == 1662607 && apu.testNoisePeriod() == 14 && apu.testDmcRate() == 316;
    apu.setTiming(ConsoleTiming::Dendy);
    apu.cpuWrite(0x400E, 0x02);
    apu.cpuWrite(0x4010, 0x02);
    const bool dendyTables = apu.cpuClockHz() == 1773448 && apu.testNoisePeriod() == 16 && apu.testDmcRate() == 340;
    std::cout << "apu_region_tables=" << ((palTables && dendyTables) ? "PASS" : "FAIL") << "\n";
    pass &= palTables && dendyTables;

    APU palFrameApu;
    palFrameApu.setTiming(ConsoleTiming::PAL);
    palFrameApu.powerOn();
    uint32_t palIrqClocks = 0;
    while (!palFrameApu.testFrameIrqFlag() && palIrqClocks < 40000) {
        palFrameApu.clockFrameCounterPhase();
        ++palIrqClocks;
    }
    APU dendyFrameApu;
    dendyFrameApu.setTiming(ConsoleTiming::Dendy);
    dendyFrameApu.powerOn();
    uint32_t dendyIrqClocks = 0;
    while (!dendyFrameApu.testFrameIrqFlag() && dendyIrqClocks < 40000) {
        dendyFrameApu.clockFrameCounterPhase();
        ++dendyIrqClocks;
    }
    const bool frameCounterPass = palIrqClocks == 33250u && dendyIrqClocks == 29826u;
    std::cout << "apu_frame_irq pal=" << palIrqClocks << " dendy=" << dendyIrqClocks
              << " " << (frameCounterPass ? "PASS" : "FAIL") << "\n";
    pass &= frameCounterPass;

    std::error_code ec;
    std::filesystem::remove(ntscPath, ec); std::filesystem::remove(palPath, ec);
    std::filesystem::remove(multiPath, ec); std::filesystem::remove(dendyPath, ec);

    std::cout << (pass ? "PASS" : "FAIL") << "\n";
    return pass ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runTimingConformanceProbe(); }
#endif
