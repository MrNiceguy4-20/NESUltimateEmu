#include "../../src/core/Bus.hpp"
#include "../../src/core/PPU.hpp"
#include "../../src/core/APU.hpp"
#include <cstdint>
#include <iostream>

static uint64_t runOam(bool injectDmc, uint64_t injectAt)
{
    Bus bus;
    PPU ppu;
    APU apu;
    bus.connectPPU(&ppu);
    bus.connectAPU(&apu);
    apu.connectBus(&bus);
    bus.powerOn();

    bus.testStartOamDma(0x02);
    const uint64_t start = bus.cpuCycleCounter();
    bool requested = false;
    while (bus.dmaActive() || bus.dmcDmaActive()) {
        const uint64_t elapsed = bus.cpuCycleCounter() - start;
        if (injectDmc && !requested && elapsed == injectAt) {
            if (!bus.requestDmcDma(0xC000))
                return UINT64_MAX;
            requested = true;
        }
        bus.clock();
        if (bus.cpuCycleCounter() - start > 600)
            return UINT64_MAX;
    }
    return bus.cpuCycleCounter() - start;
}

int runDmaArbitrationProbe()
{

    const uint64_t plain = runOam(false, 0);
    const uint64_t overlapped = runOam(true, 9);
    bool ok = plain == 513 && overlapped == 515;
    std::cout << "oam=" << plain << " dmc_oam=" << overlapped
              << " delta=" << (overlapped >= plain ? overlapped - plain : 0)
              << (ok ? " PASS\n" : " FAIL\n");

    Bus activationBus;
    PPU activationPpu;
    APU activationApu;
    activationBus.connectPPU(&activationPpu);
    activationBus.connectAPU(&activationApu);
    activationApu.connectBus(&activationBus);
    activationBus.powerOn();
    activationApu.testSetFrameIrqFlag(true);
    activationBus.testStartOamDma(0x40);
    unsigned guard = 0;
    while (activationBus.dmaActive() && guard++ < 520)
        activationBus.clock();
    const bool oamDoesNotDecodeApu = activationApu.testFrameIrqFlag();
    std::cout << "oam_apu_decode_gated=" << (oamDoesNotDecodeApu ? "PASS" : "FAIL") << "\n";
    ok &= oamDoesNotDecodeApu;

    Bus positiveBus;
    PPU positivePpu;
    APU positiveApu;
    positiveBus.connectPPU(&positivePpu);
    positiveBus.connectAPU(&positiveApu);
    positiveApu.connectBus(&positiveBus);
    positiveBus.powerOn();
    positiveBus.write(0x0000, 0x40);
    positiveApu.testSetFrameIrqFlag(true);
    const uint8_t apu4015 = positiveBus.testReadOamDmaSource(0x4015, 0x4001);

    positiveBus.setController1(0x01);
    positiveBus.testLatchControllers();
    const uint8_t openControllerHigh = positiveBus.testReadOamDmaSource(0x4016, 0x4001);

    positiveApu.testSetFrameIrqFlag(false);
    const uint8_t apu4035 = positiveBus.testReadOamDmaSource(0x4035, 0x4001);
    positiveBus.setController1(0x01);
    positiveBus.testLatchControllers();
    const uint8_t openControllerLow = positiveBus.testReadOamDmaSource(0x4036, 0x4001);

    positiveBus.setController1(0x00);
    positiveBus.testLatchControllers();
    positiveBus.write(0x0216, 0xFF);
    const uint8_t drivenController = positiveBus.testReadOamDmaSource(0x0216, 0x4001);

    const bool positiveDecode = apu4015 == 0x40 && openControllerHigh == 0x41 &&
                                apu4035 == 0x00 && openControllerLow == 0x01 &&
                                drivenController == 0xFF;
    std::cout << "oam_apu_decode_active=" << (positiveDecode ? "PASS" : "FAIL")
              << " s15=" << unsigned(apu4015) << " joy_hi=" << unsigned(openControllerHigh)
              << " s35=" << unsigned(apu4035) << " joy_lo=" << unsigned(openControllerLow)
              << " joy_driven=" << unsigned(drivenController) << "\n";
    ok &= positiveDecode;

    Bus stateBus;
    PPU statePpu;
    APU stateApu;
    stateBus.connectPPU(&statePpu);
    stateBus.connectAPU(&stateApu);
    stateApu.connectBus(&stateBus);
    stateBus.powerOn();
    stateBus.testStartOamDma(0x40, 0x4015);
    std::vector<uint8_t> busState;
    stateBus.saveState(busState);

    Bus restoredBus;
    PPU restoredPpu;
    APU restoredApu;
    restoredBus.connectPPU(&restoredPpu);
    restoredBus.connectAPU(&restoredApu);
    restoredApu.connectBus(&restoredBus);
    restoredBus.powerOn();
    const uint8_t* stateCursor = busState.data();
    const bool stateLoad = restoredBus.loadState(stateCursor, busState.data() + busState.size()) &&
        stateCursor == busState.data() + busState.size();
    const bool oamHeldState = stateLoad && restoredBus.dmaActive() &&
        restoredBus.testOamDmaHeldCpuAddressValid() &&
        restoredBus.testOamDmaHeldCpuAddress() == 0x4015;
    std::cout << "oam_mid_dma_state=" << (oamHeldState ? "PASS" : "FAIL")
              << " held=" << std::hex << restoredBus.testOamDmaHeldCpuAddress() << std::dec << "\n";
    ok &= oamHeldState;

    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runDmaArbitrationProbe(); }
#endif
