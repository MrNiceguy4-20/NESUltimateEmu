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
    // Starting on this deterministic PUT phase produces the 513-cycle OAM
    // form: halt + 256 GET/PUT pairs. A reload-style DMC request made while
    // OAM owns the CPU overlaps its halt/dummy/alignment slots; only the DMC
    // GET plus OAM's re-alignment extend the transfer, for +2 clocks.
    const uint64_t plain = runOam(false, 0);
    const uint64_t overlapped = runOam(true, 9);
    bool ok = plain == 513 && overlapped == 515;
    std::cout << "oam=" << plain << " dmc_oam=" << overlapped
              << " delta=" << (overlapped >= plain ? overlapped - plain : 0)
              << (ok ? " PASS\n" : " FAIL\n");

    // AccuracyCoin Page 14 / APU Register Activation test 4. OAM DMA from
    // page $40 must not activate $4015 merely because the OAM source address
    // reaches $4015. The internal APU decoder is enabled by the halted 6502's
    // A15-A5; here there is no CPU read in $4000-$401F, so frame IRQ must
    // survive the entire DMA.
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

    // AccuracyCoin Page 14 / APU Register Activation test 6. Once RDY has
    // frozen the CPU address bus inside $4000-$401F, A4-A0 from the OAM bus
    // select readable APU/I/O registers and repeat them every $20 bytes.
    // The open-bus keeper is asymmetric: an internal register may discharge
    // a previously-high bit, but a high internal bit does not charge a low
    // external/open-bus bit. Reproduce the key $40 -> status -> controller ->
    // $00 transition used by AccuracyCoin's expected OAM table.
    Bus positiveBus;
    PPU positivePpu;
    APU positiveApu;
    positiveBus.connectPPU(&positivePpu);
    positiveBus.connectAPU(&positiveApu);
    positiveApu.connectBus(&positiveBus);
    positiveBus.powerOn();
    positiveBus.write(0x0000, 0x40); // seed external/open bus
    positiveApu.testSetFrameIrqFlag(true);
    const uint8_t apu4015 = positiveBus.testReadOamDmaSource(0x4015, 0x4001);

    positiveBus.setController1(0x01);
    positiveBus.testLatchControllers();
    const uint8_t openControllerHigh = positiveBus.testReadOamDmaSource(0x4016, 0x4001);

    // By the next $20 mirror the first status read has acknowledged the frame
    // IRQ. The second status read must return zero and discharge D6 from the
    // persistent open-bus latch; controller 1 then contributes only D0.
    positiveApu.testSetFrameIrqFlag(false);
    const uint8_t apu4035 = positiveBus.testReadOamDmaSource(0x4035, 0x4001);
    positiveBus.setController1(0x01);
    positiveBus.testLatchControllers();
    const uint8_t openControllerLow = positiveBus.testReadOamDmaSource(0x4036, 0x4001);

    // With a real external RAM byte driving D0-D7, controller activation may
    // still clock the port but must not replace the OAM DMA byte.
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

    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runDmaArbitrationProbe(); }
#endif
