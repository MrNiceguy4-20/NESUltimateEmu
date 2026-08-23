#include "APU.hpp"
#include <cstdio>

namespace {
void write4017OnCpuClock(APU& apu, uint8_t value)
{
    // Mirror Bus::clock ordering for the portion relevant to an APU register
    // write: APU timer phase, CPU bus write, then frame-counter phase.
    apu.clock();
    apu.cpuWrite(0x4017, value);
    apu.clockFrameCounterPhase();
}

unsigned clocksUntilMode5(APU& apu, unsigned limit = 8)
{
    unsigned clocks = 0;
    while (!apu.testFrameMode5() && clocks < limit) {
        apu.clock();
        apu.clockFrameCounterPhase();
        ++clocks;
    }
    return clocks;
}
}

int runApuConformanceProbe()
{
    bool ok = true;

    // $4017's sequencer reset/mode switch is delayed by CPU/APU alignment.
    // Entering 5-step mode clocks quarter+half frame only at that delayed
    // reset instant, not on the write cycle.
    APU a;
    a.powerOn();
    a.testSetPulse1Length(10);
    write4017OnCpuClock(a, 0x80);
    const bool aDelayed = !a.testFrameMode5() && a.testPulse1Length() == 10;
    const unsigned aDelay = clocksUntilMode5(a);
    const bool aHalfClock = a.testPulse1Length() == 9;

    APU b;
    b.powerOn();
    // Advance one complete CPU/APU clock to select the opposite divider phase.
    b.clock();
    b.clockFrameCounterPhase();
    b.testSetPulse1Length(10);
    write4017OnCpuClock(b, 0x80);
    const bool bDelayed = !b.testFrameMode5() && b.testPulse1Length() == 10;
    const unsigned bDelay = clocksUntilMode5(b);
    const bool bHalfClock = b.testPulse1Length() == 9;

    // powerOn() starts m_apuPhase=false. write4017OnCpuClock() first calls
    // APU::clock(), so test A writes with m_apuPhase=true and takes the
    // 3-clock path; test B performs one extra complete clock first and writes
    // with m_apuPhase=false, which takes the 4-clock path. This polarity is
    // also required by blargg 01.len_ctr's $4017=$80 immediate-clock case.
    const bool phaseDelay = (aDelay == 3 && bDelay == 4);
    std::printf("4017_delay=%u/%u deferred=%s halfclock=%s\n",
        aDelay, bDelay,
        (aDelayed && bDelayed) ? "PASS" : "FAIL",
        (aHalfClock && bHalfClock) ? "PASS" : "FAIL");
    ok &= phaseDelay && aDelayed && bDelayed && aHalfClock && bHalfClock;

    // Bit 6 is not delayed: it suppresses frame IRQ generation and clears an
    // already asserted frame IRQ immediately, even though the divider reset is
    // still pending.
    APU inhibit;
    inhibit.powerOn();
    inhibit.testSetFrameIrqFlag(true);
    inhibit.cpuWrite(0x4017, 0x40);
    const bool inhibitImmediate = !inhibit.testFrameIrqFlag();
    const bool resetStillPending = inhibit.testFrameResetDelay() != 0;
    std::printf("4017_irq_inhibit=%s reset_pending=%s\n",
        inhibitImmediate ? "PASS" : "FAIL",
        resetStillPending ? "PASS" : "FAIL");
    ok &= inhibitImmediate && resetStillPending;

    // Blargg's hardware-tested 2005 frame-counter suite measures the new
    // sequence from the $4017 write itself, even though the divider/mode
    // visibility is delayed by 3/4 clocks. On NTSC the first half-frame must
    // therefore land 14915 CPU clocks after the write.
    APU lenTiming;
    lenTiming.powerOn();
    lenTiming.testSetPulse1Length(10);
    write4017OnCpuClock(lenTiming, 0x00);
    unsigned halfAt = 0;
    for (unsigned i = 1; i <= 14920; ++i) {
        lenTiming.clock();
        lenTiming.clockFrameCounterPhase();
        if (lenTiming.testPulse1Length() == 9) { halfAt = i; break; }
    }
    const bool halfWriteRelative = halfAt == 14915;
    std::printf("frame_half_write_relative=%u %s\n", halfAt,
        halfWriteRelative ? "PASS" : "FAIL");
    ok &= halfWriteRelative;

    // The divider phase must remain observable as one clock of frame-counter
    // jitter.  The opposite $4017 write phase is one CPU clock later than the
    // synchronized path; do not compensate this away when seeding the new
    // sequence epoch.
    APU irqJitter;
    irqJitter.powerOn();
    irqJitter.clock();
    irqJitter.clockFrameCounterPhase();
    write4017OnCpuClock(irqJitter, 0x00);
    unsigned jitterIrqAt = 0;
    for (unsigned i = 1; i <= 29836; ++i) {
        irqJitter.clock();
        irqJitter.clockFrameCounterPhase();
        if (irqJitter.testFrameIrqFlag()) { jitterIrqAt = i; break; }
    }
    const bool irqJitterVisible = jitterIrqAt == 29831;
    std::printf("frame_irq_opposite_phase=%u %s\n", jitterIrqAt,
        irqJitterVisible ? "PASS" : "FAIL");
    ok &= irqJitterVisible;

    // In 5-step mode the post-step-0 wrap is one CPU clock longer than a
    // simple zero restart.  With length=4, the delayed immediate half clock
    // plus the regular half clocks must make the counter reach zero at the
    // hardware-tested read boundary: the frame phase at clock 52197 runs
    // after that clock's CPU read, so blargg observes playing at 52197 and
    // silent at 52198.
    APU mode5Timing;
    mode5Timing.powerOn();
    mode5Timing.testSetPulse1Length(4);
    write4017OnCpuClock(mode5Timing, 0x80);
    unsigned mode5ThirdAt = 0;
    for (unsigned i = 1; i <= 52202; ++i) {
        mode5Timing.clock();
        mode5Timing.clockFrameCounterPhase();
        if (mode5Timing.testPulse1Length() == 0) { mode5ThirdAt = i; break; }
    }
    const bool mode5ThirdBoundary = mode5ThirdAt == 52197;
    std::printf("mode5_third_length=%u %s\n", mode5ThirdAt,
        mode5ThirdBoundary ? "PASS" : "FAIL");
    ok &= mode5ThirdBoundary;

    // The first frame-IRQ assertion is write-relative clock 29830, followed
    // by assertions at 29831 and 29832. Reading $4015 reports and clears it.
    APU irq;
    irq.powerOn();
    write4017OnCpuClock(irq, 0x00);
    unsigned irqAt = 0;
    for (unsigned i = 1; i <= 29835; ++i) {
        irq.clock();
        irq.clockFrameCounterPhase();
        if (irq.testFrameIrqFlag()) { irqAt = i; break; }
    }
    const bool irqWriteRelative = irqAt == 29830;
    const uint8_t status = irq.cpuRead(0x4015);
    const bool statusReports = (status & 0x40) != 0;

    // Test the delayed acknowledge away from the terminal reassertion window:
    // a $4015 read leaves the latch set until the next APU GET phase.
    APU ack;
    ack.powerOn();
    ack.testSetFrameIrqFlag(true);
    const uint8_t ackStatus = ack.cpuRead(0x4015);
    const bool clearIsDeferred = (ackStatus & 0x40) != 0 && ack.testFrameIrqFlag();
    bool sawClear = false;
    for (unsigned i = 0; i < 2; ++i) {
        ack.clock();
        ack.clockFrameCounterPhase();
        if (!ack.testFrameIrqFlag()) { sawClear = true; break; }
    }
    const bool statusClearsOnGet = sawClear;
    std::printf("frame_irq_write_relative=%u %s status_report=%s deferred=%s clear_get=%s\n", irqAt,
        irqWriteRelative ? "PASS" : "FAIL",
        statusReports ? "PASS" : "FAIL",
        clearIsDeferred ? "PASS" : "FAIL",
        statusClearsOnGet ? "PASS" : "FAIL");
    ok &= irqWriteRelative && statusReports && clearIsDeferred && statusClearsOnGet;


    // AccuracyCoin Page 14 / Length Counter test 9: two 5-step immediate
    // clocks while halt is set must leave the counter unchanged; after halt
    // is cleared, two more $4017=$80 writes must reduce length 2 to zero
    // *before* the following $4015 read samples status. The 4-clock divider
    // phase makes the final delayed reset land on that read's CPU period.
    // The reset/half-frame signal is visible pre-M2 on that period.
    APU lengthHaltBoundary;
    lengthHaltBoundary.powerOn();
    auto fullCycle = [&](APU& apu) {
        apu.clock();
        apu.clockFrameCounterPreCpuPhase();
        apu.clockFrameCounterPhase();
    };
    auto ldaImm2 = [&](APU& apu) { fullCycle(apu); fullCycle(apu); };
    auto staAbs = [&](APU& apu, uint16_t addr, uint8_t value) {
        for (int i = 0; i < 3; ++i) fullCycle(apu);
        apu.clock();
        apu.clockFrameCounterPreCpuPhase();
        apu.cpuWrite(addr, value);
        apu.clockFrameCounterPhase();
    };
    auto lda4015 = [&](APU& apu) {
        for (int i = 0; i < 3; ++i) fullCycle(apu);
        apu.clock();
        apu.clockFrameCounterPreCpuPhase();
        const uint8_t value = apu.cpuRead(0x4015);
        apu.clockFrameCounterPhase();
        return value;
    };

    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4015, 0x01);
    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4017, 0x00);
    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4003, 0x18); // length=2
    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4000, 0x30); // halt
    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4017, 0x80);
    staAbs(lengthHaltBoundary, 0x4017, 0x80);
    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4000, 0x10); // unhalt
    ldaImm2(lengthHaltBoundary); staAbs(lengthHaltBoundary, 0x4017, 0x80);
    staAbs(lengthHaltBoundary, 0x4017, 0x80);
    const uint8_t finalLengthStatus = lda4015(lengthHaltBoundary);
    const bool lengthHaltBoundaryOk = (finalLengthStatus & 0x01) == 0 &&
                                      lengthHaltBoundary.testPulse1Length() == 0;
    std::printf("length_halt_preserve_boundary=%s status=%02X len=%u\n",
        lengthHaltBoundaryOk ? "PASS" : "FAIL", finalLengthStatus,
        lengthHaltBoundary.testPulse1Length());
    ok &= lengthHaltBoundaryOk;


    // DMC revision regression. All RP2A03G-era revisions share the two
    // one-cycle implicit-abort timing positions, but late RP2A03G/RP2A03H
    // additionally perform a full duplicate reload when the one-byte load
    // completes on the same APU cycle that the output unit requests a reload.
    APU earlyDmc;
    earlyDmc.powerOn();
    earlyDmc.setDmcCpuRevision(APU::DmcCpuRevision::PreMid1990);
    earlyDmc.testPrimeOneByteDmcLoad(0xC123, 0);
    earlyDmc.completeDmcDma(0x5A);
    const bool earlyNoUnexpectedReload = !earlyDmc.testDmcForcedReloadPending() &&
                                         earlyDmc.testDmcCurrentAddr() == 0xC124;

    APU lateDmc;
    lateDmc.powerOn();
    lateDmc.setDmcCpuRevision(APU::DmcCpuRevision::Mid1990OrLater);
    lateDmc.testPrimeOneByteDmcLoad(0xC123, 0);
    lateDmc.completeDmcDma(0x5A);
    const bool lateUnexpectedReload = lateDmc.testDmcForcedReloadPending() &&
                                      lateDmc.testDmcCurrentAddr() == 0xC123;

    auto sharedAbortCase = [](APU::DmcCpuRevision revision) {
        APU apu;
        apu.powerOn();
        apu.setDmcCpuRevision(revision);
        apu.testPrimeOneByteDmcLoad(0xC200, 2);
        apu.completeDmcDma(0xA5);
        apu.clock(); // timer 2 -> 1
        apu.clock(); // timer 1 -> 0
        apu.clock(); // output reload consumes buffer and arms one-cycle abort
        return apu.testDmcAbortPending() && !apu.testDmcForcedReloadPending();
    };
    const bool earlyAbortShared = sharedAbortCase(APU::DmcCpuRevision::PreMid1990);
    const bool lateAbortShared = sharedAbortCase(APU::DmcCpuRevision::Mid1990OrLater);
    std::printf("dmc_revision early_samecycle=%s late_samecycle=%s shared_abort=%s/%s\n",
        earlyNoUnexpectedReload ? "PASS" : "FAIL",
        lateUnexpectedReload ? "PASS" : "FAIL",
        earlyAbortShared ? "PASS" : "FAIL",
        lateAbortShared ? "PASS" : "FAIL");
    ok &= earlyNoUnexpectedReload && lateUnexpectedReload && earlyAbortShared && lateAbortShared;

    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runApuConformanceProbe(); }
#endif
