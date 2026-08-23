#include "PPU.hpp"
#include "CPU.hpp"
#include "Bus.hpp"
#include <cstdio>

namespace {
void setV(PPU& p, uint16_t v)
{
    p.cpuWrite(0x2006, static_cast<uint8_t>((v >> 8) & 0x3F));
    p.cpuWrite(0x2006, static_cast<uint8_t>(v & 0xFF));
    // A real CPU cannot issue the following PPU register access before the
    // second $2006 write has propagated through these ~5 PPU dots. Direct
    // probe calls can, so settle the transfer here unless a test explicitly
    // targets the delay itself.
    for (int i = 0; i < 5; ++i) p.clock();
}

unsigned clocksToVisibleStart(PPU& p, unsigned limit)
{
    unsigned clocks = 0;
    while (!(p.scanline() == 0 && p.cycle() == 0) && clocks < limit) {
        p.clock();
        ++clocks;
    }
    return clocks;
}

struct RegionalVblankRaceResult {
    bool dot0Suppress = false;
    bool sameDotSuppress = false;
    bool plusOneCancels = false;
    bool plusTwoCommitted = false;
};

RegionalVblankRaceResult probeRegionalVblankRace(ConsoleTiming timing)
{
    const int vblankLine = consoleVblankStartScanline(timing);
    RegionalVblankRaceResult result;

    // Dot 0: the read still observes VBlank clear and prevents the dot-1
    // assertion/NMI from occurring for this frame.
    {
        Bus bus;
        CPU cpu(bus);
        PPU p;
        bus.connectCPU(&cpu);
        bus.connectPPU(&p);
        p.connectCPU(&cpu);
        p.setTiming(timing);
        p.testBypassRegisterWriteInhibit();
        p.cpuWrite(0x2000, 0x80);
        while (!(p.scanline() == vblankLine && p.cycle() == 0))
            p.clock();
        const uint8_t status = p.cpuRead(0x2002);
        p.clock();
        p.clock();
        result.dot0Suppress = (status & 0x80) == 0 &&
                              (p.testStatus() & 0x80) == 0 &&
                              !cpu.testNmiPending();
    }

    // Dot 1: the read returns VBlank set, but the pulse is suppressed before
    // the CPU can commit the NMI.
    {
        Bus bus;
        CPU cpu(bus);
        PPU p;
        bus.connectCPU(&cpu);
        bus.connectPPU(&p);
        p.connectCPU(&cpu);
        p.setTiming(timing);
        p.testBypassRegisterWriteInhibit();
        p.cpuWrite(0x2000, 0x80);
        while (!(p.scanline() == vblankLine && p.cycle() == 1))
            p.clock();
        const uint8_t status = p.cpuRead(0x2002);
        p.clock();
        result.sameDotSuppress = (status & 0x80) != 0 &&
                                 (p.testStatus() & 0x80) == 0 &&
                                 !cpu.testNmiPending();
    }

    // One dot after assertion: VBlank reads set and the still-short NMI pulse
    // is cancelable by the status read.
    {
        Bus bus;
        CPU cpu(bus);
        PPU p;
        bus.connectCPU(&cpu);
        bus.connectPPU(&p);
        p.connectCPU(&cpu);
        p.setTiming(timing);
        p.testBypassRegisterWriteInhibit();
        p.cpuWrite(0x2000, 0x80);
        while (!(p.scanline() == vblankLine && p.cycle() == 1))
            p.clock();
        p.clock();
        const bool edgeWasPending = cpu.testNmiPending() && p.testNmiCancelWindow() == 1;
        const uint8_t status = p.cpuRead(0x2002);
        result.plusOneCancels = edgeWasPending && (status & 0x80) != 0 &&
                                !cpu.testNmiPending();
    }

    // Two dots after assertion: the CPU has committed the edge. Reading
    // PPUSTATUS still clears VBlank, but must not retract the pending NMI.
    {
        Bus bus;
        CPU cpu(bus);
        PPU p;
        bus.connectCPU(&cpu);
        bus.connectPPU(&p);
        p.connectCPU(&cpu);
        p.setTiming(timing);
        p.testBypassRegisterWriteInhibit();
        p.cpuWrite(0x2000, 0x80);
        while (!(p.scanline() == vblankLine && p.cycle() == 1))
            p.clock();
        p.clock();
        p.clock();
        const bool edgeCommitted = cpu.testNmiPending() && p.testNmiCancelWindow() == 0;
        const uint8_t status = p.cpuRead(0x2002);
        result.plusTwoCommitted = edgeCommitted && (status & 0x80) != 0 &&
                                  cpu.testNmiPending();
    }

    return result;
}
}

int runPpuConformanceProbe()
{
    bool ok = true;

    PPU p;
    p.testBypassRegisterWriteInhibit();

    // Outside rendering, PPUDATA obeys PPUCTRL's linear +1/+32 increment.
    p.cpuWrite(0x2001, 0x00);
    p.cpuWrite(0x2000, 0x00);
    setV(p, 0x2000);
    (void)p.cpuRead(0x2007);
    const bool linear1 = p.testVramAddress() == 0x2001;

    p.cpuWrite(0x2000, 0x04);
    setV(p, 0x2000);
    (void)p.cpuRead(0x2007);
    const bool linear32 = p.testVramAddress() == 0x2020;

    // PPUMASK background/sprite enable does not reach the rendering pipeline
    // immediately. The hardware-tested deterministic model uses a three-PPU-dot delay.
    // Register bits are accepted at once, while rendering remains off for the
    // first two dots and becomes active on the third. Disable follows the
    // same propagation path.
    PPU maskDelay;
    maskDelay.testBypassRegisterWriteInhibit();
    maskDelay.cpuWrite(0x2001, 0x18);
    const bool delayArmed = maskDelay.testEffectiveRenderMask() == 0 && maskDelay.testRenderMaskDelay() == 3;
    maskDelay.clock();
    maskDelay.clock();
    const bool enableHeld = maskDelay.testEffectiveRenderMask() == 0 && maskDelay.testRenderMaskDelay() == 1;
    maskDelay.clock();
    const bool enableApplied = maskDelay.testEffectiveRenderMask() == 0x18 && maskDelay.testRenderMaskDelay() == 0;
    maskDelay.cpuWrite(0x2001, 0x00);
    maskDelay.clock();
    maskDelay.clock();
    const bool disableHeld = maskDelay.testEffectiveRenderMask() == 0x18 && maskDelay.testRenderMaskDelay() == 1;
    maskDelay.clock();
    const bool disableApplied = maskDelay.testEffectiveRenderMask() == 0;
    std::printf("ppumask_render_delay=armed:%s enable_hold:%s enable_apply:%s disable_hold:%s disable_apply:%s\n",
        delayArmed ? "PASS" : "FAIL", enableHeld ? "PASS" : "FAIL", enableApplied ? "PASS" : "FAIL",
        disableHeld ? "PASS" : "FAIL", disableApplied ? "PASS" : "FAIL");
    ok &= delayArmed && enableHeld && enableApplied && disableHeld && disableApplied;

    // The second PPUADDR write updates t immediately but the active v/address
    // bus receives the completed address five PPU dots later in the CPU/PPU
    // phase modeled by the default RP2C02G profile.
    PPU addrDelay;
    addrDelay.testBypassRegisterWriteInhibit();
    addrDelay.testClearFetchTrace();
    const uint16_t addrBefore = addrDelay.testVramAddress();
    addrDelay.cpuWrite(0x2006, 0x21);
    const bool firstWriteLeavesV = addrDelay.testVramAddress() == addrBefore && addrDelay.testVramAddressDelay() == 0;
    addrDelay.cpuWrite(0x2006, 0x34);
    const bool addrDelayArmed = addrDelay.testVramAddress() == addrBefore && addrDelay.testVramAddressDelay() == 5 && addrDelay.testFetchTrace().empty();
    for (int i = 0; i < 4; ++i) addrDelay.clock();
    const bool addrHeldTwoDots = addrDelay.testVramAddress() == addrBefore && addrDelay.testVramAddressDelay() == 1 && addrDelay.testFetchTrace().empty();
    addrDelay.clock();
    const bool addrAppliedThirdDot = addrDelay.testVramAddress() == 0x2134 && addrDelay.testVramAddressDelay() == 0;
    bool addrBusVisibleThirdDot = false;
    for (uint32_t entry : addrDelay.testFetchTrace()) {
        if ((entry & 0x3FFFu) == 0x2134u) { addrBusVisibleThirdDot = true; break; }
    }
    std::printf("ppuaddr_delay=first_hold:%s armed:%s two_dot_hold:%s third_apply:%s bus:%s\n",
        firstWriteLeavesV ? "PASS" : "FAIL", addrDelayArmed ? "PASS" : "FAIL",
        addrHeldTwoDots ? "PASS" : "FAIL", addrAppliedThirdDot ? "PASS" : "FAIL",
        addrBusVisibleThirdDot ? "PASS" : "FAIL");
    ok &= firstWriteLeavesV && addrDelayArmed && addrHeldTwoDots && addrAppliedThirdDot && addrBusVisibleThirdDot;

    // If that delayed reload lands on a rendering increment dot, the 2C02's
    // internal buses fight. Only the component being incremented is affected:
    // it becomes the bitwise AND of the incremented old-v input and the t-load
    // input, and that corrupted component feeds back into both v and t.
    PPU xConflict;
    xConflict.testBypassRegisterWriteInhibit();
    xConflict.testForceEffectiveRenderMask(0x08);
    xConflict.testSetScrollAddresses(0x0000, 0x0000);
    xConflict.cpuWrite(0x2006, 0x24);
    xConflict.cpuWrite(0x2006, 0x13);
    xConflict.testSetTimingPosition(0, 4); // five-dot transfer applies on dot 8
    for (int i = 0; i < 5; ++i) xConflict.clock();
    const bool xConflictV = xConflict.testVramAddress() == 0x2001;
    const bool xConflictT = xConflict.testTempVramAddress() == 0x2001;

    PPU xyConflict;
    xyConflict.testBypassRegisterWriteInhibit();
    xyConflict.testForceEffectiveRenderMask(0x08);
    xyConflict.testSetScrollAddresses(0x0000, 0x0000);
    xyConflict.cpuWrite(0x2006, 0x34);
    xyConflict.cpuWrite(0x2006, 0x13);
    xyConflict.testSetTimingPosition(0, 252); // five-dot transfer applies on dot 256
    for (int i = 0; i < 5; ++i) xyConflict.clock();
    const bool xyConflictV = xyConflict.testVramAddress() == 0x1001;
    const bool xyConflictT = xyConflict.testTempVramAddress() == 0x1001;

    std::printf("ppuaddr_increment_conflict=x_v:%s x_t:%s xy_v:%s xy_t:%s\n",
        xConflictV ? "PASS" : "FAIL", xConflictT ? "PASS" : "FAIL",
        xyConflictV ? "PASS" : "FAIL", xyConflictT ? "PASS" : "FAIL");
    ok &= xConflictV && xConflictT && xyConflictV && xyConflictT;

    // During visible/pre-render rendering, PPUDATA's rendering-carry increment
    // reaches v only after its internal propagation delay. PPUCTRL's +1/+32
    // selection is ignored for this update.
    PPU dataDelay;
    dataDelay.testBypassRegisterWriteInhibit();
    dataDelay.testForceEffectiveRenderMask(0x08);
    dataDelay.testSetTimingPosition(0, 270);
    dataDelay.testSetScrollAddresses(0x2000, 0x2000);
    (void)dataDelay.cpuRead(0x2007);
    const bool renderReadHeld = dataDelay.testVramAddress() == 0x2000 && dataDelay.testPpudataIncrementDelay() == 6;
    for (int i = 0; i < 5; ++i) dataDelay.clock();
    const bool renderReadHeldFour = dataDelay.testVramAddress() == 0x2000 && dataDelay.testPpudataIncrementDelay() == 1;
    dataDelay.clock();
    const bool renderReadIncrement = dataDelay.testVramAddress() == 0x3001;

    // Alternate CPU/PPU master-clock alignment delays the same rendering-time
    // PPUDATA increment by one additional PPU dot (6 instead of 5).
    PPU dataDelayLate;
    dataDelayLate.testBypassRegisterWriteInhibit();
    dataDelayLate.testForceEffectiveRenderMask(0x08);
    dataDelayLate.testSetCpuPpuIoLatePhase(true);
    dataDelayLate.testSetTimingPosition(0, 270);
    dataDelayLate.testSetScrollAddresses(0x2000, 0x2000);
    (void)dataDelayLate.cpuRead(0x2007);
    const bool renderLateArmed = dataDelayLate.testPpudataIncrementDelay() == 6;
    for (int i = 0; i < 5; ++i) dataDelayLate.clock();
    const bool renderLateHeldFive = dataDelayLate.testVramAddress() == 0x2000 && dataDelayLate.testPpudataIncrementDelay() == 1;
    dataDelayLate.clock();
    const bool renderLateIncrement = dataDelayLate.testVramAddress() == 0x3001;

    PPU dataWrite;
    dataWrite.testBypassRegisterWriteInhibit();
    dataWrite.testForceEffectiveRenderMask(0x08);
    dataWrite.testSetTimingPosition(0, 270);
    dataWrite.testSetScrollAddresses(0x2000, 0x2000);
    dataWrite.cpuWrite(0x2007, 0x5A);
    for (int i = 0; i < 5; ++i) dataWrite.clock();
    const bool renderWriteIncrement = dataWrite.testVramAddress() == 0x3001;

    PPU dataWrap;
    dataWrap.testBypassRegisterWriteInhibit();
    dataWrap.testForceEffectiveRenderMask(0x08);
    dataWrap.testSetTimingPosition(0, 270);
    dataWrap.testSetScrollAddresses(0x33BF, 0x33BF); // fineY=3, coarseY=29, coarseX=31
    (void)dataWrap.cpuRead(0x2007);
    for (int i = 0; i < 6; ++i) dataWrap.clock();
    const bool renderWrap = dataWrap.testVramAddress() == 0x47A0;

    // A delayed PPUDATA increment that arrives on dot 257 fights the horizontal
    // t->v reload. The X component is ANDed and fed back into t.
    PPU dataXConflict;
    dataXConflict.testBypassRegisterWriteInhibit();
    dataXConflict.testForceEffectiveRenderMask(0x08);
    dataXConflict.testSetScrollAddresses(0x001F, 0x001F);
    dataXConflict.testSetTimingPosition(0, 252);
    (void)dataXConflict.cpuRead(0x2007);
    for (int i = 0; i < 6; ++i) dataXConflict.clock();
    const bool dataXConflictV = (dataXConflict.testVramAddress() & 0x041F) == 0x0001;
    const bool dataXConflictT = (dataXConflict.testTempVramAddress() & 0x041F) == 0x0001;

    // On pre-render dots 280-304 the same mechanism applies to the vertical
    // component while the horizontal component remains from the increment.
    PPU dataYConflict;
    dataYConflict.testBypassRegisterWriteInhibit();
    dataYConflict.testForceEffectiveRenderMask(0x08);
    dataYConflict.testSetScrollAddresses(0x0000, 0x03E0);
    dataYConflict.testSetTimingPosition(-1, 275);
    (void)dataYConflict.cpuRead(0x2007);
    for (int i = 0; i < 6; ++i) dataYConflict.clock();
    const bool dataYConflictV = (dataYConflict.testVramAddress() & 0x7BE0) == 0x0000;
    const bool dataYConflictT = (dataYConflict.testTempVramAddress() & 0x7BE0) == 0x0000;

    std::printf("ppudata_linear=+1:%s +32:%s delayed:%s/%s render_read:%s align6:%s/%s/%s render_write:%s wrap:%s conflict_x:%s/%s conflict_y:%s/%s\n",
        linear1 ? "PASS" : "FAIL", linear32 ? "PASS" : "FAIL",
        renderReadHeld ? "PASS" : "FAIL", renderReadHeldFour ? "PASS" : "FAIL",
        renderReadIncrement ? "PASS" : "FAIL", renderLateArmed ? "PASS" : "FAIL",
        renderLateHeldFive ? "PASS" : "FAIL", renderLateIncrement ? "PASS" : "FAIL",
        renderWriteIncrement ? "PASS" : "FAIL",
        renderWrap ? "PASS" : "FAIL", dataXConflictV ? "PASS" : "FAIL",
        dataXConflictT ? "PASS" : "FAIL", dataYConflictV ? "PASS" : "FAIL",
        dataYConflictT ? "PASS" : "FAIL");

    ok &= linear1 && linear32 && renderReadHeld && renderReadHeldFour && renderReadIncrement &&
        renderLateArmed && renderLateHeldFive && renderLateIncrement && renderWriteIncrement &&
        renderWrap && dataXConflictV && dataXConflictT && dataYConflictV && dataYConflictT;

    // Frame-complete is raised when entering the pre-render scanline. The first
    // such pre-render after power-on is marked odd by this implementation.
    PPU timing;
    timing.testBypassRegisterWriteInhibit();
    timing.cpuWrite(0x2001, 0x08);
    unsigned guard = 0;
    while (!timing.frameComplete() && guard < 100000) {
        timing.clock();
        ++guard;
    }
    const bool entersPreRenderAtDot0 = timing.frameComplete() && timing.scanline() == -1 && timing.cycle() == 0;
    timing.clearFrameComplete();

    const unsigned oddPreRenderClocks = clocksToVisibleStart(timing, 400);
    const bool oddSkipAtEnd = oddPreRenderClocks == 340 && timing.scanline() == 0 && timing.cycle() == 0;

    guard = 0;
    while (!timing.frameComplete() && guard < 100000) {
        timing.clock();
        ++guard;
    }
    const bool nextPreRenderAtDot0 = timing.frameComplete() && timing.scanline() == -1 && timing.cycle() == 0;
    timing.clearFrameComplete();
    const unsigned evenPreRenderClocks = clocksToVisibleStart(timing, 400);
    const bool evenFullLength = evenPreRenderClocks == 341;

    std::printf("prerender_entry=%s odd_clocks=%u odd_skip=%s even_clocks=%u even_full=%s\n",
        entersPreRenderAtDot0 && nextPreRenderAtDot0 ? "PASS" : "FAIL",
        oddPreRenderClocks,
        oddSkipAtEnd ? "PASS" : "FAIL",
        evenPreRenderClocks,
        evenFullLength ? "PASS" : "FAIL");

    ok &= entersPreRenderAtDot0 && nextPreRenderAtDot0 && oddSkipAtEnd && evenFullLength;

    // End-of-scanline bus sequencing: after the two background-prefetch tiles
    // (321-336), the PPU performs two otherwise-unused nametable reads at
    // dots 337 and 339. Both must be mapper-visible and use the same current
    // nametable address; there are no additional external reads at 338/340 in
    // this core's one-read-per-two-dot bus representation.
    PPU fetchTiming;
    fetchTiming.testBypassRegisterWriteInhibit();
    fetchTiming.cpuWrite(0x2001, 0x08);
    while (!(fetchTiming.scanline() == 0 && fetchTiming.cycle() == 337))
        fetchTiming.clock();
    fetchTiming.testClearFetchTrace();
    for (unsigned i = 0; i < 4; ++i)
        fetchTiming.clock();

    unsigned dot337Count = 0, dot339Count = 0, otherTailCount = 0;
    uint16_t dot337Addr = 0xFFFF, dot339Addr = 0xFFFF;
    for (uint32_t entry : fetchTiming.testFetchTrace()) {
        const unsigned dot = static_cast<unsigned>(entry >> 16);
        const uint16_t addr = static_cast<uint16_t>(entry & 0x3FFF);
        if (dot == 337) { ++dot337Count; dot337Addr = addr; }
        else if (dot == 339) { ++dot339Count; dot339Addr = addr; }
        else if (dot >= 337 && dot <= 340) { ++otherTailCount; }
    }
    const bool tailFetches = dot337Count == 1 && dot339Count == 1 &&
                             otherTailCount == 0 &&
                             (dot337Addr & 0x3000) == 0x2000 &&
                             dot337Addr == dot339Addr;

    std::printf("tail_fetches_337_339=%s addr=$%04X/%04X counts=%u/%u other=%u\n",
        tailFetches ? "PASS" : "FAIL", dot337Addr, dot339Addr,
        dot337Count, dot339Count, otherTailCount);
    ok &= tailFetches;

    // VBlank race: reading PPUSTATUS on dot 0 immediately before the flag is
    // asserted on dot 1 suppresses both the flag and the corresponding NMI
    // for that frame. A normal entry without the dot-0 read must still set it.
    PPU vblankNormal;
    vblankNormal.testBypassRegisterWriteInhibit();
    while (!(vblankNormal.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             vblankNormal.cycle() == 0))
        vblankNormal.clock();
    vblankNormal.clock(); // dot 0 -> dot 1
    vblankNormal.clock(); // execute dot 1 VBlank set
    const bool normalVblankSet = (vblankNormal.testStatus() & 0x80) != 0;

    PPU vblankSuppressed;
    vblankSuppressed.testBypassRegisterWriteInhibit();
    while (!(vblankSuppressed.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             vblankSuppressed.cycle() == 0))
        vblankSuppressed.clock();
    const uint8_t beforeVblank = vblankSuppressed.cpuRead(0x2002);
    vblankSuppressed.clock(); // dot 0 -> dot 1
    vblankSuppressed.clock(); // dot 1 attempts to set VBlank
    const bool dot0ReadWasClear = (beforeVblank & 0x80) == 0;
    const bool dot0SuppressedVblank = (vblankSuppressed.testStatus() & 0x80) == 0;

    std::printf("vblank_dot0_race=normal:%s read_clear:%s suppressed:%s\n",
        normalVblankSet ? "PASS" : "FAIL",
        dot0ReadWasClear ? "PASS" : "FAIL",
        dot0SuppressedVblank ? "PASS" : "FAIL");
    ok &= normalVblankSet && dot0ReadWasClear && dot0SuppressedVblank;

    // Complete the $2002/VBlank race window. A read on the exact set dot must
    // return VBlank=1 even though this scheduler has not yet executed dot 1;
    // it then suppresses the frame's NMI/flag. A read one dot after the set
    // sees 1 and remains inside the cancelable NMI window. By two dots after,
    // the edge is committed and the special cancellation window is over.
    PPU vblankSameDot;
    vblankSameDot.testBypassRegisterWriteInhibit();
    vblankSameDot.cpuWrite(0x2000, 0x80); // enable NMI output for cancellation-window checks
    while (!(vblankSameDot.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             vblankSameDot.cycle() == 1))
        vblankSameDot.clock();
    const uint8_t sameDotStatus = vblankSameDot.cpuRead(0x2002);
    vblankSameDot.clock();
    const bool sameDotReadsSet = (sameDotStatus & 0x80) != 0;
    const bool sameDotSuppresses = (vblankSameDot.testStatus() & 0x80) == 0;

    PPU vblankPlusOne;
    vblankPlusOne.testBypassRegisterWriteInhibit();
    vblankPlusOne.cpuWrite(0x2000, 0x80); // enable NMI output for cancellation-window checks
    while (!(vblankPlusOne.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             vblankPlusOne.cycle() == 1))
        vblankPlusOne.clock();
    vblankPlusOne.clock(); // execute dot 1 set; now positioned at dot 2
    const bool oneDotWindowOpen = vblankPlusOne.testNmiCancelWindow() == 1;
    const uint8_t plusOneStatus = vblankPlusOne.cpuRead(0x2002);
    const bool plusOneReadsSet = (plusOneStatus & 0x80) != 0;

    PPU vblankPlusTwo;
    vblankPlusTwo.testBypassRegisterWriteInhibit();
    vblankPlusTwo.cpuWrite(0x2000, 0x80); // enable NMI output for cancellation-window checks
    while (!(vblankPlusTwo.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             vblankPlusTwo.cycle() == 1))
        vblankPlusTwo.clock();
    vblankPlusTwo.clock(); // dot 1 set -> dot 2, cancel window = 1
    vblankPlusTwo.clock(); // execute dot 2 -> dot 3, window expires
    const bool twoDotWindowClosed = vblankPlusTwo.testNmiCancelWindow() == 0;
    const uint8_t plusTwoStatus = vblankPlusTwo.cpuRead(0x2002);
    const bool plusTwoReadsSetNormally = (plusTwoStatus & 0x80) != 0;

    const bool vblankRaceWindow = sameDotReadsSet && sameDotSuppresses &&
                                  oneDotWindowOpen && plusOneReadsSet &&
                                  twoDotWindowClosed && plusTwoReadsSetNormally;
    std::printf("vblank_read_window=same_set:%s same_suppress:%s plus1_set:%s plus1_cancel:%s plus2_set:%s plus2_committed:%s\n",
        sameDotReadsSet ? "PASS" : "FAIL",
        sameDotSuppresses ? "PASS" : "FAIL",
        plusOneReadsSet ? "PASS" : "FAIL",
        oneDotWindowOpen ? "PASS" : "FAIL",
        plusTwoReadsSetNormally ? "PASS" : "FAIL",
        twoDotWindowClosed ? "PASS" : "FAIL");
    ok &= vblankRaceWindow;

    // The local VBlank/$2002 race is a PPU property and must remain identical
    // across the supported NTSC, PAL, and Dendy frame geometries. Keep this
    // as an integrated PPU+CPU assertion so a future regional timing change
    // cannot accidentally alter NMI cancellation semantics.
    const RegionalVblankRaceResult ntscRace = probeRegionalVblankRace(ConsoleTiming::NTSC);
    const RegionalVblankRaceResult palRace = probeRegionalVblankRace(ConsoleTiming::PAL);
    const RegionalVblankRaceResult dendyRace = probeRegionalVblankRace(ConsoleTiming::Dendy);
    const auto racePass = [](const RegionalVblankRaceResult& r) {
        return r.dot0Suppress && r.sameDotSuppress && r.plusOneCancels && r.plusTwoCommitted;
    };
    const bool regionalVblankRace = racePass(ntscRace) && racePass(palRace) && racePass(dendyRace);
    std::printf("vblank_race_regions=NTSC:%s PAL:%s Dendy:%s\n",
        racePass(ntscRace) ? "PASS" : "FAIL",
        racePass(palRace) ? "PASS" : "FAIL",
        racePass(dendyRace) ? "PASS" : "FAIL");
    ok &= regionalVblankRace;

    // PPUCTRL NMI-output edge behavior while VBlank is already active.
    // The PPU's /NMI output is (vblank_flag && PPUCTRL.7), so enabling bit 7
    // during VBlank must create an immediate edge. Disabling it inside the
    // one-dot cancellation window can withdraw an asynchronous edge; enabling
    // it again must create a fresh edge, allowing multiple NMIs in one VBlank.
    Bus nmiBus;
    CPU nmiCpu(nmiBus);
    PPU nmiCtrl;
    nmiBus.connectCPU(&nmiCpu);
    nmiBus.connectPPU(&nmiCtrl);
    nmiCtrl.connectCPU(&nmiCpu);
    nmiCtrl.testBypassRegisterWriteInhibit();
    while (!(nmiCtrl.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             nmiCtrl.cycle() == 1))
        nmiCtrl.clock();
    nmiCtrl.clock(); // VBlank flag sets with NMI output disabled.
    const bool noEdgeWhileDisabled = !nmiCpu.testNmiPending();

    nmiCtrl.cpuWrite(0x2000, 0x80);
    const bool enableDuringVblankEdges = nmiCpu.testNmiPending() && nmiCtrl.testNmiCancelWindow() == 1;
    nmiCtrl.cpuWrite(0x2000, 0x00);
    const bool immediateDisableCancels = !nmiCpu.testNmiPending();
    nmiCtrl.cpuWrite(0x2000, 0x80);
    const bool reenableCreatesFreshEdge = nmiCpu.testNmiPending() && nmiCtrl.testNmiCancelWindow() == 1;

    // A disable one PPU dot after the edge is still inside the short-pulse
    // cancellation window. Two dots after, the edge is committed and lowering
    // PPUCTRL.7 must not erase the CPU's already-latched NMI.
    Bus nmiPlusOneBus;
    CPU nmiPlusOneCpu(nmiPlusOneBus);
    PPU nmiPlusOne;
    nmiPlusOneBus.connectCPU(&nmiPlusOneCpu);
    nmiPlusOneBus.connectPPU(&nmiPlusOne);
    nmiPlusOne.connectCPU(&nmiPlusOneCpu);
    nmiPlusOne.testBypassRegisterWriteInhibit();
    nmiPlusOne.cpuWrite(0x2000, 0x80); // VBlank itself will create the edge.
    while (!(nmiPlusOne.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             nmiPlusOne.cycle() == 1))
        nmiPlusOne.clock();
    nmiPlusOne.clock(); // execute VBlank-set dot; now at +1 with window open
    nmiPlusOne.cpuWrite(0x2000, 0x00);
    const bool disablePlusOneCancels = !nmiPlusOneCpu.testNmiPending();

    Bus nmiPlusTwoBus;
    CPU nmiPlusTwoCpu(nmiPlusTwoBus);
    PPU nmiPlusTwo;
    nmiPlusTwoBus.connectCPU(&nmiPlusTwoCpu);
    nmiPlusTwoBus.connectPPU(&nmiPlusTwo);
    nmiPlusTwo.connectCPU(&nmiPlusTwoCpu);
    nmiPlusTwo.testBypassRegisterWriteInhibit();
    nmiPlusTwo.cpuWrite(0x2000, 0x80); // VBlank itself will create the edge.
    while (!(nmiPlusTwo.scanline() == consoleVblankStartScanline(ConsoleTiming::NTSC) &&
             nmiPlusTwo.cycle() == 1))
        nmiPlusTwo.clock();
    nmiPlusTwo.clock(); // +1, cancelable
    nmiPlusTwo.clock(); // +2, window expired
    nmiPlusTwo.cpuWrite(0x2000, 0x00);
    const bool disablePlusTwoCommitted = nmiPlusTwoCpu.testNmiPending();

    const bool nmiCtrlEdges = noEdgeWhileDisabled && enableDuringVblankEdges &&
                              immediateDisableCancels && reenableCreatesFreshEdge &&
                              disablePlusOneCancels && disablePlusTwoCommitted;
    std::printf("ppuctrl_nmi_edges=disabled:%s enable:%s cancel:%s reenable:%s plus1_cancel:%s plus2_committed:%s\n",
        noEdgeWhileDisabled ? "PASS" : "FAIL",
        enableDuringVblankEdges ? "PASS" : "FAIL",
        immediateDisableCancels ? "PASS" : "FAIL",
        reenableCreatesFreshEdge ? "PASS" : "FAIL",
        disablePlusOneCancels ? "PASS" : "FAIL",
        disablePlusTwoCommitted ? "PASS" : "FAIL");
    ok &= nmiCtrlEdges;

    // Pre-render status clear boundary. VBlank, sprite-0 hit, and sprite
    // overflow remain latched through pre-render dot 0 and are all cleared
    // together when dot 1 executes. A PPUSTATUS read immediately before that
    // hardware clear may clear VBlank itself, but must leave the two sprite
    // flags alone until the dot-1 clear event.
    PPU preRenderStatus;
    preRenderStatus.testBypassRegisterWriteInhibit();
    while (!(preRenderStatus.scanline() == -1 && preRenderStatus.cycle() == 0))
        preRenderStatus.clock();
    preRenderStatus.testSetStatusFlags(0xE0);
    const bool preDot0AllSet = (preRenderStatus.testStatus() & 0xE0) == 0xE0;
    preRenderStatus.clock(); // execute dot 0 -> positioned at dot 1
    const bool beforeDot1AllSet = (preRenderStatus.testStatus() & 0xE0) == 0xE0;
    preRenderStatus.clock(); // execute dot 1 clear -> positioned at dot 2
    const bool afterDot1AllClear = (preRenderStatus.testStatus() & 0xE0) == 0x00;

    PPU preRenderRead;
    preRenderRead.testBypassRegisterWriteInhibit();
    while (!(preRenderRead.scanline() == -1 && preRenderRead.cycle() == 0))
        preRenderRead.clock();
    preRenderRead.testSetStatusFlags(0xE0);
    preRenderRead.clock(); // dot 0 -> dot 1, flags still set
    const uint8_t preClearRead = preRenderRead.cpuRead(0x2002);
    // A CPU read beginning on pre-render dot 1 samples the cleared status by
    // M2 fall. The internal dot-1 clear itself has not executed yet in this
    // direct probe, so only the CPU read side effect (VBlank clear) is visible
    // in m_status until the next PPU clock.
    const bool readSeesBoundaryClear = (preClearRead & 0xE0) == 0x00;
    const bool readOnlyClearsVblank = (preRenderRead.testStatus() & 0xE0) == 0x60;
    preRenderRead.clock(); // hardware dot-1 clear removes sprite flags too
    const bool hardwareClearFinishes = (preRenderRead.testStatus() & 0xE0) == 0x00;

    const bool preRenderClear = preDot0AllSet && beforeDot1AllSet && afterDot1AllClear &&
                                readSeesBoundaryClear && readOnlyClearsVblank && hardwareClearFinishes;
    std::printf("prerender_status_clear=dot0:%s before_dot1:%s after_dot1:%s read_boundary:%s read_vbl_only:%s finish:%s\n",
        preDot0AllSet ? "PASS" : "FAIL",
        beforeDot1AllSet ? "PASS" : "FAIL",
        afterDot1AllClear ? "PASS" : "FAIL",
        readSeesBoundaryClear ? "PASS" : "FAIL",
        readOnlyClearsVblank ? "PASS" : "FAIL",
        hardwareClearFinishes ? "PASS" : "FAIL");
    ok &= preRenderClear;

    // PPU reset warm-up: $2000/$2001/$2005/$2006 remain inhibited until
    // the internal reset signal releases. Ignored $2005/$2006 writes must not
    // advance the shared write toggle. Other ports remain usable immediately.
    PPU warmup;
    const bool warmupStartsInhibited = warmup.testRegisterWriteInhibited();
    warmup.cpuWrite(0x2005, 0x17);
    const bool ignoredScrollKeepsToggle = !warmup.testWriteToggle();
    warmup.cpuWrite(0x2006, 0x21);
    warmup.cpuWrite(0x2006, 0x34);
    const bool ignoredAddrKeepsState = !warmup.testWriteToggle() && warmup.testVramAddress() == 0x0000;
    warmup.cpuWrite(0x2003, 0x20);
    warmup.cpuWrite(0x2004, 0x5A);
    warmup.cpuWrite(0x2003, 0x20);
    const bool liveOamDuringWarmup = warmup.cpuRead(0x2004) == 0x5A;
    for (unsigned i = 0; i < 88973; ++i) warmup.clock();
    const bool stillInhibitedBeforeBoundary = warmup.testRegisterWriteInhibited();
    warmup.cpuWrite(0x2005, 0x08);
    const bool boundaryMinusOneIgnored = !warmup.testWriteToggle();
    warmup.clock();
    const bool releasedAtBoundary = !warmup.testRegisterWriteInhibited();
    warmup.cpuWrite(0x2005, 0x08);
    const bool acceptedAfterBoundary = warmup.testWriteToggle();

    PPU palWarmup;
    palWarmup.setTiming(ConsoleTiming::PAL);
    for (unsigned i = 0; i < 106022; ++i) palWarmup.clock();
    const bool palBeforeBoundary = palWarmup.testRegisterWriteInhibited();
    palWarmup.clock();
    const bool palAtBoundary = !palWarmup.testRegisterWriteInhibited();

    std::printf("ppu_warmup=NTSC:%s toggle:%s oam_live:%s boundary:%s PAL:%s\n",
        warmupStartsInhibited && ignoredAddrKeepsState ? "PASS" : "FAIL",
        ignoredScrollKeepsToggle && boundaryMinusOneIgnored && acceptedAfterBoundary ? "PASS" : "FAIL",
        liveOamDuringWarmup ? "PASS" : "FAIL",
        stillInhibitedBeforeBoundary && releasedAtBoundary ? "PASS" : "FAIL",
        palBeforeBoundary && palAtBoundary ? "PASS" : "FAIL");
    ok &= warmupStartsInhibited && ignoredScrollKeepsToggle && ignoredAddrKeepsState &&
          liveOamDuringWarmup && stillInhibitedBeforeBoundary && boundaryMinusOneIgnored &&
          releasedAtBoundary && acceptedAfterBoundary && palBeforeBoundary && palAtBoundary;

    // PAL and Dendy swap PPUMASK's red/green emphasis controls relative to
    // NTSC. With the same base palette entry, NTSC bit 5 must match PAL/Dendy
    // bit 6, and NTSC bit 6 must match PAL/Dendy bit 5.
    PPU ntscColor; ntscColor.testBypassRegisterWriteInhibit();
    ntscColor.cpuWrite(0x2001, 0x20);
    const uint32_t ntscRed = ntscColor.testColor(0x21);
    ntscColor.cpuWrite(0x2001, 0x40);
    const uint32_t ntscGreen = ntscColor.testColor(0x21);

    PPU palColor; palColor.setTiming(ConsoleTiming::PAL); palColor.testBypassRegisterWriteInhibit();
    palColor.cpuWrite(0x2001, 0x20);
    const uint32_t palBit5 = palColor.testColor(0x21);
    palColor.cpuWrite(0x2001, 0x40);
    const uint32_t palBit6 = palColor.testColor(0x21);

    PPU dendyColor; dendyColor.setTiming(ConsoleTiming::Dendy); dendyColor.testBypassRegisterWriteInhibit();
    dendyColor.cpuWrite(0x2001, 0x20);
    const uint32_t dendyBit5 = dendyColor.testColor(0x21);
    dendyColor.cpuWrite(0x2001, 0x40);
    const uint32_t dendyBit6 = dendyColor.testColor(0x21);

    const bool emphasisSwap = ntscRed == palBit6 && ntscGreen == palBit5 &&
                              ntscRed == dendyBit6 && ntscGreen == dendyBit5;
    std::printf("region_emphasis_swap=%s\n", emphasisSwap ? "PASS" : "FAIL");
    ok &= emphasisSwap;

    // Sprite-0 hit timing/eligibility. X=0 (cycle 1) can hit when PPUMASK's
    // left-edge enables expose both pixels; X=255 (cycle 256) never hits.
    // Left-edge masking and transparent pixels reach this rule as zero-valued
    // pixels, while sprite priority is intentionally absent from the condition.
    PPU spriteHitRules;
    const bool spriteHitCycle1Allowed = spriteHitRules.testSpriteZeroHitRule(1, true, true, true);
    const bool spriteHitCycle2Allowed = spriteHitRules.testSpriteZeroHitRule(2, true, true, true);
    const bool spriteHitX255Suppressed = !spriteHitRules.testSpriteZeroHitRule(256, true, true, true);
    const bool spriteHitTransparentFg = !spriteHitRules.testSpriteZeroHitRule(100, true, false, true);
    const bool spriteHitTransparentBg = !spriteHitRules.testSpriteZeroHitRule(100, true, true, false);
    const bool spriteHitNonZeroIdentityRequired = !spriteHitRules.testSpriteZeroHitRule(100, false, true, true);
    const bool spriteHitRulesOk = spriteHitCycle1Allowed && spriteHitCycle2Allowed &&
                                  spriteHitX255Suppressed && spriteHitTransparentFg &&
                                  spriteHitTransparentBg && spriteHitNonZeroIdentityRequired;
    std::printf("sprite0_hit_rules=cycle1_allowed:%s cycle2:%s x255:%s transparent:%s identity:%s\n",
        spriteHitCycle1Allowed ? "PASS" : "FAIL",
        spriteHitCycle2Allowed ? "PASS" : "FAIL",
        spriteHitX255Suppressed ? "PASS" : "FAIL",
        spriteHitTransparentFg && spriteHitTransparentBg ? "PASS" : "FAIL",
        spriteHitNonZeroIdentityRequired ? "PASS" : "FAIL");
    ok &= spriteHitRulesOk;

    // Exercise the real renderer, including PPUMASK left-edge clipping.
    // The hit flag must become visible only after the collision dot executes.
    auto renderHitAt = [](int cycle, uint8_t mask, bool bgOpaque = true, bool spriteOpaque = true) {
        PPU p;
        p.testPrimeSpriteZeroHitPixel(cycle, mask, bgOpaque, spriteOpaque);
        const bool clearBefore = (p.testStatus() & 0x40) == 0;
        p.clock();
        return clearBefore && (p.testStatus() & 0x40) != 0;
    };
    auto renderNoHitAt = [](int cycle, uint8_t mask, bool bgOpaque = true, bool spriteOpaque = true) {
        PPU p;
        p.testPrimeSpriteZeroHitPixel(cycle, mask, bgOpaque, spriteOpaque);
        p.clock();
        return (p.testStatus() & 0x40) == 0;
    };

    constexpr uint8_t kRenderBoth = 0x18;
    constexpr uint8_t kShowBgLeft = 0x02;
    constexpr uint8_t kShowSpriteLeft = 0x04;
    const bool leftCycle2BothClipped = renderNoHitAt(2, kRenderBoth);
    const bool leftCycle8BothClipped = renderNoHitAt(8, kRenderBoth);
    const bool leftCycle2BgOnly = renderNoHitAt(2, kRenderBoth | kShowBgLeft);
    const bool leftCycle2SpriteOnly = renderNoHitAt(2, kRenderBoth | kShowSpriteLeft);
    const bool leftCycle2BothShown = renderHitAt(2, kRenderBoth | kShowBgLeft | kShowSpriteLeft);
    const bool cycle8BothShown = renderHitAt(8, kRenderBoth | kShowBgLeft | kShowSpriteLeft);
    const bool cycle9IgnoresLeftMask = renderHitAt(9, kRenderBoth);
    const bool rendererTransparentBg = renderNoHitAt(20, kRenderBoth, false, true);
    const bool rendererTransparentSprite = renderNoHitAt(20, kRenderBoth, true, false);
    const bool spriteHitRendererOk = leftCycle2BothClipped && leftCycle8BothClipped &&
        leftCycle2BgOnly && leftCycle2SpriteOnly && leftCycle2BothShown &&
        cycle8BothShown && cycle9IgnoresLeftMask && rendererTransparentBg && rendererTransparentSprite;
    std::printf("sprite0_hit_renderer=left2:%s left8:%s one_layer:%s both:%s cycle9:%s transparent:%s\n",
        leftCycle2BothClipped ? "PASS" : "FAIL",
        leftCycle8BothClipped ? "PASS" : "FAIL",
        leftCycle2BgOnly && leftCycle2SpriteOnly ? "PASS" : "FAIL",
        leftCycle2BothShown && cycle8BothShown ? "PASS" : "FAIL",
        cycle9IgnoresLeftMask ? "PASS" : "FAIL",
        rendererTransparentBg && rendererTransparentSprite ? "PASS" : "FAIL");
    ok &= spriteHitRendererOk;

    // AccuracyCoin OAM Y=$EE (238) appears on the final visible scanline 239.
    // The collision circuit must therefore still operate on scanline 239,
    // while scanline 240 is outside the visible picture and cannot hit.
    auto renderHitAtScanline = [](int scanline) {
        PPU p;
        p.testPrimeSpriteZeroHitPixel(20, kRenderBoth | kShowBgLeft | kShowSpriteLeft, true, true, scanline);
        p.clock();
        return (p.testStatus() & 0x40) != 0;
    };
    const bool spriteHitLastVisible = renderHitAtScanline(239);
    const bool spriteHitPostVisibleSuppressed = !renderHitAtScanline(240);
    std::printf("sprite0_hit_vertical_guard=line239:%s line240:%s\n",
        spriteHitLastVisible ? "PASS" : "FAIL", spriteHitPostVisibleSuppressed ? "PASS" : "FAIL");
    ok &= spriteHitLastVisible && spriteHitPostVisibleSuppressed;

    // AccuracyCoin documents the physical 2C02 pattern serial inputs: low
    // bitplane shifts in 0, high bitplane shifts in 1. The external ROM test
    // reveals the high-plane 1 by forcing blank across the normal reload.
    PPU bgSerial;
    bgSerial.testForceEffectiveRenderMask(0x08);
    bgSerial.testSetBackgroundPatternShifters(0, 0);
    bgSerial.testClockBackgroundShifters();
    const bool bgSerialLo = (bgSerial.testBackgroundPatternLo() & 1) == 0;
    const bool bgSerialHi = (bgSerial.testBackgroundPatternHi() & 1) != 0;
    std::printf("bg_serial_in=lo0:%s hi1:%s\n", bgSerialLo ? "PASS" : "FAIL", bgSerialHi ? "PASS" : "FAIL");
    ok &= bgSerialLo && bgSerialHi;


    // Sprite overflow follows the 2C02's diagonal OAM scan bug after the first
    // eight in-range sprites fill secondary OAM. This produces both false
    // positives (a non-Y byte is interpreted as Y) and false negatives (a real
    // ninth sprite's Y byte is skipped). Exercise the actual timed evaluator.
    auto overflowCase = [](int mode) {
        PPU p;
        p.testBypassRegisterWriteInhibit();

        // Fill OAM with values that are safely out of range for scanline 20.
        p.cpuWrite(0x2003, 0x00);
        for (unsigned i = 0; i < 256; ++i)
            p.cpuWrite(0x2004, 0xF0);

        auto writeSprite = [&p](unsigned n, uint8_t y, uint8_t tile, uint8_t attr, uint8_t x) {
            p.cpuWrite(0x2003, static_cast<uint8_t>(n * 4));
            p.cpuWrite(0x2004, y);
            p.cpuWrite(0x2004, tile);
            p.cpuWrite(0x2004, attr);
            p.cpuWrite(0x2004, x);
        };

        // Eight ordinary in-range sprites fill secondary OAM.
        for (unsigned n = 0; n < 8; ++n)
            writeSprite(n, 20, 0xF0, 0xF0, 0xF0);

        if (mode == 0) {
            // Reliable true overflow: the immediately following sprite is in range.
            writeSprite(8, 20, 0xF0, 0xF0, 0xF0);
        } else if (mode == 1) {
            // False positive: sprite 8 Y is out of range, then the diagonal scan
            // treats sprite 9's tile byte as a Y coordinate and finds it in range.
            writeSprite(8, 0xF0, 0xF0, 0xF0, 0xF0);
            writeSprite(9, 0xF0, 20, 0xF0, 0xF0);
        } else {
            // False negative: there really is a ninth in-range sprite at #9, but
            // after sprite 8 misses the diagonal scan checks #9's tile, not its Y.
            writeSprite(8, 0xF0, 0xF0, 0xF0, 0xF0);
            writeSprite(9, 20, 0xF0, 0xF0, 0xF0);
        }

        // Normal gameplay resets OAMADDR before rendering. Keep this overflow
        // test isolated from the separate 2C02 rendering-start row-copy bug.
        p.cpuWrite(0x2003, 0x00);
        p.cpuWrite(0x2001, 0x18);
        while (!(p.scanline() == 20 && p.cycle() == 65))
            p.clock();
        while (p.scanline() == 20 && p.cycle() <= 256)
            p.clock();
        return (p.testStatus() & 0x20) != 0;
    };

    const bool overflowTrueNinth = overflowCase(0);
    const bool overflowFalsePositive = overflowCase(1);
    const bool overflowFalseNegative = !overflowCase(2);
    const bool spriteOverflowBugOk = overflowTrueNinth && overflowFalsePositive && overflowFalseNegative;
    std::printf("sprite_overflow_bug=true9:%s false_pos:%s false_neg:%s\n",
        overflowTrueNinth ? "PASS" : "FAIL",
        overflowFalsePositive ? "PASS" : "FAIL",
        overflowFalseNegative ? "PASS" : "FAIL");
    ok &= spriteOverflowBugOk;

    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runPpuConformanceProbe(); }
#endif
