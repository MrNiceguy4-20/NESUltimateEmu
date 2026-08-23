#pragma once
#include <cstdint>
#include <array>
#include <vector>
#include "Mapper.hpp"
#include "Timing.hpp"

class Cartridge;
class CPU;

class PPU {
public:
    PPU();

    void connectCartridge(Cartridge* cart);
    void connectCPU(CPU* cpu);

    void reset();

    void powerOn();

    void clock();
    void setTiming(ConsoleTiming timing);
    ConsoleTiming timing() const { return m_timing; }

    void setCpuPpuIoLatePhase(bool late) { m_cpuPpuIoLatePhase = late; }

    uint8_t cpuRead(uint16_t addr);
    void    cpuWrite(uint16_t addr, uint8_t data);

    void oamDmaWrite(uint8_t data);

    const uint32_t* framebuffer() const { return m_framebuffer.data(); }
    bool frameComplete() const { return m_frameComplete; }
    void clearFrameComplete() { m_frameComplete = false; }

    int scanline() const { return m_scanline; }
    int cycle() const { return m_cycle; }

#ifdef NES_HEADLESS

    uint16_t testVramAddress() const { return m_v; }
    uint8_t testReadBuffer() const { return m_dataBuffer; }
    uint8_t testStatus() const { return m_status; }
    bool testWriteToggle() const { return m_w; }
    bool testSpriteZeroPossible() const { return m_spriteZeroPossible; }
    uint8_t testSpriteX(unsigned slot) const { return slot < 8 ? m_spriteX[slot] : 0; }
    uint8_t testSpriteAttr(unsigned slot) const { return slot < 8 ? m_spriteAttr[slot] : 0; }
    void testClearFetchTrace() { m_testFetchTrace.clear(); }
    const std::vector<uint32_t>& testFetchTrace() const { return m_testFetchTrace; }
    void testBypassRegisterWriteInhibit() { m_registerWriteInhibitUntilClock = m_masterClock; }
    bool testRegisterWriteInhibited() const { return registerWriteInhibited(); }
    uint32_t testColor(uint8_t index) const { return nesColor(index); }
    uint8_t testNmiCancelWindow() const { return m_nmiDelay; }
    uint8_t testEffectiveRenderMask() const { return m_renderMask; }
    uint8_t testRenderMaskDelay() const { return m_renderMaskDelay; }
    uint8_t testVramAddressDelay() const { return m_vramAddressDelay; }
    uint8_t testPpudataIncrementDelay() const { return m_ppudataIncrementDelay; }
    bool testOamCorruptionPending() const { return m_oamCorruptionPending; }
    uint8_t testOamCorruptionSeed() const { return m_oamCorruptionSeed; }
    void testSetCpuPpuIoLatePhase(bool late) { m_cpuPpuIoLatePhase = late; }
    uint16_t testTempVramAddress() const { return m_t; }
    void testSetTimingPosition(int scanline, int cycle) { m_scanline = scanline; m_cycle = cycle; }
    void testSetScrollAddresses(uint16_t v, uint16_t t) { m_v = v; m_t = t; }
    void testForceEffectiveRenderMask(uint8_t mask) { m_mask = mask; m_renderMask = static_cast<uint8_t>(mask & 0x18); m_pendingRenderMask = m_renderMask; m_renderMaskDelay = 0; }
    void testSetStatusFlags(uint8_t flags) { m_status = static_cast<uint8_t>((m_status & 0x1F) | (flags & 0xE0)); updateNmiLine(); }
    bool testSpriteZeroHitRule(int cycle, bool spriteZeroActive, bool fgOpaque, bool bgOpaque) const {
        return spriteZeroHitEligible(cycle, spriteZeroActive, fgOpaque ? 1 : 0, bgOpaque ? 1 : 0);
    }
    void testSetBackgroundPatternShifters(uint16_t lo, uint16_t hi) { m_bgShifterPatternLo = lo; m_bgShifterPatternHi = hi; }
    void testClockBackgroundShifters() { updateBackgroundShifters(); }
    uint16_t testBackgroundPatternLo() const { return m_bgShifterPatternLo; }
    uint16_t testBackgroundPatternHi() const { return m_bgShifterPatternHi; }

    void testPrimeSpriteZeroHitPixel(int cycle, uint8_t mask, bool bgOpaque = true, bool spriteOpaque = true, int scanline = 0) {
        m_scanline = scanline;
        m_cycle = cycle;
        m_mask = mask;
        m_renderMask = static_cast<uint8_t>(mask & 0x18);
        m_pendingRenderMask = m_renderMask;
        m_renderMaskDelay = 0;
        m_status &= static_cast<uint8_t>(~0x40);
        m_x = 0;
        m_bgShifterPatternLo = bgOpaque ? 0xFFFF : 0x0000;
        m_bgShifterPatternHi = 0;
        m_bgShifterAttrLo = 0;
        m_bgShifterAttrHi = 0;
        m_spriteCount = 1;
        m_spriteZeroPossible = true;
        m_spriteX[0] = 0;
        m_spriteAttr[0] = 0;
        m_spriteShifterLo[0] = spriteOpaque ? 0xFF : 0x00;
        m_spriteShifterHi[0] = 0;
        for (unsigned i = 1; i < 8; ++i) {
            m_spriteX[i] = 0xFF;
            m_spriteShifterLo[i] = 0;
            m_spriteShifterHi[i] = 0;
            m_spriteAttr[i] = 0;
        }
    }
#endif

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

private:
    Cartridge* m_cart = nullptr;
    CPU* m_cpu = nullptr;

    int  m_scanline = 0;
    int  m_cycle = 0;
    bool m_frameComplete = false;
    bool m_oddFrame = false;
    ConsoleTiming m_timing = ConsoleTiming::NTSC;

    uint8_t m_ctrl = 0;
    uint8_t m_mask = 0;

    uint8_t m_renderMask = 0;
    uint8_t m_pendingRenderMask = 0;
    uint8_t m_renderMaskDelay = 0;
    uint8_t m_status = 0;
    uint8_t m_oamAddr = 0;

    uint16_t m_v = 0;
    uint16_t m_t = 0;
    uint8_t  m_x = 0;
    bool     m_w = false;

    uint8_t m_dataBuffer = 0;
    enum class RenderFetchTarget : uint8_t { None = 0, BgNt, BgAttr, BgLow, BgHigh, SpriteGarbage, SpriteLow, SpriteHigh, DummyNt337, DummyNt339 };
    bool m_renderFetchPending = false;
    uint16_t m_renderFetchAddress = 0;
    PpuFetchKind m_renderFetchKind = PpuFetchKind::Background;
    RenderFetchTarget m_renderFetchTarget = RenderFetchTarget::None;
    uint8_t m_renderFetchSpriteSlot = 0;
    bool m_renderFetchStoreData = true;

    uint8_t m_ppuAddressLatchLow = 0;
    bool m_renderAleThisClock = false;
    uint16_t m_renderAleAddress = 0;
    uint8_t m_renderAlePreviousLow = 0;
    bool m_renderFetchLowOverride = false;
    bool m_renderFetchUseLiveHigh = false;

    bool m_ppudataReadPending = false;
    uint8_t m_ppudataReadDelay = 0;
    uint16_t m_ppudataReadAddress = 0;
    bool m_ppuBusReadThisClock = false;
    uint8_t m_ppuBusReadData = 0;
    uint16_t m_ppuBusReadAddress = 0;

    bool m_ppuBusReadHeldThisClock = false;
    uint8_t m_ppuBusReadHeldData = 0;
    uint16_t m_ppuBusReadHeldAddress = 0;
    const char* m_ppuBusReadHeldSource = "none";

    uint8_t m_ppuBusTraceDots = 0;
    const char* m_ppuBusReadSource = "none";
    uint8_t m_oam[256] = {};
    uint8_t m_oamSecondary[32] = {};
    uint8_t m_spriteCount = 0;
    bool    m_spriteZeroPossible = false;
    bool    m_spriteZeroBeingRendered = false;

    uint8_t m_spriteEvalN = 0;
    uint8_t m_spriteEvalM = 0;
    uint8_t m_spriteEvalFound = 0;
    uint8_t m_spriteEvalData = 0;
    uint8_t m_spriteEvalStartAddr = 0;
    uint8_t m_spriteEvalCopyRemaining = 0;
    uint8_t m_spriteEvalWrapBusData = 0xFF;
    uint8_t m_spriteEvalOverflowIncRemaining = 0;
    bool    m_spriteEvalOverflowHandoff = false;
    bool    m_spriteEvalFull = false;
    bool    m_spriteEvalOverflowDiagonal = false;
    bool    m_spriteEvalWrapped = false;
    bool    m_spriteZeroNext = false;

    uint8_t m_spriteShifterLo[8] = {};
    uint8_t m_spriteShifterHi[8] = {};
    uint8_t m_spriteX[8] = {};
    uint8_t m_spriteAttr[8] = {};

    uint8_t m_nametable[4096] = {};
    uint8_t m_palette[32] = {};

#ifdef NES_PROBE_SUITE

    std::vector<uint32_t> m_framebuffer = std::vector<uint32_t>(256 * 240);
#else
    std::array<uint32_t, 256 * 240> m_framebuffer{};
#endif

    uint8_t  m_bgNextTileId = 0;
    uint8_t  m_bgNextTileAttr = 0;
    uint8_t  m_bgNextTileLsb = 0;
    uint8_t  m_bgNextTileMsb = 0;
    uint16_t m_bgShifterPatternLo = 0;
    uint16_t m_bgShifterPatternHi = 0;
    uint16_t m_bgShifterAttrLo = 0;
    uint16_t m_bgShifterAttrHi = 0;

    bool m_bgPatternLoadArmed = false;

    uint64_t m_masterClock = 0;
    uint32_t m_oamDecayCycles[32] = {};
    uint8_t  m_oamLfsr = 0x5A;

    uint8_t  m_busLatch = 0;
    uint64_t m_busBitRefreshClock[8] = {};

    bool     m_nmiLine = false;
    uint8_t  m_nmiDelay = 0;
    bool     m_suppressVBlank = false;
    bool     m_renderingActiveLastClock = false;
    bool     m_oamCorruptionPending = false;
    uint8_t  m_oamCorruptionSeed = 0;

    uint64_t m_registerWriteInhibitUntilClock = 0;

    uint16_t m_pendingVramAddress = 0;
    uint8_t  m_vramAddressDelay = 0;
    uint8_t  m_ppudataIncrementDelay = 0;
    bool m_ppudataIncrementPending = false;
    bool m_cpuPpuIoLatePhase = false;
    bool m_suppressScrollXThisClock = false;
    bool m_suppressScrollYThisClock = false;

#ifdef NES_HEADLESS

    std::vector<uint32_t> m_testFetchTrace;
#endif

    uint8_t  ppuRead(uint16_t addr, PpuFetchKind kind = PpuFetchKind::Cpu, bool driveAddress = true);
    void     ppuWrite(uint16_t addr, uint8_t data);
    uint16_t mirrorNametable(uint16_t addr) const;

    void setVBlank();
    void clearVBlank(bool allowPendingEdgeCancel = true);
    void updateNmiLine();
    void clockNmiDelay();
    void notifyPpuAddress(uint16_t addr);
    void armPpuBusTrace(const char* event, uint16_t cpuAddr, uint8_t data = 0);
    void tracePpuBus(const char* event, const char* source, uint16_t addr, uint8_t data);
    bool renderingEnabled() const { return (m_renderMask & 0x18) != 0; }
    bool backgroundRenderingEnabled() const { return (m_renderMask & 0x08) != 0; }
    bool spriteRenderingEnabled() const { return (m_renderMask & 0x10) != 0; }
    void beginRenderFetch(uint16_t addr, PpuFetchKind kind, RenderFetchTarget target, uint8_t spriteSlot = 0, bool storeData = true);
    void completeRenderFetch();
    void clockRenderMaskDelay();
    void clockPpudataIncrementDelay();
    void clockPpudataReadBufferDelay();
    void clockVramAddressDelay();
    bool registerWriteInhibited() const { return m_masterClock < m_registerWriteInhibitUntilClock; }
    uint64_t registerWriteInhibitDuration() const;

    void loadBackgroundShifters();
    void updateBackgroundShifters();
    void incrementScrollX();
    void incrementScrollY();
    void incrementVramAddressAfterCpuAccess();
    void transferAddressX();
    void transferAddressY();

    void resetSpriteOverflowEvaluation();
    void clockSpriteOverflowEvaluation();
    void rebuildSpriteOverflowEvaluation();
    bool spriteEvalValueInRange(uint8_t value) const;
    void clockSpriteFetches();
    uint16_t spritePatternAddress(uint8_t slot) const;
    static uint8_t reverseBits(uint8_t value);
    void getSpritePixel(uint8_t& pixel, uint8_t& palette, uint8_t& priority);
    static bool spriteZeroHitEligible(int cycle, bool spriteZeroActive, uint8_t fgPixel, uint8_t bgPixel);

    void touchOamRow(uint8_t row);
    void updateOamDecay();
    void corruptOamRow(uint8_t row);
    uint8_t oamRandomByte();

    uint8_t readBusLatchWithDecay();
    uint8_t oamDataReadValue() const;
    uint8_t currentSecondaryOamAddressForCorruption() const;
    void driveBusLatch(uint8_t value, uint8_t drivenMask);

    uint32_t nesColor(uint8_t index) const;

    void resetState(bool clearMemory);
};
