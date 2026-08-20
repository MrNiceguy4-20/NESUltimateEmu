#pragma once
#include <cstdint>
#include <array>
#include <vector>
#include "Mapper.hpp"

class Cartridge;
class CPU;

class PPU {
public:
    PPU();

    void connectCartridge(Cartridge* cart);
    void connectCPU(CPU* cpu);

    // Reset the PPU's live register/timing pipeline while preserving PPU
    // memory. This models the console Reset button rather than a cold boot.
    void reset();

    // Cold-boot the PPU for a newly inserted cartridge. In addition to the
    // register/timing pipeline, this clears VRAM/OAM/palette/framebuffer state
    // so a newly loaded game cannot inherit state from the previous image.
    void powerOn();

    void clock();

    uint8_t cpuRead(uint16_t addr);
    void    cpuWrite(uint16_t addr, uint8_t data);

    // OAM DMA writes behave like writes through $2004: they target the
    // current OAMADDR and increment/wrap it after every byte.
    void oamDmaWrite(uint8_t data);

    const uint32_t* framebuffer() const { return m_framebuffer.data(); }
    bool frameComplete() const { return m_frameComplete; }
    void clearFrameComplete() { m_frameComplete = false; }

    int scanline() const { return m_scanline; }
    int cycle() const { return m_cycle; }

#ifdef NES_HEADLESS
    // Regression-only observability. These accessors do not mutate emulated
    // state and are omitted from the normal frontend build.
    uint16_t testVramAddress() const { return m_v; }
    uint8_t testReadBuffer() const { return m_dataBuffer; }
    uint8_t testStatus() const { return m_status; }
    bool testWriteToggle() const { return m_w; }
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

    uint8_t m_ctrl = 0;
    uint8_t m_mask = 0;
    uint8_t m_status = 0;
    uint8_t m_oamAddr = 0;

    uint16_t m_v = 0;
    uint16_t m_t = 0;
    uint8_t  m_x = 0;
    bool     m_w = false;

    uint8_t m_dataBuffer = 0;
    uint8_t m_oam[256] = {};
    uint8_t m_oamSecondary[32] = {};  // 8 sprites × 4 bytes
    uint8_t m_spriteCount = 0;
    bool    m_spriteZeroPossible = false;
    bool    m_spriteZeroBeingRendered = false;

    // Dot-timed sprite evaluator state used for the $2002 overflow flag.
    // Rendering still consumes the batch-built secondary OAM below, but the
    // overflow detector follows primary OAM during dots 65-256 so flag timing
    // and the hardware diagonal-byte bug remain externally observable.
    uint8_t m_spriteEvalN = 0;      // primary OAM sprite index (0-64)
    uint8_t m_spriteEvalM = 0;      // byte within sprite (0-3)
    uint8_t m_spriteEvalFound = 0;  // sprites copied into secondary OAM (0-8)
    uint8_t m_spriteEvalData = 0;   // odd-dot primary OAM read latch
    bool    m_spriteEvalFull = false;

    // Per-sprite shift state for current scanline
    uint8_t m_spriteShifterLo[8] = {};
    uint8_t m_spriteShifterHi[8] = {};
    uint8_t m_spriteX[8] = {};
    uint8_t m_spriteAttr[8] = {};

    uint8_t m_nametable[4096] = {};  // 4KB for four-screen
    uint8_t m_palette[32] = {};

    std::array<uint32_t, 256 * 240> m_framebuffer{};

    // Background pipeline
    uint8_t  m_bgNextTileId = 0;
    uint8_t  m_bgNextTileAttr = 0;
    uint8_t  m_bgNextTileLsb = 0;
    uint8_t  m_bgNextTileMsb = 0;
    uint16_t m_bgShifterPatternLo = 0;
    uint16_t m_bgShifterPatternHi = 0;
    uint16_t m_bgShifterAttrLo = 0;
    uint16_t m_bgShifterAttrHi = 0;

    // OAM decay (Mesen-style curve)
    uint64_t m_masterClock = 0;
    uint32_t m_oamDecayCycles[32] = {};
    uint8_t  m_oamLfsr = 0x5A;

    // PPU I/O open-bus / decay latch. Each bit is dynamic and decays
    // independently toward 0 if it is not actively refreshed. CPU writes
    // drive all 8 bits. Reads only refresh the bits actually driven by the
    // selected PPU register.
    uint8_t  m_busLatch = 0;
    uint64_t m_busBitRefreshClock[8] = {};

    // VBlank/NMI edge state. The PPU NMI output is effectively
    // (PPUSTATUS.VBlank && PPUCTRL.NMI-enable). A low-to-high edge is
    // presented to the CPU immediately, but for the edge dot and the next
    // PPU dot it remains cancelable by the VBlank race: reading $2002 or
    // disabling NMI through $2000 can withdraw an edge not yet sampled.
    bool     m_nmiLine = false;
    uint8_t  m_nmiDelay = 0; // remaining cancelable PPU dots (legacy state slot)
    bool     m_suppressVBlank = false;

    uint8_t  ppuRead(uint16_t addr, PpuFetchKind kind = PpuFetchKind::Cpu);
    void     ppuWrite(uint16_t addr, uint8_t data);
    uint16_t mirrorNametable(uint16_t addr) const;

    void setVBlank();
    void clearVBlank();
    void updateNmiLine();
    void clockNmiDelay();
    void notifyPpuAddress(uint16_t addr);
    bool renderingEnabled() const { return (m_mask & 0x18) != 0; }

    void loadBackgroundShifters();
    void updateBackgroundShifters();
    void incrementScrollX();
    void incrementScrollY();
    void transferAddressX();
    void transferAddressY();

    void evaluateSprites();
    void resetSpriteOverflowEvaluation();
    void clockSpriteOverflowEvaluation();
    void rebuildSpriteOverflowEvaluation();
    bool spriteEvalValueInRange(uint8_t value) const;
    void loadSpriteShifters();
    void getSpritePixel(uint8_t& pixel, uint8_t& palette, uint8_t& priority);

    // OAM decay helpers
    void touchOamRow(uint8_t row);
    void updateOamDecay();
    void corruptOamRow(uint8_t row);
    uint8_t oamRandomByte();

    // PPU I/O open-bus helpers
    uint8_t readBusLatchWithDecay();
    void driveBusLatch(uint8_t value, uint8_t drivenMask);

    uint32_t nesColor(uint8_t index) const;

    void resetState(bool clearMemory);
};






















