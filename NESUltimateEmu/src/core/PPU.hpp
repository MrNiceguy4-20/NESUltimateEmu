#pragma once
#include <cstdint>
#include <array>
#include <vector>

class Cartridge;
class CPU;

class PPU {
public:
    PPU();

    void connectCartridge(Cartridge* cart);
    void connectCPU(CPU* cpu);

    void clock();

    uint8_t cpuRead(uint16_t addr);
    void    cpuWrite(uint16_t addr, uint8_t data);

    void oamWrite(uint8_t index, uint8_t data) { m_oam[index] = data; }

    const uint32_t* framebuffer() const { return m_framebuffer.data(); }
    bool frameComplete() const { return m_frameComplete; }
    void clearFrameComplete() { m_frameComplete = false; }

    int scanline() const { return m_scanline; }
    int cycle() const { return m_cycle; }

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

private:
    Cartridge* m_cart = nullptr;
    CPU* m_cpu = nullptr;

    int  m_scanline = 0;
    int  m_cycle = 0;
    bool m_frameComplete = false;

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

    // Per-sprite shift state for current scanline
    uint8_t m_spriteShifterLo[8] = {};
    uint8_t m_spriteShifterHi[8] = {};
    uint8_t m_spriteX[8] = {};
    uint8_t m_spriteAttr[8] = {};

    uint8_t m_nametable[2048] = {};
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

    // Mesen-Lite open bus latch
    uint8_t  m_busLatch = 0;

    uint8_t  ppuRead(uint16_t addr) const;
    void     ppuWrite(uint16_t addr, uint8_t data);
    uint16_t mirrorNametable(uint16_t addr) const;

    void setVBlank();
    void clearVBlank();
    bool renderingEnabled() const { return (m_mask & 0x18) != 0; }

    void loadBackgroundShifters();
    void updateBackgroundShifters();
    void incrementScrollX();
    void incrementScrollY();
    void transferAddressX();
    void transferAddressY();

    void evaluateSprites();
    void loadSpriteShifters();
    void getSpritePixel(uint8_t& pixel, uint8_t& palette, uint8_t& priority);

    // OAM decay helpers
    void touchOamRow(uint8_t row);
    void updateOamDecay();
    void corruptOamRow(uint8_t row);
    uint8_t oamRandomByte();

    uint32_t nesColor(uint8_t index) const;
};







