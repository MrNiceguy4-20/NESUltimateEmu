#include "PPU.hpp"
#include <cstring>
#include "Cartridge.hpp"
#include "CPU.hpp"

static const uint32_t kNesPalette[64] = {
    0xFF666666, 0xFF002A88, 0xFF1412A7, 0xFF3B00A4, 0xFF5C007E, 0xFF6E0040, 0xFF6C0600, 0xFF561D00,
    0xFF333500, 0xFF0B4800, 0xFF005200, 0xFF004F08, 0xFF00404D, 0xFF000000, 0xFF000000, 0xFF000000,
    0xFFADADAD, 0xFF155FD9, 0xFF4240FF, 0xFF7527FE, 0xFFA01ACC, 0xFFB71E7B, 0xFFB62926, 0xFF994E00,
    0xFF6B6D00, 0xFF388700, 0xFF0C9300, 0xFF008F32, 0xFF007C8D, 0xFF000000, 0xFF000000, 0xFF000000,
    0xFFFFFEFF, 0xFF64B0FF, 0xFF9290FF, 0xFFC676FF, 0xFFF36AFF, 0xFFFE6ECC, 0xFFFE8170, 0xFFEA9E22,
    0xFFBCBE00, 0xFF88D800, 0xFF5CE430, 0xFF45E082, 0xFF48CDDE, 0xFF4F4F4F, 0xFF000000, 0xFF000000,
    0xFFFFFEFF, 0xFFC0DFFF, 0xFFD3D2FF, 0xFFE8C8FF, 0xFFFBC2FF, 0xFFFEC4EA, 0xFFFECCC5, 0xFFF7D8A5,
    0xFFE4E594, 0xFFCFEF96, 0xFFBDF4AB, 0xFFB3F3CC, 0xFFB5EBF2, 0xFFB8B8B8, 0xFF000000, 0xFF000000
};

PPU::PPU()
{
    m_framebuffer.fill(0xFF000000);

    m_masterClock = 0;
    for (int i = 0; i < 32; i++) {
        m_oamDecayCycles[i] = 0;
    }
    m_oamLfsr = 0x5A;

    m_busLatch = 0;
}

void PPU::connectCartridge(Cartridge* cart) { m_cart = cart; }
void PPU::connectCPU(CPU* cpu) { m_cpu = cpu; }

uint32_t PPU::nesColor(uint8_t index) const
{
    return kNesPalette[index & 0x3F];
}

uint16_t PPU::mirrorNametable(uint16_t addr) const
{
    addr &= 0x0FFF;
    if (!m_cart)
        return addr & 0x07FF;

    using M = Cartridge::Mirror;
    switch (m_cart->mirroring()) {
    case M::Vertical:
        if (addr >= 0x0800) addr -= 0x0800;
        return addr;
    case M::Horizontal:
        if (addr >= 0x0400 && addr < 0x0800) addr -= 0x0400;
        else if (addr >= 0x0800 && addr < 0x0C00) addr -= 0x0400;
        else if (addr >= 0x0C00) addr -= 0x0800;
        return addr;
    case M::OnescreenLo:
        return addr & 0x03FF;
    case M::OnescreenHi:
        return 0x0400 + (addr & 0x03FF);
    }
    return addr & 0x07FF;
}

uint8_t PPU::ppuRead(uint16_t addr) const
{
    addr &= 0x3FFF;
    if (addr <= 0x1FFF) {
        if (m_cart) return m_cart->ppuRead(addr);
        return 0;
    }
    if (addr >= 0x2000 && addr <= 0x3EFF)
        return m_nametable[mirrorNametable(addr)];
    if (addr >= 0x3F00 && addr <= 0x3FFF) {
        addr &= 0x1F;
        if ((addr & 0x13) == 0x10) addr &= ~0x10;
        return m_palette[addr];
    }
    return 0;
}

void PPU::ppuWrite(uint16_t addr, uint8_t data)
{
    addr &= 0x3FFF;
    if (addr <= 0x1FFF) {
        if (m_cart) m_cart->ppuWrite(addr, data);
        return;
    }
    if (addr >= 0x2000 && addr <= 0x3EFF) {
        m_nametable[mirrorNametable(addr)] = data;
        return;
    }
    if (addr >= 0x3F00 && addr <= 0x3FFF) {
        addr &= 0x1F;
        if ((addr & 0x13) == 0x10) addr &= ~0x10;
        m_palette[addr] = data;
        return;
    }
}

void PPU::incrementScrollX()
{
    if (!renderingEnabled()) return;
    if ((m_v & 0x001F) == 31) {
        m_v &= ~0x001F;
        m_v ^= 0x0400;
    }
    else {
        m_v++;
    }
}

void PPU::incrementScrollY()
{
    if (!renderingEnabled()) return;
    if ((m_v & 0x7000) != 0x7000) {
        m_v += 0x1000;
    }
    else {
        m_v &= ~0x7000;
        uint16_t y = (m_v & 0x03E0) >> 5;
        if (y == 29) {
            y = 0;
            m_v ^= 0x0800;
        }
        else if (y == 31) {
            y = 0;
        }
        else {
            y++;
        }
        m_v = (m_v & ~0x03E0) | (y << 5);
    }
}

void PPU::transferAddressX()
{
    if (!renderingEnabled()) return;
    m_v = (m_v & 0xFBE0) | (m_t & 0x041F);
}

void PPU::transferAddressY()
{
    if (!renderingEnabled()) return;
    m_v = (m_v & 0x841F) | (m_t & 0x7BE0);
}

void PPU::loadBackgroundShifters()
{
    m_bgShifterPatternLo = (m_bgShifterPatternLo & 0xFF00) | m_bgNextTileLsb;
    m_bgShifterPatternHi = (m_bgShifterPatternHi & 0xFF00) | m_bgNextTileMsb;
    m_bgShifterAttrLo = (m_bgShifterAttrLo & 0xFF00) | ((m_bgNextTileAttr & 0x01) ? 0xFF : 0x00);
    m_bgShifterAttrHi = (m_bgShifterAttrHi & 0xFF00) | ((m_bgNextTileAttr & 0x02) ? 0xFF : 0x00);
}

void PPU::updateBackgroundShifters()
{
    if (m_mask & 0x08) {
        m_bgShifterPatternLo <<= 1;
        m_bgShifterPatternHi <<= 1;
        m_bgShifterAttrLo <<= 1;
        m_bgShifterAttrHi <<= 1;
    }
}

// ---------------------------------------------------------
// OAM decay helpers (Mesen-style curve)
// ---------------------------------------------------------
void PPU::touchOamRow(uint8_t row)
{
    if (row < 32) {
        m_oamDecayCycles[row] = static_cast<uint32_t>(m_masterClock);
    }
}

uint8_t PPU::oamRandomByte()
{
    // Simple 8-bit LFSR for pseudo-random decay noise
    uint8_t bit = ((m_oamLfsr >> 0) ^ (m_oamLfsr >> 2) ^ (m_oamLfsr >> 3) ^ (m_oamLfsr >> 4)) & 1;
    m_oamLfsr = static_cast<uint8_t>((m_oamLfsr >> 1) | (bit << 7));
    return m_oamLfsr;
}

void PPU::corruptOamRow(uint8_t row)
{
    if (row >= 32) return;

    int base = row * 8;

    // Primary OAM bytes (8 bytes per row)
    for (int i = 0; i < 8; i++) {
        uint8_t v = m_oam[base + i];
        uint8_t noise = oamRandomByte();
        // Mix existing bits with noise to simulate gradual decay
        m_oam[base + i] = (v & noise) | (~noise & (v ^ noise));
    }

    // Secondary OAM byte (9th byte in the DRAM row)
    uint8_t v2 = m_oamSecondary[row];
    uint8_t noise2 = oamRandomByte();
    m_oamSecondary[row] = (v2 & noise2) | (~noise2 & (v2 ^ noise2));
}

void PPU::updateOamDecay()
{
    // Approximate Mesen decay curve: rows decay at slightly different ages
    for (uint8_t row = 0; row < 32; ++row) {
        uint32_t threshold = 60000u + static_cast<uint32_t>(row) * 2000u;
        uint32_t last = m_oamDecayCycles[row];
        uint32_t age = static_cast<uint32_t>(m_masterClock - static_cast<uint64_t>(last));
        if (age > threshold) {
            corruptOamRow(row);
            m_oamDecayCycles[row] = static_cast<uint32_t>(m_masterClock);
        }
    }
}

// ---------------------------------------------------------
// Sprites
// ---------------------------------------------------------
void PPU::evaluateSprites()
{
    // Clear secondary OAM
    for (int i = 0; i < 32; i++) {
        m_oamSecondary[i] = 0xFF;
        touchOamRow(static_cast<uint8_t>(i));
    }
    m_spriteCount = 0;
    m_spriteZeroPossible = false;

    const int spriteH = (m_ctrl & 0x20) ? 16 : 8;

    for (int i = 0; i < 64 && m_spriteCount < 9; i++) {
        int diff = m_scanline - (int)m_oam[i * 4 + 0];
        // Touch the row for this sprite's primary OAM bytes
        uint8_t row = static_cast<uint8_t>((i * 4) >> 3);
        touchOamRow(row);

        if (diff >= 0 && diff < spriteH) {
            if (m_spriteCount < 8) {
                if (i == 0)
                    m_spriteZeroPossible = true;
                m_oamSecondary[m_spriteCount * 4 + 0] = m_oam[i * 4 + 0];
                m_oamSecondary[m_spriteCount * 4 + 1] = m_oam[i * 4 + 1];
                m_oamSecondary[m_spriteCount * 4 + 2] = m_oam[i * 4 + 2];
                m_oamSecondary[m_spriteCount * 4 + 3] = m_oam[i * 4 + 3];
                touchOamRow(m_spriteCount);
            }
            m_spriteCount++;
        }
    }

    if (m_spriteCount > 8) {
        m_status |= 0x20; // sprite overflow
        m_spriteCount = 8;
    }
}

void PPU::loadSpriteShifters()
{
    for (uint8_t i = 0; i < m_spriteCount; i++) {
        uint8_t tileY = m_oamSecondary[i * 4 + 0];
        uint8_t tileId = m_oamSecondary[i * 4 + 1];
        uint8_t attr = m_oamSecondary[i * 4 + 2];
        uint8_t tileX = m_oamSecondary[i * 4 + 3];

        touchOamRow(i);

        m_spriteAttr[i] = attr;
        m_spriteX[i] = tileX;

        uint16_t addr = 0;
        int row = m_scanline - (int)tileY;

        if (m_ctrl & 0x20) {
            // 8x16
            if (attr & 0x80) row = 15 - row;
            uint8_t half = tileId & 0x01;
            tileId &= 0xFE;
            if (row >= 8) {
                tileId |= 1;
                row -= 8;
            }
            addr = (uint16_t)half * 0x1000 + (uint16_t)tileId * 16 + (uint16_t)row;
        }
        else {
            // 8x8
            if (attr & 0x80) row = 7 - row;
            uint16_t base = (m_ctrl & 0x08) ? 0x1000 : 0x0000;
            addr = base + (uint16_t)tileId * 16 + (uint16_t)row;
        }

        uint8_t lo = ppuRead(addr);
        uint8_t hi = ppuRead(addr + 8);

        if (attr & 0x40) {
            // horizontal flip
            auto flip = [](uint8_t b) {
                b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
                b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
                b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
                return b;
                };
            lo = flip(lo);
            hi = flip(hi);
        }

        m_spriteShifterLo[i] = lo;
        m_spriteShifterHi[i] = hi;
    }

    for (int i = m_spriteCount; i < 8; i++) {
        m_spriteShifterLo[i] = 0;
        m_spriteShifterHi[i] = 0;
        m_spriteX[i] = 0xFF;
        m_spriteAttr[i] = 0;
    }
}

void PPU::getSpritePixel(uint8_t& pixel, uint8_t& palette, uint8_t& priority)
{
    pixel = 0;
    palette = 0;
    priority = 0;
    m_spriteZeroBeingRendered = false;

    if (!(m_mask & 0x10))
        return;

    for (uint8_t i = 0; i < m_spriteCount; i++) {
        if (m_spriteX[i] == 0) {
            uint8_t p0 = (m_spriteShifterLo[i] & 0x80) ? 1 : 0;
            uint8_t p1 = (m_spriteShifterHi[i] & 0x80) ? 1 : 0;
            uint8_t px = (p1 << 1) | p0;

            if (px != 0) {
                if (i == 0 && m_spriteZeroPossible)
                    m_spriteZeroBeingRendered = true;
                pixel = px;
                palette = (m_spriteAttr[i] & 0x03) + 4; // sprite palettes $3F10+
                priority = (m_spriteAttr[i] & 0x20) ? 1 : 0; // 1 = behind bg
                break; // first opaque sprite wins
            }
        }
    }
}

// ---------------------------------------------------------
// Main clock
// ---------------------------------------------------------
void PPU::clock()
{
    // Master clock for OAM decay timing
    m_masterClock++;

    if (m_scanline >= -1 && m_scanline < 240) {

        if (m_scanline == 0 && m_cycle == 0)
            m_cycle = 1;

        if (m_scanline == -1 && m_cycle == 1) {
            clearVBlank();
            m_status &= ~0x40;
            m_status &= ~0x20;
        }

        if ((m_cycle >= 2 && m_cycle < 258) || (m_cycle >= 321 && m_cycle < 338)) {
            updateBackgroundShifters();

            // Shift sprites
            if (m_mask & 0x10) {
                for (int i = 0; i < 8; i++) {
                    if (m_spriteX[i] > 0) {
                        m_spriteX[i]--;
                    }
                    else {
                        m_spriteShifterLo[i] <<= 1;
                        m_spriteShifterHi[i] <<= 1;
                    }
                }
            }

            switch ((m_cycle - 1) % 8) {
            case 0:
                loadBackgroundShifters();
                m_bgNextTileId = ppuRead(0x2000 | (m_v & 0x0FFF));
                break;
            case 2:
                m_bgNextTileAttr = ppuRead(0x23C0 | (m_v & 0x0C00) | ((m_v >> 4) & 0x38) | ((m_v >> 2) & 0x07));
                if (m_v & 0x40) m_bgNextTileAttr >>= 4;
                if (m_v & 0x02) m_bgNextTileAttr >>= 2;
                m_bgNextTileAttr &= 0x03;
                break;
            case 4: {
                uint16_t base = (m_ctrl & 0x10) ? 0x1000 : 0x0000;
                m_bgNextTileLsb = ppuRead(base + ((uint16_t)m_bgNextTileId << 4) + ((m_v >> 12) & 0x7));
                break;
            }
            case 6: {
                uint16_t base = (m_ctrl & 0x10) ? 0x1000 : 0x0000;
                m_bgNextTileMsb = ppuRead(base + ((uint16_t)m_bgNextTileId << 4) + ((m_v >> 12) & 0x7) + 8);
                break;
            }
            case 7:
                incrementScrollX();
                break;
            }
        }

        if (m_cycle == 256)
            incrementScrollY();

        if (m_cycle == 257) {
            transferAddressX();
            // Evaluate sprites for NEXT scanline (simplified: evaluate at end of line)
            if (m_scanline >= 0)
                evaluateSprites();
        }

        if (m_cycle == 340 && m_scanline >= 0)
            loadSpriteShifters();

        if (m_scanline == -1 && m_cycle >= 280 && m_cycle < 305)
            transferAddressY();

        // ----- Render pixel -----
        if (m_scanline >= 0 && m_scanline < 240 && m_cycle >= 1 && m_cycle <= 256) {
            // Background
            uint8_t bgPixel = 0, bgPalette = 0;
            if (m_mask & 0x08) {
                uint16_t bitMux = 0x8000 >> m_x;
                uint8_t p0 = (m_bgShifterPatternLo & bitMux) ? 1 : 0;
                uint8_t p1 = (m_bgShifterPatternHi & bitMux) ? 1 : 0;
                bgPixel = (p1 << 1) | p0;
                uint8_t a0 = (m_bgShifterAttrLo & bitMux) ? 1 : 0;
                uint8_t a1 = (m_bgShifterAttrHi & bitMux) ? 1 : 0;
                bgPalette = (a1 << 1) | a0;
            }
            if (m_cycle <= 8 && !(m_mask & 0x02))
                bgPixel = 0;

            // Sprite
            uint8_t fgPixel = 0, fgPalette = 0, fgPriority = 0;
            getSpritePixel(fgPixel, fgPalette, fgPriority);
            if (m_cycle <= 8 && !(m_mask & 0x04))
                fgPixel = 0;

            // Sprite 0 hit
            if (m_spriteZeroBeingRendered && bgPixel != 0 && m_cycle != 256)
                m_status |= 0x40;

            // Multiplexer
            uint8_t pixel = 0, palette = 0;
            if (bgPixel == 0 && fgPixel == 0) {
                pixel = 0; palette = 0;
            }
            else if (bgPixel == 0 && fgPixel > 0) {
                pixel = fgPixel; palette = fgPalette;
            }
            else if (bgPixel > 0 && fgPixel == 0) {
                pixel = bgPixel; palette = bgPalette;
            }
            else {
                // both opaque
                if (fgPriority == 0) {
                    pixel = fgPixel; palette = fgPalette;
                }
                else {
                    pixel = bgPixel; palette = bgPalette;
                }
            }

            uint8_t colorIndex = ppuRead(0x3F00 + ((palette << 2) | pixel));
            m_framebuffer[m_scanline * 256 + (m_cycle - 1)] = nesColor(colorIndex);
        }
    }

    if (m_scanline == 241 && m_cycle == 1) {
        setVBlank();
        if ((m_ctrl & 0x80) && m_cpu)
            m_cpu->nmi();
    }

    // Apply OAM decay once per PPU cycle
    updateOamDecay();

    m_cycle = m_cycle + 1;
    if (m_cycle > 340) {
        m_cycle = 0;
        m_scanline = m_scanline + 1;
        // MMC3 IRQ clock: once per scanline while rendering (0-239 and pre-render)
        if (m_scanline >= 0 && m_scanline < 240 && renderingEnabled() && m_cart)
            m_cart->scanlineTick();
        if (m_scanline > 260) {
            m_scanline = -1;
            m_frameComplete = true;
        }
    }
}

uint8_t PPU::cpuRead(uint16_t addr)
{
    uint8_t data = m_busLatch;

    switch (addr & 0x7) {
    case 2:
        data = (m_status & 0xE0) | (m_dataBuffer & 0x1F);
        clearVBlank();
        m_w = false;
        m_busLatch = data;
        break;
    case 4: {
        // OAM read
        uint8_t row = static_cast<uint8_t>(m_oamAddr >> 3);
        touchOamRow(row);
        data = m_oam[m_oamAddr];
        m_busLatch = data;
        break;
    }
    case 7:
        data = m_dataBuffer;
        m_dataBuffer = ppuRead(m_v);
        if (m_v >= 0x3F00)
            data = m_dataBuffer;
        m_v += (m_ctrl & 0x04) ? 32 : 1;
        m_busLatch = data;
        break;
    default:
        // Unused/unknown registers return open bus
        break;
    }

    return data;
}

void PPU::cpuWrite(uint16_t addr, uint8_t data)
{
    switch (addr & 0x7) {
    case 0:
        m_ctrl = data;
        m_t = (m_t & 0xF3FF) | ((uint16_t)(data & 0x03) << 10);
        break;
    case 1:
        m_mask = data;
        break;
    case 3:
        m_oamAddr = data;
        break;
    case 4: {
        // OAM write
        uint8_t row = static_cast<uint8_t>(m_oamAddr >> 3);
        touchOamRow(row);
        m_oam[m_oamAddr++] = data;
        break;
    }
    case 5:
        if (!m_w) {
            m_x = data & 0x07;
            m_t = (m_t & 0xFFE0) | (data >> 3);
            m_w = true;
        }
        else {
            m_t = (m_t & 0x8FFF) | ((uint16_t)(data & 0x07) << 12);
            m_t = (m_t & 0xFC1F) | ((uint16_t)(data & 0xF8) << 2);
            m_w = false;
        }
        break;
    case 6:
        if (!m_w) {
            m_t = (m_t & 0x00FF) | ((uint16_t)(data & 0x3F) << 8);
            m_w = true;
        }
        else {
            m_t = (m_t & 0xFF00) | data;
            m_v = m_t;
            m_w = false;
        }
        break;
    case 7:
        ppuWrite(m_v, data);
        m_v += (m_ctrl & 0x04) ? 32 : 1;
        break;
    }

    // Mesen-Lite masked-write open bus: only D0-D4 updated, D5-D7 preserved
    m_busLatch = (m_busLatch & 0xE0) | (data & 0x1F);
}

void PPU::setVBlank() { m_status |= 0x80; }
void PPU::clearVBlank() { m_status &= ~0x80; }

void PPU::saveState(std::vector<uint8_t>& out) const
{
    auto put8 = [&](uint8_t v) { out.push_back(v); };
    auto put16 = [&](uint16_t v) { out.push_back(v & 0xFF); out.push_back((v >> 8) & 0xFF); };
    auto put32 = [&](uint32_t v) {
        for (int i = 0; i < 4; i++) out.push_back((v >> (i * 8)) & 0xFF);
        };
    auto putBytes = [&](const uint8_t* d, size_t n) {
        out.insert(out.end(), d, d + n);
        };

    put32((uint32_t)(int32_t)m_scanline);
    put32((uint32_t)m_cycle);
    put8(m_frameComplete ? 1 : 0);
    put8(m_ctrl); put8(m_mask); put8(m_status); put8(m_oamAddr);
    put16(m_v); put16(m_t); put8(m_x); put8(m_w ? 1 : 0);
    put8(m_dataBuffer);
    putBytes(m_oam, 256);
    putBytes(m_nametable, 2048);
    putBytes(m_palette, 32);
    put8(m_busLatch);

    // OAM decay state
    put32((uint32_t)(m_masterClock & 0xFFFFFFFFu));
    put32((uint32_t)((m_masterClock >> 32) & 0xFFFFFFFFu));
    for (int i = 0; i < 32; i++) {
        put32(m_oamDecayCycles[i]);
    }
    put8(m_oamLfsr);
}

bool PPU::loadState(const uint8_t*& p, const uint8_t* end)
{
    auto need = [&](size_t n) { return p + n <= end; };
    auto get8 = [&](uint8_t& v) -> bool {
        if (!need(1)) return false; v = *p++; return true;
        };
    auto get16 = [&](uint16_t& v) -> bool {
        if (!need(2)) return false;
        v = p[0] | (uint16_t(p[1]) << 8); p += 2; return true;
        };
    auto get32 = [&](uint32_t& v) -> bool {
        if (!need(4)) return false;
        v = p[0] | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
        p += 4; return true;
        };
    auto getBytes = [&](uint8_t* d, size_t n) -> bool {
        if (!need(n)) return false;
        memcpy(d, p, n); p += n; return true;
        };

    uint32_t sl = 0, cy = 0;
    uint8_t fc = 0, w = 0;
    if (!get32(sl) || !get32(cy) || !get8(fc)) return false;
    m_scanline = (int)(int32_t)sl;
    m_cycle = (int)cy;
    m_frameComplete = fc != 0;
    if (!get8(m_ctrl) || !get8(m_mask) || !get8(m_status) || !get8(m_oamAddr)) return false;
    if (!get16(m_v) || !get16(m_t) || !get8(m_x) || !get8(w)) return false;
    m_w = w != 0;
    if (!get8(m_dataBuffer)) return false;
    if (!getBytes(m_oam, 256)) return false;
    if (!getBytes(m_nametable, 2048)) return false;
    if (!getBytes(m_palette, 32)) return false;
    if (!get8(m_busLatch)) return false;

    // OAM decay state
    uint32_t mcLo = 0, mcHi = 0;
    if (!get32(mcLo) || !get32(mcHi)) return false;
    m_masterClock = (uint64_t)mcLo | ((uint64_t)mcHi << 32);
    for (int i = 0; i < 32; i++) {
        uint32_t t = 0;
        if (!get32(t)) return false;
        m_oamDecayCycles[i] = t;
    }
    if (!get8(m_oamLfsr)) return false;

    return true;
}








