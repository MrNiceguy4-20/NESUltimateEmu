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
    powerOn();
}

void PPU::connectCartridge(Cartridge* cart) { m_cart = cart; }
void PPU::connectCPU(CPU* cpu) { m_cpu = cpu; }

void PPU::reset()
{
    resetState(false);
}

void PPU::powerOn()
{
    resetState(true);
}

void PPU::resetState(bool clearMemory)
{
    m_scanline = 0;
    m_cycle = 0;
    m_frameComplete = false;
    m_oddFrame = false;

    m_ctrl = 0;
    m_mask = 0;
    m_status = 0;
    m_oamAddr = 0;

    m_v = 0;
    m_t = 0;
    m_x = 0;
    m_w = false;
    m_dataBuffer = 0;

    std::memset(m_oamSecondary, 0, sizeof(m_oamSecondary));
    m_spriteCount = 0;
    m_spriteZeroPossible = false;
    m_spriteZeroBeingRendered = false;
    resetSpriteOverflowEvaluation();
    std::memset(m_spriteShifterLo, 0, sizeof(m_spriteShifterLo));
    std::memset(m_spriteShifterHi, 0, sizeof(m_spriteShifterHi));
    std::memset(m_spriteX, 0, sizeof(m_spriteX));
    std::memset(m_spriteAttr, 0, sizeof(m_spriteAttr));

    m_bgNextTileId = 0;
    m_bgNextTileAttr = 0;
    m_bgNextTileLsb = 0;
    m_bgNextTileMsb = 0;
    m_bgShifterPatternLo = 0;
    m_bgShifterPatternHi = 0;
    m_bgShifterAttrLo = 0;
    m_bgShifterAttrHi = 0;

    m_busLatch = 0;
    for (uint64_t& t : m_busBitRefreshClock) t = m_masterClock;
    m_nmiLine = false;
    m_nmiDelay = 0;
    m_suppressVBlank = false;

    // Soft reset deliberately leaves the master timestamp and OAM decay
    // history alone because mapper A12 filters use this same clock domain.
    if (clearMemory) {
        m_masterClock = 0;
        std::memset(m_oamDecayCycles, 0, sizeof(m_oamDecayCycles));
        m_oamLfsr = 0x5A;
        std::memset(m_oam, 0, sizeof(m_oam));
        std::memset(m_nametable, 0, sizeof(m_nametable));
        std::memset(m_palette, 0, sizeof(m_palette));
        m_framebuffer.fill(0xFF000000);
    }
}

void PPU::oamDmaWrite(uint8_t data)
{
    const uint8_t row = static_cast<uint8_t>(m_oamAddr >> 3);
    touchOamRow(row);
    const uint8_t addr = m_oamAddr++;
    m_oam[addr] = ((addr & 0x03) == 0x02) ? static_cast<uint8_t>(data & 0xE3) : data;
}

uint32_t PPU::nesColor(uint8_t index) const
{
    // Grayscale: force color to one of the gray columns
    if (m_mask & 0x01)
        index &= 0x30;

    uint32_t c = kNesPalette[index & 0x3F];

    // Color emphasis ($2001 bits 5-7): darken channels slightly
    // Bit5=red emphasis → darken G+B, Bit6=green → darken R+B, Bit7=blue → darken R+G
    uint8_t emp = (m_mask >> 5) & 0x07;
    if (emp != 0) {
        uint8_t r = (c >> 16) & 0xFF;
        uint8_t g = (c >> 8) & 0xFF;
        uint8_t b = c & 0xFF;
        // Approximate hardware: emphasized color is boosted relatively by attenuating others
        if (emp & 0x01) { g = (uint8_t)(g * 0.75f); b = (uint8_t)(b * 0.75f); } // red emp
        if (emp & 0x02) { r = (uint8_t)(r * 0.75f); b = (uint8_t)(b * 0.75f); } // green emp
        if (emp & 0x04) { r = (uint8_t)(r * 0.75f); g = (uint8_t)(g * 0.75f); } // blue emp
        c = 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }
    return c;
}

uint16_t PPU::mirrorNametable(uint16_t addr) const
{
    addr &= 0x0FFF;
    if (!m_cart)
        return addr & 0x07FF;

    using M = Cartridge::Mirror;
    switch (m_cart->mirroring()) {
    case M::Vertical:
        // $2000=$2800, $2400=$2C00
        return addr & 0x07FF;
    case M::Horizontal:
        // $2000=$2400, $2800=$2C00
        if (addr >= 0x0400 && addr < 0x0800) return addr - 0x0400;
        if (addr >= 0x0800 && addr < 0x0C00) return addr - 0x0400;
        if (addr >= 0x0C00) return addr - 0x0800;
        return addr;
    case M::OnescreenLo:
        return addr & 0x03FF;
    case M::OnescreenHi:
        return 0x0400 + (addr & 0x03FF);
    case M::FourScreen:
        // All 4 nametables distinct — full 4KB
        return addr & 0x0FFF;
    }
    return addr & 0x07FF;
}


uint8_t PPU::ppuRead(uint16_t addr, PpuFetchKind kind)
{
    addr &= 0x3FFF;
    notifyPpuAddress(addr);
    if (addr <= 0x1FFF) {
        uint32_t ciram = 0;
        if (m_cart && m_cart->mapPatternCiram(addr, ciram))
            return m_nametable[ciram & 0x0FFF];
        if (m_cart) return m_cart->ppuRead(addr, kind);
        return 0;
    }
    if (addr >= 0x2000 && addr <= 0x3EFF) {
        NametableSource source = NametableSource::Ciram;
        uint32_t mapped = 0;
        if (m_cart && m_cart->mapNametable(addr, source, mapped)) {
            if (source == NametableSource::Ciram)
                return m_nametable[mapped & 0x0FFF];
            return m_cart->readNametableBacking(source, mapped);
        }
        return m_nametable[mirrorNametable(addr)];
    }
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
    notifyPpuAddress(addr);
    if (addr <= 0x1FFF) {
        uint32_t ciram = 0;
        if (m_cart && m_cart->mapPatternCiram(addr, ciram)) {
            m_nametable[ciram & 0x0FFF] = data;
            return;
        }
        if (m_cart) m_cart->ppuWrite(addr, data);
        return;
    }
    if (addr >= 0x2000 && addr <= 0x3EFF) {
        NametableSource source = NametableSource::Ciram;
        uint32_t mapped = 0;
        if (m_cart && m_cart->mapNametable(addr, source, mapped)) {
            if (source == NametableSource::Ciram)
                m_nametable[mapped & 0x0FFF] = data;
            else
                m_cart->writeNametableBacking(source, mapped, data);
            return;
        }
        m_nametable[mirrorNametable(addr)] = data;
        return;
    }
    if (addr >= 0x3F00 && addr <= 0x3FFF) {
        addr &= 0x1F;
        if ((addr & 0x13) == 0x10) addr &= ~0x10;
        m_palette[addr] = data & 0x3F;
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
void PPU::resetSpriteOverflowEvaluation()
{
    m_spriteEvalN = 0;
    m_spriteEvalM = 0;
    m_spriteEvalFound = 0;
    m_spriteEvalData = 0xFF;
    m_spriteEvalFull = false;
}

bool PPU::spriteEvalValueInRange(uint8_t value) const
{
    const int spriteH = (m_ctrl & 0x20) ? 16 : 8;
    const int row = m_scanline - static_cast<int>(value);
    return row >= 0 && row < spriteH;
}

void PPU::clockSpriteOverflowEvaluation()
{
    if (m_scanline < 0 || m_scanline >= 240 || m_cycle < 65 || m_cycle > 256)
        return;

    // A new scanline always starts with a fresh evaluator position. With
    // rendering disabled the OAM evaluation circuit then remains idle; $2001
    // never clears an overflow flag produced earlier in the frame.
    if (m_cycle == 65)
        resetSpriteOverflowEvaluation();

    if (!renderingEnabled())
        return;

    if (m_spriteEvalN >= 64)
        return;

    if (m_cycle & 1) {
        // Odd evaluation dots read primary OAM. The following even dot makes
        // the range/copy decision. Only the overflow flag needs this timed
        // path; the renderer's secondary-OAM image is still built in batch.
        const uint16_t index = static_cast<uint16_t>(m_spriteEvalN) * 4u + m_spriteEvalM;
        m_spriteEvalData = m_oam[index & 0xFF];
        touchOamRow(static_cast<uint8_t>((index & 0xFF) >> 3));
        return;
    }

    // Before secondary OAM is full, an out-of-range Y skips the whole sprite;
    // an in-range Y causes all four bytes to consume evaluation cycles.
    if (m_spriteEvalFound < 8) {
        if (m_spriteEvalM == 0) {
            if (spriteEvalValueInRange(m_spriteEvalData)) {
                ++m_spriteEvalFound;
                m_spriteEvalM = 1;
            }
            else {
                ++m_spriteEvalN;
            }
        }
        else {
            ++m_spriteEvalM;
            if (m_spriteEvalM >= 4) {
                m_spriteEvalM = 0;
                ++m_spriteEvalN;
            }
        }
        return;
    }

    // The eighth sprite's remaining three bytes still have to be copied
    // before the overflow search begins. Once complete, m may later become
    // non-zero again due to the diagonal bug, so track the full condition
    // independently rather than inferring it from m.
    if (!m_spriteEvalFull) {
        ++m_spriteEvalM;
        if (m_spriteEvalM >= 4) {
            m_spriteEvalM = 0;
            ++m_spriteEvalN;
            m_spriteEvalFull = true;
        }
        return;
    }

    // Secondary OAM is full. The real 2C02 now has the famous diagonal-read
    // bug: after each failed range comparison both n and m advance, so the
    // next candidate is byte 1 of the following sprite, then byte 2 of the
    // next, byte 3, byte 0, and so on. Any byte that looks like an in-range Y
    // sets sprite overflow. Stop naturally at sprite 64; never wrap to OAM 0.
    if (spriteEvalValueInRange(m_spriteEvalData))
        m_status |= 0x20;

    ++m_spriteEvalN;
    m_spriteEvalM = static_cast<uint8_t>((m_spriteEvalM + 1) & 0x03);
}

void PPU::rebuildSpriteOverflowEvaluation()
{
    // The save-state payload predates the per-dot evaluator fields. Rebuild
    // the n/m position from the saved scanline/cycle and current OAM without
    // changing the already-serialized overflow flag. This preserves the file
    // format while giving mid-scanline loads a deterministic continuation.
    resetSpriteOverflowEvaluation();
    if (m_scanline < 0 || m_scanline >= 240 || m_cycle <= 65 || !renderingEnabled())
        return;

    const uint8_t savedStatus = m_status;
    const int savedCycle = m_cycle;
    uint32_t savedDecayCycles[32];
    std::memcpy(savedDecayCycles, m_oamDecayCycles, sizeof(savedDecayCycles));

    for (int cy = 65; cy < savedCycle && cy <= 256; ++cy) {
        m_cycle = cy;
        clockSpriteOverflowEvaluation();
    }

    m_cycle = savedCycle;
    m_status = savedStatus;
    // Replaying the evaluator must not refresh OAM DRAM rows as a side effect
    // of loading a save state; restore the serialized decay timestamps.
    std::memcpy(m_oamDecayCycles, savedDecayCycles, sizeof(savedDecayCycles));
}

void PPU::evaluateSprites()
{
    // Secondary OAM is cleared before evaluation. Evaluation is performed for
    // the *next* scanline; using the current scanline value reproduces the
    // NES OAM Y convention (stored Y is one less than the first visible row).
    for (int i = 0; i < 32; i++) {
        m_oamSecondary[i] = 0xFF;
        touchOamRow(static_cast<uint8_t>(i));
    }
    m_spriteCount = 0;
    m_spriteZeroPossible = false;

    if (!renderingEnabled())
        return;

    const int evalLine = m_scanline;
    const int spriteH = (m_ctrl & 0x20) ? 16 : 8;
    uint8_t found = 0;

    for (uint8_t i = 0; i < 64; ++i) {
        const uint8_t base = static_cast<uint8_t>(i * 4);
        const int row = evalLine - static_cast<int>(m_oam[base]);
        touchOamRow(static_cast<uint8_t>(base >> 3));

        // Do not perform the subtraction in 8 bits. Sprite Y values near the
        // bottom of OAM ($F0-$FF) are offscreen and must not wrap around into
        // the top visible scanlines.
        if (row >= 0 && row < spriteH) {
            if (found < 8) {
                if (i == 0)
                    m_spriteZeroPossible = true;
                const uint8_t dst = static_cast<uint8_t>(found * 4);
                m_oamSecondary[dst + 0] = m_oam[base + 0];
                m_oamSecondary[dst + 1] = m_oam[base + 1];
                m_oamSecondary[dst + 2] = m_oam[base + 2];
                m_oamSecondary[dst + 3] = m_oam[base + 3];
                touchOamRow(found);
            }

            ++found;
            if (found == 9) {
                // This covers the normal overflow case. The silicon's
                // diagonal-byte false-positive bug is intentionally left for
                // a later microcycle-accurate sprite evaluator.
                m_status |= 0x20;
                break;
            }
        }
    }

    m_spriteCount = found > 8 ? 8 : found;
}

void PPU::loadSpriteShifters()
{
    // The hardware performs pattern fetches for all eight sprite slots during
    // HBlank, including dummy slots. We still batch the fetches in this
    // higher-level renderer, but issue all eight pattern-table accesses so
    // mapper A12 observes the correct table even when no sprites are visible.
    for (uint8_t i = 0; i < 8; i++) {
        if (i >= m_spriteCount) {
            uint16_t dummyAddr = 0;
            if (m_ctrl & 0x20) {
                // Empty secondary OAM contains $FF; in 8x16 mode bit 0 of the
                // tile number selects pattern table $1000.
                dummyAddr = 0x1000 + 0x0FE0;
            }
            else {
                const uint16_t base = (m_ctrl & 0x08) ? 0x1000 : 0x0000;
                dummyAddr = static_cast<uint16_t>(base + 0x0FF0);
            }
            (void)ppuRead(dummyAddr, PpuFetchKind::Sprite);
            (void)ppuRead(static_cast<uint16_t>(dummyAddr + 8), PpuFetchKind::Sprite);
            m_spriteShifterLo[i] = 0;
            m_spriteShifterHi[i] = 0;
            m_spriteX[i] = 0xFF;
            m_spriteAttr[i] = 0;
            continue;
        }

        uint8_t tileY = m_oamSecondary[i * 4 + 0];
        uint8_t tileId = m_oamSecondary[i * 4 + 1];
        uint8_t attr = m_oamSecondary[i * 4 + 2];
        uint8_t tileX = m_oamSecondary[i * 4 + 3];

        touchOamRow(i);

        m_spriteAttr[i] = attr;
        m_spriteX[i] = tileX;

        uint16_t addr = 0;
        // Sprite Y is stored one less than the first visible scanline. The
        // evaluation/fetch scanline wraps at the pre-render line (-1 -> 255).
        int row = static_cast<uint8_t>(static_cast<uint8_t>(m_scanline) - tileY);

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

        uint8_t lo = ppuRead(addr, PpuFetchKind::Sprite);
        uint8_t hi = ppuRead(addr + 8, PpuFetchKind::Sprite);

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
    // Age the short cancelable-NMI window before processing this dot. An NMI
    // edge created later in this same clock therefore remains cancelable for
    // the edge dot and the immediately following PPU dot.
    clockNmiDelay();

    if (m_cycle == 0 && m_cart)
        m_cart->notifyPpuScanline(m_scanline, renderingEnabled());

    // Master clock for OAM decay timing
    m_masterClock++;

    if (m_scanline >= -1 && m_scanline < 240) {

        // Dot 0 exists on every visible scanline. The NTSC odd-frame
        // shortening is handled only at the pre-render/frame boundary below;
        // skipping scanline 0 dot 0 here made every frame one PPU clock short
        // (and rendered odd frames two clocks short).

        if (m_scanline == -1 && m_cycle == 1) {
            clearVBlank();
            m_status &= ~0x40;
            m_status &= ~0x20;
        }

        // Background shifters continue through the two-tile prefetch window
        // at dots 321-336. Sprite X counters/shifters do not: those only run
        // while visible pixels are being produced. Advancing sprite state in
        // the background-prefetch window shifts every next-scanline sprite
        // left before the scanline begins (Punch-Out!! makes this very obvious).
        if (renderingEnabled() && ((m_cycle >= 2 && m_cycle < 258) || (m_cycle >= 321 && m_cycle < 338))) {
            updateBackgroundShifters();

            switch ((m_cycle - 1) % 8) {
            case 0:
                loadBackgroundShifters();
                m_bgNextTileId = ppuRead(0x2000 | (m_v & 0x0FFF), PpuFetchKind::Background);
                break;
            case 2:
                m_bgNextTileAttr = ppuRead(0x23C0 | (m_v & 0x0C00) | ((m_v >> 4) & 0x38) | ((m_v >> 2) & 0x07), PpuFetchKind::Background);
                if (m_v & 0x40) m_bgNextTileAttr >>= 4;
                if (m_v & 0x02) m_bgNextTileAttr >>= 2;
                m_bgNextTileAttr &= 0x03;
                break;
            case 4: {
                uint16_t base = (m_ctrl & 0x10) ? 0x1000 : 0x0000;
                m_bgNextTileLsb = ppuRead(base + ((uint16_t)m_bgNextTileId << 4) + ((m_v >> 12) & 0x7), PpuFetchKind::Background);
                break;
            }
            case 6: {
                uint16_t base = (m_ctrl & 0x10) ? 0x1000 : 0x0000;
                m_bgNextTileMsb = ppuRead(base + ((uint16_t)m_bgNextTileId << 4) + ((m_v >> 12) & 0x7) + 8, PpuFetchKind::Background);
                break;
            }
            case 7:
                incrementScrollX();
                break;
            }
        }


        // Sprite X counters and pattern shifters advance only during the
        // visible-pixel region. With this renderer, cycle 1 consumes the
        // first pixel without a pre-shift, so cycles 2-256 advance before
        // sampling the next pixel. Do not run this during dots 321-337.
        if (renderingEnabled() && (m_mask & 0x10) && m_cycle >= 2 && m_cycle <= 256) {
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

        // Sprite overflow evaluation is an independently visible OAM bus
        // process and must occur during dots 65-256, not when the renderer
        // batches secondary OAM at dot 257.
        clockSpriteOverflowEvaluation();

        if (m_cycle == 256)
            incrementScrollY();

        if (m_cycle == 257) {
            transferAddressX();
            // Secondary OAM clear/evaluation occurs on visible scanlines, not
            // on the pre-render scanline. OAM Y is one less than the first
            // visible row, so evaluating scanline N prepares sprites for N+1.
            // In particular, $EF-$FF must not wrap around and become scanline-0
            // sprites merely because the pre-render scanline is represented as -1.
            if (m_scanline >= 0 && m_scanline < 240)
                evaluateSprites();
        }

        // The current renderer loads all sprite patterns as a batch, but place
        // that synthetic fetch at the start of the hardware sprite-fetch
        // region so mapper A12 timing lands near the real dot (~260), not 340.
        if (m_cycle == 260 && m_scanline >= -1 && m_scanline < 240 && renderingEnabled()) {
            loadSpriteShifters();

            // The PPU still performs sprite fetches on the pre-render line, but
            // those fetches do not produce visible sprites on scanline 0. Keep
            // the bus activity (important for mapper latches/A12), then suppress
            // the renderer's use of the fetched sprite slots for scanline 0.
            if (m_scanline == -1) {
                m_spriteCount = 0;
                m_spriteZeroPossible = false;
                m_spriteZeroBeingRendered = false;
            }
        }

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

            // Sprite 0 hit requires an opaque sprite-zero pixel *after*
            // left-edge clipping, an opaque background pixel, and cannot occur
            // at x=255 (PPU cycle 256).
            if (m_spriteZeroBeingRendered && fgPixel != 0 && bgPixel != 0 && m_cycle != 256)
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

            // Palette RAM is internal to the PPU. Do not route this per-pixel
            // lookup through ppuRead(), because it is not an external CHR/VRAM
            // bus transaction and therefore must not generate mapper A12 edges.
            uint16_t paletteAddr = static_cast<uint16_t>((palette << 2) | pixel) & 0x1F;
            if ((paletteAddr & 0x13) == 0x10) paletteAddr &= ~0x10;
            uint8_t colorIndex = m_palette[paletteAddr];
            m_framebuffer[m_scanline * 256 + (m_cycle - 1)] = nesColor(colorIndex);
        }
    }

    if (m_scanline == 241 && m_cycle == 1)
        setVBlank();

    // Apply OAM decay once per PPU cycle
    updateOamDecay();

    m_cycle = m_cycle + 1;
    if (m_cycle > 340) {
        m_cycle = 0;
        m_scanline = m_scanline + 1;
        // MMC3 IRQ clocks via filtered PPU address-bus A12 edges
        if (m_scanline > 260) {
            m_scanline = -1;
            m_frameComplete = true;
            // NTSC: skip one PPU cycle on odd frames when rendering is enabled
            if (m_oddFrame && renderingEnabled())
                m_cycle = 1;
            m_oddFrame = !m_oddFrame;
        }
    }
}

uint8_t PPU::readBusLatchWithDecay()
{
    // Blargg's ppu_open_bus test only requires that all bits have decayed
    // by one second and that undriven bits are not refreshed by reads.
    // Real hardware varies by PPU, temperature, and individual bit. Use
    // deterministic per-bit lifetimes spanning roughly 500-650 ms.
    static constexpr uint64_t kPpuClocksPerSecond = 5369318ull;
    static constexpr uint16_t kDecayMs[8] = { 520, 610, 560, 640, 500, 590, 545, 625 };
    for (unsigned bit = 0; bit < 8; ++bit) {
        const uint8_t mask = static_cast<uint8_t>(1u << bit);
        if ((m_busLatch & mask) == 0)
            continue;
        const uint64_t threshold = (kPpuClocksPerSecond * kDecayMs[bit]) / 1000ull;
        if (m_masterClock - m_busBitRefreshClock[bit] >= threshold)
            m_busLatch = static_cast<uint8_t>(m_busLatch & ~mask);
    }
    return m_busLatch;
}

void PPU::driveBusLatch(uint8_t value, uint8_t drivenMask)
{
    readBusLatchWithDecay();
    m_busLatch = static_cast<uint8_t>((m_busLatch & ~drivenMask) | (value & drivenMask));
    for (unsigned bit = 0; bit < 8; ++bit) {
        if (drivenMask & (1u << bit))
            m_busBitRefreshClock[bit] = m_masterClock;
    }
}

uint8_t PPU::cpuRead(uint16_t addr)
{
    uint8_t data = readBusLatchWithDecay();

    switch (addr & 0x7) {
    case 2: {
        // PPUSTATUS drives only bits 7-5. Bits 4-0 are the independently
        // decaying PPU I/O bus and are not refreshed by this read.
        data = static_cast<uint8_t>((m_status & 0xE0) | (data & 0x1F));
        if (m_scanline == 241 && m_cycle == 1)
            m_suppressVBlank = true;
        clearVBlank();
        m_w = false;
        driveBusLatch(data, 0xE0);
        break;
    }
    case 4: {
        // OAMDATA drives all 8 PPU I/O bus bits. Attribute bytes physically
        // lack bits 2-4, so those bits always read as zero.
        const uint8_t row = static_cast<uint8_t>(m_oamAddr >> 3);
        touchOamRow(row);
        data = m_oam[m_oamAddr];
        if ((m_oamAddr & 0x03) == 0x02)
            data = static_cast<uint8_t>(data & 0xE3);
        driveBusLatch(data, 0xFF);
        break;
    }
    case 7: {
        const uint16_t vramAddr = m_v & 0x3FFF;
        if (vramAddr >= 0x3F00) {
            // Palette data drives only bits 5-0; bits 7-6 come from the PPU
            // open bus and therefore are not refreshed by palette reads.
            const uint8_t latch = readBusLatchWithDecay();
            data = static_cast<uint8_t>((latch & 0xC0) | (ppuRead(vramAddr) & 0x3F));
            m_dataBuffer = ppuRead(static_cast<uint16_t>((vramAddr - 0x1000) & 0x3FFF));
            driveBusLatch(data, 0x3F);
        }
        else {
            data = m_dataBuffer;
            m_dataBuffer = ppuRead(vramAddr);
            driveBusLatch(data, 0xFF);
        }
        m_v += (m_ctrl & 0x04) ? 32 : 1;
        notifyPpuAddress(m_v);
        break;
    }
    default:
        // Reads from write-only PPU registers return the decaying PPU bus
        // without refreshing any bit.
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
        updateNmiLine();
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
        const uint8_t oamAddr = m_oamAddr++;
        m_oam[oamAddr] = ((oamAddr & 0x03) == 0x02) ? static_cast<uint8_t>(data & 0xE3) : data;
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
            notifyPpuAddress(m_v);
        }
        break;
    case 7:
        ppuWrite(m_v, data);
        m_v += (m_ctrl & 0x04) ? 32 : 1;

        // As with PPUDATA reads, the post-write address increment is visible
        // to mapper hardware and can itself create an MMC3 A12 transition.
        notifyPpuAddress(m_v);
        break;
    }

    // CPU writes drive and refresh all eight PPU I/O bus bits, including
    // writes to nominally read-only registers such as $2002.
    driveBusLatch(data, 0xFF);
}

void PPU::notifyPpuAddress(uint16_t addr)
{
    if (m_cart)
        m_cart->notifyPpuAddress(static_cast<uint16_t>(addr & 0x3FFF), m_masterClock, m_scanline, m_cycle);
}

void PPU::updateNmiLine()
{
    const bool newLine = (m_status & 0x80) != 0 && (m_ctrl & 0x80) != 0;
    if (newLine && !m_nmiLine) {
        // Present the edge immediately. For this dot and the following dot,
        // the PPU can still cancel an edge the CPU has not sampled yet.
        m_nmiDelay = 2;
        if (m_cpu)
            m_cpu->nmi();
    }
    else if (!newLine) {
        if (m_nmiLine && m_nmiDelay != 0 && m_cpu)
            m_cpu->cancelPendingNmi();
        m_nmiDelay = 0;
    }
    m_nmiLine = newLine;
}

void PPU::clockNmiDelay()
{
    if (m_nmiDelay != 0)
        --m_nmiDelay;
}

void PPU::setVBlank()
{
    if (m_suppressVBlank) {
        m_suppressVBlank = false;
        clearVBlank();
        return;
    }
    m_status |= 0x80;
    updateNmiLine();
}

void PPU::clearVBlank()
{
    m_status &= ~0x80;
    updateNmiLine();
}

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
    put8(m_oddFrame ? 1 : 0);
    put8(m_ctrl); put8(m_mask); put8(m_status); put8(m_oamAddr);
    put16(m_v); put16(m_t); put8(m_x); put8(m_w ? 1 : 0);
    put8(m_dataBuffer);
    putBytes(m_oam, 256);
    putBytes(m_oamSecondary, 32);
    put8(m_spriteCount);
    put8(m_spriteZeroPossible ? 1 : 0);
    put8(m_spriteZeroBeingRendered ? 1 : 0);
    putBytes(m_spriteShifterLo, 8);
    putBytes(m_spriteShifterHi, 8);
    putBytes(m_spriteX, 8);
    putBytes(m_spriteAttr, 8);
    putBytes(m_nametable, 4096);
    putBytes(m_palette, 32);

    for (uint32_t pixel : m_framebuffer)
        put32(pixel);

    put8(m_bgNextTileId);
    put8(m_bgNextTileAttr);
    put8(m_bgNextTileLsb);
    put8(m_bgNextTileMsb);
    put16(m_bgShifterPatternLo);
    put16(m_bgShifterPatternHi);
    put16(m_bgShifterAttrLo);
    put16(m_bgShifterAttrHi);
    put8(m_busLatch);
    for (uint64_t t : m_busBitRefreshClock) {
        put32(static_cast<uint32_t>(t & 0xFFFFFFFFu));
        put32(static_cast<uint32_t>((t >> 32) & 0xFFFFFFFFu));
    }
    put8(m_nmiLine ? 1 : 0);
    put8(m_nmiDelay);
    put8(m_suppressVBlank ? 1 : 0);

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
    uint8_t fc = 0, odd = 0, w = 0;
    if (!get32(sl) || !get32(cy) || !get8(fc) || !get8(odd)) return false;
    m_scanline = (int)(int32_t)sl;
    m_cycle = (int)cy;
    m_frameComplete = fc != 0;
    m_oddFrame = odd != 0;
    if (!get8(m_ctrl) || !get8(m_mask) || !get8(m_status) || !get8(m_oamAddr)) return false;
    if (!get16(m_v) || !get16(m_t) || !get8(m_x) || !get8(w)) return false;
    m_w = w != 0;
    if (!get8(m_dataBuffer)) return false;
    if (!getBytes(m_oam, 256)) return false;
    if (!getBytes(m_oamSecondary, 32)) return false;
    if (!get8(m_spriteCount)) return false;
    if (!get8(fc)) return false; m_spriteZeroPossible = fc != 0;
    if (!get8(fc)) return false; m_spriteZeroBeingRendered = fc != 0;
    if (!getBytes(m_spriteShifterLo, 8)) return false;
    if (!getBytes(m_spriteShifterHi, 8)) return false;
    if (!getBytes(m_spriteX, 8)) return false;
    if (!getBytes(m_spriteAttr, 8)) return false;
    if (!getBytes(m_nametable, 4096)) return false;
    if (!getBytes(m_palette, 32)) return false;

    for (uint32_t& pixel : m_framebuffer) {
        uint32_t value = 0;
        if (!get32(value)) return false;
        pixel = value;
    }

    if (!get8(m_bgNextTileId) || !get8(m_bgNextTileAttr) ||
        !get8(m_bgNextTileLsb) || !get8(m_bgNextTileMsb)) return false;
    if (!get16(m_bgShifterPatternLo) || !get16(m_bgShifterPatternHi) ||
        !get16(m_bgShifterAttrLo) || !get16(m_bgShifterAttrHi)) return false;
    if (!get8(m_busLatch)) return false;
    for (uint64_t& t : m_busBitRefreshClock) {
        uint32_t lo = 0, hi = 0;
        if (!get32(lo) || !get32(hi)) return false;
        t = static_cast<uint64_t>(lo) | (static_cast<uint64_t>(hi) << 32);
    }
    if (!get8(fc)) return false; m_nmiLine = fc != 0;
    if (!get8(m_nmiDelay)) return false;
    if (!get8(fc)) return false; m_suppressVBlank = fc != 0;
    if (m_nmiDelay > 2) return false;

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

    rebuildSpriteOverflowEvaluation();
    return true;
}



































