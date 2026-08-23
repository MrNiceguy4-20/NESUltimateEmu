#include "PPU.hpp"
#include <algorithm>
#include <cstring>
#include "Cartridge.hpp"
#include "CPU.hpp"
#include <fstream>

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

uint64_t PPU::registerWriteInhibitDuration() const
{

    switch (m_timing) {
    case ConsoleTiming::PAL:   return 106023ull;
    case ConsoleTiming::Dendy: return 106024ull;
    default:                   return 88974ull;
    }
}

void PPU::setTiming(ConsoleTiming timing)
{
    const bool inhibited = registerWriteInhibited();
    const uint64_t elapsed = inhibited
        ? (registerWriteInhibitDuration() - (m_registerWriteInhibitUntilClock - m_masterClock))
        : 0;
    m_timing = timing;
    if (inhibited) {
        const uint64_t duration = registerWriteInhibitDuration();
        m_registerWriteInhibitUntilClock = m_masterClock + (elapsed < duration ? duration - elapsed : 0);
    }
}

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
    m_renderMask = 0;
    m_pendingRenderMask = 0;
    m_renderMaskDelay = 0;
    m_pendingVramAddress = 0;
    m_vramAddressDelay = 0;
    m_ppudataIncrementDelay = 0;
    m_ppudataIncrementPending = false;
    m_ppudataReadPending = false;
    m_ppudataReadDelay = 0;
    m_ppudataReadAddress = 0;
    m_ppuBusReadThisClock = false;
    m_ppuBusReadData = 0;
    m_ppuBusTraceDots = 0;
    m_ppuBusReadSource = "none";
    m_status = 0;
    m_oamAddr = 0;

    m_v = 0;
    m_t = 0;
    m_x = 0;
    m_w = false;
    m_dataBuffer = 0;
    m_renderFetchPending = false;
    m_renderFetchAddress = 0;
    m_renderFetchKind = PpuFetchKind::Background;
    m_renderFetchTarget = RenderFetchTarget::None;
    m_renderFetchSpriteSlot = 0;
    m_renderFetchStoreData = true;
    m_ppuAddressLatchLow = 0;
    m_renderAleThisClock = false;
    m_renderAleAddress = 0;
    m_renderAlePreviousLow = 0;
    m_renderFetchLowOverride = false;
    m_renderFetchUseLiveHigh = false;

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
    m_bgPatternLoadArmed = false;

    m_busLatch = 0;
    for (uint64_t& t : m_busBitRefreshClock) t = m_masterClock;
    m_nmiLine = false;
    m_nmiDelay = 0;
    m_suppressVBlank = false;
    m_renderingActiveLastClock = false;
    m_oamCorruptionPending = false;
    m_oamCorruptionSeed = 0;

    if (clearMemory) {
        m_masterClock = 0;
        std::memset(m_oamDecayCycles, 0, sizeof(m_oamDecayCycles));
        m_oamLfsr = 0x5A;
        std::memset(m_oam, 0, sizeof(m_oam));
        std::memset(m_nametable, 0, sizeof(m_nametable));
        std::memset(m_palette, 0, sizeof(m_palette));
        std::fill(m_framebuffer.begin(), m_framebuffer.end(), 0xFF000000);
    }

    m_registerWriteInhibitUntilClock = m_masterClock + registerWriteInhibitDuration();
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

    if (m_mask & 0x01)
        index &= 0x30;

    uint32_t c = kNesPalette[index & 0x3F];

    uint8_t emp = (m_mask >> 5) & 0x07;
    if (m_timing != ConsoleTiming::NTSC) {

        emp = static_cast<uint8_t>((emp & 0x04) | ((emp & 0x01) << 1) | ((emp & 0x02) >> 1));
    }
    if (emp != 0) {
        uint8_t r = (c >> 16) & 0xFF;
        uint8_t g = (c >> 8) & 0xFF;
        uint8_t b = c & 0xFF;

        if (emp & 0x01) { g = (uint8_t)(g * 0.75f); b = (uint8_t)(b * 0.75f); }
        if (emp & 0x02) { r = (uint8_t)(r * 0.75f); b = (uint8_t)(b * 0.75f); }
        if (emp & 0x04) { r = (uint8_t)(r * 0.75f); g = (uint8_t)(g * 0.75f); }
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

        return addr & 0x07FF;
    case M::Horizontal:

        if (addr >= 0x0400 && addr < 0x0800) return addr - 0x0400;
        if (addr >= 0x0800 && addr < 0x0C00) return addr - 0x0400;
        if (addr >= 0x0C00) return addr - 0x0800;
        return addr;
    case M::OnescreenLo:
        return addr & 0x03FF;
    case M::OnescreenHi:
        return 0x0400 + (addr & 0x03FF);
    case M::FourScreen:

        return addr & 0x0FFF;
    }
    return addr & 0x07FF;
}

uint8_t PPU::ppuRead(uint16_t addr, PpuFetchKind kind, bool driveAddress)
{
    addr &= 0x3FFF;
    const char* source = kind == PpuFetchKind::Background ? "BG" :
                         kind == PpuFetchKind::Sprite ? "SPR" : "CPU";
    m_ppuBusReadSource = source;
    if (driveAddress)
        notifyPpuAddress(addr);

    uint8_t value = 0;
    if (addr <= 0x1FFF) {
        uint32_t ciram = 0;
        if (m_cart && m_cart->mapPatternCiram(addr, ciram))
            value = m_nametable[ciram & 0x0FFF];
        else if (m_cart)
            value = m_cart->ppuRead(addr, kind);
    }
    else if (addr >= 0x2000 && addr <= 0x3EFF) {
        NametableSource source = NametableSource::Ciram;
        uint32_t mapped = 0;
        if (m_cart && m_cart->mapNametable(addr, source, mapped)) {
            if (source == NametableSource::Ciram)
                value = m_nametable[mapped & 0x0FFF];
            else
                value = m_cart->readNametableBacking(source, mapped);
        }
        else {
            value = m_nametable[mirrorNametable(addr)];
        }
    }
    else if (addr >= 0x3F00 && addr <= 0x3FFF) {
        uint16_t pal = static_cast<uint16_t>(addr & 0x1F);
        if ((pal & 0x13) == 0x10) pal &= static_cast<uint16_t>(~0x10);
        value = m_palette[pal];
    }

    m_ppuBusReadThisClock = true;
    m_ppuBusReadData = value;
    m_ppuBusReadAddress = addr;
    tracePpuBus("READ", source, addr, value);
    return value;
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
        if (m_cart && m_cart->mapNametableWrite(addr, source, mapped)) {
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
    if (m_suppressScrollXThisClock) {
        m_suppressScrollXThisClock = false;
        return;
    }
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
    if (m_suppressScrollYThisClock) {
        m_suppressScrollYThisClock = false;
        return;
    }
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

void PPU::incrementVramAddressAfterCpuAccess()
{

    if (renderingEnabled() && m_scanline >= -1 && m_scanline < 240) {
        incrementScrollX();
        incrementScrollY();
    }
    else {
        m_v = static_cast<uint16_t>(m_v + ((m_ctrl & 0x04) ? 32 : 1));
    }
}

void PPU::transferAddressX()
{
    if (!renderingEnabled()) return;
    if (m_suppressScrollXThisClock) {
        m_suppressScrollXThisClock = false;
        return;
    }
    m_v = (m_v & 0xFBE0) | (m_t & 0x041F);
}

void PPU::transferAddressY()
{
    if (!renderingEnabled()) return;
    if (m_suppressScrollYThisClock) {
        m_suppressScrollYThisClock = false;
        return;
    }
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

    if (renderingEnabled()) {

        m_bgShifterPatternLo = static_cast<uint16_t>(m_bgShifterPatternLo << 1);
        m_bgShifterPatternHi = static_cast<uint16_t>((m_bgShifterPatternHi << 1) | 0x0001u);
        m_bgShifterAttrLo <<= 1;
        m_bgShifterAttrHi <<= 1;
    }
}

void PPU::touchOamRow(uint8_t row)
{
    if (row < 32) {
        m_oamDecayCycles[row] = static_cast<uint32_t>(m_masterClock);
    }
}

uint8_t PPU::oamRandomByte()
{

    uint8_t bit = ((m_oamLfsr >> 0) ^ (m_oamLfsr >> 2) ^ (m_oamLfsr >> 3) ^ (m_oamLfsr >> 4)) & 1;
    m_oamLfsr = static_cast<uint8_t>((m_oamLfsr >> 1) | (bit << 7));
    return m_oamLfsr;
}

void PPU::corruptOamRow(uint8_t row)
{
    if (row >= 32) return;

    int base = row * 8;

    for (int i = 0; i < 8; i++) {
        uint8_t v = m_oam[base + i];
        uint8_t noise = oamRandomByte();

        m_oam[base + i] = (v & noise) | (~noise & (v ^ noise));
    }

    uint8_t v2 = m_oamSecondary[row];
    uint8_t noise2 = oamRandomByte();
    m_oamSecondary[row] = (v2 & noise2) | (~noise2 & (v2 ^ noise2));
}

void PPU::updateOamDecay()
{

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

namespace {
void tracePpuFlagTiming(const char* event, int scanline, int cycle, uint8_t status)
{
    (void)event; (void)scanline; (void)cycle; (void)status;
}
}

void PPU::resetSpriteOverflowEvaluation()
{

    m_spriteEvalStartAddr = m_oamAddr;
    m_spriteEvalN = static_cast<uint8_t>(m_oamAddr >> 2);
    m_spriteEvalM = static_cast<uint8_t>(m_oamAddr & 0x03);
    m_spriteEvalFound = 0;
    m_spriteEvalData = 0xFF;
    m_spriteEvalCopyRemaining = 0;
    m_spriteEvalWrapBusData = 0xFF;
    m_spriteEvalOverflowIncRemaining = 0;
    m_spriteEvalOverflowHandoff = false;
    m_spriteEvalFull = false;
    m_spriteEvalOverflowDiagonal = false;
    m_spriteEvalWrapped = false;
    m_spriteZeroNext = false;

    std::memset(m_oamSecondary, 0xFF, sizeof(m_oamSecondary));
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

    if (!renderingEnabled())
        return;

    if (m_cycle == 65)
        resetSpriteOverflowEvaluation();

    if (m_cycle & 1) {

        if (m_spriteEvalOverflowHandoff)
            return;

        const uint16_t index = static_cast<uint16_t>(m_spriteEvalN) * 4u + m_spriteEvalM;
        m_spriteEvalData = m_oam[index & 0xFF];
        touchOamRow(static_cast<uint8_t>((index & 0xFF) >> 3));
        return;
    }

    if (m_spriteEvalOverflowHandoff) {

        m_spriteEvalOverflowHandoff = false;
        return;
    }

    if (m_spriteEvalWrapped) {

        m_spriteEvalN = static_cast<uint8_t>((m_spriteEvalN + 1u) & 0x3Fu);
        m_spriteEvalM = 0;
        return;
    }

    if (m_cycle == 66)
        m_spriteZeroNext = spriteEvalValueInRange(m_spriteEvalData);

    if (m_spriteEvalFound < 8) {
        uint16_t addr = static_cast<uint16_t>(m_spriteEvalN) * 4u + m_spriteEvalM;

        if (m_spriteEvalCopyRemaining != 0) {

            const uint8_t dst = static_cast<uint8_t>((m_spriteEvalFound - 1u) * 4u +
                                                     (4u - m_spriteEvalCopyRemaining));
            m_oamSecondary[dst] = m_spriteEvalData;
            ++addr;
            --m_spriteEvalCopyRemaining;
        }
        else if (spriteEvalValueInRange(m_spriteEvalData)) {

            const uint8_t dst = static_cast<uint8_t>(m_spriteEvalFound * 4u);
            m_oamSecondary[dst] = m_spriteEvalData;
            ++m_spriteEvalFound;
            ++addr;
            m_spriteEvalCopyRemaining = 3;
        }
        else {

            addr = static_cast<uint16_t>((addr + 4u) & ~3u);
        }

        const bool wrapped = addr >= 0x100u;
        addr &= 0x00FFu;
        m_spriteEvalN = static_cast<uint8_t>(addr >> 2);
        m_spriteEvalM = static_cast<uint8_t>(addr & 0x03);
        if (wrapped) {
            m_spriteEvalWrapped = true;
            m_spriteEvalM = 0;

            m_spriteEvalWrapBusData = m_spriteEvalData;
        }

        if (m_spriteEvalFound == 8 && m_spriteEvalCopyRemaining == 0) {
            m_spriteEvalFull = true;
            m_spriteEvalOverflowDiagonal = true;
        }
        return;
    }

    if (!m_spriteEvalFull) {

        if (m_spriteEvalCopyRemaining) {
            const uint8_t dst = static_cast<uint8_t>((m_spriteEvalFound - 1u) * 4u +
                                                     (4u - m_spriteEvalCopyRemaining));
            m_oamSecondary[dst] = m_spriteEvalData;
        }
        uint16_t addr = static_cast<uint16_t>(m_spriteEvalN) * 4u + m_spriteEvalM + 1u;
        if (m_spriteEvalCopyRemaining)
            --m_spriteEvalCopyRemaining;
        addr &= 0x00FFu;
        m_spriteEvalN = static_cast<uint8_t>(addr >> 2);
        m_spriteEvalM = static_cast<uint8_t>(addr & 0x03);
        if (!m_spriteEvalCopyRemaining) {
            m_spriteEvalFull = true;
            m_spriteEvalOverflowDiagonal = true;
        }
        return;
    }

    uint16_t addr = static_cast<uint16_t>(m_spriteEvalN) * 4u + m_spriteEvalM;

    if (m_spriteEvalOverflowIncRemaining != 0) {
        if (m_spriteEvalOverflowIncRemaining > 1) {
            addr = static_cast<uint16_t>((addr + 1u) & 0x00FFu);
            --m_spriteEvalOverflowIncRemaining;
        } else {
            addr = static_cast<uint16_t>((addr + 4u) & ~3u);
            addr &= 0x00FFu;
            m_spriteEvalOverflowIncRemaining = 0;
            m_spriteEvalOverflowHandoff = true;
            m_spriteEvalOverflowDiagonal = false;
        }
    } else if (m_spriteEvalOverflowDiagonal) {
        if (spriteEvalValueInRange(m_spriteEvalData)) {
            const bool wasClear = (m_status & 0x20) == 0;
            m_status |= 0x20;
            if (wasClear)
                tracePpuFlagTiming("OVERFLOW_SET", m_scanline, m_cycle, m_status);
            addr = static_cast<uint16_t>((addr + 1u) & 0x00FFu);
            m_spriteEvalOverflowIncRemaining = 3;
        } else {

            addr = static_cast<uint16_t>(((addr + 4u) & 0x00FCu) | ((addr + 1u) & 0x0003u));
        }
    } else {

        addr = static_cast<uint16_t>((addr + 4u) & 0x00FCu);
    }

    const uint8_t oldAddr = static_cast<uint8_t>((static_cast<uint16_t>(m_spriteEvalN) * 4u + m_spriteEvalM) & 0xFFu);
    const uint8_t newAddr = static_cast<uint8_t>(addr);
    m_spriteEvalN = static_cast<uint8_t>(newAddr >> 2);
    m_spriteEvalM = static_cast<uint8_t>(newAddr & 0x03);
    if (newAddr < oldAddr) {
        m_spriteEvalWrapped = true;
        m_spriteEvalM = 0;
    }
}

void PPU::rebuildSpriteOverflowEvaluation()
{

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

    std::memcpy(m_oamDecayCycles, savedDecayCycles, sizeof(savedDecayCycles));
}

uint8_t PPU::reverseBits(uint8_t value)
{
    value = static_cast<uint8_t>(((value & 0xF0) >> 4) | ((value & 0x0F) << 4));
    value = static_cast<uint8_t>(((value & 0xCC) >> 2) | ((value & 0x33) << 2));
    value = static_cast<uint8_t>(((value & 0xAA) >> 1) | ((value & 0x55) << 1));
    return value;
}

uint16_t PPU::spritePatternAddress(uint8_t slot) const
{
    if (slot >= m_spriteCount) {

        if (m_ctrl & 0x20)
            return static_cast<uint16_t>(0x1000 + 0x0FE0);
        const uint16_t base = (m_ctrl & 0x08) ? 0x1000 : 0x0000;
        return static_cast<uint16_t>(base + 0x0FF0);
    }

    const uint8_t tileY = m_oamSecondary[slot * 4 + 0];
    uint8_t tileId = m_oamSecondary[slot * 4 + 1];
    const uint8_t attr = m_oamSecondary[slot * 4 + 2];

    const int fetchScanline = (m_scanline < 0)
        ? ((consoleScanlines(m_timing) - 1) & 0xFF)
        : m_scanline;
    int row = static_cast<uint8_t>(static_cast<uint8_t>(fetchScanline) - tileY);

    if (m_ctrl & 0x20) {
        if (attr & 0x80) row = 15 - row;
        const uint8_t half = tileId & 0x01;
        tileId &= 0xFE;
        if (row >= 8) {
            tileId |= 1;
            row -= 8;
        }
        return static_cast<uint16_t>(static_cast<uint16_t>(half) * 0x1000 +
                                     static_cast<uint16_t>(tileId) * 16 +
                                     static_cast<uint16_t>(row));
    }

    if (attr & 0x80) row = 7 - row;
    const uint16_t base = (m_ctrl & 0x08) ? 0x1000 : 0x0000;
    return static_cast<uint16_t>(base + static_cast<uint16_t>(tileId) * 16 +
                                 static_cast<uint16_t>(row));
}

void PPU::clockSpriteFetches()
{
    if (!renderingEnabled() || m_cycle < 257 || m_cycle > 320)
        return;

    const unsigned fetch = static_cast<unsigned>(m_cycle - 257);
    const uint8_t slot = static_cast<uint8_t>(fetch >> 3);
    const uint8_t phase = static_cast<uint8_t>(fetch & 7u);
    const bool valid = slot < m_spriteCount;

    bool patternValid = valid;
    if (valid) {
        const uint8_t y = m_oamSecondary[slot * 4];

        const int fetchScanline = (m_scanline < 0)
            ? ((consoleScanlines(m_timing) - 1) & 0xFF)
            : m_scanline;
        const uint8_t row = static_cast<uint8_t>(static_cast<uint8_t>(fetchScanline) - y);
        const int height = (m_ctrl & 0x20) ? 16 : 8;
        patternValid = row < height;
    }

    if (phase == 0) {
        beginRenderFetch(static_cast<uint16_t>(0x2000 | (m_v & 0x0FFF)),
                         PpuFetchKind::Background, RenderFetchTarget::SpriteGarbage, slot);
    }
    else if (phase == 1) {
        completeRenderFetch();
    }
    else if (phase == 2) {
        beginRenderFetch(static_cast<uint16_t>(0x2000 | (m_v & 0x0FFF)),
                         PpuFetchKind::Background, RenderFetchTarget::SpriteGarbage, slot);
        m_spriteAttr[slot] = valid ? m_oamSecondary[slot * 4 + 2] : 0;
        if (valid)
            touchOamRow(slot);
    }
    else if (phase == 3) {
        completeRenderFetch();
        m_spriteX[slot] = valid ? m_oamSecondary[slot * 4 + 3] : 0xFF;
    }

    const uint16_t addr = spritePatternAddress(slot);
    if (phase == 4) {
        beginRenderFetch(addr, PpuFetchKind::Sprite, RenderFetchTarget::SpriteLow,
                         slot, patternValid);
    }
    else if (phase == 5) {
        completeRenderFetch();
    }
    else if (phase == 6) {
        beginRenderFetch(static_cast<uint16_t>(addr + 8), PpuFetchKind::Sprite,
                         RenderFetchTarget::SpriteHigh, slot, patternValid);
    }
    else if (phase == 7) {
        completeRenderFetch();
    }
}

bool PPU::spriteZeroHitEligible(int cycle, bool spriteZeroActive, uint8_t fgPixel, uint8_t bgPixel)
{

    return spriteZeroActive && fgPixel != 0 && bgPixel != 0 && cycle >= 1 && cycle != 256;
}

void PPU::getSpritePixel(uint8_t& pixel, uint8_t& palette, uint8_t& priority)
{
    pixel = 0;
    palette = 0;
    priority = 0;
    m_spriteZeroBeingRendered = false;

    if (!spriteRenderingEnabled())
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
                palette = (m_spriteAttr[i] & 0x03) + 4;
                priority = (m_spriteAttr[i] & 0x20) ? 1 : 0;
                break;
            }
        }
    }
}

void PPU::beginRenderFetch(uint16_t addr, PpuFetchKind kind, RenderFetchTarget target, uint8_t spriteSlot, bool storeData)
{

    m_renderFetchPending = true;
    m_renderFetchAddress = static_cast<uint16_t>(addr & 0x3FFF);
    m_renderFetchKind = kind;
    m_renderFetchTarget = target;
    m_renderFetchSpriteSlot = spriteSlot;
    m_renderFetchStoreData = storeData;
    m_renderFetchLowOverride = false;
    m_renderFetchUseLiveHigh = false;
    m_renderAleThisClock = true;
    m_renderAleAddress = m_renderFetchAddress;
    m_renderAlePreviousLow = m_ppuAddressLatchLow;
    m_ppuAddressLatchLow = static_cast<uint8_t>(m_renderFetchAddress);
    notifyPpuAddress(m_renderFetchAddress);
}

void PPU::completeRenderFetch()
{
    if (!m_renderFetchPending)
        return;

    uint16_t readAddress = m_renderFetchAddress;
    if (m_renderFetchUseLiveHigh) {
        uint16_t live = m_renderFetchAddress;
        if (m_renderFetchTarget == RenderFetchTarget::BgNt ||
            m_renderFetchTarget == RenderFetchTarget::SpriteGarbage ||
            m_renderFetchTarget == RenderFetchTarget::DummyNt337) {
            live = static_cast<uint16_t>(0x2000 | (m_v & 0x0FFF));
        } else if (m_renderFetchTarget == RenderFetchTarget::BgAttr ||
                   m_renderFetchTarget == RenderFetchTarget::DummyNt339) {
            live = static_cast<uint16_t>(0x23C0 | (m_v & 0x0C00) |
                   ((m_v >> 4) & 0x38) | ((m_v >> 2) & 0x07));
        }
        readAddress = static_cast<uint16_t>((live & 0x3F00) | m_ppuAddressLatchLow);
    } else if (m_renderFetchLowOverride) {
        readAddress = static_cast<uint16_t>((m_renderFetchAddress & 0x3F00) | m_ppuAddressLatchLow);
    }
    const uint8_t value = ppuRead(readAddress, m_renderFetchKind, false);
    const uint8_t slot = m_renderFetchSpriteSlot;
    switch (m_renderFetchTarget) {
    case RenderFetchTarget::BgNt:
        m_bgNextTileId = m_renderFetchStoreData ? value : 0;
        break;
    case RenderFetchTarget::BgAttr: {
        uint8_t attr = m_renderFetchStoreData ? value : 0;
        if (m_v & 0x40) attr >>= 4;
        if (m_v & 0x02) attr >>= 2;
        m_bgNextTileAttr = static_cast<uint8_t>(attr & 0x03);
        break;
    }
    case RenderFetchTarget::BgLow:
        m_bgNextTileLsb = m_renderFetchStoreData ? value : 0;
        break;
    case RenderFetchTarget::BgHigh:
        m_bgNextTileMsb = m_renderFetchStoreData ? value : 0;
        m_bgPatternLoadArmed = true;
        break;
    case RenderFetchTarget::SpriteLow: {
        uint8_t v = value;
        if (slot < 8 && (m_spriteAttr[slot] & 0x40))
            v = reverseBits(v);
        if (slot < 8)
            m_spriteShifterLo[slot] = m_renderFetchStoreData ? v : 0;
        break;
    }
    case RenderFetchTarget::SpriteHigh: {
        uint8_t v = value;
        if (slot < 8 && (m_spriteAttr[slot] & 0x40))
            v = reverseBits(v);
        if (slot < 8)
            m_spriteShifterHi[slot] = m_renderFetchStoreData ? v : 0;
        break;
    }
    case RenderFetchTarget::SpriteGarbage:
    case RenderFetchTarget::DummyNt337:
    case RenderFetchTarget::DummyNt339:
    case RenderFetchTarget::None:
        break;
    }

    m_renderFetchPending = false;
    m_renderFetchTarget = RenderFetchTarget::None;
    m_renderFetchLowOverride = false;
    m_renderFetchUseLiveHigh = false;
}

void PPU::clockRenderMaskDelay()
{
    if (m_renderMaskDelay == 0)
        return;
    if (--m_renderMaskDelay == 0)
        m_renderMask = static_cast<uint8_t>(m_pendingRenderMask & 0x18);
}

void PPU::clockPpudataIncrementDelay()
{
    if (!m_ppudataIncrementPending || m_ppudataIncrementDelay == 0)
        return;
    if (--m_ppudataIncrementDelay != 0)
        return;

    const uint16_t oldV = m_v;
    uint16_t incremented = oldV;
    if (renderingEnabled() && m_scanline >= -1 && m_scanline < 240) {

        if ((incremented & 0x001F) == 31) {
            incremented &= static_cast<uint16_t>(~0x001F);
            incremented ^= 0x0400;
        } else {
            ++incremented;
        }
        if ((incremented & 0x7000) != 0x7000) {
            incremented = static_cast<uint16_t>(incremented + 0x1000);
        } else {
            incremented &= static_cast<uint16_t>(~0x7000);
            uint16_t y = static_cast<uint16_t>((incremented & 0x03E0) >> 5);
            if (y == 29) { y = 0; incremented ^= 0x0800; }
            else if (y == 31) y = 0;
            else ++y;
            incremented = static_cast<uint16_t>((incremented & ~0x03E0) | (y << 5));
        }

        if (m_cycle == 257) {
            constexpr uint16_t xMask = 0x041F;
            const uint16_t xConflict = static_cast<uint16_t>((incremented & m_t) & xMask);
            incremented = static_cast<uint16_t>((incremented & ~xMask) | xConflict);
            m_t = static_cast<uint16_t>((m_t & ~xMask) | xConflict);
            m_suppressScrollXThisClock = true;
        }
        if (m_scanline == -1 && m_cycle >= 280 && m_cycle <= 304) {
            constexpr uint16_t yMask = 0x7BE0;
            const uint16_t yConflict = static_cast<uint16_t>((incremented & m_t) & yMask);
            incremented = static_cast<uint16_t>((incremented & ~yMask) | yConflict);
            m_t = static_cast<uint16_t>((m_t & ~yMask) | yConflict);
            m_suppressScrollYThisClock = true;
        }
        m_v = incremented;
    } else {
        m_v = static_cast<uint16_t>(m_v + ((m_ctrl & 0x04) ? 32 : 1));
    }

    m_ppudataIncrementPending = false;
    notifyPpuAddress(m_v);
}

void PPU::clockPpudataReadBufferDelay()
{
    if (!m_ppudataReadPending || m_ppudataReadDelay == 0)
        return;

    const uint8_t phase = m_ppudataReadDelay;
    --m_ppudataReadDelay;

    if (phase == 3) {
        if (!m_renderAleThisClock) {
            m_ppuAddressLatchLow = static_cast<uint8_t>(m_ppudataReadAddress);
            notifyPpuAddress(m_ppudataReadAddress);
        }
        return;
    }

    if (phase != 1)
        return;

    if (m_ppuBusReadThisClock) {

        m_dataBuffer = m_ppuBusReadData;
        tracePpuBus("BUF_REFILL_SHARED", m_ppuBusReadSource, m_ppuBusReadAddress, m_dataBuffer);
    } else if (m_renderAleThisClock) {

        const uint16_t addr = static_cast<uint16_t>((m_renderAleAddress & 0x3F00) | m_renderAlePreviousLow);
        m_dataBuffer = ppuRead(addr, PpuFetchKind::Cpu, false);
        m_ppuAddressLatchLow = m_dataBuffer;
        m_renderFetchLowOverride = true;
        tracePpuBus("BUF_REFILL_ALE_RD", "CPU", addr, m_dataBuffer);
    } else {

        const uint16_t addr = static_cast<uint16_t>((m_ppudataReadAddress & 0x3F00) | m_ppuAddressLatchLow);
        m_dataBuffer = ppuRead(addr, PpuFetchKind::Cpu, false);
        tracePpuBus("BUF_REFILL_OWN", "CPU", addr, m_dataBuffer);
    }

    m_ppudataReadPending = false;
}

void PPU::clockVramAddressDelay()
{
    if (m_vramAddressDelay == 0)
        return;
    if (--m_vramAddressDelay == 0) {
        const uint16_t oldV = m_v;
        uint16_t loaded = m_pendingVramAddress;

        const bool activeRenderLine = renderingEnabled() && m_scanline >= -1 && m_scanline < 240;
        const bool xIncrementDot = activeRenderLine &&
            (((m_cycle >= 2 && m_cycle < 258) || (m_cycle >= 321 && m_cycle < 338)) &&
             (((m_cycle - 1) & 7) == 7));
        const bool yIncrementDot = activeRenderLine && m_cycle == 256;

        if (xIncrementDot) {
            uint16_t xInc = oldV;
            if ((xInc & 0x001F) == 31) {
                xInc &= static_cast<uint16_t>(~0x001F);
                xInc ^= 0x0400;
            } else {
                ++xInc;
            }
            constexpr uint16_t xMask = 0x041F;
            const uint16_t xConflict = static_cast<uint16_t>((xInc & loaded) & xMask);
            loaded = static_cast<uint16_t>((loaded & ~xMask) | xConflict);
            m_t = static_cast<uint16_t>((m_t & ~xMask) | xConflict);
            m_suppressScrollXThisClock = true;
        }

        if (yIncrementDot) {
            uint16_t yInc = oldV;
            if ((yInc & 0x7000) != 0x7000) {
                yInc = static_cast<uint16_t>(yInc + 0x1000);
            } else {
                yInc &= static_cast<uint16_t>(~0x7000);
                uint16_t y = static_cast<uint16_t>((yInc & 0x03E0) >> 5);
                if (y == 29) {
                    y = 0;
                    yInc ^= 0x0800;
                } else if (y == 31) {
                    y = 0;
                } else {
                    ++y;
                }
                yInc = static_cast<uint16_t>((yInc & ~0x03E0) | (y << 5));
            }
            constexpr uint16_t yMask = 0x7BE0;
            const uint16_t yConflict = static_cast<uint16_t>((yInc & loaded) & yMask);
            loaded = static_cast<uint16_t>((loaded & ~yMask) | yConflict);
            m_t = static_cast<uint16_t>((m_t & ~yMask) | yConflict);
            m_suppressScrollYThisClock = true;
        }

        if (m_renderFetchPending) {
            m_renderFetchUseLiveHigh = true;
        }
        m_v = loaded;

        notifyPpuAddress(m_v);
    }
}

void PPU::clock()
{

    m_renderAleThisClock = false;
    m_renderAleAddress = 0;
    m_renderAlePreviousLow = m_ppuAddressLatchLow;
    m_ppuBusReadHeldThisClock = false;
    m_ppuBusReadHeldData = 0;
    m_ppuBusReadHeldAddress = 0;
    m_ppuBusReadHeldSource = "none";
    m_ppuBusReadThisClock = false;
    m_ppuBusReadData = 0;
    m_ppuBusReadAddress = 0;
    m_ppuBusReadSource = "none";
    if (m_ppuBusTraceDots != 0)
        tracePpuBus("DOT", "-", static_cast<uint16_t>(m_v & 0x3FFF), m_dataBuffer);

    clockRenderMaskDelay();
    m_suppressScrollXThisClock = false;
    m_suppressScrollYThisClock = false;
    clockPpudataIncrementDelay();

    clockVramAddressDelay();

    const bool renderingActiveNow = renderingEnabled() && m_scanline >= -1 && m_scanline < 240;
    if (!renderingActiveNow && m_renderingActiveLastClock &&
        m_timing != ConsoleTiming::PAL && m_scanline >= -1 && m_scanline < 240) {
        m_oamCorruptionSeed = currentSecondaryOamAddressForCorruption();
        m_oamCorruptionPending = true;
    }
    if (renderingActiveNow && !m_renderingActiveLastClock &&
        m_oamCorruptionPending && m_timing != ConsoleTiming::PAL) {
        const uint8_t row = static_cast<uint8_t>(m_oamCorruptionSeed & 0x1F);
        const uint16_t dst = static_cast<uint16_t>(row) * 8u;
        uint8_t source[8];
        std::memcpy(source, &m_oam[0], sizeof(source));
        std::memcpy(&m_oam[dst], source, sizeof(source));
        m_oamSecondary[row] = m_oamSecondary[0];
        touchOamRow(row);
        m_oamCorruptionPending = false;
    }
    m_renderingActiveLastClock = renderingActiveNow;

    clockNmiDelay();

    if (m_cycle == 0 && m_cart)
        m_cart->notifyPpuScanline(m_scanline, renderingEnabled());

    m_masterClock++;

    if (m_scanline >= -1 && m_scanline < 240) {

        if (m_scanline == -1 && m_cycle == 1) {
            clearVBlank(false);
            m_status &= ~0x40;
            m_status &= ~0x20;
        }

        if (!renderingEnabled() &&
            ((m_cycle >= 1 && m_cycle <= 255) || (m_cycle >= 321 && m_cycle <= 336)) &&
            (m_cycle & 7) == 7) {
            m_bgPatternLoadArmed = false;
        }

        const bool bgFetchWindow = renderingEnabled() &&
            ((m_cycle >= 1 && m_cycle <= 256) || (m_cycle >= 321 && m_cycle <= 336));
        if (renderingEnabled() &&
            ((m_cycle >= 2 && m_cycle < 258) || (m_cycle >= 321 && m_cycle < 338))) {
            updateBackgroundShifters();
        }

        if (bgFetchWindow) {
            switch ((m_cycle - 1) & 7) {
            case 0:
                if (m_bgPatternLoadArmed)
                    loadBackgroundShifters();
                m_bgPatternLoadArmed = false;
                beginRenderFetch(static_cast<uint16_t>(0x2000 | (m_v & 0x0FFF)),
                                 PpuFetchKind::Background, RenderFetchTarget::BgNt);
                break;
            case 1:
                completeRenderFetch();
                break;
            case 2:
                beginRenderFetch(static_cast<uint16_t>(0x23C0 | (m_v & 0x0C00) |
                                 ((m_v >> 4) & 0x38) | ((m_v >> 2) & 0x07)),
                                 PpuFetchKind::Background, RenderFetchTarget::BgAttr);
                break;
            case 3:
                completeRenderFetch();
                break;
            case 4: {
                const uint16_t base = (m_ctrl & 0x10) ? 0x1000 : 0x0000;
                beginRenderFetch(static_cast<uint16_t>(base + (static_cast<uint16_t>(m_bgNextTileId) << 4) +
                                 ((m_v >> 12) & 0x7)),
                                 PpuFetchKind::Background, RenderFetchTarget::BgLow);
                break;
            }
            case 5:
                completeRenderFetch();
                break;
            case 6: {
                const uint16_t base = (m_ctrl & 0x10) ? 0x1000 : 0x0000;
                beginRenderFetch(static_cast<uint16_t>(base + (static_cast<uint16_t>(m_bgNextTileId) << 4) +
                                 ((m_v >> 12) & 0x7) + 8),
                                 PpuFetchKind::Background, RenderFetchTarget::BgHigh);
                break;
            }
            case 7:
                completeRenderFetch();
                incrementScrollX();
                break;
            }
        }

        if (m_cycle >= 2 && m_cycle <= 256) {
            for (int i = 0; i < 8; i++) {

                if (m_spriteX[i] > 0) {
                    m_spriteX[i]--;
                }
                else if (renderingEnabled()) {
                    m_spriteShifterLo[i] <<= 1;
                    m_spriteShifterHi[i] <<= 1;
                }
            }
        }

        if (m_cycle == 339 && !renderingEnabled()) {
            for (int i = 0; i < 8; ++i)
                m_spriteX[i] = 0;
        }

        if (renderingEnabled() && m_cycle >= 257 && m_cycle <= 320)
            m_oamAddr = 0;

        clockSpriteOverflowEvaluation();

        if (m_cycle == 256)
            incrementScrollY();

        if (m_cycle == 257) {

            clockSpriteFetches();
            transferAddressX();

            m_spriteZeroPossible = m_spriteZeroNext;

            m_spriteCount = m_spriteEvalFound > 8 ? 8 : m_spriteEvalFound;
        }

        if (m_cycle != 257)
            clockSpriteFetches();

        if (renderingEnabled()) {
            if (m_cycle == 337) {
                beginRenderFetch(static_cast<uint16_t>(0x2000 | (m_v & 0x0FFF)),
                                 PpuFetchKind::Background, RenderFetchTarget::DummyNt337);
            }
            else if (m_cycle == 338) {
                completeRenderFetch();
            }
            else if (m_cycle == 339) {
                beginRenderFetch(static_cast<uint16_t>(0x2000 | (m_v & 0x0FFF)),
                                 PpuFetchKind::Background, RenderFetchTarget::DummyNt339);
            }
            else if (m_cycle == 340) {
                completeRenderFetch();
            }
        }

        if (m_scanline == -1 && m_cycle >= 280 && m_cycle < 305)
            transferAddressY();

        if (m_scanline >= 0 && m_scanline < 240 && m_cycle >= 1 && m_cycle <= 256) {

            uint8_t bgPixel = 0, bgPalette = 0;
            if (backgroundRenderingEnabled()) {
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

            uint8_t fgPixel = 0, fgPalette = 0, fgPriority = 0;
            getSpritePixel(fgPixel, fgPalette, fgPriority);
            if (m_cycle <= 8 && !(m_mask & 0x04))
                fgPixel = 0;

            if (m_scanline <= 239 &&
                spriteZeroHitEligible(m_cycle, m_spriteZeroBeingRendered, fgPixel, bgPixel)) {
                const bool wasClear = (m_status & 0x40) == 0;
                m_status |= 0x40;
                if (wasClear)
                    tracePpuFlagTiming("SPRITE0_SET", m_scanline, m_cycle, m_status);
            }

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

                if (fgPriority == 0) {
                    pixel = fgPixel; palette = fgPalette;
                }
                else {
                    pixel = bgPixel; palette = bgPalette;
                }
            }

            uint16_t paletteAddr = static_cast<uint16_t>((palette << 2) | pixel) & 0x1F;
            if ((paletteAddr & 0x13) == 0x10) paletteAddr &= ~0x10;
            uint8_t colorIndex = m_palette[paletteAddr];
            m_framebuffer[m_scanline * 256 + (m_cycle - 1)] = nesColor(colorIndex);
        }
    }

    clockPpudataReadBufferDelay();
    if (m_ppuBusTraceDots != 0)
        --m_ppuBusTraceDots;

    if (m_scanline == consoleVblankStartScanline(m_timing) && m_cycle == 1)
        setVBlank();

    updateOamDecay();

    if (m_scanline == -1 && m_cycle == 339 && m_oddFrame && renderingEnabled() && consoleHasOddFrameSkip(m_timing)) {
        m_cycle = 0;
        m_scanline = 0;
    }
    else {
        m_cycle = m_cycle + 1;
        if (m_cycle > 340) {
            m_cycle = 0;
            m_scanline = m_scanline + 1;

            if (m_scanline > consoleScanlines(m_timing) - 2) {
                m_scanline = -1;
                m_frameComplete = true;
                m_oddFrame = !m_oddFrame;
            }
        }
    }
}

uint8_t PPU::readBusLatchWithDecay()
{

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

uint8_t PPU::currentSecondaryOamAddressForCorruption() const
{

    if (m_cycle >= 1 && m_cycle <= 64)
        return static_cast<uint8_t>((m_cycle - 1) >> 1);

    if (m_cycle >= 65 && m_cycle <= 256) {
        if (m_spriteEvalFound >= 8 && m_spriteEvalCopyRemaining == 0)
            return 0;
        unsigned next = static_cast<unsigned>(m_spriteEvalFound) * 4u;
        if (m_spriteEvalCopyRemaining <= next)
            next -= m_spriteEvalCopyRemaining;

        return static_cast<uint8_t>((next + 3u) & 0x1Cu);
    }

    if (m_cycle >= 257 && m_cycle <= 320) {
        const unsigned off = static_cast<unsigned>(m_cycle - 257);
        const unsigned slot = off >> 3;
        const unsigned phase = off & 7u;
        unsigned addr = slot * 4u + (phase < 3u ? phase : 3u);
        if (phase == 7u) ++addr;
        return static_cast<uint8_t>(addr & 0x1Fu);
    }

    return 0;
}

uint8_t PPU::oamDataReadValue() const
{

    const bool renderLine = m_scanline >= -1 && m_scanline < 240;
    if (!renderingEnabled() || !renderLine)
        return m_oam[m_oamAddr];

    if (m_cycle >= 1 && m_cycle <= 64)
        return 0xFF;

    if (m_cycle >= 65 && m_cycle <= 256) {

        if (m_spriteEvalOverflowHandoff)
            return m_oamSecondary[0];
        if (m_cycle & 1) {
            uint8_t n = m_spriteEvalN;
            uint8_t m = m_spriteEvalM;
            if (m_cycle == 65) {
                n = static_cast<uint8_t>(m_oamAddr >> 2);
                m = static_cast<uint8_t>(m_oamAddr & 0x03);
            }
            const uint16_t index = static_cast<uint16_t>(n) * 4u + m;
            return m_oam[index & 0xFF];
        }
        if (m_spriteEvalFull)
            return m_oamSecondary[0];
        if (m_spriteEvalWrapped)
            return m_spriteEvalWrapBusData;
        return m_spriteEvalData;
    }

    if (m_cycle >= 257 && m_cycle <= 320) {
        const unsigned fetch = static_cast<unsigned>(m_cycle - 257);
        const unsigned slot = fetch >> 3;
        const unsigned phase = fetch & 7u;
        const unsigned base = slot * 4u;
        const unsigned byte = phase < 4u ? phase : 3u;

        if (m_spriteEvalWrapped && m_spriteEvalFound < 8 &&
            slot == m_spriteEvalFound && phase == 0)
            return m_spriteEvalWrapBusData;
        return m_oamSecondary[base + byte];
    }

    return m_oamSecondary[0];
}

uint8_t PPU::cpuRead(uint16_t addr)
{
    const uint8_t reg = static_cast<uint8_t>(addr & 0x7);
    if (reg == 4 || reg == 6 || reg == 7)
        armPpuBusTrace("CPU_READ", addr);
    uint8_t data = readBusLatchWithDecay();

    switch (reg) {
    case 2: {

        uint8_t sampledStatus = static_cast<uint8_t>(m_status & 0xE0);

        if (m_scanline >= 0 && m_scanline < 240 &&
            m_cycle >= 65 && m_cycle <= 256 &&
            (m_cycle & 1) == 0 && m_spriteEvalFound >= 8 &&
            m_spriteEvalFull && m_spriteEvalN < 64 &&
            spriteEvalValueInRange(m_spriteEvalData)) {
            sampledStatus = static_cast<uint8_t>(sampledStatus | 0x20);
        }

        if (m_scanline == -1) {

            if (m_cycle == 340 || m_cycle == 0)
                sampledStatus = static_cast<uint8_t>((sampledStatus & 0x80));
            else if (m_cycle == 1)
                sampledStatus = 0;
        }

        data = static_cast<uint8_t>(sampledStatus | (data & 0x1F));

        if (m_scanline == consoleVblankStartScanline(m_timing)) {
            if (m_cycle == 0) {
                m_suppressVBlank = true;
            }
            else if (m_cycle == 1) {
                data = static_cast<uint8_t>(data | 0x80);
                m_suppressVBlank = true;
            }
        }
        tracePpuFlagTiming("STATUS_READ", m_scanline, m_cycle, data);
        clearVBlank();
        m_w = false;
        driveBusLatch(data, 0xE0);
        break;
    }
    case 4: {

        const bool internalBus = renderingEnabled() && m_scanline >= -1 && m_scanline < 240;
        if (!internalBus)
            touchOamRow(static_cast<uint8_t>(m_oamAddr >> 3));
        data = oamDataReadValue();
        if (!internalBus && (m_oamAddr & 0x03) == 0x02)
            data = static_cast<uint8_t>(data & 0xE3);
        driveBusLatch(data, 0xFF);
        break;
    }
    case 7: {
        const uint16_t vramAddr = m_v & 0x3FFF;
        if (vramAddr >= 0x3F00) {

            const uint8_t latch = readBusLatchWithDecay();
            uint8_t paletteData = static_cast<uint8_t>(ppuRead(vramAddr) & 0x3F);

            if (m_mask & 0x01)
                paletteData = static_cast<uint8_t>(paletteData & 0x30);
            data = static_cast<uint8_t>((latch & 0xC0) | paletteData);
            m_dataBuffer = ppuRead(static_cast<uint16_t>((vramAddr - 0x1000) & 0x3FFF));
            driveBusLatch(data, 0x3F);
        }
        else {
            data = m_dataBuffer;
            if (renderingEnabled() && m_scanline >= -1 && m_scanline < 240) {

                m_ppudataReadPending = true;
                m_ppudataReadDelay = 6;
                m_ppudataReadAddress = vramAddr;
            }
            else {
                m_dataBuffer = ppuRead(vramAddr);
            }
            driveBusLatch(data, 0xFF);
        }
        if (renderingEnabled() && m_scanline >= -1 && m_scanline < 240) {
            m_ppudataIncrementPending = true;

            m_ppudataIncrementDelay = 6;
        } else {
            incrementVramAddressAfterCpuAccess();
            notifyPpuAddress(m_v);
        }
        break;
    }
    default:

        break;
    }

    return data;
}

void PPU::cpuWrite(uint16_t addr, uint8_t data)
{
    const uint8_t reg = static_cast<uint8_t>(addr & 0x7);
    if (reg == 4 || reg == 6 || reg == 7)
        armPpuBusTrace("CPU_WRITE", addr, data);
    if (registerWriteInhibited() && (reg == 0 || reg == 1 || reg == 5 || reg == 6)) {

        driveBusLatch(data, 0xFF);
        return;
    }

    switch (reg) {
    case 0: {
        const uint8_t oldCtrl = m_ctrl;
        m_ctrl = data;
        m_t = (m_t & 0xF3FF) | ((uint16_t)(data & 0x03) << 10);

        const bool shrunkSprites = (oldCtrl & 0x20) != 0 && (data & 0x20) == 0;
        if (shrunkSprites && m_scanline >= 0 && m_scanline < 240 &&
            m_cycle >= 65 && m_cycle <= 256 &&
            m_spriteEvalFound != 0 && m_spriteEvalCopyRemaining == 3) {
            const uint8_t slot = static_cast<uint8_t>(m_spriteEvalFound - 1);
            const uint8_t y = m_oamSecondary[slot * 4];
            const int row = m_scanline - static_cast<int>(y);
            if (row < 0 || row >= 8) {
                std::memset(&m_oamSecondary[slot * 4], 0xFF, 4);
                --m_spriteEvalFound;
                m_spriteEvalCopyRemaining = 0;

                m_spriteEvalN = static_cast<uint8_t>(m_spriteEvalN + 1);
                m_spriteEvalM = 0;
                if (slot == 0)
                    m_spriteZeroNext = false;
            }
        }

        if (shrunkSprites && m_scanline >= -1 && m_scanline < 240 &&
            m_cycle >= 257 && m_cycle <= 260 && m_spriteCount != 0) {
            uint8_t writeSlot = 0;
            bool zeroSurvives = false;
            for (uint8_t readSlot = 0; readSlot < m_spriteCount; ++readSlot) {
                const uint8_t y = m_oamSecondary[readSlot * 4];
                const int row = static_cast<int>(static_cast<uint8_t>(m_scanline)) -
                                static_cast<int>(y);
                if (row < 0 || row >= 8)
                    continue;

                if (writeSlot != readSlot)
                    std::memcpy(&m_oamSecondary[writeSlot * 4],
                                &m_oamSecondary[readSlot * 4], 4);
                if (readSlot == 0 && m_spriteZeroPossible)
                    zeroSurvives = true;
                ++writeSlot;
            }
            for (uint8_t slot = writeSlot; slot < 8; ++slot)
                std::memset(&m_oamSecondary[slot * 4], 0xFF, 4);
            m_spriteCount = writeSlot;
            m_spriteZeroPossible = zeroSurvives;
            if (!zeroSurvives)
                m_spriteZeroBeingRendered = false;
        }

        updateNmiLine();
        break;
    }
    case 1: {
        const uint8_t oldRenderBits = static_cast<uint8_t>(m_mask & 0x18);
        m_mask = data;
        const uint8_t newRenderBits = static_cast<uint8_t>(data & 0x18);
        if (newRenderBits != oldRenderBits || (m_renderMaskDelay != 0 && newRenderBits != m_pendingRenderMask)) {
            m_pendingRenderMask = newRenderBits;
            m_renderMaskDelay = 3;
        }
        break;
    }
    case 3:
        m_oamAddr = data;
        break;
    case 4: {

        const bool activeRender = renderingEnabled() && m_scanline >= -1 && m_scanline < 240;
        if (activeRender) {
            m_oamAddr = static_cast<uint8_t>((m_oamAddr + 4u) & 0xFCu);
            break;
        }

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
            m_pendingVramAddress = m_t;
            m_vramAddressDelay = 5;
            m_w = false;
        }
        break;
    case 7:
        ppuWrite(m_v, data);
        incrementVramAddressAfterCpuAccess();

        notifyPpuAddress(m_v);
        break;
    }

    driveBusLatch(data, 0xFF);
}

void PPU::armPpuBusTrace(const char* event, uint16_t cpuAddr, uint8_t data)
{
    (void)event; (void)cpuAddr; (void)data;
    m_ppuBusTraceDots = 0;
}

void PPU::tracePpuBus(const char* event, const char* source, uint16_t addr, uint8_t data)
{
    (void)event; (void)source; (void)addr; (void)data;
}

void PPU::notifyPpuAddress(uint16_t addr)
{
    const uint16_t masked = static_cast<uint16_t>(addr & 0x3FFF);
#ifdef NES_HEADLESS
    m_testFetchTrace.push_back((static_cast<uint32_t>(m_cycle & 0xFFFF) << 16) | masked);
#endif
    if (m_cart)
        m_cart->notifyPpuAddress(masked, m_masterClock, m_scanline, m_cycle);
}

void PPU::updateNmiLine()
{
    const bool newLine = (m_status & 0x80) != 0 && (m_ctrl & 0x80) != 0;
    if (newLine && !m_nmiLine) {

        m_nmiDelay = 1;
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

void PPU::clearVBlank(bool allowPendingEdgeCancel)
{
    m_status &= ~0x80;

    if (!allowPendingEdgeCancel) {
        const bool oldLine = m_nmiLine;
        m_nmiLine = false;
        if (oldLine) {

            if (m_cpu)
                m_cpu->cancelPendingNmi();
            m_nmiDelay = 0;
        }
        return;
    }

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
    put8(m_ctrl); put8(m_mask); put8(m_renderMask); put8(m_pendingRenderMask); put8(m_renderMaskDelay); put8(m_status); put8(m_oamAddr);
    put16(m_pendingVramAddress); put8(m_vramAddressDelay); put8(m_ppudataIncrementDelay); put8(m_ppudataIncrementPending ? 1 : 0); put8(m_cpuPpuIoLatePhase ? 1 : 0);
    put16(m_v); put16(m_t); put8(m_x); put8(m_w ? 1 : 0);
    put8(m_dataBuffer);
    put8(m_ppudataReadPending ? 1 : 0); put8(m_ppudataReadDelay); put16(m_ppudataReadAddress);

    put8(m_renderFetchPending ? 1 : 0);
    put16(m_renderFetchAddress);
    put8(static_cast<uint8_t>(m_renderFetchKind));
    put8(static_cast<uint8_t>(m_renderFetchTarget));
    put8(m_renderFetchSpriteSlot);
    put8(m_renderFetchStoreData ? 1 : 0);
    put8(m_ppuAddressLatchLow);
    put8(m_renderAleThisClock ? 1 : 0);
    put16(m_renderAleAddress);
    put8(m_renderAlePreviousLow);
    put8(m_renderFetchLowOverride ? 1 : 0);
    put8(m_renderFetchUseLiveHigh ? 1 : 0);
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
    put8(m_bgPatternLoadArmed ? 1 : 0);
    put8(m_busLatch);
    for (uint64_t t : m_busBitRefreshClock) {
        put32(static_cast<uint32_t>(t & 0xFFFFFFFFu));
        put32(static_cast<uint32_t>((t >> 32) & 0xFFFFFFFFu));
    }
    put8(m_nmiLine ? 1 : 0);
    put8(m_nmiDelay);
    put8(m_suppressVBlank ? 1 : 0);
    put8(m_renderingActiveLastClock ? 1 : 0);
    put8(m_oamCorruptionPending ? 1 : 0);
    put8(m_oamCorruptionSeed);
    put32(static_cast<uint32_t>(m_registerWriteInhibitUntilClock & 0xFFFFFFFFu));
    put32(static_cast<uint32_t>((m_registerWriteInhibitUntilClock >> 32) & 0xFFFFFFFFu));

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
        if (!need(1)) return false;
        v = *p++;
        return true;
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
    if (!get8(m_ctrl) || !get8(m_mask) || !get8(m_renderMask) || !get8(m_pendingRenderMask) || !get8(m_renderMaskDelay) || !get8(m_status) || !get8(m_oamAddr)) return false;
    if ((m_renderMask & ~0x18) != 0 || (m_pendingRenderMask & ~0x18) != 0 || m_renderMaskDelay > 3) return false;
    if (!get16(m_pendingVramAddress) || !get8(m_vramAddressDelay) || !get8(m_ppudataIncrementDelay)) return false;
    uint8_t ppudataPending = 0; if (!get8(ppudataPending)) return false; m_ppudataIncrementPending = ppudataPending != 0;
    uint8_t ioLatePhase = 0; if (!get8(ioLatePhase)) return false; m_cpuPpuIoLatePhase = ioLatePhase != 0;
    if (m_vramAddressDelay > 5 || m_ppudataIncrementDelay > 6 || (m_pendingVramAddress & 0x8000) != 0 || ppudataPending > 1 || ioLatePhase > 1) return false;
    if (!get16(m_v) || !get16(m_t) || !get8(m_x) || !get8(w)) return false;
    m_w = w != 0;
    if (!get8(m_dataBuffer)) return false;
    uint8_t ppudataReadPending = 0;
    if (!get8(ppudataReadPending) || !get8(m_ppudataReadDelay) || !get16(m_ppudataReadAddress)) return false;
    m_ppudataReadPending = ppudataReadPending != 0;
    if (ppudataReadPending > 1 || m_ppudataReadDelay > 6 || (m_ppudataReadAddress & 0xC000) != 0) return false;
    uint8_t renderPending = 0, renderKind = 0, renderTarget = 0, renderStore = 0;
    if (!get8(renderPending) || !get16(m_renderFetchAddress) || !get8(renderKind) ||
        !get8(renderTarget) || !get8(m_renderFetchSpriteSlot) || !get8(renderStore)) return false;
    if (renderPending > 1 || renderKind > static_cast<uint8_t>(PpuFetchKind::Sprite) ||
        renderTarget > static_cast<uint8_t>(RenderFetchTarget::DummyNt339) ||
        m_renderFetchSpriteSlot > 7 || renderStore > 1 || (m_renderFetchAddress & 0xC000) != 0) return false;
    m_renderFetchPending = renderPending != 0;
    m_renderFetchKind = static_cast<PpuFetchKind>(renderKind);
    m_renderFetchTarget = static_cast<RenderFetchTarget>(renderTarget);
    m_renderFetchStoreData = renderStore != 0;
    uint8_t renderAle = 0, lowOverride = 0, liveHigh = 0;
    if (!get8(m_ppuAddressLatchLow) || !get8(renderAle) || !get16(m_renderAleAddress) ||
        !get8(m_renderAlePreviousLow) || !get8(lowOverride) || !get8(liveHigh)) return false;
    if (renderAle > 1 || lowOverride > 1 || liveHigh > 1 || (m_renderAleAddress & 0xC000) != 0) return false;
    m_renderAleThisClock = renderAle != 0;
    m_renderFetchLowOverride = lowOverride != 0;
    m_renderFetchUseLiveHigh = liveHigh != 0;
    m_ppuBusReadThisClock = false;
    m_ppuBusReadData = 0;
    m_ppuBusTraceDots = 0;
    m_ppuBusReadSource = "none";
    if (!getBytes(m_oam, 256)) return false;
    if (!getBytes(m_oamSecondary, 32)) return false;
    if (!get8(m_spriteCount)) return false;
    if (!get8(fc)) return false;
    m_spriteZeroPossible = fc != 0;
    if (!get8(fc)) return false;
    m_spriteZeroBeingRendered = fc != 0;
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
    if (!get8(fc)) return false;
    m_bgPatternLoadArmed = fc != 0;
    if (!get8(m_busLatch)) return false;
    for (uint64_t& t : m_busBitRefreshClock) {
        uint32_t lo = 0, hi = 0;
        if (!get32(lo) || !get32(hi)) return false;
        t = static_cast<uint64_t>(lo) | (static_cast<uint64_t>(hi) << 32);
    }
    if (!get8(fc)) return false;
    m_nmiLine = fc != 0;
    if (!get8(m_nmiDelay)) return false;
    if (!get8(fc)) return false;
    m_suppressVBlank = fc != 0;
    if (!get8(fc)) return false;
    m_renderingActiveLastClock = fc != 0;
    if (!get8(fc)) return false;
    m_oamCorruptionPending = fc != 0;
    if (!get8(m_oamCorruptionSeed)) return false;
    if (m_nmiDelay > 2 || m_oamCorruptionSeed > 0x1F) return false;
    uint32_t inhibitLo = 0, inhibitHi = 0;
    if (!get32(inhibitLo) || !get32(inhibitHi)) return false;
    m_registerWriteInhibitUntilClock = static_cast<uint64_t>(inhibitLo) | (static_cast<uint64_t>(inhibitHi) << 32);

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
