#include "Cartridge.hpp"
#include "CPU.hpp"
#include <fstream>
#include <filesystem>

Cartridge::Cartridge() {}

void Cartridge::loadBattery()
{
    if (!m_battery || m_batteryPath.empty() || m_prgRam.empty())
        return;
    std::ifstream f(m_batteryPath, std::ios::binary);
    if (!f) return;
    f.read(reinterpret_cast<char*>(m_prgRam.data()), (std::streamsize)m_prgRam.size());
}

void Cartridge::saveBattery() const
{
    if (!m_battery || m_batteryPath.empty() || m_prgRam.empty())
        return;
    std::ofstream f(m_batteryPath, std::ios::binary);
    if (!f) return;
    f.write(reinterpret_cast<const char*>(m_prgRam.data()), (std::streamsize)m_prgRam.size());
}

bool Cartridge::loadFromFile(const std::string& path)
{
    // Save previous battery if any
    saveBattery();

    m_loaded = false;
    m_battery = false;
    m_prgRom.clear();
    m_chrRom.clear();
    m_chrRam.clear();
    m_prgRam.assign(0x2000, 0);
    m_path.clear();
    m_fileName.clear();
    m_batteryPath.clear();
    m_prgBanks = 0;
    m_chrBanks = 0;
    m_mapper = 0;
    m_mirror = Mirror::Horizontal;

    m_shiftReg = 0x10;
    m_mmc1Ctrl = 0x0C;
    m_mmc1Chr0 = m_mmc1Chr1 = m_mmc1Prg = 0;
    m_unromBank = 0;
    m_cnromBank = 0;
    m_axromBank = 0;

    m_mmc3BankSelect = 0;
    for (int i = 0; i < 8; i++) m_mmc3Regs[i] = 0;
    m_mmc3PrgMode = m_mmc3ChrMode = 0;
    m_mmc3IrqLatch = m_mmc3IrqCounter = 0;
    m_mmc3IrqEnabled = m_mmc3IrqReload = false;

    std::ifstream rom(path, std::ios::binary);
    if (!rom.is_open()) return false;

    uint8_t header[16];
    rom.read(reinterpret_cast<char*>(header), 16);
    if (!rom || header[0] != 'N' || header[1] != 'E' || header[2] != 'S' || header[3] != 0x1A)
        return false;

    m_prgBanks = header[4];
    m_chrBanks = header[5];
    uint8_t flags6 = header[6];
    uint8_t flags7 = header[7];
    m_mapper = (flags7 & 0xF0) | (flags6 >> 4);
    m_battery = (flags6 & 0x02) != 0;

    if (flags6 & 0x08)
        m_mirror = Mirror::OnescreenLo;
    else if (flags6 & 0x01)
        m_mirror = Mirror::Vertical;
    else
        m_mirror = Mirror::Horizontal;

    if (flags6 & 0x04)
        rom.seekg(512, std::ios::cur);

    std::size_t prgSize = (std::size_t)m_prgBanks * 16 * 1024;
    std::size_t chrSize = (std::size_t)m_chrBanks * 8 * 1024;

    m_prgRom.resize(prgSize);
    if (prgSize > 0) {
        rom.read(reinterpret_cast<char*>(m_prgRom.data()), prgSize);
        if (!rom) return false;
    }

    if (chrSize > 0) {
        m_chrRom.resize(chrSize);
        rom.read(reinterpret_cast<char*>(m_chrRom.data()), chrSize);
        if (!rom) return false;
    }
    else {
        m_chrRam.assign(8 * 1024, 0);
    }

    m_path = path;
    try {
        std::filesystem::path p(path);
        m_fileName = p.filename().string();
        m_batteryPath = (p.parent_path() / (p.stem().string() + ".sav")).string();
    }
    catch (...) {
        m_fileName = path;
        m_batteryPath = path + ".sav";
    }

    if (m_battery)
        loadBattery();

    m_loaded = true;
    return true;
}

void Cartridge::applyMmc1Mirroring()
{
    switch (m_mmc1Ctrl & 0x03) {
    case 0: m_mirror = Mirror::OnescreenLo; break;
    case 1: m_mirror = Mirror::OnescreenHi; break;
    case 2: m_mirror = Mirror::Vertical; break;
    case 3: m_mirror = Mirror::Horizontal; break;
    }
}

void Cartridge::mmc1Write(uint16_t addr, uint8_t data)
{
    if (data & 0x80) {
        m_shiftReg = 0x10;
        m_mmc1Ctrl |= 0x0C;
        return;
    }
    bool complete = (m_shiftReg & 1) != 0;
    m_shiftReg = (m_shiftReg >> 1) | ((data & 1) << 4);
    if (!complete) return;

    uint8_t value = m_shiftReg & 0x1F;
    m_shiftReg = 0x10;

    if (addr < 0xA000) { m_mmc1Ctrl = value; applyMmc1Mirroring(); }
    else if (addr < 0xC000) m_mmc1Chr0 = value;
    else if (addr < 0xE000) m_mmc1Chr1 = value;
    else m_mmc1Prg = value & 0x0F;
}

void Cartridge::mmc3Write(uint16_t addr, uint8_t data)
{
    switch (addr & 0xE001) {
    case 0x8000:
        m_mmc3BankSelect = data;
        m_mmc3PrgMode = (data >> 6) & 1;
        m_mmc3ChrMode = (data >> 7) & 1;
        break;
    case 0x8001:
        m_mmc3Regs[m_mmc3BankSelect & 0x07] = data;
        break;
    case 0xA000:
        m_mirror = (data & 1) ? Mirror::Horizontal : Mirror::Vertical;
        break;
    case 0xA001: break;
    case 0xC000: m_mmc3IrqLatch = data; break;
    case 0xC001: m_mmc3IrqReload = true; break;
    case 0xE000: m_mmc3IrqEnabled = false; break;
    case 0xE001: m_mmc3IrqEnabled = true; break;
    }
}

void Cartridge::scanlineTick()
{
    if (m_mapper != 4) return;

    if (m_mmc3IrqCounter == 0 || m_mmc3IrqReload) {
        m_mmc3IrqCounter = m_mmc3IrqLatch;
        m_mmc3IrqReload = false;
    }
    else {
        m_mmc3IrqCounter--;
    }
    if (m_mmc3IrqCounter == 0 && m_mmc3IrqEnabled && m_cpu)
        m_cpu->irq();
}

uint32_t Cartridge::mmc3MapPrg(uint16_t addr) const
{
    const uint32_t bank8 = 0x2000;
    const uint32_t nBanks = (uint32_t)m_prgRom.size() / bank8;
    auto map = [&](uint8_t bank) { return (bank % nBanks) * bank8; };
    uint8_t R6 = m_mmc3Regs[6] & 0x3F;
    uint8_t R7 = m_mmc3Regs[7] & 0x3F;
    uint8_t last = (uint8_t)(nBanks - 1);
    uint8_t secondLast = (uint8_t)(nBanks - 2);

    if (m_mmc3PrgMode == 0) {
        if (addr < 0xA000) return map(R6) + (addr - 0x8000);
        if (addr < 0xC000) return map(R7) + (addr - 0xA000);
        if (addr < 0xE000) return map(secondLast) + (addr - 0xC000);
        return map(last) + (addr - 0xE000);
    }
    else {
        if (addr < 0xA000) return map(secondLast) + (addr - 0x8000);
        if (addr < 0xC000) return map(R7) + (addr - 0xA000);
        if (addr < 0xE000) return map(R6) + (addr - 0xC000);
        return map(last) + (addr - 0xE000);
    }
}

uint32_t Cartridge::mmc3MapChr(uint16_t addr) const
{
    const uint32_t nBanks = m_chrRom.empty() ? 1u : (uint32_t)(m_chrRom.size() / 0x400);
    auto map1k = [&](uint8_t bank) { return (bank % nBanks) * 0x400; };
    uint8_t r0 = m_mmc3Regs[0] & 0xFE;
    uint8_t r1 = m_mmc3Regs[1] & 0xFE;

    if (m_mmc3ChrMode == 0) {
        if (addr < 0x0800) return map1k(r0) + (addr - 0x0000);
        if (addr < 0x1000) return map1k(r1) + (addr - 0x0800);
        if (addr < 0x1400) return map1k(m_mmc3Regs[2]) + (addr - 0x1000);
        if (addr < 0x1800) return map1k(m_mmc3Regs[3]) + (addr - 0x1400);
        if (addr < 0x1C00) return map1k(m_mmc3Regs[4]) + (addr - 0x1800);
        return map1k(m_mmc3Regs[5]) + (addr - 0x1C00);
    }
    else {
        if (addr < 0x0400) return map1k(m_mmc3Regs[2]) + (addr - 0x0000);
        if (addr < 0x0800) return map1k(m_mmc3Regs[3]) + (addr - 0x0400);
        if (addr < 0x0C00) return map1k(m_mmc3Regs[4]) + (addr - 0x0800);
        if (addr < 0x1000) return map1k(m_mmc3Regs[5]) + (addr - 0x0C00);
        if (addr < 0x1800) return map1k(r0) + (addr - 0x1000);
        return map1k(r1) + (addr - 0x1800);
    }
}

uint8_t Cartridge::cpuRead(uint16_t addr) const
{
    if (!m_loaded) return 0;
    if (addr >= 0x6000 && addr <= 0x7FFF) {
        if (!m_prgRam.empty()) return m_prgRam[addr - 0x6000];
        return 0;
    }
    if (addr < 0x8000 || m_prgRom.empty()) return 0;

    const uint32_t prgSize = (uint32_t)m_prgRom.size();
    const uint32_t bank16 = 0x4000;
    const uint32_t bank32 = 0x8000;
    uint32_t offset = 0;

    switch (m_mapper) {
    case 0:
        offset = addr - 0x8000;
        if (prgSize == bank16) offset %= bank16;
        else offset %= prgSize;
        break;
    case 1: {
        uint8_t mode = (m_mmc1Ctrl >> 2) & 0x03;
        if (mode <= 1) {
            offset = (m_mmc1Prg & 0x0E) * bank16 + (addr - 0x8000);
        }
        else if (mode == 2) {
            if (addr < 0xC000) offset = addr - 0x8000;
            else offset = m_mmc1Prg * bank16 + (addr - 0xC000);
        }
        else {
            if (addr < 0xC000) offset = m_mmc1Prg * bank16 + (addr - 0x8000);
            else offset = prgSize - bank16 + (addr - 0xC000);
        }
        offset %= prgSize;
        break;
    }
    case 2:
        if (addr < 0xC000) offset = m_unromBank * bank16 + (addr - 0x8000);
        else offset = prgSize - bank16 + (addr - 0xC000);
        offset %= prgSize;
        break;
    case 3:
        offset = addr - 0x8000;
        if (prgSize == bank16) offset %= bank16;
        else offset %= prgSize;
        break;
    case 4:
        offset = mmc3MapPrg(addr) % prgSize;
        break;
    case 7: // AxROM – 32 KB banks at $8000, mirroring from bit 4
        offset = (m_axromBank & 0x07) * bank32 + (addr - 0x8000);
        offset %= prgSize;
        break;
    default:
        offset = (addr - 0x8000) % prgSize;
        break;
    }
    return m_prgRom[offset];
}

void Cartridge::cpuWrite(uint16_t addr, uint8_t data)
{
    if (!m_loaded) return;
    if (addr >= 0x6000 && addr <= 0x7FFF) {
        if (!m_prgRam.empty()) m_prgRam[addr - 0x6000] = data;
        return;
    }
    if (addr < 0x8000) return;

    switch (m_mapper) {
    case 1: mmc1Write(addr, data); break;
    case 2: m_unromBank = data & 0x0F; break;
    case 3: m_cnromBank = data & 0x03; break;
    case 4: mmc3Write(addr, data); break;
    case 7:
        m_axromBank = data;
        m_mirror = (data & 0x10) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
        break;
    default: break;
    }
}

uint8_t Cartridge::ppuRead(uint16_t addr) const
{
    addr &= 0x1FFF;
    if (!m_chrRom.empty()) {
        uint32_t offset = addr;
        const uint32_t chrSize = (uint32_t)m_chrRom.size();
        switch (m_mapper) {
        case 1:
            if (m_mmc1Ctrl & 0x10) {
                if (addr < 0x1000) offset = m_mmc1Chr0 * 0x1000 + addr;
                else offset = m_mmc1Chr1 * 0x1000 + (addr - 0x1000);
            }
            else {
                offset = (m_mmc1Chr0 & 0x1E) * 0x1000 + addr;
            }
            break;
        case 3:
            offset = m_cnromBank * 0x2000 + addr;
            break;
        case 4:
            offset = mmc3MapChr(addr);
            break;
        default: break;
        }
        return m_chrRom[offset % chrSize];
    }
    if (!m_chrRam.empty())
        return m_chrRam[addr % m_chrRam.size()];
    return 0;
}

void Cartridge::ppuWrite(uint16_t addr, uint8_t data)
{
    addr &= 0x1FFF;
    if (!m_chrRam.empty())
        m_chrRam[addr % m_chrRam.size()] = data;
}







