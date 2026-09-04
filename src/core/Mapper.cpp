#include "Mapper.hpp"
#include "MapperFamilies.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr uint64_t kNoCpuCycle = ~uint64_t(0);

void put8(std::vector<uint8_t>& out, uint8_t value)
{
    out.push_back(value);
}

void put64(std::vector<uint8_t>& out, uint64_t value)
{
    for (int i = 0; i < 8; ++i)
        put8(out, static_cast<uint8_t>(value >> (i * 8)));
}

bool get8(const uint8_t*& p, const uint8_t* end, uint8_t& value)
{
    if (p >= end) return false;
    value = *p++;
    return true;
}

bool get64(const uint8_t*& p, const uint8_t* end, uint64_t& value)
{
    if (end - p < 8) return false;
    value = 0;
    for (int i = 0; i < 8; ++i)
        value |= uint64_t(*p++) << (i * 8);
    return true;
}

uint32_t mapBank(std::size_t bank, std::size_t bankSize, std::size_t totalSize, uint32_t inBank)
{
    const std::size_t count = std::max<std::size_t>(1, totalSize / bankSize);
    return static_cast<uint32_t>((bank % count) * bankSize + inBank);
}

class MapperFallback final : public Mapper {
public:
    explicit MapperFallback(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>((addr - 0x8000) % m_config.prgRomSize);
        return true;
    }

    bool implementationSupported() const override { return false; }
};

class Mapper128 final : public Mapper {
public:
    explicit Mapper128(const MapperConfig& config) : Mapper(config) { reset(true); }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        const std::size_t outer = std::size_t(m_outerAddress >> 2);
        const std::size_t bank = (addr < 0xC000) ? (outer | (m_latch & 0x07u)) : (outer | 0x07u);
        mapped = mapBank(bank, 0x4000, m_config.prgRomSize, addr & 0x3FFFu);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;

        if (m_outerAddress < 0xF000u) m_outerAddress = addr;
        m_latch = data;
        updateMirror();
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        mapped = uint32_t(addr % size);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRomSize != 0 || m_config.chrRamSize == 0 || addr >= 0x2000) return false;
        mapped = uint32_t(addr % m_config.chrRamSize);
        return true;
    }

    void reset(bool hard) override
    {
        if (!hard) return;
        m_outerAddress = 0;
        m_latch = 0;
        updateMirror();
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, uint8_t(m_outerAddress));
        put8(out, uint8_t(m_outerAddress >> 8));
        put8(out, m_latch);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t lo=0, hi=0;
        if (!get8(p,end,lo) || !get8(p,end,hi) || !get8(p,end,m_latch)) return false;
        m_outerAddress = uint16_t(lo) | (uint16_t(hi) << 8);
        updateMirror();
        return true;
    }

private:
    void updateMirror()
    {

        m_mirror = (m_outerAddress & 0x0002u) ? Mirror::Horizontal : Mirror::Vertical;
    }

    uint16_t m_outerAddress = 0;
    uint8_t m_latch = 0;
};

class Mapper186 final : public Mapper {
public:
    explicit Mapper186(const MapperConfig& config) : Mapper(config) { reset(true); }

    bool cpuReadRegister(uint16_t addr, uint8_t& data) override
    {
        if (addr == 0x4200 || addr == 0x4201 || addr == 0x4203) { data = 0x00; return true; }
        if (addr == 0x4202) { data = 0x40; return true; }
        if (addr >= 0x4204 && addr <= 0x43FF) { data = 0xFF; return true; }
        if (addr >= 0x4400 && addr <= 0x4EFF) { data = m_internalRam[addr - 0x4400]; return true; }
        return false;
    }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        const std::size_t bank = addr < 0xC000 ? m_prgBank : 0;
        mapped = mapBank(bank, 0x4000, m_config.prgRomSize, addr & 0x3FFF);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr >= 0x4200 && addr <= 0x43FF) {
            if (addr & 1) m_prgBank = data;
            else m_ramBank = uint8_t(data >> 6);
            return true;
        }
        if (addr >= 0x4400 && addr <= 0x4EFF) {
            m_internalRam[addr - 0x4400] = data;
            return true;
        }
        return false;
    }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool) const override
    {
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0) return false;
        mapped = mapBank(m_ramBank, 0x2000, m_config.prgRamSize, addr - 0x6000);
        return true;
    }

    void reset(bool hard) override
    {
        if (!hard) return;
        m_prgBank = 0;
        m_ramBank = 0;
        m_internalRam.fill(0);
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, m_prgBank);
        put8(out, m_ramBank);
        out.insert(out.end(), m_internalRam.begin(), m_internalRam.end());
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        if (!get8(p, end, m_prgBank) || !get8(p, end, m_ramBank)) return false;
        if (end - p < static_cast<std::ptrdiff_t>(m_internalRam.size())) return false;
        std::memcpy(m_internalRam.data(), p, m_internalRam.size());
        p += m_internalRam.size();
        return true;
    }

private:
    uint8_t m_prgBank = 0;
    uint8_t m_ramBank = 0;
    std::array<uint8_t, 0xB00> m_internalRam{};
};

class Mapper9 final : public Mapper {
public:
    explicit Mapper9(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize < 0x8000) return false;
        const std::size_t n8 = std::max<std::size_t>(1, m_config.prgRomSize / 0x2000);
        if (addr < 0xA000) mapped = mapBank(m_prg, 0x2000, m_config.prgRomSize, addr - 0x8000);
        else if (addr < 0xC000) mapped = static_cast<uint32_t>((n8 - std::min<std::size_t>(3, n8)) * 0x2000 + (addr - 0xA000));
        else if (addr < 0xE000) mapped = static_cast<uint32_t>((n8 - std::min<std::size_t>(2, n8)) * 0x2000 + (addr - 0xC000));
        else mapped = static_cast<uint32_t>((n8 - 1) * 0x2000 + (addr - 0xE000));
        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0xA000) return false;
        switch (addr & 0xF000) {
        case 0xA000: m_prg = data & 0x0F; break;
        case 0xB000: m_chrFD0 = data & 0x1F; break;
        case 0xC000: m_chrFE0 = data & 0x1F; break;
        case 0xD000: m_chrFD1 = data & 0x1F; break;
        case 0xE000: m_chrFE1 = data & 0x1F; break;
        case 0xF000: m_mirror = (data & 1) ? Mirror::Horizontal : Mirror::Vertical; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        const uint8_t bank = addr < 0x1000
            ? (m_latch0 ? m_chrFE0 : m_chrFD0)
            : (m_latch1 ? m_chrFE1 : m_chrFD1);
        mapped = mapBank(bank, 0x1000, chrSize, addr & 0x0FFF);
        updateLatch(addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, static_cast<uint8_t>(m_mirror));
        put8(out, m_prg); put8(out, m_chrFD0); put8(out, m_chrFE0); put8(out, m_chrFD1); put8(out, m_chrFE1);
        put8(out, m_latch0 ? 1 : 0); put8(out, m_latch1 ? 1 : 0);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t tmp = 0;
        if (!get8(p, end, tmp)) return false;
        m_mirror = static_cast<Mirror>(tmp);
        if (!get8(p, end, m_prg) || !get8(p, end, m_chrFD0) || !get8(p, end, m_chrFE0) ||
            !get8(p, end, m_chrFD1) || !get8(p, end, m_chrFE1)) return false;
        if (!get8(p, end, tmp)) return false;
        m_latch0 = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_latch1 = tmp != 0;
        return true;
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chrFD0 = 0;
    uint8_t m_chrFE0 = 0;
    uint8_t m_chrFD1 = 0;
    uint8_t m_chrFE1 = 0;

    bool m_latch0 = true;
    bool m_latch1 = true;

    void updateLatch(uint16_t addr)
    {
        if (addr == 0x0FD8) m_latch0 = false;
        else if (addr == 0x0FE8) m_latch0 = true;
        else if (addr >= 0x1FD8 && addr <= 0x1FDF) m_latch1 = false;
        else if (addr >= 0x1FE8 && addr <= 0x1FEF) m_latch1 = true;
    }
};

class Mapper10 final : public Mapper {
public:
    explicit Mapper10(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        if (addr < 0xC000)
            mapped = mapBank(m_prg, 0x4000, m_config.prgRomSize, addr - 0x8000);
        else
            mapped = static_cast<uint32_t>(m_config.prgRomSize - 0x4000 + (addr - 0xC000));
        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0xA000) return false;
        switch (addr & 0xF000) {
        case 0xA000: m_prg = data & 0x0F; break;
        case 0xB000: m_chrFD0 = data & 0x1F; break;
        case 0xC000: m_chrFE0 = data & 0x1F; break;
        case 0xD000: m_chrFD1 = data & 0x1F; break;
        case 0xE000: m_chrFE1 = data & 0x1F; break;
        case 0xF000: m_mirror = (data & 1) ? Mirror::Horizontal : Mirror::Vertical; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        const uint8_t bank = addr < 0x1000
            ? (m_latch0 ? m_chrFE0 : m_chrFD0)
            : (m_latch1 ? m_chrFE1 : m_chrFD1);
        mapped = mapBank(bank, 0x1000, chrSize, addr & 0x0FFF);
        updateLatch(addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, static_cast<uint8_t>(m_mirror));
        put8(out, m_prg); put8(out, m_chrFD0); put8(out, m_chrFE0); put8(out, m_chrFD1); put8(out, m_chrFE1);
        put8(out, m_latch0 ? 1 : 0); put8(out, m_latch1 ? 1 : 0);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t tmp = 0;
        if (!get8(p, end, tmp)) return false;
        m_mirror = static_cast<Mirror>(tmp);
        if (!get8(p, end, m_prg) || !get8(p, end, m_chrFD0) || !get8(p, end, m_chrFE0) ||
            !get8(p, end, m_chrFD1) || !get8(p, end, m_chrFE1)) return false;
        if (!get8(p, end, tmp)) return false;
        m_latch0 = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_latch1 = tmp != 0;
        return true;
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chrFD0 = 0;
    uint8_t m_chrFE0 = 0;
    uint8_t m_chrFD1 = 0;
    uint8_t m_chrFE1 = 0;

    bool m_latch0 = true;
    bool m_latch1 = true;

    void updateLatch(uint16_t addr)
    {
        if (addr >= 0x0FD8 && addr <= 0x0FDF) m_latch0 = false;
        else if (addr >= 0x0FE8 && addr <= 0x0FEF) m_latch0 = true;
        else if (addr >= 0x1FD8 && addr <= 0x1FDF) m_latch1 = false;
        else if (addr >= 0x1FE8 && addr <= 0x1FEF) m_latch1 = true;
    }
};

class Mapper13 final : public Mapper {
public:
    explicit Mapper13(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>((addr - 0x8000) % m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_chr = data & 0x03;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000 || m_config.chrRamSize == 0) return false;
        if (addr < 0x1000)
            mapped = addr;
        else
            mapped = mapBank(m_chr, 0x1000, m_config.chrRamSize, addr - 0x1000);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override { return ppuMapRead(addr, mapped); }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    bool hasBusConflicts() const override { return true; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_chr); }

private:
    uint8_t m_chr = 0;
};

class Mapper30 final : public Mapper {
public:
    explicit Mapper30(const MapperConfig& config)
        : Mapper(config),
          m_oneScreenHeader(config.headerMirror == Mirror::OnescreenLo || config.headerMirror == Mirror::OnescreenHi),
          m_hvControlled(config.nes20 && config.submapper == 3),
          m_ledVariant(config.nes20 && config.submapper == 4),
          m_flashEnabled(config.hasBattery && (!config.nes20 || config.submapper == 0 || config.submapper == 1 || config.submapper == 4))
    {
        updateMirror();
    }

    void initializePrgImage(const uint8_t* data, std::size_t size) override
    {
        if (!m_flashEnabled) return;

        m_flash.assign(0x80000, 0xFF);
        if (data && size)
            std::copy_n(data, std::min<std::size_t>(size, m_flash.size()), m_flash.begin());
    }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        if (addr < 0xC000)
            mapped = mapBank(m_bank & 0x1F, 0x4000, m_config.prgRomSize, addr - 0x8000);
        else
            mapped = static_cast<uint32_t>(m_config.prgRomSize - 0x4000 + (addr - 0xC000));
        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuReadRegister(uint16_t addr, uint8_t& data) override
    {
        if (!m_flashEnabled || addr < 0x8000 || m_flash.empty()) return false;
        const uint32_t phys = flashPhysicalAddress(addr);
        if (m_flashIdMode) {

            const uint16_t chipAddr = uint16_t(phys & 0x7FFF);
            if (chipAddr == 0) data = 0xBF;
            else if (chipAddr == 1) data = 0xB7;
            else data = 0xFF;
        } else {
            data = m_flash[phys % uint32_t(m_flash.size())];
        }
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;

        if (m_flashEnabled && addr < 0xC000) {
            flashWrite(flashPhysicalAddress(addr), data);
            if (m_ledVariant) m_led = data;
            return true;
        }

        const bool latchHighOnly = m_flashEnabled || (m_config.nes20 && m_config.submapper >= 1);
        if (latchHighOnly && addr < 0xC000) {
            if (m_ledVariant) m_led = data;
            return true;
        }

        m_bank = data;
        updateMirror();
        return true;
    }

    bool hasBusConflicts() const override
    {
        if (m_config.nes20) {
            if (m_config.submapper == 2) return true;
            if (m_config.submapper >= 1) return false;
        }
        return !m_config.hasBattery;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000 || m_config.chrRamSize == 0) return false;
        mapped = mapBank((m_bank >> 5) & 0x03, 0x2000, m_config.chrRamSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override { return ppuMapRead(addr, mapped); }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    std::size_t mapperBatterySize() const override
    {
        return m_flashEnabled && m_flash.size() == 0x80000 ? m_flash.size() : 0;
    }

    void saveMapperBattery(std::vector<uint8_t>& out) const override
    {
        if (mapperBatterySize()) out.insert(out.end(), m_flash.begin(), m_flash.end());
    }

    bool loadMapperBattery(const uint8_t* data, std::size_t size) override
    {
        if (!m_flashEnabled) return size == 0;
        if (!data || size != 0x80000) return false;
        m_flash.assign(data, data + size);
        return true;
    }

    void reset(bool hard) override
    {
        if (hard) {
            m_bank = 0;
            m_led = 0xFF;
            updateMirror();
        }

        m_flashState = 0;
        m_flashIdMode = false;
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, m_bank);
        put8(out, m_led);
        put8(out, m_flashState);
        put8(out, m_flashIdMode ? 1 : 0);
        if (m_flashEnabled) out.insert(out.end(), m_flash.begin(), m_flash.end());
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t idMode = 0;
        if (!get8(p, end, m_bank) || !get8(p, end, m_led) ||
            !get8(p, end, m_flashState) || !get8(p, end, idMode)) return false;
        m_flashIdMode = idMode != 0;
        if (m_flashEnabled) {
            if (end - p < 0x80000) return false;
            m_flash.assign(p, p + 0x80000);
            p += 0x80000;
        }
        updateMirror();
        return true;
    }

private:
    uint8_t m_bank = 0;
    uint8_t m_led = 0xFF;
    bool m_oneScreenHeader = false;
    bool m_hvControlled = false;
    bool m_ledVariant = false;
    bool m_flashEnabled = false;
    std::vector<uint8_t> m_flash;
    uint8_t m_flashState = 0;
    bool m_flashIdMode = false;

    uint32_t flashPhysicalAddress(uint16_t addr) const
    {
        if (addr < 0xC000)
            return (uint32_t(m_bank & 0x1F) * 0x4000u + uint32_t(addr - 0x8000)) & 0x7FFFFu;
        return (0x7C000u + uint32_t(addr - 0xC000)) & 0x7FFFFu;
    }

    void flashWrite(uint32_t phys, uint8_t data)
    {
        if (m_flash.empty()) return;
        const uint16_t chipAddr = uint16_t(phys & 0x7FFF);

        if (data == 0xF0 && m_flashState != 3) {
            m_flashState = 0;
            m_flashIdMode = false;
            return;
        }

        switch (m_flashState) {
        case 0:
            m_flashState = (chipAddr == 0x5555 && data == 0xAA) ? 1 : 0;
            break;
        case 1:
            m_flashState = (chipAddr == 0x2AAA && data == 0x55) ? 2 : 0;
            break;
        case 2:
            if (chipAddr == 0x5555 && data == 0xA0) m_flashState = 3;
            else if (chipAddr == 0x5555 && data == 0x80) m_flashState = 4;
            else if (chipAddr == 0x5555 && data == 0x90) {
                m_flashIdMode = true;
                m_flashState = 0;
            } else m_flashState = 0;
            break;
        case 3:

            m_flash[phys & 0x7FFFFu] &= data;
            m_flashState = 0;
            break;
        case 4:
            m_flashState = (chipAddr == 0x5555 && data == 0xAA) ? 5 : 0;
            break;
        case 5:
            m_flashState = (chipAddr == 0x2AAA && data == 0x55) ? 6 : 0;
            break;
        case 6:
            if (data == 0x30) {
                const uint32_t base = phys & ~uint32_t(0x0FFF);
                const uint32_t limit = std::min<uint32_t>(base + 0x1000u, uint32_t(m_flash.size()));
                std::fill(m_flash.begin() + base, m_flash.begin() + limit, uint8_t{0xFF});
            } else if (chipAddr == 0x5555 && data == 0x10) {
                std::fill(m_flash.begin(), m_flash.end(), uint8_t{0xFF});
            }
            m_flashState = 0;
            break;
        default:
            m_flashState = 0;
            break;
        }
    }

    void updateMirror()
    {
        if (m_hvControlled) {

            m_mirror = (m_bank & 0x80) ? Mirror::Horizontal : Mirror::Vertical;
        }
        else if (m_oneScreenHeader) {
            m_mirror = (m_bank & 0x80) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
        }
    }
};

class Mapper34 final : public Mapper {
public:
    explicit Mapper34(const MapperConfig& config)
        : Mapper(config),
          m_nina(config.submapper == 1 || (config.submapper == 0 && config.chrRomSize > 0x2000))
    {
    }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = mapBank(m_prg, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (m_nina) {
            if (addr == 0x7FFD) { m_prg = data & 0x01; return true; }
            if (addr == 0x7FFE) { m_chr0 = data & 0x0F; return true; }
            if (addr == 0x7FFF) { m_chr1 = data & 0x0F; return true; }
            return false;
        }
        if (addr < 0x8000) return false;
        m_prg = data;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        if (!m_nina) return Mapper::ppuMapRead(addr, mapped);
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        const uint8_t bank = addr < 0x1000 ? m_chr0 : m_chr1;
        mapped = mapBank(bank, 0x1000, chrSize, addr & 0x0FFF);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    bool hasBusConflicts() const override { return !m_nina; }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool write) const override
    {
        if (!m_nina) return false;
        return Mapper::mapPrgRam(addr, mapped, write);
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, m_prg); put8(out, m_chr0); put8(out, m_chr1);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        return get8(p, end, m_prg) && get8(p, end, m_chr0) && get8(p, end, m_chr1);
    }

private:
    bool m_nina = false;
    uint8_t m_prg = 0;
    uint8_t m_chr0 = 0;
    uint8_t m_chr1 = 0;
};

class Mapper71 final : public Mapper {
public:
    explicit Mapper71(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        if (addr < 0xC000)
            mapped = mapBank(m_prg, 0x4000, m_config.prgRomSize, addr - 0x8000);
        else
            mapped = static_cast<uint32_t>(m_config.prgRomSize - 0x4000 + (addr - 0xC000));
        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr >= 0x8000 && addr <= 0x9FFF) {

            const bool fireHawk = m_config.nes20 ? (m_config.submapper == 1) : (addr >= 0x9000);
            if (!m_config.nes20 && addr >= 0x9000) m_legacyFireHawk = true;
            if (fireHawk)
                m_mirror = (data & 0x10) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
            return true;
        }
        if (addr >= 0xC000) {
            const bool fireHawk = m_config.nes20 ? (m_config.submapper == 1) : m_legacyFireHawk;
            m_prg = data & (fireHawk ? 0x07 : 0x0F);
            return true;
        }
        return false;
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, m_prg); put8(out, static_cast<uint8_t>(m_mirror));
        put8(out, m_legacyFireHawk ? 1 : 0);
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t mirror = 0, legacyFireHawk = 0;
        if (!get8(p, end, m_prg) || !get8(p, end, mirror) || !get8(p, end, legacyFireHawk)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        m_legacyFireHawk = legacyFireHawk != 0;
        return true;
    }

private:
    uint8_t m_prg = 0;
    bool m_legacyFireHawk = false;
};

class Mapper87 final : public Mapper {
public:
    explicit Mapper87(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>((addr - 0x8000) % m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x6000 || addr > 0x7FFF) return false;
        m_chr = static_cast<uint8_t>(((data & 0x01) << 1) | ((data >> 1) & 0x01));
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        mapped = mapBank(m_chr, 0x2000, chrSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_chr); }

private:
    uint8_t m_chr = 0;
};

class Mapper94 final : public Mapper {
public:
    explicit Mapper94(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        if (addr < 0xC000)
            mapped = mapBank(m_prg, 0x4000, m_config.prgRomSize, addr - 0x8000);
        else
            mapped = static_cast<uint32_t>(m_config.prgRomSize - 0x4000 + (addr - 0xC000));
        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_prg = (data >> 2) & 0x07;
        return true;
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_prg); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_prg); }

private:
    uint8_t m_prg = 0;
};

class Mapper180 final : public Mapper {
public:
    explicit Mapper180(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        if (addr < 0xC000)
            mapped = addr - 0x8000;
        else
            mapped = mapBank(m_prg, 0x4000, m_config.prgRomSize, addr - 0xC000);
        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_prg = data;
        return true;
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_prg); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_prg); }

private:
    uint8_t m_prg = 0;
};

class Mapper11 final : public Mapper {
public:
    explicit Mapper11(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = mapBank(m_prg & 3, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;

        m_prg = data & 0x03;
        m_chr = (data >> 4) & 0x0F;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        mapped = mapBank(m_chr, 0x2000, chrSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    bool hasBusConflicts() const override { return true; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_prg); put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_prg) && get8(p, end, m_chr); }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};

class Mapper66 final : public Mapper {
public:
    explicit Mapper66(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = mapBank(m_prg & 3, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_prg = (data >> 4) & 3;
        m_chr = data & 3;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        mapped = mapBank(m_chr, 0x2000, chrSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    bool hasBusConflicts() const override { return true; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_prg); put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_prg) && get8(p, end, m_chr); }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};

class Mapper185 final : public Mapper {
public:
    explicit Mapper185(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>((addr - 0x8000) % m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_latch = data & 0x0F;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000 || m_config.chrRomSize == 0) return false;
        if (!chrEnabled()) return false;

        mapped = static_cast<uint32_t>(addr % m_config.chrRomSize);
        return true;
    }

    bool ppuReadOverride(uint16_t addr, PpuFetchKind kind, uint8_t& data) override
    {

        if (addr >= 0x2000 || m_config.submapper != 0 || kind != PpuFetchKind::Cpu || m_legacyCpuReads >= 2)
            return false;
        ++m_legacyCpuReads;
        data = 0xFF;
        return true;
    }

    bool ppuMapWrite(uint16_t, uint32_t&) override { return false; }
    bool hasBusConflicts() const override { return true; }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    bool implementationSupported() const override
    {
        return m_config.submapper == 0 || (m_config.nes20 && m_config.submapper >= 4 && m_config.submapper <= 7);
    }

    void reset(bool hard) override
    {
        if (hard) m_latch = 0;
        m_legacyCpuReads = 0;
    }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_latch); put8(out, m_legacyCpuReads); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_latch) && get8(p, end, m_legacyCpuReads); }

private:
    uint8_t m_latch = 0;
    uint8_t m_legacyCpuReads = 0;

    bool chrEnabled() const
    {
        if (m_config.submapper == 0) return true;
        if (!m_config.nes20 || m_config.submapper < 4 || m_config.submapper > 7)
            return false;
        return (m_latch & 0x03) == uint8_t(m_config.submapper - 4);
    }
};

class Mapper228 final : public Mapper {
public:
    explicit Mapper228(const MapperConfig& config) : Mapper(config) { updateMirror(); }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;

        const uint8_t chip = uint8_t((m_regAddr >> 11) & 0x03);
        if (chip == 2) return false;

        const std::size_t chipBase = chip == 3 ? 0x100000u : std::size_t(chip) * 0x80000u;
        if (chipBase >= m_config.prgRomSize) return false;

        uint8_t page = uint8_t((m_regAddr >> 6) & 0x1F);
        const bool mirror16 = (m_regAddr & 0x20) != 0;
        if (mirror16) {
            const std::size_t bank16 = page;
            const uint32_t in = addr & 0x3FFF;
            mapped = static_cast<uint32_t>(chipBase + bank16 * 0x4000u + in);
        } else {

            page = uint8_t((page & 0x1E) | ((addr >> 14) & 1));
            mapped = static_cast<uint32_t>(chipBase + std::size_t(page) * 0x4000u + (addr & 0x3FFF));
        }
        return mapped < m_config.prgRomSize;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_regAddr = addr;
        m_chrLow = data & 0x03;
        updateMirror();
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        const uint8_t bank = uint8_t(((m_regAddr & 0x000F) << 2) | m_chrLow);
        mapped = mapBank(bank, 0x2000, chrSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        return m_config.chrRamSize != 0 && ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void reset(bool hard) override
    {
        if (!hard) return;
        m_regAddr = 0x8000;
        m_chrLow = 0;
        updateMirror();
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, uint8_t(m_regAddr));
        put8(out, uint8_t(m_regAddr >> 8));
        put8(out, m_chrLow);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t lo=0, hi=0;
        if (!get8(p,end,lo) || !get8(p,end,hi) || !get8(p,end,m_chrLow)) return false;
        m_regAddr = uint16_t(lo) | (uint16_t(hi) << 8);
        updateMirror();
        return true;
    }

private:
    uint16_t m_regAddr = 0x8000;
    uint8_t m_chrLow = 0;

    void updateMirror()
    {
        m_mirror = (m_regAddr & 0x2000) ? Mirror::Horizontal : Mirror::Vertical;
    }
};

#include "MapperMore.inc"
#include "MapperFds.inc"

class Mapper104 final : public Mapper {
public:
    explicit Mapper104(const MapperConfig& config) : Mapper(config) { reset(true); }
    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        const std::size_t outerBase = std::size_t(m_outer & 0x07) * 0x40000;
        const std::size_t bank16 = (addr < 0xC000) ? (m_inner & 0x0F) : 0x0F;
        mapped = uint32_t((outerBase + bank16 * 0x4000 + (addr & 0x3FFF)) % m_config.prgRomSize);
        return true;
    }
    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr < 0x8000) return false;
        if (addr < 0xC000) {
            if (!m_locked) { m_outer = data & 0x07; m_locked = (data & 0x08) != 0; }
        } else m_inner = data & 0x0F;
        return true;
    }
    void reset(bool hard) override { if (hard) { m_outer=0; m_inner=0; m_locked=false; } }
    void saveState(std::vector<uint8_t>& out) const override { put8(out,m_outer);put8(out,m_inner);put8(out,m_locked?1:0); }
    bool loadState(const uint8_t*& p,const uint8_t* end) override { uint8_t l=0; if(!get8(p,end,m_outer)||!get8(p,end,m_inner)||!get8(p,end,l))return false;m_locked=l!=0;return true; }
private:
    uint8_t m_outer=0,m_inner=0; bool m_locked=false;
};

class Mapper117 final : public Mapper {
public:
    explicit Mapper117(const MapperConfig& config) : Mapper(config) { reset(true); }
    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if(addr<0x8000||m_config.prgRomSize==0)return false;
        const int slot=(addr-0x8000)>>13;
        mapped=mapBank(m_prg[slot],0x2000,m_config.prgRomSize,addr&0x1FFF);return true;
    }
    bool cpuWrite(uint16_t addr,uint8_t data,uint64_t) override {
        if(addr>=0x8000&&addr<=0x8003){m_prg[addr&3]=data;return true;}
        if(addr>=0xA000&&addr<=0xA007){m_chr[addr&7]=data;return true;}
        switch(addr){
        case 0xC001:m_irqReload=data;return true;
        case 0xC002:m_irq=false;return true;
        case 0xC003:m_irqCounter=m_irqReload;m_irqAlt=true;return true;
        case 0xD000:m_mirror=(data&1)?Mirror::Horizontal:Mirror::Vertical;return true;
        case 0xE000:m_irqEnabled=(data&1)!=0;m_irq=false;return true;
        default:return addr>=0x8000;
        }
    }
    bool ppuMapRead(uint16_t addr,uint32_t& mapped) override {
        if(addr>=0x2000)return false;
        const std::size_t sz=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!sz)return false;
        mapped=mapBank(m_chr[addr>>10],0x400,sz,addr&0x3FF);return true;
    }
    bool ppuMapWrite(uint16_t addr,uint32_t& mapped) override { if(!m_config.chrRamSize)return false;return ppuMapRead(addr,mapped); }
    void notifyPpuAddress(uint16_t addr,uint64_t cyc) override {
        const bool hi=(addr&0x1000)!=0;
        if(!hi){ if(m_lastA12||!m_lowValid){m_lowStart=cyc;m_lowValid=true;} }
        else { if(!m_lastA12&&m_lowValid&&cyc-m_lowStart>=8) clockA12(); m_lowValid=false; }
        m_lastA12=hi;
    }
    bool irqActive() const override{return m_irq;}
    void reset(bool hard) override { if(hard){for(auto&v:m_prg)v=0; m_prg[0]=uint8_t(std::max<std::size_t>(4,prgBanks(0x2000))-4);m_prg[1]=m_prg[0]+1;m_prg[2]=m_prg[0]+2;m_prg[3]=m_prg[0]+3;for(auto&v:m_chr)v=0;m_irqCounter=m_irqReload=0;}m_irqEnabled=m_irqAlt=m_irq=false;m_lastA12=m_lowValid=false;m_lowStart=0; }
    void saveState(std::vector<uint8_t>&out)const override{for(auto v:m_prg)put8(out,v);for(auto v:m_chr)put8(out,v);put8(out,m_irqCounter);put8(out,m_irqReload);put8(out,m_irqEnabled);put8(out,m_irqAlt);put8(out,m_irq);put8(out,m_lastA12);put8(out,m_lowValid);put64(out,m_lowStart);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;uint8_t a,b,c,d,f;if(!get8(p,e,m_irqCounter)||!get8(p,e,m_irqReload)||!get8(p,e,a)||!get8(p,e,b)||!get8(p,e,c)||!get8(p,e,d)||!get8(p,e,f)||!get64(p,e,m_lowStart))return false;m_irqEnabled=a;m_irqAlt=b;m_irq=c;m_lastA12=d;m_lowValid=f;return true;}
private:
    void clockA12(){if(m_irqEnabled&&m_irqAlt&&m_irqCounter){if(--m_irqCounter==0){m_irq=true;m_irqAlt=false;}}}
    uint8_t m_prg[4]{},m_chr[8]{},m_irqCounter=0,m_irqReload=0;bool m_irqEnabled=false,m_irqAlt=false,m_irq=false,m_lastA12=false,m_lowValid=false;uint64_t m_lowStart=0;
};

class Mapper120 final : public Mapper {
public:
    explicit Mapper120(const MapperConfig& config):Mapper(config){}
    bool cpuMapRead(uint16_t addr,uint32_t& mapped)const override{
        if(m_config.prgRomSize==0)return false;
        if(addr>=0x6000&&addr<0x8000){mapped=mapBank(m_bank,0x2000,m_config.prgRomSize,addr&0x1FFF);return true;}
        if(addr>=0x8000){mapped=mapBank(8+((addr-0x8000)>>13),0x2000,m_config.prgRomSize,addr&0x1FFF);return true;}
        return false;
    }
    bool cpuWrite(uint16_t addr,uint8_t data,uint64_t)override{if(addr==0x41FF){m_bank=data;return true;}return false;}
    void reset(bool hard)override{if(hard)m_bank=0;}
    void saveState(std::vector<uint8_t>&out)const override{put8(out,m_bank);} bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_bank);}
private:uint8_t m_bank=0;
};

class Mapper111Mmc1 final : public Mapper {
public:
    explicit Mapper111Mmc1(const MapperConfig& config) : Mapper(config) { reset(true); }
    bool implementationSupported() const override { return m_config.chrRomSize != 0; }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        const uint8_t mode = (m_ctrl >> 2) & 3;
        if (mode <= 1) {
            mapped = mapBank((m_prg & 0x0E) >> 1, 0x8000, m_config.prgRomSize, addr - 0x8000);
        } else if (mode == 2) {
            mapped = addr < 0xC000
                ? uint32_t(addr - 0x8000)
                : mapBank(m_prg & 0x0F, 0x4000, m_config.prgRomSize, addr - 0xC000);
        } else {
            mapped = addr < 0xC000
                ? mapBank(m_prg & 0x0F, 0x4000, m_config.prgRomSize, addr - 0x8000)
                : uint32_t(m_config.prgRomSize - 0x4000 + (addr - 0xC000));
        }
        mapped %= uint32_t(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr < 0x8000) return false;

        switch ((addr & 0x6000) >> 13) {
        case 0: m_ctrl = data; updateMirror(); break;
        case 1: m_chr0 = data; break;
        case 2: m_chr1 = data; break;
        case 3: m_prg = data; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000 || m_config.chrRomSize == 0) return false;

        uint8_t page;
        if (m_ctrl & 0x10)
            page = addr < 0x1000 ? m_chr0 : m_chr1;
        else
            page = uint8_t((m_chr0 & 0xFE) + (addr >= 0x1000 ? 1 : 0));
        mapped = mapBank(page, 0x1000, m_config.chrRomSize, addr & 0x0FFF);
        return true;
    }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool) const override {
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0 || (m_prg & 0x10)) return false;
        mapped = uint32_t((addr - 0x6000) % m_config.prgRamSize);
        return true;
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_ctrl = 0x0C; m_chr0 = 0; m_chr1 = 0; m_prg = 0; updateMirror();
    }
    void saveState(std::vector<uint8_t>& out) const override {
        put8(out,m_ctrl); put8(out,m_chr0); put8(out,m_chr1); put8(out,m_prg);
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        if (!get8(p,end,m_ctrl) || !get8(p,end,m_chr0) || !get8(p,end,m_chr1) || !get8(p,end,m_prg)) return false;
        updateMirror(); return true;
    }
private:
    uint8_t m_ctrl=0x0C,m_chr0=0,m_chr1=0,m_prg=0;
    void updateMirror() {
        switch (m_ctrl & 3) {
        case 0: m_mirror=Mirror::OnescreenLo; break;
        case 1: m_mirror=Mirror::OnescreenHi; break;
        case 2: m_mirror=Mirror::Vertical; break;
        case 3: m_mirror=Mirror::Horizontal; break;
        }
    }
};

class Mapper111 final : public Mapper {
public:
    explicit Mapper111(const MapperConfig& config) : Mapper(config) { reset(true); }
    bool implementationSupported() const override { return m_config.chrRomSize == 0; }

    void initializePrgImage(const uint8_t* data, std::size_t size) override {
        m_flash.assign(0x80000, 0xFF);
        if (data && size) std::copy_n(data, std::min<std::size_t>(size, m_flash.size()), m_flash.begin());
    }

    bool cpuMapRead(uint16_t, uint32_t&) const override { return false; }
    bool cpuReadRegister(uint16_t addr, uint8_t& data) override {
        if (addr >= 0x8000 && !m_flash.empty()) {
            if (m_idMode) {
                const uint16_t off = addr & 0x7FFF;
                if (off == 0) data = 0xBF;
                else if (off == 1) data = 0xB7;
                else data = 0xFF;
            } else {
                data = m_flash[(std::size_t(m_reg & 0x0F) * 0x8000 + (addr & 0x7FFF)) % m_flash.size()];
            }
            return true;
        }
        return false;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr >= 0x8000) {
            const uint32_t phys = uint32_t((std::size_t(m_reg & 0x0F) * 0x8000 + (addr & 0x7FFF)) & 0x7FFFF);
            flashWrite(phys, data);
        }

        if ((addr & 0x5000) == 0x5000) m_reg = data;
        return addr >= 0x5000;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000 || m_config.chrRamSize == 0) return false;
        mapped = uint32_t(((m_reg >> 4) & 1) * 0x4000 + addr);
        mapped %= uint32_t(m_config.chrRamSize);
        return true;
    }
    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override { return ppuMapRead(addr, mapped); }
    bool ppuUsesChrRam(uint16_t addr) const override { return addr < 0x2000; }
    bool mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const override {
        if (addr < 0x2000 || addr >= 0x3F00 || m_config.chrRamSize == 0) return false;
        source = NametableSource::ChrRam;
        mapped = uint32_t(((m_reg >> 5) & 1) * 0x4000 + (addr & 0x1FFF));
        mapped %= uint32_t(m_config.chrRamSize);
        return true;
    }
    bool mapNametableWrite(uint16_t addr, NametableSource& source, uint32_t& mapped) const override {
        return mapNametable(addr, source, mapped);
    }

    void reset(bool hard) override {
        if (hard) m_reg = 0;
        m_flashState = 0; m_idMode = false;
    }

    std::size_t mapperBatterySize() const override { return m_flash.size() == 0x80000 ? m_flash.size() : 0; }
    void saveMapperBattery(std::vector<uint8_t>& out) const override { out.insert(out.end(), m_flash.begin(), m_flash.end()); }
    bool loadMapperBattery(const uint8_t* data, std::size_t size) override {
        if (size != 0x80000 || !data) return false;
        m_flash.assign(data, data + size); return true;
    }

    void saveState(std::vector<uint8_t>& out) const override {
        out.push_back(m_reg); out.push_back(m_flashState); out.push_back(m_idMode ? 1 : 0);
        out.insert(out.end(), m_flash.begin(), m_flash.end());
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        if (end - p < 3 + 0x80000) return false;
        m_reg=*p++; m_flashState=*p++; m_idMode=*p++ != 0;
        m_flash.assign(p, p + 0x80000); p += 0x80000; return true;
    }

private:
    void flashWrite(uint32_t phys, uint8_t data) {
        if (m_flash.empty()) return;
        const uint16_t a = uint16_t(phys & 0x7FFF);
        if (data == 0xF0) { m_flashState = 0; m_idMode = false; return; }
        switch (m_flashState) {
        case 0: m_flashState = (a == 0x5555 && data == 0xAA) ? 1 : 0; break;
        case 1: m_flashState = (a == 0x2AAA && data == 0x55) ? 2 : 0; break;
        case 2:
            if (a == 0x5555 && data == 0xA0) m_flashState = 3;
            else if (a == 0x5555 && data == 0x80) m_flashState = 4;
            else if (a == 0x5555 && data == 0x90) { m_idMode = true; m_flashState = 0; }
            else m_flashState = 0;
            break;
        case 3: m_flash[phys & 0x7FFFF] &= data; m_flashState = 0; break;
        case 4: m_flashState = (a == 0x5555 && data == 0xAA) ? 5 : 0; break;
        case 5: m_flashState = (a == 0x2AAA && data == 0x55) ? 6 : 0; break;
        case 6:
            if (data == 0x30) {
                const std::size_t base = static_cast<std::size_t>(phys & ~uint32_t(0x0FFF));
                const std::size_t limit = std::min(base + std::size_t{0x1000}, m_flash.size());
                std::fill(m_flash.begin() + base, m_flash.begin() + limit, uint8_t{0xFF});
            } else if (a == 0x5555 && data == 0x10) std::fill(m_flash.begin(), m_flash.end(), uint8_t{0xFF});
            m_flashState = 0; break;
        default: m_flashState = 0; break;
        }
    }
    uint8_t m_reg=0;
    uint8_t m_flashState=0;
    bool m_idMode=false;
    std::vector<uint8_t> m_flash;
};

}

Mapper::Mapper(const MapperConfig& config)
    : m_config(config), m_mirror(config.headerMirror)
{
}

bool Mapper::cpuReadRegister(uint16_t, uint8_t&) { return false; }
void Mapper::observeCpuRead(uint16_t, uint8_t) {}
void Mapper::observeCpuWrite(uint16_t, uint8_t) {}
uint8_t Mapper::resolveBusConflict(uint16_t, uint8_t cpuData, uint8_t romData) const { return uint8_t(cpuData & romData); }
bool Mapper::cpuWrite(uint16_t, uint8_t, uint64_t) { return false; }

bool Mapper::ppuMapRead(uint16_t addr, uint32_t& mapped)
{
    if (addr >= 0x2000) return false;
    const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
    if (chrSize == 0) return false;
    mapped = static_cast<uint32_t>(addr % chrSize);
    return true;
}

bool Mapper::ppuMapReadEx(uint16_t addr, uint32_t& mapped, PpuFetchKind)
{
    return ppuMapRead(addr, mapped);
}

bool Mapper::ppuReadOverride(uint16_t, PpuFetchKind, uint8_t&)
{
    return false;
}

bool Mapper::ppuMapWrite(uint16_t addr, uint32_t& mapped)
{
    if (m_config.chrRamSize == 0 || !ppuUsesChrRam(addr)) return false;
    return ppuMapRead(addr, mapped);
}

bool Mapper::ppuUsesChrRam(uint16_t addr) const
{
    return addr < 0x2000 && m_config.chrRomSize == 0 && m_config.chrRamSize != 0;
}

bool Mapper::mapPatternCiram(uint16_t, uint32_t&) const { return false; }

bool Mapper::mapNametable(uint16_t, NametableSource&, uint32_t&) const { return false; }
bool Mapper::mapNametableWrite(uint16_t addr, NametableSource& source, uint32_t& mapped) const { return mapNametable(addr, source, mapped); }
uint8_t Mapper::readMapperNametable(uint32_t) const { return 0; }
void Mapper::writeMapperNametable(uint32_t, uint8_t) {}

bool Mapper::mapPrgRam(uint16_t addr, uint32_t& mapped, bool) const
{
    if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
        return false;
    mapped = static_cast<uint32_t>((addr - 0x6000) % m_config.prgRamSize);
    return true;
}

uint8_t Mapper::transformPrgRamRead(uint16_t, uint8_t data) const { return data; }

void Mapper::notifyCpuAddress(uint16_t) {}
void Mapper::notifyPpuAddress(uint16_t, uint64_t) {}
void Mapper::notifyPpuAddressContext(uint16_t addr, uint64_t ppuCycle, int, int) { notifyPpuAddress(addr, ppuCycle); }
void Mapper::notifyPpuScanline(int, bool) {}
void Mapper::clockCpu() {}
void Mapper::scanlineTick() {}
void Mapper::reset(bool) {}
void Mapper::initializePrgImage(const uint8_t*, std::size_t) {}
bool Mapper::irqActive() const { return false; }
float Mapper::expansionAudioSample(bool) const { return 0.0f; }
std::size_t Mapper::mapperBatterySize() const { return 0; }
void Mapper::saveMapperBattery(std::vector<uint8_t>&) const {}
bool Mapper::loadMapperBattery(const uint8_t*, std::size_t size) { return size == 0; }
bool Mapper::loadDiskImage(const std::vector<uint8_t>&) { return false; }
std::size_t Mapper::diskSideCount() const { return 0; }
int Mapper::currentDiskSide() const { return -1; }
bool Mapper::diskInserted() const { return false; }
bool Mapper::setDiskSide(std::size_t) { return false; }
void Mapper::ejectDisk() {}
void Mapper::saveState(std::vector<uint8_t>&) const {}
bool Mapper::loadState(const uint8_t*&, const uint8_t*) { return true; }

std::size_t Mapper::prgBanks(std::size_t bankSize) const
{
    return bankSize ? m_config.prgRomSize / bankSize : 0;
}

std::size_t Mapper::chrBanks(std::size_t bankSize) const
{
    const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
    return bankSize ? size / bankSize : 0;
}

class Mapper487 final : public Mapper {
public:
    explicit Mapper487(const MapperConfig& config) : Mapper(config) { reset(true); }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        const uint32_t bank = prgBank32();
        mapped = static_cast<uint32_t>((uint64_t(bank) * 0x8000u + (addr - 0x8000u)) % m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {

        if ((addr & 0xC180u) == 0x4180u) {
            m_outer = data;
            updateMirror();
            return true;
        }
        const bool colorDreamsMode = (m_outer & 0x20) != 0;
        if (!colorDreamsMode && (addr & 0xC180u) == 0x4100u) {
            m_inner = data;
            return true;
        }
        if (colorDreamsMode && addr >= 0x8000) {
            m_inner = data;
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (size == 0) return false;
        const uint32_t bank = chrBank8();
        mapped = static_cast<uint32_t>((uint64_t(bank) * 0x2000u + addr) % size);
        return true;
    }
    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }
    bool ppuUsesChrRam(uint16_t addr) const override { return addr < 0x2000 && m_config.chrRamSize != 0; }

    void reset(bool hard) override {
        if (hard) { m_outer = 0; m_inner = 0; }
        updateMirror();
    }
    void saveState(std::vector<uint8_t>& out) const override { out.push_back(m_outer); out.push_back(m_inner); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        if (p + 2 > end) return false;
        m_outer = *p++; m_inner = *p++; updateMirror(); return true;
    }

private:
    uint8_t m_outer = 0;
    uint8_t m_inner = 0;

    uint32_t outerGroup() const {
        uint32_t group = (m_outer >> 1) & 0x0F;
        if ((m_outer & 0x20) == 0) group &= 0x07;
        return group;
    }
    uint32_t prgBank32() const {
        const bool largeInner = (m_outer & 0x40) != 0;
        const bool colorDreams = (m_outer & 0x20) != 0;
        const uint32_t innerA15 = largeInner
            ? (colorDreams ? (m_inner & 0x01) : ((m_inner >> 3) & 0x01))
            : (m_outer & 0x01);
        return (outerGroup() << 1) | innerA15;
    }
    uint32_t chrBank8() const {
        const bool largeInner = (m_outer & 0x40) != 0;
        const bool colorDreams = (m_outer & 0x20) != 0;
        const uint32_t chrA15 = largeInner
            ? (colorDreams ? ((m_inner >> 6) & 0x01) : ((m_inner >> 2) & 0x01))
            : (m_outer & 0x01);
        const uint32_t chrA14A13 = colorDreams ? ((m_inner >> 4) & 0x03) : (m_inner & 0x03);
        return (outerGroup() << 3) | (chrA15 << 2) | chrA14A13;
    }
    void updateMirror() { m_mirror = (m_outer & 0x80) ? Mirror::Horizontal : Mirror::Vertical; }
};

std::unique_ptr<Mapper> createMapper(const MapperConfig& config)
{
    if (auto mapper = createNintendoDiscreteMapper(config))
        return mapper;

    switch (config.id) {
    case 487: return std::make_unique<Mapper487>(config);
    case 1: case 155: return createMmc1Mapper(config);
    case 4: case 14: case 37: case 44: case 45: case 47: case 49: case 52: case 74: case 114: case 115: case 118: case 119: case 121: case 123: case 191: case 192: case 194: case 195: case 197: case 215:
        return createMmc3Mapper(config);
    case 182: {
        MapperConfig compat = config; compat.id = 114;
        return createMmc3Mapper(compat);
    }
    case 5: return createMmc5Mapper(config);
    case 6: case 8: case 17: return createUnlicensedMapper(config);
    case 12:
        if (config.submapper == 1) return createUnlicensedMapper(config);
        return createMmc3Mapper(config);
    case 9: return std::make_unique<Mapper9>(config);
    case 10: return std::make_unique<Mapper10>(config);
    case 11: return std::make_unique<Mapper11>(config);
    case 144: return std::make_unique<Mapper144>(config);
    case 13: return std::make_unique<Mapper13>(config);
    case 15: return std::make_unique<Mapper15>(config);
    case 16: case 157: case 159: return std::make_unique<Mapper16>(config);
    case 153: return std::make_unique<Mapper153>(config);
    case 116: return std::make_unique<Mapper116>(config);
    case 117: return std::make_unique<Mapper117>(config);
    case 18: return std::make_unique<Mapper18>(config);
    case 19: case 210: return createNamcoMapper(config);
    case 20: return std::make_unique<Mapper20>(config);
    case 21: case 22: case 23: case 25: case 27: return createVrcMapper(config);
    case 28: return std::make_unique<Mapper28>(config);
    case 29: return std::make_unique<Mapper29>(config);
    case 24: case 26: return createVrcMapper(config);
    case 30: return std::make_unique<Mapper30>(config);
    case 31: return std::make_unique<Mapper31>(config);
    case 32: return std::make_unique<Mapper32>(config);
    case 33: return std::make_unique<Mapper33>(config);
    case 34: return std::make_unique<Mapper34>(config);
    case 35: return createUnlicensedMapper(config);
    case 36: return std::make_unique<Mapper36>(config);
    case 38: return std::make_unique<Mapper38>(config);
    case 39: return std::make_unique<Mapper39>(config);
    case 40: return std::make_unique<Mapper40>(config);
    case 41: return std::make_unique<Mapper41>(config);
    case 42: return std::make_unique<Mapper42>(config);
    case 43: return std::make_unique<Mapper43>(config);
    case 46: return std::make_unique<Mapper46>(config);
    case 48: return std::make_unique<Mapper48>(config);
    case 50: return std::make_unique<Mapper50>(config);
    case 51: return std::make_unique<Mapper51>(config);
    case 53: return std::make_unique<Mapper53>(config);
    case 54: case 201: return std::make_unique<Mapper54>(config);
    case 55: return std::make_unique<Mapper55>(config);
    case 56: return std::make_unique<Mapper56>(config);
    case 57: return std::make_unique<Mapper57>(config);
    case 58: return std::make_unique<Mapper58>(config);
    case 59: return std::make_unique<Mapper59>(config);
    case 60: return std::make_unique<Mapper60>(config);
    case 61: return std::make_unique<Mapper61>(config);
    case 62: return std::make_unique<Mapper62>(config);
    case 63: return std::make_unique<Mapper63>(config);
    case 64: case 158: return std::make_unique<Mapper64>(config);
    case 65: return std::make_unique<Mapper65>(config);
    case 66: return std::make_unique<Mapper66>(config);
    case 67: return createSunsoftMapper(config);
    case 68: return createSunsoftMapper(config);
    case 69: return createSunsoftMapper(config);
    case 70: case 152: return std::make_unique<Mapper70>(config);
    case 71: return std::make_unique<Mapper71>(config);
    case 72: case 92: return std::make_unique<Mapper72>(config);
    case 73: return createVrcMapper(config);
    case 75: case 151: return createVrcMapper(config);
    case 76: return std::make_unique<Mapper76>(config);
    case 77: return std::make_unique<Mapper77>(config);
    case 78: return std::make_unique<Mapper78>(config);
    case 79: case 146: return std::make_unique<Mapper79>(config);
    case 113: return std::make_unique<Mapper113>(config);
    case 80: case 207: return std::make_unique<Mapper80>(config);
    case 81: return std::make_unique<Mapper81>(config);
    case 82: return std::make_unique<Mapper82>(config);
    case 83: return createUnlicensedMapper(config);
    case 109: { MapperConfig compat = config; compat.id = 137; return std::make_unique<Mapper137>(compat); }
    case 130: case 331: return std::make_unique<Mapper331>(config);
    case 110: { MapperConfig compat = config; compat.id = 243; return std::make_unique<Mapper243>(compat); }
    case 85: return createVrcMapper(config);
    case 86: return std::make_unique<Mapper86>(config);
    case 87: return std::make_unique<Mapper87>(config);
    case 88: case 154: return std::make_unique<Mapper88>(config);
    case 89: return createSunsoftMapper(config);
    case 90: case 209: case 211: return createUnlicensedMapper(config);
    case 91: return std::make_unique<Mapper91>(config);
    case 93: return createSunsoftMapper(config);
    case 94: return std::make_unique<Mapper94>(config);
    case 95: return std::make_unique<Mapper95>(config);
    case 96: return std::make_unique<Mapper96>(config);
    case 97: return std::make_unique<Mapper97>(config);
    case 101: return std::make_unique<Mapper101>(config);
    case 103: return std::make_unique<Mapper103>(config);
    case 104: return std::make_unique<Mapper104>(config);
    case 105: return std::make_unique<Mapper105>(config);
    case 106: return std::make_unique<Mapper106>(config);
    case 107: return std::make_unique<Mapper107>(config);
    case 108: return std::make_unique<Mapper108>(config);
    case 111: if (config.chrRomSize) return std::make_unique<Mapper111Mmc1>(config); else return std::make_unique<Mapper111>(config);
    case 112: return std::make_unique<Mapper112>(config);
    case 120: return std::make_unique<Mapper120>(config);
    case 129: { MapperConfig compat=config; compat.id=58; return std::make_unique<Mapper58>(compat); }
    case 131: { MapperConfig compat=config; compat.id=205; return createMmc3Mapper(compat); }

    case 122: case 184: return createSunsoftMapper(config);
    case 125: return std::make_unique<Mapper125>(config);
    case 128: return std::make_unique<Mapper128>(config);
    case 132: return std::make_unique<Mapper132>(config);
    case 133: return std::make_unique<Mapper133>(config);
    case 136: return std::make_unique<Mapper136>(config);
    case 137: return std::make_unique<Mapper137>(config);
    case 135: case 138: case 139: case 141: return std::make_unique<Mapper8259>(config);
    case 140: return std::make_unique<Mapper140>(config);
    case 142: return std::make_unique<Mapper142>(config);
    case 143: return std::make_unique<Mapper143>(config);
    case 145: return std::make_unique<Mapper145>(config);
    case 147: return std::make_unique<Mapper147>(config);
    case 148: return std::make_unique<Mapper148>(config);
    case 149: return std::make_unique<Mapper149>(config);
    case 150: return std::make_unique<Mapper150>(config);
    case 156: return std::make_unique<Mapper156>(config);
    case 160: { MapperConfig compat = config; compat.id = 90; return createUnlicensedMapper(compat); }
    case 161: { MapperConfig compat = config; compat.id = 1; return createMmc1Mapper(compat); }
    case 162: return std::make_unique<Mapper162>(config);
    case 163: return std::make_unique<Mapper163>(config);
    case 164: return std::make_unique<Mapper164>(config);
    case 165: return createMmc3Mapper(config);
    case 166: case 167: return std::make_unique<Mapper166_167>(config);
    case 168: return std::make_unique<Mapper168>(config);
    case 169: return std::make_unique<Mapper169>(config);
    case 170: return std::make_unique<Mapper170>(config);
    case 172: return std::make_unique<Mapper172>(config);
    case 171: return std::make_unique<Mapper171>(config);
    case 174: return std::make_unique<Mapper174>(config);
    case 175: return std::make_unique<Mapper175>(config);
    case 176: case 179: return createUnlicensedMapper(config);
    case 177: return std::make_unique<Mapper177>(config);
    case 173: return std::make_unique<Mapper173>(config);
    case 178: return std::make_unique<Mapper178>(config);
    case 180: return std::make_unique<Mapper180>(config);
    case 181: { MapperConfig compat=config; compat.id=185; return std::make_unique<Mapper185>(compat); }
    case 183: return std::make_unique<Mapper183>(config);
    case 185: return std::make_unique<Mapper185>(config);
    case 186: return std::make_unique<Mapper186>(config);
    case 187: return createMmc3Mapper(config);
    case 188: return std::make_unique<Mapper188>(config);
    case 189: return createMmc3Mapper(config);
    case 190: return std::make_unique<Mapper190>(config);
    case 193: return std::make_unique<Mapper193>(config);
    case 196: return createMmc3Mapper(config);
    case 126: case 422: case 534: return createMmc3Mapper(config);
    case 134: return createMmc3Mapper(config);
    case 198: case 199: return createMmc3Mapper(config);
    case 200: return std::make_unique<Mapper200>(config);
    case 202: case 203: case 204: case 214: case 217: case 229: case 231:
        return std::make_unique<MapperBmcBundled47>(config);
    case 205: return createMmc3Mapper(config);
    case 206: return std::make_unique<Mapper206>(config);
    case 208: return createMmc3Mapper(config);
    case 212: return std::make_unique<Mapper212>(config);
    case 213: return std::make_unique<Mapper213>(config);
    case 216: return std::make_unique<Mapper216>(config);
    case 218: return std::make_unique<Mapper218>(config);
    case 219: return createMmc3Mapper(config);
    case 223: { MapperConfig compat = config; compat.id = 199; return createMmc3Mapper(compat); }
    case 221: return std::make_unique<Mapper221>(config);
    case 222: return std::make_unique<Mapper222>(config);
    case 224: return createMmc3Mapper(config);
    case 225: case 255: case 226: case 227: case 230: case 235: case 236: case 237: return std::make_unique<MapperBmcBundled48>(config);
    case 228: return std::make_unique<Mapper228>(config);
    case 232: return std::make_unique<Mapper232>(config);
    case 233: return std::make_unique<Mapper233>(config);
    case 234: return std::make_unique<Mapper234>(config);
    case 238: return createMmc3Mapper(config);
    case 240: return std::make_unique<Mapper240>(config);
    case 241: return std::make_unique<Mapper241>(config);
    case 242: return std::make_unique<Mapper242>(config);
    case 243: return std::make_unique<Mapper243>(config);
    case 244: return std::make_unique<Mapper244>(config);
    case 245: return createMmc3Mapper(config);
    case 248: { MapperConfig compat = config; compat.id = 115; return createMmc3Mapper(compat); }
    case 249: case 250: return createMmc3Mapper(config);
    case 251: { MapperConfig compat = config; compat.id = 45; return createMmc3Mapper(compat); }
    case 252: return std::make_unique<Mapper252>(config);
    case 253: return std::make_unique<Mapper253>(config);
    case 254: return createMmc3Mapper(config);
    case 259: case 263: return createMmc3Mapper(config);
    case 265: return std::make_unique<Mapper265>(config);
    case 264: return createUnlicensedMapper(config);
    case 268: return createUnlicensedMapper(config);
    case 246: return std::make_unique<Mapper246>(config);
    default: return std::make_unique<MapperFallback>(config);
    }
}

bool mapperImplementationSupported(uint16_t mapper, uint8_t submapper)
{

    switch (mapper) {
    case 29: return submapper == 0;
    case 1: return submapper == 0 || submapper == 5;
    case 155: return submapper == 0 || submapper == 5;
    case 2: case 3: case 7: return submapper <= 2;
    case 6: return submapper <= 7;
    case 8: return submapper == 0 || submapper == 4;
    case 12: return submapper <= 1;
    case 14: return submapper == 0;
    case 17: return submapper <= 3;
    case 30: return submapper <= 4;
    case 32: return submapper <= 1;
    case 40: return submapper <= 1;
    case 63: return submapper <= 1;
    case 68: return submapper <= 1;
    case 34: return submapper <= 2;
    case 71: return submapper <= 1;
    case 78: return submapper == 0 || submapper == 1 || submapper == 3;
    case 83: return submapper <= 3;
    case 85: return submapper <= 2;
    case 91: return submapper <= 1;
    case 99: return submapper == 0;
    case 108: return submapper <= 4;
    case 113: return submapper == 0;
    case 109: case 110: case 160: case 161: case 162: case 163: case 164: case 165: case 166: case 167: case 168: case 169: case 170: case 172: case 175: case 177: case 183: case 187: case 233: return submapper == 0;
    case 114: return submapper <= 1;
    case 182: return submapper <= 1;
    case 116: return submapper <= 3;
    case 122: case 184: return submapper == 0;
    case 125: return submapper == 0;
    case 126: case 422: case 534: return submapper == 0;
    case 128: case 129: case 130: case 131: case 331: return submapper == 0;
    case 132: return submapper == 0;
    case 133: return submapper == 0;
    case 134: return submapper == 0;
    case 136: return submapper == 0;
    case 137: return submapper == 0;
    case 135: case 138: case 139: case 141: return submapper == 0;
    case 142: return submapper == 0;
    case 143: return submapper == 0;
    case 144: return submapper == 0;
    case 147: return submapper == 0;
    case 148: return submapper == 0;
    case 149: return submapper == 0;
    case 150: return submapper == 0;
    case 152: return submapper == 0;
    case 153: return submapper == 0;
    case 243: return submapper == 0;
    case 245: return submapper == 0;
    case 249: case 250: return submapper == 0;
    case 156: return submapper == 0;
    case 171: return submapper == 0;
    case 173: case 178: case 181: case 186: case 188: case 189: case 190: case 196: return submapper == 0;
    case 174: return submapper == 0;
    case 176: case 179: return submapper <= 5;
    case 185: return submapper == 0 || (submapper >= 4 && submapper <= 7);
    case 193: return submapper == 0;
    case 198: case 199: return submapper == 0;
    case 200: return submapper == 0;
    case 201: return submapper == 0;
    case 202: case 203: case 204: case 214: case 217: case 229: case 231: return submapper == 0;
    case 225: case 255: case 226: case 227: case 230: case 235: case 236: case 237: return submapper == 0;
    case 197: return submapper == 0 || submapper == 1 || submapper == 3;
    case 205: return submapper == 0;
    case 206: return submapper <= 1;
    case 208: return submapper <= 1;
    case 210: return submapper <= 2;
    case 212: return submapper == 0;
    case 213: return submapper == 0;
    case 216: case 218: case 219: case 221: case 222: case 223: case 224: case 234: case 238: case 244: case 248: case 251: case 252: case 253: case 254: return submapper == 0;
    case 215: return submapper <= 1;
    case 259: case 263: case 264: case 265: return submapper == 0;
    case 268: return submapper <= 11;
    case 487: return submapper == 0;
    default: break;
    }

    switch (mapper) {
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10: case 11: case 12: case 13: case 15:
    case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28:
    case 30: case 31: case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39: case 40: case 41: case 42: case 43: case 44: case 45: case 46: case 47: case 48: case 49: case 50: case 51: case 52: case 53: case 54: case 55: case 56: case 57: case 58: case 59: case 60: case 61: case 62: case 63: case 64: case 65: case 66: case 67:
    case 68: case 69: case 70: case 71: case 72: case 73: case 74: case 75: case 76:
    case 77: case 78: case 79: case 80: case 81: case 82: case 85: case 86: case 87: case 88:
    case 89: case 90: case 92: case 93: case 94: case 95: case 96: case 97: case 101: case 103: case 104: case 105: case 106: case 107: case 108: case 111: case 112: case 113: case 114: case 115: case 117: case 120: case 121: case 123:
    case 118: case 119: case 125: case 133: case 135: case 136: case 137: case 138: case 139: case 140: case 141: case 143: case 144: case 145: case 146: case 147: case 148: case 149: case 151: case 152: case 153:
    case 154: case 155: case 157: case 158: case 159: case 174: case 176: case 180: case 184: case 185: case 186: case 191:
    case 189: case 192: case 193: case 194: case 195: case 196: case 197: case 200: case 205: case 206: case 207: case 209: case 211: case 213: case 215: case 228: case 232: case 240: case 241:
    case 242: case 243: case 246: case 249: case 250: case 268:
        return true;
    default:
        return false;
    }
}
