#include "MapperFamilies.hpp"
#include <algorithm>

namespace {
void put8(std::vector<uint8_t>& out, uint8_t value) { out.push_back(value); }
bool get8(const uint8_t*& p, const uint8_t* end, uint8_t& value) {
    if (p >= end) return false;
    value = *p++;
    return true;
}
uint32_t mapBank(std::size_t bank, std::size_t bankSize, std::size_t totalSize, uint32_t inBank) {
    const std::size_t count = std::max<std::size_t>(1, totalSize / bankSize);
    return static_cast<uint32_t>((bank % count) * bankSize + inBank);
}

class Mapper0 final : public Mapper {
public:
    explicit Mapper0(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>(m_config.prgRomSize == 0x4000
            ? (addr & 0x3FFF)
            : ((addr - 0x8000) % m_config.prgRomSize));
        return true;
    }
};

class Mapper2 final : public Mapper {
public:
    explicit Mapper2(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize < 0x4000) return false;
        if (addr < 0xC000)
            mapped = mapBank(m_bank, 0x4000, m_config.prgRomSize, addr - 0x8000);
        else
            mapped = static_cast<uint32_t>(m_config.prgRomSize - 0x4000 + (addr - 0xC000));
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_bank = data & 0x0F;
        return true;
    }

    bool hasBusConflicts() const override
    {

        return !(m_config.nes20 && m_config.submapper == 1);
    }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_bank); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_bank); }

private:
    uint8_t m_bank = 0;
};

class Mapper3 final : public Mapper {
public:
    explicit Mapper3(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>(m_config.prgRomSize == 0x4000
            ? (addr & 0x3FFF)
            : ((addr - 0x8000) % m_config.prgRomSize));
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_bank = data & 0x03;
        return true;
    }

    bool hasBusConflicts() const override
    {

        return !(m_config.nes20 && m_config.submapper == 1);
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        mapped = mapBank(m_bank, 0x2000, chrSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_bank); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_bank); }

private:
    uint8_t m_bank = 0;
};

class Mapper99 final : public Mapper {
public:
    explicit Mapper99(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = static_cast<uint32_t>(m_config.prgRomSize == 0x4000
            ? (addr & 0x3FFF)
            : ((addr - 0x8000) % m_config.prgRomSize));
        return true;
    }

    void observeCpuWrite(uint16_t addr, uint8_t data) override
    {
        if (addr == 0x4016)
            m_chrBank = static_cast<uint8_t>((data >> 2) & 0x01);
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;
        mapped = mapBank(m_chrBank, 0x2000, chrSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_chrBank); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_chrBank); }

private:
    uint8_t m_chrBank = 0;
};
class Mapper7 final : public Mapper {
public:
    explicit Mapper7(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = mapBank(m_bank & 0x07, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_bank = data;
        m_mirror = (data & 0x10) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
        return true;
    }

    bool hasBusConflicts() const override
    {

        return m_config.nes20 && m_config.submapper == 2;
    }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_bank); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        if (!get8(p, end, m_bank)) return false;
        m_mirror = (m_bank & 0x10) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
        return true;
    }

private:
    uint8_t m_bank = 0;
};

}

std::unique_ptr<Mapper> createNintendoDiscreteMapper(const MapperConfig& config)
{
    switch (config.id) {
    case 0: return std::make_unique<Mapper0>(config);
    case 2: return std::make_unique<Mapper2>(config);
    case 3: return std::make_unique<Mapper3>(config);
    case 7: return std::make_unique<Mapper7>(config);
    case 99: return std::make_unique<Mapper99>(config);
    default: return nullptr;
    }
}
