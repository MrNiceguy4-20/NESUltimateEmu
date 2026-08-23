#include "Mapper.hpp"
#include "MapperFamilies.hpp"
#include <algorithm>

namespace {
constexpr uint64_t kNoCpuCycle = ~uint64_t(0);
inline void put8(std::vector<uint8_t>& out, uint8_t value) { out.push_back(value); }
inline void put64(std::vector<uint8_t>& out, uint64_t value) { for (int i=0;i<8;++i) put8(out, static_cast<uint8_t>(value >> (i*8))); }
inline bool get8(const uint8_t*& p, const uint8_t* end, uint8_t& value) { if (p>=end) return false; value=*p++; return true; }
inline bool get64(const uint8_t*& p, const uint8_t* end, uint64_t& value) { if (end-p<8) return false; value=0; for(int i=0;i<8;++i) value |= uint64_t(*p++) << (i*8); return true; }
inline uint32_t mapBank(std::size_t bank, std::size_t bankSize, std::size_t totalSize, uint32_t inBank) { const std::size_t count=std::max<std::size_t>(1,totalSize/bankSize); return static_cast<uint32_t>((bank%count)*bankSize+inBank); }
class Mapper1 final : public Mapper {
public:
    explicit Mapper1(const MapperConfig& config) : Mapper(config) {}

    bool implementationSupported() const override
    {

        return !m_config.nes20 || m_config.submapper == 0 || m_config.submapper == 5;
    }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;

        if (m_config.nes20 && m_config.submapper == 5) {
            mapped = static_cast<uint32_t>((addr - 0x8000) % m_config.prgRomSize);
            return true;
        }

        constexpr std::size_t bank16 = 0x4000;
        constexpr std::size_t bank32 = 0x8000;
        constexpr std::size_t region256 = 0x40000;

        std::size_t regionBase = 0;
        if (m_config.prgRomSize > region256 && (m_chr0 & 0x10))
            regionBase = region256;
        if (regionBase >= m_config.prgRomSize)
            regionBase = 0;

        const std::size_t regionSize = std::min(region256, m_config.prgRomSize - regionBase);
        const auto mapRegionBank = [&](std::size_t bank, std::size_t bankSize, uint32_t inBank) {
            return static_cast<uint32_t>(regionBase + mapBank(bank, bankSize, regionSize, inBank));
        };

        const uint8_t mode = (m_ctrl >> 2) & 3;
        const uint8_t prgReg = (m_config.id == 116 && m_config.nes20 && m_config.submapper == 2)
            ? uint8_t((m_prg << 1) & 0x1F) : m_prg;

        const bool mmc1aHalfSelect = m_config.id == 155 && (m_prg & 0x10) != 0 && regionSize > 0x20000;
        const std::size_t mmc1aHalfBase = mmc1aHalfSelect ? ((m_prg & 0x08) ? 0x20000 : 0) : 0;
        const std::size_t mmc1aHalfSize = mmc1aHalfSelect ? std::min<std::size_t>(0x20000, regionSize - mmc1aHalfBase) : regionSize;

        if (mode == 0 || mode == 1) {

            mapped = mapRegionBank((prgReg & 0x0E) >> 1, bank32, addr - 0x8000);
        }
        else if (mode == 2) {
            if (addr < 0xC000) {
                mapped = static_cast<uint32_t>(regionBase + mmc1aHalfBase + (addr - 0x8000));
            }
            else if (mmc1aHalfSelect) {
                mapped = static_cast<uint32_t>(regionBase + mmc1aHalfBase +
                    mapBank(prgReg & 0x07, bank16, mmc1aHalfSize, addr - 0xC000));
            }
            else {
                mapped = mapRegionBank(prgReg & 0x0F, bank16, addr - 0xC000);
            }
        }
        else {
            if (addr < 0xC000) {
                if (mmc1aHalfSelect)
                    mapped = static_cast<uint32_t>(regionBase + mmc1aHalfBase +
                        mapBank(prgReg & 0x07, bank16, mmc1aHalfSize, addr - 0x8000));
                else
                    mapped = mapRegionBank(prgReg & 0x0F, bank16, addr - 0x8000);
            }
            else {
                mapped = static_cast<uint32_t>(regionBase + mmc1aHalfBase + mmc1aHalfSize - bank16 + (addr - 0xC000));
            }
        }

        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t cpuCycle) override
    {
        if (addr < 0x8000) return false;

        if (data & 0x80) {
            if (cpuCycle != kNoCpuCycle)
                m_lastWriteCycle = cpuCycle;
            m_shift = 0x10;
            m_ctrl |= 0x0C;
            updateMirror();
            return true;
        }

        if (cpuCycle != kNoCpuCycle) {
            if (m_lastWriteCycle != kNoCpuCycle && cpuCycle <= m_lastWriteCycle + 1)
                return true;
            m_lastWriteCycle = cpuCycle;
        }

        const bool complete = (m_shift & 1) != 0;
        m_shift = (m_shift >> 1) | ((data & 1) << 4);
        if (!complete) return true;

        const uint8_t value = m_shift & 0x1F;
        m_shift = 0x10;
        switch ((addr >> 13) & 3) {
        case 0: m_ctrl = value; updateMirror(); break;
        case 1: m_chr0 = value; break;
        case 2: m_chr1 = value; break;
        case 3: m_prg = value; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t chrSize = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (chrSize == 0) return false;

        if ((m_ctrl & 0x10) == 0) {
            const uint8_t bank = m_chr0 & 0x1E;
            mapped = mapBank(bank, 0x1000, chrSize, addr);
        }
        else if (addr < 0x1000) {
            mapped = mapBank(m_chr0, 0x1000, chrSize, addr);
        }
        else {
            mapped = mapBank(m_chr1, 0x1000, chrSize, addr - 0x1000);
        }
        mapped %= static_cast<uint32_t>(chrSize);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (m_config.chrRamSize == 0) return false;
        return ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool  ) const override
    {
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
            return false;

        if (m_config.id != 155 && (m_prg & 0x10))
            return false;

        std::size_t ramBank = 0;
        if (m_config.prgRamSize >= 0x8000)
            ramBank = (m_chr0 >> 2) & 0x03;
        else if (m_config.prgRamSize >= 0x4000)
            ramBank = (m_chr0 >> 3) & 0x01;

        mapped = mapBank(ramBank, 0x2000, m_config.prgRamSize, addr - 0x6000);
        return true;
    }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, m_shift); put8(out, m_ctrl); put8(out, m_chr0); put8(out, m_chr1); put8(out, m_prg);
        put64(out, m_lastWriteCycle);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        if (!get8(p, end, m_shift) || !get8(p, end, m_ctrl) || !get8(p, end, m_chr0) ||
            !get8(p, end, m_chr1) || !get8(p, end, m_prg) || !get64(p, end, m_lastWriteCycle))
            return false;
        updateMirror();
        return true;
    }

private:
    uint8_t m_shift = 0x10;
    uint8_t m_ctrl = 0x0C;
    uint8_t m_chr0 = 0;
    uint8_t m_chr1 = 0;
    uint8_t m_prg = 0;
    uint64_t m_lastWriteCycle = kNoCpuCycle;

    void updateMirror()
    {
        switch (m_ctrl & 3) {
        case 0: m_mirror = Mirror::OnescreenLo; break;
        case 1: m_mirror = Mirror::OnescreenHi; break;
        case 2: m_mirror = Mirror::Vertical; break;
        case 3: m_mirror = Mirror::Horizontal; break;
        }
    }
};

}

std::unique_ptr<Mapper> createMmc1Mapper(const MapperConfig& config) { return std::make_unique<Mapper1>(config); }
