#include "Mapper.hpp"
#include <algorithm>
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

class Mapper1 final : public Mapper {
public:
    explicit Mapper1(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;

        // SxROM boards with 512 KiB PRG-ROM (SUROM/SXROM) reuse CHR A16
        // as PRG A18.  This selects one 256 KiB PRG region; the normal
        // MMC1 PRG register then banks within that region.  The fixed bank
        // in modes 2/3 is fixed inside the selected region, not globally.
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
        if (mode == 0 || mode == 1) {
            // 32 KiB mode ignores PRG register bit 0.
            mapped = mapRegionBank((m_prg & 0x0E) >> 1, bank32, addr - 0x8000);
        }
        else if (mode == 2) {
            if (addr < 0xC000)
                mapped = static_cast<uint32_t>(regionBase + (addr - 0x8000));
            else
                mapped = mapRegionBank(m_prg & 0x0F, bank16, addr - 0xC000);
        }
        else {
            if (addr < 0xC000)
                mapped = mapRegionBank(m_prg & 0x0F, bank16, addr - 0x8000);
            else
                mapped = static_cast<uint32_t>(regionBase + regionSize - bank16 + (addr - 0xC000));
        }

        mapped %= static_cast<uint32_t>(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t cpuCycle) override
    {
        if (addr < 0x8000) return false;

        // MMC1 ignores the second write of a read-modify-write instruction.
        if (cpuCycle != kNoCpuCycle) {
            if (m_lastWriteCycle != kNoCpuCycle && cpuCycle <= m_lastWriteCycle + 1)
                return true;
            m_lastWriteCycle = cpuCycle;
        }

        if (data & 0x80) {
            m_shift = 0x10;
            m_ctrl |= 0x0C;
            updateMirror();
            return true;
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

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool /*write*/) const override
    {
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
            return false;

        // MMC1B/C PRG register bit 4 disables PRG-RAM on SxROM boards too.
        if (m_prg & 0x10)
            return false;

        // SOROM uses CHR A15 (CHR0 bit 3) to select one of two 8 KiB
        // WRAM banks. SXROM additionally uses CHR A14 (CHR0 bit 2), giving
        // four 8 KiB WRAM banks. Boards with only 8 KiB simply ignore them.
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

class Mapper4 final : public Mapper {
public:
    explicit Mapper4(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        const std::size_t count = std::max<std::size_t>(1, m_config.prgRomSize / 0x2000);
        const uint8_t r6 = m_regs[6] & 0x3F;
        const uint8_t r7 = m_regs[7] & 0x3F;
        const std::size_t last = count - 1;
        const std::size_t lastM1 = count > 1 ? count - 2 : 0;
        std::size_t bank = 0;
        uint32_t inBank = addr & 0x1FFF;

        if (!m_prgMode) {
            if (addr < 0xA000) bank = r6;
            else if (addr < 0xC000) bank = r7;
            else if (addr < 0xE000) bank = lastM1;
            else bank = last;
        }
        else {
            if (addr < 0xA000) bank = lastM1;
            else if (addr < 0xC000) bank = r7;
            else if (addr < 0xE000) bank = r6;
            else bank = last;
        }
        mapped = mapBank(bank, 0x2000, m_config.prgRomSize, inBank);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        switch (addr & 0xE001) {
        case 0x8000:
            m_bankSelect = data;
            m_prgMode = (data >> 6) & 1;
            m_chrMode = (data >> 7) & 1;
            break;
        case 0x8001:
            m_regs[m_bankSelect & 7] = data;
            break;
        case 0xA000:
            if (m_config.id != 118 && !m_config.fourScreen)
                m_mirror = (data & 1) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        case 0xA001:
            m_prgRamWriteProtect = (data & 0x40) != 0;
            m_prgRamEnable = (data & 0x80) != 0;
            break;
        case 0xC000: m_irqLatch = data; break;
        case 0xC001: m_irqReload = true; break;
        case 0xE000: m_irqEnabled = false; m_irqPending = false; break;
        case 0xE001: m_irqEnabled = true; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        const std::size_t bank = chrBankForAddress(addr);
        const bool useRam = chrBankUsesRam(bank);
        const std::size_t chrSize = useRam ? m_config.chrRamSize : m_config.chrRomSize;
        if (chrSize == 0) return false;
        std::size_t physicalBank = bank;
        if (useRam) physicalBank -= chrRamBankBase();
        mapped = mapBank(physicalBank, 0x400, chrSize, addr & 0x03FF);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override
    {
        if (!ppuUsesChrRam(addr)) return false;
        return ppuMapRead(addr, mapped);
    }

    bool ppuUsesChrRam(uint16_t addr) const override
    {
        if (addr >= 0x2000 || m_config.chrRamSize == 0) return false;
        return chrBankUsesRam(chrBankForAddress(addr));
    }

    bool mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const override
    {
        if (m_config.id != 118 || addr < 0x2000 || addr > 0x3EFF) return false;
        const uint8_t page = static_cast<uint8_t>((addr & 0x0FFF) >> 10);
        const uint8_t nt = !m_chrMode
            ? static_cast<uint8_t>((page < 2 ? m_regs[0] : m_regs[1]) >> 7)
            : static_cast<uint8_t>(m_regs[2 + page] >> 7);
        source = NametableSource::Ciram;
        mapped = uint32_t(nt) * 0x400 + (addr & 0x03FF);
        return true;
    }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool write) const override
    {
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
            return false;
        if (!m_prgRamEnable || (write && m_prgRamWriteProtect))
            return false;
        mapped = static_cast<uint32_t>((addr - 0x6000) % m_config.prgRamSize);
        return true;
    }

    void notifyPpuAddress(uint16_t addr, uint64_t ppuCycle) override
    {
        const bool a12High = (addr & 0x1000) != 0;
        if (!a12High) {
            if (m_lastA12 || !m_a12LowValid) {
                m_a12LowStart = ppuCycle;
                m_a12LowValid = true;
            }
        }
        else {
            if (!m_lastA12 && m_a12LowValid && ppuCycle - m_a12LowStart >= 8)
                clockIrq();
            m_a12LowValid = false;
        }
        m_lastA12 = a12High;
    }

    void scanlineTick() override { clockIrq(); }
    bool irqActive() const override { return m_irqPending; }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, static_cast<uint8_t>(m_mirror));
        put8(out, m_bankSelect);
        for (uint8_t value : m_regs) put8(out, value);
        put8(out, m_prgMode); put8(out, m_chrMode);
        put8(out, m_irqLatch); put8(out, m_irqCounter);
        put8(out, m_irqEnabled ? 1 : 0); put8(out, m_irqReload ? 1 : 0); put8(out, m_irqPending ? 1 : 0);
        put8(out, m_lastA12 ? 1 : 0); put8(out, m_a12LowValid ? 1 : 0); put64(out, m_a12LowStart);
        put8(out, m_prgRamEnable ? 1 : 0); put8(out, m_prgRamWriteProtect ? 1 : 0);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t tmp = 0;
        if (!get8(p, end, tmp)) return false;
        m_mirror = static_cast<Mirror>(tmp);
        if (!get8(p, end, m_bankSelect)) return false;
        for (uint8_t& value : m_regs) if (!get8(p, end, value)) return false;
        if (!get8(p, end, m_prgMode) || !get8(p, end, m_chrMode) || !get8(p, end, m_irqLatch) || !get8(p, end, m_irqCounter)) return false;
        if (!get8(p, end, tmp)) return false;
        m_irqEnabled = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_irqReload = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_irqPending = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_lastA12 = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_a12LowValid = tmp != 0;
        if (!get64(p, end, m_a12LowStart)) return false;
        if (!get8(p, end, tmp)) return false;
        m_prgRamEnable = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_prgRamWriteProtect = tmp != 0;
        return true;
    }

private:
    uint8_t m_bankSelect = 0;
    uint8_t m_regs[8] = {};
    uint8_t m_prgMode = 0;
    uint8_t m_chrMode = 0;
    uint8_t m_irqLatch = 0;
    uint8_t m_irqCounter = 0;
    bool m_irqEnabled = false;
    bool m_irqReload = false;
    bool m_irqPending = false;
    bool m_lastA12 = false;
    bool m_a12LowValid = false;
    uint64_t m_a12LowStart = 0;
    bool m_prgRamEnable = true;
    bool m_prgRamWriteProtect = false;

    std::size_t chrBankForAddress(uint16_t addr) const
    {
        const uint8_t r0 = m_regs[0] & 0xFE;
        const uint8_t r1 = m_regs[1] & 0xFE;
        if (!m_chrMode) {
            if (addr < 0x0800) return r0 + ((addr >> 10) & 1);
            if (addr < 0x1000) return r1 + ((addr >> 10) & 1);
            if (addr < 0x1400) return m_regs[2];
            if (addr < 0x1800) return m_regs[3];
            if (addr < 0x1C00) return m_regs[4];
            return m_regs[5];
        }
        if (addr < 0x0400) return m_regs[2];
        if (addr < 0x0800) return m_regs[3];
        if (addr < 0x0C00) return m_regs[4];
        if (addr < 0x1000) return m_regs[5];
        if (addr < 0x1800) return r0 + ((addr >> 10) & 1);
        return r1 + ((addr >> 10) & 1);
    }

    bool chrBankUsesRam(std::size_t bank) const
    {
        if (!m_config.chrRamSize) return false;
        if (!m_config.chrRomSize) return true;
        switch (m_config.id) {
        case 74: return bank >= 0x08 && bank <= 0x09;
        case 119: return bank >= 0x40 && bank <= 0x7F;
        case 191: return bank >= 0x80 && bank <= 0xFF;
        case 192: return bank >= 0x08 && bank <= 0x0B;
        case 194: return bank <= 0x01;
        case 195: return bank <= 0x03;
        default: return false;
        }
    }

    std::size_t chrRamBankBase() const
    {
        switch (m_config.id) {
        case 74: return 0x08;
        case 119: return 0x40;
        case 191: return 0x80;
        case 192: return 0x08;
        default: return 0;
        }
    }

    void clockIrq()
    {
        if (m_irqCounter == 0 || m_irqReload) {
            m_irqCounter = m_irqLatch;
            m_irqReload = false;
        }
        else {
            --m_irqCounter;
        }
        if (m_irqCounter == 0 && m_irqEnabled)
            m_irqPending = true;
    }
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
    // MMC2 powers up selecting the FE banks. The left latch responds only to
    // the exact $0FD8/$0FE8 addresses; the right latch responds to the eight-
    // byte $1FD8-$1FDF/$1FE8-$1FEF ranges.
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
    // MMC4 powers up selecting the FE banks. Unlike MMC2, both pattern-table
    // latches respond to eight-address ranges.
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

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_chr); }

private:
    uint8_t m_chr = 0;
};

class Mapper30 final : public Mapper {
public:
    explicit Mapper30(const MapperConfig& config)
        : Mapper(config),
          m_mapperControlsMirror(config.headerMirror == Mirror::OnescreenLo || config.headerMirror == Mirror::OnescreenHi)
    {
        updateMirror();
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

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        if (addr < 0x8000) return false;
        m_bank = data;
        updateMirror();
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000 || m_config.chrRamSize == 0) return false;
        mapped = mapBank((m_bank >> 5) & 0x03, 0x2000, m_config.chrRamSize, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override { return ppuMapRead(addr, mapped); }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_bank); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        if (!get8(p, end, m_bank)) return false;
        updateMirror();
        return true;
    }

private:
    uint8_t m_bank = 0;
    bool m_mapperControlsMirror = false;

    void updateMirror()
    {
        if (m_mapperControlsMirror)
            m_mirror = (m_bank & 0x80) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
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
            // BF9097/Fire Hawk mirroring register. Other Camerica boards do not
            // normally write here, so accepting it also keeps legacy iNES dumps usable.
            m_mirror = (data & 0x10) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
            return true;
        }
        if (addr >= 0xC000) {
            m_prg = data;
            return true;
        }
        return false;
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override
    {
        put8(out, m_prg); put8(out, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override
    {
        uint8_t mirror = 0;
        if (!get8(p, end, m_prg) || !get8(p, end, mirror)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    uint8_t m_prg = 0;
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
        m_prg = (data >> 4) & 3;
        m_chr = data & 0x0F;
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

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_prg); put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_prg) && get8(p, end, m_chr); }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};

#include "MapperMore.inc"
#include "MapperHard.inc"
#include "MapperFds.inc"

} // namespace

Mapper::Mapper(const MapperConfig& config)
    : m_config(config), m_mirror(config.headerMirror)
{
}

bool Mapper::cpuReadRegister(uint16_t, uint8_t&) { return false; }
void Mapper::observeCpuRead(uint16_t, uint8_t) {}
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
uint8_t Mapper::readMapperNametable(uint32_t) const { return 0; }
void Mapper::writeMapperNametable(uint32_t, uint8_t) {}

bool Mapper::mapPrgRam(uint16_t addr, uint32_t& mapped, bool) const
{
    if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
        return false;
    mapped = static_cast<uint32_t>((addr - 0x6000) % m_config.prgRamSize);
    return true;
}

void Mapper::notifyPpuAddress(uint16_t, uint64_t) {}
void Mapper::notifyPpuAddressContext(uint16_t addr, uint64_t ppuCycle, int, int) { notifyPpuAddress(addr, ppuCycle); }
void Mapper::notifyPpuScanline(int, bool) {}
void Mapper::clockCpu() {}
void Mapper::scanlineTick() {}
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

std::unique_ptr<Mapper> createMapper(const MapperConfig& config)
{
    switch (config.id) {
    case 0: return std::make_unique<Mapper0>(config);
    case 1: case 155: return std::make_unique<Mapper1>(config);
    case 2: return std::make_unique<Mapper2>(config);
    case 3: return std::make_unique<Mapper3>(config);
    case 4: case 74: case 118: case 119: case 191: case 192: case 194: case 195:
        return std::make_unique<Mapper4>(config);
    case 5: return std::make_unique<Mapper5>(config);
    case 7: return std::make_unique<Mapper7>(config);
    case 9: return std::make_unique<Mapper9>(config);
    case 10: return std::make_unique<Mapper10>(config);
    case 11: case 144: return std::make_unique<Mapper11>(config);
    case 13: return std::make_unique<Mapper13>(config);
    case 16: case 153: case 157: case 159: return std::make_unique<Mapper16>(config);
    case 18: return std::make_unique<Mapper18>(config);
    case 19: case 210: return std::make_unique<Mapper19>(config);
    case 20: return std::make_unique<Mapper20>(config);
    case 21: case 22: case 23: case 25: case 27: return std::make_unique<MapperVrc24>(config);
    case 24: case 26: return std::make_unique<MapperVrc6>(config);
    case 30: return std::make_unique<Mapper30>(config);
    case 32: return std::make_unique<Mapper32>(config);
    case 33: return std::make_unique<Mapper33>(config);
    case 34: return std::make_unique<Mapper34>(config);
    case 48: return std::make_unique<Mapper48>(config);
    case 64: case 158: return std::make_unique<Mapper64>(config);
    case 65: return std::make_unique<Mapper65>(config);
    case 66: return std::make_unique<Mapper66>(config);
    case 67: return std::make_unique<Mapper67>(config);
    case 68: return std::make_unique<Mapper68>(config);
    case 69: return std::make_unique<Mapper69>(config);
    case 70: case 152: return std::make_unique<Mapper70>(config);
    case 71: return std::make_unique<Mapper71>(config);
    case 72: case 92: return std::make_unique<Mapper72>(config);
    case 73: return std::make_unique<Mapper73>(config);
    case 75: case 151: return std::make_unique<Mapper75>(config);
    case 76: return std::make_unique<Mapper76>(config);
    case 77: return std::make_unique<Mapper77>(config);
    case 78: return std::make_unique<Mapper78>(config);
    case 79: case 113: case 146: return std::make_unique<Mapper79>(config);
    case 80: case 207: return std::make_unique<Mapper80>(config);
    case 82: return std::make_unique<Mapper82>(config);
    case 85: return std::make_unique<Mapper85>(config);
    case 86: return std::make_unique<Mapper86>(config);
    case 87: return std::make_unique<Mapper87>(config);
    case 88: case 154: return std::make_unique<Mapper88>(config);
    case 89: return std::make_unique<Mapper89>(config);
    case 93: return std::make_unique<Mapper93>(config);
    case 94: return std::make_unique<Mapper94>(config);
    case 95: return std::make_unique<Mapper95>(config);
    case 96: return std::make_unique<Mapper96>(config);
    case 97: return std::make_unique<Mapper97>(config);
    case 101: return std::make_unique<Mapper101>(config);
    case 140: return std::make_unique<Mapper140>(config);
    case 180: return std::make_unique<Mapper180>(config);
    case 184: return std::make_unique<Mapper184>(config);
    case 206: return std::make_unique<Mapper206>(config);
    case 232: return std::make_unique<Mapper232>(config);
    case 240: return std::make_unique<Mapper240>(config);
    case 241: return std::make_unique<Mapper241>(config);
    case 242: return std::make_unique<Mapper242>(config);
    case 246: return std::make_unique<Mapper246>(config);
    default: return std::make_unique<MapperFallback>(config);
    }
}

bool mapperImplementationSupported(uint16_t mapper, uint8_t submapper)
{
    if (mapper == 210)
        return submapper == 1 || submapper == 2;

    switch (mapper) {
    case 0: case 1: case 2: case 3: case 4: case 5: case 7: case 9: case 10: case 11: case 13:
    case 16: case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27:
    case 30: case 32: case 33: case 34: case 48: case 64: case 65: case 66: case 67:
    case 68: case 69: case 70: case 71: case 72: case 73: case 74: case 75: case 76:
    case 77: case 78: case 79: case 80: case 82: case 85: case 86: case 87: case 88:
    case 89: case 92: case 93: case 94: case 95: case 96: case 97: case 101: case 113:
    case 118: case 119: case 140: case 144: case 146: case 151: case 152: case 153:
    case 154: case 155: case 157: case 158: case 159: case 180: case 184: case 191:
    case 192: case 194: case 195: case 206: case 207: case 232: case 240: case 241:
    case 242: case 246:
        return true;
    default:
        return false;
    }
}
