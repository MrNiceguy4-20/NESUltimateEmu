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

    bool implementationSupported() const override
    {
        // NES 2.0 submapper 5 is SEROM/SHROM/SH1ROM: 32 KiB PRG-ROM is
        // physically unbanked. Submappers 6 (2ME EEPROM) and 7 (KS-7058)
        // require additional board-specific hardware not modeled here.
        return !m_config.nes20 || m_config.submapper == 0 || m_config.submapper == 5;
    }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;

        // SEROM/SHROM/SH1ROM (NES 2.0 submapper 5) wires 32 KiB PRG-ROM
        // directly into $8000-$FFFF. MMC1 PRG mode/bank writes therefore do
        // not alter the CPU mapping on these boards.
        if (m_config.nes20 && m_config.submapper == 5) {
            mapped = static_cast<uint32_t>((addr - 0x8000) % m_config.prgRomSize);
            return true;
        }

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
        const uint8_t prgReg = (m_config.id == 116 && m_config.nes20 && m_config.submapper == 2)
            ? uint8_t((m_prg << 1) & 0x1F) : m_prg;

        // Mapper 155 identifies the MMC1A revision. Unlike MMC1B/C, PRG
        // register bit 4 is not a WRAM-disable bit. When it is set, PRG bit 3
        // bypasses the fixed-bank logic and drives PRG A17 across the entire
        // CPU ROM window. In 16 KiB modes this means the fixed bank is the
        // first/last bank of the selected 128 KiB half rather than the
        // first/last bank of the whole 256 KiB MMC1 region.
        const bool mmc1aHalfSelect = m_config.id == 155 && (m_prg & 0x10) != 0 && regionSize > 0x20000;
        const std::size_t mmc1aHalfBase = mmc1aHalfSelect ? ((m_prg & 0x08) ? 0x20000 : 0) : 0;
        const std::size_t mmc1aHalfSize = mmc1aHalfSelect ? std::min<std::size_t>(0x20000, regionSize - mmc1aHalfBase) : regionSize;

        if (mode == 0 || mode == 1) {
            // 32 KiB mode ignores PRG register bit 0.
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

        // MMC1 ignores the data-bit write on the second cycle of an RMW, but
        // D7 reset is asynchronous to that serial-write lockout and must never
        // be suppressed. A reset still occupies this CPU write cycle, so a
        // normal serial write on the immediately following cycle is ignored.
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

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool /*write*/) const override
    {
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
            return false;

        // MMC1B/C use PRG-register bit 4 as PRG-RAM disable. MMC1A
        // (iNES Mapper 155) predates that function and leaves WRAM enabled.
        if (m_config.id != 155 && (m_prg & 0x10))
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

    bool hasBusConflicts() const override
    {
        // UxROM normally exposes PRG ROM during mapper-register writes, so the
        // effective latch value is CPU data AND ROM data. NES 2.0 submapper 1
        // explicitly disables conflicts and submapper 2 explicitly enables
        // them. Submapper 0/legacy keeps the strict hardware-compatible default.
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
        // CNROM uses the same discrete bus-conflict convention as UxROM:
        // legacy/submapper 0 and explicit submapper 2 use wired-AND conflicts,
        // while NES 2.0 submapper 1 identifies conflict-free hardware.
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

class Mapper4 final : public Mapper {
public:
    explicit Mapper4(const MapperConfig& config) : Mapper(config), m_mmc6(config.nes20 && config.submapper == 1)
    {
        if (m_config.id == 121) {
            reset121();
        }
        if (m_config.id == 45) {
            m_m45Outer[2] = 0x0F;
            // GA23C multicarts are observed powering up with a useful linear
            // CHR arrangement; Famicom Yarou Vol. 1 writes CHR RAM before it
            // initializes MMC3 bank registers and depends on this mapping.
            m_regs[0] = 0; m_regs[1] = 2; m_regs[2] = 4;
            m_regs[3] = 5; m_regs[4] = 6; m_regs[5] = 7;
        }
    }

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

        // Mapper 123 / H2288 NROM override. $5800 bits are physically
        // scrambled: output bank bits 3,2,1,0 come from D5,D2,D4,D0.
        // D6 enables the override; D1 selects NROM-256 (CPU A14 supplies
        // bank bit 0) versus NROM-128 (the selected 16 KiB bank mirrored).
        if (m_config.id == 123 && (m_m123Mode & 0x40)) {
            std::size_t bank16 = std::size_t(m_m123Mode & 0x01) |
                std::size_t(m_m123Mode & 0x04) |
                (std::size_t(m_m123Mode & 0x10) >> 3) |
                (std::size_t(m_m123Mode & 0x20) >> 2);
            if (m_m123Mode & 0x02)
                bank16 = (bank16 & ~std::size_t(1)) | ((addr >> 14) & 1);
            const std::size_t bank8 = bank16 * 2 + ((addr >> 13) & 1);
            mapped = mapBank(bank8, 0x2000, m_config.prgRomSize, inBank);
            return true;
        }

        // Mapper 114/115 can override MMC3 PRG banking with an NROM-like
        // 16/32 KiB window selected by their $6000 mode register.
        if ((m_config.id == 114 || m_config.id == 115) && (m_extMode & 0x80)) {
            uint8_t bank16 = m_extMode & 0x0F;
            if (m_extMode & 0x20) bank16 = uint8_t((bank16 & 0x0E) | ((addr >> 14) & 1));
            const std::size_t bank8 = std::size_t(bank16) * 2 + ((addr >> 13) & 1);
            mapped = mapBank(bank8, 0x2000, m_config.prgRomSize, inBank);
            return true;
        }

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
        // Mapper 121 / Kǎshèng A9711/A9713 adds a protection-controlled
        // PRG override layer around the already-decoded MMC3 bank.
        if (m_config.id == 121) {
            const std::size_t outer = std::size_t((m_m121Ex[3] & 0x80) >> 2);
            if (m_m121Ex[5] & 0x3F) {
                if (addr >= 0xE000) bank = std::size_t(m_m121Ex[0]) | outer;
                else if (addr >= 0xC000) bank = std::size_t(m_m121Ex[1]) | outer;
                else if (addr >= 0xA000) bank = std::size_t(m_m121Ex[2]) | outer;
                else bank = (bank & 0x1F) | outer;
            } else {
                bank = (bank & 0x1F) | outer;
            }
            mapped = mapBank(bank, 0x2000, m_config.prgRomSize, inBank);
            return true;
        }
        if (m_config.id == 37) {
            const uint8_t q = m_outerReg & 0x07;
            const uint8_t inner = static_cast<uint8_t>(bank);
            const uint8_t bit4 = (q >> 2) & 1;
            const uint8_t bit3 = ((q & 0x03) == 0x03) ? 1 : (bit4 ? ((inner >> 3) & 1) : 0);
            bank = (std::size_t(bit4) << 4) | (std::size_t(bit3) << 3) | (inner & 0x07);
        } else if (m_config.id == 47) {
            bank = (std::size_t(m_outerReg & 1) << 4) | (bank & 0x0F);
        } else if (m_config.id == 44) {
            const uint8_t block = std::min<uint8_t>(m_outerReg & 7, 6);
            if (block == 6) bank = (bank & 0x1F) | 0x60;
            else bank = (bank & 0x0F) | (std::size_t(block) << 4);
        } else if (m_config.id == 49) {
            if (m_outerReg & 0x01) {
                bank = (std::size_t((m_outerReg >> 6) & 0x03) << 4) | (bank & 0x0F);
            } else {
                const std::size_t bank32 = (m_outerReg >> 4) & 0x0F;
                bank = bank32 * 4 + ((addr - 0x8000) >> 13);
                inBank = addr & 0x1FFF;
            }
        } else if (m_config.id == 52) {
            // Realtec 8213 outer banking. D2 supplies PRG A19, D1 supplies
            // A18, and D3 selects whether A17 comes from MMC3 or D0.
            const std::size_t lowMask = (m_outerReg & 0x08) ? 0x0F : 0x1F;
            const std::size_t a17 = (m_outerReg & 0x08) ? std::size_t(m_outerReg & 1) : ((bank >> 4) & 1);
            bank = (bank & lowMask) | (a17 << 4) | (std::size_t((m_outerReg >> 1) & 1) << 5) |
                (std::size_t((m_outerReg >> 2) & 1) << 6);
        } else if (m_config.id == 115) {
            // $6000 bit 6 supplies PRG A18 in normal MMC3 mode.
            bank = (std::size_t((m_extMode >> 6) & 1) << 5) | (bank & 0x1F);
        } else if (m_config.id == 197 && m_config.nes20 && m_config.submapper == 3 && (m_outerReg & 0x08)) {
            // Submapper 3 can force PRG A17 from bit 0, creating a 128 KiB window.
            bank = (std::size_t(m_outerReg & 1) << 4) | (bank & 0x0F);
        } else if (m_config.id == 45) {
            // GA23C outer PRG banking. Register #3 contains an inverted
            // six-bit AND mask for the MMC3 bank number. Register #1 ORs
            // PRG A13-A20, while register #2 bits 6-7 extend this to A21-A22.
            const std::size_t andMask = std::size_t(0x3F ^ (m_m45Outer[3] & 0x3F));
            const std::size_t outer = std::size_t(m_m45Outer[1]) |
                (std::size_t(m_m45Outer[2] & 0xC0) << 2);
            bank = (bank & andMask) | outer;
        } else if (m_config.id == 215) {
            if (m_m215Mode & 0x80) {
                // NROM override. $5000 bits 0-3 form a 16 KiB bank. Bit 5
                // replaces its low bit with CPU A14 for an NROM-256 pair.
                std::size_t bank16 = m_m215Mode & 0x0F;
                if (m_m215Mode & 0x40)
                    bank16 = (bank16 & ~std::size_t(0x08)) | (std::size_t((m_m215Outer >> 4) & 1) << 3);
                if (m_m215Mode & 0x20)
                    bank16 = (bank16 & ~std::size_t(1)) | ((addr >> 14) & 1);
                bank = (bank16 << 1) | ((addr >> 13) & 1);
            }
            // UNL-8237A overlaps three PRG outer-bank bits across $5001
            // bits 0,1,3; UNL-8237 uses the two independent low bits.
            const std::size_t outerPrg = m_config.submapper == 1
                ? std::size_t((m_m215Outer & 0x03) | ((m_m215Outer >> 1) & 0x04))
                : std::size_t(m_m215Outer & 0x03);
            const std::size_t lowMask = (m_m215Mode & 0x40) ? 0x0F : 0x1F;
            bank = (bank & lowMask) | (outerPrg << 5);
            if (m_m215Mode & 0x40) bank = (bank & ~std::size_t(0x10)) | (std::size_t((m_m215Outer >> 4) & 1) << 4);
        }
        mapped = mapBank(bank, 0x2000, m_config.prgRomSize, inBank);
        return true;
    }

    bool cpuReadRegister(uint16_t addr, uint8_t& data) override
    {
        // SL-5020B (mapper 12.0): reading the GAL register returns the
        // language-select input on D0. No known PCB exposes a configurable
        // jumper, so use the observed/default low state while still driving
        // the register read instead of open bus.
        if (m_config.id == 12 && addr == 0x4132) { data = 0; return true; }
        if (m_config.id == 121 && addr >= 0x5000 && addr <= 0x5FFF) { data = m_m121Ex[4]; return true; }
        if (!m_mmc6 || addr < 0x7000 || addr > 0x7FFF) return false;
        if (!m_mmc6RamGlobalEnable) return false;
        if (!m_mmc6LowRead && !m_mmc6HighRead) return false;
        const bool highHalf = (addr & 0x0200) != 0;
        const bool readable = highHalf ? m_mmc6HighRead : m_mmc6LowRead;
        if (!readable) { data = 0; return true; }
        data = m_mmc6Ram[addr & 0x03FF];
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override
    {
        // SL-5020B GAL: independent CHR A18 selection for the two PPU
        // pattern-table halves. This logic sits outside the Huang-1/MMC3A
        // ASIC, so it takes effect immediately and is not affected by $8000.7.
        if (m_config.id == 12 && addr == 0x4132) { m_m12ChrOuter = data; return true; }
        if (m_config.id == 121 && addr >= 0x5000 && addr <= 0x5FFF) {
            static constexpr uint8_t lookup[4] = {0x83, 0x83, 0x42, 0x00};
            m_m121Ex[4] = lookup[data & 0x03];
            if ((addr & 0x5180) == 0x5180) m_m121Ex[3] = data;
            return true;
        }
        if (m_config.id == 215 && (addr & 0xF007) == 0x5000) { m_m215Mode = data; return true; }
        if (m_config.id == 215 && (addr & 0xF007) == 0x5001) { m_m215Outer = data; return true; }
        if (m_config.id == 215 && (addr & 0xF007) == 0x5007) { m_m215Scramble = data & 7; return true; }
        if (m_config.id == 123 && addr >= 0x5800 && addr <= 0x5FFF) { m_m123Mode = data; return true; }
        if (m_config.id == 45 && (addr & 0xF001) == 0x6000) {
            if ((m_m45Outer[3] & 0x40) == 0) {
                m_m45Outer[m_m45Index] = data;
                m_m45Index = uint8_t((m_m45Index + 1) & 3);
            }
            return true;
        }
        if (m_config.id == 45 && (addr & 0xF001) == 0x6001) {
            reset45Outer();
            return true;
        }
        if ((m_config.id == 114 || m_config.id == 115) && (addr & 0xE001) == 0x6000) {
            m_extMode = data;
            return true;
        }
        if ((m_config.id == 114 || m_config.id == 115) && (addr & 0xE001) == 0x6001) {
            m_extChr = data & 1;
            return true;
        }
        if (m_config.id == 197 && m_config.nes20 && m_config.submapper == 3 && addr >= 0x6000 && addr <= 0x7FFF) {
            if (m_prgRamEnable && !m_prgRamWriteProtect) m_outerReg = data;
            return true;
        }
        if (m_config.id == 52 && addr >= 0x6000 && addr <= 0x7FFF) {
            // The Realtec outer register overlaps WRAM and is writable only
            // while MMC3 WRAM is enabled/writeable. D7 locks it until reset;
            // after locking, writes pass through to actual WRAM.
            if (m_prgRamEnable && !m_prgRamWriteProtect && (m_outerReg & 0x80) == 0)
                m_outerReg = data;
            return true;
        }
        if ((m_config.id == 37 || m_config.id == 47 || m_config.id == 49) && addr >= 0x6000 && addr <= 0x7FFF) {
            // These multicart registers replace PRG RAM.  Mapper 37/47 and 49
            // route the write through the MMC3 WRAM enable/protect outputs.
            if (m_prgRamEnable && !m_prgRamWriteProtect) m_outerReg = data;
            return true;
        }
        if (m_mmc6 && addr >= 0x7000 && addr <= 0x7FFF) {
            if (!m_mmc6RamGlobalEnable) return true;
            const bool highHalf = (addr & 0x0200) != 0;
            const bool readable = highHalf ? m_mmc6HighRead : m_mmc6LowRead;
            const bool writable = highHalf ? m_mmc6HighWrite : m_mmc6LowWrite;
            if (readable && writable) m_mmc6Ram[addr & 0x03FF] = data;
            return true;
        }
        if (addr < 0x8000) return false;
        uint16_t regAddr = addr & 0xE001;
        if (m_config.id == 121 && addr < 0xA000) {
            if ((addr & 0x03) == 0x03) {
                m_m121Ex[5] = data;
                update121ExRegs();
                regAddr = 0x8000;
            } else if (addr & 1) {
                m_m121Ex[6] = reverse121Low6(data);
                if (!m_m121Ex[7]) update121ExRegs();
                regAddr = 0x8001;
            } else {
                regAddr = 0x8000;
            }
        }
        if (m_config.id == 114) regAddr = unscramble114Address(regAddr);
        else if (m_config.id == 215) regAddr = unscramble215Address(regAddr);
        switch (regAddr) {
        case 0x8000:
            if (m_config.id == 114) data = uint8_t((data & 0xC0) | unscramble114Index(data & 7));
            else if (m_config.id == 215) data = uint8_t((data & 0xC0) | unscramble215Index(data & 7));
            else if (m_config.id == 123) {
                static constexpr uint8_t kIndex[8] = {0,3,1,5,6,7,2,4};
                data = uint8_t((data & 0xC0) | kIndex[data & 7]);
            }
            m_bankSelect = data;
            m_prgMode = (data >> 6) & 1;
            m_chrMode = (data >> 7) & 1;
            if (m_mmc6) {
                m_mmc6RamGlobalEnable = (data & 0x20) != 0;
                if (!m_mmc6RamGlobalEnable) {
                    m_mmc6LowRead = m_mmc6LowWrite = false;
                    m_mmc6HighRead = m_mmc6HighWrite = false;
                }
            }
            break;
        case 0x8001:
            m_regs[m_bankSelect & 7] = data;
            break;
        case 0xA000:
            if (m_config.id != 118 && !m_config.fourScreen)
                m_mirror = (data & 1) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        case 0xA001:
            if (m_config.id == 44) m_outerReg = data & 7;
            if (m_mmc6) {
                if (m_mmc6RamGlobalEnable) {
                    m_mmc6LowWrite = (data & 0x10) != 0;
                    m_mmc6LowRead = (data & 0x20) != 0;
                    m_mmc6HighWrite = (data & 0x40) != 0;
                    m_mmc6HighRead = (data & 0x80) != 0;
                }
            } else {
                m_prgRamWriteProtect = (data & 0x40) != 0;
                m_prgRamEnable = (data & 0x80) != 0;
            }
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
        if (m_mmc6 || m_config.id == 37 || m_config.id == 47 || m_config.id == 49 ||
            m_config.id == 114 || m_config.id == 115 || (m_config.id == 197 && m_config.submapper == 3)) return false;
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
            return false;
        if (!m_prgRamEnable || (write && m_prgRamWriteProtect))
            return false;
        if (m_config.id == 52 && write && (m_outerReg & 0x80) == 0)
            return false;
        mapped = static_cast<uint32_t>((addr - 0x6000) % m_config.prgRamSize);
        return true;
    }

    std::size_t mapperBatterySize() const override
    {
        return (m_mmc6 && m_config.headerPrgNvRamSize != 0) ? sizeof(m_mmc6Ram) : 0;
    }

    void saveMapperBattery(std::vector<uint8_t>& out) const override
    {
        if (mapperBatterySize()) out.insert(out.end(), std::begin(m_mmc6Ram), std::end(m_mmc6Ram));
    }

    bool loadMapperBattery(const uint8_t* data, std::size_t size) override
    {
        if (!mapperBatterySize()) return size == 0;
        if (size != sizeof(m_mmc6Ram)) return false;
        std::copy(data, data + size, std::begin(m_mmc6Ram));
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

    void reset(bool hard) override
    {
        if (m_config.id == 121) reset121();
        if (m_config.id == 45) reset45Outer();
        if (m_config.id == 52) m_outerReg = 0;
        if (m_config.id == 215) {
            m_m215Outer = 0x0F;
            if (hard) { m_m215Mode = 0; m_m215Scramble = 0; }
        }
    }

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
        put8(out, m_outerReg);
        put8(out, m_extMode); put8(out, m_extChr);
        if (m_config.id == 12) put8(out, m_m12ChrOuter);
        if (m_config.id == 45) {
            put8(out, m_m45Index);
            for (uint8_t value : m_m45Outer) put8(out, value);
        }
        if (m_config.id == 215) { put8(out, m_m215Mode); put8(out, m_m215Outer); put8(out, m_m215Scramble); }
        if (m_config.id == 123) put8(out, m_m123Mode);
        if (m_config.id == 121) for (uint8_t value : m_m121Ex) put8(out, value);
        put8(out, m_mmc6 ? 1 : 0);
        if (m_mmc6) {
            put8(out, m_mmc6RamGlobalEnable ? 1 : 0);
            put8(out, m_mmc6LowRead ? 1 : 0); put8(out, m_mmc6LowWrite ? 1 : 0);
            put8(out, m_mmc6HighRead ? 1 : 0); put8(out, m_mmc6HighWrite ? 1 : 0);
            out.insert(out.end(), std::begin(m_mmc6Ram), std::end(m_mmc6Ram));
        }
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
        if (!get8(p, end, m_outerReg)) return false;
        if (!get8(p, end, m_extMode) || !get8(p, end, m_extChr)) return false;
        if (m_config.id == 12 && !get8(p, end, m_m12ChrOuter)) return false;
        if (m_config.id == 45) {
            if (!get8(p, end, m_m45Index)) return false;
            if (m_m45Index > 3) return false;
            for (uint8_t& value : m_m45Outer) if (!get8(p, end, value)) return false;
        }
        if (m_config.id == 215) {
            if (!get8(p, end, m_m215Mode) || !get8(p, end, m_m215Outer) || !get8(p, end, m_m215Scramble)) return false;
            m_m215Scramble &= 7;
        }
        if (m_config.id == 123 && !get8(p, end, m_m123Mode)) return false;
        if (m_config.id == 121) for (uint8_t& value : m_m121Ex) if (!get8(p, end, value)) return false;
        if (!get8(p, end, tmp)) return false;
        if ((tmp != 0) != m_mmc6) return false;
        if (m_mmc6) {
            if (!get8(p, end, tmp)) return false;
            m_mmc6RamGlobalEnable = tmp != 0;
            if (!get8(p, end, tmp)) return false;
            m_mmc6LowRead = tmp != 0;
            if (!get8(p, end, tmp)) return false;
            m_mmc6LowWrite = tmp != 0;
            if (!get8(p, end, tmp)) return false;
            m_mmc6HighRead = tmp != 0;
            if (!get8(p, end, tmp)) return false;
            m_mmc6HighWrite = tmp != 0;
            if (end - p < static_cast<std::ptrdiff_t>(sizeof(m_mmc6Ram))) return false;
            std::copy(p, p + sizeof(m_mmc6Ram), std::begin(m_mmc6Ram));
            p += sizeof(m_mmc6Ram);
        }
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
    uint8_t m_outerReg = 0;
    uint8_t m_extMode = 0;
    uint8_t m_extChr = 0;
    uint8_t m_m12ChrOuter = 0;
    uint8_t m_m45Outer[4] = {};
    uint8_t m_m45Index = 0;
    uint8_t m_m215Mode = 0;
    uint8_t m_m215Outer = 0x0F;
    uint8_t m_m215Scramble = 0;
    uint8_t m_m123Mode = 0;
    uint8_t m_m121Ex[8] = {};
    bool m_mmc6 = false;
    bool m_mmc6RamGlobalEnable = false;
    bool m_mmc6LowRead = false, m_mmc6LowWrite = false;
    bool m_mmc6HighRead = false, m_mmc6HighWrite = false;
    uint8_t m_mmc6Ram[0x400] = {};

    std::size_t chrBankForAddress(uint16_t addr) const
    {
        if (m_config.id == 197) {
            std::size_t b = 0;
            const uint8_t sub = m_config.nes20 ? m_config.submapper : 0;
            if (sub == 1) {
                if (addr < 0x0800) b = std::size_t(m_regs[0] & 0xFE) + ((addr >> 10) & 1);
                else if (addr < 0x1000) b = std::size_t(m_regs[1] | 1) - 1 + ((addr >> 10) & 1);
                else if (addr < 0x1800) b = std::size_t(m_regs[2]) * 2 + ((addr >> 10) & 1);
                else b = std::size_t(m_regs[5]) * 2 + ((addr >> 10) & 1);
            } else {
                if (addr < 0x1000) b = std::size_t(m_regs[0] & 0xFE) * 2 + ((addr >> 10) & 3);
                else if (addr < 0x1800) b = std::size_t(m_regs[2]) * 2 + ((addr >> 10) & 1);
                else b = std::size_t(m_regs[3]) * 2 + ((addr >> 10) & 1);
            }
            return b;
        }
        const uint8_t r0 = m_regs[0] & 0xFE;
        const uint8_t r1 = m_regs[1] & 0xFE;
        std::size_t bank = 0;
        if (!m_chrMode) {
            if (addr < 0x0800) bank = r0 + ((addr >> 10) & 1);
            else if (addr < 0x1000) bank = r1 + ((addr >> 10) & 1);
            else if (addr < 0x1400) bank = m_regs[2];
            else if (addr < 0x1800) bank = m_regs[3];
            else if (addr < 0x1C00) bank = m_regs[4];
            else bank = m_regs[5];
        } else {
            if (addr < 0x0400) bank = m_regs[2];
            else if (addr < 0x0800) bank = m_regs[3];
            else if (addr < 0x0C00) bank = m_regs[4];
            else if (addr < 0x1000) bank = m_regs[5];
            else if (addr < 0x1800) bank = r0 + ((addr >> 10) & 1);
            else bank = r1 + ((addr >> 10) & 1);
        }
        if (m_config.id == 37) bank = (std::size_t((m_outerReg >> 2) & 1) << 7) | (bank & 0x7F);
        else if (m_config.id == 44) { const uint8_t block=std::min<uint8_t>(m_outerReg&7,6); bank = block==6 ? ((bank&0xFF)|0x300) : ((bank&0x7F)|(std::size_t(block)<<7)); }
        else if (m_config.id == 47) bank = (std::size_t(m_outerReg & 1) << 7) | (bank & 0x7F);
        else if (m_config.id == 49) bank = (std::size_t((m_outerReg >> 6) & 0x03) << 7) | (bank & 0x7F);
        else if (m_config.id == 52) {
            // D2 supplies CHR A19, D5 A18; D6 chooses CHR A17 from
            // the MMC3 bank or from outer bit D4.
            const std::size_t lowMask = (m_outerReg & 0x40) ? 0x7F : 0xFF;
            const std::size_t a17 = (m_outerReg & 0x40) ? std::size_t((m_outerReg >> 4) & 1) : ((bank >> 7) & 1);
            bank = (bank & lowMask) | (a17 << 7) | (std::size_t((m_outerReg >> 5) & 1) << 8) |
                (std::size_t((m_outerReg >> 2) & 1) << 9);
        }
        else if (m_config.id == 121) {
            if (m_config.prgRomSize == m_config.chrRomSize) {
                // Super 3-in-1 A9713 wiring: the same outer selector feeds
                // CHR A18 (1 KiB bank bit 8).
                bank |= std::size_t(m_m121Ex[3] & 0x80) << 1;
            } else if (m_config.chrRomSize > 0x40000) {
                // A9711 with 512 KiB CHR wires CHR A18 directly to PPU A12,
                // independent of MMC3 CHR inversion: left table = first half,
                // right table = second half.
                bank = (bank & 0xFF) | ((addr & 0x1000) ? 0x100 : 0);
            }
        }
        else if (m_config.id == 12) {
            const uint8_t a18 = (addr & 0x1000) ? ((m_m12ChrOuter >> 4) & 1) : (m_m12ChrOuter & 1);
            bank = (std::size_t(a18) << 8) | (bank & 0xFF);
        }
        else if (m_config.id == 114 || m_config.id == 115) bank = (std::size_t(m_extChr & 1) << 8) | (bank & 0xFF);
        else if (m_config.id == 215) {
            const std::size_t outerChr = m_config.submapper == 1
                ? std::size_t((m_m215Outer >> 1) & 0x07)
                : std::size_t((m_m215Outer >> 2) & 0x03);
            const std::size_t lowMask = (m_m215Mode & 0x40) ? 0x7F : 0xFF;
            bank = (bank & lowMask) | (outerChr << 8);
            if (m_m215Mode & 0x40) bank = (bank & ~std::size_t(0x80)) | (std::size_t((m_m215Outer >> 5) & 1) << 7);
        }
        else if (m_config.id == 45 && m_config.chrRomSize != 0) {
            // Register #2 low nibble selects how many low MMC3 CHR-bank bits
            // survive (0-$7 = none, $8 = one ... $F = all eight).
            const uint8_t width = m_m45Outer[2] & 0x0F;
            const std::size_t andMask = width <= 7 ? 0 : ((std::size_t(1) << (width - 7)) - 1);
            const std::size_t outer = std::size_t(m_m45Outer[0]) |
                (std::size_t(m_m45Outer[2] & 0xF0) << 4);
            bank = (bank & andMask) | outer;
        }
        return bank;
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

    uint16_t unscramble114Address(uint16_t a) const
    {
        const bool sm1 = m_config.nes20 && m_config.submapper == 1;
        static constexpr uint16_t sm0From[8] = {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001};
        static constexpr uint16_t sm0To[8]   = {0xA001,0xA000,0x8000,0xC000,0x8001,0xC001,0xE000,0xE001};
        static constexpr uint16_t sm1To[8]   = {0xA001,0x8001,0x8000,0xC001,0xA000,0xC000,0xE000,0xE001};
        for (int i=0;i<8;++i) if (a==sm0From[i]) return sm1 ? sm1To[i] : sm0To[i];
        return a;
    }

    uint8_t unscramble114Index(uint8_t i) const
    {
        static constexpr uint8_t sm0[8] = {0,3,1,5,6,7,2,4};
        static constexpr uint8_t sm1[8] = {0,2,5,3,6,1,7,4};
        return (m_config.nes20 && m_config.submapper == 1) ? sm1[i & 7] : sm0[i & 7];
    }

    uint16_t unscramble215Address(uint16_t a) const
    {
        static constexpr uint16_t written[8] = {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001};
        static constexpr uint16_t table[8][8] = {
            {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001},
            {0xA001,0xA000,0x8000,0xC000,0x8001,0xC001,0xE000,0xE001},
            {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001},
            {0xC001,0x8000,0x8001,0xA000,0xA001,0xE001,0xE000,0xC000},
            {0xA001,0x8001,0x8000,0xC000,0xA000,0xC001,0xE000,0xE001},
            {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001},
            {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001},
            {0x8000,0x8001,0xA000,0xA001,0xC000,0xC001,0xE000,0xE001}
        };
        for (int i=0;i<8;++i) if (a==written[i]) return table[m_m215Scramble & 7][i];
        return a;
    }

    uint8_t unscramble215Index(uint8_t i) const
    {
        static constexpr uint8_t table[8][8] = {
            {0,1,2,3,4,5,6,7}, {0,2,6,1,7,3,4,5},
            {0,5,4,1,7,2,6,3}, {0,6,3,7,5,2,4,1},
            {0,2,5,3,6,1,7,4}, {0,1,2,3,4,5,6,7},
            {0,1,2,3,4,5,6,7}, {0,1,2,3,4,5,6,7}
        };
        return table[m_m215Scramble & 7][i & 7];
    }

    static uint8_t reverse121Low6(uint8_t value)
    {
        return uint8_t(((value & 0x01) << 5) | ((value & 0x02) << 3) |
            ((value & 0x04) << 1) | ((value & 0x08) >> 1) |
            ((value & 0x10) >> 3) | ((value & 0x20) >> 5));
    }

    void update121ExRegs()
    {
        switch (m_m121Ex[5] & 0x3F) {
        case 0x20: case 0x29: case 0x2B: case 0x3C: case 0x3F:
            m_m121Ex[7] = 1; m_m121Ex[0] = m_m121Ex[6]; break;
        case 0x26:
            m_m121Ex[7] = 0; m_m121Ex[0] = m_m121Ex[6]; break;
        case 0x2C:
            m_m121Ex[7] = 1; if (m_m121Ex[6]) m_m121Ex[0] = m_m121Ex[6]; break;
        case 0x28:
            m_m121Ex[7] = 0; m_m121Ex[1] = m_m121Ex[6]; break;
        case 0x2A:
            m_m121Ex[7] = 0; m_m121Ex[2] = m_m121Ex[6]; break;
        case 0x2F:
            break;
        default:
            m_m121Ex[5] = 0; break;
        }
    }

    void reset121()
    {
        for (uint8_t& value : m_m121Ex) value = 0;
        m_m121Ex[3] = 0x80;
    }

    void reset45Outer()
    {
        m_m45Index = 0;
        m_m45Outer[0] = 0;
        m_m45Outer[1] = 0;
        m_m45Outer[2] = 0x0F;
        m_m45Outer[3] = 0;
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
        // Mapper 12.0 uses a Huang-1 in MMC3A-compatible mode. Like MMC3A,
        // a latch value of zero does not continuously assert an IRQ. Mapper
        // 114 has the same clone-specific zero-latch behavior.
        if (m_irqCounter == 0 && m_irqEnabled && !((m_config.id == 12 || m_config.id == 114) && m_irqLatch == 0))
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

    bool hasBusConflicts() const override
    {
        // AxROM submapper 0 keeps the historical no-conflict emulator default.
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
        // Flashable UNROM-512 uses an SST39SF040 (512 KiB). Preserve a
        // smaller supplied image at the bottom and leave unwritten space erased.
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
            // SST39SF040 software-ID mode: manufacturer BFh, device B7h.
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

        // With flash enabled, $8000-$BFFF is connected to the SST39SF040
        // write input rather than the bank latch. Submapper 4 also decodes
        // these writes into its cartridge LED register.
        if (m_flashEnabled && addr < 0xC000) {
            flashWrite(flashPhysicalAddress(addr), data);
            if (m_ledVariant) m_led = data;
            return true;
        }

        // Explicit no-conflict variants gate the mapper latch to $C000-$FFFF.
        // Submapper 4 uses $8000-$BFFF for its LED register even when a dump
        // is not marked battery-backed/self-flashable.
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
        // A console reset aborts an in-progress flash command but does not
        // erase programmed flash contents.
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
        // F0 is a software reset while idle/unlocking, but after the A0
        // byte-program command it is ordinary payload data and must be
        // programmable like any other byte.
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
            // NOR flash programming can only clear bits (1 -> 0).
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
                std::fill(m_flash.begin() + base, m_flash.begin() + limit, 0xFF);
            } else if (chipAddr == 0x5555 && data == 0x10) {
                std::fill(m_flash.begin(), m_flash.end(), 0xFF);
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
            // UNROM-512 submapper 3: D7=0 vertical, D7=1 horizontal.
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
            // NES 2.0 submapper 1 is BF9097/Fire Hawk with mapper-controlled
            // single-screen mirroring. Submapper 0 is hardwired H/V. Legacy
            // iNES keeps the established heuristic: only $9000-$9FFF enables
            // the Fire Hawk-style mirroring latch. Once observed, remember the
            // board inference because BF9097 also exposes only three PRG-bank
            // bits, unlike the four-bit BF9093 used by the normal boards.
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
        // Color Dreams: D1-D0 select 32 KiB PRG; D7-D4 select 8 KiB CHR.
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
        // Mapper 185 mounts one 8 KiB CHR-ROM. The latch is protection-only;
        // its two low bits drive the ROM's programmable chip-select pins.
        mapped = static_cast<uint32_t>(addr % m_config.chrRomSize);
        return true;
    }

    bool ppuReadOverride(uint16_t addr, PpuFetchKind kind, uint8_t& data) override
    {
        // Legacy iNES images cannot identify which one of the four CS1/CS2
        // values enables CHR. The established compatibility heuristic for all
        // known mapper-185 dumps is to expose open bus ($FF) on the first two
        // CPU PPUDATA pattern-table reads after reset, then treat CHR as
        // enabled. Rendering fetches must not consume this two-read window.
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
        if (chip == 2) return false; // Action 52 has no PRG chip in socket 2.

        const std::size_t chipBase = chip == 3 ? 0x100000u : std::size_t(chip) * 0x80000u;
        if (chipBase >= m_config.prgRomSize) return false;

        uint8_t page = uint8_t((m_regAddr >> 6) & 0x1F);
        const bool mirror16 = (m_regAddr & 0x20) != 0;
        if (mirror16) {
            const std::size_t bank16 = page;
            const uint32_t in = addr & 0x3FFF;
            mapped = static_cast<uint32_t>(chipBase + bank16 * 0x4000u + in);
        } else {
            // 32 KiB mode forces bit 0 of the 16 KiB page number from A14.
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
#include "MapperHard.inc"
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
        // Historical mapper 111 is a non-serialized MMC1 clone: each write
        // directly replaces the selected MMC1 register rather than shifting
        // one serial bit at a time.
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
        // Unlike a normal MMC1, this clone exposes enough CHR address lines
        // for 256 KiB. In 4 KiB mode the entire direct-write register is the
        // bank number; in 8 KiB mode CHR0 bit 0 is ignored.
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
        // Latch when A14 and A12 are both high; this decodes $5000-$5FFF,
        // $7000-$7FFF, $D000-$DFFF and $F000-$FFFF.
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
                const uint32_t base = phys & ~uint32_t(0x0FFF);
                std::fill(m_flash.begin()+base, m_flash.begin()+std::min<uint32_t>(base+0x1000, m_flash.size()), 0xFF);
            } else if (a == 0x5555 && data == 0x10) std::fill(m_flash.begin(), m_flash.end(), 0xFF);
            m_flashState = 0; break;
        default: m_flashState = 0; break;
        }
    }
    uint8_t m_reg=0;
    uint8_t m_flashState=0;
    bool m_idMode=false;
    std::vector<uint8_t> m_flash;
};


} // namespace

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

std::unique_ptr<Mapper> createMapper(const MapperConfig& config)
{
    switch (config.id) {
    case 0: return std::make_unique<Mapper0>(config);
    case 1: case 155: return std::make_unique<Mapper1>(config);
    case 2: return std::make_unique<Mapper2>(config);
    case 3: return std::make_unique<Mapper3>(config);
    case 4: case 37: case 44: case 45: case 47: case 49: case 52: case 74: case 114: case 115: case 118: case 119: case 121: case 123: case 191: case 192: case 194: case 195: case 197: case 215:
        return std::make_unique<Mapper4>(config);
    case 182: {
        MapperConfig compat = config; compat.id = 114;
        return std::make_unique<Mapper4>(compat);
    }
    case 5: return std::make_unique<Mapper5>(config);
    case 6: case 8: case 17: return std::make_unique<MapperSuperMagicCard>(config);
    case 12:
        if (config.submapper == 1) return std::make_unique<MapperSuperMagicCard>(config);
        return std::make_unique<Mapper4>(config);
    case 7: return std::make_unique<Mapper7>(config);
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
    case 19: case 210: return std::make_unique<Mapper19>(config);
    case 20: return std::make_unique<Mapper20>(config);
    case 21: case 22: case 23: case 25: case 27: return std::make_unique<MapperVrc24>(config);
    case 28: return std::make_unique<Mapper28>(config);
    case 24: case 26: return std::make_unique<MapperVrc6>(config);
    case 30: return std::make_unique<Mapper30>(config);
    case 31: return std::make_unique<Mapper31>(config);
    case 32: return std::make_unique<Mapper32>(config);
    case 33: return std::make_unique<Mapper33>(config);
    case 34: return std::make_unique<Mapper34>(config);
    case 35: return std::make_unique<MapperJY>(config);
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
    case 81: return std::make_unique<Mapper81>(config);
    case 82: return std::make_unique<Mapper82>(config);
    case 85: return std::make_unique<Mapper85>(config);
    case 86: return std::make_unique<Mapper86>(config);
    case 87: return std::make_unique<Mapper87>(config);
    case 88: case 154: return std::make_unique<Mapper88>(config);
    case 89: return std::make_unique<Mapper89>(config);
    case 90: case 209: case 211: return std::make_unique<MapperJY>(config);
    case 91: return std::make_unique<Mapper91>(config);
    case 93: return std::make_unique<Mapper93>(config);
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
    // Historical fwNES assignment: mapper 122 describes the same Sunsoft-1
    // hardware standardized as mapper 184. Keep both numbers on one tested
    // implementation rather than duplicating the banking logic.
    case 122: case 184: return std::make_unique<Mapper184>(config);
    case 125: return std::make_unique<Mapper125>(config);
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
    case 171: return std::make_unique<Mapper171>(config);
    case 174: return std::make_unique<Mapper174>(config);
    case 176: case 179: return std::make_unique<Mapper176>(config);
    case 180: return std::make_unique<Mapper180>(config);
    case 185: return std::make_unique<Mapper185>(config);
    case 193: return std::make_unique<Mapper193>(config);
    case 200: return std::make_unique<Mapper200>(config);
    case 202: case 203: case 204: case 214: case 217: case 229: case 231:
        return std::make_unique<MapperBmcBundled47>(config);
    case 206: return std::make_unique<Mapper206>(config);
    case 212: return std::make_unique<Mapper212>(config);
    case 213: return std::make_unique<Mapper213>(config);
    case 225: case 255: case 226: case 227: case 230: case 235: case 236: case 237: return std::make_unique<MapperBmcBundled48>(config);
    case 228: return std::make_unique<Mapper228>(config);
    case 232: return std::make_unique<Mapper232>(config);
    case 240: return std::make_unique<Mapper240>(config);
    case 241: return std::make_unique<Mapper241>(config);
    case 242: return std::make_unique<Mapper242>(config);
    case 243: return std::make_unique<Mapper243>(config);
    case 268: return std::make_unique<Mapper268>(config);
    case 246: return std::make_unique<Mapper246>(config);
    default: return std::make_unique<MapperFallback>(config);
    }
}

bool mapperImplementationSupported(uint16_t mapper, uint8_t submapper)
{
    // For NES 2.0 variants that describe materially different hardware, do
    // not silently claim support for submappers the implementation does not
    // actually emulate. Submapper 0 remains the legacy/iNES-compatible path.
    switch (mapper) {
    case 1: return submapper == 0 || submapper == 5;
    case 155: return submapper == 0 || submapper == 5;
    case 2: case 3: case 7: return submapper <= 2;
    case 6: return submapper <= 7;
    case 8: return submapper == 0 || submapper == 4;
    case 12: return submapper <= 1;
    case 17: return submapper <= 3;
    case 30: return submapper <= 4;
    case 32: return submapper <= 1;
    case 40: return submapper <= 1;
    case 63: return submapper <= 1;
    case 68: return submapper <= 1;
    case 34: return submapper <= 2;
    case 71: return submapper <= 1;
    case 78: return submapper == 0 || submapper == 1 || submapper == 3;
    case 85: return submapper <= 2;
    case 91: return submapper <= 1;
    case 108: return submapper <= 4;
    case 114: return submapper <= 1;
    case 182: return submapper <= 1;
    case 116: return submapper <= 3;
    case 122: case 184: return submapper == 0;
    case 125: return submapper == 0;
    case 132: return submapper == 0;
    case 133: return submapper == 0;
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
    case 156: return submapper == 0;
    case 171: return submapper == 0;
    case 174: return submapper == 0;
    case 176: case 179: return submapper <= 5;
    case 185: return submapper == 0 || (submapper >= 4 && submapper <= 7);
    case 193: return submapper == 0;
    case 200: return submapper == 0;
    case 201: return submapper == 0;
    case 202: case 203: case 204: case 214: case 217: case 229: case 231: return submapper == 0;
    case 225: case 255: case 226: case 227: case 230: case 235: case 236: case 237: return submapper == 0;
    case 197: return submapper == 0 || submapper == 1 || submapper == 3;
    case 206: return submapper <= 1;
    case 210: return submapper <= 2;
    case 212: return submapper == 0;
    case 213: return submapper == 0;
    case 215: return submapper <= 1;
    case 268: return submapper <= 11;
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
    case 154: case 155: case 157: case 158: case 159: case 174: case 176: case 180: case 184: case 185: case 191:
    case 192: case 193: case 194: case 195: case 197: case 200: case 206: case 207: case 209: case 211: case 213: case 215: case 228: case 232: case 240: case 241:
    case 242: case 243: case 246: case 268:
        return true;
    default:
        return false;
    }
}
