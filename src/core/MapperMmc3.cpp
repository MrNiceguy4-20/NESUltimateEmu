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
class Mapper4 final : public Mapper {
public:
    explicit Mapper4(const MapperConfig& config) : Mapper(config), m_mmc6(config.nes20 && config.submapper == 1)
    {
        if (m_config.id == 121) {
            reset121();
        }
        if (m_config.id == 45) {
            m_m45Outer[2] = 0x0F;

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

            const std::size_t lowMask = (m_outerReg & 0x08) ? 0x0F : 0x1F;
            const std::size_t a17 = (m_outerReg & 0x08) ? std::size_t(m_outerReg & 1) : ((bank >> 4) & 1);
            bank = (bank & lowMask) | (a17 << 4) | (std::size_t((m_outerReg >> 1) & 1) << 5) |
                (std::size_t((m_outerReg >> 2) & 1) << 6);
        } else if (m_config.id == 115) {

            bank = (std::size_t((m_extMode >> 6) & 1) << 5) | (bank & 0x1F);
        } else if (m_config.id == 197 && m_config.nes20 && m_config.submapper == 3 && (m_outerReg & 0x08)) {

            bank = (std::size_t(m_outerReg & 1) << 4) | (bank & 0x0F);
        } else if (m_config.id == 45) {

            const std::size_t andMask = std::size_t(0x3F ^ (m_m45Outer[3] & 0x3F));
            const std::size_t outer = std::size_t(m_m45Outer[1]) |
                (std::size_t(m_m45Outer[2] & 0xC0) << 2);
            bank = (bank & andMask) | outer;
        } else if (m_config.id == 215) {
            if (m_m215Mode & 0x80) {

                std::size_t bank16 = m_m215Mode & 0x0F;
                if (m_m215Mode & 0x40)
                    bank16 = (bank16 & ~std::size_t(0x08)) | (std::size_t((m_m215Outer >> 4) & 1) << 3);
                if (m_m215Mode & 0x20)
                    bank16 = (bank16 & ~std::size_t(1)) | ((addr >> 14) & 1);
                bank = (bank16 << 1) | ((addr >> 13) & 1);
            }

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

            if (m_prgRamEnable && !m_prgRamWriteProtect && (m_outerReg & 0x80) == 0)
                m_outerReg = data;
            return true;
        }
        if ((m_config.id == 37 || m_config.id == 47 || m_config.id == 49) && addr >= 0x6000 && addr <= 0x7FFF) {

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

            const std::size_t lowMask = (m_outerReg & 0x40) ? 0x7F : 0xFF;
            const std::size_t a17 = (m_outerReg & 0x40) ? std::size_t((m_outerReg >> 4) & 1) : ((bank >> 7) & 1);
            bank = (bank & lowMask) | (a17 << 7) | (std::size_t((m_outerReg >> 5) & 1) << 8) |
                (std::size_t((m_outerReg >> 2) & 1) << 9);
        }
        else if (m_config.id == 121) {
            if (m_config.prgRomSize == m_config.chrRomSize) {

                bank |= std::size_t(m_m121Ex[3] & 0x80) << 1;
            } else if (m_config.chrRomSize > 0x40000) {

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

        if (m_irqCounter == 0 && m_irqEnabled && !((m_config.id == 12 || m_config.id == 114) && m_irqLatch == 0))
            m_irqPending = true;
    }
};

}

std::unique_ptr<Mapper> createMmc3Mapper(const MapperConfig& config) { return std::make_unique<Mapper4>(config); }
