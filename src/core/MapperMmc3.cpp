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
        if (m_config.id == 208) {
            m_m208PrgMirror = 0x11;
            if (m_config.submapper == 0 && !m_config.fourScreen)
                m_mirror = Mirror::Vertical;
        }
        if (m_config.id == 219) {
            reset219();
        }
    }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override
    {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        if (m_config.id == 259) {
            const std::size_t mode = (m_m259ExReg >> 3) & 1u;
            const std::size_t base = std::size_t(m_m259ExReg & 0x0F) & ~mode;
            const std::size_t bank16 = (addr < 0xC000) ? base : (base | mode);
            mapped = mapBank(bank16, 0x4000, m_config.prgRomSize, addr & 0x3FFF);
            return true;
        }
        const std::size_t count = std::max<std::size_t>(1, m_config.prgRomSize / 0x2000);
        const uint8_t prgMask = (m_config.id == 198 || m_config.id == 199) ? 0x7F : 0x3F;
        const uint8_t r6 = m_regs[6] & prgMask;
        const uint8_t r7 = m_regs[7] & prgMask;
        const std::size_t last = m_config.id == 198 ? std::min<std::size_t>(0x4F, count - 1) : count - 1;
        const std::size_t lastM1 = m_config.id == 198 ? std::min<std::size_t>(0x4E, count - 1) : (count > 1 ? count - 2 : 0);
        std::size_t bank = 0;
        uint32_t inBank = addr & 0x1FFF;

        if (m_config.id == 219 && m_m219Extended) {
            const std::size_t slot = (addr - 0x8000) >> 13;
            bank = (std::size_t(m_m219Outer & 3) << 4) | (m_m219Prg[slot] & 0x0F);
            mapped = mapBank(bank, 0x2000, m_config.prgRomSize, inBank);
            return true;
        }

        if (m_config.id == 208) {
            std::size_t bank32 = 0;
            if (m_config.submapper == 1)
                bank32 = std::size_t(m_regs[6] >> 2);
            else
                bank32 = std::size_t(((m_m208PrgMirror >> 3) & 0x02) | (m_m208PrgMirror & 0x01));
            mapped = mapBank(bank32, 0x8000, m_config.prgRomSize, addr & 0x7FFF);
            return true;
        }
        if (m_config.id == 189) {
            mapped = mapBank(m_m189PrgBank, 0x8000, m_config.prgRomSize, addr & 0x7FFF);
            return true;
        }
        if (m_config.id == 187 && (m_m187Ex0 & 0x80)) {
            std::size_t bank8 = 0;
            std::size_t exPage = m_m187Ex0 & 0x1F;
            const std::size_t slot = (addr - 0x8000) >> 13;
            if (m_m187Ex0 & 0x20) {
                if (m_m187Ex0 & 0x40) {
                    exPage &= 0x1C;
                    bank8 = exPage + slot;
                } else {
                    exPage &= 0x1E;
                    bank8 = (exPage << 1) + slot;
                }
            } else {
                bank8 = (exPage << 1) + (slot & 1);
            }
            mapped = mapBank(bank8, 0x2000, m_config.prgRomSize, addr & 0x1FFF);
            return true;
        }

        if (m_config.id == 14 && (m_m14Mode & 0x02) == 0) {
            std::size_t vrcBank = 0;
            if (addr < 0xA000) vrcBank = m_m14Prg[0];
            else if (addr < 0xC000) vrcBank = m_m14Prg[1];
            else if (addr < 0xE000) vrcBank = count > 1 ? count - 2 : 0;
            else vrcBank = count - 1;
            mapped = mapBank(vrcBank, 0x2000, m_config.prgRomSize, addr & 0x1FFF);
            return true;
        }

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
        if (is126Family()) {
            bank = transform126PrgBank(bank);
            const uint8_t mode = m_m126Ex[3] & 0x03;
            if (mode != 0) {
                std::size_t base = transform126PrgBank(std::size_t(m_regs[6] & 0x3F));
                if (mode == 3) {
                    base &= ~std::size_t(3);
                    bank = base + ((addr - 0x8000) >> 13);
                } else {
                    base &= ~std::size_t(1);
                    bank = base + (((addr - 0x8000) >> 13) & 1);
                }
            }
        }
        if (m_config.id == 245)
            bank = (bank & 0x3F) | (std::size_t(m_m245PrgOuter & 1) << 6);
        if (m_config.id == 196 && m_m196Override)
            bank = std::size_t(m_m196PrgBank) * 4 + ((addr - 0x8000) >> 13);
        if (m_config.id == 205) {
            bank &= (m_m205Block <= 1) ? 0x1F : 0x0F;
            bank |= std::size_t(m_m205Block) * 0x10;
        }
        if (m_config.id == 224)
            bank = (bank & 0x3F) | (std::size_t(m_m224Outer & 1) << 6);
        if (m_config.id == 134)
            bank = (bank & 0x1F) | (std::size_t(m_m134ExReg & 0x02) << 4);
        if (m_config.id == 249 && (m_m249ExReg & 0x02))
            bank = scramble249Prg(bank);
        if (m_config.id == 219)
            bank = (bank & 0x0F) | (std::size_t(m_m219Outer & 3) << 4);
        mapped = mapBank(bank, 0x2000, m_config.prgRomSize, inBank);
        return true;
    }

    bool cpuReadRegister(uint16_t addr, uint8_t& data) override
    {

        if (m_config.id == 12 && addr == 0x4132) { data = 0; return true; }
        if (m_config.id == 121 && addr >= 0x5000 && addr <= 0x5FFF) { data = m_m121Ex[4]; return true; }
        if (m_config.id == 208 && m_config.submapper == 0 && addr >= 0x5800 && addr <= 0x5FFF) {
            data = m_m208Protection[addr & 3];
            return true;
        }
        if (m_config.id == 238 && addr >= 0x4020 && addr <= 0x7FFF) {
            data = m_m238ExReg;
            return true;
        }
        if (m_config.id == 187 && addr >= 0x5000 && addr <= 0x5FFF) {
            static constexpr uint8_t security[4] = {0x83,0x83,0x42,0x00};
            data = security[m_m187Ex1 & 3];
            return true;
        }
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
        if (m_config.id == 187 && (addr == 0x5000 || addr == 0x6000)) { m_m187Ex0 = data; return true; }
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
        if (m_config.id == 208 && m_config.submapper == 0) {
            if ((addr >= 0x4800 && addr <= 0x4FFF) || (addr >= 0x6800 && addr <= 0x6FFF)) {
                m_m208PrgMirror = data;
                if (!m_config.fourScreen)
                    m_mirror = (data & 0x20) ? Mirror::Horizontal : Mirror::Vertical;
                return true;
            }
            if (addr >= 0x5000 && addr <= 0x57FF) {
                m_m208ProtectionIndex = data;
                return true;
            }
            if (addr >= 0x5800 && addr <= 0x5FFF) {
                m_m208Protection[addr & 3] = uint8_t(data ^ mapper208ProtectionLut(m_m208ProtectionIndex));
                return true;
            }
        }
        if (is126Family() && addr >= 0x6000 && addr <= 0x7FFF) {

            if (m_prgRamEnable && !m_prgRamWriteProtect) {
                const uint8_t index = uint8_t(addr & 0x03);
                if (index == 1 || index == 2 || ((index == 0 || index == 3) && (m_m126Ex[3] & 0x80) == 0))
                    m_m126Ex[index] = data;
            }
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
        if (m_config.id == 189 && addr >= 0x4020 && addr < 0x8000) {
            m_m189PrgBank = uint8_t((data & 0x0F) | (data >> 4));
            return true;
        }
        if (m_config.id == 196 && addr >= 0x6000 && addr <= 0x6FFF) {
            m_m196Override = true;
            m_m196PrgBank = uint8_t((data & 0x0F) | (data >> 4));
            return true;
        }
        if (m_config.id == 205 && addr >= 0x6000 && addr < 0x8000) {
            m_m205Block = data & 0x03;
            return true;
        }
        if (m_config.id == 249 && addr == 0x5000) {
            m_m249ExReg = data;
            return true;
        }
        if (m_config.id == 224 && addr == 0x5000) {
            m_m224Outer = uint8_t((data >> 2) & 1);
            return true;
        }
        if (m_config.id == 134 && addr == 0x6001) {
            m_m134ExReg = data;
            return true;
        }
        if (m_config.id == 238 && addr >= 0x4020 && addr < 0x8000) {
            static constexpr uint8_t kSecurity[4] = {0x00, 0x02, 0x02, 0x03};
            m_m238ExReg = kSecurity[data & 3];
            return true;
        }
        if (m_config.id == 219 && (addr & 0xFFFC) == 0x5000) {
            if ((addr & 3) == 2) m_m219Outer = uint8_t((m_m219Outer & 2) | (data & 1));
            else if ((addr & 3) == 3) m_m219Outer = uint8_t((m_m219Outer & 1) | ((data >> 4) & 2));
            return true;
        }
        if (m_config.id == 259 && addr >= 0x6000 && addr < 0x8000) {
            if (m_prgRamEnable) m_m259ExReg = data & 0x0F;
            return true;
        }
        if (m_config.id == 263 && addr >= 0x8000) {
            data = uint8_t((data & 0xD8) | ((data & 0x20) >> 4) | ((data & 0x04) << 3) |
                ((data & 0x02) >> 1) | ((data & 0x01) << 2));
            if (addr == 0x9000) addr = 0x8001;
            else if (addr == 0xD000) addr = 0xC001;
            else if (addr == 0xF000) addr = 0xE001;
        }
        if (addr < 0x8000) return false;
        if (m_config.id == 219 && addr < 0xA000) {
            const uint16_t reg = addr & 0xE003;
            if (reg == 0x8002) {
                m_m219Selector = data;
                m_m219Extended = (data & 0x20) != 0;
                return true;
            }
            if (reg == 0x8000 && m_m219Extended) {
                m_m219Selector = data;
                return true;
            }
            if (reg == 0x8001 && m_m219Extended) {
                write219Extended(data);
                return true;
            }
        }
        if (m_config.id == 14) {
            if (addr == 0xA131) m_m14Mode = data;
            if ((m_m14Mode & 0x02) == 0) {
                if (addr >= 0xB000 && addr <= 0xEFFF) {
                    const uint8_t reg = uint8_t(((((addr >> 12) & 7) - 3) << 1) + ((addr >> 1) & 1));
                    if ((addr & 1) == 0) m_m14Chr[reg] = uint8_t((m_m14Chr[reg] & 0xF0) | (data & 0x0F));
                    else m_m14Chr[reg] = uint8_t((m_m14Chr[reg] & 0x0F) | ((data & 0x0F) << 4));
                    return true;
                }
                switch (addr & 0xF003) {
                case 0x8000: m_m14Prg[0] = data; return true;
                case 0x9000:
                    m_m14Mirror = data;
                    if (!m_config.fourScreen) m_mirror = (data & 1) ? Mirror::Horizontal : Mirror::Vertical;
                    return true;
                case 0xA000: m_m14Prg[1] = data; return true;
                default: return true;
                }
            }
        }
        if (m_config.id == 187) {
            if (addr == 0x8000) m_m187Ex1 = 1;
            else if (addr == 0x8001 && m_m187Ex1 != 1) return true;
        }
        if (m_config.id == 254) {
            if (addr == 0x8000) m_m254Unlocked = true;
            if (addr == 0xA001) m_m254Xor = data;
        }
        uint16_t regAddr = addr & 0xE001;
        if (m_config.id == 196) {
            if (addr >= 0xC000)
                regAddr = uint16_t((addr & 0xFFFE) | ((addr >> 2) & 0x01) | ((addr >> 3) & 0x01));
            else
                regAddr = uint16_t((addr & 0xFFFE) | ((addr >> 2) & 0x01) | ((addr >> 3) & 0x01) | ((addr >> 1) & 0x01));
            regAddr &= 0xE001;
        } else if (m_config.id == 250) {
            data = uint8_t(addr & 0xFF);
            regAddr = uint16_t((addr & 0xE000) | ((addr & 0x0400) >> 10));
        }
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
            if (m_config.id == 245 && (m_bankSelect & 7) <= 5)
                m_m245PrgOuter = uint8_t((data >> 1) & 1);
            break;
        case 0xA000:
            if (m_config.id != 118 && !(m_config.id == 208 && m_config.submapper == 0) && !m_config.fourScreen)
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
        case 0xC000: m_irqLatch = (m_config.id == 534) ? uint8_t(data ^ 0xFF) : data; break;
        case 0xC001: m_irqReload = true; break;
        case 0xE000: m_irqEnabled = false; m_irqPending = false; break;
        case 0xE001: m_irqEnabled = true; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override
    {
        if (addr >= 0x2000) return false;
        if (m_config.id == 219) {
            if (!m_config.chrRomSize) return false;
            std::size_t bank = 0;
            if (m_m219Extended) bank = m_m219Chr[(addr >> 10) & 7] & 0x7F;
            else bank = chrBankForAddress(addr) & 0x7F;
            bank |= std::size_t(m_m219Outer & 3) << 7;
            mapped = mapBank(bank, 0x400, m_config.chrRomSize, addr & 0x03FF);
            return true;
        }
        if (m_config.id == 165) {
            const bool upper = (addr & 0x1000) != 0;
            const uint8_t reg = upper ? (m_m165Latch[1] ? m_regs[4] : m_regs[2])
                                      : (m_m165Latch[0] ? m_regs[1] : m_regs[0]);
            const bool useRam = reg == 0 && m_config.chrRamSize != 0;
            const std::size_t size = useRam ? m_config.chrRamSize : m_config.chrRomSize;
            if (!size) return false;
            const std::size_t page4k = useRam ? 0 : (std::size_t(reg) >> 2);
            mapped = mapBank(page4k, 0x1000, size, addr & 0x0FFF);
            switch (addr & 0x2FF8) {
            case 0x0FD0: m_m165Latch[0] = false; break;
            case 0x0FE8: m_m165Latch[0] = true; break;
            case 0x1FD0: m_m165Latch[1] = false; break;
            case 0x1FE8: m_m165Latch[1] = true; break;
            default: break;
            }
            return true;
        }
        if ((m_config.id == 198 || m_config.id == 199 || m_config.id == 245) &&
            m_config.chrRomSize == 0 && m_config.chrRamSize != 0) {
            mapped = static_cast<uint32_t>(addr % m_config.chrRamSize);
            return true;
        }
        std::size_t bank = chrBankForAddress(addr);
        if (is126Family()) {
            if (m_m126Ex[3] & 0x10) {
                std::size_t base = std::size_t(m_m126Ex[2] & 0x0F) << 3;

                if ((m_m126Ex[0] & 0x80) == 0)
                    base |= std::size_t(m_m126Ex[2] & 0x80);
                base = transform126ChrBank(base);
                bank = base + ((addr >> 10) & 7);
            } else {
                bank = transform126ChrBank(bank);
            }
        }
        const bool useRam = (is126Family() && m_config.chrRomSize == 0) || chrBankUsesRam(bank);
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
        if (m_config.id == 165) {

            mapped = static_cast<uint32_t>((addr & 0x0FFF) % m_config.chrRamSize);
            return true;
        }
        return ppuMapRead(addr, mapped);
    }

    bool ppuUsesChrRam(uint16_t addr) const override
    {
        if (addr >= 0x2000 || m_config.chrRamSize == 0) return false;
        if (m_config.id == 165) {
            const bool upper=(addr&0x1000)!=0;
            const uint8_t reg=upper?(m_m165Latch[1]?m_regs[4]:m_regs[2]):(m_m165Latch[0]?m_regs[1]:m_regs[0]);
            return reg==0;
        }
        if ((m_config.id == 198 || m_config.id == 199 || m_config.id == 245) && m_config.chrRomSize == 0)
            return true;
        if (is126Family() && m_config.chrRomSize == 0) return true;
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
        if ((m_config.id == 198 || m_config.id == 199) && addr >= 0x5000 && addr <= 0x7FFF && m_config.prgRamSize >= 0x3000) {
            if (addr >= 0x6000 && (!m_prgRamEnable || (write && m_prgRamWriteProtect))) return false;
            mapped = static_cast<uint32_t>(addr - 0x5000);
            return true;
        }
        if (m_mmc6 || m_config.id == 37 || m_config.id == 47 || m_config.id == 49 ||
            m_config.id == 114 || m_config.id == 115 || (m_config.id == 197 && m_config.submapper == 3)) return false;
        if (addr < 0x6000 || addr > 0x7FFF || m_config.prgRamSize == 0)
            return false;
        if ((!m_prgRamEnable && !is126Family()) || (write && m_prgRamWriteProtect))
            return false;
        if (m_config.id == 52 && write && (m_outerReg & 0x80) == 0)
            return false;
        mapped = static_cast<uint32_t>((addr - 0x6000) % m_config.prgRamSize);
        return true;
    }

    uint8_t transformPrgRamRead(uint16_t addr, uint8_t data) const override
    {
        if (m_config.id == 254 && addr >= 0x6000 && addr <= 0x7FFF && !m_m254Unlocked)
            return uint8_t(data ^ m_m254Xor);
        return data;
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
                m_a12LowStartPpuCycle = ppuCycle;
                m_a12LowValid = true;
            }
        } else {
            if (!m_lastA12 && m_a12LowValid &&
                ppuCycle - m_a12LowStartPpuCycle >= 9) {
                clockIrq();
            }
            m_a12LowValid = false;
            m_a12LowStartPpuCycle = 0;
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
        if (m_config.id == 245 && hard) m_m245PrgOuter = 0;
        if (hard) {
            if (m_config.id == 14) { m_m14Mode = 0; m_m14Mirror = 0; std::fill(std::begin(m_m14Prg), std::end(m_m14Prg), uint8_t(0)); std::fill(std::begin(m_m14Chr), std::end(m_m14Chr), uint8_t(0)); }
            if (m_config.id == 134) m_m134ExReg = 0;
            if (m_config.id == 224) m_m224Outer = 0;
            if (m_config.id == 238) m_m238ExReg = 0;
            if (m_config.id == 189) m_m189PrgBank = 0;
            if (m_config.id == 196) { m_m196Override = false; m_m196PrgBank = 0; }
            if (m_config.id == 205) m_m205Block = 0;
            if (m_config.id == 249) m_m249ExReg = 0;
            if (m_config.id == 259) m_m259ExReg = 0;
            if (m_config.id == 254) { m_m254Unlocked = false; m_m254Xor = 0; }
            if (m_config.id == 219) reset219();
            if (is126Family()) std::fill(std::begin(m_m126Ex), std::end(m_m126Ex), uint8_t(0));
            if (m_config.id == 165) { m_m165Latch[0]=false; m_m165Latch[1]=false; }
            if (m_config.id == 187) { m_m187Ex0=0; m_m187Ex1=0; }
        }
        if (m_config.id == 208 && hard) {
            m_m208PrgMirror = 0x11;
            m_m208ProtectionIndex = 0;
            std::fill(std::begin(m_m208Protection), std::end(m_m208Protection), uint8_t(0));
            if (m_config.submapper == 0 && !m_config.fourScreen) m_mirror = Mirror::Vertical;
        }
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
        put8(out, m_lastA12 ? 1 : 0); put8(out, m_a12LowValid ? 1 : 0); put64(out, m_a12LowStartPpuCycle);
        put8(out, m_prgRamEnable ? 1 : 0); put8(out, m_prgRamWriteProtect ? 1 : 0);
        put8(out, m_outerReg);
        put8(out, m_extMode); put8(out, m_extChr);
        if (m_config.id == 245) put8(out, m_m245PrgOuter);
        if (m_config.id == 189) put8(out, m_m189PrgBank);
        if (m_config.id == 196) { put8(out, m_m196Override ? 1 : 0); put8(out, m_m196PrgBank); }
        if (m_config.id == 205) put8(out, m_m205Block);
        if (m_config.id == 249) put8(out, m_m249ExReg);
        if (m_config.id == 259) put8(out, m_m259ExReg);
        if (m_config.id == 254) { put8(out, m_m254Unlocked ? 1 : 0); put8(out, m_m254Xor); }
        if (m_config.id == 219) {
            put8(out, m_m219Outer); put8(out, m_m219Selector); put8(out, m_m219Extended ? 1 : 0); put8(out, m_m219ChrLatch);
            for (uint8_t v : m_m219Prg) put8(out, v);
            for (uint8_t v : m_m219Chr) put8(out, v);
        }
        if (is126Family()) for (uint8_t v : m_m126Ex) put8(out, v);
        if (m_config.id == 14) { put8(out, m_m14Mode); put8(out, m_m14Mirror); for (uint8_t v : m_m14Prg) put8(out, v); for (uint8_t v : m_m14Chr) put8(out, v); }
        if (m_config.id == 134) put8(out, m_m134ExReg);
        if (m_config.id == 224) put8(out, m_m224Outer);
        if (m_config.id == 238) put8(out, m_m238ExReg);
        if (m_config.id == 208) {
            put8(out, m_m208PrgMirror); put8(out, m_m208ProtectionIndex);
            for (uint8_t value : m_m208Protection) put8(out, value);
        }
        if (m_config.id == 12) put8(out, m_m12ChrOuter);
        if (m_config.id == 45) {
            put8(out, m_m45Index);
            for (uint8_t value : m_m45Outer) put8(out, value);
        }
        if (m_config.id == 215) { put8(out, m_m215Mode); put8(out, m_m215Outer); put8(out, m_m215Scramble); }
        if (m_config.id == 123) put8(out, m_m123Mode);
        if (m_config.id == 121) for (uint8_t value : m_m121Ex) put8(out, value);
        if (m_config.id == 165) { put8(out, m_m165Latch[0] ? 1 : 0); put8(out, m_m165Latch[1] ? 1 : 0); }
        if (m_config.id == 187) { put8(out, m_m187Ex0); put8(out, m_m187Ex1); }
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
        if (!get64(p, end, m_a12LowStartPpuCycle)) return false;

        if (m_a12LowValid && m_a12LowStartPpuCycle <= 3) {
            m_a12LowValid = false;
            m_a12LowStartPpuCycle = 0;
        }
        if (!get8(p, end, tmp)) return false;
        m_prgRamEnable = tmp != 0;
        if (!get8(p, end, tmp)) return false;
        m_prgRamWriteProtect = tmp != 0;
        if (!get8(p, end, m_outerReg)) return false;
        if (!get8(p, end, m_extMode) || !get8(p, end, m_extChr)) return false;
        if (m_config.id == 245 && !get8(p, end, m_m245PrgOuter)) return false;
        if (m_config.id == 189 && !get8(p, end, m_m189PrgBank)) return false;
        if (m_config.id == 196) {
            if (!get8(p, end, tmp)) return false;
            m_m196Override = tmp != 0;
            if (!get8(p, end, m_m196PrgBank)) return false;
        }
        if (m_config.id == 205 && !get8(p, end, m_m205Block)) return false;
        if (m_config.id == 249 && !get8(p, end, m_m249ExReg)) return false;
        if (m_config.id == 259 && !get8(p, end, m_m259ExReg)) return false;
        if (m_config.id == 254) { if (!get8(p,end,tmp)) return false; m_m254Unlocked = tmp != 0; if (!get8(p,end,m_m254Xor)) return false; }
        if (m_config.id == 219) {
            if (!get8(p,end,m_m219Outer) || !get8(p,end,m_m219Selector) || !get8(p,end,tmp)) return false;
            m_m219Extended = tmp != 0;
            if (!get8(p,end,m_m219ChrLatch)) return false;
            for (uint8_t& v : m_m219Prg) if (!get8(p,end,v)) return false;
            for (uint8_t& v : m_m219Chr) if (!get8(p,end,v)) return false;
        }
        if (is126Family()) for (uint8_t& v : m_m126Ex) if (!get8(p,end,v)) return false;
        if (m_config.id == 14) { if (!get8(p,end,m_m14Mode) || !get8(p,end,m_m14Mirror)) return false; for (uint8_t& v : m_m14Prg) if(!get8(p,end,v)) return false; for (uint8_t& v : m_m14Chr) if(!get8(p,end,v)) return false; }
        if (m_config.id == 134 && !get8(p,end,m_m134ExReg)) return false;
        if (m_config.id == 224 && !get8(p,end,m_m224Outer)) return false;
        if (m_config.id == 238 && !get8(p,end,m_m238ExReg)) return false;
        if (m_config.id == 208) {
            if (!get8(p, end, m_m208PrgMirror) || !get8(p, end, m_m208ProtectionIndex)) return false;
            for (uint8_t& value : m_m208Protection) if (!get8(p, end, value)) return false;
        }
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
        if (m_config.id == 165) {
            if (!get8(p, end, tmp)) return false;
            m_m165Latch[0] = tmp != 0;
            if (!get8(p, end, tmp)) return false;
            m_m165Latch[1] = tmp != 0;
        }
        if (m_config.id == 187) { if (!get8(p,end,m_m187Ex0) || !get8(p,end,m_m187Ex1)) return false; }
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
    uint64_t m_a12LowStartPpuCycle = 0;
    bool m_prgRamEnable = true;
    bool m_prgRamWriteProtect = false;
    uint8_t m_outerReg = 0;
    uint8_t m_extMode = 0;
    uint8_t m_extChr = 0;
    uint8_t m_m245PrgOuter = 0;
    uint8_t m_m189PrgBank = 0;
    bool m_m196Override = false;
    uint8_t m_m196PrgBank = 0;
    uint8_t m_m205Block = 0;
    uint8_t m_m249ExReg = 0;
    uint8_t m_m259ExReg = 0;
    bool m_m254Unlocked = false;
    uint8_t m_m254Xor = 0;
    uint8_t m_m14Mode = 0;
    uint8_t m_m14Mirror = 0;
    uint8_t m_m14Prg[2] = {};
    uint8_t m_m14Chr[8] = {};
    uint8_t m_m134ExReg = 0;
    uint8_t m_m224Outer = 0;
    uint8_t m_m238ExReg = 0;
    uint8_t m_m219Outer = 3;
    uint8_t m_m219Selector = 0;
    bool m_m219Extended = false;
    uint8_t m_m219ChrLatch = 0;
    uint8_t m_m219Prg[4] = {12,13,14,15};
    uint8_t m_m219Chr[8] = {0,1,2,3,4,5,6,7};
    bool m_m165Latch[2] = {false,false};
    uint8_t m_m187Ex0 = 0, m_m187Ex1 = 0;
    uint8_t m_m208PrgMirror = 0x11;
    uint8_t m_m208ProtectionIndex = 0;
    uint8_t m_m208Protection[4] = {};
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
    uint8_t m_m126Ex[4] = {};

    bool is126Family() const
    {
        return m_config.id == 126 || m_config.id == 422 || m_config.id == 534;
    }

    std::size_t transform126PrgBank(std::size_t bank) const
    {

        const uint8_t reg = m_m126Ex[0];
        if (reg & 0x40)
            bank = (bank & ~std::size_t(0x10)) | (std::size_t(reg & 0x01) << 4);
        bank = (bank & 0x1F) | (std::size_t(reg & 0x06) << 4);
        bank |= std::size_t(reg & 0x10) << 3;
        bank |= std::size_t((~reg >> 5) & 0x01) << 8;
        return bank;
    }

    std::size_t transform126ChrBank(std::size_t bank) const
    {

        const uint8_t reg = m_m126Ex[0];
        if (reg & 0x80)
            bank = (bank & ~std::size_t(0x80)) | (std::size_t((reg >> 3) & 1) << 7);
        bank = (bank & 0xFF) | (std::size_t((reg >> 3) & 1) << 10);
        const std::size_t bit4 = (reg >> 4) & 1;
        const std::size_t bit5 = (reg >> 5) & 1;
        if (m_config.id == 126) {
            bank |= bit5 << 8;
            bank |= bit4 << 9;
        } else {
            bank |= bit4 << 8;
            bank |= bit5 << 9;
        }
        return bank;
    }

    void reset219()
    {
        m_m219Outer = 3;
        m_m219Selector = 0;
        m_m219Extended = false;
        m_m219ChrLatch = 0;
        m_m219Prg[0]=12; m_m219Prg[1]=13; m_m219Prg[2]=14; m_m219Prg[3]=15;
        for (uint8_t i=0;i<8;++i) m_m219Chr[i]=i;
    }

    void write219Extended(uint8_t data)
    {
        const uint8_t sel = m_m219Selector & 0x3F;
        if (sel >= 0x23 && sel <= 0x26) {
            const uint8_t bank = uint8_t(((data & 0x20) >> 5) | ((data & 0x10) >> 3) | ((data & 0x08) >> 1) | ((data & 0x04) << 1));
            m_m219Prg[0x26 - sel] = bank;
            return;
        }
        switch (sel) {
        case 0x08: case 0x0A: case 0x0E: case 0x12: case 0x16: case 0x1A: case 0x1E:
            m_m219ChrLatch = uint8_t((data & 0x07) << 4);
            return;
        case 0x09: m_m219Chr[0] = uint8_t(m_m219ChrLatch | ((data >> 1) & 0x0E)); return;
        case 0x0B: m_m219Chr[1] = uint8_t(m_m219ChrLatch | ((data >> 1) | 0x01)); return;
        case 0x0C: case 0x0D: m_m219Chr[2] = uint8_t(m_m219ChrLatch | ((data >> 1) & 0x0E)); return;
        case 0x0F: m_m219Chr[3] = uint8_t(m_m219ChrLatch | ((data >> 1) | 0x01)); return;
        case 0x10: case 0x11: m_m219Chr[4] = uint8_t(m_m219ChrLatch | ((data >> 1) & 0x0F)); return;
        case 0x14: case 0x15: m_m219Chr[5] = uint8_t(m_m219ChrLatch | ((data >> 1) & 0x0F)); return;
        case 0x18: case 0x19: m_m219Chr[6] = uint8_t(m_m219ChrLatch | ((data >> 1) & 0x0F)); return;
        case 0x1C: case 0x1D: m_m219Chr[7] = uint8_t(m_m219ChrLatch | ((data >> 1) & 0x0F)); return;
        default: return;
        }
    }

    static std::size_t scramble249Prg(std::size_t page)
    {
        if (page < 0x20) {
            return (page & 0x01) | ((page >> 3) & 0x02) | ((page >> 1) & 0x04) | ((page << 2) & 0x18);
        }
        page -= 0x20;
        return (page & 0x03) | ((page >> 1) & 0x04) | ((page >> 4) & 0x08) |
            ((page >> 2) & 0x10) | ((page << 3) & 0x20) | ((page << 2) & 0xC0);
    }

    static std::size_t scramble249Chr(std::size_t page)
    {
        return (page & 0x03) | ((page >> 1) & 0x04) | ((page >> 4) & 0x08) |
            ((page >> 2) & 0x10) | ((page << 3) & 0x20) | ((page << 2) & 0xC0);
    }

    std::size_t chrBankForAddress(uint16_t addr) const
    {
        if (m_config.id == 14 && (m_m14Mode & 0x02) == 0)
            return m_m14Chr[(addr >> 10) & 7];
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
        else if (m_config.id == 205) {
            if (m_m205Block >= 2) bank = (bank & 0x7F) | 0x100;
            if (m_m205Block == 1 || m_m205Block == 3) bank |= 0x80;
        }
        else if (m_config.id == 14) {
            const uint8_t slot = uint8_t((addr >> 10) & 7);
            uint8_t outer = 0;
            if (!m_chrMode) outer = slot < 4 ? ((m_m14Mode & 0x08) ? 1 : 0) : (slot < 6 ? ((m_m14Mode & 0x20) ? 1 : 0) : ((m_m14Mode & 0x80) ? 1 : 0));
            else outer = slot < 2 ? ((m_m14Mode & 0x20) ? 1 : 0) : (slot < 4 ? ((m_m14Mode & 0x80) ? 1 : 0) : ((m_m14Mode & 0x08) ? 1 : 0));
            bank = (bank & 0xFF) | (std::size_t(outer) << 8);
        }
        else if (m_config.id == 134) bank = (bank & 0xFF) | (std::size_t(m_m134ExReg & 0x20) << 3);
        else if (m_config.id == 249 && (m_m249ExReg & 0x02)) {
            bank = scramble249Chr(bank);
        }
        if (m_config.id == 187) {
            const uint8_t slot=uint8_t((addr>>10)&7);
            if ((m_chrMode && slot>=4) || (!m_chrMode && slot<4)) bank |= 0x100;
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

    static uint8_t mapper208ProtectionLut(uint8_t index)
    {
        static constexpr uint8_t lut[256] = {
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x49,0x19,0x09,0x59,0x49,0x19,0x09,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x51,0x41,0x11,0x01,0x51,0x41,0x11,0x01,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x49,0x19,0x09,0x59,0x49,0x19,0x09,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x51,0x41,0x11,0x01,0x51,0x41,0x11,0x01,
            0x00,0x10,0x40,0x50,0x00,0x10,0x40,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x08,0x18,0x48,0x58,0x08,0x18,0x48,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x00,0x10,0x40,0x50,0x00,0x10,0x40,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x08,0x18,0x48,0x58,0x08,0x18,0x48,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x58,0x48,0x18,0x08,0x58,0x48,0x18,0x08,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x50,0x40,0x10,0x00,0x50,0x40,0x10,0x00,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x58,0x48,0x18,0x08,0x58,0x48,0x18,0x08,
            0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x59,0x50,0x40,0x10,0x00,0x50,0x40,0x10,0x00,
            0x01,0x11,0x41,0x51,0x01,0x11,0x41,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x09,0x19,0x49,0x59,0x09,0x19,0x49,0x59,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x01,0x11,0x41,0x51,0x01,0x11,0x41,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x09,0x19,0x49,0x59,0x09,0x19,0x49,0x59,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
        };
        return lut[index];
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
