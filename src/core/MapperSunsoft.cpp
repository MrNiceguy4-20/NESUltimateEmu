#include "MapperFamilies.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {
void put8(std::vector<uint8_t>& out, uint8_t value){ out.push_back(value); }
void put64(std::vector<uint8_t>& out, uint64_t value){ for(int i=0;i<8;++i) put8(out, static_cast<uint8_t>(value>>(i*8))); }
bool get8(const uint8_t*& p,const uint8_t* end,uint8_t& value){ if(p>=end)return false; value=*p++; return true; }
bool get64(const uint8_t*& p,const uint8_t* end,uint64_t& value){ if(end-p<8)return false; value=0; for(int i=0;i<8;++i)value|=uint64_t(*p++)<<(i*8); return true; }
uint32_t mapBank(std::size_t bank,std::size_t bankSize,std::size_t totalSize,uint32_t inBank){ const std::size_t count=std::max<std::size_t>(1,totalSize/bankSize); return static_cast<uint32_t>((bank%count)*bankSize+inBank); }
namespace mapper_more_detail {
inline void setMirror4(Mirror& m, uint8_t v) {
    switch(v & 3) { case 0: m=Mirror::Vertical; break; case 1: m=Mirror::Horizontal; break;
        case 2: m=Mirror::OnescreenLo; break; default: m=Mirror::OnescreenHi; break; }
}
}

class Mapper67 final : public Mapper {public:explicit Mapper67(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF800){case 0x8800:m_chr[0]=d;return true;case 0x9800:m_chr[1]=d;return true;case 0xA800:m_chr[2]=d;return true;case 0xB800:m_chr[3]=d;return true;case 0xC800:if(!m_irqToggle){m_counter=uint16_t((m_counter&0x00FF)|(uint16_t(d)<<8));m_irqToggle=true;}else{m_counter=uint16_t((m_counter&0xFF00)|d);m_irqToggle=false;}m_pending=false;return true;case 0xD800:m_enabled=(d&0x10)!=0;m_pending=false;return true;case 0xE800:mapper_more_detail::setMirror4(m_mirror,d);return true;case 0xF800:m_prg=d&0x0F;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void clockCpu()override{if(m_enabled){if(m_counter==0xFFFF){m_pending=true;m_enabled=false;}++m_counter;}}bool irqActive()const override{return m_pending;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(auto v:m_chr)put8(o,v);put8(o,uint8_t(m_counter));put8(o,uint8_t(m_counter>>8));put8(o,m_enabled);put8(o,m_pending);put8(o,m_irqToggle);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t l,h,b;if(!get8(p,e,m_prg))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,l)||!get8(p,e,h))return false;m_counter=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_pending=b;if(!get8(p,e,b))return false;m_irqToggle=b;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr[4]={};uint16_t m_counter=0;bool m_enabled=false,m_pending=false,m_irqToggle=false;};

class Mapper68 final : public Mapper {
public:
    explicit Mapper68(const MapperConfig& c)
        : Mapper(c), m_dualCart(c.nes20 && c.submapper == 1) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;

        if (a < 0xC000) {
            if (m_dualCart && m_external) {

                if (m_timer == 0) return false;

                constexpr std::size_t internalSize = 0x20000;
                if (m_config.prgRomSize <= internalSize) return false;
                const std::size_t externalSize = m_config.prgRomSize - internalSize;
                const std::size_t off = (std::size_t(m_prg & 7) * 0x4000 + (a - 0x8000)) % externalSize;
                m = uint32_t(internalSize + off);
                return true;
            }

            const uint8_t bank = m_dualCart ? uint8_t(m_prg & 7) : uint8_t(m_prg & 0x0F);
            m = mapBank(bank, 0x4000, m_config.prgRomSize, a - 0x8000);
            return true;
        }

        std::size_t bank = 0;
        if (m_dualCart) {
            const std::size_t internalBanks = std::max<std::size_t>(1, std::min<std::size_t>(m_config.prgRomSize, 0x20000) / 0x4000);
            bank = internalBanks - 1;
        } else {
            const std::size_t banks = std::max<std::size_t>(1, m_config.prgRomSize / 0x4000);
            bank = banks - 1;
        }
        m = mapBank(bank, 0x4000, m_config.prgRomSize, a - 0xC000);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a >= 0x6000 && a <= 0x7FFF) {

            if (m_dualCart && !m_ramEnable && m_external)
                m_timer = 1024u * 105u;
            return false;
        }
        if (a < 0x8000) return false;
        switch (a & 0xF000) {
        case 0x8000: m_chr[0] = d; return true;
        case 0x9000: m_chr[1] = d; return true;
        case 0xA000: m_chr[2] = d; return true;
        case 0xB000: m_chr[3] = d; return true;
        case 0xC000: m_nt[0] = d | 0x80; return true;
        case 0xD000: m_nt[1] = d | 0x80; return true;
        case 0xE000:
            mapper_more_detail::setMirror4(m_mirror, d);
            m_chrNt = (d & 0x10) != 0;
            return true;
        case 0xF000:
            m_ramEnable = (d & 0x10) != 0;
            if (m_dualCart) {
                m_external = (d & 0x08) == 0;
                m_prg = d & 0x07;
                if (!m_external) m_timer = 0;
            } else {
                m_external = false;
                m_timer = 0;
                m_prg = d & 0x0F;
            }
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const auto s = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!s) return false;
        m = mapBank(m_chr[a >> 11], 0x800, s, a & 0x7FF);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a, m); }

    bool mapPrgRam(uint16_t a, uint32_t& m, bool) const override {
        if (!m_ramEnable || a < 0x6000 || a > 0x7FFF || !m_config.prgRamSize) return false;
        m = (a - 0x6000) % m_config.prgRamSize;
        return true;
    }

    bool mapNametable(uint16_t a, NametableSource& s, uint32_t& m) const override {
        if (!m_chrNt || a < 0x2000 || a > 0x3EFF) return false;
        const uint8_t p = uint8_t((a & 0x0FFF) >> 10);
        uint8_t r = 0;
        switch (m_mirror) {
        case Mirror::Vertical: r = p & 1; break;
        case Mirror::Horizontal: r = (p >> 1) & 1; break;
        case Mirror::OnescreenLo: r = 0; break;
        case Mirror::OnescreenHi: r = 1; break;
        case Mirror::FourScreen: r = p & 1; break;
        }
        s = m_config.chrRomSize ? NametableSource::ChrRom : NametableSource::ChrRam;
        m = uint32_t(m_nt[r]) * 0x400 + (a & 0x3FF);
        return true;
    }

    void clockCpu() override { if (m_timer) --m_timer; }

    void reset(bool hard) override {
        if (!hard) return;
        m_prg = 0;
        std::fill(std::begin(m_chr), std::end(m_chr), 0);
        std::fill(std::begin(m_nt), std::end(m_nt), 0);
        m_chrNt = false;
        m_ramEnable = false;
        m_external = false;
        m_timer = 0;
        m_mirror = m_config.headerMirror;
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_prg); for (auto v : m_chr) put8(o, v); for (auto v : m_nt) put8(o, v);
        put8(o, m_chrNt); put8(o, m_ramEnable); put8(o, m_external); put64(o, m_timer);
        put8(o, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t b;
        if (!get8(p,e,m_prg)) return false;
        for (auto& v : m_chr) if (!get8(p,e,v)) return false;
        for (auto& v : m_nt) if (!get8(p,e,v)) return false;
        if (!get8(p,e,b)) return false;
        m_chrNt = b != 0;
        if (!get8(p,e,b)) return false;
        m_ramEnable = b != 0;
        if (!get8(p,e,b)) return false;
        m_external = b != 0;
        if (!get64(p,e,m_timer) || !get8(p,e,b)) return false;
        m_mirror = static_cast<Mirror>(b);
        return true;
    }

private:
    uint8_t m_prg = 0, m_chr[4] = {}, m_nt[2] = {};
    bool m_chrNt = false, m_ramEnable = false, m_external = false;
    bool m_dualCart = false;
    uint64_t m_timer = 0;
};

class Mapper69 final : public Mapper {
public:
    explicit Mapper69(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a >= 0x6000 && a < 0x8000) {
            if ((m_reg8 & 0x40) == 0) {
                m = mapBank(m_reg8 & 0x3F, 0x2000, m_config.prgRomSize, a - 0x6000);
                return true;
            }
            return false;
        }
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const size_t n = std::max<size_t>(1, m_config.prgRomSize / 0x2000);
        const size_t bank = a < 0xA000 ? m_prg[0] : a < 0xC000 ? m_prg[1] : a < 0xE000 ? m_prg[2] : n - 1;
        m = mapBank(bank, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a >= 0x8000 && a < 0xA000) {
            m_cmd = d & 0x0F;
            return true;
        }
        if (a >= 0xA000 && a < 0xC000) {
            switch (m_cmd) {
            case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7:
                m_chr[m_cmd] = d; break;
            case 8: m_reg8 = d; break;
            case 9: case 10: case 11: m_prg[m_cmd - 9] = d & 0x3F; break;
            case 12: mapper_more_detail::setMirror4(m_mirror, d); break;
            case 13: {

                const bool wasCounterEnabled = m_counterEnable;
                m_pending = false;
                m_counterEnable = (d & 0x80) != 0;
                m_irqEnable = (d & 1) != 0;
                if (!wasCounterEnabled && m_counterEnable) clockIrqCounter();
                break;
            }
            case 14: m_counter = uint16_t((m_counter & 0xFF00) | d); break;
            case 15: m_counter = uint16_t((m_counter & 0x00FF) | (uint16_t(d) << 8)); break;
            }
            return true;
        }
        if (a >= 0xC000 && a < 0xE000) {
            m_audioSelect = d & 0x0F;
            return true;
        }
        if (a >= 0xE000) {
            writeAudioReg(m_audioSelect, d);
            return true;
        }
        return false;
    }

    bool mapPrgRam(uint16_t a, uint32_t& m, bool) const override {
        if (a < 0x6000 || a > 0x7FFF || !m_config.prgRamSize || (m_reg8 & 0x40) == 0 || (m_reg8 & 0x80) == 0)
            return false;
        m = mapBank(m_reg8 & 0x3F, 0x2000, m_config.prgRamSize, a - 0x6000);
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const auto s = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!s) return false;
        m = mapBank(m_chr[a >> 10], 0x400, s, a & 0x3FF);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a, m); }

    void clockCpu() override {
        clockIrqCounter();

        if (++m_psgDivider >= 8) {
            m_psgDivider = 0;
            clockPsg();
        }
    }

    bool irqActive() const override { return m_pending; }

    float expansionAudioSample(bool = false) const override {
        const int sum = int(m_psgOut[0]) + int(m_psgOut[1]) + int(m_psgOut[2]);

        return -(float(sum) / (255.0f * 3.0f)) * 0.64f;
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_cmd); put8(o, m_reg8);
        for (auto v : m_prg) put8(o, v);
        for (auto v : m_chr) put8(o, v);
        put16(o, m_counter);
        put8(o, m_counterEnable); put8(o, m_irqEnable); put8(o, m_pending);
        put8(o, static_cast<uint8_t>(m_mirror));
        put8(o, m_audioSelect);
        for (auto v : m_audioReg) put8(o, v);
        put8(o, m_psgDivider);
        for (auto v : m_toneCount) put16(o, v);
        for (auto v : m_toneFreq) put16(o, v);
        for (bool v : m_toneEdge) put8(o, v);
        put16(o, m_noiseCount);
        put32(o, m_noiseSeed);
        put8(o, m_noiseFreq);
        put32(o, m_envCount);
        put16(o, m_envFreq);
        put8(o, m_envPtr);
        put8(o, m_envContinue); put8(o, m_envAttack); put8(o, m_envAlternate);
        put8(o, m_envHold); put8(o, m_envFace); put8(o, m_envPause);
        for (auto v : m_psgOut) put8(o, v);
    }

    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t b = 0;
        if (!get8(p, e, m_cmd) || !get8(p, e, m_reg8)) return false;
        for (auto& v : m_prg) if (!get8(p, e, v)) return false;
        for (auto& v : m_chr) if (!get8(p, e, v)) return false;
        if (!get16(p, e, m_counter)) return false;
        if (!get8(p, e, b)) return false;
        m_counterEnable = b != 0;
        if (!get8(p, e, b)) return false;
        m_irqEnable = b != 0;
        if (!get8(p, e, b)) return false;
        m_pending = b != 0;
        if (!get8(p, e, b)) return false;
        m_mirror = static_cast<Mirror>(b);
        if (!get8(p, e, m_audioSelect)) return false;
        for (auto& v : m_audioReg) if (!get8(p, e, v)) return false;
        if (!get8(p, e, m_psgDivider)) return false;
        for (auto& v : m_toneCount) if (!get16(p, e, v)) return false;
        for (auto& v : m_toneFreq) if (!get16(p, e, v)) return false;
        for (auto& v : m_toneEdge) { if (!get8(p, e, b)) return false; v = b != 0; }
        if (!get16(p, e, m_noiseCount) || !get32(p, e, m_noiseSeed) || !get8(p, e, m_noiseFreq)) return false;
        if (!get32(p, e, m_envCount) || !get16(p, e, m_envFreq) || !get8(p, e, m_envPtr)) return false;
        if (!get8(p, e, b)) return false;
        m_envContinue = b != 0;
        if (!get8(p, e, b)) return false;
        m_envAttack = b != 0;
        if (!get8(p, e, b)) return false;
        m_envAlternate = b != 0;
        if (!get8(p, e, b)) return false;
        m_envHold = b != 0;
        if (!get8(p, e, b)) return false;
        m_envFace = b != 0;
        if (!get8(p, e, b)) return false;
        m_envPause = b != 0;
        for (auto& v : m_psgOut) if (!get8(p, e, v)) return false;
        return true;
    }

private:
    uint8_t m_cmd = 0, m_reg8 = 0, m_prg[3]{}, m_chr[8]{};
    uint16_t m_counter = 0;
    bool m_counterEnable = false, m_irqEnable = false, m_pending = false;

    uint8_t m_audioSelect = 0, m_audioReg[16]{};
    uint8_t m_psgDivider = 0;
    uint16_t m_toneCount[3]{0x1000, 0x1000, 0x1000};
    uint16_t m_toneFreq[3]{};
    bool m_toneEdge[3]{};
    uint16_t m_noiseCount = 0x40;
    uint32_t m_noiseSeed = 0xFFFF;
    uint8_t m_noiseFreq = 0;
    uint32_t m_envCount = 0;
    uint16_t m_envFreq = 0;
    uint8_t m_envPtr = 0;
    bool m_envContinue = false, m_envAttack = false, m_envAlternate = false;
    bool m_envHold = false, m_envFace = false, m_envPause = true;
    uint8_t m_psgOut[3]{};

    void clockIrqCounter() {
        if (!m_counterEnable) return;

        if (m_counter == 0) {
            m_counter = 0xFFFF;
            if (m_irqEnable) m_pending = true;
            return;
        }

        --m_counter;
        if (m_counter == 0 && m_irqEnable) m_pending = true;
    }

    static void put16(std::vector<uint8_t>& o, uint16_t v) {
        put8(o, uint8_t(v)); put8(o, uint8_t(v >> 8));
    }
    static bool get16(const uint8_t*& p, const uint8_t* e, uint16_t& v) {
        uint8_t lo = 0, hi = 0;
        if (!get8(p, e, lo) || !get8(p, e, hi)) return false;
        v = uint16_t(lo) | (uint16_t(hi) << 8);
        return true;
    }
    static void put32(std::vector<uint8_t>& o, uint32_t v) {
        for (int i = 0; i < 4; ++i) put8(o, uint8_t(v >> (i * 8)));
    }
    static bool get32(const uint8_t*& p, const uint8_t* e, uint32_t& v) {
        v = 0;
        for (int i = 0; i < 4; ++i) {
            uint8_t b = 0;
            if (!get8(p, e, b)) return false;
            v |= uint32_t(b) << (i * 8);
        }
        return true;
    }

    static uint8_t volumeLevel(uint8_t index) {
        static constexpr uint8_t kVol[32] = {
            0x00,0x01,0x01,0x02,0x02,0x03,0x03,0x04,
            0x05,0x06,0x07,0x09,0x0B,0x0D,0x0F,0x12,
            0x16,0x1A,0x1F,0x25,0x2D,0x35,0x3F,0x4C,
            0x5A,0x6A,0x7F,0x97,0xB4,0xD6,0xEB,0xFF
        };
        return kVol[index & 31];
    }

    void writeAudioReg(uint8_t reg, uint8_t value) {
        reg &= 0x0F;
        m_audioReg[reg] = value;
        switch (reg) {
        case 0: case 1: case 2: case 3: case 4: case 5: {
            const int ch = reg >> 1;
            m_toneFreq[ch] = uint16_t(((m_audioReg[ch * 2 + 1] & 0x0F) << 8) | m_audioReg[ch * 2]);
            break;
        }
        case 6:
            m_noiseFreq = value == 0 ? 1 : uint8_t((value & 31) << 1);
            break;
        case 11: case 12:
            m_envFreq = uint16_t((uint16_t(m_audioReg[12]) << 8) | m_audioReg[11]);
            break;
        case 13:
            m_envContinue = ((value >> 3) & 1) != 0;
            m_envAttack = ((value >> 2) & 1) != 0;
            m_envAlternate = ((value >> 1) & 1) != 0;
            m_envHold = (value & 1) != 0;
            m_envFace = m_envAttack;
            m_envPause = false;
            m_envCount = 0x10000u - m_envFreq;
            m_envPtr = m_envFace ? 0 : 0x1F;
            break;
        default:
            break;
        }
    }

    void clockPsg() {

        ++m_envCount;
        while (m_envCount >= 0x10000u && m_envFreq != 0) {
            if (!m_envPause) {
                m_envPtr = m_envFace ? uint8_t((m_envPtr + 1) & 0x3F)
                                     : uint8_t((m_envPtr + 0x3F) & 0x3F);
            }
            if (m_envPtr & 0x20) {
                if (m_envContinue) {
                    if (m_envAlternate ^ m_envHold) m_envFace = !m_envFace;
                    if (m_envHold) m_envPause = true;
                    m_envPtr = m_envFace ? 0 : 0x1F;
                } else {
                    m_envPause = true;
                    m_envPtr = 0;
                }
            }
            m_envCount -= m_envFreq;
        }

        ++m_noiseCount;
        if (m_noiseCount & 0x40) {
            if (m_noiseSeed & 1) m_noiseSeed ^= 0x24000u;
            m_noiseSeed >>= 1;
            m_noiseCount = uint16_t(m_noiseCount - m_noiseFreq);
        }
        const bool noise = (m_noiseSeed & 1) != 0;

        for (int ch = 0; ch < 3; ++ch) {
            ++m_toneCount[ch];
            if (m_toneCount[ch] & 0x1000) {
                if (m_toneFreq[ch] > 1) {
                    m_toneEdge[ch] = !m_toneEdge[ch];
                    m_toneCount[ch] = uint16_t(m_toneCount[ch] - m_toneFreq[ch]);
                } else {
                    m_toneEdge[ch] = true;
                }
            }

            const bool toneMasked = (m_audioReg[7] & (1u << ch)) != 0;
            const bool noiseMasked = (m_audioReg[7] & (1u << (ch + 3))) != 0;
            if ((toneMasked || m_toneEdge[ch]) && (noiseMasked || noise)) {
                const uint8_t vol = uint8_t(m_audioReg[8 + ch] << 1);
                m_psgOut[ch] = (vol & 32) ? volumeLevel(m_envPtr) : volumeLevel(vol);
            } else {
                m_psgOut[ch] = 0;
            }
        }
    }
};

class Mapper89 final : public Mapper {public:explicit Mapper89(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0xC000)return false;m_prg=(d>>4)&7;m_chr=(d&7)|((d>>4)&8);m_mirror=(d&8)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper93 final : public Mapper {public:explicit Mapper93(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=(d>>4)&7;return true;}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);}private:uint8_t m_prg=0;};

class Mapper184 final : public Mapper {public:explicit Mapper184(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=(a-0x8000)%m_config.prgRomSize;return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_chr[0]=d&7;m_chr[1]=0x80|((d>>4)&7);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>12],0x1000,s,a&0xFFF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_chr[0]);put8(o,m_chr[1]);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_chr[0])&&get8(p,e,m_chr[1]);}private:uint8_t m_chr[2]={};};

}

std::unique_ptr<Mapper> createSunsoftMapper(const MapperConfig& config)
{
    switch (config.id) {
    case 67: return std::make_unique<Mapper67>(config);
    case 68: return std::make_unique<Mapper68>(config);
    case 69: return std::make_unique<Mapper69>(config);
    case 89: return std::make_unique<Mapper89>(config);
    case 93: return std::make_unique<Mapper93>(config);
    case 122: case 184: return std::make_unique<Mapper184>(config);
    default: return nullptr;
    }
}
