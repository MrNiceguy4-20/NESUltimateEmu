
namespace mapper_hard_detail {
inline void put16(std::vector<uint8_t>& o, uint16_t v) { put8(o, uint8_t(v)); put8(o, uint8_t(v >> 8)); }
inline bool get16(const uint8_t*& p, const uint8_t* e, uint16_t& v) { uint8_t l,h; if(!get8(p,e,l)||!get8(p,e,h)) return false; v=uint16_t(l)|(uint16_t(h)<<8); return true; }
}

class MapperJY final : public Mapper {
public:
    explicit MapperJY(const MapperConfig& c) : Mapper(c) { reset(true); }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if (a==0x5000 || a==0x5400 || a==0x5C00) { d=0; return true; }
        if ((a & 0xF803)==0x5800) { d=uint8_t(m_mulResult); return true; }
        if ((a & 0xF803)==0x5801) { d=uint8_t(m_mulResult>>8); return true; }
        if ((a & 0xF803)==0x5802) { d=m_accum; return true; }
        if ((a & 0xF803)==0x5803) { d=m_test; return true; }
        return false;
    }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (m_config.prgRomSize==0) return false;
        if (a>=0x6000 && a<0x8000) {
            if ((m_mode[0]&0x80)==0) return false;
            const uint8_t mode=m_mode[0]&3;
            std::size_t b=m_prg[3];
            if(mode==0) b=(std::size_t(m_prg[3])<<2)|3;
            else if(mode==1) b=(std::size_t(m_prg[3])<<1)|1;
            b=applyPrgOuter(b,0x2000);
            m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        if(a<0x8000) return false;
        const uint8_t mode=m_mode[0]&3;
        const bool switchLast=(m_mode[0]&4)!=0;
        if(mode==0){
            std::size_t b=switchLast?m_prg[3]:lastInnerBank(0x8000);
            b=applyPrgOuter(b,0x8000); m=mapBank(b,0x8000,m_config.prgRomSize,a-0x8000); return true;
        }
        if(mode==1){
            std::size_t b=(a<0xC000)?m_prg[1]:(switchLast?m_prg[3]:lastInnerBank(0x4000));
            b=applyPrgOuter(b,0x4000); m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF); return true;
        }
        unsigned slot=(a-0x8000)>>13;
        std::size_t b=(slot<3)?m_prg[slot]:(switchLast?m_prg[3]:lastInnerBank(0x2000));
        if(mode==3) b=reverse7(uint8_t(b));
        b=applyPrgOuter(b,0x2000); m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
    }

    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if ((a&0xF803)==0x5800){m_mulA=d;return true;}
        if ((a&0xF803)==0x5801){m_mulB=d;m_mulTarget=uint16_t(m_mulA)*m_mulB;m_mulCycles=8;return true;}
        if ((a&0xF803)==0x5802){m_accum=uint8_t(m_accum+d);return true;}
        if ((a&0xF803)==0x5803){m_accum=0;m_test=d;return true;}
        if ((a&0xF803)>=0x8000 && (a&0xF803)<=0x8003){m_prg[a&3]=d;return true;}
        if ((a&0xF807)>=0x9000 && (a&0xF807)<=0x9007){m_chr[a&7]=(m_chr[a&7]&0xFF00)|d;return true;}
        if ((a&0xF807)>=0xA000 && (a&0xF807)<=0xA007){m_chr[a&7]=(m_chr[a&7]&0x00FF)|(uint16_t(d)<<8);return true;}
        if ((a&0xF807)>=0xB000 && (a&0xF807)<=0xB007){unsigned i=a&3;if((a&4)==0)m_nt[i]=(m_nt[i]&0xFF00)|d;else m_nt[i]=(m_nt[i]&0x00FF)|(uint16_t(d)<<8);return true;}
        if ((a&0xF000)==0xC000){
            switch(a&7){
            case 0: if(d&1) enableIrq(); else disableIrq(); break;
            case 1: m_irqMode=d; break; case 2: disableIrq(); break; case 3: enableIrq(); break;
            case 4: m_prescaler=uint8_t(d^m_irqXor); break; case 5: m_irqCounter=uint8_t(d^m_irqXor); break;
            case 6: m_irqXor=d; break; case 7: m_irqExtra=d; break;
            } return true;
        }
        if ((a&0xF803)>=0xD000 && (a&0xF803)<=0xD003){m_mode[a&3]=d;updateMirror();return true;}
        if(a>=0x8000){ return true;}
        return false;
    }

    bool ppuMapRead(uint16_t a,uint32_t& m) override { return mapChr(a,m); }
    bool ppuMapReadEx(uint16_t a,uint32_t& m,PpuFetchKind) override { return mapChr(a,m); }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override {
        if(!m_config.chrRamSize || (m_mode[2]&0x40)==0) return false;
        return mapChr(a,m);
    }
    bool ppuUsesChrRam(uint16_t a) const override { return a<0x2000 && m_config.chrRamSize!=0 && m_config.chrRomSize==0; }

    bool mapNametable(uint16_t a,NametableSource& s,uint32_t& m) const override { return mapNt(a,s,m,false); }
    bool mapNametableWrite(uint16_t a,NametableSource& s,uint32_t& m) const override { return mapNt(a,s,m,true); }

    bool mapPrgRam(uint16_t a,uint32_t& m,bool) const override {
        if(a<0x6000||a>0x7FFF||(m_mode[0]&0x80)||!m_config.prgRamSize)return false;
        m=(a-0x6000)%m_config.prgRamSize; return true;
    }

    void notifyPpuAddress(uint16_t a,uint64_t) override {

        uint16_t x=a&0x1FFF;
        if((x>=0x0FD8&&x<=0x0FDF))m_latch[0]=0; else if(x>=0x0FE8&&x<=0x0FEF)m_latch[0]=1;
        if((x>=0x1FD8&&x<=0x1FDF))m_latch[1]=0; else if(x>=0x1FE8&&x<=0x1FEF)m_latch[1]=1;
        const bool a12=(a&0x1000)!=0;
        if((m_irqMode&3)==1 && a12 && !m_lastA12) irqClock();
        if((m_irqMode&3)==2) irqClock();
        m_lastA12=a12;
    }
    void observeCpuWrite(uint16_t, uint8_t) override { if((m_irqMode&3)==3) irqClock(); }
    void clockCpu() override { if(m_mulCycles && --m_mulCycles==0)m_mulResult=m_mulTarget; if((m_irqMode&3)==0) irqClock(); }
    bool irqActive() const override { return m_irqPending; }

    void reset(bool hard) override {
        if(!hard)return;
        for(auto&v:m_prg)v=0;
        for(auto&v:m_chr)v=0;
        for(auto&v:m_nt)v=0;
        for(auto&v:m_mode)v=0;
        m_mode[0]=2;

        m_irqMode=m_irqXor=m_irqExtra=m_prescaler=m_irqCounter=0;m_irqEnabled=m_irqPending=false;
        m_mulA=m_mulB=m_accum=m_test=0;m_mulResult=m_mulTarget=0;m_mulCycles=0;m_latch[0]=m_latch[1]=0;m_lastA12=false;updateMirror();
    }

    void saveState(std::vector<uint8_t>&o) const override {
        for(auto v:m_prg)put8(o,v);
        for(auto v:m_chr){put8(o,uint8_t(v));
        put8(o,uint8_t(v>>8));
        }
        for(auto v:m_nt){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}for(auto v:m_mode)put8(o,v);
        put8(o,m_irqMode);put8(o,m_irqXor);put8(o,m_irqExtra);put8(o,m_prescaler);put8(o,m_irqCounter);put8(o,m_irqEnabled);put8(o,m_irqPending);
        put8(o,m_mulA);put8(o,m_mulB);put8(o,uint8_t(m_mulResult));put8(o,uint8_t(m_mulResult>>8));put8(o,uint8_t(m_mulTarget));put8(o,uint8_t(m_mulTarget>>8));put8(o,m_mulCycles);put8(o,m_accum);put8(o,m_test);put8(o,m_latch[0]);put8(o,m_latch[1]);put8(o,m_lastA12);
    }
    bool loadState(const uint8_t*&p,const uint8_t*e) override {
        uint8_t lo,hi,b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr){if(!get8(p,e,lo)||!get8(p,e,hi))return false;v=uint16_t(lo)|(uint16_t(hi)<<8);}for(auto&v:m_nt){if(!get8(p,e,lo)||!get8(p,e,hi))return false;v=uint16_t(lo)|(uint16_t(hi)<<8);}for(auto&v:m_mode)if(!get8(p,e,v))return false;
        if(!get8(p,e,m_irqMode)||!get8(p,e,m_irqXor)||!get8(p,e,m_irqExtra)||!get8(p,e,m_prescaler)||!get8(p,e,m_irqCounter)||!get8(p,e,b))return false;
        m_irqEnabled=b;
        if(!get8(p,e,b))return false;
        m_irqPending=b;
        if(!get8(p,e,m_mulA)||!get8(p,e,m_mulB)||!get8(p,e,lo)||!get8(p,e,hi))return false;
        m_mulResult=uint16_t(lo)|(uint16_t(hi)<<8);
        if(!get8(p,e,lo)||!get8(p,e,hi)||!get8(p,e,m_mulCycles)||!get8(p,e,m_accum)||!get8(p,e,m_test)||!get8(p,e,m_latch[0])||!get8(p,e,m_latch[1])||!get8(p,e,b))return false;
        m_mulTarget=uint16_t(lo)|(uint16_t(hi)<<8);
        m_lastA12=b;
        updateMirror();
        return true;
    }
private:
    uint8_t m_prg[4]{}; uint16_t m_chr[8]{}; uint16_t m_nt[4]{}; uint8_t m_mode[4]{};
    uint8_t m_irqMode=0,m_irqXor=0,m_irqExtra=0,m_prescaler=0,m_irqCounter=0;bool m_irqEnabled=false,m_irqPending=false,m_lastA12=false;
    uint8_t m_mulA=0,m_mulB=0,m_mulCycles=0,m_accum=0,m_test=0,m_latch[2]={};uint16_t m_mulResult=0,m_mulTarget=0;
    static uint8_t reverse7(uint8_t x){x&=0x7F;uint8_t r=0;for(int i=0;i<7;i++)r|=((x>>i)&1)<<(6-i);return r;}
    std::size_t outerPrgBase() const { return std::size_t((m_mode[3]>>1)&3)*0x80000u; }
    std::size_t applyPrgOuter(std::size_t bank,std::size_t bankSize) const { const std::size_t inner=0x80000u/bankSize; return outerPrgBase()/bankSize + (bank&(inner-1)); }
    std::size_t lastInnerBank(std::size_t bankSize) const { return (0x80000u/bankSize)-1; }
    std::size_t chrBank1k(unsigned slot) const {
        uint16_t b=0;uint8_t cm=(m_mode[0]>>3)&3;
        if(cm==0)b=uint16_t((m_chr[0]&~7u)+(slot&7));
        else if(cm==1){unsigned half=slot>>2;unsigned reg=half?4:0;if(m_mode[3]&0x80)reg=half?(m_latch[1]?6:4):(m_latch[0]?2:0);b=uint16_t((m_chr[reg]&~3u)+(slot&3));}
        else if(cm==2){unsigned reg=slot&~1u;b=uint16_t((m_chr[reg]&~1u)+(slot&1));}
        else b=m_chr[slot];
        const bool large=(m_mode[3]&0x20)!=0;std::size_t mask=large?0x1FFu:0xFFu;std::size_t outer=std::size_t((m_mode[3]>>3)&3)<<9;if(!large)outer|=std::size_t(m_mode[3]&1)<<8;return outer+(b&mask);
    }
    bool mapChr(uint16_t a,uint32_t&m) const {if(a>=0x2000)return false;std::size_t s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(chrBank1k(a>>10),0x400,s,a&0x3FF);return true;}
    bool advancedNt() const { return m_config.id!=90; }
    bool mapNt(uint16_t a,NametableSource&s,uint32_t&m,bool write) const {
        if(a<0x2000||a>0x3EFF||!advancedNt())return false;
        uint16_t n=(a-0x2000)&0xFFF;
        unsigned slot=n>>10;
        bool ext=(m_mode[1]&8)!=0;
        bool rom=(m_mode[0]&0x20)!=0;
        if(!ext&&!rom)return false;
        bool useRom=false;
        if(rom){if(m_mode[0]&0x40)useRom=true;
        else useRom=((m_nt[slot]&0x80)!=0)!=((m_mode[2]&0x80)!=0);
        }if(write)useRom=false;
        if(useRom){std::size_t sz=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!sz)return false;s=m_config.chrRomSize?NametableSource::ChrRom:NametableSource::ChrRam;m=mapBank(m_nt[slot],0x400,sz,n&0x3FF);return true;}
        s=NametableSource::Ciram;m=uint32_t(m_nt[slot]&1)*0x400+(n&0x3FF);return true;
    }
    void updateMirror(){switch(m_mode[1]&3){case 0:m_mirror=Mirror::Vertical;break;case 1:m_mirror=Mirror::Horizontal;break;case 2:m_mirror=Mirror::OnescreenLo;break;case 3:m_mirror=Mirror::OnescreenHi;break;}}
    void disableIrq(){m_irqEnabled=false;m_irqPending=false;m_prescaler=0;}
    void enableIrq(){m_irqEnabled=true;}
    void irqClock(){if(!m_irqEnabled)return;uint8_t dir=(m_irqMode>>6)&3;if(dir!=1&&dir!=2)return;uint8_t mask=(m_irqMode&4)?7:0xFF;bool wrap=false;if(dir==1){++m_prescaler;wrap=(m_prescaler&mask)==0;}else{--m_prescaler;wrap=(m_prescaler&mask)==mask;}if(!wrap)return;if(dir==1){++m_irqCounter;if(m_irqCounter==0)m_irqPending=true;}else{--m_irqCounter;if(m_irqCounter==0xFF)m_irqPending=true;}}
};

class Mapper268 final : public Mapper {
public:
    explicit Mapper268(const MapperConfig& c) : Mapper(c) { hardReset(); }

    bool implementationSupported() const override {
        return m_config.nes20 && m_config.submapper <= 11;
    }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const unsigned slot = (a - 0x8000) >> 13;
        std::size_t inner = 0;
        if (!m_prgMode) inner = slot==0 ? m_bank[6] : slot==1 ? m_bank[7] : slot==2 ? 0xFE : 0xFF;
        else inner = slot==0 ? 0xFE : slot==1 ? m_bank[7] : slot==2 ? m_bank[6] : 0xFF;

        const uint8_t mode = m_outer[3] & 0x50;
        std::size_t bank = 0;
        if (mode & 0x10) {
            const bool bank32 = gnrom32k();
            const unsigned a13 = (a >> 13) & 1;
            const unsigned a14 = bank32 ? ((a >> 14) & 1) : ((m_outer[3] >> 1) & 1);
            const unsigned a15 = (m_outer[3] >> 2) & 1;
            const unsigned a16 = (m_outer[3] >> 3) & 1;
            bank = std::size_t(a13) | (std::size_t(a14)<<1) | (std::size_t(a15)<<2) | (std::size_t(a16)<<3);

            bank = applyPrgOuter(bank | (inner & ~std::size_t(0x0F)));
        } else {

            if ((mode & 0x40) && !m_prgMode && slot >= 2) inner = 0;
            bank = applyPrgOuter(inner);
        }
        m = mapBank(bank, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (isOuterAddress(a)) {
            writeOuter(a, d);

            return outerBase() == 0x5000;
        }
        if (a < 0x8000) return false;
        switch (a & 0xE001) {
        case 0x8000: m_select=d; m_prgMode=(d&0x40)!=0; m_chrMode=(d&0x80)!=0; return true;
        case 0x8001: m_bank[m_select&7]=d; return true;
        case 0xA000:
            if(!m_config.fourScreen) {

                if (!isOneScreen268()) m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;
            }
            return true;
        case 0xA001: m_wramEnable=(d&0x80)!=0; m_wramProtect=(d&0x40)!=0; return true;
        case 0xC000: m_irqLatch=d; return true;
        case 0xC001: m_irqReload=true; return true;
        case 0xE000: m_irqEnabled=false; m_irqPending=false; return true;
        case 0xE001: m_irqEnabled=true; return true;
        }
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override { return mapChr(a,m); }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        if (!ppuUsesChrRam(a) || chrRamWriteProtected()) return false;
        return mapChr(a,m);
    }
    bool ppuUsesChrRam(uint16_t a) const override {
        if (a >= 0x2000 || !m_config.chrRamSize) return false;
        if (!m_config.chrRomSize) return true;

        if (!(m_outer[4] & 1)) return false;
        const std::size_t b = chrBank1k(a);
        return (uint8_t(b) & 0xFE) == (m_outer[4] & 0xFE);
    }
    bool mapNametable(uint16_t a, NametableSource& source, uint32_t& mapped) const override {
        if (a < 0x2000 || a > 0x3EFF || !isOneScreen268()) return false;
        const uint16_t n=(a-0x2000)&0x0FFF; source=NametableSource::Ciram;
        mapped=uint32_t((m_outer[0]>>4)&1)*0x400u+(n&0x3FFu); return true;
    }
    bool mapNametableWrite(uint16_t a, NametableSource& source, uint32_t& mapped) const override { return mapNametable(a,source,mapped); }

    bool mapPrgRam(uint16_t a, uint32_t& m, bool write) const override {
        if (!m_config.prgRamSize || !m_wramEnable || (write && m_wramProtect)) return false;
        if (a >= 0x6000 && a <= 0x7FFF) {
            m = uint32_t(a - 0x6000) % uint32_t(m_config.prgRamSize); return true;
        }

        if (a >= 0x5000 && a <= 0x5FFF && (m_outer[3] & 0x20)) {
            m = uint32_t(a - 0x5000) % uint32_t(m_config.prgRamSize); return true;
        }
        return false;
    }

    void notifyPpuAddress(uint16_t a, uint64_t cyc) override {
        const bool hi=(a&0x1000)!=0;
        if(!hi){ if(m_lastA12||!m_lowValid){m_lowStart=cyc;m_lowValid=true;} }
        else { if(!m_lastA12&&m_lowValid&&cyc-m_lowStart>=8) clockIrq(); m_lowValid=false; }
        m_lastA12=hi;
    }
    bool irqActive() const override { return m_irqPending; }
    void reset(bool hard) override { if(hard) hardReset(); }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o,uint8_t(m_mirror)); put8(o,m_select); for(auto v:m_bank)put8(o,v);
        for(auto v:m_outer)put8(o,v);
        put8(o,m_prgMode);
        put8(o,m_chrMode);
        put8(o,m_wramEnable);
        put8(o,m_wramProtect);
        put8(o,m_chrA17);
        put8(o,m_irqLatch);put8(o,m_irqCounter);put8(o,m_irqEnabled);put8(o,m_irqReload);put8(o,m_irqPending);put8(o,m_lastA12);put8(o,m_lowValid);put64(o,m_lowStart);
    }
    bool loadState(const uint8_t*& p,const uint8_t* e) override {
        uint8_t b;if(!get8(p,e,b))return false;m_mirror=Mirror(b);if(!get8(p,e,m_select))return false;
        for(auto&v:m_bank)if(!get8(p,e,v))return false;
        for(auto&v:m_outer)if(!get8(p,e,v))return false;
        if(!get8(p,e,b))return false;
        m_prgMode=b;
        if(!get8(p,e,b))return false;
        m_chrMode=b;
        if(!get8(p,e,b))return false;
        m_wramEnable=b;
        if(!get8(p,e,b))return false;
        m_wramProtect=b;
        if(!get8(p,e,b))return false;
        m_chrA17=b;
        if(!get8(p,e,m_irqLatch)||!get8(p,e,m_irqCounter)||!get8(p,e,b))return false;
        m_irqEnabled=b;
        if(!get8(p,e,b))return false;
        m_irqReload=b;
        if(!get8(p,e,b))return false;
        m_irqPending=b;
        if(!get8(p,e,b))return false;
        m_lastA12=b;
        if(!get8(p,e,b))return false;
        m_lowValid=b;
        return get64(p,e,m_lowStart);
    }

private:
    uint8_t m_select=0,m_bank[8]{},m_outer[6]{}; bool m_prgMode=false,m_chrMode=false,m_wramEnable=true,m_wramProtect=false; mutable bool m_chrA17=false;
    uint8_t m_irqLatch=0,m_irqCounter=0;bool m_irqEnabled=false,m_irqReload=false,m_irqPending=false,m_lastA12=false,m_lowValid=false;uint64_t m_lowStart=0;

    uint16_t outerBase() const {
        if (m_config.submapper & 1) return 0x5000;
        return (m_config.submapper==2) ? 0x7000 : 0x6000;
    }
    bool isOuterAddress(uint16_t a) const { return (a & 0xF000) == outerBase() && (a & 7) <= 5; }
    void writeOuter(uint16_t a,uint8_t d) {
        const unsigned r=a&7;
        if(r==2){
            if(m_outer[2]&0x80) d=uint8_t((m_outer[2]&0xF0)|(d&0x0F));
            m_outer[2]=d;return;
        }
        const bool gnrom=(m_outer[3]&0x10)!=0;
        if((m_outer[3]&0x80) && !gnrom) return;
        m_outer[r]=d;
        if(r==0 && isOneScreen268()) m_mirror=(d&0x10)?Mirror::OnescreenHi:Mirror::OnescreenLo;
    }
    bool gnrom32k() const {

        if(m_config.submapper>=2) return (m_outer[1]&0x10)==0;
        return (m_outer[1]&0x02)!=0;
    }
    std::size_t applyPrgOuter(std::size_t bank) const {
        const uint8_t r0=m_outer[0],r1=m_outer[1];
        auto setbit=[&](unsigned bit,bool v){const std::size_t mask=std::size_t(1)<<bit;bank=v?(bank|mask):(bank&~mask);};

        setbit(4,(r0&0x40)?((r0>>0)&1):((bank>>4)&1));

        if (m_config.submapper==4 || m_config.submapper==5) {

            setbit(5,(r1&0x80)?((r0>>1)&1):((bank>>5)&1));
            setbit(6,(r1&0x40)?((bank>>6)&1):((r0>>2)&1));
            setbit(7,(r1&0x20)?((bank>>7)&1):false);
            setbit(8,(r0>>4)&1);
            return bank & 0x1FFu;
        }
        if (m_config.submapper==8 || m_config.submapper==9) {

            setbit(5,(r1&0x80)?((r0>>1)&1):((bank>>5)&1));
            setbit(6,(r1&0x40)?((bank>>6)&1):((r0>>2)&1));
            setbit(7,(r1&0x20)?((bank>>7)&1):false);
            return bank & 0xFFu;
        }

        setbit(5,(r1&0x80)?((r0>>1)&1):((bank>>5)&1));
        setbit(6,(r1&0x40)?((bank>>6)&1):((r0>>2)&1));
        bool offA20=false,offA21=false,offA22=false;
        if(m_config.submapper==2||m_config.submapper==3){offA20=(r1>>3)&1;offA21=(r1>>1)&1;offA22=(r1>>2)&1;}
        else {offA20=(r1>>4)&1;offA21=(r1>>3)&1;offA22=(r1>>2)&1;}
        setbit(7,(r1&0x20)?((bank>>7)&1):offA20);
        setbit(8,offA21); setbit(9,offA22);
        if (m_config.submapper==10 || m_config.submapper==11) return bank & 0x3FFu;
        setbit(10,(r0>>4)&1);
        setbit(11,(r0>>5)&1);

        if (m_config.submapper==6 || m_config.submapper==7) {

            const bool chip=(r0&0x80)?((r0&0x08)!=0):m_chrA17;
            const std::size_t halfBanks=(m_config.prgRomSize/2)/0x2000;
            if(halfBanks){bank%=halfBanks;if(chip)bank+=halfBanks;}
        }
        return bank;
    }
    std::size_t chrBank1k(uint16_t a) const {
        const unsigned slot=a>>10; std::size_t inner=0;
        if(m_outer[3]&0x10){
            inner=std::size_t(slot&7)|(std::size_t(m_outer[2]&0x0F)<<3);
        }else{
            const uint8_t r0=m_bank[0],r1=m_bank[1];
            if(!m_chrMode){if(a<0x800)inner=(r0&0xFE)+((a>>10)&1);else if(a<0x1000)inner=(r1&0xFE)+((a>>10)&1);else inner=m_bank[2+((a-0x1000)>>10)];}
            else {if(a<0x1000)inner=m_bank[2+(a>>10)];else if(a<0x1800)inner=(r0&0xFE)+((a>>10)&1);else inner=(r1&0xFE)+((a>>10)&1);}
        }
        const uint8_t o=m_outer[0];
        if(m_config.submapper==6 || m_config.submapper==7){

            inner &= 0x7Fu;
        } else {
            const bool a17=(o&0x80)?((o>>3)&1):((inner>>7)&1);
            inner=(inner&0x7F)|(std::size_t(a17)<<7);
            inner=(inner&0xFF)|(std::size_t((o>>4)&3)<<8);
            if(m_config.submapper==4 || m_config.submapper==5)
                inner=(inner&0x3FFu)|(std::size_t(o&0x06)<<9);
        }
        return inner;
    }
    bool mapChr(uint16_t a,uint32_t&m) const {
        if(a>=0x2000)return false;
        const std::size_t sz=ppuUsesChrRam(a)?m_config.chrRamSize:m_config.chrRomSize;
        if(!sz)return false;
        const std::size_t b=chrBank1k(a);
        if(m_config.submapper==6 || m_config.submapper==7) m_chrA17=((b>>7)&1)!=0;
        m=mapBank(b,0x400,sz,a&0x3FF);return true;
    }
    bool chrRamWriteProtected() const { return (m_config.submapper==8 || m_config.submapper==9) && (m_outer[0]&0x10); }
    bool isOneScreen268() const { return (m_config.submapper==10 || m_config.submapper==11) && !(m_outer[0]&0x20); }
    void clockIrq(){if(m_irqCounter==0||m_irqReload){m_irqCounter=m_irqLatch;m_irqReload=false;}else --m_irqCounter;if(m_irqCounter==0&&m_irqEnabled)m_irqPending=true;}
    void hardReset(){m_select=0;std::fill(std::begin(m_bank),std::end(m_bank),0);m_bank[0]=0;m_bank[1]=2;m_bank[2]=4;m_bank[3]=5;m_bank[4]=6;m_bank[5]=7;std::fill(std::begin(m_outer),std::end(m_outer),0);m_prgMode=m_chrMode=false;m_wramEnable=true;m_wramProtect=false;m_chrA17=false;m_irqLatch=m_irqCounter=0;m_irqEnabled=m_irqReload=m_irqPending=m_lastA12=m_lowValid=false;m_lowStart=0;m_mirror=m_config.headerMirror;if(m_config.submapper==10||m_config.submapper==11)m_mirror=Mirror::OnescreenLo;}
};

class Mapper176 final : public Mapper {
public:
    explicit Mapper176(const MapperConfig& c) : Mapper(c) { hardReset(); }

    bool implementationSupported() const override {
        return !m_config.nes20 || m_config.submapper <= 5;
    }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const unsigned mode = m_mode & 7;
        const std::size_t baseBank = prgBaseBank();

        if (mode <= 2) {
            const unsigned slot = (a - 0x8000) >> 13;
            std::size_t inner;
            if (m_extended) {
                if (!m_prgMode) inner = (slot==0)?m_bank[6]:(slot==1)?m_bank[7]:(slot==2)?m_bank[8]:m_bank[9];
                else inner = (slot==0)?m_bank[8]:(slot==1)?m_bank[7]:(slot==2)?m_bank[6]:m_bank[9];
            } else {
                const bool eightBit = m_config.submapper==1 || m_config.submapper==3;
                const std::size_t fixedM1 = eightBit ? 0xFEu : 0x3Eu;
                const std::size_t fixed   = eightBit ? 0xFFu : 0x3Fu;
                if (!m_prgMode) inner = (slot==0)?m_bank[6]:(slot==1)?m_bank[7]:(slot==2)?fixedM1:fixed;
                else inner = (slot==0)?fixedM1:(slot==1)?m_bank[7]:(slot==2)?m_bank[6]:fixed;
            }
            unsigned lowBits;
            if (mode==0) lowBits=(m_config.submapper==1 || m_config.submapper==3)?8:6;
            else lowBits=(mode==1)?5:4;
            const std::size_t lowMask=(std::size_t(1)<<lowBits)-1;
            const std::size_t bank=(baseBank&~lowMask)|(inner&lowMask);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        if (mode == 3) {
            const std::size_t bank=(baseBank&~std::size_t(1))|((a>>13)&1);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        if (mode == 4) {
            const std::size_t bank=(baseBank&~std::size_t(3))|((a>>13)&3);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        if (mode == 5) {
            const std::size_t inner16=(a<0xC000)?(m_unrom&7):7;
            const std::size_t bank=(baseBank&~std::size_t(0x0F))|(inner16<<1)|((a>>13)&1);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        return false;
    }

    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {

        if(m_config.submapper==5 && a>=0x4800 && a<=0x4FFF){m_prgMsb=d&0x3F;return true;}

        if(a>=0x5000 && a<0x6000){
            if(m_config.submapper==2 && !outerRegsEnabled()) return false;
            const unsigned r=(m_config.submapper==3)?(a&7):(a&3);
            switch(r){
            case 0:m_mode=d;break;
            case 1:m_prgBase=d&0x7F;break;
            case 2:m_chrBase=d;break;
            case 3:if(m_config.submapper==1||m_config.submapper==2)m_extended=(d&2)!=0;break;
            case 5:if(m_config.submapper==3)m_prgMsb=d&0x0F;break;
            case 6:if(m_config.submapper==3)m_chrMsb=d&0x0F;break;
            default:break;
            }
            return true;
        }
        if(a<0x8000)return false;
        if((m_mode&7)==5)m_unrom=d&7;
        if(m_config.submapper==1&&(m_mode&0x40)&&!(m_mode&0x20))m_cnrom=d&3;

        switch(a&0xE003){
        case 0x8000:
            m_select=d;
            m_prgMode=(d>>6)&1;
            m_chrMode=(d>>7)&1;
            return true;
        case 0x8001:{
            unsigned r=m_select&(m_extended?0x0F:7);

            if(m_config.submapper==2 && !m_extended){if(r==6)r=7;else if(r==7)r=6;}
            if(r<12)m_bank[r]=d;
            return true;
        }
        case 0xA000:
            if(!m_config.fourScreen){
                if(m_config.submapper==2 && ramConfigEnabled()){
                    switch(d&3){case 0:m_mirror=Mirror::Vertical;break;case 1:m_mirror=Mirror::Horizontal;break;case 2:m_mirror=Mirror::OnescreenLo;break;case 3:m_mirror=Mirror::OnescreenHi;break;}
                }else m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;
            }
            return true;
        case 0xA001:
            if(m_config.submapper==2 && (d&0x20)){
                m_ramConfig=d;
                m_wramEnable=(d&0x80)!=0;
                m_wramProtect=false;
            }else{
                m_ramConfig=(m_config.submapper==2)?0:m_ramConfig;
                m_wramEnable=(d&0x80)!=0;
                m_wramProtect=(d&0x40)!=0;
            }
            return true;
        case 0xC000:m_irqLatch=d;return true;
        case 0xC001:m_irqReload=true;return true;
        case 0xE000:m_irqEnabled=false;m_irqPending=false;return true;
        case 0xE001:m_irqEnabled=true;return true;
        default:return true;
        }
    }

    bool ppuMapRead(uint16_t a,uint32_t&m) override{return mapChr(a,m);}
    bool ppuMapReadEx(uint16_t a,uint32_t&m,PpuFetchKind) override{return mapChr(a,m);}
    bool ppuMapWrite(uint16_t a,uint32_t&m) override{if(!ppuUsesChrRam(a))return false;return mapChr(a,m);}
    bool ppuUsesChrRam(uint16_t a) const override{
        if(a>=0x2000||!m_config.chrRamSize)return false;
        if(!m_config.chrRomSize)return true;
        if(m_config.submapper==2&&ramConfigEnabled()&&(m_ramConfig&0x08))return true;
        return m_config.submapper==0&&(m_mode&0x20)!=0;
    }

    bool mapPrgRam(uint16_t a,uint32_t&m,bool write) const override{
        if(!m_config.prgRamSize||!m_wramEnable)return false;
        if(m_config.submapper==2&&ramConfigEnabled()){
            if(a>=0x5000&&a<=0x5FFF&&!outerRegsEnabled()){

                m=0x5000u+(a-0x5000u);return m<m_config.prgRamSize;
            }
            if(a>=0x6000&&a<=0x7FFF){
                m=uint32_t(m_ramConfig&3)*0x2000u+(a-0x6000u);return m<m_config.prgRamSize;
            }
            return false;
        }
        if(a<0x6000||a>0x7FFF||(write&&m_wramProtect))return false;
        m=(a-0x6000)%m_config.prgRamSize;return true;
    }

    void notifyPpuAddress(uint16_t a,uint64_t cyc) override{
        const bool hi=(a&0x1000)!=0;
        if(!hi){if(m_lastA12||!m_lowValid){m_lowStart=cyc;m_lowValid=true;}}
        else{if(!m_lastA12&&m_lowValid&&cyc-m_lowStart>=8)clockIrq();m_lowValid=false;}
        m_lastA12=hi;
    }
    bool irqActive() const override{return m_irqPending;}
    void reset(bool hard) override{if(hard)hardReset();}

    void saveState(std::vector<uint8_t>&o) const override{
        put8(o,uint8_t(m_mirror));put8(o,m_mode);put8(o,m_prgBase);put8(o,m_chrBase);put8(o,m_prgMsb);put8(o,m_chrMsb);put8(o,m_ramConfig);put8(o,m_unrom);put8(o,m_cnrom);
        put8(o,m_select);for(auto v:m_bank)put8(o,v);put8(o,m_prgMode);put8(o,m_chrMode);put8(o,m_extended);
        put8(o,m_wramEnable);put8(o,m_wramProtect);put8(o,m_irqLatch);put8(o,m_irqCounter);put8(o,m_irqEnabled);put8(o,m_irqReload);put8(o,m_irqPending);put8(o,m_lastA12);put8(o,m_lowValid);put64(o,m_lowStart);
    }
    bool loadState(const uint8_t*&p,const uint8_t*e) override{
        uint8_t b;if(!get8(p,e,b))return false;m_mirror=Mirror(b);
        if(!get8(p,e,m_mode)||!get8(p,e,m_prgBase)||!get8(p,e,m_chrBase)||!get8(p,e,m_prgMsb)||!get8(p,e,m_chrMsb)||!get8(p,e,m_ramConfig)||!get8(p,e,m_unrom)||!get8(p,e,m_cnrom)||!get8(p,e,m_select))return false;
        for(auto&v:m_bank)if(!get8(p,e,v))return false;
        if(!get8(p,e,b))return false;
        m_prgMode=b;
        if(!get8(p,e,b))return false;
        m_chrMode=b;
        if(!get8(p,e,b))return false;
        m_extended=b;
        if(!get8(p,e,b))return false;
        m_wramEnable=b;
        if(!get8(p,e,b))return false;
        m_wramProtect=b;
        if(!get8(p,e,m_irqLatch)||!get8(p,e,m_irqCounter)||!get8(p,e,b))return false;
        m_irqEnabled=b;
        if(!get8(p,e,b))return false;
        m_irqReload=b;
        if(!get8(p,e,b))return false;
        m_irqPending=b;
        if(!get8(p,e,b))return false;
        m_lastA12=b;
        if(!get8(p,e,b))return false;
        m_lowValid=b;
        return get64(p,e,m_lowStart);
    }

private:
    uint8_t m_mode=0,m_prgBase=0,m_chrBase=0,m_prgMsb=0,m_chrMsb=0,m_ramConfig=0,m_unrom=0,m_cnrom=0,m_select=0;
    uint8_t m_bank[12]{};bool m_prgMode=false,m_chrMode=false,m_extended=false;
    bool m_wramEnable=true,m_wramProtect=false;
    uint8_t m_irqLatch=0,m_irqCounter=0;bool m_irqEnabled=false,m_irqReload=false,m_irqPending=false,m_lastA12=false,m_lowValid=false;uint64_t m_lowStart=0;

    bool ramConfigEnabled() const{return m_config.submapper==2&&(m_ramConfig&0x20)!=0;}
    bool outerRegsEnabled() const{return !ramConfigEnabled()||(m_ramConfig&0x40)!=0;}

    std::size_t prgBaseBank() const{
        std::size_t low=std::size_t(m_prgBase)<<1;
        switch(m_config.submapper){
        case 2:
            low|=std::size_t((m_mode>>3)&1)<<8;
            low|=std::size_t((m_mode>>7)&1)<<9;
            low|=std::size_t((m_chrBase>>6)&1)<<10;
            low|=std::size_t((m_chrBase>>7)&1)<<11;
            low|=std::size_t((m_chrBase>>5)&1)<<12;
            break;
        case 3: low|=std::size_t(m_prgMsb&0x0F)<<8;break;
        case 4: low|=std::size_t((m_chrBase>>7)&1)<<8;break;
        case 5:
            low=std::size_t(m_prgBase&0x1F)<<1;
            low|=std::size_t(m_prgMsb&0x3F)<<6;
            break;
        default:break;
        }
        return low;
    }

    void hardReset(){
        m_mode=m_prgBase=m_chrBase=m_prgMsb=m_chrMsb=m_ramConfig=m_unrom=m_cnrom=m_select=0;m_prgMode=m_chrMode=m_extended=false;m_wramEnable=true;m_wramProtect=false;
        uint8_t init[12]={0,2,4,5,6,7,0,1,0xFE,0xFF,0xFF,0xFF};std::copy(std::begin(init),std::end(init),std::begin(m_bank));
        m_irqLatch=m_irqCounter=0;m_irqEnabled=m_irqReload=m_irqPending=m_lastA12=m_lowValid=false;m_lowStart=0;m_mirror=m_config.headerMirror;
    }

    std::size_t chrBank1k(uint16_t a) const{
        const unsigned slot=a>>10;
        const std::size_t base=(std::size_t(m_chrBase)<<3)|((m_config.submapper==3)?(std::size_t(m_chrMsb&0x0F)<<11):0);
        if(m_mode&0x40){
            if(m_config.submapper==1&&!(m_mode&0x20)){
                const unsigned latchBits=(m_mode&0x10)?1:2;
                return (base&~((std::size_t(1)<<(3+latchBits))-1))|(std::size_t(m_cnrom&((1u<<latchBits)-1))<<3)|(slot&7);
            }
            return base|(slot&7);
        }
        std::size_t inner=0;
        if(m_extended){
            unsigned logical=slot;if(m_chrMode)logical^=4;static const uint8_t regs[8]={0,10,1,11,2,3,4,5};inner=m_bank[regs[logical]];
        }else{
            const uint8_t r0=m_bank[0]&0xFE,r1=m_bank[1]&0xFE;
            if(!m_chrMode){if(a<0x800)inner=r0+((a>>10)&1);else if(a<0x1000)inner=r1+((a>>10)&1);else inner=m_bank[2+((a-0x1000)>>10)];}
            else{if(a<0x1000)inner=m_bank[2+(a>>10)];else if(a<0x1800)inner=r0+((a>>10)&1);else inner=r1+((a>>10)&1);}
        }
        const unsigned lowBits=(m_mode&0x10)?7:8;
        const std::size_t mask=(std::size_t(1)<<lowBits)-1;
        return (base&~mask)|(inner&mask);
    }
    bool mapChr(uint16_t a,uint32_t&m) const{
        if(a>=0x2000)return false;
        const bool ram=ppuUsesChrRam(a);const std::size_t sz=ram?m_config.chrRamSize:m_config.chrRomSize;if(!sz)return false;
        if(ram&&m_config.submapper==2&&ramConfigEnabled()){m=uint32_t(a)%uint32_t(sz);return true;}
        m=mapBank(chrBank1k(a),0x400,sz,a&0x3FF);return true;
    }
    void clockIrq(){if(m_irqCounter==0||m_irqReload){m_irqCounter=m_irqLatch;m_irqReload=false;}else --m_irqCounter;if(m_irqCounter==0&&m_irqEnabled)m_irqPending=true;}
};

class MapperSuperMagicCard final : public Mapper {
public:
    explicit MapperSuperMagicCard(const MapperConfig& c) : Mapper(c) { hardReset(); }

    bool implementationSupported() const override {
        if (m_config.id == 8) return m_config.submapper == 0 || m_config.submapper == 4;
        if (m_config.id == 12) return m_config.submapper == 1;
        if (m_config.id == 17) return m_config.submapper <= 3;
        return m_config.id == 6 && m_config.submapper <= 7;
    }

    bool cpuMapRead(uint16_t, uint32_t&) const override { return false; }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if (a == 0x4500) {
            d = uint8_t(((m_oneMAddr & 1) ? 1 : 0) |
                        ((m_oneMAddr & 2) ? 2 : 0) |
                        (m_oneMData & 0xF0));
            return true;
        }
        if (a == 0x4501) {
            d = uint8_t((m_latch & 0xFC) | (m_bankModeAddr & 3));
            return true;
        }
        return false;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xFFFC) == 0x42FC) {
            m_oneMAddr = uint8_t(a & 3);
            m_oneMData = d;
            updateMirror();
            return true;
        }
        if ((a & 0xFFFC) == 0x43FC) {
            m_bankModeAddr = uint8_t(a & 3);
            m_commonChr = d & 3;
            return true;
        }
        if (a == 0x4500) { m_smcMode = d; return true; }
        if (a == 0x4501 && m_config.id == 17) { m_irqEnable = false; m_irqPending = false; return true; }
        if (a == 0x4502 && m_config.id == 17) {
            m_irqCounter = uint16_t((m_irqCounter & 0xFF00) | d); m_irqPending = false; return true;
        }
        if (a == 0x4503 && m_config.id == 17) {
            m_irqCounter = uint16_t((m_irqCounter & 0x00FF) | (uint16_t(d) << 8));
            m_irqPending = false; m_irqEnable = true; return true;
        }
        if (a >= 0x4504 && a <= 0x4507) { m_prg4[a - 0x4504] = d & 0x3F; return true; }
        if (a >= 0x4510 && a <= 0x451B) { m_chr1[a - 0x4510] = d; return true; }

        if (a >= 0x8000) {
            const unsigned slot = (a - 0x8000) >> 13;

            m_prg2[slot & 3] = uint8_t((d >> 2) & 0x3F);
            m_commonChr = d & 3;
            if (prgWriteProtected()) m_latch = d;
            return true;
        }
        return false;
    }

    bool mapPrgRam(uint16_t a, uint32_t& m, bool write) const override {
        const std::size_t prgMem = prgMemorySize();
        const std::size_t wramBase = prgMem;
        const std::size_t scratchBase = wramBase + 0x8000;
        if (a >= 0x5000 && a < 0x6000) {
            m = uint32_t(scratchBase + (a - 0x5000)); return m < m_config.prgRamSize;
        }
        if (a >= 0x6000 && a < 0x8000) {
            const std::size_t bank = (m_smcMode >> 4) & 3;
            m = uint32_t(wramBase + bank * 0x2000 + (a & 0x1FFF)); return m < m_config.prgRamSize;
        }
        if (a < 0x8000) return false;
        if (write && prgWriteProtected()) return false;
        const std::size_t bank = prgBank8k(a);
        m = uint32_t((bank * 0x2000 + (a & 0x1FFF)) % std::max<std::size_t>(1, prgMem));
        return m < m_config.prgRamSize;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000 || !m_config.chrRamSize) return false;
        updateMmc4Latch(a);
        const std::size_t bank = chrBank1k(a);
        m = mapBank(bank, 0x400, m_config.chrRamSize, a & 0x3FF);
        return true;
    }
    bool ppuMapReadEx(uint16_t a, uint32_t& m, PpuFetchKind) override { return ppuMapRead(a, m); }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000 || !m_config.chrRamSize || chrWriteProtected()) return false;
        return ppuMapRead(a, m);
    }
    bool ppuUsesChrRam(uint16_t) const override { return true; }

    bool mapNametable(uint16_t a, NametableSource& s, uint32_t& m) const override {
        if ((m_smcMode & 0x02) || a < 0x2000 || a >= 0x3F00) return false;
        const unsigned slot = ((a - 0x2000) & 0x0FFF) >> 10;
        s = NametableSource::ChrRam;
        m = mapBank(m_chr1[8 + slot], 0x400, m_config.chrRamSize, a & 0x3FF);
        return true;
    }
    bool mapNametableWrite(uint16_t a, NametableSource& s, uint32_t& m) const override {
        return mapNametable(a, s, m);
    }

    void notifyPpuAddress(uint16_t a, uint64_t) override {
        if (m_config.id != 17 || !(m_smcMode & 0x08)) { m_lastA12 = (a & 0x1000) != 0; return; }
        const bool high = (a & 0x1000) != 0;
        if (high && !m_lastA12) clockIrq();
        m_lastA12 = high;
    }
    void clockCpu() override { if (m_config.id == 17 && !(m_smcMode & 0x08)) clockIrq(); }
    bool irqActive() const override { return m_config.id == 17 && m_irqPending; }

    void reset(bool hard) override { if (hard) hardReset(); }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o,m_oneMAddr);put8(o,m_oneMData);put8(o,m_bankModeAddr);put8(o,m_smcMode);put8(o,m_latch);put8(o,m_commonChr);
        for(auto v:m_prg2)put8(o,v);
        for(auto v:m_prg4)put8(o,v);
        for(auto v:m_chr1)put8(o,v);
        put8(o,m_mmc4Lo);put8(o,m_mmc4Hi);put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));put8(o,m_irqEnable);put8(o,m_irqPending);put8(o,m_lastA12);put8(o,uint8_t(m_mirror));
    }
    bool loadState(const uint8_t*& p,const uint8_t* e) override {
        uint8_t lo,hi,b;
        if(!get8(p,e,m_oneMAddr)||!get8(p,e,m_oneMData)||!get8(p,e,m_bankModeAddr)||!get8(p,e,m_smcMode)||!get8(p,e,m_latch)||!get8(p,e,m_commonChr))return false;
        for(auto&v:m_prg2)if(!get8(p,e,v))return false;
        for(auto&v:m_prg4)if(!get8(p,e,v))return false;
        for(auto&v:m_chr1)if(!get8(p,e,v))return false;
        if(!get8(p,e,m_mmc4Lo)||!get8(p,e,m_mmc4Hi)||!get8(p,e,lo)||!get8(p,e,hi)||!get8(p,e,b))return false;
        m_irqEnable=b;
        if(!get8(p,e,b))return false;
        m_irqPending=b;
        if(!get8(p,e,b))return false;
        m_lastA12=b;
        if(!get8(p,e,b))return false;
        m_mirror=Mirror(b);
        m_irqCounter=uint16_t(lo)|(uint16_t(hi)<<8);return true;
    }

private:
    uint8_t m_oneMAddr=3,m_oneMData=0,m_bankModeAddr=3,m_smcMode=0x42,m_latch=0,m_commonChr=0;
    uint8_t m_prg2[4]{},m_prg4[4]{},m_chr1[12]{};
    uint8_t m_mmc4Lo=0,m_mmc4Hi=0; uint16_t m_irqCounter=0; bool m_irqEnable=false,m_irqPending=false,m_lastA12=false;

    std::size_t prgMemorySize() const {
        if (m_config.id == 17 || (m_config.id == 12 && m_config.submapper == 1)) return 0x80000;
        return std::max<std::size_t>(0x20000, m_config.prgRomSize);
    }
    bool prgWriteProtected() const { return (m_oneMAddr & 2) != 0; }
    unsigned latchMode() const { return (m_oneMData >> 5) & 7; }
    bool bankingOverride() const { return (m_bankModeAddr & 1) == 0; }
    bool fourMMode() const { return bankingOverride() && (m_bankModeAddr & 2) == 0; }
    bool twoMMode() const { return bankingOverride() && (m_bankModeAddr & 2) != 0; }
    bool chr1kMode() const { return (m_smcMode & 1) != 0; }
    bool mmc4Mode() const { return chr1kMode() && (m_smcMode & 4) == 0; }
    bool chrWriteProtected() const { unsigned md=latchMode(); return md>=4; }

    void updateMirror(){
        if(m_config.fourScreen){m_mirror=Mirror::FourScreen;return;}
        const bool setting=(m_oneMData&0x10)!=0;
        if(m_oneMAddr&1)m_mirror=setting?Mirror::Horizontal:Mirror::Vertical;
        else m_mirror=setting?Mirror::OnescreenHi:Mirror::OnescreenLo;
    }
    std::size_t prgBank8k(uint16_t a) const {
        const unsigned slot=(a-0x8000)>>13;
        if(fourMMode()) return m_prg4[slot]&0x3F;
        if(twoMMode()) return m_prg2[slot]&0x1F;
        const unsigned mode=latchMode();
        switch(mode){
        case 0:return a<0xC000?std::size_t(m_latch&7)*2+((a>>13)&1):14+((a>>13)&1);
        case 1:return a<0xC000?std::size_t((m_latch>>2)&0x0F)*2+((a>>13)&1):14+((a>>13)&1);
        case 2:return a<0xC000?std::size_t(m_latch&0x0F)*2+((a>>13)&1):30+((a>>13)&1);
        case 3:return a<0xC000?30+((a>>13)&1):std::size_t(m_latch&0x0F)*2+((a>>13)&1);
        case 4:return std::size_t((m_latch>>4)&3)*4+slot;
        default:return 12+slot;
        }
    }
    std::size_t chrBank1k(uint16_t a) const {
        if(chr1kMode()){
            if(mmc4Mode()){
                const bool hi=(a&0x1000)!=0; const uint8_t latch=hi?m_mmc4Hi:m_mmc4Lo;
                const unsigned pair=(hi?4:0)+(latch?2:0); return (m_chr1[pair]>>2)*4+((a>>10)&3);
            }
            return m_chr1[a>>10];
        }
        unsigned bank=0; const unsigned mode=latchMode();
        if(twoMMode()) bank=m_commonChr&3;
        else if(mode==1||mode==4||mode==5) bank=m_latch&3;
        else if(mode==3) bank=(m_latch>>4)&3;
        else if(mode==6) bank=m_latch&1;
        return std::size_t(bank)*8+(a>>10);
    }
    void updateMmc4Latch(uint16_t a){
        if(!mmc4Mode())return;
        if(a>=0x0FD8&&a<=0x0FDF)m_mmc4Lo=0;else if(a>=0x0FE8&&a<=0x0FEF)m_mmc4Lo=1;
        else if(a>=0x1FD8&&a<=0x1FDF)m_mmc4Hi=0;else if(a>=0x1FE8&&a<=0x1FEF)m_mmc4Hi=1;
    }
    void clockIrq(){ if(!m_irqEnable)return; ++m_irqCounter; if(m_irqCounter==0)m_irqPending=true; }
    void hardReset(){
        m_latch=m_commonChr=0;m_irqCounter=0;m_irqEnable=m_irqPending=m_lastA12=false;m_mmc4Lo=m_mmc4Hi=0;
        std::fill(std::begin(m_prg2),std::end(m_prg2),0);std::fill(std::begin(m_prg4),std::end(m_prg4),0);std::fill(std::begin(m_chr1),std::end(m_chr1),0);
        if(m_config.id==17){m_smcMode=0x47;m_oneMAddr=3;m_oneMData=0x20|((m_config.headerMirror==Mirror::Horizontal)?0x10:0);m_bankModeAddr=0;const std::size_t n=std::max<std::size_t>(4,m_config.prgRomSize/0x2000);for(unsigned i=0;i<4;++i)m_prg4[i]=uint8_t((n-4+i)&0x3F);}
        else if(m_config.id==12){m_smcMode=0x42;m_oneMAddr=3;m_oneMData=0x20|((m_config.headerMirror==Mirror::Horizontal)?0x10:0);m_bankModeAddr=0;const std::size_t n=std::max<std::size_t>(4,m_config.prgRomSize/0x2000);for(unsigned i=0;i<4;++i)m_prg4[i]=uint8_t((n-4+i)&0x3F);}
        else {m_smcMode=0x42;m_oneMAddr=3;unsigned sm=m_config.id==8?4:(m_config.nes20?m_config.submapper:1);m_oneMData=uint8_t((sm&7)<<5)|((m_config.headerMirror==Mirror::Horizontal)?0x10:0);m_bankModeAddr=3;}
        updateMirror();
    }
};
