// Hard mapper subsystems: MMC5 and Namco 163. Included inside Mapper.cpp's
// anonymous namespace so no additional Visual Studio project entries are needed.
namespace mapper_hard_detail {
inline void put16(std::vector<uint8_t>& o, uint16_t v) { put8(o, uint8_t(v)); put8(o, uint8_t(v >> 8)); }
inline bool get16(const uint8_t*& p, const uint8_t* e, uint16_t& v) { uint8_t l,h; if(!get8(p,e,l)||!get8(p,e,h)) return false; v=uint16_t(l)|(uint16_t(h)<<8); return true; }
inline void put32(std::vector<uint8_t>& o, uint32_t v) { for(int i=0;i<4;i++) put8(o,uint8_t(v>>(i*8))); }
inline bool get32(const uint8_t*& p,const uint8_t* e,uint32_t& v){v=0;for(int i=0;i<4;i++){uint8_t b;if(!get8(p,e,b))return false;v|=uint32_t(b)<<(i*8);}return true;}

static constexpr uint8_t kLengthTable[32] = {
    10,254,20,2,40,4,80,6,160,8,60,10,14,12,26,14,
    12,16,24,18,48,20,96,22,192,24,72,26,16,28,32,30
};

struct Mmc5Pulse {
    bool enabled=false, constant=false, halt=false, envelopeStart=false;
    uint8_t duty=0, volume=0, length=0, dutyPos=0, envelope=0, envelopeDiv=0;
    uint16_t period=0, timer=0;
    void writeControl(uint8_t d){ duty=(d>>6)&3; halt=(d&0x20)!=0; constant=(d&0x10)!=0; volume=d&0x0F; }
    void writeLow(uint8_t d){ period=uint16_t((period&0x700)|d); if(timer>period)timer=period; }
    void writeHigh(uint8_t d){ period=uint16_t((period&0xFF)|((uint16_t(d&7))<<8)); if(timer>period)timer=period; if(enabled) length=kLengthTable[d>>3]; dutyPos=0; envelopeStart=true; }
    void timerClock(){ if(++timer>period){timer=0; dutyPos=(dutyPos+1)&15;} }
    void quarter(){ if(envelopeStart){envelopeStart=false;envelope=15;envelopeDiv=volume;} else if(envelopeDiv==0){envelopeDiv=volume;if(envelope) --envelope; else if(halt) envelope=15;} else --envelopeDiv; }
    void half(){ if(!halt&&length) --length; }
    float sample() const { static const uint8_t dtab[4][16]={{0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0},{0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0},{1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1}}; if(!enabled||!length||!dtab[duty][dutyPos]) return 0; return float(constant?volume:envelope)/15.0f; }
    void save(std::vector<uint8_t>&o)const{put8(o,enabled);put8(o,constant);put8(o,halt);put8(o,envelopeStart);put8(o,duty);put8(o,volume);put8(o,length);put8(o,dutyPos);put8(o,envelope);put8(o,envelopeDiv);put16(o,period);put16(o,timer);}
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t b;if(!get8(p,e,b))return false;enabled=b;if(!get8(p,e,b))return false;constant=b;if(!get8(p,e,b))return false;halt=b;if(!get8(p,e,b))return false;envelopeStart=b;return get8(p,e,duty)&&get8(p,e,volume)&&get8(p,e,length)&&get8(p,e,dutyPos)&&get8(p,e,envelope)&&get8(p,e,envelopeDiv)&&get16(p,e,period)&&get16(p,e,timer);}
};
}

class Mapper5 final : public Mapper {
public:
    explicit Mapper5(const MapperConfig& c) : Mapper(c) {
        m_prgMode=3; m_chrMode=3; m_prgReg[4]=0xFF;
    }

    bool cpuReadRegister(uint16_t a,uint8_t& d,bool debug) override {
        if(a==0x5015){ d=uint8_t((m_pulse[0].length?1:0)|(m_pulse[1].length?2:0)); return true; }
        if(a==0x5204){ d=uint8_t((m_irqPending?0x80:0)|(m_inFrame?0x40:0)); if(!debug)m_irqPending=false; return true; }
        if(a==0x5205){ d=uint8_t(uint16_t(m_mulA)*m_mulB); return true; }
        if(a==0x5206){ d=uint8_t((uint16_t(m_mulA)*m_mulB)>>8); return true; }
        if(a>=0x5C00&&a<=0x5FFF){ if(m_exramMode>=2){d=m_exram[a&0x3FF];return true;} d=0; return true; }
        return false;
    }

    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;
        uint8_t reg=0; uint32_t off=0; uint32_t bankSize=0; bool forceRom=false;
        if(!decodePrg(a,reg,off,bankSize,forceRom)) return false;
        if(!forceRom && (reg&0x80)==0) return false;
        size_t bank=reg&0x7F;
        if(bankSize==0x4000) bank&=~size_t(1); else if(bankSize==0x8000) bank&=~size_t(3);
        m=mapBank(bank,0x2000,m_config.prgRomSize,off); // reg is always in 8 KiB units
        return true;
    }

    bool mapPrgRam(uint16_t a,uint32_t& m,bool write) const override {
        if(!m_config.prgRamSize) return false;
        uint8_t reg=0; uint32_t off=0, bankSize=0; bool forceRom=false;
        if(a>=0x6000&&a<0x8000){ reg=m_prgReg[0]; off=a-0x6000; }
        else if(a>=0x8000){ if(!decodePrg(a,reg,off,bankSize,forceRom)||forceRom||(reg&0x80))return false; }
        else return false;
        if(write && !(m_prot1==2 && m_prot2==1)) return false;
        const size_t bank=reg&0x07;
        m=static_cast<uint32_t>((bank*0x2000 + (off&0x1FFF))%m_config.prgRamSize);
        return true;
    }

    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a>=0x5000&&a<=0x5007){int ch=(a>=0x5004);uint8_t r=a&3;if(r==0)m_pulse[ch].writeControl(d);else if(r==2)m_pulse[ch].writeLow(d);else if(r==3)m_pulse[ch].writeHigh(d);return true;}
        if(a==0x5010){m_pcmReadMode=(d&1)!=0;return true;}
        if(a==0x5011){if(!m_pcmReadMode&&d!=0)m_pcm=d;return true;}
        if(a==0x5015){for(int i=0;i<2;i++){m_pulse[i].enabled=(d&(1<<i))!=0;if(!m_pulse[i].enabled)m_pulse[i].length=0;}return true;}
        if(a==0x5100){m_prgMode=d&3;return true;} if(a==0x5101){m_chrMode=d&3;return true;}
        if(a==0x5102){m_prot1=d&3;return true;} if(a==0x5103){m_prot2=d&3;return true;}
        if(a==0x5104){m_exramMode=d&3;return true;} if(a==0x5105){m_ntMap=d;return true;}
        if(a==0x5106){m_fillTile=d;return true;} if(a==0x5107){m_fillPalette=d&3;return true;}
        if(a>=0x5113&&a<=0x5117){m_prgReg[a-0x5113]=d;if(a==0x5117)m_prgReg[4]|=0x80;return true;}
        if(a>=0x5120&&a<=0x512B){m_chrReg[a-0x5120]=d;return true;}
        if(a==0x5130){m_chrUpper=d&3;return true;}
        if(a==0x5200){m_splitCtrl=d;return true;} if(a==0x5201){m_splitScroll=d;return true;} if(a==0x5202){m_splitBank=d;return true;}
        if(a==0x5203){m_irqLine=d;return true;} if(a==0x5204){m_irqEnable=(d&0x80)!=0;return true;}
        if(a==0x5205){m_mulA=d;return true;} if(a==0x5206){m_mulB=d;return true;}
        if(a>=0x5C00&&a<=0x5FFF){if(m_exramMode<=2){m_exram[a&0x3FF]=(m_exramMode<=1&&!m_inFrame)?0:d;}return true;}
        return a>=0x5000&&a<0x6000;
    }

    void observeCpuRead(uint16_t a,uint8_t d) override {
        // In PCM read mode the MMC5 DAC snoops normal CPU reads from
        // $8000-$BFFF. Zero bytes leave the DAC unchanged.
        if(m_pcmReadMode&&a>=0x8000&&a<0xC000&&d!=0)m_pcm=d;
    }

    bool ppuMapRead(uint16_t a,uint32_t& m) override { return ppuMapReadEx(a,m,PpuFetchKind::Cpu); }
    bool ppuMapReadEx(uint16_t a,uint32_t& m,PpuFetchKind kind) override {
        if(a>=0x2000)return false;const size_t s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;
        if(kind==PpuFetchKind::Background && m_splitPatternReads){m=static_cast<uint32_t>(((uint32_t(m_splitBank)<<12)+((a&~uint16_t(7))|(m_splitFineY&7)))%s);--m_splitPatternReads;return true;}
        if(kind==PpuFetchKind::Background && m_exAttrPatternReads){m=static_cast<uint32_t>(((uint32_t(m_exAttrChrBank)<<12)+(a&0x0FFF))%s);--m_exAttrPatternReads;return true;}
        const bool bg=(kind==PpuFetchKind::Background);uint16_t bank=chrBank(a,bg);m=mapBank(bank,0x400,s,a&0x3FF);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override { if(!m_config.chrRamSize)return false;return ppuMapReadEx(a,m,PpuFetchKind::Cpu); }

    bool mapNametable(uint16_t a,NametableSource& s,uint32_t& m) const override {
        if(a<0x2000||a>0x3EFF)return false;uint16_t n=(a-0x2000)&0x0FFF;uint8_t page=(n>>10)&3;uint16_t off=n&0x3FF;
        if(m_specialAttrFetch){s=NametableSource::MapperRam;m=0x800u|m_specialPalette;return true;}
        if(m_splitNtFetch){s=NametableSource::MapperRam;m=m_splitTileOffset;return true;}
        uint8_t mode=(m_ntMap>>(page*2))&3;if(mode<2){s=NametableSource::Ciram;m=uint32_t(mode)*0x400+off;}
        else {s=NametableSource::MapperRam;m=(mode==2)?off:uint32_t(0x400+off);}return true;
    }
    uint8_t readMapperNametable(uint32_t m) const override { if(m&0x800)return uint8_t((m&3)*0x55);if(m<0x400)return m_exram[m];uint16_t off=uint16_t(m-0x400)&0x3FF;return off<0x3C0?m_fillTile:uint8_t(m_fillPalette*0x55); }
    void writeMapperNametable(uint32_t m,uint8_t d) override { if(m<0x400&&m_exramMode<=1)m_exram[m]=d; }

    void notifyPpuAddressContext(uint16_t a,uint64_t cyc,int line,int dot) override {
        notifyPpuAddress(a,cyc);m_specialAttrFetch=false;m_splitNtFetch=false;
        if(!m_inFrame||line<0||line>=240)return;
        uint16_t aa=a&0x3FFF;bool nt=aa>=0x2000&&aa<=0x2FFF;uint16_t off=aa&0x3FF;
        if(nt&&off<0x3C0){
            m_exAttrNtOffset=off;m_exAttrPending=(m_exramMode==1);
            int col=(dot>=1&&dot<=256)?((dot-1)>>3):-1;bool splitEnable=(m_splitCtrl&0x80)&&m_exramMode<=1&&col>=0&&col<32;bool right=(m_splitCtrl&0x40)!=0;uint8_t delim=m_splitCtrl&0x1F;
            m_splitActive=splitEnable&&(right?(col>=delim):(col<delim));m_splitPatternReads=0;
            if(m_splitActive){uint8_t sy=uint8_t((line+m_splitScroll)%240);m_splitFineY=sy&7;m_splitColumn=uint8_t(col);m_splitTileOffset=uint16_t((sy>>3)*32+col);m_splitNtFetch=true;m_exAttrPending=false;}
        } else if(nt&&off>=0x3C0){
            if(m_splitActive){uint8_t ty=uint8_t(((line+m_splitScroll)%240)>>3);uint16_t at=uint16_t(0x3C0+((ty>>2)*8)+(m_splitColumn>>2));uint8_t sh=uint8_t(((ty&2)?4:0)|((m_splitColumn&2)?2:0));m_specialPalette=uint8_t((m_exram[at&0x3FF]>>sh)&3);m_specialAttrFetch=true;m_splitPatternReads=2;m_exAttrPending=false;}
            else if(m_exAttrPending&&m_exramMode==1){uint8_t v=m_exram[m_exAttrNtOffset&0x3FF];m_specialPalette=v>>6;m_exAttrChrBank=uint16_t((v&0x3F)|(uint16_t(m_chrUpper)<<6));m_specialAttrFetch=true;m_exAttrPatternReads=2;m_exAttrPending=false;}
        }
    }
    void notifyPpuScanline(int line,bool rendering) override {
        m_currentScanline=line;m_specialAttrFetch=false;m_splitNtFetch=false;m_splitPatternReads=0;m_exAttrPatternReads=0;m_splitActive=false;
        if(line>=0&&line<240&&rendering){m_inFrame=true;if(line==m_irqLine)m_irqPending=true;}
        else if(line==240||line<0){m_inFrame=false;}
    }
    void clockCpu() override {
        m_pulse[0].timerClock();m_pulse[1].timerClock();
        // Unlike the 2A03, MMC5 clocks both envelopes and length counters at
        // the 240 Hz frame-sequence rate.
        if(++m_frameAudio>7458){m_frameAudio=uint16_t(m_frameAudio-7458);m_pulse[0].quarter();m_pulse[1].quarter();m_pulse[0].half();m_pulse[1].half();}
    }
    bool irqActive() const override { return m_irqEnable&&m_irqPending; }
    float expansionAudioSample(bool = false) const override { float p=(m_pulse[0].sample()+m_pulse[1].sample())*0.18f;float pcm=float(m_pcm)/255.0f*0.20f;return -(p+pcm); }

    void saveState(std::vector<uint8_t>&o)const override {put8(o,m_prgMode);put8(o,m_chrMode);put8(o,m_prot1);put8(o,m_prot2);put8(o,m_exramMode);put8(o,m_ntMap);put8(o,m_fillTile);put8(o,m_fillPalette);for(auto v:m_prgReg)put8(o,v);for(auto v:m_chrReg)put8(o,v);put8(o,m_chrUpper);put8(o,m_splitCtrl);put8(o,m_splitScroll);put8(o,m_splitBank);put8(o,m_irqLine);put8(o,m_irqEnable);put8(o,m_irqPending);put8(o,m_inFrame);put8(o,m_mulA);put8(o,m_mulB);put8(o,m_pcmReadMode);put8(o,m_pcm);mapper_hard_detail::put16(o,m_frameAudio);mapper_hard_detail::put16(o,uint16_t(m_currentScanline));mapper_hard_detail::put16(o,m_exAttrNtOffset);mapper_hard_detail::put16(o,m_exAttrChrBank);put8(o,m_exAttrPending);put8(o,m_exAttrPatternReads);put8(o,m_specialAttrFetch);put8(o,m_specialPalette);put8(o,m_splitActive);put8(o,m_splitNtFetch);put8(o,m_splitPatternReads);put8(o,m_splitColumn);put8(o,m_splitFineY);mapper_hard_detail::put16(o,m_splitTileOffset);for(auto v:m_exram)put8(o,v);m_pulse[0].save(o);m_pulse[1].save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override {uint8_t b;if(!get8(p,e,m_prgMode)||!get8(p,e,m_chrMode)||!get8(p,e,m_prot1)||!get8(p,e,m_prot2)||!get8(p,e,m_exramMode)||!get8(p,e,m_ntMap)||!get8(p,e,m_fillTile)||!get8(p,e,m_fillPalette))return false;for(auto&v:m_prgReg)if(!get8(p,e,v))return false;for(auto&v:m_chrReg)if(!get8(p,e,v))return false;if(!get8(p,e,m_chrUpper)||!get8(p,e,m_splitCtrl)||!get8(p,e,m_splitScroll)||!get8(p,e,m_splitBank)||!get8(p,e,m_irqLine))return false;if(!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,b))return false;m_inFrame=b;if(!get8(p,e,m_mulA)||!get8(p,e,m_mulB)||!get8(p,e,b))return false;m_pcmReadMode=b;if(!get8(p,e,m_pcm)||!mapper_hard_detail::get16(p,e,m_frameAudio))return false;uint16_t q=0;if(!mapper_hard_detail::get16(p,e,q))return false;m_currentScanline=int16_t(q);if(!mapper_hard_detail::get16(p,e,m_exAttrNtOffset)||!mapper_hard_detail::get16(p,e,m_exAttrChrBank))return false;if(!get8(p,e,b))return false;m_exAttrPending=b;if(!get8(p,e,m_exAttrPatternReads)||!get8(p,e,b))return false;m_specialAttrFetch=b;if(!get8(p,e,m_specialPalette)||!get8(p,e,b))return false;m_splitActive=b;if(!get8(p,e,b))return false;m_splitNtFetch=b;if(!get8(p,e,m_splitPatternReads)||!get8(p,e,m_splitColumn)||!get8(p,e,m_splitFineY)||!mapper_hard_detail::get16(p,e,m_splitTileOffset))return false;for(auto&v:m_exram)if(!get8(p,e,v))return false;return m_pulse[0].load(p,e)&&m_pulse[1].load(p,e);}
private:
    uint8_t m_prgMode=3,m_chrMode=3,m_prot1=0,m_prot2=0,m_exramMode=0,m_ntMap=0,m_fillTile=0,m_fillPalette=0;
    uint8_t m_prgReg[5]={0,0,0,0,0xFF},m_chrReg[12]={},m_chrUpper=0;
    uint8_t m_splitCtrl=0,m_splitScroll=0,m_splitBank=0,m_irqLine=0,m_mulA=0,m_mulB=0,m_pcm=0;
    bool m_irqEnable=false,m_irqPending=false,m_inFrame=false,m_pcmReadMode=false;uint16_t m_frameAudio=0;
    int m_currentScanline=-1;uint16_t m_exAttrNtOffset=0,m_exAttrChrBank=0,m_splitTileOffset=0;bool m_exAttrPending=false,m_specialAttrFetch=false,m_splitActive=false,m_splitNtFetch=false;uint8_t m_exAttrPatternReads=0,m_specialPalette=0,m_splitPatternReads=0,m_splitColumn=0,m_splitFineY=0;
    uint8_t m_exram[0x400]={};mapper_hard_detail::Mmc5Pulse m_pulse[2];

    bool decodePrg(uint16_t a,uint8_t&reg,uint32_t&off,uint32_t&size,bool&forceRom)const{
        forceRom=false;if(a<0x8000)return false;
        switch(m_prgMode){case 0:reg=m_prgReg[4];size=0x8000;off=a-0x8000;forceRom=true;break;case 1:if(a<0xC000){reg=m_prgReg[2];size=0x4000;off=a-0x8000;}else{reg=m_prgReg[4];size=0x4000;off=a-0xC000;forceRom=true;}break;case 2:if(a<0xC000){reg=m_prgReg[2];size=0x4000;off=a-0x8000;}else if(a<0xE000){reg=m_prgReg[3];size=0x2000;off=a-0xC000;}else{reg=m_prgReg[4];size=0x2000;off=a-0xE000;forceRom=true;}break;default:{int i=(a-0x8000)>>13;reg=m_prgReg[1+i];size=0x2000;off=a&0x1FFF;if(i==3)forceRom=true;}break;}return true;}
    uint16_t chrBank(uint16_t a,bool bg)const{uint8_t slot=(a>>10)&7;uint8_t r=0;if(bg){uint8_t s=slot&3;switch(m_chrMode){case 0:r=m_chrReg[11]&~7;break;case 1:r=uint8_t((m_chrReg[11]&~3)+(slot&3));break;case 2:r=uint8_t((m_chrReg[9+(s>>1)]&~1)+(s&1));break;default:r=m_chrReg[8+s];break;}}else{switch(m_chrMode){case 0:r=uint8_t((m_chrReg[7]&~7)+slot);break;case 1:r=uint8_t((m_chrReg[(slot<4)?3:7]&~3)+(slot&3));break;case 2:r=uint8_t((m_chrReg[(slot&~1)|1]&~1)+(slot&1));break;default:r=m_chrReg[slot];break;}}return uint16_t((uint16_t(m_chrUpper)<<8)|r);}
};

class Mapper19 final : public Mapper {
public:
    enum class Variant : uint8_t { Namco163, Namco175, Namco340 };
    explicit Mapper19(const MapperConfig&c):Mapper(c){if(c.id==210)m_variant=(c.submapper==2)?Variant::Namco340:Variant::Namco175;}
    bool implementationSupported() const override{return m_config.id!=210||m_config.submapper==1||m_config.submapper==2;}
    bool cpuReadRegister(uint16_t a,uint8_t&d,bool debug)override{
        if(m_variant!=Variant::Namco163)return false;
        if(a==0x4800){d=m_ram[m_portAddr&0x7F];if(!debug&&(m_portAddr&0x80))m_portAddr=uint8_t(0x80|((m_portAddr+1)&0x7F));return true;}
        if(a==0x5000){d=uint8_t(m_irqCounter);return true;}if(a==0x5800){d=uint8_t((m_irqCounter>>8)&0x7F)|(m_irqEnabled?0x80:0);return true;}return false;
    }
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        switch(a&0xF800){
        case 0x4800:if(m_variant!=Variant::Namco163)return false;{uint8_t ra=m_portAddr&0x7F;m_ram[ra]=d;syncChipPhaseWrite(ra);if(m_portAddr&0x80)m_portAddr=uint8_t(0x80|((m_portAddr+1)&0x7F));return true;}
        case 0x5000:if(m_variant!=Variant::Namco163)return false;m_irqCounter=uint16_t((m_irqCounter&0x7F00)|d);m_irqPending=false;return true;
        case 0x5800:if(m_variant!=Variant::Namco163)return false;m_irqCounter=uint16_t((m_irqCounter&0x00FF)|((uint16_t(d&0x7F))<<8));m_irqEnabled=(d&0x80)!=0;m_irqPending=false;return true;
        case 0x8000:case 0x8800:case 0x9000:case 0x9800:m_chr[(a-0x8000)>>11]=d;return true;
        case 0xA000:case 0xA800:case 0xB000:case 0xB800:m_chr[4+((a-0xA000)>>11)]=d;return true;
        case 0xC000:case 0xC800:case 0xD000:case 0xD800:if(m_variant==Variant::Namco175){m_ramProtect=d;return true;}if(m_variant==Variant::Namco163){m_nt[(a-0xC000)>>11]=d;return true;}return false;
        case 0xE000:m_prg[0]=d&0x3F;if(m_variant==Variant::Namco340){switch((d>>6)&3){case 0:m_mirror=Mirror::OnescreenLo;break;case 1:m_mirror=Mirror::Vertical;break;case 2:m_mirror=Mirror::Horizontal;break;case 3:m_mirror=Mirror::OnescreenHi;break;}}else if(m_variant==Variant::Namco163)m_audioDisabled=(d&0x40)!=0;return true;
        case 0xE800:m_prg[1]=d&0x3F;if(m_variant==Variant::Namco163){m_chrRamDisable[0]=(d&0x40)!=0;m_chrRamDisable[1]=(d&0x80)!=0;}return true;
        case 0xF000:m_prg[2]=d&0x3F;return true;
        case 0xF800:if(m_variant!=Variant::Namco163)return false;m_portAddr=d;m_ramProtect=d;return true;}return false;
    }
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;uint8_t b=m_chr[a>>10];m=mapBank(b,0x400,s,a&0x3FF);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{if(!m_config.chrRamSize||a>=0x2000)return false;uint8_t slot=a>>10;if((slot<4&&m_chrRamDisable[0])||(slot>=4&&m_chrRamDisable[1]))return false;m=mapBank(m_chr[slot],0x400,m_config.chrRamSize,a&0x3FF);return true;}
    bool ppuUsesChrRam(uint16_t a)const override{return a<0x2000&&m_config.chrRomSize==0&&m_config.chrRamSize!=0;}
    bool mapPatternCiram(uint16_t a,uint32_t&m)const override{if(m_variant!=Variant::Namco163||a>=0x2000)return false;uint8_t slot=a>>10;uint8_t b=m_chr[slot];bool forceChr=(slot<4)?m_chrRamDisable[0]:m_chrRamDisable[1];if(!forceChr&&b>=0xE0){m=uint32_t(b&1)*0x400+(a&0x3FF);return true;}return false;}
    bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(m_variant!=Variant::Namco163||a<0x2000||a>0x3EFF)return false;uint16_t n=(a-0x2000)&0xFFF;uint8_t b=m_nt[(n>>10)&3];if(b>=0xE0){s=NametableSource::Ciram;m=uint32_t(b&1)*0x400+(n&0x3FF);}else{s=m_config.chrRomSize?NametableSource::ChrRom:NametableSource::ChrRam;m=mapBank(b,0x400,m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize,n&0x3FF);}return true;}
    bool mapPrgRam(uint16_t a,uint32_t&m,bool write)const override{if(a<0x6000||a>0x7FFF||!m_config.prgRamSize)return false;if(m_variant==Variant::Namco340)return false;if(m_variant==Variant::Namco175){if(write&&(m_ramProtect&1)==0)return false;m=(a-0x6000)%m_config.prgRamSize;return true;}uint8_t seg=(a-0x6000)>>11;if(write){if((m_ramProtect&0x40)==0)return false;if(m_ramProtect&(1u<<seg))return false;}m=(a-0x6000)%m_config.prgRamSize;return true;}
    void clockCpu()override{if(m_variant!=Variant::Namco163)return;if(m_irqEnabled&&m_irqCounter<0x7FFF){++m_irqCounter;if(m_irqCounter==0x7FFF)m_irqPending=true;}if(!m_audioDisabled){clockChipAudio();if(++m_audioDiv>=15){m_audioDiv=0;clockAudioChannel();}}}
    bool irqActive()const override{return m_variant==Variant::Namco163&&m_irqEnabled&&m_irqPending;}
    float expansionAudioSample(bool chipMod = false)const override{if(m_variant!=Variant::Namco163||m_audioDisabled)return 0.0f;uint8_t count=channelCount();if(!count)return 0.0f;if(!chipMod)return std::max(-1.0f,std::min(1.0f,float(m_serialOut)/120.0f))*0.55f;float sum=0.0f;for(int ch=8-count;ch<8;ch++){uint8_t base=uint8_t(0x40+ch*8);uint16_t len=uint16_t(256-(m_ram[base+4]&0xFC));if(len<4)len=4;uint32_t rawPos=uint32_t(m_chipPhase[ch]/65536.0)+m_ram[base+6];uint8_t pos=uint8_t(std::min<uint32_t>(255,rawPos));uint8_t packed=m_ram[pos>>1];uint8_t sample=(pos&1)?(packed>>4):(packed&0x0F);uint8_t vol=m_ram[base+7]&0x0F;sum+=float(8-int(sample))*vol;}uint8_t divisor=std::min<uint8_t>(count,6);return std::max(-1.0f,std::min(1.0f,sum/(120.0f*divisor)))*0.55f;}
    std::size_t mapperBatterySize()const override{return m_variant==Variant::Namco163?sizeof(m_ram):0;}
    void saveMapperBattery(std::vector<uint8_t>&o)const override{if(m_variant==Variant::Namco163)o.insert(o.end(),m_ram,m_ram+sizeof(m_ram));}
    bool loadMapperBattery(const uint8_t*d,std::size_t n)override{if(m_variant!=Variant::Namco163)return n==0;if(n!=sizeof(m_ram))return false;std::memcpy(m_ram,d,n);rebuildChipPhases();return true;}
    void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_chr)put8(o,v);for(auto v:m_nt)put8(o,v);for(auto v:m_prg)put8(o,v);for(auto v:m_ram)put8(o,v);put8(o,m_portAddr);put8(o,m_ramProtect);mapper_hard_detail::put16(o,m_irqCounter);put8(o,m_irqEnabled);put8(o,m_irqPending);put8(o,m_audioDiv);put8(o,m_audioChannel);for(auto v:m_channelOut){int16_t q=int16_t(v);mapper_hard_detail::put16(o,uint16_t(q));}mapper_hard_detail::put16(o,uint16_t(m_serialOut));put8(o,m_chrRamDisable[0]);put8(o,m_chrRamDisable[1]);put8(o,m_audioDisabled);for(double v:m_chipPhase){uint64_t bits=0;static_assert(sizeof(bits)==sizeof(v),"unexpected double size");std::memcpy(&bits,&v,sizeof(bits));put64(o,bits);}}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_chr)if(!get8(p,e,v))return false;for(auto&v:m_nt)if(!get8(p,e,v))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_ram)if(!get8(p,e,v))return false;if(!get8(p,e,m_portAddr)||!get8(p,e,m_ramProtect)||!mapper_hard_detail::get16(p,e,m_irqCounter))return false;if(!get8(p,e,b))return false;m_irqEnabled=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,m_audioDiv)||!get8(p,e,m_audioChannel))return false;for(auto&v:m_channelOut){uint16_t q;if(!mapper_hard_detail::get16(p,e,q))return false;v=int16_t(q);}uint16_t serial;if(!mapper_hard_detail::get16(p,e,serial))return false;m_serialOut=int16_t(serial);if(!get8(p,e,b))return false;m_chrRamDisable[0]=b;if(!get8(p,e,b))return false;m_chrRamDisable[1]=b;if(!get8(p,e,b))return false;m_audioDisabled=b;for(double&v:m_chipPhase){uint64_t bits=0;if(!get64(p,e,bits))return false;std::memcpy(&v,&bits,sizeof(v));if(!std::isfinite(v))return false;}return true;}
private:
    Variant m_variant=Variant::Namco163;uint8_t m_chr[8]={},m_nt[4]={0xE0,0xE1,0xE0,0xE1},m_prg[3]={},m_ram[128]={},m_portAddr=0,m_ramProtect=0,m_audioDiv=0,m_audioChannel=7;bool m_chrRamDisable[2]={};uint16_t m_irqCounter=0;bool m_irqEnabled=false,m_irqPending=false,m_audioDisabled=false;int16_t m_channelOut[8]={},m_serialOut=0;double m_chipPhase[8]={};
    uint8_t channelCount()const{return uint8_t(((m_ram[0x7F]>>4)&7)+1);}
    uint32_t ramPhase(int ch)const{uint8_t base=uint8_t(0x40+ch*8);return uint32_t(m_ram[base+1])|(uint32_t(m_ram[base+3])<<8)|(uint32_t(m_ram[base+5])<<16);}
    void rebuildChipPhases(){for(int ch=0;ch<8;ch++)m_chipPhase[ch]=double(ramPhase(ch));}
    void syncChipPhaseWrite(uint8_t ra){if(ra<0x40)return;uint8_t rel=uint8_t(ra-0x40),field=rel&7;if(field!=1&&field!=3&&field!=5)return;int ch=rel>>3;if(ch>=0&&ch<8)m_chipPhase[ch]=double(ramPhase(ch));}
    void clockChipAudio(){uint8_t count=channelCount();if(!count)return;for(int ch=8-count;ch<8;ch++){uint8_t base=uint8_t(0x40+ch*8);uint32_t freq=uint32_t(m_ram[base])|(uint32_t(m_ram[base+2])<<8)|(uint32_t(m_ram[base+4]&3)<<16);uint16_t len=uint16_t(256-(m_ram[base+4]&0xFC));if(len<4)len=4;double mod=double(uint32_t(len)<<16);m_chipPhase[ch]+=double(freq)/(15.0*double(count));if(m_chipPhase[ch]>=mod)m_chipPhase[ch]=std::fmod(m_chipPhase[ch],mod);}}
    void clockAudioChannel(){uint8_t count=channelCount();uint8_t first=uint8_t(8-count);if(m_audioChannel<first||m_audioChannel>7)m_audioChannel=7;uint8_t ch=m_audioChannel;uint8_t base=uint8_t(0x40+ch*8);uint32_t freq=uint32_t(m_ram[base])|(uint32_t(m_ram[base+2])<<8)|(uint32_t(m_ram[base+4]&3)<<16);uint32_t phase=uint32_t(m_ram[base+1])|(uint32_t(m_ram[base+3])<<8)|(uint32_t(m_ram[base+5])<<16);uint16_t len=uint16_t(256-(m_ram[base+4]&0xFC));if(len<4)len=4;uint32_t mod=uint32_t(len)<<16;phase=(phase+freq)%mod;m_ram[base+1]=uint8_t(phase);m_ram[base+3]=uint8_t(phase>>8);m_ram[base+5]=uint8_t(phase>>16);uint8_t pos=uint8_t(((phase>>16)+m_ram[base+6])&0xFF);uint8_t packed=m_ram[pos>>1];uint8_t sample=(pos&1)?(packed>>4):(packed&0x0F);uint8_t vol=m_ram[base+7]&0x0F;m_channelOut[ch]=int16_t(8-int(sample))*vol;m_serialOut=m_channelOut[ch];if(ch==first)m_audioChannel=7;else --m_audioChannel;}
};
