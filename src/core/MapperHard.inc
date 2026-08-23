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

    bool cpuReadRegister(uint16_t a,uint8_t& d) override {
        if(a==0x5015){ d=uint8_t((m_pulse[0].length?1:0)|(m_pulse[1].length?2:0)); return true; }
        if(a==0x5204){ d=uint8_t((m_irqPending?0x80:0)|(m_inFrame?0x40:0)); if(true)m_irqPending=false; return true; }
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
        // Reading the NMI vector resets MMC5's scanline detector/frame state.
        // These are ordinary cartridge ROM reads, so they naturally arrive
        // through this observation hook.
        if(a==0xFFFA || a==0xFFFB) resetScanlineDetector(true);
        // In PCM read mode the MMC5 DAC snoops normal CPU reads from
        // $8000-$BFFF. Zero bytes leave the DAC unchanged.
        if(m_pcmReadMode&&a>=0x8000&&a<0xC000&&d!=0)m_pcm=d;
    }
    void observeCpuWrite(uint16_t a,uint8_t d) override {
        // Rendering disable is observable by MMC5 and immediately tears down
        // its in-frame detector. OAM DMA is also believed to reset the scanline
        // detector; model it here because Bus exposes every CPU write.
        if((a&0xE007)==0x2001 && (d&0x18)==0) resetScanlineDetector(true);
        else if(a==0x4014) resetScanlineDetector(true);
    }

    bool ppuMapRead(uint16_t a,uint32_t& m) override { return ppuMapReadEx(a,m,PpuFetchKind::Cpu); }
    bool ppuMapReadEx(uint16_t a,uint32_t& m,PpuFetchKind kind) override {
        if(a>=0x2000)return false;
        const size_t s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!s)return false;
        if(kind==PpuFetchKind::Background && m_splitPatternReads){m=static_cast<uint32_t>(((uint32_t(m_splitBank)<<12)+((a&~uint16_t(7))|(m_splitFineY&7)))%s);--m_splitPatternReads;return true;}
        if(kind==PpuFetchKind::Background && m_exAttrPatternReads){m=static_cast<uint32_t>(((uint32_t(m_exAttrChrBank)<<12)+(a&0x0FFF))%s);--m_exAttrPatternReads;return true;}
        const bool bg=(kind==PpuFetchKind::Background);uint16_t bank=chrBank(a,bg);m=mapBank(bank,0x400,s,a&0x3FF);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override { if(!m_config.chrRamSize)return false;return ppuMapReadEx(a,m,PpuFetchKind::Cpu); }

    bool mapNametable(uint16_t a,NametableSource& s,uint32_t& m) const override {
        if(a<0x2000||a>0x3EFF)return false;
        uint16_t n=(a-0x2000)&0x0FFF;
        uint8_t page=(n>>10)&3;
        uint16_t off=n&0x3FF;
        if(m_specialAttrFetch){s=NametableSource::MapperRam;m=0x800u|m_specialPalette;return true;}
        if(m_splitNtFetch){s=NametableSource::MapperRam;m=m_splitTileOffset;return true;}
        uint8_t mode=(m_ntMap>>(page*2))&3;if(mode<2){s=NametableSource::Ciram;m=uint32_t(mode)*0x400+off;}
        else {s=NametableSource::MapperRam;m=(mode==2)?off:uint32_t(0x400+off);}return true;
    }
    uint8_t readMapperNametable(uint32_t m) const override { if(m&0x800)return uint8_t((m&3)*0x55);if(m<0x400)return m_exram[m];uint16_t off=uint16_t(m-0x400)&0x3FF;return off<0x3C0?m_fillTile:uint8_t(m_fillPalette*0x55); }
    void writeMapperNametable(uint32_t m,uint8_t d) override { if(m<0x400&&m_exramMode<=1)m_exram[m]=d; }

    void notifyPpuAddressContext(uint16_t a,uint64_t cyc,int line,int /*dot*/) override {
        notifyPpuAddress(a,cyc);
        clockScanlineDetector(a);
        m_specialAttrFetch=false;m_splitNtFetch=false;
        uint16_t aa=a&0x3FFF;bool nt=aa>=0x2000&&aa<=0x2FFF;uint16_t off=aa&0x3FF;

        // MMC5 locates the vertical split from the number of nametable tile
        // fetches, not from the PPU dot or the nametable address. The two
        // background tiles prefetched at the end of the previous scanline are
        // columns 0/1, so the dot-1 tile fetch is screen column 2. Sprite-fetch
        // garbage nametable reads advance the same hardware counter through
        // slots 32-39. The 337/339/1 synchronization sequence resets it.
        const bool tileNtFetch=nt&&off<0x3C0;
        if(tileNtFetch && m_splitTileNumber<63) ++m_splitTileNumber;
        if(!m_inFrame||line<0||line>=240)return;
        if(tileNtFetch){
            m_exAttrNtOffset=off;m_exAttrPending=(m_exramMode==1);
            const uint8_t col=uint8_t((m_splitTileNumber+1)%42);
            const bool splitEnable=(m_splitCtrl&0x80)&&m_exramMode<=1&&col<32;
            const bool right=(m_splitCtrl&0x40)!=0;const uint8_t delim=m_splitCtrl&0x1F;
            m_splitActive=splitEnable&&(right?(col>=delim):(col<delim));m_splitPatternReads=0;
            if(m_splitActive){uint8_t sy=uint8_t((line+m_splitScroll)%240);m_splitFineY=sy&7;m_splitColumn=col;m_splitTileOffset=uint16_t((sy>>3)*32+col);m_splitNtFetch=true;m_exAttrPending=false;}
        } else if(nt&&off>=0x3C0){
            if(m_splitActive){uint8_t ty=uint8_t(((line+m_splitScroll)%240)>>3);uint16_t at=uint16_t(0x3C0+((ty>>2)*8)+(m_splitColumn>>2));uint8_t sh=uint8_t(((ty&2)?4:0)|((m_splitColumn&2)?2:0));m_specialPalette=uint8_t((m_exram[at&0x3FF]>>sh)&3);m_specialAttrFetch=true;m_splitPatternReads=2;m_exAttrPending=false;}
            else if(m_exAttrPending&&m_exramMode==1){uint8_t v=m_exram[m_exAttrNtOffset&0x3FF];m_specialPalette=v>>6;m_exAttrChrBank=uint16_t((v&0x3F)|(uint16_t(m_chrUpper)<<6));m_specialAttrFetch=true;m_exAttrPatternReads=2;m_exAttrPending=false;}
        }
    }
    void notifyPpuScanline(int line,bool) override {
        // Keep the PPU line number only for split-screen vertical addressing.
        // MMC5 scanline/frame/IRQ state is derived from the PPU read stream,
        // not from this emulator-side scanline callback.
        m_currentScanline=line;m_specialAttrFetch=false;m_splitNtFetch=false;m_splitPatternReads=0;m_exAttrPatternReads=0;m_splitActive=false;
    }
    void clockCpu() override {
        // The in-frame detector times out after three CPU clocks with no PPU
        // read. This is how rendering disable and the post-render gap become
        // visible to the real MMC5 even without a dedicated scanline input.
        if(m_ppuReadSeen){m_ppuReadSeen=false;m_ppuIdleCpu=0;}
        else if(m_ppuIdleCpu<3 && ++m_ppuIdleCpu>=3) resetScanlineDetector(true);
        m_pulse[0].timerClock();m_pulse[1].timerClock();
        // Unlike the 2A03, MMC5 clocks both envelopes and length counters at
        // the 240 Hz frame-sequence rate.
        if(++m_frameAudio>7458){m_frameAudio=uint16_t(m_frameAudio-7458);m_pulse[0].quarter();m_pulse[1].quarter();m_pulse[0].half();m_pulse[1].half();}
    }
    bool irqActive() const override { return m_irqEnable&&m_irqPending; }
    float expansionAudioSample(bool = false) const override { float p=(m_pulse[0].sample()+m_pulse[1].sample())*0.18f;float pcm=float(m_pcm)/255.0f*0.20f;return -(p+pcm); }

    void saveState(std::vector<uint8_t>&o)const override {put8(o,m_prgMode);put8(o,m_chrMode);put8(o,m_prot1);put8(o,m_prot2);put8(o,m_exramMode);put8(o,m_ntMap);put8(o,m_fillTile);put8(o,m_fillPalette);for(auto v:m_prgReg)put8(o,v);for(auto v:m_chrReg)put8(o,v);put8(o,m_chrUpper);put8(o,m_splitCtrl);put8(o,m_splitScroll);put8(o,m_splitBank);put8(o,m_irqLine);put8(o,m_irqEnable);put8(o,m_irqPending);put8(o,m_inFrame);mapper_hard_detail::put16(o,m_scanLastNt);put8(o,m_scanMatchCount);put8(o,m_scanlineCounter);put8(o,m_ppuIdleCpu);put8(o,m_scanSyncPending);put8(o,m_ppuReadSeen);put8(o,m_mulA);put8(o,m_mulB);put8(o,m_pcmReadMode);put8(o,m_pcm);mapper_hard_detail::put16(o,m_frameAudio);mapper_hard_detail::put16(o,uint16_t(m_currentScanline));mapper_hard_detail::put16(o,m_exAttrNtOffset);mapper_hard_detail::put16(o,m_exAttrChrBank);put8(o,m_exAttrPending);put8(o,m_exAttrPatternReads);put8(o,m_specialAttrFetch);put8(o,m_specialPalette);put8(o,m_splitActive);put8(o,m_splitNtFetch);put8(o,m_splitPatternReads);put8(o,m_splitColumn);put8(o,m_splitFineY);put8(o,m_splitTileNumber);mapper_hard_detail::put16(o,m_splitTileOffset);for(auto v:m_exram)put8(o,v);m_pulse[0].save(o);m_pulse[1].save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override {uint8_t b;if(!get8(p,e,m_prgMode)||!get8(p,e,m_chrMode)||!get8(p,e,m_prot1)||!get8(p,e,m_prot2)||!get8(p,e,m_exramMode)||!get8(p,e,m_ntMap)||!get8(p,e,m_fillTile)||!get8(p,e,m_fillPalette))return false;for(auto&v:m_prgReg)if(!get8(p,e,v))return false;for(auto&v:m_chrReg)if(!get8(p,e,v))return false;if(!get8(p,e,m_chrUpper)||!get8(p,e,m_splitCtrl)||!get8(p,e,m_splitScroll)||!get8(p,e,m_splitBank)||!get8(p,e,m_irqLine))return false;if(!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,b))return false;m_inFrame=b;if(!mapper_hard_detail::get16(p,e,m_scanLastNt)||!get8(p,e,m_scanMatchCount)||!get8(p,e,m_scanlineCounter)||!get8(p,e,m_ppuIdleCpu)||!get8(p,e,b))return false;m_scanSyncPending=b;if(!get8(p,e,b))return false;m_ppuReadSeen=b;if(!get8(p,e,m_mulA)||!get8(p,e,m_mulB)||!get8(p,e,b))return false;m_pcmReadMode=b;if(!get8(p,e,m_pcm)||!mapper_hard_detail::get16(p,e,m_frameAudio))return false;uint16_t q=0;if(!mapper_hard_detail::get16(p,e,q))return false;m_currentScanline=int16_t(q);if(!mapper_hard_detail::get16(p,e,m_exAttrNtOffset)||!mapper_hard_detail::get16(p,e,m_exAttrChrBank))return false;if(!get8(p,e,b))return false;m_exAttrPending=b;if(!get8(p,e,m_exAttrPatternReads)||!get8(p,e,b))return false;m_specialAttrFetch=b;if(!get8(p,e,m_specialPalette)||!get8(p,e,b))return false;m_splitActive=b;if(!get8(p,e,b))return false;m_splitNtFetch=b;if(!get8(p,e,m_splitPatternReads)||!get8(p,e,m_splitColumn)||!get8(p,e,m_splitFineY)||!get8(p,e,m_splitTileNumber)||!mapper_hard_detail::get16(p,e,m_splitTileOffset))return false;for(auto&v:m_exram)if(!get8(p,e,v))return false;return m_pulse[0].load(p,e)&&m_pulse[1].load(p,e);}
private:
    uint8_t m_prgMode=3,m_chrMode=3,m_prot1=0,m_prot2=0,m_exramMode=0,m_ntMap=0,m_fillTile=0,m_fillPalette=0;
    uint8_t m_prgReg[5]={0,0,0,0,0xFF},m_chrReg[12]={},m_chrUpper=0;
    uint8_t m_splitCtrl=0,m_splitScroll=0,m_splitBank=0,m_irqLine=0,m_mulA=0,m_mulB=0,m_pcm=0;
    bool m_irqEnable=false,m_irqPending=false,m_inFrame=false,m_pcmReadMode=false;uint16_t m_frameAudio=0;
    uint16_t m_scanLastNt=0;uint8_t m_scanMatchCount=0,m_scanlineCounter=0,m_ppuIdleCpu=0;bool m_scanSyncPending=false,m_ppuReadSeen=false;
    int m_currentScanline=-1;uint16_t m_exAttrNtOffset=0,m_exAttrChrBank=0,m_splitTileOffset=0;bool m_exAttrPending=false,m_specialAttrFetch=false,m_splitActive=false,m_splitNtFetch=false;uint8_t m_exAttrPatternReads=0,m_specialPalette=0,m_splitPatternReads=0,m_splitColumn=0,m_splitFineY=0,m_splitTileNumber=0;
    uint8_t m_exram[0x400]={};mapper_hard_detail::Mmc5Pulse m_pulse[2];


    void resetScanlineDetector(bool clearIrq) {
        m_inFrame=false;m_scanMatchCount=0;m_scanSyncPending=false;m_scanLastNt=0;
        m_scanlineCounter=0;m_ppuIdleCpu=0;m_ppuReadSeen=false;m_splitTileNumber=0;
        if(clearIrq)m_irqPending=false;
    }
    void clockScanlineDetector(uint16_t a) {
        m_ppuReadSeen=true;
        const uint16_t aa=a&0x3FFF;

        // After three identical nametable reads, the following PPU read marks
        // the new scanline. With a normal 2C02 stream these are dots 337, 339,
        // and 1 followed by the attribute fetch at dot 3.
        if(m_scanSyncPending){
            m_scanSyncPending=false;
            m_scanMatchCount=0;
            if(!m_inFrame){m_inFrame=true;m_scanlineCounter=0;m_irqPending=false;}
            else {
                ++m_scanlineCounter;
                if(m_irqLine!=0 && m_scanlineCounter==m_irqLine)m_irqPending=true;
            }
        }

        const bool nt=aa>=0x2000&&aa<=0x2FFF;
        if(nt){
            if(m_scanMatchCount!=0 && aa==m_scanLastNt){
                if(m_scanMatchCount<3)++m_scanMatchCount;
            } else m_scanMatchCount=1;
            m_scanLastNt=aa;
            if(m_scanMatchCount>=3){m_scanSyncPending=true;m_splitTileNumber=0;}
        } else {
            m_scanMatchCount=0;
        }
    }

    bool decodePrg(uint16_t a,uint8_t&reg,uint32_t&off,uint32_t&size,bool&forceRom)const{
        forceRom=false;if(a<0x8000)return false;
        switch(m_prgMode){case 0:reg=m_prgReg[4];size=0x8000;off=a-0x8000;forceRom=true;break;case 1:if(a<0xC000){reg=m_prgReg[2];size=0x4000;off=a-0x8000;}else{reg=m_prgReg[4];size=0x4000;off=a-0xC000;forceRom=true;}break;case 2:if(a<0xC000){reg=m_prgReg[2];size=0x4000;off=a-0x8000;}else if(a<0xE000){reg=m_prgReg[3];size=0x2000;off=a-0xC000;}else{reg=m_prgReg[4];size=0x2000;off=a-0xE000;forceRom=true;}break;default:{int i=(a-0x8000)>>13;reg=m_prgReg[1+i];size=0x2000;off=a&0x1FFF;if(i==3)forceRom=true;}break;}return true;}
    uint16_t chrBank(uint16_t a,bool bg)const{uint8_t slot=(a>>10)&7;uint8_t r=0;if(bg){uint8_t s=slot&3;switch(m_chrMode){case 0:r=m_chrReg[11]&~7;break;case 1:r=uint8_t((m_chrReg[11]&~3)+(slot&3));break;case 2:r=uint8_t((m_chrReg[9+(s>>1)]&~1)+(s&1));break;default:r=m_chrReg[8+s];break;}}else{switch(m_chrMode){case 0:r=uint8_t((m_chrReg[7]&~7)+slot);break;case 1:r=uint8_t((m_chrReg[(slot<4)?3:7]&~3)+(slot&3));break;case 2:r=uint8_t((m_chrReg[(slot&~1)|1]&~1)+(slot&1));break;default:r=m_chrReg[slot];break;}}return uint16_t((uint16_t(m_chrUpper)<<8)|r);}
};

class Mapper19 final : public Mapper {
public:
    enum class Variant : uint8_t { Namco163, Namco175, Namco340 };
    explicit Mapper19(const MapperConfig&c):Mapper(c){
        if(c.id==210){
            // NES 2.0 submapper 0 preserves the legacy mapper-210 ambiguity.
            // Resolve it using the standardized compatibility rule: images
            // declaring PRG RAM/battery are Namco 175, otherwise Namco 340.
            if(c.submapper==0)m_variant=(c.prgRamSize||c.headerPrgNvRamSize||c.hasBattery)?Variant::Namco175:Variant::Namco340;
            else m_variant=(c.submapper==2)?Variant::Namco340:Variant::Namco175;
        }
    }
    bool implementationSupported() const override{return m_config.id!=210||m_config.submapper<=2;}
    bool cpuReadRegister(uint16_t a,uint8_t&d)override{
        if(m_variant!=Variant::Namco163)return false;
        if(a==0x4800){d=m_ram[m_portAddr&0x7F];if((m_portAddr&0x80)&&(m_portAddr&0x7F)!=0x7F)++m_portAddr;return true;}
        if(a==0x5000){d=uint8_t(m_irqCounter);return true;}if(a==0x5800){d=uint8_t((m_irqCounter>>8)&0x7F)|(m_irqEnabled?0x80:0);return true;}return false;
    }
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        switch(a&0xF800){
        case 0x4800:if(m_variant!=Variant::Namco163)return false;{uint8_t ra=m_portAddr&0x7F;m_ram[ra]=d;syncChipPhaseWrite(ra);if((m_portAddr&0x80)&&ra!=0x7F)++m_portAddr;return true;}
        case 0x5000:if(m_variant!=Variant::Namco163)return false;m_irqCounter=uint16_t((m_irqCounter&0x7F00)|d);m_irqPending=false;return true;
        case 0x5800:if(m_variant!=Variant::Namco163)return false;m_irqCounter=uint16_t((m_irqCounter&0x00FF)|((uint16_t(d&0x7F))<<8));m_irqEnabled=(d&0x80)!=0;m_irqPending=false;return true;
        case 0x8000:case 0x8800:case 0x9000:case 0x9800:m_chr[(a-0x8000)>>11]=d;return true;
        case 0xA000:case 0xA800:case 0xB000:case 0xB800:m_chr[4+((a-0xA000)>>11)]=d;return true;
        case 0xC000:if(m_variant==Variant::Namco175){m_ramProtect=d;return true;}if(m_variant==Variant::Namco163){m_nt[0]=d;return true;}return false;
        case 0xC800:case 0xD000:case 0xD800:if(m_variant==Variant::Namco163){m_nt[(a-0xC000)>>11]=d;return true;}return false;
        case 0xE000:m_prg[0]=d&0x3F;if(m_variant==Variant::Namco340){switch((d>>6)&3){case 0:m_mirror=Mirror::OnescreenLo;break;case 1:m_mirror=Mirror::Vertical;break;case 2:m_mirror=Mirror::OnescreenHi;break;case 3:m_mirror=Mirror::Horizontal;break;}}else if(m_variant==Variant::Namco163)m_audioDisabled=(d&0x40)!=0;return true;
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
    float expansionAudioSample(bool chipMod = false)const override{
        if(m_variant!=Variant::Namco163||m_audioDisabled)return 0.0f;
        const float mixGain=n163MixGain();
        if(mixGain==0.0f)return 0.0f;
        uint8_t count=channelCount();if(!count)return 0.0f;
        if(!chipMod)return std::max(-1.0f,std::min(1.0f,float(m_serialOut)/120.0f))*mixGain;
        float sum=0.0f;for(int ch=8-count;ch<8;ch++){uint8_t base=uint8_t(0x40+ch*8);uint16_t len=uint16_t(256-(m_ram[base+4]&0xFC));if(len<4)len=4;uint32_t rawPos=uint32_t(m_chipPhase[ch]/65536.0)+m_ram[base+6];uint8_t pos=uint8_t(std::min<uint32_t>(255,rawPos));uint8_t packed=m_ram[pos>>1];uint8_t sample=(pos&1)?(packed>>4):(packed&0x0F);uint8_t vol=m_ram[base+7]&0x0F;sum+=float(8-int(sample))*vol;}uint8_t divisor=std::min<uint8_t>(count,6);return std::max(-1.0f,std::min(1.0f,sum/(120.0f*divisor)))*mixGain;
    }
    std::size_t mapperBatterySize()const override{return m_variant==Variant::Namco163?sizeof(m_ram):0;}
    void saveMapperBattery(std::vector<uint8_t>&o)const override{if(m_variant==Variant::Namco163)o.insert(o.end(),m_ram,m_ram+sizeof(m_ram));}
    bool loadMapperBattery(const uint8_t*d,std::size_t n)override{if(m_variant!=Variant::Namco163)return n==0;if(n!=sizeof(m_ram))return false;std::memcpy(m_ram,d,n);rebuildChipPhases();return true;}
    void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_chr)put8(o,v);for(auto v:m_nt)put8(o,v);for(auto v:m_prg)put8(o,v);for(auto v:m_ram)put8(o,v);put8(o,m_portAddr);put8(o,m_ramProtect);mapper_hard_detail::put16(o,m_irqCounter);put8(o,m_irqEnabled);put8(o,m_irqPending);put8(o,m_audioDiv);put8(o,m_audioChannel);for(auto v:m_channelOut){int16_t q=int16_t(v);mapper_hard_detail::put16(o,uint16_t(q));}mapper_hard_detail::put16(o,uint16_t(m_serialOut));put8(o,m_chrRamDisable[0]);put8(o,m_chrRamDisable[1]);put8(o,m_audioDisabled);for(double v:m_chipPhase){uint64_t bits=0;static_assert(sizeof(bits)==sizeof(v),"unexpected double size");std::memcpy(&bits,&v,sizeof(bits));put64(o,bits);}}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_chr)if(!get8(p,e,v))return false;for(auto&v:m_nt)if(!get8(p,e,v))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_ram)if(!get8(p,e,v))return false;if(!get8(p,e,m_portAddr)||!get8(p,e,m_ramProtect)||!mapper_hard_detail::get16(p,e,m_irqCounter))return false;if(!get8(p,e,b))return false;m_irqEnabled=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,m_audioDiv)||!get8(p,e,m_audioChannel))return false;for(auto&v:m_channelOut){uint16_t q;if(!mapper_hard_detail::get16(p,e,q))return false;v=int16_t(q);}uint16_t serial;if(!mapper_hard_detail::get16(p,e,serial))return false;m_serialOut=int16_t(serial);if(!get8(p,e,b))return false;m_chrRamDisable[0]=b;if(!get8(p,e,b))return false;m_chrRamDisable[1]=b;if(!get8(p,e,b))return false;m_audioDisabled=b;for(double&v:m_chipPhase){uint64_t bits=0;if(!get64(p,e,bits))return false;std::memcpy(&v,&bits,sizeof(v));if(!std::isfinite(v))return false;}return true;}
private:
    Variant m_variant=Variant::Namco163;uint8_t m_chr[8]={},m_nt[4]={0xE0,0xE1,0xE0,0xE1},m_prg[3]={},m_ram[128]={},m_portAddr=0,m_ramProtect=0,m_audioDiv=0,m_audioChannel=7;bool m_chrRamDisable[2]={};uint16_t m_irqCounter=0;bool m_irqEnabled=false,m_irqPending=false,m_audioDisabled=false;int16_t m_channelOut[8]={},m_serialOut=0;double m_chipPhase[8]={};
    float n163MixGain() const {
        // NES 2.0 mapper 19 submappers describe the cartridge mixing
        // resistor. Submappers 1/2 have no N163 expansion sound. For 3-5,
        // keep the emulator's established submapper-3 absolute calibration
        // (0.55) and scale the other resistor variants by the center of the
        // documented dB ranges: 12.0, 16.5, and 18.75 dB above the 2A03.
        // This preserves the existing output domain while honoring the
        // hardware-relative cartridge mix levels.
        if (!m_config.nes20 || m_config.submapper == 0) return 0.55f;
        switch (m_config.submapper) {
        case 1: case 2: return 0.0f;
        case 3: return 0.55f;
        case 4: return 0.9234f; // 0.55 * 10^((16.5 - 12.0) / 20)
        case 5: return 1.1969f; // 0.55 * 10^((18.75 - 12.0) / 20)
        default: return 0.55f;
        }
    }
    uint8_t channelCount()const{return uint8_t(((m_ram[0x7F]>>4)&7)+1);}
    uint32_t ramPhase(int ch)const{uint8_t base=uint8_t(0x40+ch*8);return uint32_t(m_ram[base+1])|(uint32_t(m_ram[base+3])<<8)|(uint32_t(m_ram[base+5])<<16);}
    void rebuildChipPhases(){for(int ch=0;ch<8;ch++)m_chipPhase[ch]=double(ramPhase(ch));}
    void syncChipPhaseWrite(uint8_t ra){if(ra<0x40)return;uint8_t rel=uint8_t(ra-0x40),field=rel&7;if(field!=1&&field!=3&&field!=5)return;int ch=rel>>3;if(ch>=0&&ch<8)m_chipPhase[ch]=double(ramPhase(ch));}
    void clockChipAudio(){uint8_t count=channelCount();if(!count)return;for(int ch=8-count;ch<8;ch++){uint8_t base=uint8_t(0x40+ch*8);uint32_t freq=uint32_t(m_ram[base])|(uint32_t(m_ram[base+2])<<8)|(uint32_t(m_ram[base+4]&3)<<16);uint16_t len=uint16_t(256-(m_ram[base+4]&0xFC));if(len<4)len=4;double mod=double(uint32_t(len)<<16);m_chipPhase[ch]+=double(freq)/(15.0*double(count));if(m_chipPhase[ch]>=mod)m_chipPhase[ch]=std::fmod(m_chipPhase[ch],mod);}}
    void clockAudioChannel(){uint8_t count=channelCount();uint8_t first=uint8_t(8-count);if(m_audioChannel<first||m_audioChannel>7)m_audioChannel=7;uint8_t ch=m_audioChannel;uint8_t base=uint8_t(0x40+ch*8);uint32_t freq=uint32_t(m_ram[base])|(uint32_t(m_ram[base+2])<<8)|(uint32_t(m_ram[base+4]&3)<<16);uint32_t phase=uint32_t(m_ram[base+1])|(uint32_t(m_ram[base+3])<<8)|(uint32_t(m_ram[base+5])<<16);uint16_t len=uint16_t(256-(m_ram[base+4]&0xFC));if(len<4)len=4;uint32_t mod=uint32_t(len)<<16;phase=(phase+freq)%mod;m_ram[base+1]=uint8_t(phase);m_ram[base+3]=uint8_t(phase>>8);m_ram[base+5]=uint8_t(phase>>16);uint8_t pos=uint8_t(((phase>>16)+m_ram[base+6])&0xFF);uint8_t packed=m_ram[pos>>1];uint8_t sample=(pos&1)?(packed>>4):(packed&0x0F);uint8_t vol=m_ram[base+7]&0x0F;m_channelOut[ch]=int16_t(8-int(sample))*vol;m_serialOut=m_channelOut[ch];if(ch==first)m_audioChannel=7;else --m_audioChannel;}
};

// J.Y. Company ASIC (iNES 90/209/211). These mapper numbers are PCB wiring
// variants around one ASIC: mapper 90 suppresses ROM/extended nametables,
// while 209 and the legacy duplicate 211 expose them.
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
        // MMC4-like latches are independently controlled for each 4 KiB half.
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
        // useful power-on 8 KiB mode
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

// Mapper 176: 8025 / FK23C family. This implementation covers the two
// foundational layouts (NES 2.0 submappers 0 and 1): the six-bit MMC3
// variant and FK23C/FK23CA with extended MMC3 and CNROM/NROM CHR modes.
// Later Waixing/JX9003/HST variants (submappers 2-5) have additional RAM and
// high-address wiring and remain explicitly gated until implemented.

// NES 2.0 mapper 268: AA6023/SMD132/SMD133 (CoolBoy/MindKids).
// This implements the original and AA6023B wiring families (submappers 0-3):
// oversize MMC3, GNROM/NROM mode, six outer registers, lockout, WRAM overlay,
// and the submapper-dependent 5K/6K/7K register decode.
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
        if (mode & 0x10) { // GNROM / NROM-like mode
            const bool bank32 = gnrom32k();
            const unsigned a13 = (a >> 13) & 1;
            const unsigned a14 = bank32 ? ((a >> 14) & 1) : ((m_outer[3] >> 1) & 1);
            const unsigned a15 = (m_outer[3] >> 2) & 1;
            const unsigned a16 = (m_outer[3] >> 3) & 1;
            bank = std::size_t(a13) | (std::size_t(a14)<<1) | (std::size_t(a15)<<2) | (std::size_t(a16)<<3);
            // A17+ still pass through the AA6023 mask/offset network.
            bank = applyPrgOuter(bank | (inner & ~std::size_t(0x0F)));
        } else {
            // Weird MMC3 mode fixes the final 16 KiB to bank zero while in
            // non-inverted PRG mode. No known cartridge relies on the odd CHR
            // half behavior, but the documented PRG behavior is inexpensive.
            if ((mode & 0x40) && !m_prgMode && slot >= 2) inner = 0;
            bank = applyPrgOuter(inner);
        }
        m = mapBank(bank, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (isOuterAddress(a)) {
            writeOuter(a, d);
            // Registers overlay WRAM on the 6K/7K wiring. Returning false lets
            // Cartridge also route the write into WRAM when it is enabled.
            return outerBase() == 0x5000;
        }
        if (a < 0x8000) return false;
        switch (a & 0xE001) {
        case 0x8000: m_select=d; m_prgMode=(d&0x40)!=0; m_chrMode=(d&0x80)!=0; return true;
        case 0x8001: m_bank[m_select&7]=d; return true;
        case 0xA000:
            if(!m_config.fourScreen) {
                // SMD172C-L1 (submappers 10/11) can force one-screen
                // mirroring from outer register 0.  When S=0 the MMC3
                // mirroring register is physically disconnected.
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
        // $xxx4 selects mixed CHR-ROM/RAM operation. D0 enables the mode;
        // D1-D7 name the MMC3 bank-register value that is backed by RAM.
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
        // Register $xxx3 bit 5 optionally mirrors WRAM at $5000-$5FFF.
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
        if(r==2){ // mask lock applies only to S/T fields; low CHR offset stays writable
            if(m_outer[2]&0x80) d=uint8_t((m_outer[2]&0xF0)|(d&0x0F));
            m_outer[2]=d;return;
        }
        const bool gnrom=(m_outer[3]&0x10)!=0;
        if((m_outer[3]&0x80) && !gnrom) return; // lockout only works in MMC3 mode
        m_outer[r]=d;
        if(r==0 && isOneScreen268()) m_mirror=(d&0x10)?Mirror::OnescreenHi:Mirror::OnescreenLo;
    }
    bool gnrom32k() const {
        // The AA6023B revision flips both the position and sense of L.
        if(m_config.submapper>=2) return (m_outer[1]&0x10)==0;
        return (m_outer[1]&0x02)!=0;
    }
    std::size_t applyPrgOuter(std::size_t bank) const {
        const uint8_t r0=m_outer[0],r1=m_outer[1];
        auto setbit=[&](unsigned bit,bool v){const std::size_t mask=std::size_t(1)<<bit;bank=v?(bank|mask):(bank&~mask);};
        // bank bit 4 is CPU PRG A17.
        setbit(4,(r0&0x40)?((r0>>0)&1):((bank>>4)&1));

        if (m_config.submapper==4 || m_config.submapper==5) {
            // KP-6022/LD622D: at most 4 MiB PRG.  A20/A21 come from
            // the two C bits in $xxx0 instead of the original A23/A24.
            setbit(5,(r1&0x80)?((r0>>1)&1):((bank>>5)&1));
            setbit(6,(r1&0x40)?((bank>>6)&1):((r0>>2)&1));
            setbit(7,(r1&0x20)?((bank>>7)&1):false);
            setbit(8,(r0>>4)&1);
            return bank & 0x1FFu;
        }
        if (m_config.submapper==8 || m_config.submapper==9) {
            // SMD72A: max 2 MiB PRG, no high outer offset bits.
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
        if (m_config.submapper==10 || m_config.submapper==11) return bank & 0x3FFu; // max 8 MiB
        setbit(10,(r0>>4)&1); // A23
        setbit(11,(r0>>5)&1); // A24

        if (m_config.submapper==6 || m_config.submapper==7) {
            // J-852C has two equal-sized PRG chips.  A=1 takes chip select
            // directly from D; A=0 takes it from current MMC3 CHR A17.
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
            // J-852C exposes up to 128 KiB CHR-RAM.  Its CHR A17 output is
            // also optionally reused as PRG chip-select.
            inner &= 0x7Fu;
        } else {
            const bool a17=(o&0x80)?((o>>3)&1):((inner>>7)&1);
            inner=(inner&0x7F)|(std::size_t(a17)<<7);
            inner=(inner&0xFF)|(std::size_t((o>>4)&3)<<8); // CHR A18/A19
            if(m_config.submapper==4 || m_config.submapper==5)
                inner=(inner&0x3FFu)|(std::size_t(o&0x06)<<9); // A20/A21
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
        if (mode == 3) { // NROM-128, mirrored 16 KiB
            const std::size_t bank=(baseBank&~std::size_t(1))|((a>>13)&1);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        if (mode == 4) { // NROM-256
            const std::size_t bank=(baseBank&~std::size_t(3))|((a>>13)&3);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        if (mode == 5) { // UNROM: 16 KiB switch + fixed #7 within block
            const std::size_t inner16=(a<0xC000)?(m_unrom&7):7;
            const std::size_t bank=(baseBank&~std::size_t(0x0F))|(inner16<<1)|((a>>13)&1);
            m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF); return true;
        }
        return false;
    }

    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        // HST-162 supplies its upper PRG address bits from a separate $4800
        // register (decoded throughout $4800-$4FFF).
        if(m_config.submapper==5 && a>=0x4800 && a<=0x4FFF){m_prgMsb=d&0x3F;return true;}

        if(a>=0x5000 && a<0x6000){
            if(m_config.submapper==2 && !outerRegsEnabled()) return false; // this area is WRAM
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
            // FS005/006 swaps the ordinary MMC3 bank registers 6 and 7.
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
                // Second 4 KiB of 8 KiB WRAM bank 2, used by FS005 protection.
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
            low|=std::size_t((m_mode>>3)&1)<<8;   // PRG A21
            low|=std::size_t((m_mode>>7)&1)<<9;   // PRG A22
            low|=std::size_t((m_chrBase>>6)&1)<<10; // PRG A23
            low|=std::size_t((m_chrBase>>7)&1)<<11; // PRG A24
            low|=std::size_t((m_chrBase>>5)&1)<<12; // PRG A25
            break;
        case 3: low|=std::size_t(m_prgMsb&0x0F)<<8;break; // A24..A21
        case 4: low|=std::size_t((m_chrBase>>7)&1)<<8;break; // A21
        case 5:
            low=std::size_t(m_prgBase&0x1F)<<1; // A18..A14 only
            low|=std::size_t(m_prgMsb&0x3F)<<6; // A24..A19
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
            return base|(slot&7); // NROM CHR mode on all non-CNROM variants
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
            // The 2M registers overlap the latch and accept writes regardless
            // of which banking mode is currently selected.
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
