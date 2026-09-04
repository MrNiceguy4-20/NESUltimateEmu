#include "MapperFamilies.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

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

namespace mapper_hard_detail {
inline void put16(std::vector<uint8_t>& o, uint16_t v) { put8(o, uint8_t(v)); put8(o, uint8_t(v >> 8)); }
inline bool get16(const uint8_t*& p, const uint8_t* e, uint16_t& v) { uint8_t l,h; if(!get8(p,e,l)||!get8(p,e,h)) return false; v=uint16_t(l)|(uint16_t(h)<<8); return true; }
inline void put32(std::vector<uint8_t>& o, uint32_t v) { for(int i=0;i<4;i++) put8(o,uint8_t(v>>(i*8))); }
inline bool get32(const uint8_t*& p,const uint8_t* e,uint32_t& v){v=0;for(int i=0;i<4;i++){uint8_t b;if(!get8(p,e,b))return false;v|=uint32_t(b)<<(i*8);}return true;}

static constexpr uint8_t kLengthTable[32] = {
    10,254,20,2,40,4,80,6,160,8,60,10,14,12,26,14,
    12,16,24,18,48,20,96,22,192,24,72,26,0,28,32,30
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
        if(a==0x5010){ d=uint8_t((m_pcmIrqEnable&&m_pcmIrqTrip?0x80:0)|0x01); m_pcmIrqTrip=false; return true; }
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
        m=mapBank(bank,0x2000,m_config.prgRomSize,off);
        return true;
    }

    bool mapPrgRam(uint16_t a,uint32_t& m,bool write) const override {
        if(!m_config.prgRamSize) return false;
        uint8_t reg=0; uint32_t off=0, bankSize=0; bool forceRom=false;
        if(a>=0x6000&&a<0x8000){ reg=m_prgReg[0]; off=a-0x6000; }
        else if(a>=0x8000){ if(!decodePrg(a,reg,off,bankSize,forceRom)||forceRom||(reg&0x80))return false; }
        else return false;
        if(write && !(m_prot1==2 && m_prot2==1)) return false;
        const uint8_t bank=reg&0x0F;
        size_t physicalBank=0;
        const size_t ram=m_config.prgRamSize;
        if(ram<=0x2000){

            if(bank&0x04)return false;
            physicalBank=0;
        }else if(ram<=0x4000){

            physicalBank=(bank&0x04)?1:0;
        }else if(ram<=0x8000){

            if(bank&0x04)return false;
            physicalBank=bank&0x03;
        }else if(ram<=0x10000){

            physicalBank=bank&0x07;
        }else{

            physicalBank=bank&0x0F;
        }
        const size_t byteOffset=physicalBank*0x2000+(off&0x1FFF);

        m=static_cast<uint32_t>(byteOffset%ram);
        return true;
    }

    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a>=0x5000&&a<=0x5007){int ch=(a>=0x5004);uint8_t r=a&3;if(r==0)m_pulse[ch].writeControl(d);else if(r==2)m_pulse[ch].writeLow(d);else if(r==3)m_pulse[ch].writeHigh(d);return true;}
        if(a==0x5010){m_pcmReadMode=(d&1)!=0;m_pcmIrqEnable=(d&0x80)!=0;return true;}
        if(a==0x5011){if(!m_pcmReadMode){if(d==0)m_pcmIrqTrip=true;else{m_pcm=d;m_pcmIrqTrip=false;}}return true;}
        if(a==0x5015){for(int i=0;i<2;i++){m_pulse[i].enabled=(d&(1<<i))!=0;if(!m_pulse[i].enabled)m_pulse[i].length=0;}return true;}
        if(a==0x5100){m_prgMode=d&3;return true;} if(a==0x5101){m_chrMode=d&3;return true;}
        if(a==0x5102){m_prot1=d&3;return true;} if(a==0x5103){m_prot2=d&3;return true;}
        if(a==0x5104){m_exramMode=d&3;return true;} if(a==0x5105){m_ntMap=d;return true;}
        if(a==0x5106){m_fillTile=d;return true;} if(a==0x5107){m_fillPalette=d&3;return true;}
        if(a>=0x5113&&a<=0x5117){m_prgReg[a-0x5113]=d;if(a==0x5117)m_prgReg[4]|=0x80;return true;}
        if(a>=0x5120&&a<=0x512B){m_chrReg[a-0x5120]=d;m_chrLastSet=(a>=0x5128);return true;}
        if(a==0x5130){m_chrUpper=d&3;return true;}
        if(a==0x5200){m_splitCtrl=d;return true;} if(a==0x5201){m_splitScroll=d;return true;} if(a==0x5202){m_splitBank=d;return true;}
        if(a==0x5203){m_irqLine=d;return true;} if(a==0x5204){m_irqEnable=(d&0x80)!=0;return true;}
        if(a==0x5205){m_mulA=d;return true;} if(a==0x5206){m_mulB=d;return true;}
        if(a>=0x5C00&&a<=0x5FFF){if(m_exramMode<=2){m_exram[a&0x3FF]=(m_exramMode<=1&&!m_inFrame)?0:d;}return true;}
        return a>=0x5000&&a<0x6000;
    }

    void observeCpuRead(uint16_t a,uint8_t d) override {

        if(a==0xFFFA || a==0xFFFB) resetScanlineDetector(true);

        if(m_pcmReadMode&&a>=0x8000&&a<0xC000){if(d==0)m_pcmIrqTrip=true;else{m_pcm=d;m_pcmIrqTrip=false;}}
    }
    void observeCpuWrite(uint16_t a,uint8_t d) override {

        if((a&0xE007)==0x2000) m_sprite16=(d&0x20)!=0;

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
        const bool useBgSet=(kind==PpuFetchKind::Background)?m_sprite16:
                            (kind==PpuFetchKind::Cpu? (m_sprite16&&m_chrLastSet):false);
        uint16_t bank=chrBank(a,useBgSet);m=mapBank(bank,0x400,s,a&0x3FF);return true;
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

    void notifyPpuAddressContext(uint16_t a,uint64_t cyc,int line,int  ) override {
        notifyPpuAddress(a,cyc);
        clockScanlineDetector(a);
        m_specialAttrFetch=false;m_splitNtFetch=false;
        uint16_t aa=a&0x3FFF;bool nt=aa>=0x2000&&aa<=0x2FFF;uint16_t off=aa&0x3FF;

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

        m_currentScanline=line;m_specialAttrFetch=false;m_splitNtFetch=false;m_splitPatternReads=0;m_exAttrPatternReads=0;m_splitActive=false;
    }
    void clockCpu() override {

        if(m_ppuReadSeen){m_ppuReadSeen=false;m_ppuIdleCpu=0;}
        else if(m_ppuIdleCpu<3 && ++m_ppuIdleCpu>=3) resetScanlineDetector(true);
        m_pulse[0].timerClock();m_pulse[1].timerClock();

        if(++m_frameAudio>7458){m_frameAudio=uint16_t(m_frameAudio-7458);m_pulse[0].quarter();m_pulse[1].quarter();m_pulse[0].half();m_pulse[1].half();}
    }
    bool irqActive() const override { return (m_irqEnable&&m_irqPending)||(m_pcmIrqEnable&&m_pcmIrqTrip); }
    float expansionAudioSample(bool = false) const override { float p=(m_pulse[0].sample()+m_pulse[1].sample())*0.18f;float pcm=float(m_pcm)/255.0f*0.20f;return -(p+pcm); }

    void saveState(std::vector<uint8_t>&o)const override {put8(o,m_prgMode);put8(o,m_chrMode);put8(o,m_prot1);put8(o,m_prot2);put8(o,m_exramMode);put8(o,m_ntMap);put8(o,m_fillTile);put8(o,m_fillPalette);for(auto v:m_prgReg)put8(o,v);for(auto v:m_chrReg)put8(o,v);put8(o,m_chrUpper);put8(o,m_splitCtrl);put8(o,m_splitScroll);put8(o,m_splitBank);put8(o,m_irqLine);put8(o,m_irqEnable);put8(o,m_irqPending);put8(o,m_inFrame);mapper_hard_detail::put16(o,m_scanLastNt);put8(o,m_scanMatchCount);put8(o,m_scanlineCounter);put8(o,m_ppuIdleCpu);put8(o,m_scanSyncPending);put8(o,m_ppuReadSeen);put8(o,m_mulA);put8(o,m_mulB);put8(o,m_pcmReadMode);put8(o,m_pcmIrqEnable);put8(o,m_pcmIrqTrip);put8(o,m_pcm);put8(o,m_sprite16);put8(o,m_chrLastSet);mapper_hard_detail::put16(o,m_frameAudio);mapper_hard_detail::put16(o,uint16_t(m_currentScanline));mapper_hard_detail::put16(o,m_exAttrNtOffset);mapper_hard_detail::put16(o,m_exAttrChrBank);put8(o,m_exAttrPending);put8(o,m_exAttrPatternReads);put8(o,m_specialAttrFetch);put8(o,m_specialPalette);put8(o,m_splitActive);put8(o,m_splitNtFetch);put8(o,m_splitPatternReads);put8(o,m_splitColumn);put8(o,m_splitFineY);put8(o,m_splitTileNumber);mapper_hard_detail::put16(o,m_splitTileOffset);for(auto v:m_exram)put8(o,v);m_pulse[0].save(o);m_pulse[1].save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override {uint8_t b;if(!get8(p,e,m_prgMode)||!get8(p,e,m_chrMode)||!get8(p,e,m_prot1)||!get8(p,e,m_prot2)||!get8(p,e,m_exramMode)||!get8(p,e,m_ntMap)||!get8(p,e,m_fillTile)||!get8(p,e,m_fillPalette))return false;for(auto&v:m_prgReg)if(!get8(p,e,v))return false;for(auto&v:m_chrReg)if(!get8(p,e,v))return false;if(!get8(p,e,m_chrUpper)||!get8(p,e,m_splitCtrl)||!get8(p,e,m_splitScroll)||!get8(p,e,m_splitBank)||!get8(p,e,m_irqLine))return false;if(!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,b))return false;m_inFrame=b;if(!mapper_hard_detail::get16(p,e,m_scanLastNt)||!get8(p,e,m_scanMatchCount)||!get8(p,e,m_scanlineCounter)||!get8(p,e,m_ppuIdleCpu)||!get8(p,e,b))return false;m_scanSyncPending=b;if(!get8(p,e,b))return false;m_ppuReadSeen=b;if(!get8(p,e,m_mulA)||!get8(p,e,m_mulB)||!get8(p,e,b))return false;m_pcmReadMode=b;if(!get8(p,e,b))return false;m_pcmIrqEnable=b;if(!get8(p,e,b))return false;m_pcmIrqTrip=b;if(!get8(p,e,m_pcm)||!get8(p,e,b))return false;m_sprite16=b;if(!get8(p,e,b))return false;m_chrLastSet=b;if(!mapper_hard_detail::get16(p,e,m_frameAudio))return false;uint16_t q=0;if(!mapper_hard_detail::get16(p,e,q))return false;m_currentScanline=int16_t(q);if(!mapper_hard_detail::get16(p,e,m_exAttrNtOffset)||!mapper_hard_detail::get16(p,e,m_exAttrChrBank))return false;if(!get8(p,e,b))return false;m_exAttrPending=b;if(!get8(p,e,m_exAttrPatternReads)||!get8(p,e,b))return false;m_specialAttrFetch=b;if(!get8(p,e,m_specialPalette)||!get8(p,e,b))return false;m_splitActive=b;if(!get8(p,e,b))return false;m_splitNtFetch=b;if(!get8(p,e,m_splitPatternReads)||!get8(p,e,m_splitColumn)||!get8(p,e,m_splitFineY)||!get8(p,e,m_splitTileNumber)||!mapper_hard_detail::get16(p,e,m_splitTileOffset))return false;for(auto&v:m_exram)if(!get8(p,e,v))return false;return m_pulse[0].load(p,e)&&m_pulse[1].load(p,e);}
private:
    uint8_t m_prgMode=3,m_chrMode=3,m_prot1=0,m_prot2=0,m_exramMode=0,m_ntMap=0,m_fillTile=0,m_fillPalette=0;
    uint8_t m_prgReg[5]={0,0,0,0,0xFF},m_chrReg[12]={},m_chrUpper=0;
    uint8_t m_splitCtrl=0,m_splitScroll=0,m_splitBank=0,m_irqLine=0,m_mulA=0,m_mulB=0,m_pcm=0;
    bool m_irqEnable=false,m_irqPending=false,m_inFrame=false,m_pcmReadMode=false,m_pcmIrqEnable=false,m_pcmIrqTrip=false,m_sprite16=false,m_chrLastSet=false;uint16_t m_frameAudio=0;
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

}

std::unique_ptr<Mapper> createMmc5Mapper(const MapperConfig& config)
{
    if (config.id != 5) return nullptr;
    return std::make_unique<Mapper5>(config);
}
