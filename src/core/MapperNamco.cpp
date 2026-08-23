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
namespace mapper_hard_detail {
inline void put16(std::vector<uint8_t>& o,uint16_t v){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}
inline bool get16(const uint8_t*& p,const uint8_t* e,uint16_t& v){uint8_t l,h;if(!get8(p,e,l)||!get8(p,e,h))return false;v=uint16_t(l)|(uint16_t(h)<<8);return true;}
}

class Mapper19 final : public Mapper {
public:
    enum class Variant : uint8_t { Namco163, Namco175, Namco340 };
    explicit Mapper19(const MapperConfig&c):Mapper(c){
        if(c.id==210){

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
    bool mapPrgRam(uint16_t a,uint32_t&m,bool write)const override{if(a<0x6000||a>0x7FFF||!m_config.prgRamSize)return false;if(m_variant==Variant::Namco340)return false;if(m_variant==Variant::Namco175){if(write&&(m_ramProtect&1)==0)return false;m=(a-0x6000)%m_config.prgRamSize;return true;}uint8_t seg=(a-0x6000)>>11;if(write){if((m_ramProtect&0xF0)!=0x40)return false;if(m_ramProtect&(1u<<seg))return false;}m=(a-0x6000)%m_config.prgRamSize;return true;}
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
    std::size_t mapperBatterySize()const override{return m_variant==Variant::Namco163&&m_config.hasBattery?sizeof(m_ram):0;}
    void saveMapperBattery(std::vector<uint8_t>&o)const override{if(mapperBatterySize())o.insert(o.end(),m_ram,m_ram+sizeof(m_ram));}
    bool loadMapperBattery(const uint8_t*d,std::size_t n)override{if(!mapperBatterySize())return n==0;if(n!=sizeof(m_ram))return false;std::memcpy(m_ram,d,n);rebuildChipPhases();return true;}
    void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_chr)put8(o,v);for(auto v:m_nt)put8(o,v);for(auto v:m_prg)put8(o,v);for(auto v:m_ram)put8(o,v);put8(o,m_portAddr);put8(o,m_ramProtect);mapper_hard_detail::put16(o,m_irqCounter);put8(o,m_irqEnabled);put8(o,m_irqPending);put8(o,m_audioDiv);put8(o,m_audioChannel);for(auto v:m_channelOut){int16_t q=int16_t(v);mapper_hard_detail::put16(o,uint16_t(q));}mapper_hard_detail::put16(o,uint16_t(m_serialOut));put8(o,m_chrRamDisable[0]);put8(o,m_chrRamDisable[1]);put8(o,m_audioDisabled);for(double v:m_chipPhase){uint64_t bits=0;static_assert(sizeof(bits)==sizeof(v),"unexpected double size");std::memcpy(&bits,&v,sizeof(bits));put64(o,bits);}}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_chr)if(!get8(p,e,v))return false;for(auto&v:m_nt)if(!get8(p,e,v))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_ram)if(!get8(p,e,v))return false;if(!get8(p,e,m_portAddr)||!get8(p,e,m_ramProtect)||!mapper_hard_detail::get16(p,e,m_irqCounter))return false;if(!get8(p,e,b))return false;m_irqEnabled=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,m_audioDiv)||!get8(p,e,m_audioChannel))return false;for(auto&v:m_channelOut){uint16_t q;if(!mapper_hard_detail::get16(p,e,q))return false;v=int16_t(q);}uint16_t serial;if(!mapper_hard_detail::get16(p,e,serial))return false;m_serialOut=int16_t(serial);if(!get8(p,e,b))return false;m_chrRamDisable[0]=b;if(!get8(p,e,b))return false;m_chrRamDisable[1]=b;if(!get8(p,e,b))return false;m_audioDisabled=b;for(double&v:m_chipPhase){uint64_t bits=0;if(!get64(p,e,bits))return false;std::memcpy(&v,&bits,sizeof(v));if(!std::isfinite(v))return false;}return true;}
private:
    Variant m_variant=Variant::Namco163;uint8_t m_chr[8]={},m_nt[4]={0xE0,0xE1,0xE0,0xE1},m_prg[3]={},m_ram[128]={},m_portAddr=0,m_ramProtect=0,m_audioDiv=0,m_audioChannel=7;bool m_chrRamDisable[2]={};uint16_t m_irqCounter=0;bool m_irqEnabled=false,m_irqPending=false,m_audioDisabled=false;int16_t m_channelOut[8]={},m_serialOut=0;double m_chipPhase[8]={};
    float n163MixGain() const {

        if (!m_config.nes20 || m_config.submapper == 0) return 0.55f;
        switch (m_config.submapper) {
        case 1: case 2: return 0.0f;
        case 3: return 0.55f;
        case 4: return 0.9234f;
        case 5: return 1.1969f;
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

}

std::unique_ptr<Mapper> createNamcoMapper(const MapperConfig& config)
{
    if (config.id != 19 && config.id != 210) return nullptr;
    return std::make_unique<Mapper19>(config);
}
