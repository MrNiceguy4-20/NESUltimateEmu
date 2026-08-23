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
struct VrcIrq {
    uint8_t latch=0,counter=0; int16_t prescaler=341; bool enabled=false,ackEnable=false,cycleMode=false,pending=false;
    void writeControl(uint8_t d){ackEnable=(d&1)!=0;enabled=(d&2)!=0;cycleMode=(d&4)!=0;pending=false;if(enabled){counter=latch;prescaler=341;}}
    void acknowledge(){pending=false;enabled=ackEnable;}
    void clockCounter(){if(counter==0xFF){counter=latch;pending=true;}else ++counter;}
    void clockCpu(){if(!enabled)return;if(cycleMode)clockCounter();else{prescaler-=3;if(prescaler<=0){prescaler+=341;clockCounter();}}}
    void save(std::vector<uint8_t>&o)const{put8(o,latch);put8(o,counter);put8(o,uint8_t(prescaler));put8(o,uint8_t(prescaler>>8));put8(o,enabled);put8(o,ackEnable);put8(o,cycleMode);put8(o,pending);}
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t lo,hi,b;if(!get8(p,e,latch)||!get8(p,e,counter)||!get8(p,e,lo)||!get8(p,e,hi))return false;prescaler=int16_t(uint16_t(lo)|(uint16_t(hi)<<8));if(!get8(p,e,b))return false;enabled=b; if(!get8(p,e,b))return false;ackEnable=b;if(!get8(p,e,b))return false;cycleMode=b;if(!get8(p,e,b))return false;pending=b;return true;}
};

struct Vrc7Opll {
    enum class EnvState : uint8_t { Off, Attack, Decay, Sustain, Release };
    struct Channel {
        uint32_t modPhase=0, carPhase=0;
        uint16_t env=1023;
        int16_t lastMod=0;
        EnvState state=EnvState::Off;
        bool key=false;
    } ch[6];
    uint8_t regs[0x40]{};
    uint8_t divider=0;
    int16_t output=0;

    static const uint8_t* preset(uint8_t inst) {
        static const uint8_t table[15][8] = {
            {0x03,0x21,0x05,0x06,0xE8,0x81,0x42,0x27},
            {0x13,0x41,0x14,0x0D,0xD8,0xF6,0x23,0x12},
            {0x11,0x11,0x08,0x08,0xFA,0xB2,0x20,0x12},
            {0x31,0x61,0x0C,0x07,0xA8,0x64,0x61,0x27},
            {0x32,0x21,0x1E,0x06,0xE1,0x76,0x01,0x28},
            {0x02,0x01,0x06,0x00,0xA3,0xE2,0xF4,0xF4},
            {0x21,0x61,0x1D,0x07,0x82,0x81,0x11,0x07},
            {0x23,0x21,0x22,0x17,0xA2,0x72,0x01,0x17},
            {0x35,0x11,0x25,0x00,0x40,0x73,0x72,0x01},
            {0xB5,0x01,0x0F,0x0F,0xA8,0xA5,0x51,0x02},
            {0x17,0xC1,0x24,0x07,0xF8,0xF8,0x22,0x12},
            {0x71,0x23,0x11,0x06,0x65,0x74,0x18,0x16},
            {0x01,0x02,0xD3,0x05,0xC9,0x95,0x03,0x02},
            {0x61,0x63,0x0C,0x00,0x94,0xC0,0x33,0xF6},
            {0x21,0x72,0x0D,0x00,0xC1,0xD5,0x56,0x06}
        };
        return table[(inst - 1) % 15];
    }
    static uint8_t mul(uint8_t v) {
        static const uint8_t m[16]={1,2,4,6,8,10,12,14,16,18,20,20,24,24,30,30};
        return m[v&15];
    }
    const uint8_t* patch(uint8_t inst, uint8_t custom[8]) const {
        if(inst){return preset(inst);}
        for(int i=0;i<8;i++)custom[i]=regs[i];
        return custom;
    }
    static uint16_t rateStep(uint8_t rate, uint8_t block, bool attack) {
        if(!rate)return 0;
        unsigned r=std::min<unsigned>(15, unsigned(rate)+(block>>1));

        unsigned base=1u << (r/3);
        if(attack) base*=3;
        return uint16_t(std::min<unsigned>(512,base));
    }
    static double sine(uint32_t phase){
        constexpr double tau=6.283185307179586476925286766559;
        return std::sin(tau*double(phase&0x7FFFFu)/524288.0);
    }
    void resetAudio(){
        std::fill(std::begin(regs), std::end(regs), uint8_t(0));
        for(auto& c : ch) c = Channel{};
        divider = 0;
        output = 0;
        m_accum = 0.0;

    }
    void write(uint8_t r,uint8_t d){
        r&=0x3F; regs[r]=d;
        if(r>=0x20&&r<=0x25){
            int i=r-0x20; bool now=(d&0x10)!=0;
            if(now&&!ch[i].key){ch[i].state=EnvState::Attack;ch[i].env=1023;ch[i].modPhase=ch[i].carPhase=0;ch[i].lastMod=0;}
            else if(!now&&ch[i].key&&ch[i].state!=EnvState::Off) ch[i].state=EnvState::Release;
            ch[i].key=now;
        }
    }
    void tickChannel(int i){
        uint16_t fnum=uint16_t(regs[0x10+i])|uint16_t(regs[0x20+i]&1)<<8;
        uint8_t block=(regs[0x20+i]>>1)&7, inst=regs[0x30+i]>>4, volume=regs[0x30+i]&15;
        uint8_t custom[8]; const uint8_t* p=patch(inst,custom);
        uint8_t arC=p[5]>>4,drC=p[5]&15,slC=p[7]>>4,rrC=p[7]&15;
        Channel& c=ch[i];
        switch(c.state){
        case EnvState::Attack:{uint16_t step=rateStep(arC,block,true);if(!step)break;if(c.env<=step)c.env=0;else c.env=uint16_t(c.env-step);if(c.env==0)c.state=EnvState::Decay;break;}
        case EnvState::Decay:{uint16_t target=uint16_t(slC)*64;uint16_t step=rateStep(drC,block,false);c.env=uint16_t(std::min<unsigned>(1023,c.env+step));if(c.env>=target)c.state=EnvState::Sustain;break;}
        case EnvState::Sustain:if(!c.key)c.state=EnvState::Release;else if(!(p[1]&0x20)){uint16_t st=rateStep(rrC?rrC:5,block,false);c.env=uint16_t(std::min<unsigned>(1023,c.env+st));}break;
        case EnvState::Release:{uint8_t rate=(regs[0x20+i]&0x20)?5:(rrC?rrC:7);uint16_t st=rateStep(rate,block,false);c.env=uint16_t(std::min<unsigned>(1023,c.env+st));if(c.env>=1023)c.state=EnvState::Off;break;}
        default:break;
        }
        if(c.state==EnvState::Off||fnum==0)return;

        uint32_t base=((uint32_t(fnum)*2u)<<block)>>2;
        uint32_t modInc=(base*mul(p[0]))&0x7FFFFu;
        uint32_t carInc=(base*mul(p[1]))&0x7FFFFu;
        c.modPhase=(c.modPhase+modInc)&0x7FFFFu;
        int fb=p[3]&7;
        double mod=sine(c.modPhase + uint32_t(int32_t(c.lastMod)*(fb?fb:0)*32));
        double modLevel=1.0-double(p[2]&0x3F)/63.0;
        int16_t modNow=int16_t(mod*2047.0*modLevel);
        c.lastMod=int16_t((int(c.lastMod)+int(modNow))/2);
        c.carPhase=(c.carPhase+carInc)&0x7FFFFu;
        double envGain=(1023.0-double(c.env))/1023.0;
        double volGain=(15.0-double(volume))/15.0;
        double car=sine(c.carPhase + uint32_t(int32_t(modNow)*24));
        m_accum += car*envGain*volGain;
    }
    void clockCpu(){

        m_clockAccum += 49716u;
        if(m_clockAccum < 1789773u)return;
        m_clockAccum -= 1789773u;
        m_accum=0.0;
        for(int i=0;i<6;i++)tickChannel(i);
        output=int16_t(std::max(-32767.0,std::min(32767.0,m_accum*(32767.0/6.0))));
    }
    float sample(bool chipMod=false)const{double v=chipMod?(m_accum/6.0):(double(output)/32768.0);return float(std::max(-1.0,std::min(1.0,v))*0.40); }
    void save(std::vector<uint8_t>& o) const {
        for (auto v : regs)
            put8(o, v);
        put32Local(o, m_clockAccum);
        put8(o, uint8_t(output));
        put8(o, uint8_t(uint16_t(output) >> 8));

        uint64_t accumBits = 0;
        static_assert(sizeof(accumBits) == sizeof(m_accum), "unexpected double size");
        std::memcpy(&accumBits, &m_accum, sizeof(accumBits));
        put64(o, accumBits);

        for (const auto& c : ch) {
            put32Local(o, c.modPhase);
            put32Local(o, c.carPhase);
            put8(o, uint8_t(c.env));
            put8(o, uint8_t(c.env >> 8));
            put8(o, uint8_t(c.lastMod));
            put8(o, uint8_t(uint16_t(c.lastMod) >> 8));
            put8(o, uint8_t(c.state));
            put8(o, c.key);
        }
    }
    bool load(const uint8_t*& p, const uint8_t* e) {
        for (auto& v : regs) {
            if (!get8(p, e, v))
                return false;
        }
        if (!get32Local(p, e, m_clockAccum))
            return false;

        uint8_t lo, hi;
        if (!get8(p, e, lo) || !get8(p, e, hi))
            return false;
        output = int16_t(uint16_t(lo) | (uint16_t(hi) << 8));

        uint64_t accumBits = 0;
        if (!get64(p, e, accumBits))
            return false;
        std::memcpy(&m_accum, &accumBits, sizeof(m_accum));
        if (!std::isfinite(m_accum))
            return false;

        for (auto& c : ch) {
            if (!get32Local(p, e, c.modPhase) || !get32Local(p, e, c.carPhase) ||
                !get8(p, e, lo) || !get8(p, e, hi))
                return false;
            c.env = uint16_t(lo) | (uint16_t(hi) << 8);
            if (!get8(p, e, lo) || !get8(p, e, hi))
                return false;
            c.lastMod = int16_t(uint16_t(lo) | (uint16_t(hi) << 8));
            if (!get8(p, e, lo))
                return false;
            c.state = EnvState(lo);
            if (!get8(p, e, lo))
                return false;
            c.key = lo != 0;
        }
        return true;
    }
private:
    uint32_t m_clockAccum=0;
    double m_accum=0.0;
    static void put32Local(std::vector<uint8_t>&o,uint32_t v){for(int i=0;i<4;i++)put8(o,uint8_t(v>>(i*8)));}
    static bool get32Local(const uint8_t*&p,const uint8_t*e,uint32_t&v){v=0;for(int i=0;i<4;i++){uint8_t b;if(!get8(p,e,b))return false;v|=uint32_t(b)<<(8*i);}return true;}
};
}

class MapperVrc24 final : public Mapper {
public:
    explicit MapperVrc24(const MapperConfig& c):Mapper(c){m_isVrc2=c.id==22||(c.id==23&&c.submapper!=2)||(c.id==25&&c.submapper==3);}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000),last=n-1,last2=n>1?n-2:0;size_t b;if(!m_prgMode)b=a<0xA000?m_prg0:a<0xC000?m_prg1:a<0xE000?last2:last;else b=a<0xA000?last2:a<0xC000?m_prg1:a<0xE000?m_prg0:last;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000)return false;if(a<0x8000){m_latch=d&1;return true;}uint16_t r=translate(a)&0xF003;if((r&0xF000)==0x8000){m_prg0=d&0x1F;return true;}if((r&0xF000)==0x9000){if((r&3)<=1)mapper_more_detail::setMirror4(m_mirror,d);else if(!m_isVrc2)m_prgMode=(d>>1)&1;return true;}if((r&0xF000)==0xA000){m_prg1=d&0x1F;return true;}if(r>=0xB000&&r<=0xE003){int i=(((r>>12)-0xB)*2)+((r>>1)&1);if(r&1)m_hi[i]=d&0x1F;else m_lo[i]=d&0x0F;return true;}if(!m_isVrc2&&r==0xF000){m_irq.latch=uint8_t((m_irq.latch&0xF0)|(d&0xF));return true;}if(!m_isVrc2&&r==0xF001){m_irq.latch=uint8_t((m_irq.latch&0x0F)|((d&0xF)<<4));return true;}if(!m_isVrc2&&r==0xF002){m_irq.writeControl(d);return true;}if(!m_isVrc2&&r==0xF003){m_irq.acknowledge();return true;}return false;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;int i=a>>10;uint16_t b=uint16_t(m_lo[i]|(m_hi[i]<<4));if(m_config.id==22)b>>=1;m=mapBank(b,0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void clockCpu()override{if(!m_isVrc2)m_irq.clockCpu();}bool irqActive()const override{return !m_isVrc2&&m_irq.pending;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg0);put8(o,m_prg1);put8(o,m_prgMode);put8(o,m_latch);for(auto v:m_lo)put8(o,v);for(auto v:m_hi)put8(o,v);m_irq.save(o);put8(o,static_cast<uint8_t>(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg0)||!get8(p,e,m_prg1)||!get8(p,e,m_prgMode)||!get8(p,e,m_latch))return false;for(auto&v:m_lo)if(!get8(p,e,v))return false;for(auto&v:m_hi)if(!get8(p,e,v))return false;if(!m_irq.load(p,e)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}
private:bool m_isVrc2=false;uint8_t m_prg0=0,m_prg1=0,m_prgMode=0,m_latch=0,m_lo[8]={},m_hi[8]={};mapper_more_detail::VrcIrq m_irq;
    uint16_t translate(uint16_t a)const{uint8_t a0=0,a1=0;bool h=m_config.submapper==0&&m_config.id!=22&&m_config.id!=27;if(h){if(m_config.id==25){a0=((a>>1)&1)|((a>>3)&1);a1=(a&1)|((a>>2)&1);}else if(m_config.id==21){a0=((a>>1)&1)|((a>>6)&1);a1=((a>>2)&1)|((a>>7)&1);}else{a0=(a&1)|((a>>2)&1);a1=((a>>1)&1)|((a>>3)&1);}}else if(m_config.id==22){a0=(a>>1)&1;a1=a&1;}else if(m_config.id==21&&m_config.submapper==2){a0=(a>>6)&1;a1=(a>>7)&1;}else if(m_config.id==21){a0=(a>>1)&1;a1=(a>>2)&1;}else if(m_config.id==23&&m_config.submapper==2){a0=(a>>2)&1;a1=(a>>3)&1;}else if(m_config.id==23){a0=a&1;a1=(a>>1)&1;}else if(m_config.id==25&&m_config.submapper==2){a0=(a>>3)&1;a1=(a>>2)&1;}else{a0=(a>>1)&1;a1=a&1;}return uint16_t((a&0xFF00)|(a1<<1)|a0);}
};

class MapperVrc6 final : public Mapper {
public:
    explicit MapperVrc6(const MapperConfig&c):Mapper(c),m_swap(c.id==26){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;if(a<0xC000)m=mapBank(m_prg16,0x4000,m_config.prgRomSize,a-0x8000);else if(a<0xE000)m=mapBank(m_prg8,0x2000,m_config.prgRomSize,a-0xC000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x2000+(a-0xE000));return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if (a < 0x8000) return false;
        uint16_t r = translate(a);
        if((r&0xF003)==0x8000){m_prg16=d&0x0F;return true;}
        if(r>=0x9000&&r<=0x9002){m_pulse[0].write(r&3,d,audioShift());return true;}
        if(r==0x9003){
            m_audioCtrl=d;
            const uint8_t shift=audioShift();
            m_pulse[0].clampCounter(shift);
            m_pulse[1].clampCounter(shift);
            m_saw.clampCounter(shift);
            return true;
        }
        if(r>=0xA000&&r<=0xA002){m_pulse[1].write(r&3,d,audioShift());return true;}
        if(r>=0xB000&&r<=0xB002){m_saw.write(r&3,d,audioShift());return true;}
        if((r&0xF003)==0xB003){
            m_ppuCtrl=d;
            mapper_more_detail::setMirror4(m_mirror,(d>>2)&3);
            return true;
        }
        if((r&0xF003)==0xC000){m_prg8=d&0x1F;return true;}
        if(r>=0xD000&&r<=0xE003){int i=((r>>12)-0xD)*4+(r&3);if(i<8)m_chr[i]=d;return true;}
        if((r&0xF003)==0xF000){m_irq.latch=d;return true;}if((r&0xF003)==0xF001){m_irq.writeControl(d);return true;}if((r&0xF003)==0xF002){m_irq.acknowledge();return true;}return false;
    }
    bool ppuMapRead(uint16_t a,uint32_t&m)override{
        if(a>=0x2000)return false;
        const auto size=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!size)return false;
        const uint8_t slot=uint8_t(a>>10);
        const uint8_t mode=m_ppuCtrl&3;
        uint8_t reg=slot;
        bool paired=false;
        if(mode==1){reg=uint8_t(slot>>1);paired=true;}
        else if(mode>=2&&slot>=4){reg=uint8_t(4+((slot-4)>>1));paired=true;}
        uint8_t bank=m_chr[reg];
        if(paired&&(m_ppuCtrl&0x20))bank=uint8_t((bank&0xFE)|(slot&1));
        m=mapBank(bank,0x400,size,a&0x3FF);
        return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapPrgRam(uint16_t a,uint32_t&m,bool)const override{
        if(a<0x6000||a>0x7FFF||!m_config.prgRamSize||(m_ppuCtrl&0x80)==0)return false;
        m=uint32_t(a-0x6000)%m_config.prgRamSize;
        return true;
    }
    bool mapNametable(uint16_t a,NametableSource& source,uint32_t& mapped)const override{
        if(a<0x2000||a>0x3EFF)return false;
        const uint8_t page=uint8_t((a>>10)&3);
        const uint16_t off=a&0x3FF;

        const uint8_t style=m_ppuCtrl&0x07;
        uint8_t reg=6;
        switch(style){
        case 0: case 6: case 7:
            reg=uint8_t(page<2?6:7);
            break;
        case 1: case 5:
            reg=uint8_t(4+page);
            break;
        default:
            reg=uint8_t(6+(page&1));
            break;
        }

        uint8_t bank=m_chr[reg];

        if(m_ppuCtrl&0x20){
            int forced=-1;
            switch(m_ppuCtrl&0x0F){
            case 0x0: { static constexpr uint8_t v[4]={0,1,0,1}; forced=v[page]; break; }
            case 0x4: { static constexpr uint8_t v[4]={0,0,1,1}; forced=v[page]; break; }
            case 0x8: forced=0; break;
            case 0xC: forced=1; break;
            case 0x3: { static constexpr uint8_t v[4]={0,0,1,1}; forced=v[page]; break; }
            case 0x7: { static constexpr uint8_t v[4]={0,1,0,1}; forced=v[page]; break; }
            case 0xB: forced=1; break;
            case 0xF: forced=0; break;
            default: break;
            }
            if(forced>=0)bank=uint8_t((bank&0xFE)|uint8_t(forced));
        }

        if(m_ppuCtrl&0x10){

            const auto size=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
            if(!size)return false;
            source=m_config.chrRomSize?NametableSource::ChrRom:NametableSource::ChrRam;
            mapped=mapBank(bank,0x400,size,off);
        }else{
            source=NametableSource::Ciram;
            mapped=uint32_t(bank&1)*0x400+off;
        }
        return true;
    }
    bool mapNametableWrite(uint16_t a,NametableSource& source,uint32_t& mapped)const override{return mapNametable(a,source,mapped);}
    void clockCpu() override {
        m_irq.clockCpu();
        if ((m_audioCtrl & 1) == 0) {
            const uint8_t shift = audioShift();
            m_pulse[0].clock(shift);
            m_pulse[1].clock(shift);
            m_saw.clock(shift);
        }
    }
    bool irqActive()const override{return m_irq.pending;}
    float expansionAudioSample(bool chipMod = false) const override {
        const uint8_t shift = audioShift();
        const float saw = (chipMod ? m_saw.sampleSmooth(shift) : float(m_saw.sample())) * 0.88f;
        const float raw = float(m_pulse[0].sample() + m_pulse[1].sample()) + saw;

        return -(raw / 61.0f) * 0.55f;
    }
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg16);put8(o,m_prg8);for(auto v:m_chr)put8(o,v);m_irq.save(o);put8(o,static_cast<uint8_t>(m_mirror));put8(o,m_ppuCtrl);put8(o,m_audioCtrl);for(auto&i:m_pulse)i.save(o);m_saw.save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg16)||!get8(p,e,m_prg8))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!m_irq.load(p,e)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);if(!get8(p,e,m_ppuCtrl)||!get8(p,e,m_audioCtrl))return false;for(auto&i:m_pulse)if(!i.load(p,e))return false;return m_saw.load(p,e);}
private:
    struct Pulse {
        uint8_t ctrl = 0, hi = 0, step = 0;
        uint16_t freq = 0, timer = 0;

        uint16_t effectivePeriod(uint8_t shift) const { return uint16_t(freq >> shift); }
        bool enabled() const { return (hi & 0x80) != 0; }

        void clampCounter(uint8_t shift) {
            const uint16_t period = effectivePeriod(shift);
            if (timer > period) timer = period;
        }
        void write(uint8_t r, uint8_t d, uint8_t shift = 0) {
            if (r == 0) {
                ctrl = d;
                return;
            }
            if (r == 1) {
                freq = uint16_t((freq & 0xF00) | d);
                clampCounter(shift);
                return;
            }
            if (r == 2) {
                const bool wasEnabled = enabled();
                freq = uint16_t((freq & 0x0FF) | ((uint16_t(d & 0x0F)) << 8));
                hi = d;
                clampCounter(shift);
                if (!wasEnabled && enabled()) step = 0;
            }
        }
        void clock(uint8_t shift) {
            if (!enabled()) return;
            const uint16_t period = effectivePeriod(shift);
            ++timer;
            if (timer > period) {
                timer = uint16_t(timer - (period + 1u));
                step = uint8_t((step + 1) & 15);
            }
        }
        uint8_t sample() const {
            static constexpr uint8_t dutyTable[8][16] = {
                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1},
                {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1},
                {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1},
                {0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1},
                {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1},
                {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
                {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1}
            };
            if (!enabled()) return 0;
            const uint8_t duty = (ctrl >> 4) & 7;
            return ((ctrl & 0x80) || dutyTable[duty][step]) ? (ctrl & 0x0F) : 0;
        }
        void save(std::vector<uint8_t>&o)const{put8(o,ctrl);put8(o,hi);put8(o,step);put8(o,uint8_t(freq));put8(o,uint8_t(freq>>8));put8(o,uint8_t(timer));put8(o,uint8_t(timer>>8));}
        bool load(const uint8_t*&p,const uint8_t*e){uint8_t fl,fh,tl,th;return get8(p,e,ctrl)&&get8(p,e,hi)&&get8(p,e,step)&&get8(p,e,fl)&&get8(p,e,fh)&&get8(p,e,tl)&&get8(p,e,th)&&(freq=uint16_t(fl)|(uint16_t(fh)<<8),timer=uint16_t(tl)|(uint16_t(th)<<8),true);}
    };

    struct Saw {
        uint8_t rate = 0, hi = 0, step = 0, acc = 0;
        uint16_t freq = 0, timer = 0;

        uint16_t effectivePeriod(uint8_t shift) const { return uint16_t(freq >> shift); }
        bool enabled() const { return (hi & 0x80) != 0; }
        void clampCounter(uint8_t shift) {
            const uint16_t period = effectivePeriod(shift);
            if (timer > period) timer = period;
        }
        void write(uint8_t r, uint8_t d, uint8_t shift = 0) {
            if (r == 0) {
                rate = d & 0x3F;
                return;
            }
            if (r == 1) {
                freq = uint16_t((freq & 0xF00) | d);
                clampCounter(shift);
                return;
            }
            if (r == 2) {
                const bool wasEnabled = enabled();
                freq = uint16_t((freq & 0x0FF) | ((uint16_t(d & 0x0F)) << 8));
                hi = d;
                clampCounter(shift);
                if (!wasEnabled && enabled()) {
                    step = 0;
                    acc = 0;
                }
            }
        }
        void clock(uint8_t shift) {
            if (!enabled()) return;
            const uint16_t period = effectivePeriod(shift);
            ++timer;
            if (timer > period) {
                timer = uint16_t(timer - (period + 1u));
                ++step;
                if (step >= 14) {
                    step = 0;
                    acc = 0;
                }
                else if ((step & 1) == 0) {
                    acc = uint8_t(acc + rate);
                }
            }
        }
        uint8_t sample() const { return enabled() ? uint8_t(acc >> 3) : 0; }
        float sampleSmooth(uint8_t shift) const {
            if (!enabled()) return 0.0f;
            const float period = float(effectivePeriod(shift)) + 1.0f;
            const float sub = period > 0.0f ? std::clamp(float(timer) / period, 0.0f, 1.0f) : 0.0f;
            const float phase = std::clamp((float(step) + sub) / 14.0f, 0.0f, 1.0f);
            return float(rate) * phase;
        }
        void save(std::vector<uint8_t>&o)const{put8(o,rate);put8(o,hi);put8(o,step);put8(o,acc);put8(o,uint8_t(freq));put8(o,uint8_t(freq>>8));put8(o,uint8_t(timer));put8(o,uint8_t(timer>>8));}
        bool load(const uint8_t*&p,const uint8_t*e){uint8_t fl,fh,tl,th;return get8(p,e,rate)&&get8(p,e,hi)&&get8(p,e,step)&&get8(p,e,acc)&&get8(p,e,fl)&&get8(p,e,fh)&&get8(p,e,tl)&&get8(p,e,th)&&(freq=uint16_t(fl)|(uint16_t(fh)<<8),timer=uint16_t(tl)|(uint16_t(th)<<8),true);}
    };

    bool m_swap;uint8_t m_prg16=0,m_prg8=0,m_chr[8]={},m_ppuCtrl=0,m_audioCtrl=0;Pulse m_pulse[2];Saw m_saw;mapper_more_detail::VrcIrq m_irq;
    uint8_t audioShift() const { return (m_audioCtrl & 4) ? 8 : ((m_audioCtrl & 2) ? 4 : 0); }
    uint16_t translate(uint16_t a)const{if(!m_swap)return a;uint16_t lo=a&3;lo=((lo&1)<<1)|((lo&2)>>1);return uint16_t((a&~3)|lo);}
};

class Mapper73 final : public Mapper {public:explicit Mapper73(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF000){case 0x8000:m_latch=(m_latch&0xFFF0)|(d&0xF);return true;case 0x9000:m_latch=(m_latch&0xFF0F)|((d&0xF)<<4);return true;case 0xA000:m_latch=(m_latch&0xF0FF)|((d&0xF)<<8);return true;case 0xB000:m_latch=(m_latch&0x0FFF)|((d&0xF)<<12);return true;case 0xC000:m_ack=d&1;m_enabled=d&2;m_mode8=d&4;m_pending=false;if(m_enabled)m_counter=m_latch;return true;case 0xD000:m_pending=false;m_enabled=m_ack;return true;case 0xF000:m_prg=d&7;return true;}return false;}void clockCpu()override{if(!m_enabled)return;if(m_mode8){uint8_t v=uint8_t(m_counter);if(v==0xFF){m_counter=(m_counter&0xFF00)|(m_latch&0xFF);m_pending=true;}else m_counter=(m_counter&0xFF00)|uint8_t(v+1);}else{if(m_counter==0xFFFF){m_counter=m_latch;m_pending=true;}else ++m_counter;}}bool irqActive()const override{return m_pending;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(uint16_t v:{m_latch,m_counter}){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}put8(o,m_enabled);put8(o,m_ack);put8(o,m_mode8);put8(o,m_pending);}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t l,h,b;if(!get8(p,e,m_prg)||!get8(p,e,l)||!get8(p,e,h))return false;m_latch=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,l)||!get8(p,e,h))return false;m_counter=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_ack=b;if(!get8(p,e,b))return false;m_mode8=b;if(!get8(p,e,b))return false;m_pending=b;return true;}private:uint8_t m_prg=0;uint16_t m_latch=0,m_counter=0;bool m_enabled=false,m_ack=false,m_mode8=false,m_pending=false;};

class Mapper75 final : public Mapper {public:explicit Mapper75(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF000){case 0x8000:m_prg[0]=d&0x0F;return true;case 0x9000:m_chr[0]=uint8_t((m_chr[0]&0x0F)|((d&2)<<3));m_chr[1]=uint8_t((m_chr[1]&0x0F)|((d&4)<<2));if(!m_config.fourScreen)m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;return true;case 0xA000:m_prg[1]=d&0x0F;return true;case 0xC000:m_prg[2]=d&0x0F;return true;case 0xE000:m_chr[0]=uint8_t((m_chr[0]&0x10)|(d&0x0F));return true;case 0xF000:m_chr[1]=uint8_t((m_chr[1]&0x10)|(d&0x0F));return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>12],0x1000,s,a&0xFFF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg[3]={},m_chr[2]={};};

class Mapper85 final : public Mapper {
public:
    explicit Mapper85(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        const size_t banks = std::max<size_t>(1, m_config.prgRomSize / 0x2000);
        size_t bank = banks - 1;
        if (addr < 0xA000) bank = m_prg[0];
        else if (addr < 0xC000) bank = m_prg[1];
        else if (addr < 0xE000) bank = m_prg[2];
        mapped = mapBank(bank, 0x2000, m_config.prgRomSize, addr & 0x1FFF);
        return true;
    }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool) const override {
        if (addr < 0x6000 || addr > 0x7FFF || !m_config.prgRamSize || !(m_control & 0x80)) return false;
        mapped = (addr - 0x6000) % m_config.prgRamSize;
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {

        const uint16_t audioDecode = addr & 0xF030;
        const bool audioPort = audioDecode == 0x9010 || audioDecode == 0x9030;
        if (!audioPort && m_config.nes20 && m_config.submapper == 1) {
            addr &= static_cast<uint16_t>(~0x10);
        } else if (!audioPort && m_config.nes20 && m_config.submapper == 2) {
            const bool an = (addr & 0x10) != 0;
            addr &= static_cast<uint16_t>(~0x18);
            if (an) addr |= 0x08;
        } else if (!audioPort && (addr & 0x10)) {

            addr |= 0x08;
            addr &= static_cast<uint16_t>(~0x10);
        }

        switch (addr & 0xF038) {
        case 0x8000: m_prg[0] = data & 0x3F; return true;
        case 0x8008: m_prg[1] = data & 0x3F; return true;
        case 0x9000: m_prg[2] = data & 0x3F; return true;

        case 0x9010:
            if (!(m_control & 0x40)) m_opllAddress = data & 0x3F;
            return true;
        case 0x9030:
            if (!(m_control & 0x40)) {
                m_opllRegisters[m_opllAddress & 0x3F] = data;
                m_opll.write(m_opllAddress, data);
            }
            return true;

        case 0xA000: m_chr[0] = data; return true;
        case 0xA008: m_chr[1] = data; return true;
        case 0xB000: m_chr[2] = data; return true;
        case 0xB008: m_chr[3] = data; return true;
        case 0xC000: m_chr[4] = data; return true;
        case 0xC008: m_chr[5] = data; return true;
        case 0xD000: m_chr[6] = data; return true;
        case 0xD008: m_chr[7] = data; return true;

        case 0xE000:
            m_control = data;
            if (data & 0x40) {

                std::fill(std::begin(m_opllRegisters), std::end(m_opllRegisters), uint8_t(0));
                m_opll.resetAudio();
            }
            switch (data & 0x03) {
            case 0: m_mirror = Mirror::Vertical; break;
            case 1: m_mirror = Mirror::Horizontal; break;
            case 2: m_mirror = Mirror::OnescreenLo; break;
            case 3: m_mirror = Mirror::OnescreenHi; break;
            }
            return true;
        case 0xE008: m_irq.latch = data; return true;
        case 0xF000: m_irq.writeControl(data); return true;
        case 0xF008: m_irq.acknowledge(); return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        mapped = mapBank(m_chr[addr >> 10], 0x400, size, addr & 0x3FF);
        return true;
    }
    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    void clockCpu() override { m_irq.clockCpu(); m_opll.clockCpu(); }
    bool irqActive() const override { return m_irq.pending; }
    float expansionAudioSample(bool chipMod = false) const override { return (m_control & 0x40) || (m_config.nes20 && m_config.submapper == 1) ? 0.0f : m_opll.sample(chipMod); }

    void saveState(std::vector<uint8_t>& out) const override {
        for (auto v : m_prg) put8(out, v);
        for (auto v : m_chr) put8(out, v);
        put8(out, m_control);
        put8(out, m_opllAddress);
        for (auto v : m_opllRegisters) put8(out, v);
        m_opll.save(out);
        m_irq.save(out);
        put8(out, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        uint8_t mirror;
        for (auto& v : m_prg) if (!get8(p, end, v)) return false;
        for (auto& v : m_chr) if (!get8(p, end, v)) return false;
        if (!get8(p, end, m_control) || !get8(p, end, m_opllAddress)) return false;
        for (auto& v : m_opllRegisters) if (!get8(p, end, v)) return false;
        if (!m_opll.load(p, end)) return false;
        if (!m_irq.load(p, end) || !get8(p, end, mirror)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    uint8_t m_prg[3]{};
    uint8_t m_chr[8]{};
    uint8_t m_control = 0;
    uint8_t m_opllAddress = 0;
    uint8_t m_opllRegisters[0x40]{};
    mapper_more_detail::Vrc7Opll m_opll;
    mapper_more_detail::VrcIrq m_irq;
};

}

std::unique_ptr<Mapper> createVrcMapper(const MapperConfig& config)
{
    switch (config.id) {
    case 21: case 22: case 23: case 25: case 27: return std::make_unique<MapperVrc24>(config);
    case 24: case 26: return std::make_unique<MapperVrc6>(config);
    case 73: return std::make_unique<Mapper73>(config);
    case 75: case 151: return std::make_unique<Mapper75>(config);
    case 85: return std::make_unique<Mapper85>(config);
    default: return nullptr;
    }
}
