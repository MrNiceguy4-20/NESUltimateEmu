// Additional licensed/discrete NES mappers. Included inside Mapper.cpp's
// anonymous namespace so these helpers stay private to the cartridge core.
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

    // VRC7 instrument ROM. Instrument 0 is the eight writable custom registers.
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
        // The real OPLL envelope is logarithmic and advances on a shared EG
        // counter. This fixed-point step preserves the documented relative
        // rate ordering while remaining stable at the 49.716 kHz OPLL clock.
        unsigned base=1u << (r/3);
        if(attack) base*=3;
        return uint16_t(std::min<unsigned>(512,base));
    }
    static double sine(uint32_t phase){
        constexpr double tau=6.283185307179586476925286766559;
        return std::sin(tau*double(phase&0x7FFFFu)/524288.0);
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
        // emu2413's OPLL phase generator is a 19-bit accumulator.  VRC7 is
        // clocked at ~49.716 kHz; this follows the same f-number/block/multiplier
        // progression while the surrounding emulator supplies that clock.
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
        // NTSC CPU / 36 ~= 49.716 kHz, the VRC7 OPLL output cadence used by
        // hardware-oriented emulators. Keep the fractional error bounded by
        // alternating 36/35-cycle intervals with a 49716:1789773 accumulator.
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

struct SerialEeprom {
    enum class Mode:uint8_t{Idle,Address,Read,Write,SendAck,WaitAck,ChipAddress};
    bool type02=false; uint16_t size=0; uint8_t mem[256]={}; Mode mode=Mode::Idle,next=Mode::Idle;
    uint8_t chip=0,address=0,data=0,counter=0,output=1,prevScl=0,prevSda=0;
    SerialEeprom()=default;
    SerialEeprom(bool c02,uint16_t n):type02(c02),size(n){std::memset(mem,0xFF,sizeof(mem));}
    void writeBit(uint8_t&dst,uint8_t bit){if(counter>=8)return;uint8_t pos=type02?uint8_t(7-counter):counter;dst=uint8_t((dst&~(1u<<pos))|((bit&1u)<<pos));++counter;}
    void readBit(){if(counter>=8)return;uint8_t pos=type02?uint8_t(7-counter):counter;output=(data>>pos)&1;++counter;}
    void write(uint8_t scl,uint8_t sda){
        if(prevScl&&scl&&sda<prevSda){mode=type02?Mode::ChipAddress:Mode::Address;chip=0;address=0;counter=0;output=1;}
        else if(prevScl&&scl&&sda>prevSda){mode=Mode::Idle;output=1;}
        else if(scl>prevScl){
            if(type02){switch(mode){case Mode::ChipAddress:writeBit(chip,sda);break;case Mode::Address:writeBit(address,sda);break;case Mode::Read:readBit();break;case Mode::Write:writeBit(data,sda);break;case Mode::SendAck:output=0;break;case Mode::WaitAck:if(!sda){next=Mode::Read;data=mem[address];}break;default:break;}}
            else {switch(mode){case Mode::Address:if(counter<7)writeBit(address,sda);else if(counter==7){counter=8;if(sda){next=Mode::Read;data=mem[address&0x7F];}else next=Mode::Write;}break;case Mode::SendAck:output=0;break;case Mode::Read:readBit();break;case Mode::Write:writeBit(data,sda);break;case Mode::WaitAck:if(!sda)next=Mode::Idle;break;default:break;}}
        } else if(scl<prevScl){
            if(type02){switch(mode){case Mode::ChipAddress:if(counter==8){if((chip&0xA0)==0xA0){mode=Mode::SendAck;counter=0;output=1;if(chip&1){next=Mode::Read;data=mem[address];}else next=Mode::Address;}else{mode=Mode::Idle;counter=0;output=1;}}break;case Mode::Address:if(counter==8){counter=0;mode=Mode::SendAck;next=Mode::Write;output=1;}break;case Mode::Read:if(counter==8){mode=Mode::WaitAck;address=uint8_t(address+1);}break;case Mode::Write:if(counter==8){counter=0;mode=Mode::SendAck;next=Mode::Write;mem[address]=data;address=uint8_t(address+1);}break;case Mode::SendAck:case Mode::WaitAck:mode=next;counter=0;output=1;break;default:break;}}
            else {switch(mode){case Mode::Address:if(counter==8){mode=Mode::SendAck;output=1;}break;case Mode::SendAck:mode=next;counter=0;output=1;break;case Mode::Read:if(counter==8){mode=Mode::WaitAck;address=uint8_t((address+1)&0x7F);}break;case Mode::Write:if(counter==8){mode=Mode::SendAck;next=Mode::Idle;mem[address&0x7F]=data;address=uint8_t((address+1)&0x7F);}break;default:break;}}
        }
        prevScl=scl;prevSda=sda;
    }
    void writeScl(uint8_t v){write(v,prevSda);} void writeSda(uint8_t v){write(prevScl,v);}
    void save(std::vector<uint8_t>&o)const{put8(o,type02);put8(o,uint8_t(size));put8(o,uint8_t(size>>8));put8(o,uint8_t(mode));put8(o,uint8_t(next));put8(o,chip);put8(o,address);put8(o,data);put8(o,counter);put8(o,output);put8(o,prevScl);put8(o,prevSda);o.insert(o.end(),mem,mem+size);}
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t b,l,h;if(!get8(p,e,b))return false;type02=b;if(!get8(p,e,l)||!get8(p,e,h))return false;uint16_t n=uint16_t(l)|(uint16_t(h)<<8);if(n!=size||size_t(e-p)<9+size)return false;if(!get8(p,e,b))return false;mode=Mode(b);if(!get8(p,e,b))return false;next=Mode(b);if(!get8(p,e,chip)||!get8(p,e,address)||!get8(p,e,data)||!get8(p,e,counter)||!get8(p,e,output)||!get8(p,e,prevScl)||!get8(p,e,prevSda))return false;std::memcpy(mem,p,size);p+=size;return true;}
};
}

class Mapper16 final : public Mapper {
public:
    explicit Mapper16(const MapperConfig&c):Mapper(c){
        if(c.id==157){m_std=mapper_more_detail::SerialEeprom(true,256);m_hasStd=true;if(!c.nes20||c.headerPrgNvRamSize==128){m_extra=mapper_more_detail::SerialEeprom(false,128);m_hasExtra=true;}}
        else if(c.id==159){m_std=mapper_more_detail::SerialEeprom(false,128);m_hasStd=true;}
        else if(c.id==16 && (!c.nes20 || (c.submapper==5&&c.headerPrgNvRamSize==256) || (c.submapper==0&&c.headerPrgNvRamSize==256))){m_std=mapper_more_detail::SerialEeprom(true,256);m_hasStd=true;}
    }
    bool cpuReadRegister(uint16_t a,uint8_t&d)override{if(a<0x6000||a>0x7FFF||m_config.id==153||(!m_hasStd&&!m_hasExtra))return false;bool bit=true;if(m_hasStd)bit&=m_std.output!=0;if(m_hasExtra)bit&=m_extra.output!=0;d=bit?0x10:0;return true;}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t outer=m_prgOuter?16:0;if(a<0xC000)m=mapBank((m_prg&0x0F)|outer,0x4000,m_config.prgRomSize,a-0x8000);else m=mapBank(0x0F|outer,0x4000,m_config.prgRomSize,a-0xC000);return true;}
    bool mapPrgRam(uint16_t a,uint32_t&m,bool)const override{if(m_config.id!=153||a<0x6000||a>0x7FFF||!m_config.prgRamSize||!m_ramEnable)return false;m=(a-0x6000)%m_config.prgRamSize;return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if(a<0x6000)return false;
        if(m_config.id==153&&a<0x8000)return false;
        if((m_config.id==157||m_config.id==159)&&a<0x8000)return false;
        if(m_config.id==16&&m_config.nes20&&m_config.submapper==4&&a>=0x8000)return false;
        if(m_config.id==16&&m_config.nes20&&m_config.submapper==5&&a<0x8000)return false;
        uint8_t r=a&0x0F;
        if(r<8){m_chr[r]=d;if(m_config.id==153||m_config.prgRomSize>=0x80000){m_prgOuter=0;for(auto v:m_chr)m_prgOuter|=v&1;}if(m_hasExtra&&m_config.id==157&&r<=3)m_extra.writeScl((d>>3)&1);}
        else if(r==8)m_prg=d&0x0F;
        else if(r==9)mapper_more_detail::setMirror4(m_mirror,d);
        else if(r==0xA){m_irqEnabled=(d&1)!=0;m_irqPending=false;if(m_config.id!=16||!m_config.nes20||m_config.submapper==5)m_irqCounter=m_irqLatch;}
        else if(r==0xB){if(m_config.id==16&&m_config.nes20&&m_config.submapper==4)m_irqCounter=uint16_t((m_irqCounter&0xFF00)|d);else m_irqLatch=uint16_t((m_irqLatch&0xFF00)|d);}
        else if(r==0xC){if(m_config.id==16&&m_config.nes20&&m_config.submapper==4)m_irqCounter=uint16_t((m_irqCounter&0x00FF)|(uint16_t(d)<<8));else m_irqLatch=uint16_t((m_irqLatch&0x00FF)|(uint16_t(d)<<8));}
        else if(r==0xD){if(m_config.id==153)m_ramEnable=(d&0x20)!=0;else{uint8_t scl=(d>>5)&1,sda=(d>>6)&1;if(m_hasStd)m_std.write(scl,sda);if(m_hasExtra)m_extra.writeSda(sda);}}
        return true;
    }
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(m_config.id==157||m_config.id==153||m_config.chrRomSize==0)m=a%s;else m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void clockCpu()override{if(m_irqEnabled){if(m_irqCounter==0)m_irqPending=true;--m_irqCounter;}}
    bool irqActive()const override{return m_irqPending;}
    std::size_t mapperBatterySize()const override{return (m_hasStd?m_std.size:0)+(m_hasExtra?m_extra.size:0);}
    void saveMapperBattery(std::vector<uint8_t>&o)const override{if(m_hasStd)o.insert(o.end(),m_std.mem,m_std.mem+m_std.size);if(m_hasExtra)o.insert(o.end(),m_extra.mem,m_extra.mem+m_extra.size);}
    bool loadMapperBattery(const uint8_t*d,std::size_t n)override{if(n!=mapperBatterySize())return false;size_t off=0;if(m_hasStd){std::memcpy(m_std.mem,d+off,m_std.size);off+=m_std.size;}if(m_hasExtra)std::memcpy(m_extra.mem,d+off,m_extra.size);return true;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(auto v:m_chr)put8(o,v);put8(o,uint8_t(m_irqLatch));put8(o,uint8_t(m_irqLatch>>8));put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));put8(o,m_irqEnabled);put8(o,m_irqPending);put8(o,m_prgOuter);put8(o,m_ramEnable);put8(o,static_cast<uint8_t>(m_mirror));put8(o,m_hasStd);if(m_hasStd)m_std.save(o);put8(o,m_hasExtra);if(m_hasExtra)m_extra.save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;if(!get8(p,e,m_prg))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqLatch=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqCounter=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,b))return false;m_irqEnabled=b;if(!get8(p,e,b))return false;m_irqPending=b;if(!get8(p,e,m_prgOuter)||!get8(p,e,b))return false;m_ramEnable=b;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);if(!get8(p,e,b)||bool(b)!=m_hasStd)return false;if(m_hasStd&&!m_std.load(p,e))return false;if(!get8(p,e,b)||bool(b)!=m_hasExtra)return false;if(m_hasExtra&&!m_extra.load(p,e))return false;return true;}
private:uint8_t m_prg=0,m_chr[8]={},m_prgOuter=0;uint16_t m_irqLatch=0,m_irqCounter=0;bool m_irqEnabled=false,m_irqPending=false,m_ramEnable=true,m_hasStd=false,m_hasExtra=false;mapper_more_detail::SerialEeprom m_std,m_extra;
};

class Mapper18 final : public Mapper {
public:
    explicit Mapper18(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{uint16_t r=a&0xF003;if(a<0x8000)return false;if(r>=0x8000&&r<=0x9001){int i=(r>=0x9000)?2:((r>>1)&1);bool hi=r&1;m_prg[i]=uint8_t(hi?((m_prg[i]&0x0F)|((d&0x0F)<<4)):((m_prg[i]&0xF0)|(d&0x0F)));return true;}if(r>=0xA000&&r<=0xD003){int q=((r>>12)-0xA)*2+((r>>1)&1);bool hi=r&1;m_chr[q]=uint8_t(hi?((m_chr[q]&0x0F)|((d&0x0F)<<4)):((m_chr[q]&0xF0)|(d&0x0F)));return true;}if(r>=0xE000&&r<=0xE003){int sh=(r&3)*4;m_latch=uint16_t((m_latch&~(0xFu<<sh))|((d&0xF)<<sh));return true;}if(r==0xF000){m_counter=m_latch;m_pending=false;return true;}if(r==0xF001){m_enabled=(d&1)!=0;m_pending=false;return true;}if(r==0xF002){mapper_more_detail::setMirror4(m_mirror,d);return true;}return false;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void clockCpu()override{if(m_enabled){if(m_counter==0){m_pending=true;m_enabled=false;}else --m_counter;}}bool irqActive()const override{return m_pending;}
    void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);for(uint16_t v:{m_latch,m_counter}){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}put8(o,m_enabled);put8(o,m_pending);put8(o,static_cast<uint8_t>(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_latch=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_counter=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_pending=b;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}
private:uint8_t m_prg[3]={},m_chr[8]={};uint16_t m_latch=0,m_counter=0;bool m_enabled=false,m_pending=false;
};

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
        if((r&0xF003)==0xB003){mapper_more_detail::setMirror4(m_mirror,(d>>2)&3);return true;}
        if((r&0xF003)==0xC000){m_prg8=d&0x1F;return true;}
        if(r>=0xD000&&r<=0xE003){int i=((r>>12)-0xD)*4+(r&3);if(i<8)m_chr[i]=d;return true;}
        if((r&0xF003)==0xF000){m_irq.latch=d;return true;}if((r&0xF003)==0xF001){m_irq.writeControl(d);return true;}if((r&0xF003)==0xF002){m_irq.acknowledge();return true;}return false;
    }
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
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
        // Keep the stock pulse gain unchanged when Chip Mod is enabled. The
        // smooth KYLXBN saw may legitimately exceed the stock staircase's
        // peak; final APU output headroom handles that rather than clipping the
        // expansion chip here.
        return -(raw / 61.0f) * 0.55f;
    }
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg16);put8(o,m_prg8);for(auto v:m_chr)put8(o,v);m_irq.save(o);put8(o,static_cast<uint8_t>(m_mirror));put8(o,m_audioCtrl);for(auto&i:m_pulse)i.save(o);m_saw.save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg16)||!get8(p,e,m_prg8))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!m_irq.load(p,e)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);if(!get8(p,e,m_audioCtrl))return false;for(auto&i:m_pulse)if(!i.load(p,e))return false;return m_saw.load(p,e);}
private:
    struct Pulse {
        uint8_t ctrl = 0, hi = 0, step = 0;
        uint16_t freq = 0, timer = 0; // timer is the reference up-counter

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
        uint16_t freq = 0, timer = 0; // reference up-counter

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
                    acc = uint8_t(acc + rate); // 8-bit hardware wrapping
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

    bool m_swap;uint8_t m_prg16=0,m_prg8=0,m_chr[8]={},m_audioCtrl=0;Pulse m_pulse[2];Saw m_saw;mapper_more_detail::VrcIrq m_irq;
    uint8_t audioShift() const { return (m_audioCtrl & 4) ? 8 : ((m_audioCtrl & 2) ? 4 : 0); }
    uint16_t translate(uint16_t a)const{if(!m_swap)return a;uint16_t lo=a&3;lo=((lo&1)<<1)|((lo&2)>>1);return uint16_t((a&~3)|lo);}
};

class Mapper32 final : public Mapper {public:explicit Mapper32(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000),last=n-1,last2=n>1?n-2:0;size_t b=!m_mode?(a<0xA000?m_p0:a<0xC000?m_p1:a<0xE000?last2:last):(a<0xA000?last2:a<0xC000?m_p1:a<0xE000?m_p0:last);m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;switch(a&0xF000){case 0x8000:m_p0=d;return true;case 0x9000:m_mode=(d>>1)&1;if(!m_config.fourScreen)m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;return true;case 0xA000:m_p1=d;return true;case 0xB000:m_chr[a&7]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_p0);put8(o,m_p1);put8(o,m_mode);for(auto v:m_chr)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_p0)||!get8(p,e,m_p1)||!get8(p,e,m_mode))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_p0=0,m_p1=0,m_mode=0,m_chr[8]={};};

class Mapper33 : public Mapper {public:explicit Mapper33(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xE003){case 0x8000:m_prg[0]=d&0x3F;if(!m_config.fourScreen)m_mirror=(d&0x40)?Mirror::Horizontal:Mirror::Vertical;return true;case 0x8001:m_prg[1]=d&0x3F;return true;case 0x8002:m_chr2[0]=d;return true;case 0x8003:m_chr2[1]=d;return true;case 0xA000:m_chr1[0]=d;return true;case 0xA001:m_chr1[1]=d;return true;case 0xA002:m_chr1[2]=d;return true;case 0xA003:m_chr1[3]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x1000)m=mapBank(m_chr2[a>>11],0x800,s,a&0x7FF);else m=mapBank(m_chr1[(a-0x1000)>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr2)put8(o,v);for(auto v:m_chr1)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr2)if(!get8(p,e,v))return false;for(auto&v:m_chr1)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}protected:uint8_t m_prg[2]={},m_chr2[2]={},m_chr1[4]={};};

class Mapper48 final : public Mapper33 {
public:explicit Mapper48(const MapperConfig&c):Mapper33(c){}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t c)override{switch(a&0xE003){case 0xC000:m_pending=false;m_delay=0;m_latch=uint8_t((d^0xFF)+1);return true;case 0xC001:m_pending=false;m_delay=0;m_counter=0;m_reload=true;return true;case 0xC002:m_enabled=true;return true;case 0xC003:m_enabled=false;m_pending=false;m_delay=0;return true;case 0xE000:if(!m_config.fourScreen)m_mirror=(d&0x40)?Mirror::Horizontal:Mirror::Vertical;return true;}return Mapper33::cpuWrite(a,d,c);}
    void notifyPpuAddress(uint16_t a,uint64_t cyc)override{bool hi=a&0x1000;if(!hi){if(m_lastA12||!m_lowValid){m_lowStart=cyc;m_lowValid=true;}}else{if(!m_lastA12&&m_lowValid&&cyc-m_lowStart>=8)clockIrq();m_lowValid=false;}m_lastA12=hi;}
    void clockCpu()override{if(m_delay&&--m_delay==0)m_pending=true;}bool irqActive()const override{return m_pending;}
    void saveState(std::vector<uint8_t>&o)const override{Mapper33::saveState(o);put8(o,m_latch);put8(o,m_counter);put8(o,m_delay);put8(o,m_enabled);put8(o,m_reload);put8(o,m_pending);put8(o,m_lastA12);put8(o,m_lowValid);put64(o,m_lowStart);}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!Mapper33::loadState(p,e)||!get8(p,e,m_latch)||!get8(p,e,m_counter)||!get8(p,e,m_delay))return false;if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_reload=b;if(!get8(p,e,b))return false;m_pending=b;if(!get8(p,e,b))return false;m_lastA12=b;if(!get8(p,e,b))return false;m_lowValid=b;return get64(p,e,m_lowStart);}
private:uint8_t m_latch=0,m_counter=0,m_delay=0;bool m_enabled=false,m_reload=false,m_pending=false,m_lastA12=false,m_lowValid=false;uint64_t m_lowStart=0;void clockIrq(){if(m_counter==0||m_reload){m_counter=m_latch;m_reload=false;}else --m_counter;if(m_counter==0&&m_enabled&&!m_pending&&!m_delay)m_delay=6;}
};

class Mapper64 final : public Mapper {
public:explicit Mapper64(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000),last=n-1;bool sw=m_current&0x40;size_t b=a<0xA000?(sw?m_regs[15]:m_regs[6]):a<0xC000?m_regs[7]:a<0xE000?(sw?m_regs[6]:m_regs[15]):last;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xE001){case 0x8000:m_current=d;return true;case 0x8001:m_regs[m_current&0x0F]=d;return true;case 0xA000:if(m_config.id!=158&&!m_config.fourScreen)m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;return true;case 0xC000:m_reloadValue=d;return true;case 0xC001:if(m_cycleMode&&!(d&1))m_force=true;m_cycleMode=d&1;if(m_cycleMode)m_cpuDiv=0;m_needReload=true;return true;case 0xE000:m_enabled=false;m_pending=false;m_delay=0;return true;case 0xE001:m_enabled=true;return true;}return false;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;uint8_t slot=a>>10,inv=(m_current&0x80)?4:0,l=slot^inv,b=0;switch(l){case 0:b=m_regs[0];break;case 1:b=(m_current&0x20)?m_regs[8]:uint8_t(m_regs[0]+1);break;case 2:b=m_regs[1];break;case 3:b=(m_current&0x20)?m_regs[9]:uint8_t(m_regs[1]+1);break;case 4:b=m_regs[2];break;case 5:b=m_regs[3];break;case 6:b=m_regs[4];break;case 7:b=m_regs[5];break;}m=mapBank(b,0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(m_config.id!=158||a<0x2000||a>0x3EFF)return false;uint8_t p=(a&0xFFF)>>10;uint8_t nt=(m_current&0x80)?uint8_t(m_regs[2+p]>>7):uint8_t((p<2?m_regs[0]:m_regs[1])>>7);s=NametableSource::Ciram;m=uint32_t(nt)*0x400+(a&0x3FF);return true;}
    void notifyPpuAddress(uint16_t a,uint64_t cyc)override{if(m_cycleMode)return;bool hi=a&0x1000;if(!hi){if(m_lastA12||!m_lowValid){m_lowStart=cyc;m_lowValid=true;}}else{if(!m_lastA12&&m_lowValid&&cyc-m_lowStart>=8)clockCounter(2);m_lowValid=false;}m_lastA12=hi;}
    void clockCpu()override{if(m_delay&&--m_delay==0)m_pending=true;if(m_cycleMode||m_force){m_cpuDiv=(m_cpuDiv+1)&3;if(m_cpuDiv==0){clockCounter(1);m_force=false;}}}bool irqActive()const override{return m_pending;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_current);for(auto v:m_regs)put8(o,v);put8(o,m_enabled);put8(o,m_cycleMode);put8(o,m_needReload);put8(o,m_counter);put8(o,m_reloadValue);put8(o,m_cpuDiv);put8(o,m_delay);put8(o,m_pending);put8(o,m_force);put8(o,m_lastA12);put8(o,m_lowValid);put64(o,m_lowStart);put8(o,static_cast<uint8_t>(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_current))return false;for(auto&v:m_regs)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_cycleMode=b;if(!get8(p,e,b))return false;m_needReload=b;if(!get8(p,e,m_counter)||!get8(p,e,m_reloadValue)||!get8(p,e,m_cpuDiv)||!get8(p,e,m_delay))return false;if(!get8(p,e,b))return false;m_pending=b;if(!get8(p,e,b))return false;m_force=b;if(!get8(p,e,b))return false;m_lastA12=b;if(!get8(p,e,b))return false;m_lowValid=b;if(!get64(p,e,m_lowStart)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}
private:uint8_t m_current=0,m_regs[16]={};bool m_enabled=false,m_cycleMode=false,m_needReload=false;uint8_t m_counter=0,m_reloadValue=0,m_cpuDiv=0,m_delay=0;bool m_pending=false,m_force=false,m_lastA12=false,m_lowValid=false;uint64_t m_lowStart=0;void clockCounter(uint8_t delay){if(m_needReload){m_counter=m_reloadValue<=1?uint8_t(m_reloadValue+1):uint8_t(m_reloadValue+2);m_needReload=false;}else if(m_counter==0)m_counter=uint8_t(m_reloadValue+1);--m_counter;if(m_counter==0&&m_enabled&&!m_pending&&!m_delay)m_delay=delay;}
};

class Mapper65 final : public Mapper {public:explicit Mapper65(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a==0x8000){m_prg[0]=d;return true;}if(a==0xA000){m_prg[1]=d;return true;}if(a==0xC000){m_prg[2]=d;return true;}if(a>=0xB000&&a<=0xB007){m_chr[a&7]=d;return true;}if(a==0x9000){if(!m_config.fourScreen)m_mirror=(d&0x40)?Mirror::Horizontal:Mirror::Vertical;return true;}if(a==0x9003){m_enabled=d&0x80;m_pending=false;return true;}if(a==0x9004){m_counter=m_latch;m_pending=false;return true;}if(a==0x9005){m_latch=uint16_t((m_latch&0xFF)|(uint16_t(d)<<8));return true;}if(a==0x9006){m_latch=uint16_t((m_latch&0xFF00)|d);return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void clockCpu()override{if(m_enabled){if(m_counter==0){m_pending=true;m_enabled=false;}else --m_counter;}}bool irqActive()const override{return m_pending;}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);put8(o,uint8_t(m_latch));put8(o,uint8_t(m_latch>>8));put8(o,uint8_t(m_counter));put8(o,uint8_t(m_counter>>8));put8(o,m_enabled);put8(o,m_pending);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t l,h,b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,l)||!get8(p,e,h))return false;m_latch=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,l)||!get8(p,e,h))return false;m_counter=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_pending=b;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg[3]={},m_chr[8]={};uint16_t m_latch=0,m_counter=0;bool m_enabled=false,m_pending=false;};

class Mapper67 final : public Mapper {public:explicit Mapper67(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF800){case 0x8800:m_chr[0]=d;return true;case 0x9800:m_chr[1]=d;return true;case 0xA800:m_chr[2]=d;return true;case 0xB800:m_chr[3]=d;return true;case 0xC800:if(!m_irqToggle){m_counter=uint16_t((m_counter&0x00FF)|(uint16_t(d)<<8));m_irqToggle=true;}else{m_counter=uint16_t((m_counter&0xFF00)|d);m_irqToggle=false;}m_pending=false;return true;case 0xD800:m_enabled=(d&0x10)!=0;m_pending=false;return true;case 0xE800:mapper_more_detail::setMirror4(m_mirror,d);return true;case 0xF800:m_prg=d&0x0F;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void clockCpu()override{if(m_enabled){if(m_counter==0xFFFF){m_pending=true;m_enabled=false;}++m_counter;}}bool irqActive()const override{return m_pending;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(auto v:m_chr)put8(o,v);put8(o,uint8_t(m_counter));put8(o,uint8_t(m_counter>>8));put8(o,m_enabled);put8(o,m_pending);put8(o,m_irqToggle);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t l,h,b;if(!get8(p,e,m_prg))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,l)||!get8(p,e,h))return false;m_counter=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_pending=b;if(!get8(p,e,b))return false;m_irqToggle=b;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr[4]={};uint16_t m_counter=0;bool m_enabled=false,m_pending=false,m_irqToggle=false;};

class Mapper68 final : public Mapper {
public:explicit Mapper68(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;if(a<0xC000){if(m_external&&m_timer==0)return false;m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);}else m=mapBank(7,0x4000,m_config.prgRomSize,a-0xC000);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a>=0x6000&&a<=0x7FFF){if(m_ramEnable&&m_external)m_timer=1024u*105u;return false;}if(a<0x8000)return false;switch(a&0xF000){case 0x8000:m_chr[0]=d;return true;case 0x9000:m_chr[1]=d;return true;case 0xA000:m_chr[2]=d;return true;case 0xB000:m_chr[3]=d;return true;case 0xC000:m_nt[0]=d|0x80;return true;case 0xD000:m_nt[1]=d|0x80;return true;case 0xE000:mapper_more_detail::setMirror4(m_mirror,d);m_chrNt=d&0x10;return true;case 0xF000:{size_t pages=m_config.prgRomSize/0x4000;m_external=((d&8)==0&&pages>8);if(m_external){size_t extra=pages-8;m_externalPage=uint8_t(8+((d&7)%std::max<size_t>(1,extra)));m_prg=m_externalPage;}else m_prg=d&7;m_ramEnable=d&0x10;return true;}}return false;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapPrgRam(uint16_t a,uint32_t&m,bool)const override{if(!m_ramEnable||a<0x6000||a>0x7FFF||!m_config.prgRamSize)return false;m=(a-0x6000)%m_config.prgRamSize;return true;}
    bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(!m_chrNt||a<0x2000||a>0x3EFF)return false;uint8_t p=(a&0xFFF)>>10,r=0;switch(m_mirror){case Mirror::Vertical:r=p&1;break;case Mirror::Horizontal:r=(p>>1)&1;break;case Mirror::OnescreenLo:r=0;break;case Mirror::OnescreenHi:r=1;break;case Mirror::FourScreen:r=p&1;break;}s=m_config.chrRomSize?NametableSource::ChrRom:NametableSource::ChrRam;m=uint32_t(m_nt[r])*0x400+(a&0x3FF);return true;}
    void clockCpu()override{if(m_timer)--m_timer;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(auto v:m_chr)put8(o,v);for(auto v:m_nt)put8(o,v);put8(o,m_chrNt);put8(o,m_ramEnable);put8(o,m_external);put8(o,m_externalPage);put64(o,m_timer);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;for(auto&v:m_nt)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_chrNt=b;if(!get8(p,e,b))return false;m_ramEnable=b;if(!get8(p,e,b))return false;m_external=b;if(!get8(p,e,m_externalPage)||!get64(p,e,m_timer)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}
private:uint8_t m_prg=0,m_chr[4]={},m_nt[2]={};bool m_chrNt=false,m_ramEnable=false,m_external=false;uint8_t m_externalPage=0;uint64_t m_timer=0;
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
            case 13:
                m_counterEnable = (d & 0x80) != 0;
                m_irqEnable = (d & 1) != 0;
                if (!m_irqEnable) m_pending = false;
                break;
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
        m = (a - 0x6000) % m_config.prgRamSize;
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
        if (m_counterEnable) {
            if (m_counter == 0) {
                m_counter = 0xFFFF;
                if (m_irqEnable) m_pending = true;
            } else {
                --m_counter;
            }
        }

        // The Sunsoft 5B PSG receives one emu2149 state update per eight
        // NTSC CPU clocks. The chip itself runs at twice the CPU clock and
        // divides internally by 16, yielding this exact /8 cadence.
        if (++m_psgDivider >= 8) {
            m_psgDivider = 0;
            clockPsg();
        }
    }

    bool irqActive() const override { return m_pending; }

    float expansionAudioSample(bool = false) const override {
        const int sum = int(m_psgOut[0]) + int(m_psgOut[1]) + int(m_psgOut[2]);
        // Sunsoft 5B output is inverted relative to the 2A03 path. 0.64 is
        // the reference NSFPlay master level after the three PSG channels.
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
        // Envelope generator (emu2149/AY-3-8910 compatible behavior).
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

        // 17-bit noise LFSR used by the Sunsoft 5B PSG.
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

class Mapper70 final : public Mapper {public:explicit Mapper70(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=(d>>4)&7;m_chr=d&0x0F;if(m_config.id==152)m_mirror=(d&0x80)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper72 final : public Mapper {public:explicit Mapper72(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(m_config.id==92){if((d&0xF0)==0xD0)m_prg=d&0x0F;else if((d&0xF0)==0xE0)m_chr=d&0x0F;}else{if(d&0x80)m_prg=d&0x0F;if(d&0x40)m_chr=d&0x0F;}return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper73 final : public Mapper {public:explicit Mapper73(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF000){case 0x8000:m_latch=(m_latch&0xFFF0)|(d&0xF);return true;case 0x9000:m_latch=(m_latch&0xFF0F)|((d&0xF)<<4);return true;case 0xA000:m_latch=(m_latch&0xF0FF)|((d&0xF)<<8);return true;case 0xB000:m_latch=(m_latch&0x0FFF)|((d&0xF)<<12);return true;case 0xC000:m_ack=d&1;m_enabled=d&2;m_mode8=d&4;m_pending=false;if(m_enabled)m_counter=m_latch;return true;case 0xD000:m_pending=false;m_enabled=m_ack;return true;case 0xF000:m_prg=d&7;return true;}return false;}void clockCpu()override{if(!m_enabled)return;if(m_mode8){uint8_t v=uint8_t(m_counter);if(v==0xFF){m_counter=(m_counter&0xFF00)|(m_latch&0xFF);m_pending=true;}else m_counter=(m_counter&0xFF00)|uint8_t(v+1);}else{if(m_counter==0xFFFF){m_counter=m_latch;m_pending=true;}else ++m_counter;}}bool irqActive()const override{return m_pending;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(uint16_t v:{m_latch,m_counter}){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}put8(o,m_enabled);put8(o,m_ack);put8(o,m_mode8);put8(o,m_pending);}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t l,h,b;if(!get8(p,e,m_prg)||!get8(p,e,l)||!get8(p,e,h))return false;m_latch=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,l)||!get8(p,e,h))return false;m_counter=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_ack=b;if(!get8(p,e,b))return false;m_mode8=b;if(!get8(p,e,b))return false;m_pending=b;return true;}private:uint8_t m_prg=0;uint16_t m_latch=0,m_counter=0;bool m_enabled=false,m_ack=false,m_mode8=false,m_pending=false;};

class Mapper75 final : public Mapper {public:explicit Mapper75(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF000){case 0x8000:m_prg[0]=d&0x0F;return true;case 0x9000:m_chr[0]=uint8_t((m_chr[0]&0x0F)|((d&2)<<3));m_chr[1]=uint8_t((m_chr[1]&0x0F)|((d&4)<<2));if(!m_config.fourScreen)m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;return true;case 0xA000:m_prg[1]=d&0x0F;return true;case 0xC000:m_prg[2]=d&0x0F;return true;case 0xE000:m_chr[0]=uint8_t((m_chr[0]&0x10)|(d&0x0F));return true;case 0xF000:m_chr[1]=uint8_t((m_chr[1]&0x10)|(d&0x0F));return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>12],0x1000,s,a&0xFFF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg[3]={},m_chr[2]={};};

class Mapper76 final : public Mapper {public:explicit Mapper76(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE001)==0x8000){m_sel=d&7;return true;}if((a&0xE001)==0x8001){if(m_sel>=2&&m_sel<=5)m_chr[m_sel-2]=d;else if(m_sel>=6)m_prg[m_sel-6]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_prg[2]={},m_chr[4]={};};

class Mapper77 final : public Mapper {public:explicit Mapper77(const MapperConfig&c):Mapper(c){m_mirror=Mirror::FourScreen;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&7;m_chr=d>>4;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;if(a<0x0800){if(!m_config.chrRomSize)return false;m=mapBank(m_chr,0x800,m_config.chrRomSize,a);return true;}if(!m_config.chrRamSize)return false;m=a-0x0800;return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{if(a<0x0800||a>=0x2000||!m_config.chrRamSize)return false;m=a-0x0800;return true;}bool ppuUsesChrRam(uint16_t a)const override{return a>=0x0800&&a<0x2000&&m_config.chrRamSize;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper78 final : public Mapper {public:explicit Mapper78(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&7;m_chr=(d>>4)&0x0F;if(m_config.submapper==1)m_mirror=(d&8)?Mirror::Horizontal:Mirror::Vertical;else m_mirror=(d&8)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper79 final : public Mapper {public:explicit Mapper79(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE100)!=0x4100)return false;m_prg=(d>>3)&1;m_chr=d&7;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper80 final : public Mapper {
public:explicit Mapper80(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x7EF0||a>0x7EFF)return false;switch(a){case 0x7EF0:m_chr[0]=d;m_chr[1]=uint8_t(d+1);break;case 0x7EF1:m_chr[2]=d;m_chr[3]=uint8_t(d+1);break;case 0x7EF2:m_chr[4]=d;break;case 0x7EF3:m_chr[5]=d;break;case 0x7EF4:m_chr[6]=d;break;case 0x7EF5:m_chr[7]=d;break;case 0x7EF6:case 0x7EF7:if(m_config.id!=207)m_mirror=(d&1)?Mirror::Vertical:Mirror::Horizontal;break;case 0x7EF8:case 0x7EF9:m_perm=d;break;case 0x7EFA:case 0x7EFB:m_prg[0]=d;break;case 0x7EFC:case 0x7EFD:m_prg[1]=d;break;case 0x7EFE:case 0x7EFF:m_prg[2]=d;break;}return true;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t a,uint32_t&m,bool)const override{if(a<0x7F00||a>0x7FFF||!m_config.prgRamSize||m_perm!=0xA3)return false;m=(a-0x7F00)%m_config.prgRamSize;return true;}
    bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(m_config.id!=207||a<0x2000||a>0x3EFF)return false;uint8_t p=(a&0xFFF)>>10,nt=(p<2?m_chr[0]:m_chr[2])>>7;s=NametableSource::Ciram;m=uint32_t(nt)*0x400+(a&0x3FF);return true;}
    void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);put8(o,m_perm);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,m_perm)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}
private:uint8_t m_prg[3]={},m_chr[8]={},m_perm=0;
};

class Mapper82 final : public Mapper {public:explicit Mapper82(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x7EF0||a>0x7EFF)return false;if(a<=0x7EF5){m_chr[a&0xF]=d;return true;}switch(a){case 0x7EF6:m_mirror=(d&1)?Mirror::Vertical:Mirror::Horizontal;m_mode=(d>>1)&1;break;case 0x7EF7:case 0x7EF8:case 0x7EF9:m_perm[(a&0xF)-7]=d;break;case 0x7EFA:m_prg[0]=d>>2;break;case 0x7EFB:m_prg[1]=d>>2;break;case 0x7EFC:m_prg[2]=d>>2;break;}return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;uint8_t slot=a>>10;uint8_t b;if(!m_mode){b=slot<2?uint8_t((m_chr[0]&0xFE)+(slot&1)):slot<4?uint8_t((m_chr[1]&0xFE)+(slot&1)):m_chr[slot-2];}else{b=slot<4?m_chr[slot+2]:slot<6?uint8_t((m_chr[0]&0xFE)+(slot&1)):uint8_t((m_chr[1]&0xFE)+(slot&1));}m=mapBank(b,0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t a,uint32_t&m,bool)const override{if(!m_config.prgRamSize)return false;if(a>=0x6000&&a<=0x67FF&&m_perm[0]==0xCA)m=a-0x6000;else if(a>=0x6800&&a<=0x6FFF&&m_perm[1]==0x69)m=a-0x6000;else if(a>=0x7000&&a<=0x73FF&&m_perm[2]==0x84)m=a-0x6000;else return false;m%=m_config.prgRamSize;return true;}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);for(auto v:m_perm)put8(o,v);put8(o,m_mode);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;for(auto&v:m_perm)if(!get8(p,e,v))return false;if(!get8(p,e,m_mode)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg[3]={},m_chr[6]={},m_perm[3]={},m_mode=0;};

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
        // VRC7 boards partially decode A3/A4. Normalize aliases the same way
        // as the hardware register map before dispatching.
        if ((addr & 0x10) && (addr & 0xF010) != 0x9010) {
            addr |= 0x08;
            addr &= static_cast<uint16_t>(~0x10);
        }

        switch (addr & 0xF038) {
        case 0x8000: m_prg[0] = data & 0x3F; return true;
        case 0x8008: m_prg[1] = data & 0x3F; return true;
        case 0x9000: m_prg[2] = data & 0x3F; return true;

        // VRC7 OPLL address/data ports.  The register image is retained here
        // so save states and a future dedicated OPLL core see exact writes.
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
    float expansionAudioSample(bool chipMod = false) const override { return (m_control & 0x40) ? 0.0f : m_opll.sample(chipMod); }

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

class Mapper86 final : public Mapper {public:explicit Mapper86(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_prg=(d>>4)&3;m_chr=(d&3)|((d>>4)&4);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper88 final : public Mapper {public:explicit Mapper88(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(m_config.id==154)m_mirror=(d&0x40)?Mirror::OnescreenHi:Mirror::OnescreenLo;if((a&0xE001)==0x8000){m_sel=d&7;return true;}if((a&0xE001)==0x8001){if(m_sel<=1)m_chr2[m_sel]=d>>1;else if(m_sel<=5)m_chr1[m_sel-2]=(d&0x3F)|0x40;else m_prg[m_sel-6]=d&0x3F;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x1000)m=mapBank(m_chr2[a>>11],0x800,s,a&0x7FF);else m=mapBank(m_chr1[(a-0x1000)>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr2)put8(o,v);for(auto v:m_chr1)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr2)if(!get8(p,e,v))return false;for(auto&v:m_chr1)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_sel=0,m_prg[2]={},m_chr2[2]={},m_chr1[4]={};};

class Mapper89 final : public Mapper {public:explicit Mapper89(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0xC000)return false;m_prg=(d>>4)&7;m_chr=(d&7)|((d>>4)&8);m_mirror=(d&8)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper93 final : public Mapper {public:explicit Mapper93(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=(d>>4)&7;return true;}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);}private:uint8_t m_prg=0;};

class Mapper95 final : public Mapper {public:explicit Mapper95(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_r[6]:a<0xC000?m_r[7]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(!(a&1))m_sel=d&7;else m_r[m_sel]=d;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x800)m=mapBank((m_r[0]&0xFE)>>1,0x800,s,a&0x7FF);else if(a<0x1000)m=mapBank((m_r[1]&0xFE)>>1,0x800,s,a&0x7FF);else m=mapBank(m_r[2+((a-0x1000)>>10)],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(a<0x2000||a>0x3EFF)return false;uint8_t p=(a&0xFFF)>>10,nt=((p<2?m_r[0]:m_r[1])>>5)&1;s=NametableSource::Ciram;m=uint32_t(nt)*0x400+(a&0x3FF);return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_r)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_r)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_r[8]={};};

class Mapper96 final : public Mapper {public:explicit Mapper96(const MapperConfig&c):Mapper(c){m_mirror=Mirror::OnescreenLo;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&3;m_outer=(d>>2)&1;m_mirror=(d&4)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000||!m_config.chrRamSize)return false;uint8_t b=uint8_t((m_outer<<2)|m_inner);m=mapBank(b,0x800,m_config.chrRamSize,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return ppuMapRead(a,m);}void notifyPpuAddress(uint16_t a,uint64_t)override{if((a&0x3000)==0x2000)m_inner=(a>>8)&3;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_outer);put8(o,m_inner);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_outer)||!get8(p,e,m_inner)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_outer=0,m_inner=0;};

class Mapper97 final : public Mapper {public:explicit Mapper97(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=a-0x8000;else m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0xC000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&0x0F;if(!m_config.fourScreen)m_mirror=(d&0x80)?Mirror::Horizontal:Mirror::Vertical;return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0;};

class Mapper101 final : public Mapper {public:explicit Mapper101(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=(a-0x8000)%m_config.prgRomSize;return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000)return false;m_chr=d&3;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_chr);}private:uint8_t m_chr=0;};

class Mapper113 final : public Mapper {public:explicit Mapper113(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE100)!=0x4100&&a<0x8000)return false;m_prg=(d>>3)&7;m_chr=(d&7)|((d>>3)&8);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper140 final : public Mapper {public:explicit Mapper140(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_prg=(d>>4)&3;m_chr=d&0x0F;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper184 final : public Mapper {public:explicit Mapper184(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=(a-0x8000)%m_config.prgRomSize;return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_chr[0]=d&7;m_chr[1]=0x80|((d>>4)&7);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>12],0x1000,s,a&0xFFF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_chr[0]);put8(o,m_chr[1]);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_chr[0])&&get8(p,e,m_chr[1]);}private:uint8_t m_chr[2]={};};

class Mapper206 final : public Mapper {public:explicit Mapper206(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE001)==0x8000){m_sel=d&7;return true;}if((a&0xE001)==0x8001){if(m_sel<=1)m_chr2[m_sel]=d>>1;else if(m_sel<=5)m_chr1[m_sel-2]=d;else m_prg[m_sel-6]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x1000)m=mapBank(m_chr2[a>>11],0x800,s,a&0x7FF);else m=mapBank(m_chr1[(a-0x1000)>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr2)put8(o,v);for(auto v:m_chr1)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr2)if(!get8(p,e,v))return false;for(auto&v:m_chr1)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_prg[2]={},m_chr2[2]={},m_chr1[4]={};};

class Mapper232 final : public Mapper {public:explicit Mapper232(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t b=(size_t(m_block)<<2)|(a<0xC000?m_page:3);m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(a>=0xC000)m_page=d&3;else m_block=m_config.submapper==1?(((d>>4)&1)|((d>>2)&2)):((d>>3)&3);return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_block);put8(o,m_page);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_block)&&get8(p,e,m_page);}private:uint8_t m_block=0,m_page=0;};

class Mapper240 final : public Mapper {public:explicit Mapper240(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x4020||a>0x5FFF)return false;m_prg=(d>>4)&0x0F;m_chr=d&0x0F;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};
class Mapper241 final : public Mapper {public:explicit Mapper241(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d;return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);}private:uint8_t m_prg=0;};
class Mapper242 final : public Mapper {public:explicit Mapper242(const MapperConfig&c):Mapper(c){m_mirror=Mirror::Vertical;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t,uint64_t)override{if(a<0x8000)return false;m_prg=(a>>3)&0x0F;m_mirror=(a&2)?Mirror::Horizontal:Mirror::Vertical;return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0;};
class Mapper246 final : public Mapper {public:explicit Mapper246(const MapperConfig&c):Mapper(c){m_prg[3]=0xFF;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg[(a-0x8000)>>13],0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x67FF)return false;uint8_t r=a&7;if(r<=3)m_prg[r]=d;else m_chr[r&3]=d;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;return true;}private:uint8_t m_prg[4]={0,0,0,0xFF},m_chr[4]={};};
