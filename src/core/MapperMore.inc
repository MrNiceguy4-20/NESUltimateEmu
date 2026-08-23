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
    void resetAudio(){
        std::fill(std::begin(regs), std::end(regs), uint8_t(0));
        for(auto& c : ch) c = Channel{};
        divider = 0;
        output = 0;
        m_accum = 0.0;
        // The VRC7 reset input clears the programmable audio state. Keep the
        // external-clock phase accumulator running: the real VRC7 oscillator
        // is independent of the NES CPU and is not phase-locked to $E000.
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
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t b,l,h;if(!get8(p,e,b))return false;type02=b;if(!get8(p,e,l)||!get8(p,e,h))return false;uint16_t n=uint16_t(l)|(uint16_t(h)<<8);if(n!=size||size_t(e-p) < static_cast<size_t>(9u + size))return false;if(!get8(p,e,b))return false;mode=Mode(b);if(!get8(p,e,b))return false;next=Mode(b);if(!get8(p,e,chip)||!get8(p,e,address)||!get8(p,e,data)||!get8(p,e,counter)||!get8(p,e,output)||!get8(p,e,prevScl)||!get8(p,e,prevSda))return false;std::memcpy(mem,p,size);p+=size;return true;}
};
}


class Mapper15 final : public Mapper {
public:
    explicit Mapper15(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const uint8_t cpuA13 = (a >> 13) & 1;
        uint8_t bank8 = 0;
        switch (m_mode & 3) {
        case 0: // NROM-256: CPU A14,A13 pass through; latched A14 is ignored.
            bank8 = uint8_t(((m_prg & 0x3E) << 1) | ((a >> 13) & 3));
            break;
        case 1: // UNROM: upper 16 KiB forces PRG A16..A14 high.
            if (a < 0xC000) bank8 = uint8_t((m_prg << 1) | cpuA13);
            else bank8 = uint8_t((((m_prg & 0x38) | 0x07) << 1) | cpuA13);
            break;
        case 2: // NROM-64: one 8 KiB bank mirrored four times.
            bank8 = uint8_t((m_prg << 1) | m_prgA13);
            break;
        default: // NROM-128: one 16 KiB bank mirrored twice.
            bank8 = uint8_t((m_prg << 1) | cpuA13);
            break;
        }
        m = mapBank(bank8, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a < 0x8000) return false;
        m_mode = a & 3;
        m_prg = d & 0x3F;
        m_prgA13 = (d >> 7) & 1;
        if (!m_config.fourScreen) m_mirror = (d & 0x40) ? Mirror::Horizontal : Mirror::Vertical;
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        if ((m_mode == 0 || m_mode == 3) || m_config.chrRamSize == 0) return false;
        return Mapper::ppuMapWrite(a, m);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_prg); put8(o, m_prgA13); put8(o, m_mode); put8(o, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t b=0;
        if (!get8(p,e,m_prg)||!get8(p,e,m_prgA13)||!get8(p,e,m_mode)||!get8(p,e,b)) return false;
        m_mirror=static_cast<Mirror>(b); return true;
    }
private:
    uint8_t m_prg=0, m_prgA13=0, m_mode=0;
};

class Mapper31 final : public Mapper {
public:
    explicit Mapper31(const MapperConfig& c) : Mapper(c) { m_bank[7] = 0xFF; }
    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const uint8_t slot = uint8_t((a - 0x8000) >> 12);
        m = mapBank(m_bank[slot], 0x1000, m_config.prgRomSize, a & 0x0FFF);
        return true;
    }
    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a < 0x5000 || a > 0x5FFF) return false;
        m_bank[a & 7] = d; return true;
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    void saveState(std::vector<uint8_t>& o) const override { for (auto v:m_bank) put8(o,v); }
    bool loadState(const uint8_t*& p,const uint8_t* e) override { for(auto&v:m_bank) if(!get8(p,e,v)) return false; return true; }
private:
    uint8_t m_bank[8] = {};
};

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

class Mapper153 final : public Mapper {
public:
    explicit Mapper153(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const size_t outer = size_t(m_outer[m_ppuOuterSlot] & 1) << 4; // 16 x 16 KiB = 256 KiB
        const size_t bank = a < 0xC000 ? (size_t(m_prg & 0x0F) | outer) : (size_t(0x0F) | outer);
        m = mapBank(bank, 0x4000, m_config.prgRomSize, a & 0x3FFF);
        return true;
    }

    bool mapPrgRam(uint16_t a, uint32_t& m, bool) const override {
        if (a < 0x6000 || a > 0x7FFF || !m_config.prgRamSize || !m_ramEnable) return false;
        m = (a - 0x6000) % m_config.prgRamSize;
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a < 0x8000) return false;
        const uint8_t r = uint8_t(a & 0x0F); // LZ93D50 mask $800F
        if (r < 4) {
            m_outer[r] = d & 1;
            return true;
        }
        if (r >= 4 && r < 8) return true; // ASIC registers exist, but PA12/PA13 are grounded on this board.
        if (r == 8) { m_prg = d & 0x0F; return true; }
        if (r == 9) { mapper_more_detail::setMirror4(m_mirror, d); return true; }
        if (r == 0xA) {
            m_irqEnabled = (d & 1) != 0;
            m_irqPending = false;
            m_irqCounter = m_irqLatch;
            return true;
        }
        if (r == 0xB) { m_irqLatch = uint16_t((m_irqLatch & 0xFF00) | d); return true; }
        if (r == 0xC) { m_irqLatch = uint16_t((m_irqLatch & 0x00FF) | (uint16_t(d) << 8)); return true; }
        if (r == 0xD) { m_ramEnable = (d & 0x20) != 0; return true; }
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000 || !m_config.chrRamSize) return false;
        m_ppuOuterSlot = uint8_t((a >> 10) & 3);
        m = a % m_config.chrRamSize; // 8 KiB unbanked CHR-RAM
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return ppuMapRead(a, m);
    }

    void clockCpu() override {
        if (!m_irqEnabled) return;
        if (m_irqCounter == 0) m_irqPending = true;
        --m_irqCounter;
    }

    bool irqActive() const override { return m_irqPending; }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_prg);
        for (auto v : m_outer) put8(o, v);
        put8(o, m_ppuOuterSlot);
        put8(o, uint8_t(m_irqLatch)); put8(o, uint8_t(m_irqLatch >> 8));
        put8(o, uint8_t(m_irqCounter)); put8(o, uint8_t(m_irqCounter >> 8));
        put8(o, m_irqEnabled); put8(o, m_irqPending); put8(o, m_ramEnable);
        put8(o, static_cast<uint8_t>(m_mirror));
    }

    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t lo, hi, b;
        if (!get8(p, e, m_prg)) return false;
        for (auto& v : m_outer) if (!get8(p, e, v)) return false;
        if (!get8(p, e, m_ppuOuterSlot) || m_ppuOuterSlot > 3) return false;
        if (!get8(p, e, lo) || !get8(p, e, hi)) return false;
        m_irqLatch = uint16_t(lo) | (uint16_t(hi) << 8);
        if (!get8(p, e, lo) || !get8(p, e, hi)) return false;
        m_irqCounter = uint16_t(lo) | (uint16_t(hi) << 8);
        if (!get8(p, e, b)) return false;
        m_irqEnabled = b != 0;
        if (!get8(p, e, b)) return false;
        m_irqPending = b != 0;
        if (!get8(p, e, b)) return false;
        m_ramEnable = b != 0;
        if (!get8(p, e, b)) return false;
        m_mirror = static_cast<Mirror>(b);
        return true;
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_outer[4] = {};
    uint8_t m_ppuOuterSlot = 0;
    uint16_t m_irqLatch = 0, m_irqCounter = 0;
    bool m_irqEnabled = false, m_irqPending = false, m_ramEnable = false;
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

        // VRC6 nametable register selection is controlled by B003 bits 0-2,
        // independently of whether the backing store is CIRAM or CHR-ROM.
        // The ASIC always drives CIRAM A10 from its CHR A10 output, so the
        // same low-bank-bit rules apply to both kinds of nametable backing.
        const uint8_t style=m_ppuCtrl&0x07;
        uint8_t reg=6;
        switch(style){
        case 0: case 6: case 7:
            reg=uint8_t(page<2?6:7);             // R6,R6,R7,R7
            break;
        case 1: case 5:
            reg=uint8_t(4+page);                 // R4,R5,R6,R7
            break;
        default: // 2,3,4
            reg=uint8_t(6+(page&1));             // R6,R7,R6,R7
            break;
        }

        uint8_t bank=m_chr[reg];

        // With B003.5 set, the existing VRC6 PCB enables the ASIC's CHR-A10
        // override network.  Modes 0 and 3 force the low bank bit to produce
        // the documented 2 KiB spread / H / V / one-screen arrangements.
        // Modes 1 and 2 leave each selected register's LSB intact.
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
            // ROM nametables use exactly the CHR bank selected above.
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
        // Keep the stock pulse gain unchanged when Chip Mod is enabled. The
        // smooth KYLXBN saw may legitimately exceed the stock staircase's
        // peak; final APU output headroom handles that rather than clipping the
        // expansion chip here.
        return -(raw / 61.0f) * 0.55f;
    }
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg16);put8(o,m_prg8);for(auto v:m_chr)put8(o,v);m_irq.save(o);put8(o,static_cast<uint8_t>(m_mirror));put8(o,m_ppuCtrl);put8(o,m_audioCtrl);for(auto&i:m_pulse)i.save(o);m_saw.save(o);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg16)||!get8(p,e,m_prg8))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!m_irq.load(p,e)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);if(!get8(p,e,m_ppuCtrl)||!get8(p,e,m_audioCtrl))return false;for(auto&i:m_pulse)if(!i.load(p,e))return false;return m_saw.load(p,e);}
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

    bool m_swap;uint8_t m_prg16=0,m_prg8=0,m_chr[8]={},m_ppuCtrl=0,m_audioCtrl=0;Pulse m_pulse[2];Saw m_saw;mapper_more_detail::VrcIrq m_irq;
    uint8_t audioShift() const { return (m_audioCtrl & 4) ? 8 : ((m_audioCtrl & 2) ? 4 : 0); }
    uint16_t translate(uint16_t a)const{if(!m_swap)return a;uint16_t lo=a&3;lo=((lo&1)<<1)|((lo&2)>>1);return uint16_t((a&~3)|lo);}
};


class Mapper116 final : public Mapper {
public:
    explicit Mapper116(const MapperConfig& c) : Mapper(c) {
        MapperConfig mmc1c = c;
        // Keep id 116 on the synthetic MMC1 so Huang-2 can apply its altered
        // PRG-bank wiring inside the shared MMC1 implementation.
        m_mmc1 = std::make_unique<Mapper1>(mmc1c);

        MapperConfig mmc3c = c;
        mmc3c.id = 4;
        mmc3c.submapper = 0;
        m_mmc3 = std::make_unique<Mapper4>(mmc3c);

        MapperConfig vrc2c = c;
        vrc2c.id = 23;
        vrc2c.submapper = 1; // VRC2b, A0/A1 decode.
        vrc2c.prgRamSize = 0;
        vrc2c.headerPrgNvRamSize = 0;
        m_vrc2 = std::make_unique<MapperVrc24>(vrc2c);

        // Huang-1/Huang-2 power up in MMC3 mode. The VRC2 CHR registers power
        // up to $FF; at least one Somari revision relies on that state.
        m_supervisor = 0x01;
        for (uint16_t page = 0xB000; page <= 0xE000; page += 0x1000) {
            m_vrc2->cpuWrite(page + 0, 0x0F, kNoCpuCycle);
            m_vrc2->cpuWrite(page + 1, 0x0F, kNoCpuCycle);
            m_vrc2->cpuWrite(page + 2, 0x0F, kNoCpuCycle);
            m_vrc2->cpuWrite(page + 3, 0x0F, kNoCpuCycle);
        }
        syncMirror();
    }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (!active()->cpuMapRead(a, m)) return false;
        applyResetPrgWindow(m);
        return true;
    }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        return active()->cpuReadRegister(a, d);
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t cycle) override {
        if ((a & 0xE100) == 0x4100) {
            const uint8_t oldMode = mode();
            m_supervisor = d & 0x07;
            const uint8_t newMode = mode();
            // Huang-1 -W resets only the MMC1 serial latch when entering MMC1;
            // all other MMC1 registers remain intact. -P (submapper 1) does not.
            if (m_config.submapper == 0 && oldMode != 2 && newMode == 2)
                m_mmc1->cpuWrite(0x8000, 0x80, cycle);
            syncMirror();
            return true;
        }
        const bool handled = active()->cpuWrite(a, d, cycle);
        if (handled) syncMirror();
        return handled;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (!active()->ppuMapRead(a, m)) return false;
        applyOuterChr(m);
        return true;
    }
    bool ppuMapReadEx(uint16_t a, uint32_t& m, PpuFetchKind kind) override {
        if (!active()->ppuMapReadEx(a, m, kind)) return false;
        applyOuterChr(m);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        if (!active()->ppuMapWrite(a, m)) return false;
        applyOuterChr(m);
        return true;
    }
    bool ppuUsesChrRam(uint16_t a) const override { return active()->ppuUsesChrRam(a); }

    bool mapPrgRam(uint16_t a, uint32_t& m, bool w) const override {
        // The board disconnects WRAM while the VRC2 personality is selected.
        if (mode() == 0) return false;
        return active()->mapPrgRam(a, m, w);
    }

    void notifyPpuAddress(uint16_t a, uint64_t c) override { active()->notifyPpuAddress(a, c); }
    void notifyPpuAddressContext(uint16_t a, uint64_t c, int s, int d) override {
        active()->notifyPpuAddressContext(a, c, s, d);
    }
    void notifyPpuScanline(int s, bool r) override { active()->notifyPpuScanline(s, r); }
    void clockCpu() override { active()->clockCpu(); }
    void scanlineTick() override { active()->scanlineTick(); }
    void reset(bool hard) override {
        if (!(m_config.nes20 && m_config.submapper == 3)) return;
        // Mario 5-in-1 uses reset as an outer game selector. A cold power-on
        // returns to game 0; each console Reset advances through the remaining
        // four games, then wraps back to the first.
        m_resetGame = hard ? 0 : uint8_t((m_resetGame + 1) % 5);
    }
    bool irqActive() const override { return active()->irqActive(); }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_supervisor);
        put8(o, m_resetGame);
        saveChild(o, *m_vrc2);
        saveChild(o, *m_mmc3);
        saveChild(o, *m_mmc1);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t supervisor = 0, resetGame = 0;
        if (!get8(p, e, supervisor) || !get8(p, e, resetGame)) return false;
        if (!loadChild(p, e, *m_vrc2) || !loadChild(p, e, *m_mmc3) || !loadChild(p, e, *m_mmc1))
            return false;
        m_supervisor = supervisor & 0x07;
        m_resetGame = uint8_t(resetGame % 5);
        syncMirror();
        return true;
    }

    bool implementationSupported() const override {
        return !m_config.nes20 || m_config.submapper <= 3;
    }

private:
    std::unique_ptr<Mapper> m_vrc2, m_mmc3, m_mmc1;
    uint8_t m_supervisor = 0x01;
    uint8_t m_resetGame = 0;

    uint8_t mode() const {
        const uint8_t m = m_supervisor & 3;
        return m >= 2 ? 2 : m;
    }
    Mapper* active() {
        switch (mode()) { case 0: return m_vrc2.get(); case 1: return m_mmc3.get(); default: return m_mmc1.get(); }
    }
    const Mapper* active() const {
        switch (mode()) { case 0: return m_vrc2.get(); case 1: return m_mmc3.get(); default: return m_mmc1.get(); }
    }
    void syncMirror() { m_mirror = active()->mirroring(); }
    void applyOuterChr(uint32_t& mapped) const {
        if ((m_supervisor & 0x04) && m_config.chrRomSize > 0x40000)
            mapped = uint32_t((mapped & 0x3FFFFu) | 0x40000u) % uint32_t(m_config.chrRomSize);
        applyResetChrWindow(mapped);
    }
    void applyResetPrgWindow(uint32_t& mapped) const {
        if (!(m_config.nes20 && m_config.submapper == 3) || !m_config.prgRomSize) return;
        // NESdev documents one 256 KiB game followed by four 128 KiB games.
        // The reset selector therefore constrains the simulated mapper to the
        // currently selected physical ROM window.
        const uint32_t base = m_resetGame == 0 ? 0u : 0x40000u + uint32_t(m_resetGame - 1) * 0x20000u;
        const uint32_t mask = m_resetGame == 0 ? 0x3FFFFu : 0x1FFFFu;
        mapped = (base + (mapped & mask)) % uint32_t(m_config.prgRomSize);
    }
    void applyResetChrWindow(uint32_t& mapped) const {
        if (!(m_config.nes20 && m_config.submapper == 3) || !m_config.chrRomSize) return;
        const uint32_t base = m_resetGame == 0 ? 0u : 0x40000u + uint32_t(m_resetGame - 1) * 0x20000u;
        const uint32_t mask = m_resetGame == 0 ? 0x3FFFFu : 0x1FFFFu;
        mapped = (base + (mapped & mask)) % uint32_t(m_config.chrRomSize);
    }
    static void saveChild(std::vector<uint8_t>& o, const Mapper& child) {
        std::vector<uint8_t> tmp;
        child.saveState(tmp);
        put64(o, static_cast<uint64_t>(tmp.size()));
        o.insert(o.end(), tmp.begin(), tmp.end());
    }
    static bool loadChild(const uint8_t*& p, const uint8_t* e, Mapper& child) {
        uint64_t n = 0;
        if (!get64(p, e, n) || n > static_cast<uint64_t>(e - p)) return false;
        const uint8_t* q = p;
        const uint8_t* qe = p + static_cast<std::ptrdiff_t>(n);
        if (!child.loadState(q, qe) || q != qe) return false;
        p = qe;
        return true;
    }
};

class Mapper32 final : public Mapper {public:explicit Mapper32(const MapperConfig&c):Mapper(c){if(c.nes20&&c.submapper==1){m_majorLeague=true;m_mirror=Mirror::OnescreenHi;}}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000),last=n-1,last2=n>1?n-2:0;size_t b=(!m_mode||m_majorLeague)?(a<0xA000?m_p0:a<0xC000?m_p1:a<0xE000?last2:last):(a<0xA000?last2:a<0xC000?m_p1:a<0xE000?m_p0:last);m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;switch(a&0xF000){case 0x8000:m_p0=d;return true;case 0x9000:if(!m_majorLeague){m_mode=(d>>1)&1;if(!m_config.fourScreen)m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;}return true;case 0xA000:m_p1=d;return true;case 0xB000:m_chr[a&7]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_p0);put8(o,m_p1);put8(o,m_mode);for(auto v:m_chr)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_p0)||!get8(p,e,m_p1)||!get8(p,e,m_mode))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=m_majorLeague?Mirror::OnescreenHi:static_cast<Mirror>(b);if(m_majorLeague)m_mode=0;return true;}private:uint8_t m_p0=0,m_p1=0,m_mode=0,m_chr[8]={};bool m_majorLeague=false;};

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
public:
    explicit Mapper68(const MapperConfig& c)
        : Mapper(c), m_dualCart(c.nes20 && c.submapper == 1) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;

        if (a < 0xC000) {
            if (m_dualCart && m_external) {
                // NTB-ROM external option cartridge. A disabled/expired
                // licensing timer disconnects the option ROM from the bus.
                if (m_timer == 0) return false;

                // NES 2.0 describes the base NTB-ROM as 128 KiB internal PRG.
                // If a dump/tool concatenates external option-ROM bytes after
                // that image, expose them here. If no external bytes are
                // present, the socket is effectively empty/open bus.
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

        // Standard Sunsoft-4 fixes the upper 16 KiB to the final PRG bank.
        // NTB-ROM fixes it to the final bank of its 128 KiB internal ROM.
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
            // On NTB-ROM the licensing IC is active only while WRAM is
            // disabled and the external socket is selected. Any write resets
            // its approximately 107,520-M2-cycle timeout.
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
                // Every IRQ-control write acknowledges a pending FME-7 IRQ,
                // independently of the new enable state. Bus::clock() advances
                // the mapper before the CPU performs its bus access, so a 0->1
                // counter-enable write must also account for the counter edge in
                // this CPU period; otherwise the IRQ is one CPU poll late.
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

    void clockIrqCounter() {
        if (!m_counterEnable) return;

        // Keep 0000 as a real counter state for one mapper clock so the next
        // tick naturally wraps to FFFF. The externally visible IRQ edge is
        // asserted when the decrement reaches zero; this matches the CPU poll
        // phase while retaining the FME-7's 65536-cycle free-running period.
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

class Mapper70 final : public Mapper {public:explicit Mapper70(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=(d>>4)&7;m_chr=d&0x0F;if(m_config.id==152)m_mirror=(d&0x80)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool hasBusConflicts()const override{return m_config.id==152;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper72 final : public Mapper {public:explicit Mapper72(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(m_config.id==92){if((d&0xF0)==0xD0)m_prg=d&0x0F;else if((d&0xF0)==0xE0)m_chr=d&0x0F;}else{if(d&0x80)m_prg=d&0x0F;if(d&0x40)m_chr=d&0x0F;}return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper73 final : public Mapper {public:explicit Mapper73(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF000){case 0x8000:m_latch=(m_latch&0xFFF0)|(d&0xF);return true;case 0x9000:m_latch=(m_latch&0xFF0F)|((d&0xF)<<4);return true;case 0xA000:m_latch=(m_latch&0xF0FF)|((d&0xF)<<8);return true;case 0xB000:m_latch=(m_latch&0x0FFF)|((d&0xF)<<12);return true;case 0xC000:m_ack=d&1;m_enabled=d&2;m_mode8=d&4;m_pending=false;if(m_enabled)m_counter=m_latch;return true;case 0xD000:m_pending=false;m_enabled=m_ack;return true;case 0xF000:m_prg=d&7;return true;}return false;}void clockCpu()override{if(!m_enabled)return;if(m_mode8){uint8_t v=uint8_t(m_counter);if(v==0xFF){m_counter=(m_counter&0xFF00)|(m_latch&0xFF);m_pending=true;}else m_counter=(m_counter&0xFF00)|uint8_t(v+1);}else{if(m_counter==0xFFFF){m_counter=m_latch;m_pending=true;}else ++m_counter;}}bool irqActive()const override{return m_pending;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);for(uint16_t v:{m_latch,m_counter}){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}put8(o,m_enabled);put8(o,m_ack);put8(o,m_mode8);put8(o,m_pending);}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t l,h,b;if(!get8(p,e,m_prg)||!get8(p,e,l)||!get8(p,e,h))return false;m_latch=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,l)||!get8(p,e,h))return false;m_counter=uint16_t(l)|(uint16_t(h)<<8);if(!get8(p,e,b))return false;m_enabled=b;if(!get8(p,e,b))return false;m_ack=b;if(!get8(p,e,b))return false;m_mode8=b;if(!get8(p,e,b))return false;m_pending=b;return true;}private:uint8_t m_prg=0;uint16_t m_latch=0,m_counter=0;bool m_enabled=false,m_ack=false,m_mode8=false,m_pending=false;};

class Mapper75 final : public Mapper {public:explicit Mapper75(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xF000){case 0x8000:m_prg[0]=d&0x0F;return true;case 0x9000:m_chr[0]=uint8_t((m_chr[0]&0x0F)|((d&2)<<3));m_chr[1]=uint8_t((m_chr[1]&0x0F)|((d&4)<<2));if(!m_config.fourScreen)m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;return true;case 0xA000:m_prg[1]=d&0x0F;return true;case 0xC000:m_prg[2]=d&0x0F;return true;case 0xE000:m_chr[0]=uint8_t((m_chr[0]&0x10)|(d&0x0F));return true;case 0xF000:m_chr[1]=uint8_t((m_chr[1]&0x10)|(d&0x0F));return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>12],0x1000,s,a&0xFFF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg[3]={},m_chr[2]={};};

class Mapper76 final : public Mapper {public:explicit Mapper76(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE001)==0x8000){m_sel=d&7;return true;}if((a&0xE001)==0x8001){if(m_sel>=2&&m_sel<=5)m_chr[m_sel-2]=d;else if(m_sel>=6)m_prg[m_sel-6]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_prg[2]={},m_chr[4]={};};

class Mapper77 final : public Mapper {public:explicit Mapper77(const MapperConfig&c):Mapper(c){m_mirror=Mirror::FourScreen;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&7;m_chr=d>>4;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;if(a<0x0800){if(!m_config.chrRomSize)return false;m=mapBank(m_chr,0x800,m_config.chrRomSize,a);return true;}if(!m_config.chrRamSize)return false;m=a-0x0800;return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{if(a<0x0800||a>=0x2000||!m_config.chrRamSize)return false;m=a-0x0800;return true;}bool ppuUsesChrRam(uint16_t a)const override{return a>=0x0800&&a<0x2000&&m_config.chrRamSize;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

// Mapper 78 combines UNROM-style 16 KiB PRG banking with CNROM-style 8 KiB
// CHR banking. Two incompatible boards share the mapper number: Uchuusen
// (NES 2.0 submapper 1) uses AxROM-like one-screen mirroring, while Holy
// Diver (submapper 3) switches horizontal/vertical mirroring. Legacy images
// commonly use the alternative-nametables flag to identify Holy Diver.
class Mapper78 final : public Mapper {
public:
    explicit Mapper78(const MapperConfig& c)
        : Mapper(c), m_holyDiver(c.submapper == 3 || (c.submapper == 0 && c.fourScreen)) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || m_config.prgRomSize < 0x4000) return false;
        if (a < 0xC000) m = mapBank(m_prg, 0x4000, m_config.prgRomSize, a - 0x8000);
        else m = static_cast<uint32_t>(m_config.prgRomSize - 0x4000 + (a - 0xC000));
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a < 0x8000) return false;
        m_prg = d & 7;
        m_chr = (d >> 4) & 0x0F;
        if (m_holyDiver) m_mirror = (d & 8) ? Mirror::Vertical : Mirror::Horizontal;
        else m_mirror = (d & 8) ? Mirror::OnescreenHi : Mirror::OnescreenLo;
        return true;
    }

    bool hasBusConflicts() const override { return true; }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        auto s = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!s) return false;
        m = mapBank(m_chr, 0x2000, s, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a, m); }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_prg); put8(o, m_chr); put8(o, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t b;
        if (!get8(p,e,m_prg) || !get8(p,e,m_chr) || !get8(p,e,b)) return false;
        m_mirror = static_cast<Mirror>(b);
        return true;
    }

private:
    uint8_t m_prg = 0, m_chr = 0;
    bool m_holyDiver = false;
};

class Mapper79 final : public Mapper {public:explicit Mapper79(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE100)!=0x4100)return false;m_prg=(d>>3)&1;m_chr=d&7;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

// iNES Mapper 148 is the NINA-06/GNROM-style latch moved to the normal
// cartridge ROM write region.  D3 selects the 32 KiB PRG bank and D2-D0
// select the 8 KiB CHR bank.  Because the latch overlaps PRG-ROM output,
// writes are subject to wired-AND bus conflicts.
// iNES Mapper 147 / Sachen TCU-01.  The low cartridge register at
// $4102 (mirrored by mask $E103) latches both banks: PRG bank bits are
// data D7,D2 and the 8 KiB CHR bank is data D7-D3.  Mirroring is fixed by
// the cartridge header and the board has no PRG RAM.
class Mapper147 final : public Mapper {
public:
    explicit Mapper147(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        mapped = mapBank(m_prgBank, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if ((addr & 0xE103) != 0x4102) return false;
        m_prgBank = uint8_t(((data >> 6) & 0x02) | ((data >> 2) & 0x01));
        m_chrBank = uint8_t(data >> 3);
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        mapped = mapBank(m_chrBank, 0x2000, size, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void reset(bool hard) override {
        if (!hard) return;
        m_prgBank = 0;
        m_chrBank = 0;
        m_mirror = m_config.headerMirror;
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_prgBank);
        put8(out, m_chrBank);
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        return get8(p, end, m_prgBank) && get8(p, end, m_chrBank);
    }

private:
    uint8_t m_prgBank = 0;
    uint8_t m_chrBank = 0;
};

class Mapper148 final : public Mapper {
public:
    explicit Mapper148(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        mapped = mapBank(m_prg, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr < 0x8000) return false;
        m_prg = (data >> 3) & 1;
        m_chr = data & 7;
        return true;
    }

    bool hasBusConflicts() const override { return true; }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const auto size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        mapped = mapBank(m_chr, 0x2000, size, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_prg); put8(out, m_chr);
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        return get8(p, end, m_prg) && get8(p, end, m_chr);
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};

// iNES Mapper 149 / Sachen SA-0036 (Taiwan Mahjong 16).  PRG-ROM is
// fixed; writes in $8000-$FFFF overlap PRG-ROM and are therefore subject to
// wired-AND bus conflicts.  The conflicted value's D7 selects one of two
// 8 KiB CHR-ROM banks.
class Mapper149 final : public Mapper {
public:
    explicit Mapper149(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        mapped = uint32_t(addr - 0x8000) % uint32_t(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr < 0x8000) return false;
        m_chr = uint8_t((data >> 7) & 1);
        return true;
    }

    bool hasBusConflicts() const override { return true; }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const auto size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        mapped = mapBank(m_chr, 0x2000, size, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_chr); }

private:
    uint8_t m_chr = 0;
};

// iNES Mapper 150 / Sachen SA-015 (74LS374N).  An 8-register ASIC is
// selected through $4100 and read/written through $4101 (both under mask
// $C101). R5 selects the 32 KiB PRG bank, R4/R6 compose the 8 KiB CHR
// bank, and R7 selects one of four nametable arrangements.  Register reads
// drive only D0-D2; D3-D7 remain CPU open bus.
class Mapper150 final : public Mapper {
public:
    explicit Mapper150(const MapperConfig& c) : Mapper(c) { updateMirror(); }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        mapped = mapBank(uint8_t(m_reg[5] & 3), 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuReadRegister(uint16_t addr, uint8_t& data) override {
        if ((addr & 0xC101) != 0x4101) return false;
        data = uint8_t((data & 0xF8) | (m_reg[m_select] & 7));
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        const uint16_t reg = uint16_t(addr & 0xC101);
        if (reg == 0x4100) {
            m_select = uint8_t(data & 7);
            return true;
        }
        if (reg == 0x4101) {
            m_reg[m_select] = uint8_t(data & 7);
            if (m_select == 7) updateMirror();
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        const uint8_t bank = uint8_t((m_reg[6] & 3) | ((m_reg[4] & 1) << 2));
        mapped = mapBank(bank, 0x2000, size, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    bool mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const override {
        if (addr < 0x2000 || addr > 0x3EFF) return false;
        const uint8_t page = uint8_t((addr & 0x0FFF) >> 10);
        uint8_t nt = 0;
        switch ((m_reg[7] >> 1) & 3) {
        case 0: nt = uint8_t(page == 3 ? 1 : 0); break; // S0,S0,S0,S1 (lower-right unique)
        case 1: nt = uint8_t((page >> 1) & 1); break;   // horizontal
        case 2: nt = uint8_t(page & 1); break;          // vertical
        default: nt = 1; break;                         // one-screen, page 1
        }
        source = NametableSource::Ciram;
        mapped = uint32_t(nt) * 0x400u + (addr & 0x3FFu);
        return true;
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_select = 0;
        std::fill(std::begin(m_reg), std::end(m_reg), uint8_t(0));
        updateMirror();
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_select);
        for (auto v : m_reg) put8(out, v);
        put8(out, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        uint8_t mirror = 0;
        if (!get8(p, end, m_select)) return false;
        for (auto& v : m_reg) if (!get8(p, end, v)) return false;
        if (!get8(p, end, mirror)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    void updateMirror() {
        switch ((m_reg[7] >> 1) & 3) {
        case 1: m_mirror = Mirror::Horizontal; break;
        case 2: m_mirror = Mirror::Vertical; break;
        case 3: m_mirror = Mirror::OnescreenHi; break;
        default: m_mirror = Mirror::OnescreenLo; break; // custom layout via mapNametable()
        }
    }

    uint8_t m_select = 0;
    uint8_t m_reg[8] = {};
};


// iNES Mapper 243 / Sachen SA-020A (74LS374NA).  This uses the same
// eight-register ASIC interface as Mapper 150, but the PCB wires the CHR
// address lines differently: R2->CHR A13, R4->A14 and R6.1-0->A16-A15.
// All three data bits in every internal register are readable.
class Mapper243 final : public Mapper {
public:
    explicit Mapper243(const MapperConfig& c) : Mapper(c) { updateMirror(); }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        mapped = mapBank(uint8_t(m_reg[5] & 3), 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuReadRegister(uint16_t addr, uint8_t& data) override {
        if ((addr & 0xC101) != 0x4101) return false;
        data = uint8_t((data & 0xF8) | (m_reg[m_select] & 7));
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        const uint16_t reg = uint16_t(addr & 0xC101);
        if (reg == 0x4100) {
            m_select = uint8_t(data & 7);
            return true;
        }
        if (reg == 0x4101) {
            m_reg[m_select] = uint8_t(data & 7);
            if (m_select == 7) updateMirror();
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        const uint8_t bank = uint8_t((m_reg[2] & 1) | ((m_reg[4] & 1) << 1) | ((m_reg[6] & 3) << 2));
        mapped = mapBank(bank, 0x2000, size, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    bool mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const override {
        if (addr < 0x2000 || addr > 0x3EFF) return false;
        const uint8_t page = uint8_t((addr & 0x0FFF) >> 10);
        uint8_t nt = 0;
        switch ((m_reg[7] >> 1) & 3) {
        case 0: nt = uint8_t(page == 3 ? 1 : 0); break;
        case 1: nt = uint8_t((page >> 1) & 1); break;
        case 2: nt = uint8_t(page & 1); break;
        default: nt = 1; break;
        }
        source = NametableSource::Ciram;
        mapped = uint32_t(nt) * 0x400u + (addr & 0x3FFu);
        return true;
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_select = 0;
        std::fill(std::begin(m_reg), std::end(m_reg), uint8_t(0));
        updateMirror();
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_select);
        for (auto v : m_reg) put8(out, v);
        put8(out, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        uint8_t mirror = 0;
        if (!get8(p, end, m_select)) return false;
        for (auto& v : m_reg) if (!get8(p, end, v)) return false;
        if (!get8(p, end, mirror)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    void updateMirror() {
        switch ((m_reg[7] >> 1) & 3) {
        case 1: m_mirror = Mirror::Horizontal; break;
        case 2: m_mirror = Mirror::Vertical; break;
        case 3: m_mirror = Mirror::OnescreenHi; break;
        default: m_mirror = Mirror::OnescreenLo; break;
        }
    }

    uint8_t m_select = 0;
    uint8_t m_reg[8] = {};
};

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
        // NES 2.0 distinguishes the two physical VRC7 address-line wirings.
        // VRC7b (submapper 1) connects ASIC An to CPU A3; CPU A4 therefore
        // aliases the same mapper register instead of making the write invalid.
        // VRC7a (submapper 2) connects An to CPU A4; CPU A3 is similarly
        // ignored.  Canonicalize both variants to the A3-shaped register map
        // used by the switch below.  The dedicated OPLL ports keep their
        // A4/A5 decode on both chips and must not be canonicalized here.
        const uint16_t audioDecode = addr & 0xF030;
        const bool audioPort = audioDecode == 0x9010 || audioDecode == 0x9030;
        if (!audioPort && m_config.nes20 && m_config.submapper == 1) {
            addr &= static_cast<uint16_t>(~0x10);
        } else if (!audioPort && m_config.nes20 && m_config.submapper == 2) {
            const bool an = (addr & 0x10) != 0;
            addr &= static_cast<uint16_t>(~0x18);
            if (an) addr |= 0x08;
        } else if (!audioPort && (addr & 0x10)) {
            // Legacy mapper 85 does not identify whether An is A3 or A4.
            // Preserve the historical permissive behavior by accepting A4
            // as an alias for the canonical A3 register-select bit.
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
            if (data & 0x40) {
                // VRC7 audio reset is stateful, not merely a mixer mute: it
                // clears the internal audio registers/channels, and writes to
                // $9010/$9030 remain ignored until reset is released.
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



// Mapper 38 (UNL-PCI556): GNROM-style latch at $7000-$7FFF.
class Mapper38 final : public Mapper { public: explicit Mapper38(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x7000||a>0x7FFF)return false;m_prg=d&3;m_chr=(d>>2)&3;return true;} bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_chr,0x2000,z,a);return true;} bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);} bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;} void reset(bool)override{m_prg=m_chr=0;} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);} bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);} private:uint8_t m_prg=0,m_chr=0;};

class Mapper39 final : public Mapper { public: explicit Mapper39(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d;return true;} void reset(bool)override{m_prg=0;} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);} bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);} private:uint8_t m_prg=0;};

class Mapper50 final : public Mapper { public: explicit Mapper50(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x6000||!m_config.prgRomSize)return false;size_t b=a<0x8000?0x0F:a<0xA000?8:a<0xC000?9:a<0xE000?m_bank:0x0B;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0x4120){case 0x4020:m_bank=(d&8)|((d&1)<<2)|((d&6)>>1);return true;case 0x4120:if(d&1)m_irqEnable=true;else{m_irqEnable=false;m_irq=false;m_irqCounter=0;}return true;default:return false;}} bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;} void clockCpu()override{if(m_irqEnable&&!m_irq&&++m_irqCounter==0x1000){m_irq=true;m_irqEnable=false;}} bool irqActive()const override{return m_irq;} void reset(bool)override{m_bank=0;m_irqEnable=m_irq=false;m_irqCounter=0;} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_bank);put8(o,m_irqEnable);put8(o,m_irq);put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));} bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;if(!get8(p,e,m_bank)||!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irq=b;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqCounter=uint16_t(lo)|(uint16_t(hi)<<8);return true;} private:uint8_t m_bank=0;bool m_irqEnable=false,m_irq=false;uint16_t m_irqCounter=0;};


// Phase 28: legacy multicart gaps.
// Mapper 51 / Ball Games 11-in-1. PRG-ROM is also visible at $6000-$7FFF.
class Mapper51 final : public Mapper {
public:
    explicit Mapper51(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x6000||!m_config.prgRomSize)return false;
        if(a<0x8000){ const size_t b=((m_mode&1)?0x23:0x2F)|(size_t(m_bank)<<2); m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF); return true; }
        if(m_mode&1){ m=mapBank(m_bank,0x8000,m_config.prgRomSize,a-0x8000); return true; }
        const size_t b16=(size_t(m_bank)<<1)|((a<0xC000)?(m_mode>>1):7); m=mapBank(b16,0x4000,m_config.prgRomSize,a&0x3FFF); return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a>=0x6000&&a<0x8000){m_mode=uint8_t(((d>>3)&2)|((d>>1)&1));updateMirror();return true;}
        if(a>=0x8000&&a<0xC000){m_bank=d&0x0F;return true;}
        if(a>=0xC000&&a<0xE000){m_bank=d&0x0F;m_mode=uint8_t(((d>>3)&2)|(m_mode&1));updateMirror();return true;}
        if(a>=0xE000){m_bank=d&0x0F;return true;} return false;
    }
    bool mapPrgRam(uint16_t,uint32_t&,bool) const override{return false;}
    void reset(bool hard) override{if(hard){m_bank=0;m_mode=1;updateMirror();}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_bank);put8(o,m_mode);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_bank)||!get8(p,e,m_mode))return false;updateMirror();return true;}
private:
    uint8_t m_bank=0,m_mode=1; void updateMirror(){m_mirror=(m_mode==3)?Mirror::Horizontal:Mirror::Vertical;}
};

// Mapper 61 / 20-in-1: banking is entirely selected by CPU address lines.
class Mapper61 final : public Mapper {
public: explicit Mapper61(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;if((m_addr&0x30)==0||(m_addr&0x30)==0x30){m=mapBank(m_addr&0x0F,0x8000,m_config.prgRomSize,a-0x8000);}else{const size_t b=((size_t(m_addr)<<1)&0x1E)|((m_addr>>4)&2);m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);}return true;}
    bool cpuWrite(uint16_t a,uint8_t,uint64_t)override{if(a<0x8000)return false;m_addr=a;m_mirror=(a&0x80)?Mirror::Horizontal:Mirror::Vertical;return true;}
    void reset(bool hard)override{if(hard){m_addr=0x8000;m_mirror=Mirror::Vertical;}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,uint8_t(m_addr));put8(o,uint8_t(m_addr>>8));put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,mi;if(!get8(p,e,lo)||!get8(p,e,hi)||!get8(p,e,mi))return false;m_addr=uint16_t(lo)|(uint16_t(hi)<<8);m_mirror=Mirror(mi);return true;}
private:uint16_t m_addr=0x8000;
};

// Mapper 63 / NTDEC TH2291-3 (submapper 0) and 82AB (submapper 1).
// The mapper latches CPU address lines on writes in $8000-$FFFF.  It can
// present an NROM-128 (mirrored 16 KiB) or NROM-256 (32 KiB) PRG window,
// controls H/V mirroring, and can write-protect its unbanked 8 KiB CHR-RAM.
// Selecting a PRG bank beyond the populated ROM leaves the cartridge data bus
// undriven; cpuMapRead therefore returns false instead of wrapping the bank.
class Mapper63 final : public Mapper {
public:
    explicit Mapper63(const MapperConfig& c):Mapper(c){}

    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;

        // Submapper 0: A9-A3 -> PRG A21-A15, A2 -> PRG A14 in NROM-128.
        // Submapper 1: A8-A3 -> PRG A20-A15 (one fewer outer PRG bit),
        //              A2 -> PRG A14 in NROM-128.
        const unsigned outerBits=(m_config.nes20&&m_config.submapper==1)?6u:7u;
        const size_t outer=(size_t(m_addr>>3)&((size_t(1)<<outerBits)-1));
        const bool nrom256=(m_addr&0x0002)!=0;
        const size_t lowBit=nrom256?size_t((a>>14)&1):size_t((m_addr>>2)&1);
        const size_t bank16=(outer<<1)|lowBit;
        const size_t mapped=bank16*0x4000u+size_t(a&0x3FFF);
        if(mapped>=m_config.prgRomSize)return false; // real open bus, no wrapping
        m=uint32_t(mapped);return true;
    }

    bool cpuWrite(uint16_t a,uint8_t,uint64_t) override {
        if(a<0x8000)return false;
        m_addr=a;
        m_mirror=(a&1)?Mirror::Horizontal:Mirror::Vertical;
        return true;
    }

    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000||!m_config.chrRamSize)return false;
        m=uint32_t(a%m_config.chrRamSize);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override {
        if(chrWriteProtected())return false;
        return ppuMapRead(a,m);
    }
    bool ppuUsesChrRam(uint16_t a) const override {return a<0x2000&&m_config.chrRamSize!=0;}

    void reset(bool hard) override {
        if(hard){m_addr=0x8000;m_mirror=Mirror::Vertical;}
    }
    void saveState(std::vector<uint8_t>&o)const override {
        put8(o,uint8_t(m_addr));put8(o,uint8_t(m_addr>>8));put8(o,uint8_t(m_mirror));
    }
    bool loadState(const uint8_t*&p,const uint8_t*e)override {
        uint8_t lo,hi,mi;
        if(!get8(p,e,lo)||!get8(p,e,hi)||!get8(p,e,mi))return false;
        m_addr=uint16_t(lo)|(uint16_t(hi)<<8);m_mirror=Mirror(mi);return true;
    }
private:
    bool chrWriteProtected() const {
        // TH2291-3 uses CPU A10; 82AB moves /WE protection to CPU A9.
        return (m_addr&((m_config.nes20&&m_config.submapper==1)?0x0200:0x0400))!=0;
    }
    uint16_t m_addr=0x8000;
};


// Phase 29: remaining legacy 54-59 mapper gaps with well-defined hardware.
// Mapper 53 / Supervision 16-in-1. Two legacy dump orderings exist; the
// cartridge hash database resolves m_config.boardVariant=1 for the EPROM-first
// ordering used by one known dump set.
class Mapper53 final : public Mapper {
public:
    explicit Mapper53(const MapperConfig& c):Mapper(c){m_mirror=Mirror::Vertical;}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x6000||!m_config.prgRomSize)return false;
        const size_t r=(size_t(m_reg0)<<3)&0x78;
        if(a<0x8000){
            const size_t b=((r<<1)|0x0F)+(m_config.boardVariant?4:0);
            m=mapBank(b,0x2000,m_config.prgRomSize,a-0x6000);return true;
        }
        const bool switched=(m_reg0&0x10)!=0;
        size_t b;
        if(a<0xC000){
            b=switched ? ((r|(m_reg1&7))+(m_config.boardVariant?2:0)) : (m_config.boardVariant?0x00:0x80);
            m=mapBank(b,0x4000,m_config.prgRomSize,a-0x8000);
        } else {
            b=switched ? ((r|7)+(m_config.boardVariant?2:0)) : (m_config.boardVariant?0x01:0x81);
            m=mapBank(b,0x4000,m_config.prgRomSize,a-0xC000);
        }
        return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a>=0x6000&&a<0x8000){m_reg0=d;m_mirror=(d&0x20)?Mirror::Horizontal:Mirror::Vertical;return true;}
        if(a>=0x8000){m_reg1=d;return true;} return false;
    }
    bool mapPrgRam(uint16_t,uint32_t&,bool) const override{return false;}
    void reset(bool hard) override{if(hard){m_reg0=m_reg1=0;m_mirror=Mirror::Vertical;}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_reg0);put8(o,m_reg1);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t mi;if(!get8(p,e,m_reg0)||!get8(p,e,m_reg1)||!get8(p,e,mi))return false;m_mirror=Mirror(mi);return true;}
private:uint8_t m_reg0=0,m_reg1=0;
};

// Mapper 54 / Novel Diamond 9999999-in-1: write-address lines select both
// a 32 KiB PRG bank and an 8 KiB CHR bank; written data is ignored.
class Mapper54 final : public Mapper {
public:
    explicit Mapper54(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;
        m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;
    }
    bool cpuWrite(uint16_t a,uint8_t,uint64_t) override {
        if(a<0x8000)return false;
        m_prg=uint8_t(a&3);
        m_chr=uint8_t(a&7);
        return true;
    }
    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000)return false;
        auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!z)return false;
        m=mapBank(m_chr,0x2000,z,a);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void reset(bool hard) override{if(hard)m_prg=m_chr=0;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}
private:uint8_t m_prg=0,m_chr=0;
};

// Mapper 55 / BTL-MARIO1-MALEE2: NROM-256 plus 2 KiB expansion PRG-ROM
// at $6000-$6FFF and 2 KiB RAM mirrored through $7000-$7FFF.
class Mapper55 final : public Mapper {
public:
    explicit Mapper55(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(!m_config.prgRomSize)return false;
        if(a>=0x6000&&a<0x7000){ if(m_config.prgRomSize<=0x8000)return false; m=uint32_t(0x8000+((a-0x6000)&0x07FF)); return m<m_config.prgRomSize; }
        if(a>=0x8000){m=uint32_t((a-0x8000)%std::min<std::size_t>(m_config.prgRomSize,0x8000));return true;}
        return false;
    }
    bool mapPrgRam(uint16_t a,uint32_t& m,bool) const override {
        if(a<0x7000||a>0x7FFF||!m_config.prgRamSize)return false;
        m=uint32_t((a-0x7000)&0x07FF);
        m%=uint32_t(m_config.prgRamSize);
        return true;
    }
};

// Mapper 56 / Kaiser KS-202: VRC3-like nibble IRQ plus four 8 KiB PRG
// registers, eight 1 KiB CHR registers and optional PRG-ROM at $6000.
class Mapper56 final : public Mapper {
public:
    explicit Mapper56(const MapperConfig& c):Mapper(c){m_prg[3]=0xFF;}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(!m_config.prgRomSize)return false;
        if(a>=0x6000&&a<0x8000&&m_useRom){m=mapBank(m_prg[3],0x2000,m_config.prgRomSize,a&0x1FFF);return true;}
        if(a<0x8000)return false;
        const size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);
        const size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?m_prg[2]:n-1;
        m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;
    }
    bool mapPrgRam(uint16_t a,uint32_t& m,bool) const override {
        if(m_useRom||a<0x6000||a>0x7FFF||!m_config.prgRamSize)return false;
        m=uint32_t((a-0x6000)%m_config.prgRamSize);
        return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a<0x8000)return false;
        switch(a&0xF000){
        case 0x8000:m_reload=uint16_t((m_reload&0xFFF0)|(d&0x0F));return true;
        case 0x9000:m_reload=uint16_t((m_reload&0xFF0F)|((d&0x0F)<<4));return true;
        case 0xA000:m_reload=uint16_t((m_reload&0xF0FF)|((d&0x0F)<<8));return true;
        case 0xB000:m_reload=uint16_t((m_reload&0x0FFF)|((d&0x0F)<<12));return true;
        case 0xC000:m_control=d;m_irq=false;if(m_control&2)m_counter=m_reload;return true;
        case 0xD000:m_irq=false;return true;
        case 0xE000:m_selected=int(d&7)-1;return true;
        case 0xF000:
            if(m_selected>=0&&m_selected<=3)m_prg[m_selected]=uint8_t((m_prg[m_selected]&0x10)|(d&0x0F));
            else if(m_selected==4)m_useRom=(d&0x04)!=0;
            switch(a&0xFC00){
            case 0xF000:{const unsigned b=a&3;m_prg[b]=uint8_t((d&0x10)|(m_prg[b]&0x0F));break;}
            case 0xF800:if(!m_config.fourScreen)m_mirror=(d&1)?Mirror::Vertical:Mirror::Horizontal;break;
            case 0xFC00:m_chr[a&7]=d;break;
            }
            return true;
        }
        return false;
    }
    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000)return false;
        auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!z)return false;
        m=mapBank(m_chr[a>>10],0x400,z,a&0x3FF);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void clockCpu() override {if(m_control&2){++m_counter;if(m_counter==0xFFFF){m_counter=m_reload;m_control&=uint8_t(~2);m_irq=true;}}}
    bool irqActive()const override{return m_irq;}
    void reset(bool hard)override{m_irq=false;m_control=0;m_reload=m_counter=0;m_selected=-1;m_useRom=false;if(hard){for(auto&v:m_prg)v=0;m_prg[3]=0xFF;for(auto&v:m_chr)v=0;}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,uint8_t(m_reload));put8(o,uint8_t(m_reload>>8));put8(o,uint8_t(m_counter));put8(o,uint8_t(m_counter>>8));put8(o,m_control);put8(o,m_irq);put8(o,uint8_t(m_selected+1));put8(o,m_useRom);for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,irq,sel,use,mi;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_reload=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_counter=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,m_control)||!get8(p,e,irq)||!get8(p,e,sel)||!get8(p,e,use))return false;m_irq=irq;m_selected=int(sel)-1;m_useRom=use;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;if(!get8(p,e,mi))return false;m_mirror=Mirror(mi);return true;}
private:
    uint16_t m_reload=0,m_counter=0;uint8_t m_control=0;bool m_irq=false;int m_selected=-1;bool m_useRom=false;uint8_t m_prg[4]={0,0,0,0xFF},m_chr[8]={};
};

// Mapper 57 / GK-54 family: two registers independently select PRG mode,
// CHR bank and mirroring. Some physical carts expose DIP switches at $6000;
// this core currently returns the default switch position (0).
class Mapper57 final : public Mapper {
public:
    explicit Mapper57(const MapperConfig& c):Mapper(c){}
    bool cpuReadRegister(uint16_t a,uint8_t&d)override{if(a==0x6000){d=0;return true;}return false;}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;const uint8_t r=m_regs[1];size_t b;if(r&0x10)b=((r>>5)&6)+(a>=0xC000);else b=(r>>5)&7;m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if((a&0x8800)==0x8000)m_regs[0]=d;else if((a&0x8800)==0x8800)m_regs[1]=d;else return false;updateMirror();return true;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;const uint8_t b=uint8_t(((m_regs[0]&0x40)>>3)|((m_regs[0]|m_regs[1])&7));m=mapBank(b,0x2000,z,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void reset(bool hard)override{if(hard){m_regs[0]=m_regs[1]=0;updateMirror();}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_regs[0]);put8(o,m_regs[1]);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_regs[0])||!get8(p,e,m_regs[1]))return false;updateMirror();return true;}
private:uint8_t m_regs[2]={};void updateMirror(){m_mirror=(m_regs[1]&8)?Mirror::Horizontal:Mirror::Vertical;}
};

// Mapper 59 / UNL-D1038: write address lines select PRG/CHR/mirroring and
// optionally replace normal reads with the two-bit DIP switch value. The
// frontend has no mapper-DIP UI yet, so the hardware-default value is zero.
class Mapper59 final : public Mapper {
public:
    explicit Mapper59(const MapperConfig& c):Mapper(c){}
    bool cpuReadRegister(uint16_t,uint8_t&d)override{if(!m_dip)return false;d=0;return true;}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;if(m_16k)m=mapBank(m_prg16,0x4000,m_config.prgRomSize,a&0x3FFF);else m=mapBank(m_prg32,0x8000,m_config.prgRomSize,a-0x8000);return true;}
    bool cpuWrite(uint16_t a,uint8_t,uint64_t)override{if(a<0x8000)return false;m_16k=(a&0x80)!=0;if(m_16k)m_prg16=uint8_t((a&0x70)>>4);else m_prg32=uint8_t((a&0x60)>>5);m_chr=uint8_t(a&7);m_mirror=(a&8)?Mirror::Horizontal:Mirror::Vertical;m_dip=(a&0x100)!=0;return true;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_chr,0x2000,z,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void reset(bool hard)override{if(hard){m_16k=false;m_prg16=m_prg32=m_chr=0;m_dip=false;m_mirror=Mirror::Vertical;}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_16k);put8(o,m_prg16);put8(o,m_prg32);put8(o,m_chr);put8(o,m_dip);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b,mi;if(!get8(p,e,b))return false;m_16k=b;if(!get8(p,e,m_prg16)||!get8(p,e,m_prg32)||!get8(p,e,m_chr)||!get8(p,e,b)||!get8(p,e,mi))return false;m_dip=b;m_mirror=Mirror(mi);return true;}
private:bool m_16k=false;uint8_t m_prg16=0,m_prg32=0,m_chr=0;bool m_dip=false;
};

class Mapper58 final : public Mapper { public: explicit Mapper58(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t b=m_same?m_prg:(m_prg&~1u)+(a>=0xC000);m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);return true;} bool cpuWrite(uint16_t a,uint8_t,uint64_t)override{if(a<0x8000)return false;m_prg=a&7;m_chr=(a>>3)&7;m_same=a&0x40;m_mirror=(a&0x80)?Mirror::Horizontal:Mirror::Vertical;return true;} bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_chr,0x2000,z,a);return true;} bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,m_same);put8(o,uint8_t(m_mirror));} bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_same=b;if(!get8(p,e,b))return false;m_mirror=Mirror(b);return true;} private:uint8_t m_prg=0,m_chr=0;bool m_same=false;};


// Mapper 171 / Kaiser KS-7058.  CPU PRG is a fixed 32 KiB window while
// two independent 4 KiB CHR registers are selected by CPU A7.  Only the
// address decode $F000/$F080 under mask $F080 matters; mirroring is hardwired
// by the cartridge header and there is no PRG-RAM or IRQ hardware.
class Mapper171 final : public Mapper {
public:
    explicit Mapper171(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(0, 0x8000, m_config.prgRomSize, a & 0x7FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        const uint16_t reg = a & 0xF080;
        if (reg == 0xF000) { m_chr[0] = d; return true; }
        if (reg == 0xF080) { m_chr[1] = d; return true; }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        const unsigned slot = (a >> 12) & 1;
        m = mapBank(m_chr[slot], 0x1000, size, a & 0x0FFF);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m);
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_chr[0] = m_chr[1] = 0;
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_chr[0]); put8(o, m_chr[1]);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        return get8(p,e,m_chr[0]) && get8(p,e,m_chr[1]);
    }

private:
    uint8_t m_chr[2] = {0, 0};
};


// Mapper 174 / NTDEC 5-in-1.  The register is the CPU write address itself:
// A7 selects 16/32 KiB PRG mode, A6-A4 select PRG, A3-A1 select CHR, and
// A0 selects H/V mirroring.  The data byte is not connected.  A soft reset
// intentionally leaves the latch untouched; the multicart returns to its menu
// only after a power cycle, so reset(false) is a no-op as in the base class.
class Mapper174 final : public Mapper {
public:
    explicit Mapper174(const MapperConfig& c) : Mapper(c) { m_mirror = Mirror::Vertical; }

    void reset(bool hard) override {
        if (!hard) return;
        m_prg = 0; m_chr = 0; m_mode32 = false; m_mirror = Mirror::Vertical;
    }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        if (m_mode32) {
            // In 32 KiB mode P2:P1 choose the bank; P0 is ignored.
            m = mapBank(m_prg >> 1, 0x8000, m_config.prgRomSize, a & 0x7FFF);
        } else {
            // In 16 KiB mode the selected bank appears at both CPU halves.
            m = mapBank(m_prg, 0x4000, m_config.prgRomSize, a & 0x3FFF);
        }
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t, uint64_t) override {
        if (a < 0x8000) return false;
        m_mode32 = (a & 0x0080) != 0;
        m_prg = uint8_t((a >> 4) & 0x07);
        m_chr = uint8_t((a >> 1) & 0x07);
        m_mirror = (a & 0x0001) ? Mirror::Horizontal : Mirror::Vertical;
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m);
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_prg); put8(o, m_chr); put8(o, m_mode32 ? 1 : 0); put8(o, uint8_t(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t mode = 0, mirror = 0;
        if (!get8(p,e,m_prg) || !get8(p,e,m_chr) || !get8(p,e,mode) || !get8(p,e,mirror)) return false;
        m_mode32 = mode != 0; m_mirror = Mirror(mirror); return true;
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
    bool m_mode32 = false;
};



// Phase 47 bundled multicart family. These seven iNES assignments are small
// boards with no IRQ/RAM hardware; all bank state is captured by one high-
// address write latch.  Keeping them in one implementation avoids duplicating
// identical 16/32 KiB PRG and 8 KiB CHR mapping plumbing while leaving each
// board's address/data wiring explicit in cpuWrite().
//   202 BMC-150IN1        203 BMC-35IN1       204 BMC-64IN1
//   214 BMC-SUPERGUN20IN1 217 BMC-500IN1      229 BMC-31IN1
//   231 BMC-20IN1
class MapperBmcBundled47 final : public Mapper {
public:
    explicit MapperBmcBundled47(const MapperConfig& c) : Mapper(c) { reset(true); }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        if (m_mode32) {
            m = mapBank(m_prg32, 0x8000, m_config.prgRomSize, a & 0x7FFF);
        } else {
            const uint8_t bank = a < 0xC000 ? m_prgLo : m_prgHi;
            m = mapBank(bank, 0x4000, m_config.prgRomSize, a & 0x3FFF);
        }
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a < 0x8000) return false;
        const uint16_t o = uint16_t(a - 0x8000);
        switch (m_config.id) {
        case 202: { // BMC-150IN1: A3-A1 bank, A0 mirroring; data disconnected.
            const uint8_t bank = uint8_t((o >> 1) & 0x07);
            const uint8_t mode = ((bank & 0x06) == 0x06) ? 1 : 0;
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
            m_chr = bank;
            m_mirror = (o & 1) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        }
        case 203: { // BMC-35IN1: D3-D2 PRG, D1-D0 CHR.
            const uint8_t bank = uint8_t((d >> 2) & 0x03);
            set16(bank, bank);
            m_chr = uint8_t(d & 0x03);
            break;
        }
        case 204: { // BMC-64IN1: address banking; D4 mirroring.
            const uint16_t join = uint16_t(((o >> 1) & (o >> 2)) & 1);
            set16(uint8_t(o & ~join), uint8_t(o | join));
            m_chr = uint8_t(o & ~join);
            m_mirror = (d & 0x10) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        }
        case 214: { // BMC-SUPERGUN20IN1: only $8xxx/$Fxxx decode.
            const uint16_t region = uint16_t(o & 0x7000);
            if (region != 0x0000 && region != 0x7000) return true;
            const uint8_t bank = uint8_t(o >> 2);
            set16(bank, bank);
            m_chr = bank;
            break;
        }
        case 217: // BMC-500IN1: A4-A2 PRG32, A2-A0 CHR.
            m_mode32 = true;
            m_prg32 = uint8_t((o >> 2) & 0x07);
            m_chr = uint8_t(o & 0x07);
            break;
        case 229: { // BMC-31IN1: A4-A0 bank, A5 mirroring.
            m_mirror = (o & 0x20) ? Mirror::Horizontal : Mirror::Vertical;
            const uint8_t bank = uint8_t(o & 0x1F);
            m_chr = bank;
            if (bank & 0x1E) set16(bank, bank);
            else { m_mode32 = true; m_prg32 = 0; }
            break;
        }
        case 231: { // BMC-20IN1: A4-A1 base, A5 selects upper half, A7 mirror.
            const uint8_t base = uint8_t(o & 0x1E);
            set16(base, uint8_t(base | ((o >> 5) & 1)));
            m_mirror = (o & 0x80) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        }
        default: return false;
        }
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m);
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void reset(bool hard) override {
        if (!hard) return;
        m_mode32 = true; m_prg32 = 0; m_prgLo = 0; m_prgHi = 1; m_chr = 0;
        m_mirror = m_config.fourScreen ? Mirror::FourScreen : m_config.headerMirror;
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_mode32 ? 1 : 0); put8(o, m_prg32); put8(o, m_prgLo); put8(o, m_prgHi);
        put8(o, m_chr); put8(o, uint8_t(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t mode = 0, mirror = 0;
        if (!get8(p,e,mode) || !get8(p,e,m_prg32) || !get8(p,e,m_prgLo) || !get8(p,e,m_prgHi) ||
            !get8(p,e,m_chr) || !get8(p,e,mirror)) return false;
        if (mirror > uint8_t(Mirror::FourScreen)) return false;
        m_mode32 = mode != 0; m_mirror = Mirror(mirror); return true;
    }

private:
    void set16(uint8_t lo, uint8_t hi) { m_mode32 = false; m_prgLo = lo; m_prgHi = hi; }
    bool m_mode32 = true;
    uint8_t m_prg32 = 0, m_prgLo = 0, m_prgHi = 1, m_chr = 0;
};


// Phase 49: Mapper 212 / BMC Super HiK 300-in-1.
// Address bits A2-A0 feed both PRG and CHR banking, A3 selects H/V
// mirroring, and A14 selects 32 KiB versus mirrored 16 KiB PRG mode.
// Reads matching $6000 under mask $E010 drive only D7; D0-D6 remain open.
class Mapper212 final : public Mapper {
public:
    explicit Mapper212(const MapperConfig& c) : Mapper(c) { reset(true); }
    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        if (m_mode32) m = mapBank(m_prg32, 0x8000, m_config.prgRomSize, a & 0x7FFF);
        else m = mapBank(m_prg16, 0x4000, m_config.prgRomSize, a & 0x3FFF);
        return true;
    }
    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if ((a & 0xE010) != 0x6000) return false;
        d = uint8_t((d & 0x7F) | 0x80);
        return true;
    }
    bool cpuWrite(uint16_t a, uint8_t, uint64_t) override {
        if (a < 0x8000) return false;
        const uint8_t b = uint8_t(a & 0x07);
        m_chr = b;
        m_mirror = (a & 0x0008) ? Mirror::Horizontal : Mirror::Vertical;
        m_mode32 = (a & 0x4000) != 0;
        if (m_mode32) m_prg32 = uint8_t(b >> 1); else m_prg16 = b;
        return true;
    }
    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a); return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m); }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    void reset(bool hard) override { if (!hard) return; m_mode32=false; m_prg16=m_prg32=m_chr=0; m_mirror=m_config.fourScreen?Mirror::FourScreen:m_config.headerMirror; }
    void saveState(std::vector<uint8_t>& o) const override { put8(o,m_mode32?1:0);put8(o,m_prg16);put8(o,m_prg32);put8(o,m_chr);put8(o,uint8_t(m_mirror)); }
    bool loadState(const uint8_t*& p,const uint8_t* e) override { uint8_t mode=0,mir=0; if(!get8(p,e,mode)||!get8(p,e,m_prg16)||!get8(p,e,m_prg32)||!get8(p,e,m_chr)||!get8(p,e,mir))return false; if(mir>uint8_t(Mirror::FourScreen))return false; m_mode32=mode!=0;m_mirror=Mirror(mir);return true; }
private:
    bool m_mode32=false; uint8_t m_prg16=0,m_prg32=0,m_chr=0;
};

// Phase 48 bundled multicarts: mappers 225, 226, 230, 235 and 236.
// These boards are all small discrete multicarts, but use sufficiently
// different latch wiring that one implementation keeps the distinctions
// explicit while sharing the common PRG/CHR/state plumbing.
class MapperBmcBundled48 final : public Mapper {
public:
    explicit MapperBmcBundled48(const MapperConfig& c) : Mapper(c) { reset(true); }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        std::size_t bank16 = 0;
        switch (m_config.id) {
        case 225:
        case 255:
        case 226:
        case 227:
        case 235:
        case 236:
        case 237:
            bank16 = (a < 0xC000) ? m_prgLo : m_prgHi;
            break;
        case 230:
            if (!m_multiMode) {
                bank16 = (a < 0xC000) ? m_prgLo : 7;
            } else {
                bank16 = (a < 0xC000) ? m_prgLo : m_prgHi;
            }
            break;
        default: return false;
        }
        const std::size_t base = bank16 * 0x4000;
        // Mapper 235 can select physically absent ROM sockets.  If the
        // selected linear bank is beyond the dumped image, leave the CPU bus
        // undriven instead of wrapping through mapBank().
        if (m_config.id == 235 && base >= m_config.prgRomSize) return false;
        uint32_t inBank = uint32_t(a & 0x3FFF);
        if (m_config.id == 237 && (m_latch[0] & 1)) inBank |= 0x0002;
        m = uint32_t((base + inBank) % m_config.prgRomSize);
        return true;
    }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if ((m_config.id != 225 && m_config.id != 255) || a < 0x5800 || a > 0x5FFF) return false;
        // 74LS670 RAM: only D0-D3 are driven, upper data-bus bits remain open.
        d = uint8_t((d & 0xF0) | (m_ram4[a & 3] & 0x0F));
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((m_config.id == 225 || m_config.id == 255) && a >= 0x5800 && a <= 0x5FFF) {
            m_ram4[a & 3] = uint8_t(d & 0x0F);
            return true;
        }
        if (a < 0x8000) return false;
        const uint16_t o = uint16_t(a - 0x8000);
        switch (m_config.id) {
        case 225:
        case 255: {
            // BMC 72/110-in-1 (mapper 255 is a documented duplicate).  Address bits are the complete latch.
            const uint8_t high = uint8_t((o >> 8) & 0x40);
            const uint8_t bank = uint8_t(high | ((o >> 6) & 0x3F));
            const uint8_t mode = uint8_t(((o & 0x1000) == 0) ? 1 : 0);
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
            m_chr = uint8_t(high | (o & 0x3F));
            m_mirror = (o & 0x2000) ? Mirror::Horizontal : Mirror::Vertical;
            return true;
        }
        case 226: {
            m_reg[o & 1] = d;
            const uint8_t bank = uint8_t((m_reg[1] << 6) | ((m_reg[0] & 0x80) >> 2) | (m_reg[0] & 0x1F));
            const uint8_t mode = uint8_t(((m_reg[0] & 0x20) == 0) ? 1 : 0);
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
            m_mirror = (m_reg[0] & 0x40) ? Mirror::Vertical : Mirror::Horizontal;
            return true;
        }
        case 227: {
            const uint8_t bank = uint8_t(((o >> 2) & 0x1F) | ((o & 0x0100) >> 3));
            if (o & 0x0080) { // NROM modes
                set16(bank, uint8_t(bank + (o & 1)));
            } else { // UNROM-like modes
                const uint8_t lowMask = (o & 1) ? 0x3E : 0xFF;
                m_prgLo = uint8_t(bank & lowMask);
                m_prgHi = uint8_t((o & 0x0200) ? (bank | 0x07) : (bank & 0x38));
            }
            m_mirror = (o & 0x0002) ? Mirror::Horizontal : Mirror::Vertical;
            m_chrWriteProtect = (o & 0x0080) != 0 && m_config.submapper != 0;
            return true;
        }
        case 230: {
            if (!m_multiMode) {
                m_prgLo = uint8_t(d & 0x07);
                m_prgHi = 7;
                m_mirror = Mirror::Vertical;
            } else {
                const uint8_t bank = uint8_t((d & 0x1F) + 8);
                const uint8_t mode = uint8_t(((d & 0x20) == 0) ? 1 : 0);
                set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
                m_mirror = (d & 0x40) ? Mirror::Vertical : Mirror::Horizontal;
            }
            return true;
        }
        case 235: {
            // Golden Game multicarts. Equivalent to MAME's bitswap
            // bitswap<8>(offset,9,8,4,3,2,1,0,12).
            const uint8_t bank = uint8_t((((o >> 9) & 1) << 7) |
                                         (((o >> 8) & 1) << 6) |
                                         (((o >> 4) & 1) << 5) |
                                         (((o >> 3) & 1) << 4) |
                                         (((o >> 2) & 1) << 3) |
                                         (((o >> 1) & 1) << 2) |
                                         (((o >> 0) & 1) << 1) |
                                         (((o >> 12) & 1) << 0));
            const uint8_t mode = uint8_t(((o & 0x0800) == 0) ? 1 : 0);
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
            if (o & 0x0400) m_mirror = Mirror::OnescreenLo;
            else m_mirror = (o & 0x2000) ? Mirror::Horizontal : Mirror::Vertical;
            return true;
        }
        case 236: {
            m_latch[(o >> 14) & 1] = o;
            update236();
            return true;
        }
        case 237: {
            if (m_latch[0] & 0x0002) { // locked: inner bank remains writable
                m_reg[0] = uint8_t((m_reg[0] & ~0x07) | (d & 0x07));
            } else {
                m_latch[0] = o;
                m_reg[0] = d;
            }
            const uint8_t bank = uint8_t(((m_latch[0] & 0x0004) ? 0x20 : 0) | (m_reg[0] & 0x1F));
            const uint8_t mode = uint8_t((m_reg[0] >> 6) & 1);
            m_prgLo = uint8_t(bank & ~mode);
            m_prgHi = uint8_t(bank | ((m_reg[0] & 0x80) ? mode : 0x07));
            m_mirror = (m_reg[0] & 0x20) ? Mirror::Horizontal : Mirror::Vertical;
            return true;
        }
        default: return false;
        }
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        const std::size_t bank = ((m_config.id == 225 || m_config.id == 255) || (m_config.id == 236 && m_config.chrRomSize)) ? m_chr : 0;
        m = mapBank(bank, 0x2000, size, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        if (m_config.id == 227 && m_chrWriteProtect) return false;
        return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m);
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void reset(bool hard) override {
        if (m_config.id == 230) {
            if (hard) m_multiMode = false;
            else m_multiMode = !m_multiMode;
        } else if (!hard && m_config.id != 226 && m_config.id != 227 && m_config.id != 235 && m_config.id != 237) {
            return;
        }
        m_reg[0] = m_reg[1] = 0;
        m_latch[0] = m_latch[1] = 0;
        m_chr = 0;
        if (hard) for (auto& n : m_ram4) n = 0;
        m_mirror = m_config.fourScreen ? Mirror::FourScreen : m_config.headerMirror;
        switch (m_config.id) {
        case 225: case 255: set16(0,1); m_mirror = Mirror::Vertical; break;
        case 226: set16(0,1); m_mirror = Mirror::Horizontal; break;
        case 227: set16(0,0); m_mirror = Mirror::Vertical; m_chrWriteProtect=false; break;
        case 230:
            if (m_multiMode) { set16(8,9); m_mirror = Mirror::Horizontal; }
            else { set16(0,7); m_mirror = Mirror::Vertical; }
            break;
        case 235: set16(0,1); m_mirror = Mirror::Vertical; break;
        case 236: update236(); break;
        case 237: set16(0,7); m_mirror=Mirror::Vertical; m_chrWriteProtect=false; break;
        default: break;
        }
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o,m_prgLo); put8(o,m_prgHi); put8(o,m_chr);
        put8(o,m_reg[0]); put8(o,m_reg[1]);
        put8(o,uint8_t(m_latch[0])); put8(o,uint8_t(m_latch[0]>>8));
        put8(o,uint8_t(m_latch[1])); put8(o,uint8_t(m_latch[1]>>8));
        put8(o,m_multiMode?1:0); put8(o,m_chrWriteProtect?1:0); put8(o,uint8_t(m_mirror));
        for (uint8_t n : m_ram4) put8(o,n);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t multi=0, protect=0, mirror=0;
        if(!get8(p,e,m_prgLo)||!get8(p,e,m_prgHi)||!get8(p,e,m_chr)||
           !get8(p,e,m_reg[0])||!get8(p,e,m_reg[1])||
           !get8(p,e,m_latchBytes[0])||!get8(p,e,m_latchBytes[1])||
           !get8(p,e,m_latchBytes[2])||!get8(p,e,m_latchBytes[3])||
           !get8(p,e,multi)||!get8(p,e,protect)||!get8(p,e,mirror)) return false;
        m_latch[0]=uint32_t(m_latchBytes[0])|(uint32_t(m_latchBytes[1])<<8);
        m_latch[1]=uint32_t(m_latchBytes[2])|(uint32_t(m_latchBytes[3])<<8);
        if (mirror > uint8_t(Mirror::FourScreen)) return false;
        for (auto& n : m_ram4) if(!get8(p,e,n)) return false;
        m_multiMode = multi != 0; m_chrWriteProtect = protect != 0; m_mirror = Mirror(mirror); return true;
    }

private:
    void set16(uint8_t lo, uint8_t hi) { m_prgLo=lo; m_prgHi=hi; }
    void update236() {
        const uint16_t lo=m_latch[0], hi=m_latch[1];
        uint8_t bank=0;
        if (m_config.chrRomSize) {
            bank = uint8_t(hi & 0x07);
            m_chr = uint8_t(lo & 0x07);
        } else {
            bank = uint8_t(((lo & 0x07) << 3) | (hi & 0x07));
        }
        if (hi & 0x20) { // NROM mode
            const uint8_t mode = uint8_t((hi & 0x10) ? 0 : 1);
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
        } else { // UNROM-like mode
            set16(bank, uint8_t(bank | 0x07));
        }
        m_mirror = (lo & 0x20) ? Mirror::Horizontal : Mirror::Vertical;
    }

    uint8_t m_prgLo=0, m_prgHi=1, m_chr=0;
    uint8_t m_reg[2]{0,0};
    uint32_t m_latch[2]{0,0};
    uint8_t m_latchBytes[4]{0,0,0,0};
    bool m_multiMode=false;
    bool m_chrWriteProtect=false;
    uint8_t m_ram4[4]{0,0,0,0};
};

// Mapper 200 / simple NROM multicart. The CPU write address is the latch:
// A2-A0 select a shared 16 KiB PRG / 8 KiB CHR bank and A3 selects
// vertical/horizontal mirroring. The selected 16 KiB PRG bank is mirrored
// into both CPU halves; the data byte is not connected.
class Mapper200 final : public Mapper {
public:
    explicit Mapper200(const MapperConfig& c) : Mapper(c) { m_mirror = Mirror::Vertical; }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(m_bank, 0x4000, m_config.prgRomSize, a & 0x3FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t, uint64_t) override {
        if (a < 0x8000) return false;
        m_bank = uint8_t(a & 0x0007);
        m_mirror = (a & 0x0008) ? Mirror::Horizontal : Mirror::Vertical;
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_bank, 0x2000, size, a);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m);
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_bank = 0;
        m_mirror = Mirror::Vertical;
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_bank); put8(o, uint8_t(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t mirror = 0;
        if (!get8(p,e,m_bank) || !get8(p,e,mirror)) return false;
        if (mirror > uint8_t(Mirror::FourScreen)) return false;
        m_mirror = Mirror(mirror);
        return true;
    }

private:
    uint8_t m_bank = 0;
};

class Mapper60 final : public Mapper { public: explicit Mapper60(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_game,0x4000,m_config.prgRomSize,a&0x3FFF);return true;} bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_game,0x2000,z,a);return true;} bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);} void reset(bool hard)override{m_game=hard?0:uint8_t((m_game+1)&3);} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_game);} bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_game);} private:uint8_t m_game=0;};

class Mapper62 final : public Mapper { public: explicit Mapper62(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t b=m_same?m_prg:(m_prg&~1u)+(a>=0xC000);m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=((a&0x3F00)>>8)|(a&0x40);m_chr=((a&0x1F)<<2)|(d&3);m_same=a&0x20;m_mirror=(a&0x80)?Mirror::Horizontal:Mirror::Vertical;return true;} bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_chr,0x2000,z,a);return true;} bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,m_same);put8(o,uint8_t(m_mirror));} bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_same=b;if(!get8(p,e,b))return false;m_mirror=Mirror(b);return true;} private:uint8_t m_prg=0,m_chr=0;bool m_same=false;};

// Mapper 40 (NTDEC 2722): SMB2J cartridge conversions. The normal board has
// a fixed 6/4/5/?/7 8 KiB PRG layout, a switchable $C000 bank, and a 4096-M2
// one-shot IRQ. NES 2.0 submapper 1 adds the 2752 multicart outer register.


// Phase 26: additional conventional/discrete mapper families.
class Mapper28 final : public Mapper {
public:
    explicit Mapper28(const MapperConfig& c) : Mapper(c) {
        // A deterministic boot state matching the documented requirement that
        // the final 16 KiB be visible at $C000-$FFFF.
        m_mode = 0x0C; // fixed-$C000 mode, 32 KiB outer size, 1-screen lower
        const std::size_t banks32 = std::max<std::size_t>(1, c.prgRomSize / 0x8000);
        m_outer = uint8_t((banks32 - 1) & 0xFF);
    }
    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const uint8_t size = (m_mode >> 4) & 3;
        const uint8_t pmode = (m_mode >> 2) & 3;
        const std::size_t h = (a >> 14) & 1;
        std::size_t b16 = 0;
        if (pmode <= 1) {
            const std::size_t mask = (std::size_t(1) << size) - 1;
            b16 = (std::size_t(m_outer & ~uint8_t(mask)) << 1) |
                  (std::size_t(m_inner & mask) << 1) | h;
        } else if (pmode == 2) {
            if (h == 0) b16 = std::size_t(m_outer) << 1;
            else {
                const std::size_t mask = (std::size_t(1) << (size + 1)) - 1;
                b16 = (std::size_t(m_outer & ~uint8_t((std::size_t(1) << size) - 1)) << 1) |
                      (std::size_t(m_inner) & mask);
            }
        } else {
            if (h != 0) b16 = (std::size_t(m_outer) << 1) | 1;
            else {
                const std::size_t mask = (std::size_t(1) << (size + 1)) - 1;
                b16 = (std::size_t(m_outer & ~uint8_t((std::size_t(1) << size) - 1)) << 1) |
                      (std::size_t(m_inner) & mask);
            }
        }
        m = mapBank(b16, 0x4000, m_config.prgRomSize, a & 0x3FFF);
        return true;
    }
    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a >= 0x5000 && a <= 0x5FFF) { m_select = uint8_t((d & 0x80) | (d & 1)); return true; }
        if (a < 0x8000) return false;
        switch (m_select) {
        case 0x00:
            m_chr = d & 3;
            if ((m_mode & 3) < 2) m_mode = uint8_t((m_mode & ~1u) | ((d >> 4) & 1));
            break;
        case 0x01:
            m_inner = d & 0x0F;
            if ((m_mode & 3) < 2) m_mode = uint8_t((m_mode & ~1u) | ((d >> 4) & 1));
            break;
        case 0x80: m_mode = d & 0x3F; updateMirror(); break;
        case 0x81: m_outer = d; break;
        }
        updateMirror();
        return true;
    }
    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a,m); }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
    void saveState(std::vector<uint8_t>& o) const override { put8(o,m_select);put8(o,m_chr);put8(o,m_inner);put8(o,m_mode);put8(o,m_outer);put8(o,uint8_t(m_mirror)); }
    bool loadState(const uint8_t*& p,const uint8_t* e) override { uint8_t mm; if(!get8(p,e,m_select)||!get8(p,e,m_chr)||!get8(p,e,m_inner)||!get8(p,e,m_mode)||!get8(p,e,m_outer)||!get8(p,e,mm))return false; m_mirror=Mirror(mm); return true; }
private:
    void updateMirror(){ switch(m_mode&3){case 0:m_mirror=Mirror::OnescreenLo;break;case 1:m_mirror=Mirror::OnescreenHi;break;case 2:m_mirror=Mirror::Vertical;break;default:m_mirror=Mirror::Horizontal;break;} }
    uint8_t m_select=0,m_chr=0,m_inner=0,m_mode=0,m_outer=0;
};

class Mapper36 final : public Mapper {
public:
    explicit Mapper36(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prgLatched&3,0x8000,m_config.prgRomSize,a-0x8000);return true;}
    bool cpuReadRegister(uint16_t a,uint8_t&d)override{if((a&0xE100)!=0x4100)return false;d=uint8_t((d&0xCF)|((m_rr&3)<<4));return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if((a&0xE200)==0x4200){m_chr=d&0x0F;return true;}
        switch(a&0xE103){
        case 0x4100: if(m_inc)m_rr=uint8_t((m_rr+1)&3); else m_rr=m_inv?uint8_t((~m_pp)&3):m_pp; return true;
        case 0x4101: m_inv=(d&0x10)!=0; return true;
        case 0x4102: m_pp=(d>>4)&3; return true;
        case 0x4103: m_inc=(d&0x10)!=0; return true;
        default: break;
        }
        if(a>=0x8000){m_prgLatched=m_rr;return true;}
        return false;
    }
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_pp);put8(o,m_rr);put8(o,m_prgLatched);put8(o,m_chr);put8(o,m_inv);put8(o,m_inc);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_pp)||!get8(p,e,m_rr)||!get8(p,e,m_prgLatched)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_inv=b;if(!get8(p,e,b))return false;m_inc=b;return true;}
private:uint8_t m_pp=0,m_rr=0,m_prgLatched=0,m_chr=0;bool m_inv=false,m_inc=false;
};

class Mapper46 final : public Mapper {
public:
    explicit Mapper46(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if(a>=0x6000&&a<=0x7FFF){m_prg=uint8_t((m_prg&1)|((d<<1)&0x1E));m_chr=uint8_t((m_chr&7)|((d>>1)&0x78));return true;}
        if(a>=0x8000){m_prg=uint8_t((m_prg&0x1E)|(d&1));m_chr=uint8_t((m_chr&0x78)|((d>>4)&7));return true;}
        return false;
    }
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}
private:uint8_t m_prg=0,m_chr=0;
};

class Mapper40 final : public Mapper {
public:
    explicit Mapper40(const MapperConfig& c) : Mapper(c) {}
    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x6000 || !m_config.prgRomSize) return false;

        size_t bank = 0;
        if (m_config.submapper == 1 && m_nromMode) {
            // NTDEC 2752 outer register, P=1: regular NROM mode. P also
            // drives PRG A16 high. In NROM-128 mode pp drive A15..A14 and
            // CPU A14 is ignored, mirroring the selected 16 KiB bank. In
            // NROM-256 mode CPU A14 is restored; p1 selects A15 and p0 is
            // consequently don't-care.
            if (m_nrom256) {
                bank = 8u | (size_t((m_page >> 1) & 1) << 2)
                       | (size_t((a >> 14) & 1) << 1)
                       | size_t((a >> 13) & 1);
            } else {
                bank = 8u | (size_t(m_page & 3) << 1)
                       | size_t((a >> 13) & 1);
            }
        } else {
            // NTDEC 2722 / 2752 P=0 SMB2J mode. The five 8 KiB windows are
            // fixed to 6,4,5,?,7, with only $C000-$DFFF switchable.
            if (a < 0x8000) bank = 6;
            else if (a < 0xA000) bank = 4;
            else if (a < 0xC000) bank = 5;
            else if (a < 0xE000) bank = m_bank;
            else bank = 7;
        }
        m = mapBank(bank, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }
    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        const uint16_t r = a & 0xE000;
        if (r == 0x8000) { m_irqEnable=false; m_irq=false; m_irqCount=0; return true; }
        if (r == 0xA000) { m_irqEnable=true; m_irq=false; m_irqCount=0; return true; }
        if (r == 0xC000 && m_config.submapper == 1) {
            // NTDEC 2752: the outer register is address-latched. Address
            // bits are ...ppNPCCM: M=mirroring, CC=CHR A14..A13,
            // P=SMB2J/NROM mode + PRG A16, N=NROM-128/256, pp=outer PRG.
            m_mirror = (a & 0x0001) ? Mirror::Horizontal : Mirror::Vertical;
            m_chr = uint8_t((a >> 1) & 3);
            m_nromMode = (a & 0x0008) != 0;
            m_nrom256 = (a & 0x0010) != 0;
            m_page = uint8_t((a >> 5) & 3);
            return true;
        }
        if (r == 0xE000) { m_bank=d & 7; return true; }
        return false;
    }
    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const size_t s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize; if(!s)return false;
        m=mapBank(m_config.submapper==1?m_chr:0,0x2000,s,a); return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t&m) override { return m_config.chrRamSize && ppuMapRead(a,m); }
    bool mapPrgRam(uint16_t,uint32_t&,bool) const override { return false; }
    void clockCpu() override { if(m_irqEnable && !m_irq && ++m_irqCount>=4096) m_irq=true; }
    bool irqActive() const override { return m_irq; }
    void reset(bool hard) override { if(hard && m_config.submapper==1){m_chr=m_page=0;m_nromMode=false;m_nrom256=false;m_mirror=m_config.headerMirror;} m_irqEnable=false;m_irq=false;m_irqCount=0; }
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_bank);put8(o,m_irqEnable);put8(o,m_irq);put8(o,uint8_t(m_irqCount));put8(o,uint8_t(m_irqCount>>8));put8(o,m_chr);put8(o,m_page);put8(o,m_nromMode);put8(o,m_nrom256);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;if(!get8(p,e,m_bank)||!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irq=b;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqCount=uint16_t(lo)|(uint16_t(hi)<<8);if(!get8(p,e,m_chr)||!get8(p,e,m_page)||!get8(p,e,b))return false;m_nromMode=b;if(!get8(p,e,b))return false;m_nrom256=b;if(!get8(p,e,b))return false;m_mirror=Mirror(b);return true;}
private: uint8_t m_bank=0,m_chr=0,m_page=0; bool m_irqEnable=false,m_irq=false,m_nromMode=false,m_nrom256=false; uint16_t m_irqCount=0;
};

// Mapper 41 (Caltron 6-in-1): address-latched outer PRG/CHR/mirroring plus a
// bus-conflicted two-bit inner CHR register enabled only for PRG banks 4-7.
class Mapper41 final : public Mapper {
public:
    explicit Mapper41(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if(a>=0x6000&&a<=0x67FF){m_prg=a&3;m_innerEnable=(a&4)!=0;m_chrOuter=(a>>3)&3;m_mirror=(a&0x20)?Mirror::Horizontal:Mirror::Vertical;return true;}
        if(a>=0x8000&&m_innerEnable){m_chrInner=d&3;return true;} return false;
    }
    bool hasBusConflicts()const override{return true;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;size_t bank=(size_t(m_chrOuter)<<2)|(m_innerEnable?m_chrInner:0);m=mapBank(bank,0x2000,s,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}
    void reset(bool)override{m_prg=m_chrOuter=m_chrInner=0;m_innerEnable=false;m_mirror=m_config.headerMirror;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chrOuter);put8(o,m_chrInner);put8(o,m_innerEnable);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chrOuter)||!get8(p,e,m_chrInner)||!get8(p,e,b))return false;m_innerEnable=b;if(!get8(p,e,b))return false;m_mirror=Mirror(b);return true;}
private:uint8_t m_prg=0,m_chrOuter=0,m_chrInner=0;bool m_innerEnable=false;
};

// Mapper 42: FDS conversions with an 8 KiB PRG window at $6000, fixed upper
// 32 KiB, CHR bank, mirroring, and a free-running 15-bit M2 IRQ counter.
class Mapper42 final : public Mapper {
public: explicit Mapper42(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x6000||!m_config.prgRomSize)return false;if(a<0x8000)m=mapBank(m_prg,0x2000,m_config.prgRomSize,a-0x6000);else{size_t base=m_config.prgRomSize>0x8000?m_config.prgRomSize-0x8000:0;m=uint32_t(base+(a-0x8000));}return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0xE003){case 0x8000:m_chr=d&0x0F;return true;case 0xE000:m_prg=d&0x0F;return true;case 0xE001:m_mirror=(d&8)?Mirror::Horizontal:Mirror::Vertical;return true;case 0xE002:m_irqEnable=(d&2)!=0;if(!m_irqEnable)m_irqCounter=0;return true;default:return false;}}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_config.chrRomSize?m_chr:0,0x2000,s,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}
    void clockCpu()override{if(m_irqEnable)m_irqCounter=(m_irqCounter+1)&0x7FFF;}
    bool irqActive()const override{return m_irqEnable && (m_irqCounter&0x6000)==0x6000;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,m_irqEnable);put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqCounter=(uint16_t(lo)|(uint16_t(hi)<<8))&0x7FFF;if(!get8(p,e,b))return false;m_mirror=Mirror(b);return true;}
private:uint8_t m_prg=0,m_chr=0;bool m_irqEnable=false;uint16_t m_irqCounter=0;
};

// Mapper 43 (TONY-I/YS-612): unusual SMB2J FDS-conversion ROM layout with a
// 2 KiB expansion ROM at $5000, fixed 8 KiB windows, one selectable $C000
// bank, and a 12-bit M2 overflow IRQ.
class Mapper43 final : public Mapper {
public: explicit Mapper43(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{
        if(!m_config.prgRomSize)return false;
        if(a>=0x5000&&a<0x6000){m=mapBank(32,0x800,m_config.prgRomSize,a&0x7FF);return true;}
        size_t bank;if(a>=0x6000&&a<0x8000)bank=2;else if(a>=0x8000&&a<0xA000)bank=1;else if(a>=0xA000&&a<0xC000)bank=0;else if(a>=0xC000&&a<0xE000)bank=m_bank;else if(a>=0xE000)bank=9;else return false;
        m=mapBank(bank,0x2000,m_config.prgRomSize,a&0x1FFF);return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if((a&0x71FF)==0x4022){static const uint8_t lut[8]={4,3,4,4,4,7,5,6};m_bank=lut[d&7];return true;}
        if(a==0x4122||a==0x8122){m_irqEnable=(d&1)!=0;m_irq=false;m_irqCounter=0;return true;}return false;
    }
    bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}
    void clockCpu()override{if(m_irqEnable&&!m_irq){m_irqCounter=(m_irqCounter+1)&0x0FFF;if(m_irqCounter==0)m_irq=true;}}
    bool irqActive()const override{return m_irq;}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,m_bank);put8(o,m_irqEnable);put8(o,m_irq);put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;if(!get8(p,e,m_bank)||!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irq=b;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqCounter=(uint16_t(lo)|(uint16_t(hi)<<8))&0x0FFF;return true;}
private:uint8_t m_bank=4;bool m_irqEnable=false,m_irq=false;uint16_t m_irqCounter=0;
};

class Mapper86 final : public Mapper {public:explicit Mapper86(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_prg=(d>>4)&3;m_chr=(d&3)|((d>>4)&4);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper88 final : public Mapper {public:explicit Mapper88(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(m_config.id==154)m_mirror=(d&0x40)?Mirror::OnescreenHi:Mirror::OnescreenLo;if((a&0xE001)==0x8000){m_sel=d&7;return true;}if((a&0xE001)==0x8001){if(m_sel<=1)m_chr2[m_sel]=d>>1;else if(m_sel<=5)m_chr1[m_sel-2]=(d&0x3F)|0x40;else m_prg[m_sel-6]=d&0x3F;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x1000)m=mapBank(m_chr2[a>>11],0x800,s,a&0x7FF);else m=mapBank(m_chr1[(a-0x1000)>>10],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr2)put8(o,v);for(auto v:m_chr1)put8(o,v);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr2)if(!get8(p,e,v))return false;for(auto&v:m_chr1)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_sel=0,m_prg[2]={},m_chr2[2]={},m_chr1[4]={};};

class Mapper89 final : public Mapper {public:explicit Mapper89(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0xC000)return false;m_prg=(d>>4)&7;m_chr=(d&7)|((d>>4)&8);m_mirror=(d&8)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper93 final : public Mapper {public:explicit Mapper93(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=(d>>4)&7;return true;}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);}private:uint8_t m_prg=0;};

class Mapper95 final : public Mapper {public:explicit Mapper95(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_r[6]:a<0xC000?m_r[7]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(!(a&1))m_sel=d&7;else m_r[m_sel]=d;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x800)m=mapBank((m_r[0]&0xFE)>>1,0x800,s,a&0x7FF);else if(a<0x1000)m=mapBank((m_r[1]&0xFE)>>1,0x800,s,a&0x7FF);else m=mapBank(m_r[2+((a-0x1000)>>10)],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(a<0x2000||a>0x3EFF)return false;uint8_t p=(a&0xFFF)>>10,nt=((p<2?m_r[0]:m_r[1])>>5)&1;s=NametableSource::Ciram;m=uint32_t(nt)*0x400+(a&0x3FF);return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_r)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_r)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_r[8]={};};

// Mapper 91 / JY830623C and EJ-006-1 (Super Fighter III).
// Submapper 0 uses an unfiltered PPU-A12 64-rise IRQ and address-latched
// outer PRG/CHR banking. Submapper 1 uses a programmable M2-derived IRQ,
// selectable H/V mirroring, and no outer-bank register.
class Mapper91 final : public Mapper {
public:
    explicit Mapper91(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const std::size_t bankCount = std::max<std::size_t>(1, m_config.prgRomSize / 0x2000);
        std::size_t bank;
        if (a < 0xA000) bank = m_prg[0];
        else if (a < 0xC000) bank = m_prg[1];
        else bank = (a < 0xE000) ? 14u : 15u;
        if (m_config.submapper == 0) bank = (std::size_t(m_outerPrg) << 4) | (bank & 0x0F);
        m = mapBank(bank, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        (void)bankCount;
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a >= 0x6000 && a < 0x7000) {
            const uint16_t r = uint16_t(0x6000 | (a & (m_config.submapper == 1 ? 7 : 3)));
            if (r <= 0x6003) { m_chr[r & 3] = d; return true; }
            if (m_config.submapper == 1) {
                if (r == 0x6004) { if (!m_config.fourScreen) m_mirror = Mirror::Horizontal; return true; }
                if (r == 0x6005) { if (!m_config.fourScreen) m_mirror = Mirror::Vertical; return true; }
                if (r == 0x6006) { m_irqCounter = uint16_t((m_irqCounter & 0xFF00) | d); return true; }
                if (r == 0x6007) { m_irqCounter = uint16_t((m_irqCounter & 0x00FF) | (uint16_t(d) << 8)); return true; }
            }
            return false;
        }
        if (a >= 0x7000 && a < 0x8000) {
            const uint16_t r = uint16_t(0x7000 | (a & (m_config.submapper == 1 ? 7 : 3)));
            if (r == 0x7000) { m_prg[0] = d; return true; }
            if (r == 0x7001) { m_prg[1] = d; return true; }
            if (r == (m_config.submapper == 1 ? 0x7006 : 0x7002)) {
                m_irqEnabled = false; m_irqPending = false; return true;
            }
            if (r == (m_config.submapper == 1 ? 0x7007 : 0x7003)) {
                m_irqPending = false; m_irqEnabled = true; m_irqDivider = 0;
                if (m_config.submapper == 0) m_a12Count = 0;
                return true;
            }
            return false;
        }
        if (m_config.submapper == 0 && a >= 0x8000 && a < 0xA000) {
            m_outerChr = uint8_t(a & 1);
            m_outerPrg = uint8_t((a >> 1) & 3);
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t s = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!s) return false;
        std::size_t bank = m_chr[a >> 11];
        if (m_config.submapper == 0) bank |= std::size_t(m_outerChr) << 8;
        m = mapBank(bank, 0x800, s, a & 0x7FF);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a, m); }

    void notifyPpuAddress(uint16_t a, uint64_t) override {
        if (m_config.submapper != 0) return;
        const bool high = (a & 0x1000) != 0;
        if (m_irqEnabled && high && !m_lastA12) {
            if (++m_a12Count >= 64) { m_irqPending = true; m_irqEnabled = false; }
        }
        m_lastA12 = high;
    }

    void clockCpu() override {
        if (m_config.submapper != 1 || !m_irqEnabled) return;
        if (++m_irqDivider < 4) return;
        m_irqDivider = 0;
        if (m_irqCounter <= 5) { m_irqCounter = 0; m_irqPending = true; m_irqEnabled = false; }
        else m_irqCounter = uint16_t(m_irqCounter - 5);
    }

    bool irqActive() const override { return m_irqPending; }
    void reset(bool hard) override {
        if (!hard) { m_irqEnabled = false; m_irqPending = false; m_irqDivider = 0; m_a12Count = 0; }
    }
    void saveState(std::vector<uint8_t>& o) const override {
        for (auto v : m_prg) put8(o, v);
        for (auto v : m_chr) put8(o, v);
        put8(o, m_outerPrg); put8(o, m_outerChr); put8(o, uint8_t(m_irqCounter)); put8(o, uint8_t(m_irqCounter >> 8));
        put8(o, m_irqDivider); put8(o, m_a12Count); put8(o, m_irqEnabled); put8(o, m_irqPending); put8(o, m_lastA12);
        put8(o, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t lo, hi, b;
        for (auto& v : m_prg) if (!get8(p,e,v)) return false;
        for (auto& v : m_chr) if (!get8(p,e,v)) return false;
        if (!get8(p,e,m_outerPrg)||!get8(p,e,m_outerChr)||!get8(p,e,lo)||!get8(p,e,hi)) return false;
        m_irqCounter = uint16_t(lo) | (uint16_t(hi) << 8);
        if (!get8(p,e,m_irqDivider)||!get8(p,e,m_a12Count)||!get8(p,e,b)) return false;
        m_irqEnabled=b;
        if (!get8(p,e,b)) return false;
        m_irqPending=b;
        if (!get8(p,e,b)) return false;
        m_lastA12=b;
        if (!get8(p,e,b)) return false;
        m_mirror=static_cast<Mirror>(b);
        return true;
    }
private:
    uint8_t m_prg[2] = {};
    uint8_t m_chr[4] = {};
    uint8_t m_outerPrg = 0, m_outerChr = 0;
    uint16_t m_irqCounter = 0;
    uint8_t m_irqDivider = 0, m_a12Count = 0;
    bool m_irqEnabled = false, m_irqPending = false, m_lastA12 = false;
};

class Mapper96 final : public Mapper {public:explicit Mapper96(const MapperConfig&c):Mapper(c){m_mirror=Mirror::OnescreenLo;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&3;m_outer=(d>>2)&1;m_mirror=(d&4)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000||!m_config.chrRamSize)return false;uint8_t b=uint8_t((m_outer<<2)|m_inner);m=mapBank(b,0x800,m_config.chrRamSize,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return ppuMapRead(a,m);}void notifyPpuAddress(uint16_t a,uint64_t)override{if((a&0x3000)==0x2000)m_inner=(a>>8)&3;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_outer);put8(o,m_inner);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_outer)||!get8(p,e,m_inner)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_outer=0,m_inner=0;};

class Mapper97 final : public Mapper {public:explicit Mapper97(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=a-0x8000;else m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0xC000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&0x0F;if(!m_config.fourScreen)m_mirror=(d&0x80)?Mirror::Horizontal:Mirror::Vertical;return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0;};

class Mapper101 final : public Mapper {public:explicit Mapper101(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=(a-0x8000)%m_config.prgRomSize;return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000)return false;m_chr=d&3;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_chr);}private:uint8_t m_chr=0;};

class Mapper113 final : public Mapper {public:explicit Mapper113(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE100)!=0x4100&&a<0x8000)return false;m_prg=(d>>3)&7;m_chr=(d&7)|((d>>3)&8);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

// Sachen SA-72008 (Jovial Race, 72-pin release).  The board decodes a
// single low cartridge register: $4100 under mask $E100. D2 selects the
// 32 KiB PRG bank and D1-D0 select the 8 KiB CHR bank.  The older 60-pin
// Sachen 3009 board has more complex hardware, but its software also contains
// the SA-72008 write path and Mapper 133 compatibility conventionally models
// this simpler latch variant.
class Mapper133 final : public Mapper {
public:
    explicit Mapper133(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(m_prg, 0x8000, m_config.prgRomSize, a - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xE100) != 0x4100) return false;
        m_prg = uint8_t((d >> 2) & 1);
        m_chr = uint8_t(d & 3);
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && ppuMapRead(a, m);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_prg);
        put8(out, m_chr);
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        return get8(p, end, m_prg) && get8(p, end, m_chr);
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};

// Sachen 8259A/B/C family. Mappers 141/138/139 differ only in how the
// six mapper-controlled CHR address bits are wired to CHR-ROM. Registers are
// selected at $4100 and written at $4101 (mask $C101). Internal register 5
// selects a 32 KiB PRG bank; registers 0-4 select CHR; register 7 controls
// simple CHR mode and nametable mirroring.
// Sachen 8259D / mapper 137. This variant uses four 1 KiB switchable
// banks in the lower pattern table and fixes the upper 4 KiB to the end of
// CHR-ROM. Its 74-series glue gives each lower slot different CHR high bits,
// so it cannot use the A/B/C 2 KiB-bank rewiring used by Mapper8259 below.
class Mapper137 final : public Mapper {
public:
    explicit Mapper137(const MapperConfig& c) : Mapper(c) { updateMirror(); }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(uint8_t(m_reg[5] & 7), 0x8000, m_config.prgRomSize, a - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        const uint16_t reg = uint16_t(a & 0xC101);
        if (reg == 0x4100) {
            m_select = uint8_t(d & 7);
            return true;
        }
        if (reg == 0x4101) {
            m_reg[m_select] = uint8_t(d & 7);
            if (m_select == 7) updateMirror();
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000 || !m_config.chrRomSize) return false;
        if (a >= 0x1000) {
            const std::size_t fixedBase = m_config.chrRomSize > 0x1000 ? m_config.chrRomSize - 0x1000 : 0;
            m = uint32_t(fixedBase + (a & 0x0FFF));
            return true;
        }

        const uint8_t slot = uint8_t((a >> 10) & 3);
        const uint8_t low = uint8_t(m_reg[(m_reg[7] & 1) ? 0 : slot] & 7);
        uint8_t bank = low;
        switch (slot) {
        case 1: bank = uint8_t(low | ((m_reg[4] & 0x01) << 4)); break;
        case 2: bank = uint8_t(low | ((m_reg[4] & 0x02) << 3)); break;
        case 3:
            bank = uint8_t(low | ((m_reg[6] & 0x01) << 3) | ((m_reg[4] & 0x04) << 2));
            break;
        default: break;
        }
        m = mapBank(bank, 0x400, m_config.chrRomSize, a & 0x03FF);
        return true;
    }

    bool ppuMapWrite(uint16_t, uint32_t&) override { return false; }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    bool mapNametable(uint16_t a, NametableSource& source, uint32_t& mapped) const override {
        if (a < 0x2000 || a > 0x3EFF) return false;
        const uint8_t page = uint8_t((a & 0x0FFF) >> 10);
        uint8_t nt = 0;
        if (m_reg[7] & 1) {
            // 8259D simple mode fixes horizontal mirroring.
            nt = uint8_t((page >> 1) & 1);
        } else {
            switch ((m_reg[7] >> 1) & 3) {
            case 0: nt = uint8_t((page >> 1) & 1); break;       // horizontal
            case 1: nt = uint8_t(page & 1); break;              // vertical
            case 2: nt = uint8_t(page == 0 ? 0 : 1); break;     // (0,1,1,1)
            default: nt = 0; break;                              // one-screen A
            }
        }
        source = NametableSource::Ciram;
        mapped = uint32_t(nt) * 0x400u + (a & 0x3FFu);
        return true;
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_select = 0;
        std::fill(std::begin(m_reg), std::end(m_reg), uint8_t(0));
        updateMirror();
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_select);
        for (auto v : m_reg) put8(out, v);
        put8(out, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        uint8_t mirror = 0;
        if (!get8(p, end, m_select)) return false;
        for (auto& v : m_reg) if (!get8(p, end, v)) return false;
        if (!get8(p, end, mirror)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    void updateMirror() {
        if (m_reg[7] & 1) {
            m_mirror = Mirror::Horizontal;
            return;
        }
        switch ((m_reg[7] >> 1) & 3) {
        case 0: m_mirror = Mirror::Horizontal; break;
        case 1: m_mirror = Mirror::Vertical; break;
        case 3: m_mirror = Mirror::OnescreenLo; break;
        default: m_mirror = Mirror::Horizontal; break; // custom (0,1,1,1)
        }
    }

    uint8_t m_select = 0;
    uint8_t m_reg[8] = {};
};

class Mapper8259 final : public Mapper {
public:
    explicit Mapper8259(const MapperConfig& c) : Mapper(c) { updateMirror(); }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(uint8_t(m_reg[5] & 7), 0x8000, m_config.prgRomSize, a - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        const uint16_t reg = uint16_t(a & 0xC101);
        if (reg == 0x4100) {
            m_select = uint8_t(d & 7);
            return true;
        }
        if (reg == 0x4101) {
            m_reg[m_select] = uint8_t(d & 7);
            if (m_select == 7) updateMirror();
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;

        // Q-Boy's mapper-141 board replaces CHR-ROM with 8 KiB of unbanked
        // CHR-RAM. The 8259 address outputs are not relevant in that variant.
        if (!m_config.chrRomSize && m_config.chrRamSize) {
            m = uint32_t(a % m_config.chrRamSize);
            return true;
        }

        const uint8_t slot = uint8_t((a >> 11) & 3);
        const bool simple = (m_reg[7] & 1) != 0;
        const uint8_t chrReg = uint8_t((m_reg[simple ? 0 : slot] & 7) | ((m_reg[4] & 7) << 3));
        const unsigned shift = chrShift();
        const uint8_t leaked = uint8_t((a >> 11) & ((1u << shift) - 1u));
        const uint16_t bank = uint16_t((uint16_t(chrReg) << shift) | leaked);
        m = mapBank(bank, 0x800, size, a & 0x7FF);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        if (!m_config.chrRamSize) return false;
        return ppuMapRead(a, m);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    bool mapNametable(uint16_t a, NametableSource& source, uint32_t& mapped) const override {
        if (a < 0x2000 || a > 0x3EFF) return false;
        const uint8_t page = uint8_t((a & 0x0FFF) >> 10);
        uint8_t nt = 0;
        if (m_reg[7] & 1) {
            // Simple mode forces vertical mirroring on 8259A/B/C.
            nt = uint8_t(page & 1);
        } else {
            switch ((m_reg[7] >> 1) & 3) {
            case 0: nt = uint8_t(page & 1); break;              // vertical
            case 1: nt = uint8_t((page >> 1) & 1); break;       // horizontal
            case 2: nt = uint8_t(page == 0 ? 0 : 1); break;     // (0,1,1,1)
            default: nt = 0; break;                              // one-screen A
            }
        }
        source = NametableSource::Ciram;
        mapped = uint32_t(nt) * 0x400u + (a & 0x3FFu);
        return true;
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_select = 0;
        std::fill(std::begin(m_reg), std::end(m_reg), uint8_t(0));
        updateMirror();
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_select);
        for (auto v : m_reg) put8(out, v);
        put8(out, static_cast<uint8_t>(m_mirror));
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        uint8_t mirror = 0;
        if (!get8(p, end, m_select)) return false;
        for (auto& v : m_reg) if (!get8(p, end, v)) return false;
        if (!get8(p, end, mirror)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    unsigned chrShift() const {
        switch (m_config.id) {
        case 135: case 141: return 1; // 8259A (135 is the legacy CHR-ROM assignment)
        case 139: return 2; // 8259C
        default: return 0;  // mapper 138, 8259B
        }
    }
    void updateMirror() {
        if (m_reg[7] & 1) {
            m_mirror = Mirror::Vertical;
            return;
        }
        switch ((m_reg[7] >> 1) & 3) {
        case 0: m_mirror = Mirror::Vertical; break;
        case 1: m_mirror = Mirror::Horizontal; break;
        case 3: m_mirror = Mirror::OnescreenLo; break;
        default: m_mirror = Mirror::Vertical; break; // custom (0,1,1,1) via mapNametable()
        }
    }

    uint8_t m_select = 0;
    uint8_t m_reg[8] = {};
};

// TXC 22111 / mapper 132.  The 05-00002-010 ASIC exposes four low
// registers through the $4100 region.  Reads matching mask $E100 return the
// protection value (R1 XOR R2) with D6 set.  A write anywhere in
// $8000-$FFFF copies register 2 to the output latch; output bits 3-2 select
// the 32 KiB PRG bank and the complete latched value feeds the 8 KiB CHR
// bank inputs (naturally masked by the installed ROM size).
class Mapper132 final : public Mapper {
public:
    explicit Mapper132(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(uint8_t(m_output >> 2), 0x8000, m_config.prgRomSize, a - 0x8000);
        return true;
    }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if ((a & 0xE100) != 0x4100) return false;
        d = uint8_t((m_reg[1] ^ m_reg[2]) | 0x40);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xE100) == 0x4100) {
            m_reg[a & 3] = d;
            return true;
        }
        if (a >= 0x8000) {
            m_output = m_reg[2];
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_output, 0x2000, size, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && ppuMapRead(a, m);
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void reset(bool hard) override {
        if (!hard) return;
        for (auto& r : m_reg) r = 0;
        m_output = 0;
    }

    void saveState(std::vector<uint8_t>& out) const override {
        for (auto r : m_reg) put8(out, r);
        put8(out, m_output);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        for (auto& r : m_reg) if (!get8(p,e,r)) return false;
        return get8(p,e,m_output);
    }

private:
    uint8_t m_reg[4] = {};
    uint8_t m_output = 0;
};

// Sachen JV001 / mapper 136. The JV001 protection ASIC exposes a six-bit
// register through $4100-$4103. D6-D7 are not driven on reads. Register writes
// can either load a staged six-bit input (optionally inverting D0-D3) or
// increment the low nibble. A CPU write anywhere in $8000-$FFFF copies the
// protection register into the mapper output latch; output D4 selects the
// 32 KiB PRG bank and D2-D0 select the 8 KiB CHR bank.
class Mapper136 final : public Mapper {
public:
    explicit Mapper136(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(uint8_t((m_output >> 4) & 1), 0x8000, m_config.prgRomSize, a - 0x8000);
        return true;
    }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if ((a & 0xE100) != 0x4100) return false;
        uint8_t value = uint8_t(m_reg & 0x3F);
        if (m_invert) value ^= 0x30; // readback inverts register D4-D5
        d = uint8_t((d & 0xC0) | value); // D6-D7 remain CPU open bus
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xE100) == 0x4100) {
            switch (a & 3) {
            case 0:
                if (!m_mode) {
                    m_reg = uint8_t(m_input & 0x3F);
                    if (m_invert) m_reg ^= 0x0F; // write path inverts D0-D3
                } else {
                    m_reg = uint8_t((m_reg & 0x30) | ((m_reg + 1) & 0x0F));
                }
                break;
            case 1: m_invert = (d & 1) != 0; break;
            case 2: m_input = uint8_t(d & 0x3F); break;
            case 3: m_mode = (d & 1) != 0; break;
            }
            return true;
        }
        if (a >= 0x8000) {
            m_output = m_reg;
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(uint8_t(m_output & 7), 0x2000, size, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && ppuMapRead(a, m);
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_reg); put8(out, m_input); put8(out, m_output);
        put8(out, m_invert); put8(out, m_mode);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t b = 0;
        if (!get8(p,e,m_reg) || !get8(p,e,m_input) || !get8(p,e,m_output) || !get8(p,e,b)) return false;
        m_invert = b != 0;
        if (!get8(p,e,b)) return false;
        m_mode = b != 0;
        return true;
    }

private:
    uint8_t m_reg = 0;
    uint8_t m_input = 0;
    uint8_t m_output = 0;
    bool m_invert = false;
    bool m_mode = false;
};

// BMC 9999999-in-1 / mapper 213.  The multicart latches CPU address
// lines on writes in $8000-$FFFF; the data byte is not connected.  A2-A1
// select one of four 32 KiB PRG banks and A5-A3 select one of eight 8 KiB
// CHR banks.  Mirroring is fixed by the cartridge header.
class Mapper213 final : public Mapper {
public:
    explicit Mapper213(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = mapBank(m_prg, 0x8000, m_config.prgRomSize, a - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t, uint64_t) override {
        if (a < 0x8000) return false;
        m_prg = uint8_t((a >> 1) & 0x03);
        m_chr = uint8_t((a >> 3) & 0x07);
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && ppuMapRead(a, m);
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& o) const override { put8(o, m_prg); put8(o, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* e) override { return get8(p,e,m_prg) && get8(p,e,m_chr); }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};

class Mapper140 final : public Mapper {public:explicit Mapper140(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_prg=(d>>4)&3;m_chr=d&0x0F;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

// Sachen SA-72007 (Sidewinder).  The board is CNROM-like but decodes a
// low cartridge register instead of overlapping PRG-ROM: writes matching
// $4100 under mask $E100 latch D7 as CHR A13.  PRG is fixed and there are
// no bus conflicts.

// Mapper 143 / Sachen TCA-01.  The cartridge is otherwise NROM-256 with
// fixed 32 KiB PRG and 8 KiB CHR.  Its protection ASIC responds throughout
// $4100-$5FFF; addresses whose A8 is high (the $4100 + k*$200 aliases)
// return the complemented low six address bits with D6 forced high.  Other
// addresses in the protection window return zero.
// Mapper 142 / Kaiser KS7032.  The KS202 ASIC provides four switchable
// 8 KiB PRG windows ($6000-$DFFF), a fixed final bank at $E000, unbanked
// 8 KiB CHR-RAM, and a VRC3-like 16-bit cycle IRQ.  Bank data is four bits;
// the register selected by $E000 chooses which of the four switchable PRG
// windows is updated by $F000.
// Mapper 125 / UNL-LH32. FDS-conversion board used by Monty no Doki
// Doki Daidassou. $6000-$7FFF is a switchable 8 KiB PRG window; the upper
// 32 KiB is fixed to the final ROM quarter, with 8 KiB WRAM overlaid at
// $C000-$DFFF. Only an exact write to $6000 changes the low PRG bank.
class Mapper125 final : public Mapper {
public:
    explicit Mapper125(const MapperConfig& c) : Mapper(c) { reset(true); }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (!m_config.prgRomSize) return false;
        if (addr >= 0x6000 && addr < 0x8000) {
            mapped = mapBank(m_bank, 0x2000, m_config.prgRomSize, addr & 0x1FFF);
            return true;
        }
        if (addr >= 0x8000) {
            const std::size_t window = std::min<std::size_t>(0x8000, m_config.prgRomSize);
            const std::size_t base = m_config.prgRomSize - window;
            mapped = uint32_t(base + ((addr - 0x8000) % window));
            return true;
        }
        return false;
    }

    bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool) const override {
        if (addr < 0xC000 || addr >= 0xE000 || !m_config.prgRamSize) return false;
        mapped = uint32_t((addr - 0xC000) % m_config.prgRamSize);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr != 0x6000) return false;
        m_bank = data & 0x0F;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000 || !m_config.chrRamSize) return false;
        mapped = uint32_t(addr % m_config.chrRamSize);
        return true;
    }
    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override { return ppuMapRead(addr, mapped); }

    void reset(bool) override { m_bank = 0x0F; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_bank); }
    bool loadState(const uint8_t*& p, const uint8_t* e) override { return get8(p,e,m_bank); }

private:
    uint8_t m_bank = 0x0F;
};

class Mapper142 final : public Mapper {
public:
    explicit Mapper142(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (!m_config.prgRomSize) return false;
        uint8_t bank = 0;
        if (addr >= 0x6000 && addr < 0x8000) bank = m_prg[4];
        else if (addr >= 0x8000 && addr < 0xA000) bank = m_prg[1];
        else if (addr >= 0xA000 && addr < 0xC000) bank = m_prg[2];
        else if (addr >= 0xC000 && addr < 0xE000) bank = m_prg[3];
        else if (addr >= 0xE000) {
            const size_t banks = std::max<size_t>(1, m_config.prgRomSize / 0x2000);
            bank = uint8_t(banks - 1);
        } else return false;
        mapped = mapBank(bank, 0x2000, m_config.prgRomSize, addr & 0x1FFF);
        return true;
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr < 0x8000) return false;
        switch (addr & 0xF000) {
        case 0x8000: setReloadNibble(0, data); return true;
        case 0x9000: setReloadNibble(4, data); return true;
        case 0xA000: setReloadNibble(8, data); return true;
        case 0xB000: setReloadNibble(12, data); return true;
        case 0xC000:
            m_irqEnabled = (data & 0x02) != 0;
            if (m_irqEnabled) m_irqCounter = m_irqReload;
            m_irqPending = false;
            return true;
        case 0xD000:
            m_irqPending = false;
            return true;
        case 0xE000:
            m_select = data & 0x07;
            return true;
        case 0xF000:
            if (m_select < 5)
                m_prg[m_select] = uint8_t((m_prg[m_select] & 0xF0) | (data & 0x0F));
            return true;
        default:
            return false;
        }
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000 || !m_config.chrRamSize) return false;
        mapped = uint32_t(addr % m_config.chrRamSize);
        return true;
    }
    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return ppuMapRead(addr, mapped);
    }

    void clockCpu() override {
        if (!m_irqEnabled) return;
        ++m_irqCounter;
        if (m_irqCounter == 0) {
            m_irqPending = true;
            m_irqEnabled = false;
            m_irqCounter = m_irqReload;
        }
    }
    bool irqActive() const override { return m_irqPending; }

    void reset(bool hard) override {
        m_irqEnabled = false;
        m_irqPending = false;
        m_irqCounter = 0;
        m_irqReload = 0;
        m_select = 0;
        if (hard) for (auto& bank : m_prg) bank = 0;
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, uint8_t(m_irqReload)); put8(out, uint8_t(m_irqReload >> 8));
        put8(out, uint8_t(m_irqCounter)); put8(out, uint8_t(m_irqCounter >> 8));
        put8(out, m_irqEnabled); put8(out, m_irqPending); put8(out, m_select);
        for (auto bank : m_prg) put8(out, bank);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        uint8_t lo, hi, enabled, pending;
        if (!get8(p,e,lo) || !get8(p,e,hi)) return false;
        m_irqReload = uint16_t(lo) | (uint16_t(hi) << 8);
        if (!get8(p,e,lo) || !get8(p,e,hi)) return false;
        m_irqCounter = uint16_t(lo) | (uint16_t(hi) << 8);
        if (!get8(p,e,enabled) || !get8(p,e,pending) || !get8(p,e,m_select)) return false;
        m_irqEnabled = enabled != 0; m_irqPending = pending != 0;
        for (auto& bank : m_prg) if (!get8(p,e,bank)) return false;
        return true;
    }

private:
    void setReloadNibble(unsigned shift, uint8_t data) {
        const uint16_t mask = uint16_t(0x000F) << shift;
        m_irqReload = uint16_t((m_irqReload & ~mask) | (uint16_t(data & 0x0F) << shift));
    }

    uint16_t m_irqReload = 0;
    uint16_t m_irqCounter = 0;
    bool m_irqEnabled = false;
    bool m_irqPending = false;
    uint8_t m_select = 0;
    uint8_t m_prg[5] = {};
};

class Mapper143 final : public Mapper {
public:
    explicit Mapper143(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = uint32_t(a - 0x8000) % uint32_t(m_config.prgRomSize);
        return true;
    }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if (a < 0x4100 || a > 0x5FFF) return false;
        d = (a & 0x0100) ? uint8_t((~a & 0x3F) | 0x40) : uint8_t(0x00);
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = uint32_t(a) % uint32_t(size);
        return true;
    }
    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && ppuMapRead(a, m);
    }
    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }
};

class Mapper145 final : public Mapper {
public:
    explicit Mapper145(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        m = uint32_t(a - 0x8000) % uint32_t(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xE100) != 0x4100) return false;
        m_chr = uint8_t((d >> 7) & 1);
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        m = mapBank(m_chr, 0x2000, size, a);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && ppuMapRead(a, m);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void saveState(std::vector<uint8_t>& out) const override { put8(out, m_chr); }
    bool loadState(const uint8_t*& p, const uint8_t* end) override { return get8(p, end, m_chr); }

private:
    uint8_t m_chr = 0;
};

class Mapper184 final : public Mapper {public:explicit Mapper184(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=(a-0x8000)%m_config.prgRomSize;return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x7FFF)return false;m_chr[0]=d&7;m_chr[1]=0x80|((d>>4)&7);return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>12],0x1000,s,a&0xFFF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_chr[0]);put8(o,m_chr[1]);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_chr[0])&&get8(p,e,m_chr[1]);}private:uint8_t m_chr[2]={};};

// Mapper 206: Namco 118 / Tengen MIMIC-1 (DxROM family). NES 2.0
// submapper 1 identifies the 3407/3417/3451 boards where a 32 KiB PRG-ROM
// is physically unbanked: CPU A13/A14 connect directly to PRG-ROM, so writes
// to the two nominal PRG bank registers must not change $8000-$FFFF.
class Mapper206 final : public Mapper {
public:
    explicit Mapper206(const MapperConfig& c) : Mapper(c) {}

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        if (m_config.nes20 && m_config.submapper == 1) {
            const std::size_t window = std::min<std::size_t>(0x8000, m_config.prgRomSize);
            m = uint32_t((a - 0x8000) % window);
            return true;
        }
        const std::size_t n = std::max<std::size_t>(1, m_config.prgRomSize / 0x2000);
        const std::size_t b = a < 0xA000 ? m_prg[0] : a < 0xC000 ? m_prg[1] : a < 0xE000 ? (n > 1 ? n - 2 : 0) : n - 1;
        m = mapBank(b, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xE001) == 0x8000) { m_sel = d & 7; return true; }
        if ((a & 0xE001) == 0x8001) {
            if (m_sel <= 1) m_chr2[m_sel] = d >> 1;
            else if (m_sel <= 5) m_chr1[m_sel - 2] = d;
            else if (!(m_config.nes20 && m_config.submapper == 1)) m_prg[m_sel - 6] = d;
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const auto s = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!s) return false;
        if (a < 0x1000) m = mapBank(m_chr2[a >> 11], 0x800, s, a & 0x7FF);
        else m = mapBank(m_chr1[(a - 0x1000) >> 10], 0x400, s, a & 0x3FF);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a, m); }
    void saveState(std::vector<uint8_t>& o) const override { put8(o, m_sel); for (auto v : m_prg) put8(o, v); for (auto v : m_chr2) put8(o, v); for (auto v : m_chr1) put8(o, v); }
    bool loadState(const uint8_t*& p, const uint8_t* e) override { if (!get8(p, e, m_sel)) return false; for (auto& v : m_prg) if (!get8(p, e, v)) return false; for (auto& v : m_chr2) if (!get8(p, e, v)) return false; for (auto& v : m_chr1) if (!get8(p, e, v)) return false; return true; }

private:
    uint8_t m_sel = 0, m_prg[2] = {}, m_chr2[2] = {}, m_chr1[4] = {};
};

class Mapper232 final : public Mapper {public:explicit Mapper232(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t b=(size_t(m_block)<<2)|(a<0xC000?m_page:3);m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(a>=0xC000)m_page=d&3;else m_block=m_config.submapper==1?(((d>>4)&1)|((d>>2)&2)):((d>>3)&3);return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_block);put8(o,m_page);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_block)&&get8(p,e,m_page);}private:uint8_t m_block=0,m_page=0;};

class Mapper240 final : public Mapper {public:explicit Mapper240(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x4020||a>0x5FFF)return false;m_prg=(d>>4)&0x0F;m_chr=d&0x0F;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};
class Mapper241 final : public Mapper {public:explicit Mapper241(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d;return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);}private:uint8_t m_prg=0;};
class Mapper242 final : public Mapper {public:explicit Mapper242(const MapperConfig&c):Mapper(c){m_mirror=Mirror::Vertical;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t,uint64_t)override{if(a<0x8000)return false;m_prg=(a>>3)&0x0F;m_mirror=(a&2)?Mirror::Horizontal:Mirror::Vertical;return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0;};
class Mapper246 final : public Mapper {public:explicit Mapper246(const MapperConfig&c):Mapper(c){m_prg[3]=0xFF;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg[(a-0x8000)>>13],0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x6000||a>0x67FF)return false;uint8_t r=a&7;if(r<=3)m_prg[r]=d;else m_chr[r&3]=d;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;}void saveState(std::vector<uint8_t>&o)const override{for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;return true;}private:uint8_t m_prg[4]={0,0,0,0xFF},m_chr[4]={};};

// Phase 32: additional legacy mapper gaps.
// Mapper 81 / NTDEC N715021. The hardware latches CPU address A3-A0 on
// writes in $8000-$FFFF; data is ignored.
class Mapper81 final : public Mapper {
public:
    explicit Mapper81(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;
        const size_t bank=(a<0xC000)?m_prg:std::max<size_t>(1,m_config.prgRomSize/0x4000)-1;
        m=mapBank(bank,0x4000,m_config.prgRomSize,a&0x3FFF);return true;
    }
    bool cpuWrite(uint16_t a,uint8_t,uint64_t) override {
        if(a<0x8000)return false;
        m_prg=uint8_t((a>>2)&3);
        m_chr=uint8_t(a&3);
        return true;
    }
    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000)return false;
        auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!z)return false;
        m=mapBank(m_chr,0x2000,z,a);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void reset(bool hard) override{if(hard)m_prg=m_chr=0;}
    void saveState(std::vector<uint8_t>&o) const override{put8(o,m_prg);put8(o,m_chr);}
    bool loadState(const uint8_t*&p,const uint8_t*e) override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}
private:uint8_t m_prg=0,m_chr=0;
};

// Mapper 103 / pirate Doki Doki Panic FDS conversion.
class Mapper103 final : public Mapper {
public:
    explicit Mapper103(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(!m_config.prgRomSize)return false;
        if(a>=0x6000&&a<0x8000){if(!m_ramDisable)return false;m=mapBank(m_bank,0x2000,m_config.prgRomSize,a-0x6000);return true;}
        if(a>=0x8000&&a<0xB800){m=uint32_t(0x18000+(a-0x8000));return m<m_config.prgRomSize;}
        if(a>=0xB800&&a<0xD800){if(!m_ramDisable)return false;m=uint32_t(0x1B800+(a-0xB800));return m<m_config.prgRomSize;}
        if(a>=0xD800){m=uint32_t(0x1D800+(a-0xD800));return m<m_config.prgRomSize;}
        return false;
    }
    bool mapPrgRam(uint16_t a,uint32_t& m,bool write) const override {
        const bool r0=a>=0x6000&&a<0x8000, r1=a>=0xB800&&a<0xD800;
        if((!r0&&!r1)||!m_config.prgRamSize)return false;
        if(!write&&m_ramDisable)return false;
        m=uint32_t((a-(r0?0x6000:0xB800))%m_config.prgRamSize);return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a>=0x8000&&a<0x9000){m_bank=d&0x0F;return true;}
        if(a>=0xE000&&a<0xF000){m_mirror=(d&8)?Mirror::Horizontal:Mirror::Vertical;return true;}
        if(a>=0xF000){m_ramDisable=(d&0x10)!=0;return true;}
        return false;
    }
    void saveState(std::vector<uint8_t>&o) const override{put8(o,m_bank);put8(o,m_ramDisable);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e) override{uint8_t b,mi;if(!get8(p,e,m_bank)||!get8(p,e,b)||!get8(p,e,mi))return false;m_ramDisable=b;m_mirror=Mirror(mi);return true;}
private:uint8_t m_bank=0;bool m_ramDisable=false;
};

// Mapper 107: 32 KiB PRG + 8 KiB CHR latch.
class Mapper107 final : public Mapper {
public:
    explicit Mapper107(const MapperConfig& c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank((m_latch>>1)&3,0x8000,m_config.prgRomSize,a-0x8000);return true;}
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override{if(a<0x8000)return false;m_latch=d;return true;}
    bool ppuMapRead(uint16_t a,uint32_t& m) override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_latch&7,0x2000,z,a);return true;}
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void reset(bool hard) override{if(hard)m_latch=0xFF;}
    void saveState(std::vector<uint8_t>&o) const override{put8(o,m_latch);}
    bool loadState(const uint8_t*&p,const uint8_t*e) override{return get8(p,e,m_latch);}
private:uint8_t m_latch=0xFF;
};


// Phase 33: mapper 106, 108 and 112 gap closure.
// Mapper 106: discrete SMB3 bootleg. Sixteen registers are mirrored through
// $8000-$FFFF by A3-A0; IRQ is a 16-bit M2 up-counter that stops at $FFFF.
class Mapper106 final : public Mapper {
public:
    explicit Mapper106(const MapperConfig& c):Mapper(c) { reset(true); }
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;
        m=mapBank(m_prg[(a-0x8000)>>13],0x2000,m_config.prgRomSize,a&0x1FFF);return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        if(a<0x8000)return false;
        switch(a&0x000F){
        case 0: case 1: case 2: case 3: m_chr[a&3]=uint8_t((d&0x7E)|(a&1)); break;
        case 4: case 5: case 6: case 7: m_chr[a&7]=d&0x7F; break;
        case 8: m_prg[0]=uint8_t(0x10|(d&0x0F)); break;
        case 9: m_prg[1]=d&0x1F; break;
        case 10:m_prg[2]=d&0x1F; break;
        case 11:m_prg[3]=uint8_t(0x10|(d&0x0F)); break;
        case 12:m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical; break;
        case 13:m_irqCounter=0;m_irqEnable=false;m_irq=false; break;
        case 14:m_irqCounter=uint16_t((m_irqCounter&0xFF00)|d); break;
        case 15:m_irqCounter=uint16_t((m_irqCounter&0x00FF)|(uint16_t(d)<<8));m_irqEnable=true;m_irq=(m_irqCounter==0xFFFF); break;
        }
        return true;
    }
    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000)return false;
        auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!z)return false;
        m=mapBank(m_chr[a>>10],0x400,z,a&0x3FF);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void clockCpu() override {
        if(m_irqCounter!=0xFFFF){++m_irqCounter;if(m_irqCounter==0xFFFF&&m_irqEnable)m_irq=true;}
    }
    bool irqActive() const override{return m_irq;}
    void reset(bool hard) override {
        if(!hard)return;
        // The 74LS670 registers are observed to power up as all ones.
        for(auto&v:m_chr)v=0x7F;
        m_prg[0]=m_prg[3]=0x1F;m_prg[1]=m_prg[2]=0x0F;
        m_irqCounter=0;m_irqEnable=false;m_irq=false;m_mirror=m_config.headerMirror;
    }
    void saveState(std::vector<uint8_t>&o) const override {
        for(auto v:m_prg)put8(o,v);
        for(auto v:m_chr)put8(o,v);
        put8(o,uint8_t(m_mirror));
        put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));put8(o,m_irqEnable);put8(o,m_irq);
    }
    bool loadState(const uint8_t*&p,const uint8_t*e) override {
        for(auto&v:m_prg)if(!get8(p,e,v))return false;
        for(auto&v:m_chr)if(!get8(p,e,v))return false;
        uint8_t mi,lo,hi,en,irq;if(!get8(p,e,mi)||!get8(p,e,lo)||!get8(p,e,hi)||!get8(p,e,en)||!get8(p,e,irq))return false;
        m_mirror=Mirror(mi);m_irqCounter=uint16_t(lo)|(uint16_t(hi)<<8);m_irqEnable=en;m_irq=irq;return true;
    }
private:
    uint8_t m_prg[4]={};uint8_t m_chr[8]={};uint16_t m_irqCounter=0;bool m_irqEnable=false,m_irq=false;
};

// Mapper 108: four related Whirlwind Manu FDS-conversion boards. NES 2.0
// submappers select the exact write decode; legacy submapper 0 uses the
// documented CHR/mirroring heuristic.
class Mapper108 final : public Mapper {
public:
    explicit Mapper108(const MapperConfig& c):Mapper(c){m_variant=resolveVariant();}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(!m_config.prgRomSize)return false;
        if(a>=0x6000&&a<0x8000){
            size_t b=(m_variant==4)?(std::max<size_t>(1,m_config.prgRomSize/0x2000)-1):m_prg;
            m=mapBank(b,0x2000,m_config.prgRomSize,a-0x6000);return true;
        }
        if(a>=0x8000){
            const size_t tail=std::min<size_t>(0x8000,m_config.prgRomSize);
            m=uint32_t(m_config.prgRomSize-tail+(a-0x8000));return true;
        }
        return false;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        bool hit=false;
        switch(m_variant){
        case 1: hit=a>=0xF000; if(hit)m_prg=d; break;
        case 2: hit=a>=0xE000; if(hit){m_prg=d;m_chr=d;} break;
        case 3: hit=a>=0x8000; if(hit)m_prg=d; break;
        case 4: hit=a>=0x8000; if(hit)m_chr=d&1; break;
        }
        return hit;
    }
    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000)return false;
        auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!z)return false;
        const size_t b=(m_config.chrRomSize?m_chr:0);m=mapBank(b,0x2000,z,a);return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    bool implementationSupported() const override{return m_variant>=1&&m_variant<=4;}
    void reset(bool hard) override{if(hard){m_prg=0;m_chr=0;m_variant=resolveVariant();}}
    void saveState(std::vector<uint8_t>&o) const override{put8(o,m_prg);put8(o,m_chr);put8(o,m_variant);}
    bool loadState(const uint8_t*&p,const uint8_t*e) override{return get8(p,e,m_prg)&&get8(p,e,m_chr)&&get8(p,e,m_variant);}
private:
    uint8_t m_prg=0,m_chr=0,m_variant=0;
    uint8_t resolveVariant() const {
        if(m_config.submapper>=1&&m_config.submapper<=4)return m_config.submapper;
        if(m_config.submapper!=0)return 0;
        if(m_config.chrRomSize==0)return m_config.headerMirror==Mirror::Horizontal?1:3;
        return m_config.chrRomSize>0x4000?2:4;
    }
};

// Mapper 112 / NTDEC Asder. A selected internal register is written through
// $A000; PRG uses two switchable 8 KiB banks plus a fixed final 16 KiB, while
// CHR is 2K+2K+1K+1K+1K+1K.
class Mapper112 final : public Mapper {
public:
    explicit Mapper112(const MapperConfig& c):Mapper(c){reset(true);}
    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;
        size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);
        size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;
        m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t) override {
        switch(a&0xE001){
        case 0x8000:m_sel=d&7;return true;
        case 0xA000:
            if(m_sel<2)m_prg[m_sel]=d;
            else if(m_sel<4)m_chr2[m_sel-2]=d>>1;
            else m_chr1[m_sel-4]=d;
            return true;
        case 0xE000:m_mirror=(d&1)?Mirror::Horizontal:Mirror::Vertical;return true;
        default:return false;
        }
    }
    bool ppuMapRead(uint16_t a,uint32_t& m) override {
        if(a>=0x2000)return false;
        auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;
        if(!z)return false;
        if(a<0x1000)m=mapBank(m_chr2[a>>11],0x800,z,a&0x7FF);
        else m=mapBank(m_chr1[(a-0x1000)>>10],0x400,z,a&0x3FF);
        return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override{return m_config.chrRamSize&&ppuMapRead(a,m);}
    void reset(bool hard) override{if(hard){m_sel=0;m_prg[0]=m_prg[1]=0;for(auto&v:m_chr2)v=0;for(auto&v:m_chr1)v=0;m_mirror=m_config.headerMirror;}}
    void saveState(std::vector<uint8_t>&o) const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr2)put8(o,v);for(auto v:m_chr1)put8(o,v);put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e) override{uint8_t mi;if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr2)if(!get8(p,e,v))return false;for(auto&v:m_chr1)if(!get8(p,e,v))return false;if(!get8(p,e,mi))return false;m_mirror=Mirror(mi);return true;}
private:uint8_t m_sel=0,m_prg[2]={},m_chr2[2]={},m_chr1[4]={};
};

// Mapper 105 - NES-EVENT (Nintendo World Championships 1990).
// MMC1-derived board with event-specific PRG wiring and a 30-bit M2 timer.
class Mapper105 final : public Mapper {
public:
    explicit Mapper105(const MapperConfig& c) : Mapper(c) { updateMirror(); }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if(a < 0x8000 || !m_config.prgRomSize) return false;
        // Before the timer is started for the first time, the first 32 KiB is
        // hard-wired so the tournament bootstrap cannot bank itself away.
        if(!m_started) { m = uint32_t(a - 0x8000) % uint32_t(m_config.prgRomSize); return true; }

        const uint8_t eventBank = (m_chr0 >> 1) & 7;
        if(eventBank < 4) {
            m = mapBank(eventBank, 0x8000, std::min<std::size_t>(m_config.prgRomSize, 0x20000), a - 0x8000);
            return true;
        }

        // Values 4-7 select normal MMC1 behavior in the upper 128 KiB.
        const std::size_t base = m_config.prgRomSize > 0x20000 ? 0x20000 : 0;
        const std::size_t size = std::max<std::size_t>(0x4000, m_config.prgRomSize - base);
        const uint8_t mode = (m_ctrl >> 2) & 3;
        if(mode < 2) {
            m = uint32_t(base + mapBank((m_prg & 0x0E) >> 1, 0x8000, size, a - 0x8000));
        } else if(mode == 2) {
            m = a < 0xC000 ? uint32_t(base + (a - 0x8000))
                           : uint32_t(base + mapBank(m_prg & 0x0F, 0x4000, size, a - 0xC000));
        } else {
            m = a < 0xC000 ? uint32_t(base + mapBank(m_prg & 0x0F, 0x4000, size, a - 0x8000))
                           : uint32_t(base + size - 0x4000 + (a - 0xC000));
        }
        m %= uint32_t(m_config.prgRomSize);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t cyc) override {
        if(a < 0x8000) return false;
        if(d & 0x80) {
            if(cyc != kNoCpuCycle) m_lastWriteCycle = cyc;
            m_shift = 0x10; m_ctrl |= 0x0C; updateMirror(); return true;
        }
        if(cyc != kNoCpuCycle) {
            if(m_lastWriteCycle != kNoCpuCycle && cyc <= m_lastWriteCycle + 1) return true;
            m_lastWriteCycle = cyc;
        }
        const bool complete = (m_shift & 1) != 0;
        m_shift = uint8_t((m_shift >> 1) | ((d & 1) << 4));
        if(!complete) return true;
        const uint8_t v = m_shift & 0x1F; m_shift = 0x10;
        switch((a >> 13) & 3) {
        case 0: m_ctrl=v; updateMirror(); break;
        case 1:
            m_chr0=v;
            if(v & 0x10) { m_timer=0; m_irq=false; m_timerRunning=false; }
            else { m_timerRunning=true; m_started=true; }
            break;
        case 2: m_chr1=v; break;
        case 3: m_prg=v; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if(a >= 0x2000) return false;
        const std::size_t z = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if(!z) return false;
        // NES-EVENT normally has 8 KiB CHR RAM; retain MMC1 pattern banking if
        // a diagnostic image supplies a larger CHR device.
        if(z <= 0x2000) m = uint32_t(a % z);
        else if(!(m_ctrl & 0x10)) m = mapBank((m_chr0 & 0x1E) >> 1,0x2000,z,a);
        else if(a < 0x1000) m=mapBank(m_chr0,0x1000,z,a);
        else m=mapBank(m_chr1,0x1000,z,a-0x1000);
        return true;
    }
    bool ppuMapWrite(uint16_t a,uint32_t& m) override { return m_config.chrRamSize && ppuMapRead(a,m); }
    bool mapPrgRam(uint16_t a,uint32_t& m,bool) const override {
        if(a<0x6000||a>0x7FFF||!m_config.prgRamSize||(m_prg&0x10)) return false;
        m=uint32_t((a-0x6000)%m_config.prgRamSize); return true;
    }
    void clockCpu() override {
        if(!m_timerRunning || m_irq) return;
        if(m_timer < 0x3FFFFFFFu) ++m_timer;
        if(m_timer >= timerThreshold()) { m_irq=true; m_timerRunning=false; }
    }
    bool irqActive() const override { return m_irq; }
    void reset(bool hard) override {
        if(!hard) return;
        m_shift=0x10;m_ctrl=0x0C;m_chr0=m_chr1=m_prg=0;m_lastWriteCycle=kNoCpuCycle;
        m_timer=0;m_timerRunning=false;m_irq=false;m_started=false;updateMirror();
    }
    void saveState(std::vector<uint8_t>&o) const override {
        put8(o,m_shift);put8(o,m_ctrl);put8(o,m_chr0);put8(o,m_chr1);put8(o,m_prg);put64(o,m_lastWriteCycle);
        put64(o,m_timer);put8(o,m_timerRunning);put8(o,m_irq);put8(o,m_started);put8(o,m_dip);
    }
    bool loadState(const uint8_t*&p,const uint8_t*e) override {
        uint64_t t=0;uint8_t b=0;
        if(!get8(p,e,m_shift)||!get8(p,e,m_ctrl)||!get8(p,e,m_chr0)||!get8(p,e,m_chr1)||!get8(p,e,m_prg)||!get64(p,e,m_lastWriteCycle)||!get64(p,e,t))return false;
        m_timer=uint32_t(t&0x3FFFFFFF);
        if(!get8(p,e,b))return false;
        m_timerRunning=b;
        if(!get8(p,e,b))return false;
        m_irq=b;
        if(!get8(p,e,b))return false;
        m_started=b;
        if(!get8(p,e,m_dip))return false;
        m_dip&=0x0F;
        updateMirror();
        return true;
    }
private:
    uint8_t m_shift=0x10,m_ctrl=0x0C,m_chr0=0,m_chr1=0,m_prg=0;
    uint64_t m_lastWriteCycle=kNoCpuCycle;
    uint32_t m_timer=0; bool m_timerRunning=false,m_irq=false,m_started=false;
    // Official NWC setup: switch C closed, A/B/D open -> $28000000.
    uint8_t m_dip=0x04;
    uint32_t timerThreshold() const { return 0x20000000u | (uint32_t(m_dip & 0x0F) << 25); }
    void updateMirror(){switch(m_ctrl&3){case 0:m_mirror=Mirror::OnescreenLo;break;case 1:m_mirror=Mirror::OnescreenHi;break;case 2:m_mirror=Mirror::Vertical;break;default:m_mirror=Mirror::Horizontal;break;}}
};

// iNES Mapper 156 / DAOU DIS23C01.  The Korean DAOU 245 boards expose a
// switchable 16 KiB PRG window, eight independent 9-bit 1 KiB CHR banks,
// 8 KiB WRAM, and vertical/horizontal/one-screen-A mirroring.
class Mapper156 final : public Mapper {
public:
    explicit Mapper156(const MapperConfig& c) : Mapper(c) { m_mirror = Mirror::OnescreenLo; }

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || !m_config.prgRomSize) return false;
        if (addr < 0xC000) {
            mapped = mapBank(m_prg & 0x0F, 0x4000, m_config.prgRomSize, addr - 0x8000);
        } else {
            const std::size_t banks = std::max<std::size_t>(1, m_config.prgRomSize / 0x4000);
            mapped = mapBank(banks - 1, 0x4000, m_config.prgRomSize, addr - 0xC000);
        }
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr >= 0xC000 && addr <= 0xC00F) {
            const uint8_t off = uint8_t(addr & 0x0F);
            const uint8_t slot = uint8_t((off & 3) | ((off & 8) ? 4 : 0));
            if (off & 4)
                m_chr[slot] = uint16_t((m_chr[slot] & 0x00FFu) | (uint16_t(data & 1) << 8));
            else
                m_chr[slot] = uint16_t((m_chr[slot] & 0x0100u) | data);
            return true;
        }
        if (addr == 0xC010) {
            m_prg = uint8_t(data & 0x0F);
            return true;
        }
        if (addr == 0xC014) {
            switch (data & 3) {
            case 0: m_mirror = Mirror::Vertical; break;
            case 1: m_mirror = Mirror::Horizontal; break;
            default: m_mirror = Mirror::OnescreenLo; break;
            }
            return true;
        }
        return false;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        const uint8_t slot = uint8_t(addr >> 10);
        mapped = mapBank(m_chr[slot] & 0x01FFu, 0x0400, size, addr & 0x03FF);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    void reset(bool hard) override {
        if (!hard) return;
        m_prg = 0;
        std::fill(std::begin(m_chr), std::end(m_chr), uint16_t(0));
        m_mirror = Mirror::OnescreenLo;
    }

    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_prg);
        for (const uint16_t v : m_chr) {
            put8(out, uint8_t(v));
            put8(out, uint8_t(v >> 8));
        }
        put8(out, static_cast<uint8_t>(m_mirror));
    }

    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        uint8_t lo = 0, hi = 0, mirror = 0;
        if (!get8(p, end, m_prg)) return false;
        for (auto& v : m_chr) {
            if (!get8(p, end, lo) || !get8(p, end, hi)) return false;
            v = uint16_t(lo | ((hi & 1u) << 8));
        }
        if (!get8(p, end, mirror)) return false;
        if (mirror > static_cast<uint8_t>(Mirror::FourScreen)) return false;
        m_mirror = static_cast<Mirror>(mirror);
        return true;
    }

private:
    uint8_t m_prg = 0;
    uint16_t m_chr[8] = {};
};

// Mapper 144 - AGCI 50282 (Death Race).
// Banking matches Color Dreams (D1-D0 PRG, D7-D4 CHR), but the board's
// bus-conflict network pulls CPU D0 high through a resistor before the
// wired-AND with PRG-ROM. This is electrically distinct from Mapper 11.
class Mapper144 final : public Mapper {
public:
    explicit Mapper144(const MapperConfig& config) : Mapper(config) {}

    bool cpuMapRead(uint16_t addr, uint32_t& mapped) const override {
        if (addr < 0x8000 || m_config.prgRomSize == 0) return false;
        mapped = mapBank(m_prg & 0x03, 0x8000, m_config.prgRomSize, addr - 0x8000);
        return true;
    }

    bool cpuWrite(uint16_t addr, uint8_t data, uint64_t) override {
        if (addr < 0x8000) return false;
        m_prg = data & 0x03;
        m_chr = (data >> 4) & 0x0F;
        return true;
    }

    bool ppuMapRead(uint16_t addr, uint32_t& mapped) override {
        if (addr >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        mapped = mapBank(m_chr, 0x2000, size, addr);
        return true;
    }

    bool ppuMapWrite(uint16_t addr, uint32_t& mapped) override {
        return m_config.chrRamSize && ppuMapRead(addr, mapped);
    }

    bool hasBusConflicts() const override { return true; }
    uint8_t resolveBusConflict(uint16_t, uint8_t cpuData, uint8_t romData) const override {
        return uint8_t((cpuData | 0x01u) & romData);
    }

    void reset(bool hard) override {
        if (hard) { m_prg = 0; m_chr = 0; }
    }
    void saveState(std::vector<uint8_t>& out) const override {
        put8(out, m_prg); put8(out, m_chr);
    }
    bool loadState(const uint8_t*& p, const uint8_t* end) override {
        return get8(p, end, m_prg) && get8(p, end, m_chr);
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr = 0;
};


// Mapper 193 / NTDEC TC-112 (Fighting Hero). Four registers mirrored
// throughout $6000-$7FFF select one 8 KiB PRG bank and a 4K+2K+2K CHR
// layout. The upper three PRG banks are fixed to the final 24 KiB and the
// board is hardwired for vertical mirroring.
class Mapper193 final : public Mapper {
public:
    explicit Mapper193(const MapperConfig& c) : Mapper(c) { m_mirror = Mirror::Vertical; }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if (a < 0x8000 || !m_config.prgRomSize) return false;
        const std::size_t banks = std::max<std::size_t>(1, m_config.prgRomSize / 0x2000);
        std::size_t bank = 0;
        if (a < 0xA000) bank = m_prg;
        else if (a < 0xC000) bank = banks > 3 ? banks - 3 : 0;
        else if (a < 0xE000) bank = banks > 2 ? banks - 2 : 0;
        else bank = banks - 1;
        m = mapBank(bank, 0x2000, m_config.prgRomSize, a & 0x1FFF);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if (a < 0x6000 || a > 0x7FFF) return false;
        switch (a & 3) {
        case 0: m_chr4 = uint8_t(d >> 2); break;
        case 1: m_chr2[0] = uint8_t(d >> 1); break;
        case 2: m_chr2[1] = uint8_t(d >> 1); break;
        case 3: m_prg = d; break;
        }
        return true;
    }

    bool ppuMapRead(uint16_t a, uint32_t& m) override {
        if (a >= 0x2000) return false;
        const std::size_t size = m_config.chrRomSize ? m_config.chrRomSize : m_config.chrRamSize;
        if (!size) return false;
        if (a < 0x1000)
            m = mapBank(m_chr4, 0x1000, size, a);
        else if (a < 0x1800)
            m = mapBank(m_chr2[0], 0x0800, size, a & 0x07FF);
        else
            m = mapBank(m_chr2[1], 0x0800, size, a & 0x07FF);
        return true;
    }

    bool ppuMapWrite(uint16_t a, uint32_t& m) override {
        return m_config.chrRamSize && !m_config.chrRomSize && ppuMapRead(a, m);
    }

    bool mapPrgRam(uint16_t, uint32_t&, bool) const override { return false; }

    void reset(bool hard) override {
        if (!hard) return;
        m_prg = 0;
        m_chr4 = 0;
        m_chr2[0] = m_chr2[1] = 0;
        m_mirror = Mirror::Vertical;
    }

    void saveState(std::vector<uint8_t>& o) const override {
        put8(o, m_prg); put8(o, m_chr4); put8(o, m_chr2[0]); put8(o, m_chr2[1]);
    }
    bool loadState(const uint8_t*& p, const uint8_t* e) override {
        return get8(p,e,m_prg) && get8(p,e,m_chr4) && get8(p,e,m_chr2[0]) && get8(p,e,m_chr2[1]);
    }

private:
    uint8_t m_prg = 0;
    uint8_t m_chr4 = 0;
    uint8_t m_chr2[2] = {0, 0};
};
