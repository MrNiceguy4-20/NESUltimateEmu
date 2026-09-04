
namespace mapper_more_detail {
inline void setMirror4(Mirror& m, uint8_t v) {
    switch(v & 3) { case 0: m=Mirror::Vertical; break; case 1: m=Mirror::Horizontal; break;
        case 2: m=Mirror::OnescreenLo; break; default: m=Mirror::OnescreenHi; break; }
}

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
        case 0:
            bank8 = uint8_t(((m_prg & 0x3E) << 1) | ((a >> 13) & 3));
            break;
        case 1:
            if (a < 0xC000) bank8 = uint8_t((m_prg << 1) | cpuA13);
            else bank8 = uint8_t((((m_prg & 0x38) | 0x07) << 1) | cpuA13);
            break;
        case 2:
            bank8 = uint8_t((m_prg << 1) | m_prgA13);
            break;
        default:
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
        const size_t outer = size_t(m_outer[m_ppuOuterSlot] & 1) << 4;
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
        const uint8_t r = uint8_t(a & 0x0F);
        if (r < 4) {
            m_outer[r] = d & 1;
            return true;
        }
        if (r >= 4 && r < 8) return true;
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
        m = a % m_config.chrRamSize;
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

class Mapper116 final : public Mapper {
public:
    explicit Mapper116(const MapperConfig& c) : Mapper(c) {
        MapperConfig mmc1c = c;

        m_mmc1 = createMmc1Mapper(mmc1c);

        MapperConfig mmc3c = c;
        mmc3c.id = 4;
        mmc3c.submapper = 0;
        m_mmc3 = createMmc3Mapper(mmc3c);

        MapperConfig vrc2c = c;
        vrc2c.id = 23;
        vrc2c.submapper = 1;
        vrc2c.prgRamSize = 0;
        vrc2c.headerPrgNvRamSize = 0;
        m_vrc2 = createVrcMapper(vrc2c);

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

class Mapper70 final : public Mapper {public:explicit Mapper70(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=(d>>4)&7;m_chr=d&0x0F;if(m_config.id==152)m_mirror=(d&0x80)?Mirror::OnescreenHi:Mirror::OnescreenLo;return true;}bool hasBusConflicts()const override{return m_config.id==152;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);put8(o,static_cast<uint8_t>(m_mirror));}bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t b;if(!get8(p,e,m_prg)||!get8(p,e,m_chr)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);return true;}private:uint8_t m_prg=0,m_chr=0;};

class Mapper72 final : public Mapper {public:explicit Mapper72(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||m_config.prgRomSize<0x4000)return false;if(a<0xC000)m=mapBank(m_prg,0x4000,m_config.prgRomSize,a-0x8000);else m=static_cast<uint32_t>(m_config.prgRomSize-0x4000+(a-0xC000));return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(m_config.id==92){if((d&0xF0)==0xD0)m_prg=d&0x0F;else if((d&0xF0)==0xE0)m_chr=d&0x0F;}else{if(d&0x80)m_prg=d&0x0F;if(d&0x40)m_chr=d&0x0F;}return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr,0x2000,s,a);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

class Mapper76 final : public Mapper {public:explicit Mapper76(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_prg[0]:a<0xC000?m_prg[1]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if((a&0xE001)==0x8000){m_sel=d&7;return true;}if((a&0xE001)==0x8001){if(m_sel>=2&&m_sel<=5)m_chr[m_sel-2]=d;else if(m_sel>=6)m_prg[m_sel-6]=d;return true;}return false;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;m=mapBank(m_chr[a>>11],0x800,s,a&0x7FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_prg)put8(o,v);for(auto v:m_chr)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_prg)if(!get8(p,e,v))return false;for(auto&v:m_chr)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_prg[2]={},m_chr[4]={};};

class Mapper77 final : public Mapper {public:explicit Mapper77(const MapperConfig&c):Mapper(c){m_mirror=Mirror::FourScreen;}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d&7;m_chr=d>>4;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;if(a<0x0800){if(!m_config.chrRomSize)return false;m=mapBank(m_chr,0x800,m_config.chrRomSize,a);return true;}if(!m_config.chrRamSize)return false;m=a-0x0800;return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{if(a<0x0800||a>=0x2000||!m_config.chrRamSize)return false;m=a-0x0800;return true;}bool ppuUsesChrRam(uint16_t a)const override{return a>=0x0800&&a<0x2000&&m_config.chrRamSize;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);}bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);}private:uint8_t m_prg=0,m_chr=0;};

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

class Mapper38 final : public Mapper { public: explicit Mapper38(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x7000||a>0x7FFF)return false;m_prg=d&3;m_chr=(d>>2)&3;return true;} bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto z=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!z)return false;m=mapBank(m_chr,0x2000,z,a);return true;} bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);} bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;} void reset(bool)override{m_prg=m_chr=0;} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);put8(o,m_chr);} bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg)&&get8(p,e,m_chr);} private:uint8_t m_prg=0,m_chr=0;};

class Mapper39 final : public Mapper { public: explicit Mapper39(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;m=mapBank(m_prg,0x8000,m_config.prgRomSize,a-0x8000);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;m_prg=d;return true;} void reset(bool)override{m_prg=0;} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_prg);} bool loadState(const uint8_t*&p,const uint8_t*e)override{return get8(p,e,m_prg);} private:uint8_t m_prg=0;};

class Mapper50 final : public Mapper { public: explicit Mapper50(const MapperConfig&c):Mapper(c){} bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x6000||!m_config.prgRomSize)return false;size_t b=a<0x8000?0x0F:a<0xA000?8:a<0xC000?9:a<0xE000?m_bank:0x0B;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;} bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{switch(a&0x4120){case 0x4020:m_bank=(d&8)|((d&1)<<2)|((d&6)>>1);return true;case 0x4120:if(d&1)m_irqEnable=true;else{m_irqEnable=false;m_irq=false;m_irqCounter=0;}return true;default:return false;}} bool mapPrgRam(uint16_t,uint32_t&,bool)const override{return false;} void clockCpu()override{if(m_irqEnable&&!m_irq&&++m_irqCounter==0x1000){m_irq=true;m_irqEnable=false;}} bool irqActive()const override{return m_irq;} void reset(bool)override{m_bank=0;m_irqEnable=m_irq=false;m_irqCounter=0;} void saveState(std::vector<uint8_t>&o)const override{put8(o,m_bank);put8(o,m_irqEnable);put8(o,m_irq);put8(o,uint8_t(m_irqCounter));put8(o,uint8_t(m_irqCounter>>8));} bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,b;if(!get8(p,e,m_bank)||!get8(p,e,b))return false;m_irqEnable=b;if(!get8(p,e,b))return false;m_irq=b;if(!get8(p,e,lo)||!get8(p,e,hi))return false;m_irqCounter=uint16_t(lo)|(uint16_t(hi)<<8);return true;} private:uint8_t m_bank=0;bool m_irqEnable=false,m_irq=false;uint16_t m_irqCounter=0;};

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

class Mapper61 final : public Mapper {
public: explicit Mapper61(const MapperConfig&c):Mapper(c){}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;if((m_addr&0x30)==0||(m_addr&0x30)==0x30){m=mapBank(m_addr&0x0F,0x8000,m_config.prgRomSize,a-0x8000);}else{const size_t b=((size_t(m_addr)<<1)&0x1E)|((m_addr>>4)&2);m=mapBank(b,0x4000,m_config.prgRomSize,a&0x3FFF);}return true;}
    bool cpuWrite(uint16_t a,uint8_t,uint64_t)override{if(a<0x8000)return false;m_addr=a;m_mirror=(a&0x80)?Mirror::Horizontal:Mirror::Vertical;return true;}
    void reset(bool hard)override{if(hard){m_addr=0x8000;m_mirror=Mirror::Vertical;}}
    void saveState(std::vector<uint8_t>&o)const override{put8(o,uint8_t(m_addr));put8(o,uint8_t(m_addr>>8));put8(o,uint8_t(m_mirror));}
    bool loadState(const uint8_t*&p,const uint8_t*e)override{uint8_t lo,hi,mi;if(!get8(p,e,lo)||!get8(p,e,hi)||!get8(p,e,mi))return false;m_addr=uint16_t(lo)|(uint16_t(hi)<<8);m_mirror=Mirror(mi);return true;}
private:uint16_t m_addr=0x8000;
};

class Mapper63 final : public Mapper {
public:
    explicit Mapper63(const MapperConfig& c):Mapper(c){}

    bool cpuMapRead(uint16_t a,uint32_t& m) const override {
        if(a<0x8000||!m_config.prgRomSize)return false;

        const unsigned outerBits=(m_config.nes20&&m_config.submapper==1)?6u:7u;
        const size_t outer=(size_t(m_addr>>3)&((size_t(1)<<outerBits)-1));
        const bool nrom256=(m_addr&0x0002)!=0;
        const size_t lowBit=nrom256?size_t((a>>14)&1):size_t((m_addr>>2)&1);
        const size_t bank16=(outer<<1)|lowBit;
        const size_t mapped=bank16*0x4000u+size_t(a&0x3FFF);
        if(mapped>=m_config.prgRomSize)return false;
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

        return (m_addr&((m_config.nes20&&m_config.submapper==1)?0x0200:0x0400))!=0;
    }
    uint16_t m_addr=0x8000;
};

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

            m = mapBank(m_prg >> 1, 0x8000, m_config.prgRomSize, a & 0x7FFF);
        } else {

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
        case 202: {
            const uint8_t bank = uint8_t((o >> 1) & 0x07);
            const uint8_t mode = ((bank & 0x06) == 0x06) ? 1 : 0;
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
            m_chr = bank;
            m_mirror = (o & 1) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        }
        case 203: {
            const uint8_t bank = uint8_t((d >> 2) & 0x03);
            set16(bank, bank);
            m_chr = uint8_t(d & 0x03);
            break;
        }
        case 204: {
            const uint16_t join = uint16_t(((o >> 1) & (o >> 2)) & 1);
            set16(uint8_t(o & ~join), uint8_t(o | join));
            m_chr = uint8_t(o & ~join);
            m_mirror = (d & 0x10) ? Mirror::Horizontal : Mirror::Vertical;
            break;
        }
        case 214: {
            const uint16_t region = uint16_t(o & 0x7000);
            if (region != 0x0000 && region != 0x7000) return true;
            const uint8_t bank = uint8_t(o >> 2);
            set16(bank, bank);
            m_chr = bank;
            break;
        }
        case 217:
            m_mode32 = true;
            m_prg32 = uint8_t((o >> 2) & 0x07);
            m_chr = uint8_t(o & 0x07);
            break;
        case 229: {
            m_mirror = (o & 0x20) ? Mirror::Horizontal : Mirror::Vertical;
            const uint8_t bank = uint8_t(o & 0x1F);
            m_chr = bank;
            if (bank & 0x1E) set16(bank, bank);
            else { m_mode32 = true; m_prg32 = 0; }
            break;
        }
        case 231: {
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

        if (m_config.id == 235 && base >= m_config.prgRomSize) return false;
        uint32_t inBank = uint32_t(a & 0x3FFF);
        if (m_config.id == 237 && (m_latch[0] & 1)) inBank |= 0x0002;
        m = uint32_t((base + inBank) % m_config.prgRomSize);
        return true;
    }

    bool cpuReadRegister(uint16_t a, uint8_t& d) override {
        if ((m_config.id != 225 && m_config.id != 255) || a < 0x5800 || a > 0x5FFF) return false;

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
            if (o & 0x0080) {
                set16(bank, uint8_t(bank + (o & 1)));
            } else {
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
            if (m_latch[0] & 0x0002) {
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
        if (hi & 0x20) {
            const uint8_t mode = uint8_t((hi & 0x10) ? 0 : 1);
            set16(uint8_t(bank & ~mode), uint8_t(bank | mode));
        } else {
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

class Mapper28 final : public Mapper {
public:
    explicit Mapper28(const MapperConfig& c) : Mapper(c) {

        m_mode = 0x0C;
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

            if (m_nrom256) {
                bank = 8u | (size_t((m_page >> 1) & 1) << 2)
                       | (size_t((a >> 14) & 1) << 1)
                       | size_t((a >> 13) & 1);
            } else {
                bank = 8u | (size_t(m_page & 3) << 1)
                       | size_t((a >> 13) & 1);
            }
        } else {

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

class Mapper95 final : public Mapper {public:explicit Mapper95(const MapperConfig&c):Mapper(c){}bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0x8000||!m_config.prgRomSize)return false;size_t n=std::max<size_t>(1,m_config.prgRomSize/0x2000);size_t b=a<0xA000?m_r[6]:a<0xC000?m_r[7]:a<0xE000?(n>1?n-2:0):n-1;m=mapBank(b,0x2000,m_config.prgRomSize,a&0x1FFF);return true;}bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{if(a<0x8000)return false;if(!(a&1))m_sel=d&7;else m_r[m_sel]=d;return true;}bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000)return false;auto s=m_config.chrRomSize?m_config.chrRomSize:m_config.chrRamSize;if(!s)return false;if(a<0x800)m=mapBank((m_r[0]&0xFE)>>1,0x800,s,a&0x7FF);else if(a<0x1000)m=mapBank((m_r[1]&0xFE)>>1,0x800,s,a&0x7FF);else m=mapBank(m_r[2+((a-0x1000)>>10)],0x400,s,a&0x3FF);return true;}bool ppuMapWrite(uint16_t a,uint32_t&m)override{return m_config.chrRamSize&&ppuMapRead(a,m);}bool mapNametable(uint16_t a,NametableSource&s,uint32_t&m)const override{if(a<0x2000||a>0x3EFF)return false;uint8_t p=(a&0xFFF)>>10,nt=((p<2?m_r[0]:m_r[1])>>5)&1;s=NametableSource::Ciram;m=uint32_t(nt)*0x400+(a&0x3FF);return true;}void saveState(std::vector<uint8_t>&o)const override{put8(o,m_sel);for(auto v:m_r)put8(o,v);}bool loadState(const uint8_t*&p,const uint8_t*e)override{if(!get8(p,e,m_sel))return false;for(auto&v:m_r)if(!get8(p,e,v))return false;return true;}private:uint8_t m_sel=0,m_r[8]={};};

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

            nt = uint8_t((page >> 1) & 1);
        } else {
            switch ((m_reg[7] >> 1) & 3) {
            case 0: nt = uint8_t((page >> 1) & 1); break;
            case 1: nt = uint8_t(page & 1); break;
            case 2: nt = uint8_t(page == 0 ? 0 : 1); break;
            default: nt = 0; break;
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
        default: m_mirror = Mirror::Horizontal; break;
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

            nt = uint8_t(page & 1);
        } else {
            switch ((m_reg[7] >> 1) & 3) {
            case 0: nt = uint8_t(page & 1); break;
            case 1: nt = uint8_t((page >> 1) & 1); break;
            case 2: nt = uint8_t(page == 0 ? 0 : 1); break;
            default: nt = 0; break;
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
        case 135: case 141: return 1;
        case 139: return 2;
        default: return 0;
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
        default: m_mirror = Mirror::Vertical; break;
        }
    }

    uint8_t m_select = 0;
    uint8_t m_reg[8] = {};
};

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
        if (m_invert) value ^= 0x30;
        d = uint8_t((d & 0xC0) | value);
        return true;
    }

    bool cpuWrite(uint16_t a, uint8_t d, uint64_t) override {
        if ((a & 0xE100) == 0x4100) {
            switch (a & 3) {
            case 0:
                if (!m_mode) {
                    m_reg = uint8_t(m_input & 0x3F);
                    if (m_invert) m_reg ^= 0x0F;
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

class Mapper105 final : public Mapper {
public:
    explicit Mapper105(const MapperConfig& c) : Mapper(c) { updateMirror(); }

    bool cpuMapRead(uint16_t a, uint32_t& m) const override {
        if(a < 0x8000 || !m_config.prgRomSize) return false;

        if(!m_started) { m = uint32_t(a - 0x8000) % uint32_t(m_config.prgRomSize); return true; }

        const uint8_t eventBank = (m_chr0 >> 1) & 7;
        if(eventBank < 4) {
            m = mapBank(eventBank, 0x8000, std::min<std::size_t>(m_config.prgRomSize, 0x20000), a - 0x8000);
            return true;
        }

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

    uint8_t m_dip=0x04;
    uint32_t timerThreshold() const { return 0x20000000u | (uint32_t(m_dip & 0x0F) << 25); }
    void updateMirror(){switch(m_ctrl&3){case 0:m_mirror=Mirror::OnescreenLo;break;case 1:m_mirror=Mirror::OnescreenHi;break;case 2:m_mirror=Mirror::Vertical;break;default:m_mirror=Mirror::Horizontal;break;}}
};

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
