// Famicom Disk System (Mapper 20) disk/timer/audio subsystem.  This mapper is
// instantiated only for .fds images; Cartridge supplies the 8 KiB BIOS as PRG
// ROM and allocates the FDS 32 KiB work RAM + 8 KiB CHR RAM.
namespace fds_detail {
inline void put16(std::vector<uint8_t>& o, uint16_t v){put8(o,uint8_t(v));put8(o,uint8_t(v>>8));}
inline void put32(std::vector<uint8_t>& o, uint32_t v){for(int i=0;i<4;i++)put8(o,uint8_t(v>>(i*8)));}
inline bool get16(const uint8_t*&p,const uint8_t*e,uint16_t&v){uint8_t l,h;if(!get8(p,e,l)||!get8(p,e,h))return false;v=uint16_t(l)|(uint16_t(h)<<8);return true;}
inline bool get32(const uint8_t*&p,const uint8_t*e,uint32_t&v){v=0;for(int i=0;i<4;i++){uint8_t b;if(!get8(p,e,b))return false;v|=uint32_t(b)<<(8*i);}return true;}

struct Envelope {
    uint8_t speed=0,gain=0,master=0xFF; bool off=false,increase=false; uint16_t freq=0; uint32_t timer=0;
    void resetTimer(){timer=8u*uint32_t(speed+1)*uint32_t(master);}
    void write(uint16_t a,uint8_t d){switch(a&3){case 0:speed=d&0x3F;increase=(d&0x40)!=0;off=(d&0x80)!=0;resetTimer();if(off)gain=speed;break;case 2:freq=uint16_t((freq&0xF00)|d);break;case 3:freq=uint16_t((freq&0x0FF)|((uint16_t(d&0x0F))<<8));break;}}
    bool tick(){if(off||master==0)return false;if(timer==0)resetTimer();if(timer&&--timer==0){resetTimer();if(increase&&gain<32)++gain;else if(!increase&&gain>0)--gain;return true;}return false;}
    void save(std::vector<uint8_t>&o)const{put8(o,speed);put8(o,gain);put8(o,master);put8(o,off);put8(o,increase);put16(o,freq);put32(o,timer);}
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t b;if(!get8(p,e,speed)||!get8(p,e,gain)||!get8(p,e,master)||!get8(p,e,b))return false;off=b;if(!get8(p,e,b))return false;increase=b;return get16(p,e,freq)&&get32(p,e,timer);}
};

struct Modulator : Envelope {
    int8_t counter=0; bool disabled=false; uint8_t table[64]={},pos=0; uint16_t overflow=0; int32_t output=0;
    void setCounter(int v){v&=0x7F;counter=v>=64?int8_t(v-128):int8_t(v);}
    void writeReg(uint16_t a,uint8_t d){if(a==0x4084||a==0x4086)Envelope::write(a,d);else if(a==0x4085)setCounter(d);else if(a==0x4087){Envelope::write(a,d);disabled=(d&0x80)!=0;if(disabled)overflow=0;}}
    void writeTable(uint8_t d){if(!disabled)return;table[pos&0x3F]=d&7;table[(pos+1)&0x3F]=d&7;pos=(pos+2)&0x3F;}
    bool tickMod(){static const int lut[8]={0,1,2,4,0x7FFF,-4,-2,-1};if(disabled||freq==0)return false;uint16_t old=overflow;overflow=uint16_t(overflow+freq);if(overflow<old){int v=lut[table[pos&0x3F]];setCounter(v==0x7FFF?0:int(counter)+v);pos=(pos+1)&0x3F;return true;}return false;}
    void update(uint16_t pitch){int32_t t=int32_t(counter)*gain;int32_t rem=t&0xF;t>>=4;if(rem>0&&(t&0x80)==0)t+=counter<0?-1:2;if(t>=192)t-=256;else if(t<-64)t+=256;t*=pitch;rem=t&0x3F;t>>=6;if(rem>=32)++t;output=t;}
    int32_t out()const{return (!disabled&&freq>0)?output:0;}
    void save(std::vector<uint8_t>&o)const{Envelope::save(o);put8(o,uint8_t(counter));put8(o,disabled);for(auto v:table)put8(o,v);put8(o,pos);put16(o,overflow);put32(o,uint32_t(output));}
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t b;uint32_t q;if(!Envelope::load(p,e)||!get8(p,e,b))return false;counter=int8_t(b);if(!get8(p,e,b))return false;disabled=b;for(auto&v:table)if(!get8(p,e,v))return false;if(!get8(p,e,pos)||!get16(p,e,overflow)||!get32(p,e,q))return false;output=int32_t(q);return true;}
};

struct Audio {
    uint8_t wave[64]={}; bool waveWrite=false,disableEnvelopes=false,halt=false; uint8_t masterVol=0,wavePos=0,last=0; uint16_t waveOverflow=0; Envelope volume; Modulator mod;
    void clock(){int f=volume.freq;if(!halt&&!disableEnvelopes){volume.tick();if(mod.tick())mod.update(uint16_t(f));}if(mod.tickMod())mod.update(uint16_t(f));if(halt)wavePos=0;else if(!waveWrite&&f+mod.out()>0){uint16_t add=uint16_t(f+mod.out());uint16_t old=waveOverflow;waveOverflow=uint16_t(waveOverflow+add);if(waveOverflow<old)wavePos=(wavePos+1)&0x3F;}static const uint8_t scale[4]={36,24,17,14};uint32_t level=std::min<int>(volume.gain,32)*scale[masterVol&3];last=uint8_t((wave[wavePos]*level)/1152);}
    uint8_t read(uint16_t a)const{if(a<=0x407F)return wave[a&0x3F]&0x3F;if(a==0x4090)return volume.gain&0x3F;if(a==0x4092)return mod.gain&0x3F;return 0;}
    void write(uint16_t a,uint8_t d){if(a<=0x407F){if(waveWrite)wave[a&0x3F]=d&0x3F;return;}switch(a){case 0x4080:case 0x4082:volume.write(a,d);break;case 0x4083:disableEnvelopes=(d&0x40)!=0;halt=(d&0x80)!=0;if(disableEnvelopes){volume.resetTimer();mod.resetTimer();}volume.write(a,d);break;case 0x4084:case 0x4085:case 0x4086:case 0x4087:mod.writeReg(a,d);break;case 0x4088:mod.writeTable(d);break;case 0x4089:masterVol=d&3;waveWrite=(d&0x80)!=0;break;case 0x408A:volume.master=d;mod.master=d;break;}}
    float sample()const{return (float(last)/63.0f)*0.65f;}
    void save(std::vector<uint8_t>&o)const{for(auto v:wave)put8(o,v);put8(o,waveWrite);put8(o,disableEnvelopes);put8(o,halt);put8(o,masterVol);put8(o,wavePos);put8(o,last);put16(o,waveOverflow);volume.save(o);mod.save(o);}
    bool load(const uint8_t*&p,const uint8_t*e){uint8_t b;for(auto&v:wave)if(!get8(p,e,v))return false;if(!get8(p,e,b))return false;waveWrite=b;if(!get8(p,e,b))return false;disableEnvelopes=b;if(!get8(p,e,b))return false;halt=b;return get8(p,e,masterVol)&&get8(p,e,wavePos)&&get8(p,e,last)&&get16(p,e,waveOverflow)&&volume.load(p,e)&&mod.load(p,e);}
};
}

class Mapper20 final : public Mapper {
public:
    explicit Mapper20(const MapperConfig& c):Mapper(c){m_mirror=Mirror::Vertical;}
    bool cpuMapRead(uint16_t a,uint32_t&m)const override{if(a<0xE000||!m_config.prgRomSize)return false;m=(a-0xE000)%m_config.prgRomSize;return true;}
    bool mapPrgRam(uint16_t a,uint32_t&m,bool)const override{if(a<0x6000||a>0xDFFF||!m_config.prgRamSize)return false;m=(a-0x6000)%m_config.prgRamSize;return true;}
    bool ppuMapRead(uint16_t a,uint32_t&m)override{if(a>=0x2000||!m_config.chrRamSize)return false;m=a%m_config.chrRamSize;return true;}
    bool ppuMapWrite(uint16_t a,uint32_t&m)override{return ppuMapRead(a,m);}

    bool cpuReadRegister(uint16_t a,uint8_t&d)override{
        if(m_soundRegs&&a>=0x4040&&a<=0x4092){d=m_audio.read(a);return true;}
        if(!m_diskRegs||a<0x4030||a>0x4033)return false;
        switch(a){
        case 0x4030:d=uint8_t((m_timerPending?1:0)|(m_transferComplete?2:0)|(m_badCrc?0x10:0));if(true){m_transferComplete=false;m_timerPending=false;m_diskPending=false;}return true;
        case 0x4031:d=m_readData;if(true){m_transferComplete=false;m_diskPending=false;}return true;
        case 0x4032:d=uint8_t((!diskInserted()?1:0)|((!diskInserted()||!m_scanning)?2:0)|(!diskInserted()?4:0));return true;
        case 0x4033:d=m_ext;return true;
        }
        return false;
    }
    bool cpuWrite(uint16_t a,uint8_t d,uint64_t)override{
        if(m_soundRegs&&a>=0x4040&&a<=0x408A){m_audio.write(a,d);return true;}
        switch(a){
        case 0x4020:m_irqReload=uint16_t((m_irqReload&0xFF00)|d);return true;
        case 0x4021:m_irqReload=uint16_t((m_irqReload&0x00FF)|(uint16_t(d)<<8));return true;
        case 0x4022:m_irqRepeat=(d&1)!=0;m_irqEnabled=(d&2)!=0&&m_diskRegs;if(m_irqEnabled)m_irqCounter=m_irqReload;else m_timerPending=false;return true;
        case 0x4023:m_diskRegs=(d&1)!=0;m_soundRegs=(d&2)!=0;if(!m_diskRegs){m_irqEnabled=false;m_timerPending=false;m_diskPending=false;}return true;
        case 0x4024:if(!m_diskRegs)return true;m_writeData=d;m_transferComplete=false;m_diskPending=false;return true;
        case 0x4025:if(!m_diskRegs)return true;m_motor=(d&1)!=0;m_resetTransfer=(d&2)!=0;m_readMode=(d&4)!=0;m_mirror=(d&8)?Mirror::Horizontal:Mirror::Vertical;m_crcControl=(d&0x10)!=0;m_diskReady=(d&0x40)!=0;m_diskIrqEnabled=(d&0x80)!=0;m_diskPending=false;return true;
        case 0x4026:if(m_diskRegs)m_ext=d;return true;
        default:return a>=0x4020&&a<=0x4092;
        }
    }

    void clockCpu()override{
        m_audio.clock();
        if(m_irqEnabled){if(m_irqCounter==0){m_timerPending=true;m_irqCounter=m_irqReload;if(!m_irqRepeat)m_irqEnabled=false;}else --m_irqCounter;}
        if(!diskInserted()||!m_motor){m_endOfHead=true;m_scanning=false;return;}
        if(m_resetTransfer&&!m_scanning)return;
        if(m_endOfHead){m_delay=50000;m_endOfHead=false;m_diskPos=0;m_gapEnded=false;return;}
        if(m_delay){--m_delay;return;}
        m_scanning=true;
        auto&side=m_sides[size_t(m_currentSide)];
        if(m_diskPos>=side.size()){m_motor=false;m_endOfHead=true;return;}
        uint8_t v=0;bool wantIrq=m_diskIrqEnabled;
        if(m_readMode){
            v=side[m_diskPos];
            if(!m_prevCrc) updateCrc(v);
            if(!m_diskReady){m_gapEnded=false;m_crc=0;}
            else if(v&&!m_gapEnded){m_gapEnded=true;wantIrq=false;}
            if(m_gapEnded){m_transferComplete=true;m_readData=v;if(wantIrq)m_diskPending=true;}
        } else {
            if(!m_crcControl){m_transferComplete=true;v=m_writeData;if(wantIrq)m_diskPending=true;}
            if(!m_diskReady)v=0;
            if(!m_crcControl) updateCrc(v);
            else {
                if(!m_prevCrc){updateCrc(0);updateCrc(0);}
                v=uint8_t(m_crc&0xFF);m_crc>>=8;
            }
            if(m_diskPos>=2)side[m_diskPos-2]=v;
            m_gapEnded=false;
        }
        m_prevCrc=m_crcControl;++m_diskPos;if(m_diskPos>=side.size()){m_motor=false;m_endOfHead=true;}else m_delay=150;
    }
    bool irqActive()const override{return m_timerPending||m_diskPending;}
    float expansionAudioSample(bool = false)const override{return m_soundRegs?m_audio.sample():0.0f;}

    bool loadDiskImage(const std::vector<uint8_t>&raw)override{
        m_sides.clear();if(raw.empty())return false;size_t off=0,count=0;if(raw.size()>=16&&std::memcmp(raw.data(),"FDS\x1A",4)==0){count=raw[4];off=16;}else count=raw.size()/65500;if(!count||off+count*65500>raw.size())return false;
        for(size_t s=0;s<count;s++){const uint8_t*src=raw.data()+off+s*65500;std::vector<uint8_t>dst;dst.reserve(65500);dst.insert(dst.end(),28300/8,0);size_t j=0;uint32_t fileSize=0;while(j<65500){uint8_t type=src[j];size_t len=0;switch(type){case 1:len=56;break;case 2:len=2;break;case 3:len=16;if(j+15<65500)fileSize=uint32_t(src[j+13])|(uint32_t(src[j+14])<<8);break;case 4:len=1+fileSize;break;default:j=65500;continue;}if(!len||j+len>65500)break;dst.push_back(0x80);dst.insert(dst.end(),src+j,src+j+len);dst.push_back(0x4D);dst.push_back(0x62);dst.insert(dst.end(),976/8,0);j+=len;}if(dst.size()<65500)dst.resize(65500,0);m_sides.push_back(std::move(dst));}
        m_currentSide=m_sides.empty()?-1:0;m_diskPos=0;m_endOfHead=true;return !m_sides.empty();
    }
    std::size_t diskSideCount()const override{return m_sides.size();}
    int currentDiskSide()const override{return m_currentSide;}
    bool diskInserted()const override{return m_currentSide>=0&&size_t(m_currentSide)<m_sides.size();}
    bool setDiskSide(std::size_t s)override{if(s>=m_sides.size())return false;m_currentSide=int(s);m_diskPos=0;m_endOfHead=true;m_motor=false;return true;}
    void ejectDisk()override{m_currentSide=-1;m_motor=false;m_scanning=false;m_endOfHead=true;}

    std::size_t mapperBatterySize()const override{size_t n=4;for(auto&s:m_sides)n+=4+s.size();return n;}
    void saveMapperBattery(std::vector<uint8_t>&o)const override{fds_detail::put32(o,uint32_t(m_sides.size()));for(auto&s:m_sides){fds_detail::put32(o,uint32_t(s.size()));o.insert(o.end(),s.begin(),s.end());}}
    bool loadMapperBattery(const uint8_t*d,std::size_t n)override{const uint8_t*p=d,*e=d+n;uint32_t count;if(!fds_detail::get32(p,e,count)||count!=m_sides.size())return false;for(size_t i=0;i<count;i++){uint32_t len;if(!fds_detail::get32(p,e,len)||len!=m_sides[i].size()||size_t(e-p)<len)return false;std::memcpy(m_sides[i].data(),p,len);p+=len;}return p==e;}

    void saveState(std::vector<uint8_t>&o)const override{
        for(uint16_t v:{m_irqReload,m_irqCounter,m_crc})fds_detail::put16(o,v);put8(o,m_irqEnabled);put8(o,m_irqRepeat);put8(o,m_timerPending);put8(o,m_diskRegs);put8(o,m_soundRegs);put8(o,m_writeData);put8(o,m_motor);put8(o,m_resetTransfer);put8(o,m_readMode);put8(o,m_crcControl);put8(o,m_diskReady);put8(o,m_diskIrqEnabled);put8(o,m_ext);put8(o,m_badCrc);put8(o,m_endOfHead);put8(o,m_readData);put8(o,m_gapEnded);put8(o,m_scanning);put8(o,m_transferComplete);put8(o,m_diskPending);put8(o,m_prevCrc);fds_detail::put32(o,uint32_t(m_currentSide+1));fds_detail::put32(o,m_diskPos);fds_detail::put32(o,m_delay);put8(o,static_cast<uint8_t>(m_mirror));m_audio.save(o);saveMapperBattery(o);
    }
    bool loadState(const uint8_t*&p,const uint8_t*e)override{
        uint8_t b;uint32_t q;if(!fds_detail::get16(p,e,m_irqReload)||!fds_detail::get16(p,e,m_irqCounter)||!fds_detail::get16(p,e,m_crc))return false;
#define FDS_BOOL(x) if(!get8(p,e,b))return false;x=b
        FDS_BOOL(m_irqEnabled);FDS_BOOL(m_irqRepeat);FDS_BOOL(m_timerPending);FDS_BOOL(m_diskRegs);FDS_BOOL(m_soundRegs);if(!get8(p,e,m_writeData))return false;FDS_BOOL(m_motor);FDS_BOOL(m_resetTransfer);FDS_BOOL(m_readMode);FDS_BOOL(m_crcControl);FDS_BOOL(m_diskReady);FDS_BOOL(m_diskIrqEnabled);if(!get8(p,e,m_ext))return false;FDS_BOOL(m_badCrc);FDS_BOOL(m_endOfHead);if(!get8(p,e,m_readData))return false;FDS_BOOL(m_gapEnded);FDS_BOOL(m_scanning);FDS_BOOL(m_transferComplete);FDS_BOOL(m_diskPending);FDS_BOOL(m_prevCrc);
#undef FDS_BOOL
        if(!fds_detail::get32(p,e,q))return false;m_currentSide=int(q)-1;if(!fds_detail::get32(p,e,m_diskPos)||!fds_detail::get32(p,e,m_delay)||!get8(p,e,b))return false;m_mirror=static_cast<Mirror>(b);if(!m_audio.load(p,e))return false;uint32_t count;if(!fds_detail::get32(p,e,count)||count!=m_sides.size())return false;for(size_t i=0;i<count;i++){uint32_t len;if(!fds_detail::get32(p,e,len)||len!=m_sides[i].size()||size_t(e-p)<len)return false;std::memcpy(m_sides[i].data(),p,len);p+=len;}return true;
    }
private:
    void updateCrc(uint8_t value){
        for(uint16_t n=1;n<=0x80;n<<=1){uint8_t carry=uint8_t(m_crc&1);m_crc>>=1;if(carry)m_crc^=0x8408;if(value&n)m_crc^=0x8000;}
    }
    uint16_t m_irqReload=0,m_irqCounter=0,m_crc=0;bool m_irqEnabled=false,m_irqRepeat=false,m_timerPending=false,m_diskRegs=true,m_soundRegs=true;uint8_t m_writeData=0,m_ext=0,m_readData=0;bool m_motor=false,m_resetTransfer=false,m_readMode=false,m_crcControl=false,m_diskReady=false,m_diskIrqEnabled=false,m_badCrc=false,m_endOfHead=true,m_gapEnded=true,m_scanning=false,m_transferComplete=false,m_diskPending=false,m_prevCrc=false;int m_currentSide=-1;uint32_t m_diskPos=0,m_delay=0;std::vector<std::vector<uint8_t>>m_sides;fds_detail::Audio m_audio;
};
