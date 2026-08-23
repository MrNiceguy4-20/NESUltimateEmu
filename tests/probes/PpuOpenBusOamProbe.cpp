#include "PPU.hpp"
#include <cstdio>
int runPpuOpenBusOamProbe(){
    PPU p;
    p.testBypassRegisterWriteInhibit();

    p.cpuWrite(0x2002,0xFF);
    if(p.cpuRead(0x2000)!=0xFF){ std::puts("initial_openbus=FAIL"); return 1; }

    p.cpuWrite(0x2002,0xFF);
    for(unsigned long long i=0;i<5400000ull;i++) p.clock();
    unsigned dec=p.cpuRead(0x2000);
    std::printf("decayed=%02X\n",dec);
    if(dec!=0){ std::puts("decay=FAIL"); return 2; }

    p.cpuWrite(0x2003,2);
    p.cpuWrite(0x2004,0xFF);
    p.cpuWrite(0x2003,2);
    unsigned attr=p.cpuRead(0x2004);
    std::printf("attr=%02X\n",attr);
    if(attr!=0xE3){ std::puts("oam_attr=FAIL"); return 3; }

    unsigned attr2=p.cpuRead(0x2004);
    if(attr2!=0xE3){ std::puts("oam_read_increment=FAIL"); return 4; }

    PPU r;
    r.testBypassRegisterWriteInhibit();
    r.cpuWrite(0x2003,0x00);
    r.cpuWrite(0x2004,0x00);
    r.cpuWrite(0x2004,0x12);
    r.cpuWrite(0x2004,0xE3);
    r.cpuWrite(0x2004,0x34);
    r.cpuWrite(0x2003,0x00);
    r.cpuWrite(0x2001,0x18);
    for(int i=0;i<4;i++) r.clock();
    if(r.cpuRead(0x2004)!=0xFF){ std::puts("oam_render_clear_bus=FAIL"); return 5; }

    while(r.cycle()<66) r.clock();
    unsigned eval=r.cpuRead(0x2004);
    std::printf("eval_bus=%02X\n",eval);
    if(eval!=0x00){ std::puts("oam_render_eval_bus=FAIL"); return 6; }

    while(r.cycle()<258) r.clock();
    r.cpuWrite(0x2003,0x80);
    unsigned fetchTile=r.cpuRead(0x2004);
    std::printf("fetch_tile_bus=%02X\n",fetchTile);
    if(fetchTile!=0x12){ std::puts("oam_render_fetch_bus=FAIL"); return 7; }

    while(r.cycle()<260) r.clock();
    if(r.cpuRead(0x2004)!=0x34){ std::puts("oam_render_fetch_x_bus=FAIL"); return 8; }

    while(r.cycle()<321) r.clock();
    if(r.cpuRead(0x2004)!=0x00){ std::puts("oam_render_prefetch_bus=FAIL"); return 9; }

    PPU w;
    w.testBypassRegisterWriteInhibit();
    w.cpuWrite(0x2003,0x20);
    w.cpuWrite(0x2004,0x5A);
    w.cpuWrite(0x2003,0x20);
    w.cpuWrite(0x2001,0x18);
    while(w.cycle()<100) w.clock();
    w.cpuWrite(0x2004,0xA5);
    w.cpuWrite(0x2001,0x00);
    for(int i=0;i<4;i++) w.clock();
    w.cpuWrite(0x2003,0x20);
    if(w.cpuRead(0x2004)!=0x5A){ std::puts("oam_render_write_ignore=FAIL"); return 10; }

    PPU a;
    a.testBypassRegisterWriteInhibit();
    a.cpuWrite(0x2003,0x40);
    a.cpuWrite(0x2001,0x18);
    while(a.cycle()<258) a.clock();
    a.cpuWrite(0x2001,0x00);
    for(int i=0;i<4;i++) a.clock();
    a.cpuWrite(0x2004,0x77);
    a.cpuWrite(0x2003,0x00);
    if(a.cpuRead(0x2004)!=0x77){ std::puts("oamaddr_sprite_fetch_zero=FAIL"); return 11; }

    PPU s0;
    s0.testBypassRegisterWriteInhibit();
    s0.cpuWrite(0x2003,0x20);
    s0.cpuWrite(0x2004,0x00);
    s0.cpuWrite(0x2004,0x55);
    s0.cpuWrite(0x2004,0x00);
    s0.cpuWrite(0x2004,0x10);
    s0.cpuWrite(0x2003,0x20);
    s0.cpuWrite(0x2001,0x18);
    while(s0.cycle()<66) s0.clock();
    if(s0.cpuRead(0x2004)!=0x00){ std::puts("oamaddr_eval_start=FAIL"); return 12; }
    while(s0.cycle()<258) s0.clock();
    if(!s0.testSpriteZeroPossible()){ std::puts("arbitrary_sprite_zero=FAIL"); return 13; }

    PPU realign;
    realign.testBypassRegisterWriteInhibit();
    realign.cpuWrite(0x2003,0x21);
    realign.cpuWrite(0x2004,0xF0);
    realign.cpuWrite(0x2003,0x24);
    realign.cpuWrite(0x2004,0x33);
    realign.cpuWrite(0x2003,0x21);
    realign.cpuWrite(0x2001,0x18);
    while(realign.cycle()<68) realign.clock();
    if(realign.cpuRead(0x2004)!=0x33){ std::puts("oam_eval_plus4_realign=FAIL"); return 14; }

    PPU timed;
    timed.testBypassRegisterWriteInhibit();
    timed.cpuWrite(0x2003,0x00);
    timed.cpuWrite(0x2004,0x00);
    timed.cpuWrite(0x2004,0x6A);
    timed.cpuWrite(0x2004,0x00);
    timed.cpuWrite(0x2004,0x44);
    timed.cpuWrite(0x2003,0x00);
    timed.cpuWrite(0x2001,0x18);
    while(timed.cycle()<200) timed.clock();
    timed.cpuWrite(0x2001,0x00);
    while(timed.cycle()<254) timed.clock();
    timed.cpuWrite(0x2001,0x18);
    while(timed.cycle()<258) timed.clock();
    if(timed.cpuRead(0x2004)!=0x6A){ std::puts("timed_secondary_oam_persistence=FAIL"); return 15; }

    PPU phased;
    phased.testBypassRegisterWriteInhibit();
    phased.cpuWrite(0x2003,0x00);
    phased.cpuWrite(0x2004,0x00);
    phased.cpuWrite(0x2004,0x33);
    phased.cpuWrite(0x2004,0xA2);
    phased.cpuWrite(0x2004,0x5C);
    phased.cpuWrite(0x2003,0x00);
    phased.cpuWrite(0x2001,0x18);
    while(phased.cycle()<259) phased.clock();
    if(phased.testSpriteAttr(0)!=0x00 || phased.testSpriteX(0)!=0x00){
        std::puts("sprite_fetch_early_load=FAIL"); return 16;
    }
    phased.clock();
    if(phased.testSpriteAttr(0)!=0xA2 || phased.testSpriteX(0)!=0x00){
        std::puts("sprite_fetch_attr_phase=FAIL"); return 17;
    }
    phased.clock();
    if(phased.testSpriteX(0)!=0x5C){
        std::puts("sprite_fetch_x_phase=FAIL"); return 18;
    }

    auto writeRow = [](PPU& q, uint8_t base, const uint8_t* bytes) {
        q.cpuWrite(0x2003,base);
        for (int i=0;i<8;i++) q.cpuWrite(0x2004,bytes[i]);
    };
    auto rowMatches = [](PPU& q, uint8_t base, const uint8_t* expect) {
        for (int i=0;i<8;i++) {
            q.cpuWrite(0x2003,static_cast<uint8_t>(base+i));
            if (q.cpuRead(0x2004)!=expect[i]) return false;
        }
        return true;
    };
    const uint8_t sourceRow[8] = {0x21,0x22,0xE3,0x24,0x25,0x26,0xE3,0x28};
    const uint8_t targetRow[8] = {0x71,0x72,0xE3,0x74,0x75,0x76,0xE3,0x78};

    auto runCorruption = [&](PPU& q, bool expectCopy) {
        q.testBypassRegisterWriteInhibit();
        writeRow(q,0x00,sourceRow);

        for (uint8_t row=1; row<32; ++row) writeRow(q,static_cast<uint8_t>(row*8),targetRow);
        q.cpuWrite(0x2003,0x00);
        q.testSetTimingPosition(0,8);
        q.cpuWrite(0x2001,0x18);
        for(int i=0;i<4;i++) q.clock();

        for(int i=0;i<12;i++) q.clock();
        q.cpuWrite(0x2001,0x00);
        for(int i=0;i<4;i++) q.clock();
        const uint8_t seed = q.testOamCorruptionSeed();
        const bool armed = q.testOamCorruptionPending();
        q.cpuWrite(0x2001,0x18);
        for(int i=0;i<4;i++) q.clock();

        q.cpuWrite(0x2001,0x00);
        for(int i=0;i<4;i++) q.clock();
        const bool sourceIntact = rowMatches(q,0x00,sourceRow);
        if (!expectCopy)
            return !armed && rowMatches(q,0x20,targetRow) && sourceIntact;
        const uint8_t dst = static_cast<uint8_t>((seed & 0x1F) * 8u);
        const bool dest = rowMatches(q,dst,sourceRow);
        return armed && seed != 0 && dest && sourceIntact;
    };

    PPU corruptionNtsc;
    if(!runCorruption(corruptionNtsc,true)) {
        std::puts("oam_render_toggle_corruption_ntsc=FAIL"); return 19;
    }
    PPU corruptionPal;
    corruptionPal.setTiming(ConsoleTiming::PAL);
    if(!runCorruption(corruptionPal,false)) {
        std::puts("oam_render_toggle_corruption_pal=FAIL"); return 20;
    }
    PPU corruptionDendy;
    corruptionDendy.setTiming(ConsoleTiming::Dendy);
    if(!runCorruption(corruptionDendy,true)) {
        std::puts("oam_render_toggle_corruption_dendy=FAIL"); return 21;
    }
    std::puts("oam_render_toggle_corruption=ntsc:PASS pal:PASS dendy:PASS");

    std::puts("PASS");
    return 0;
}

#ifndef NES_PROBE_SUITE
int main() { return runPpuOpenBusOamProbe(); }
#endif
