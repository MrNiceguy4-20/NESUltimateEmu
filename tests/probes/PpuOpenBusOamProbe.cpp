#include "PPU.hpp"
#include <cstdio>
int main(){
    PPU p;
    // Any PPU register write drives all eight decay-latch bits.
    p.cpuWrite(0x2002,0xFF);
    if(p.cpuRead(0x2000)!=0xFF){ std::puts("initial_openbus=FAIL"); return 1; }
    // Re-drive then leave the PPU bus undriven for > 1 second NTSC.
    p.cpuWrite(0x2002,0xFF);
    for(unsigned long long i=0;i<5400000ull;i++) p.clock();
    unsigned dec=p.cpuRead(0x2000);
    std::printf("decayed=%02X\n",dec);
    if(dec!=0){ std::puts("decay=FAIL"); return 2; }
    // OAM attribute byte: bits 2-4 do not exist.
    p.cpuWrite(0x2003,2);
    p.cpuWrite(0x2004,0xFF);
    p.cpuWrite(0x2003,2);
    unsigned attr=p.cpuRead(0x2004);
    std::printf("attr=%02X\n",attr);
    if(attr!=0xE3){ std::puts("oam_attr=FAIL"); return 3; }
    // $2004 reads do not increment OAMADDR.
    unsigned attr2=p.cpuRead(0x2004);
    if(attr2!=0xE3){ std::puts("oam_read_increment=FAIL"); return 4; }
    std::puts("PASS");
    return 0;
}
