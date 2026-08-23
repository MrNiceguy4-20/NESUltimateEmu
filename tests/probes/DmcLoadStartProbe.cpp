#include <cstdio>
#include "Bus.hpp"
#include "APU.hpp"

static int runCase(int preclocks) {
    Bus bus; APU apu;
    bus.connectAPU(&apu); apu.connectBus(&bus);
    bus.powerOn();
    for (int i=0;i<preclocks;i++) bus.clock();
    const auto writeCycle = bus.cpuCycleCounter();
    apu.cpuWrite(0x4010,0x00);
    apu.cpuWrite(0x4012,0x00);
    apu.cpuWrite(0x4013,0x00);
    apu.cpuWrite(0x4015,0x10);
    int onset=-1;
    for(int i=1;i<=8;i++) {
        bus.clock();
        if (onset<0 && bus.dmcDmaActive()) onset=i;
    }
    std::printf("writeParity=%llu onset=%d\n", (unsigned long long)(writeCycle&1), onset);
    return onset;
}
int runDmcLoadStartProbe(){
    int a=runCase(0); int b=runCase(1);
    bool ok=(a==4 && b==3) || (a==3 && b==4);
    std::puts(ok?"PASS":"FAIL");
    return ok?0:1;
}

#ifndef NES_PROBE_SUITE
int main() { return runDmcLoadStartProbe(); }
#endif
