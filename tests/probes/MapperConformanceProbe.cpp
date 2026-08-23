#include "Mapper.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

namespace {
std::unique_ptr<Mapper> makeMapper(uint16_t id, std::size_t prgRom = 0x20000,
                                   std::size_t chrRom = 0x20000,
                                   std::size_t prgRam = 0x2000,
                                   std::size_t chrRam = 0,
                                   uint8_t submapper = 0, bool nes20 = false,
                                   Mirror mirror = Mirror::Horizontal, bool battery = false,
                                   uint8_t boardVariant = 0, bool fourScreen = false)
{
    MapperConfig c{};
    c.id = id;
    c.submapper = submapper;
    c.prgRomSize = prgRom;
    c.chrRomSize = chrRom;
    c.prgRamSize = prgRam;
    c.chrRamSize = chrRam;
    c.nes20 = nes20;
    c.headerMirror = mirror;
    c.fourScreen = fourScreen;
    c.hasBattery = battery;
    c.boardVariant = boardVariant;
    return createMapper(c);
}

void mmc1Serial(Mapper& m, uint16_t addr, uint8_t value, uint64_t firstCycle)
{
    for (unsigned bit = 0; bit < 5; ++bit)
        m.cpuWrite(addr, uint8_t((value >> bit) & 1), firstCycle + bit * 2);
}
}

int runMapperConformanceProbe()
{
    bool ok = true;

    // Namco 163's internal 128-byte sound RAM is mapper-owned persistent
    // storage only when the cartridge actually supplies battery backup. A
    // plain mapper-19 image must not acquire battery semantics merely because
    // the ASIC contains writable internal RAM.
    auto n163Volatile = makeMapper(19, 0x8000, 0x2000, 0, 0, 0, false, Mirror::Horizontal, false);
    auto n163Battery = makeMapper(19, 0x8000, 0x2000, 0, 0, 0, false, Mirror::Horizontal, true);
    const bool n163BatteryGate = n163Volatile->mapperBatterySize() == 0 &&
        n163Battery->mapperBatterySize() == 128;
    std::printf("n163_internal_ram_battery_gate=%s volatile=%zu battery=%zu\n",
        n163BatteryGate ? "PASS" : "FAIL", n163Volatile->mapperBatterySize(),
        n163Battery->mapperBatterySize());
    ok &= n163BatteryGate;

    // Phase 47 bundled multicart pass: seven small, well-defined iNES boards
    // share one implementation but retain their individual address/data wiring.
    auto m202 = makeMapper(202, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Vertical);
    m202->cpuWrite(0x800F, 0x00, 0); // bank 7 => NROM-256 pair 6/7, CHR 7, H mirror
    uint32_t m202l=0,m202h=0,m202c=0;
    const bool p47_202=m202->cpuMapRead(0x8123,m202l)&&m202->cpuMapRead(0xC123,m202h)&&
        m202->ppuMapRead(0x0456,m202c)&&m202l==0x18123&&m202h==0x1C123&&m202c==0x0E456&&
        m202->mirroring()==Mirror::Horizontal;
    std::vector<uint8_t> p47state; m202->saveState(p47state); m202->cpuWrite(0x8000,0xFF,0);
    const uint8_t* p47sp=p47state.data(); const uint8_t* p47se=p47sp+p47state.size();
    uint32_t p47restore=0; const bool p47_state=m202->loadState(p47sp,p47se)&&
        m202->cpuMapRead(0x8000,p47restore)&&p47restore==0x18000&&m202->mirroring()==Mirror::Horizontal;

    auto m203 = makeMapper(203, 0x10000, 0x8000, 0, 0);
    m203->cpuWrite(0x9234,0x0E,0); uint32_t m203l=0,m203h=0,m203c=0;
    const bool p47_203=m203->cpuMapRead(0x8123,m203l)&&m203->cpuMapRead(0xC456,m203h)&&
        m203->ppuMapRead(0x0456,m203c)&&m203l==0x0C123&&m203h==0x0C456&&m203c==0x04456;

    auto m204 = makeMapper(204, 0x20000, 0x10000, 0, 0);
    m204->cpuWrite(0x8006,0x10,0); uint32_t m204l=0,m204h=0,m204c=0;
    const bool p47_204=m204->cpuMapRead(0x8123,m204l)&&m204->cpuMapRead(0xC123,m204h)&&
        m204->ppuMapRead(0x0456,m204c)&&m204l==0x18123&&m204h==0x1C123&&m204c==0x0C456&&
        m204->mirroring()==Mirror::Horizontal;

    auto m214 = makeMapper(214, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Vertical);
    m214->cpuWrite(0x800C,0xAA,0); uint32_t m214l=0,m214h=0,m214c=0;
    const bool p47_214a=m214->cpuMapRead(0x8123,m214l)&&m214->cpuMapRead(0xC456,m214h)&&
        m214->ppuMapRead(0x0456,m214c)&&m214l==0x0C123&&m214h==0x0C456&&m214c==0x06456;
    m214->cpuWrite(0x9000,0x00,0); uint32_t m214ignored=0;
    const bool p47_214=p47_214a&&m214->cpuMapRead(0x8000,m214ignored)&&m214ignored==0x0C000;

    auto m217 = makeMapper(217, 0x40000, 0x10000, 0, 0);
    m217->cpuWrite(0x8017,0xFF,0); uint32_t m217p=0,m217c=0;
    const bool p47_217=m217->cpuMapRead(0x8123,m217p)&&m217->ppuMapRead(0x0456,m217c)&&
        m217p==0x28123&&m217c==0x0E456;

    auto m229 = makeMapper(229, 0x80000, 0x40000, 0, 0);
    m229->cpuWrite(0x803F,0x00,0); uint32_t m229l=0,m229h=0,m229c=0;
    const bool p47_229=m229->cpuMapRead(0x8123,m229l)&&m229->cpuMapRead(0xC123,m229h)&&
        m229->ppuMapRead(0x0456,m229c)&&m229l==0x7C123&&m229h==0x7C123&&m229c==0x3E456&&
        m229->mirroring()==Mirror::Horizontal;

    auto m231 = makeMapper(231, 0x80000, 0x2000, 0, 0);
    m231->cpuWrite(0x80BE,0x00,0); uint32_t m231l=0,m231h=0;
    const bool p47_231=m231->cpuMapRead(0x8123,m231l)&&m231->cpuMapRead(0xC123,m231h)&&
        m231l==0x78123&&m231h==0x7C123&&m231->mirroring()==Mirror::Horizontal;

    const bool p47_gate=mapperImplementationSupported(202,0)&&mapperImplementationSupported(203,0)&&
        mapperImplementationSupported(204,0)&&mapperImplementationSupported(214,0)&&
        mapperImplementationSupported(217,0)&&mapperImplementationSupported(229,0)&&
        mapperImplementationSupported(231,0)&&!mapperImplementationSupported(202,1)&&
        !mapperImplementationSupported(203,1)&&!mapperImplementationSupported(204,1)&&
        !mapperImplementationSupported(214,1)&&!mapperImplementationSupported(217,1)&&
        !mapperImplementationSupported(229,1)&&!mapperImplementationSupported(231,1);
    const bool phase47=p47_202&&p47_203&&p47_204&&p47_214&&p47_217&&p47_229&&p47_231&&p47_state&&p47_gate;
    std::printf("phase47_multicart_bundle 202=%s 203=%s 204=%s 214=%s 217=%s 229=%s 231=%s state=%s gate=%s\n",
        p47_202?"PASS":"FAIL",p47_203?"PASS":"FAIL",p47_204?"PASS":"FAIL",p47_214?"PASS":"FAIL",
        p47_217?"PASS":"FAIL",p47_229?"PASS":"FAIL",p47_231?"PASS":"FAIL",p47_state?"PASS":"FAIL",
        p47_gate?"PASS":"FAIL");
    ok &= phase47;

    // Phase 38: Mapper 153 / Bandai LZ93D50 + 8 KiB WRAM.  The four
    // former CHR bank outputs at $8000-$8003 become a 256 KiB outer PRG
    // selector indexed by live PPU A11:A10. CHR is fixed 8 KiB RAM.
    auto m153 = makeMapper(153, 0x80000, 0, 0x2000, 0x2000, 0, true, Mirror::Horizontal, true);
    m153->cpuWrite(0x8000,0,0); m153->cpuWrite(0x8001,1,0);
    m153->cpuWrite(0x8002,0,0); m153->cpuWrite(0x8003,1,0);
    m153->cpuWrite(0x8008,2,0);
    uint32_t m153chr0=0,m153lo0=0,m153hi0=0,m153chr1=0,m153lo1=0,m153hi1=0;
    const bool m153ppu0=m153->ppuMapRead(0x0012,m153chr0);
    const bool m153cpu0=m153->cpuMapRead(0x8123,m153lo0)&&m153->cpuMapRead(0xC123,m153hi0);
    const bool m153ppu1=m153->ppuMapRead(0x0456,m153chr1);
    const bool m153cpu1=m153->cpuMapRead(0x8123,m153lo1)&&m153->cpuMapRead(0xC123,m153hi1);
    const bool m153Chr=m153ppu0&&m153ppu1&&m153chr0==0x0012&&m153chr1==0x0456;
    const bool m153Outer=m153Chr&&m153cpu0&&m153cpu1&&m153lo0==0x08123&&m153hi0==0x3C123&&
        m153lo1==0x48123&&m153hi1==0x7C123;
    uint32_t m153ram=0;
    const bool m153RamOff=!m153->mapPrgRam(0x6123,m153ram,false);
    m153->cpuWrite(0x800D,0x20,0);
    const bool m153RamOn=m153->mapPrgRam(0x6123,m153ram,false)&&m153ram==0x0123;
    m153->cpuWrite(0x8009,1,0); const bool m153Mirror=m153->mirroring()==Mirror::Horizontal;
    m153->cpuWrite(0x800B,1,0); m153->cpuWrite(0x800C,0,0); m153->cpuWrite(0x800A,1,0);
    m153->clockCpu(); const bool m153IrqBefore=!m153->irqActive();
    m153->clockCpu(); const bool m153IrqHit=m153->irqActive();
    m153->cpuWrite(0x800A,0,0); const bool m153IrqAck=!m153->irqActive();
    std::vector<uint8_t> m153state; m153->saveState(m153state);
    m153->ppuMapRead(0x0010,m153chr0); m153->cpuWrite(0x8008,0,0); m153->cpuWrite(0x800D,0,0);
    const uint8_t* m153sp=m153state.data(); const uint8_t* m153se=m153sp+m153state.size();
    const bool m153load=m153->loadState(m153sp,m153se); uint32_t m153rp=0,m153rr=0;
    m153->cpuMapRead(0x8000,m153rp); const bool m153RamRestored=m153->mapPrgRam(0x6000,m153rr,false);
    const bool m153State=m153load&&m153rp==0x48000&&m153RamRestored;
    const bool m153Gate=mapperImplementationSupported(153,0)&&!mapperImplementationSupported(153,1);
    const bool mapper153=m153Outer&&m153RamOff&&m153RamOn&&m153Mirror&&m153IrqBefore&&m153IrqHit&&
        m153IrqAck&&m153State&&m153Gate;
    std::printf("phase38_mapper153 outer=%s chr=%s ram=%s irq=%s state=%s gate=%s\n",
        m153Outer?"PASS":"FAIL",m153Chr?"PASS":"FAIL",
        (m153RamOff&&m153RamOn)?"PASS":"FAIL",(m153IrqBefore&&m153IrqHit&&m153IrqAck)?"PASS":"FAIL",
        m153State?"PASS":"FAIL",m153Gate?"PASS":"FAIL");
    ok &= mapper153;

    // Phase 39: Mapper 125 / UNL-LH32. The FDS-conversion board exposes a
    // switchable 8 KiB PRG bank at $6000, overlays WRAM at $C000-$DFFF, and
    // keeps the final 32 KiB of PRG fixed in the normal upper CPU window.
    auto m125 = makeMapper(125, 0x20000, 0, 0x2000, 0x2000);
    uint32_t m125boot=0,m125fixed0=0,m125fixed1=0,m125ram=0,m125chr=0;
    const bool m125BootRead=m125->cpuMapRead(0x6000,m125boot);
    const bool m125FixedRead=m125->cpuMapRead(0x8000,m125fixed0)&&m125->cpuMapRead(0xE000,m125fixed1);
    const bool m125Ram=m125->mapPrgRam(0xC123,m125ram,false)&&m125ram==0x0123&&
        !m125->mapPrgRam(0xBFFF,m125ram,false)&&!m125->mapPrgRam(0xE000,m125ram,false);
    const bool m125Chr=m125->ppuMapRead(0x1234,m125chr)&&m125chr==0x1234;
    m125->cpuWrite(0x6000,0x03,0); uint32_t m125bank3=0; m125->cpuMapRead(0x6123,m125bank3);
    m125->cpuWrite(0x6001,0x07,0); uint32_t m125ignored=0; m125->cpuMapRead(0x6123,m125ignored);
    std::vector<uint8_t> m125state; m125->saveState(m125state);
    m125->cpuWrite(0x6000,0x09,0);
    const uint8_t* m125sp=m125state.data(); const uint8_t* m125se=m125sp+m125state.size();
    const bool m125load=m125->loadState(m125sp,m125se); uint32_t m125restored=0; m125->cpuMapRead(0x6000,m125restored);
    m125->reset(false); uint32_t m125reset=0; m125->cpuMapRead(0x6000,m125reset);
    const bool m125Gate=mapperImplementationSupported(125,0)&&!mapperImplementationSupported(125,1);
    const bool mapper125=m125BootRead&&m125boot==0x1E000&&m125FixedRead&&m125fixed0==0x18000&&m125fixed1==0x1E000&&
        m125Ram&&m125Chr&&m125bank3==0x06123&&m125ignored==0x06123&&m125load&&m125restored==0x06000&&
        m125reset==0x1E000&&m125Gate;
    std::printf("phase39_mapper125 prg=%s ram=%s chr=%s decode=%s state=%s reset=%s gate=%s\n",
        (m125BootRead&&m125boot==0x1E000&&m125FixedRead&&m125fixed0==0x18000&&m125fixed1==0x1E000&&m125bank3==0x06123)?"PASS":"FAIL",
        m125Ram?"PASS":"FAIL",m125Chr?"PASS":"FAIL",m125ignored==0x06123?"PASS":"FAIL",
        (m125load&&m125restored==0x06000)?"PASS":"FAIL",m125reset==0x1E000?"PASS":"FAIL",m125Gate?"PASS":"FAIL");
    ok &= mapper125;

    // Phase 40: mapper 122 is the historical fwNES assignment for the same
    // Sunsoft-1 hardware standardized as mapper 184. Both numbers must expose
    // identical fixed PRG and split 4 KiB CHR banking behavior.
    auto m122 = makeMapper(122, 0x8000, 0x20000, 0, 0);
    auto m184alias = makeMapper(184, 0x8000, 0x20000, 0, 0);
    uint32_t m122prg0=0,m122prg1=0,m184prg0=0,m184prg1=0;
    const bool m122Prg = m122->cpuMapRead(0x8000,m122prg0) && m122->cpuMapRead(0xFFFF,m122prg1) &&
        m184alias->cpuMapRead(0x8000,m184prg0) && m184alias->cpuMapRead(0xFFFF,m184prg1) &&
        m122prg0==m184prg0 && m122prg1==m184prg1 && m122prg0==0x0000 && m122prg1==0x7FFF;
    m122->cpuWrite(0x6000,0x53,0); m184alias->cpuWrite(0x7FFF,0x53,0);
    uint32_t m122chrLo=0,m122chrHi=0,m184chrLo=0,m184chrHi=0;
    const bool m122Chr = m122->ppuMapRead(0x0123,m122chrLo) && m122->ppuMapRead(0x1456,m122chrHi) &&
        m184alias->ppuMapRead(0x0123,m184chrLo) && m184alias->ppuMapRead(0x1456,m184chrHi) &&
        m122chrLo==m184chrLo && m122chrHi==m184chrHi && m122chrLo==0x03123 && m122chrHi==0x05456;
    uint32_t m122ram=0;
    const bool m122NoRam = !m122->mapPrgRam(0x6000,m122ram,false);
    std::vector<uint8_t> m122state; m122->saveState(m122state);
    m122->cpuWrite(0x6000,0x10,0);
    const uint8_t* m122sp=m122state.data(); const uint8_t* m122se=m122sp+m122state.size();
    const bool m122load=m122->loadState(m122sp,m122se); uint32_t m122restored=0;
    const bool m122State=m122load&&m122->ppuMapRead(0x0123,m122restored)&&m122restored==0x03123;
    const bool m122Gate=mapperImplementationSupported(122,0)&&!mapperImplementationSupported(122,1)&&
        mapperImplementationSupported(184,0)&&!mapperImplementationSupported(184,1);
    const bool mapper122=m122Prg&&m122Chr&&m122NoRam&&m122State&&m122Gate;
    std::printf("phase40_mapper122 alias=%s prg=%s chr=%s state=%s gate=%s\n",
        mapper122?"PASS":"FAIL",m122Prg?"PASS":"FAIL",m122Chr?"PASS":"FAIL",
        m122State?"PASS":"FAIL",m122Gate?"PASS":"FAIL");
    ok &= mapper122;

    // Phase 41: iNES Mapper 155 identifies the MMC1A silicon revision.
    // Unlike MMC1B/C, PRG-register bit 4 does not disable WRAM. When that
    // bit is set in 16 KiB PRG modes, PRG bit 3 directly drives A17 even for
    // the fixed bank. In mode 3 with bit 3 clear, MMC1A therefore fixes
    // $C000-$FFFF to bank 7 of the low 128 KiB half, while MMC1B/C fixes it
    // to bank 15 of the full 256 KiB region.
    auto mmc1b = makeMapper(1, 0x40000, 0x2000, 0x2000, 0);
    auto mmc1a = makeMapper(155, 0x40000, 0x2000, 0x2000, 0);
    mmc1Serial(*mmc1b, 0x8000, 0x0C, 100);
    mmc1Serial(*mmc1a, 0x8000, 0x0C, 100);
    mmc1Serial(*mmc1b, 0xE000, 0x10, 120);
    mmc1Serial(*mmc1a, 0xE000, 0x10, 120);
    uint32_t mmc1bLo=0,mmc1bHi=0,mmc1aLo=0,mmc1aHi=0,mmc1bRam=0,mmc1aRam=0;
    const bool mmc1bMap=mmc1b->cpuMapRead(0x8000,mmc1bLo)&&mmc1b->cpuMapRead(0xC000,mmc1bHi);
    const bool mmc1aMap=mmc1a->cpuMapRead(0x8000,mmc1aLo)&&mmc1a->cpuMapRead(0xC000,mmc1aHi);
    const bool mmc1RevPrg=mmc1bMap&&mmc1aMap&&mmc1bLo==0x00000&&mmc1aLo==0x00000&&
        mmc1bHi==0x3C000&&mmc1aHi==0x1C000;
    const bool mmc1RevRam=!mmc1b->mapPrgRam(0x6123,mmc1bRam,false)&&
        mmc1a->mapPrgRam(0x6123,mmc1aRam,false)&&mmc1aRam==0x0123;
    // With PRG bit 3 set, MMC1A selects the upper 128 KiB half for both the
    // switchable and fixed windows.
    mmc1Serial(*mmc1a, 0xE000, 0x18, 140);
    uint32_t mmc1aUpperLo=0,mmc1aUpperHi=0;
    const bool mmc1RevUpper=mmc1a->cpuMapRead(0x8000,mmc1aUpperLo)&&
        mmc1a->cpuMapRead(0xC000,mmc1aUpperHi)&&mmc1aUpperLo==0x20000&&mmc1aUpperHi==0x3C000;
    const bool mmc1aGate=mapperImplementationSupported(155,0)&&mapperImplementationSupported(155,5)&&
        !mapperImplementationSupported(155,1)&&!mapperImplementationSupported(155,6)&&!mapperImplementationSupported(155,7);
    const bool phase41=mmc1RevPrg&&mmc1RevRam&&mmc1RevUpper&&mmc1aGate;
    std::printf("phase41_mapper155 prg=%s ram=%s upper=%s gate=%s\n",
        mmc1RevPrg?"PASS":"FAIL",mmc1RevRam?"PASS":"FAIL",mmc1RevUpper?"PASS":"FAIL",mmc1aGate?"PASS":"FAIL");
    ok &= phase41;

    // Phase 42: Mapper 174 / NTDEC 5-in-1.  The write address itself is
    // the register: A7 selects 16/32 KiB PRG mode, A6-A4 select PRG,
    // A3-A1 select CHR, and A0 selects H/V mirroring.  Soft reset preserves
    // the selected game; only hard reset returns the latch to zero.
    auto m174 = makeMapper(174, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Horizontal);
    m174->reset(true);
    uint32_t m174bootLo=0,m174bootHi=0,m174bootChr=0;
    const bool m174Boot=m174->cpuMapRead(0x8123,m174bootLo)&&m174->cpuMapRead(0xC123,m174bootHi)&&
        m174->ppuMapRead(0x0456,m174bootChr)&&m174bootLo==0x0123&&m174bootHi==0x0123&&
        m174bootChr==0x0456&&m174->mirroring()==Mirror::Vertical;

    // 0x80DD: O=1, P=5, C=6, M=1. In 32 KiB mode P0 is ignored, so P=5
    // selects 32 KiB bank 2. The data byte is deliberately arbitrary.
    const bool m174Write32=m174->cpuWrite(0x80DD,0x00,0);
    uint32_t m174p32a=0,m174p32b=0,m174c32=0;
    const bool m174Mode32=m174->cpuMapRead(0x8123,m174p32a)&&m174->cpuMapRead(0xE123,m174p32b)&&
        m174->ppuMapRead(0x0456,m174c32)&&m174p32a==0x10123&&m174p32b==0x16123&&
        m174c32==0x0C456&&m174->mirroring()==Mirror::Horizontal;

    // 0x8034: O=0, P=3, C=2, M=0. The selected 16 KiB PRG bank is mirrored
    // into both halves of the CPU ROM window.
    const bool m174Write16=m174->cpuWrite(0x8034,0xFF,0);
    uint32_t m174p16a=0,m174p16b=0,m174c16=0;
    const bool m174Mode16=m174->cpuMapRead(0x8123,m174p16a)&&m174->cpuMapRead(0xC123,m174p16b)&&
        m174->ppuMapRead(0x0456,m174c16)&&m174p16a==0x0C123&&m174p16b==0x0C123&&
        m174c16==0x04456&&m174->mirroring()==Mirror::Vertical;

    std::vector<uint8_t> m174state; m174->saveState(m174state);
    m174->reset(false); uint32_t m174soft=0; const bool m174Soft=m174->cpuMapRead(0x8000,m174soft)&&m174soft==0x0C000;
    m174->cpuWrite(0x8081,0x5A,0);
    const uint8_t* m174sp=m174state.data(); const uint8_t* m174se=m174sp+m174state.size();
    const bool m174load=m174->loadState(m174sp,m174se); uint32_t m174restored=0;
    const bool m174State=m174load&&m174->cpuMapRead(0x8000,m174restored)&&m174restored==0x0C000&&
        m174->mirroring()==Mirror::Vertical;
    m174->reset(true); uint32_t m174hard=0;
    const bool m174Hard=m174->cpuMapRead(0x8000,m174hard)&&m174hard==0&&m174->mirroring()==Mirror::Vertical;
    const bool m174Gate=mapperImplementationSupported(174,0)&&!mapperImplementationSupported(174,1);
    const bool phase42=m174Boot&&m174Write32&&m174Mode32&&m174Write16&&m174Mode16&&m174Soft&&m174State&&m174Hard&&m174Gate;
    std::printf("phase42_mapper174 boot=%s prg32=%s prg16=%s chr=%s soft=%s state=%s hard=%s gate=%s\n",
        m174Boot?"PASS":"FAIL",m174Mode32?"PASS":"FAIL",m174Mode16?"PASS":"FAIL",
        (m174c32==0x0C456&&m174c16==0x04456)?"PASS":"FAIL",m174Soft?"PASS":"FAIL",
        m174State?"PASS":"FAIL",m174Hard?"PASS":"FAIL",m174Gate?"PASS":"FAIL");
    ok &= phase42;


    // Phase 43: Mapper 200 simple NROM multicart. A2-A0 select the shared
    // 16 KiB PRG / 8 KiB CHR bank and A3 selects H/V mirroring. PRG is
    // mirrored into both CPU halves and the data byte is ignored.
    auto m200 = makeMapper(200, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Horizontal);
    m200->reset(true);
    uint32_t m200bootLo=0,m200bootHi=0,m200bootChr=0;
    const bool m200Boot=m200->cpuMapRead(0x8123,m200bootLo)&&m200->cpuMapRead(0xC123,m200bootHi)&&
        m200->ppuMapRead(0x0456,m200bootChr)&&m200bootLo==0x0123&&m200bootHi==0x0123&&
        m200bootChr==0x0456&&m200->mirroring()==Mirror::Vertical;

    // Low address bits 0b101 select shared bank 5; A3=1 selects horizontal.
    // The data byte deliberately disagrees with the address to verify it is ignored.
    const bool m200Write=m200->cpuWrite(0x800D,0x00,0);
    uint32_t m200pLo=0,m200pHi=0,m200c=0;
    const bool m200Banks=m200->cpuMapRead(0x8123,m200pLo)&&m200->cpuMapRead(0xC123,m200pHi)&&
        m200->ppuMapRead(0x0456,m200c)&&m200pLo==0x14123&&m200pHi==0x14123&&
        m200c==0x0A456&&m200->mirroring()==Mirror::Horizontal;
    const bool m200Ignored=!m200->cpuWrite(0x7FFF,0xFF,0);

    // A3=0 returns to vertical mirroring while retaining address-selected banking.
    m200->cpuWrite(0x8002,0xFF,0);
    uint32_t m200p2=0,m200c2=0;
    const bool m200AddressLatch=m200->cpuMapRead(0x8000,m200p2)&&m200->ppuMapRead(0x0000,m200c2)&&
        m200p2==0x08000&&m200c2==0x04000&&m200->mirroring()==Mirror::Vertical;

    std::vector<uint8_t> m200state; m200->saveState(m200state);
    m200->cpuWrite(0x8007,0x00,0);
    const uint8_t* m200sp=m200state.data(); const uint8_t* m200se=m200sp+m200state.size();
    const bool m200load=m200->loadState(m200sp,m200se); uint32_t m200rp=0,m200rc=0;
    const bool m200State=m200load&&m200->cpuMapRead(0x8000,m200rp)&&m200->ppuMapRead(0x0000,m200rc)&&
        m200rp==0x08000&&m200rc==0x04000&&m200->mirroring()==Mirror::Vertical;
    m200->reset(false); uint32_t m200soft=0;
    const bool m200Soft=m200->cpuMapRead(0x8000,m200soft)&&m200soft==0x08000;
    m200->reset(true); uint32_t m200hard=0;
    const bool m200Hard=m200->cpuMapRead(0x8000,m200hard)&&m200hard==0&&m200->mirroring()==Mirror::Vertical;
    const bool m200Gate=mapperImplementationSupported(200,0)&&!mapperImplementationSupported(200,1);
    const bool phase43=m200Boot&&m200Write&&m200Banks&&m200Ignored&&m200AddressLatch&&m200State&&m200Soft&&m200Hard&&m200Gate;
    std::printf("phase43_mapper200 boot=%s banks=%s latch=%s state=%s soft=%s hard=%s gate=%s\n",
        m200Boot?"PASS":"FAIL",m200Banks?"PASS":"FAIL",m200AddressLatch?"PASS":"FAIL",
        m200State?"PASS":"FAIL",m200Soft?"PASS":"FAIL",m200Hard?"PASS":"FAIL",m200Gate?"PASS":"FAIL");
    ok &= phase43;

    // MMC1: D7 reset is never subject to the consecutive-cycle serial-data
    // suppression. Start a partial load, reset on the immediately following
    // CPU cycle, then load PRG bank 2. If reset were incorrectly ignored, the
    // partial serial state would corrupt/early-commit the following value.
    auto mmc1 = makeMapper(1);
    mmc1->cpuWrite(0xE000, 1, 10);
    mmc1->cpuWrite(0xE000, 0x80, 11);
    mmc1Serial(*mmc1, 0xE000, 2, 13);
    uint32_t mmc1Mapped = 0;
    const bool mmc1Read = mmc1->cpuMapRead(0x8000, mmc1Mapped);
    const bool mmc1ResetWins = mmc1Read && mmc1Mapped == 0x8000;
    std::printf("mmc1_consecutive_reset=%s mapped=%05X\n",
        mmc1ResetWins ? "PASS" : "FAIL", unsigned(mmc1Mapped));
    ok &= mmc1ResetWins;

    // NES 2.0 Mapper 1 submapper 5 (SEROM/SHROM/SH1ROM) has 32 KiB of
    // physically unbanked PRG-ROM. Even if software programs the MMC1 into
    // 16 KiB switching mode and selects bank 1, $8000 must remain the first
    // half of the ROM rather than aliasing the fixed upper half. Submappers
    // 6 (2ME EEPROM) and 7 (KS-7058) are intentionally not advertised until
    // their additional board hardware is implemented.
    auto mmc1s5 = makeMapper(1, 0x8000, 0x2000, 0, 0, 5, true);
    mmc1Serial(*mmc1s5, 0x8000, 0x0C, 30); // 16 KiB switch at $8000
    mmc1Serial(*mmc1s5, 0xE000, 0x01, 40); // would select bank 1 normally
    uint32_t mmc1s5Lo=0, mmc1s5Hi=0;
    const bool mmc1s5ReadLo = mmc1s5->cpuMapRead(0x8000, mmc1s5Lo);
    const bool mmc1s5ReadHi = mmc1s5->cpuMapRead(0xC000, mmc1s5Hi);
    const bool mmc1s5Prg = mmc1s5ReadLo && mmc1s5ReadHi && mmc1s5Lo == 0x0000 && mmc1s5Hi == 0x4000;
    const bool mmc1SubGate = mapperImplementationSupported(1,0) && mapperImplementationSupported(1,5) &&
        !mapperImplementationSupported(1,6) && !mapperImplementationSupported(1,7);
    std::printf("mmc1_submapper5=%s prg=%04X/%04X gate=%s\n",
        (mmc1s5Prg && mmc1SubGate) ? "PASS" : "FAIL", unsigned(mmc1s5Lo), unsigned(mmc1s5Hi),
        mmc1SubGate ? "PASS" : "FAIL");
    ok &= mmc1s5Prg && mmc1SubGate;

    // NES 2.0 submappers 1/2 disambiguate bus-conflict behavior for the
    // discrete UxROM/CNROM/AxROM families. Legacy/submapper-0 UxROM and CNROM
    // use the strict hardware-compatible conflict default, while AxROM keeps
    // its compatibility default of no conflicts.
    auto m2Legacy = makeMapper(2);
    auto m2s0 = makeMapper(2, 0x20000, 0, 0, 0x2000, 0, true);
    auto m2s1 = makeMapper(2, 0x20000, 0, 0, 0x2000, 1, true);
    auto m2s2 = makeMapper(2, 0x20000, 0, 0, 0x2000, 2, true);
    auto m3Legacy = makeMapper(3);
    auto m3s0 = makeMapper(3, 0x8000, 0x8000, 0, 0, 0, true);
    auto m3s1 = makeMapper(3, 0x8000, 0x8000, 0, 0, 1, true);
    auto m3s2 = makeMapper(3, 0x8000, 0x8000, 0, 0, 2, true);
    auto m7Legacy = makeMapper(7);
    auto m7s0 = makeMapper(7, 0x40000, 0, 0, 0x2000, 0, true);
    auto m7s1 = makeMapper(7, 0x40000, 0, 0, 0x2000, 1, true);
    auto m7s2 = makeMapper(7, 0x40000, 0, 0, 0x2000, 2, true);
    const bool discreteBusConflicts =
        m2Legacy->hasBusConflicts() && m2s0->hasBusConflicts() &&
        !m2s1->hasBusConflicts() && m2s2->hasBusConflicts() &&
        m3Legacy->hasBusConflicts() && m3s0->hasBusConflicts() &&
        !m3s1->hasBusConflicts() && m3s2->hasBusConflicts() &&
        !m7Legacy->hasBusConflicts() && !m7s0->hasBusConflicts() &&
        !m7s1->hasBusConflicts() && m7s2->hasBusConflicts() &&
        mapperImplementationSupported(2,0) && mapperImplementationSupported(2,1) && mapperImplementationSupported(2,2) &&
        mapperImplementationSupported(3,0) && mapperImplementationSupported(3,1) && mapperImplementationSupported(3,2) &&
        mapperImplementationSupported(7,0) && mapperImplementationSupported(7,1) && mapperImplementationSupported(7,2);
    std::printf("discrete_bus_conflicts=%s uxrom=%d/%d/%d cnrom=%d/%d/%d axrom=%d/%d/%d\n",
        discreteBusConflicts ? "PASS" : "FAIL",
        int(m2s0->hasBusConflicts()), int(m2s1->hasBusConflicts()), int(m2s2->hasBusConflicts()),
        int(m3s0->hasBusConflicts()), int(m3s1->hasBusConflicts()), int(m3s2->hasBusConflicts()),
        int(m7s0->hasBusConflicts()), int(m7s1->hasBusConflicts()), int(m7s2->hasBusConflicts()));
    ok &= discreteBusConflicts;

    // Phase 37: Mapper 132 / TXC 22111.  Four low registers live in the
    // $4100 decode region.  Reads matching mask $E100 expose (R1 XOR R2)|$40,
    // while a high write copies R2 to the PRG/CHR output latch.  Changes to
    // R2 must therefore remain invisible to banking until another high write.
    auto m132 = makeMapper(132, 0x20000, 0x8000, 0, 0, 0, true, Mirror::Vertical);
    const bool m132w1 = m132->cpuWrite(0x4101, 0x05, 0);
    const bool m132w2 = m132->cpuWrite(0x4302, 0x0A, 0); // $4102 alias under $E103
    uint8_t m132read0=0, m132readAlias=0;
    const bool m132r0 = m132->cpuReadRegister(0x4100, m132read0);
    const bool m132ra = m132->cpuReadRegister(0x43FC, m132readAlias); // read mask $E100
    const bool m132Protection = m132w1 && m132w2 && m132r0 && m132ra &&
        m132read0 == 0x4F && m132readAlias == 0x4F;

    // Bank outputs are still reset to zero before the high-address latch.
    uint32_t m132p0=0,m132c0=0;
    const bool m132Default = m132->cpuMapRead(0x8123,m132p0) && m132->ppuMapRead(0x0456,m132c0) &&
        m132p0==0x0123 && m132c0==0x0456;
    const bool m132Latch = m132->cpuWrite(0xA123,0xFF,0); // data byte ignored
    uint32_t m132p=0,m132c=0;
    m132->cpuMapRead(0x8123,m132p); m132->ppuMapRead(0x0456,m132c);
    const bool m132Banks = m132p==0x10123 && m132c==0x04456;

    // Stage a different R2 but do not latch it yet.
    m132->cpuWrite(0x4102,0x05,0);
    uint32_t m132pHold=0,m132cHold=0;
    m132->cpuMapRead(0x8000,m132pHold); m132->ppuMapRead(0x0000,m132cHold);
    const bool m132Hold = m132pHold==0x10000 && m132cHold==0x04000;
    m132->cpuWrite(0xFFFF,0x00,0);
    uint32_t m132pNew=0,m132cNew=0;
    m132->cpuMapRead(0x8000,m132pNew); m132->ppuMapRead(0x0000,m132cNew);
    const bool m132Relatch = m132pNew==0x08000 && m132cNew==0x02000;

    std::vector<uint8_t> m132state; m132->saveState(m132state);
    m132->cpuWrite(0x4102,0,0); m132->cpuWrite(0x8000,0,0);
    const uint8_t* m132sp=m132state.data(); const uint8_t* m132se=m132sp+m132state.size();
    const bool m132load=m132->loadState(m132sp,m132se); uint32_t m132rp=0,m132rc=0;
    m132->cpuMapRead(0x8000,m132rp); m132->ppuMapRead(0x0000,m132rc);
    const bool m132State=m132load&&m132rp==0x08000&&m132rc==0x02000;
    const bool m132NoRam=[&]{uint32_t x=0;return !m132->mapPrgRam(0x6000,x,false);}();
    const bool m132Gate=mapperImplementationSupported(132,0)&&!mapperImplementationSupported(132,1);
    const bool mapper132=m132Protection&&m132Default&&m132Latch&&m132Banks&&m132Hold&&m132Relatch&&
        m132State&&m132NoRam&&m132Gate&&m132->mirroring()==Mirror::Vertical;
    std::printf("phase37_mapper132 protection=%s banks=%s latch=%s state=%s gate=%s\n",
        m132Protection?"PASS":"FAIL",(m132Default&&m132Banks)?"PASS":"FAIL",
        (m132Hold&&m132Relatch)?"PASS":"FAIL",m132State?"PASS":"FAIL",m132Gate?"PASS":"FAIL");
    ok &= mapper132;

    // Mapper 152 is the one-screen-mirroring variant historically split out
    // of Mapper 70. Unlike Mapper 70, its discrete register overlaps PRG-ROM
    // output and is subject to wired-AND bus conflicts. Verify that the
    // conflicted value controls PRG, CHR, and the one-screen mirroring bit.
    auto m70 = makeMapper(70, 0x20000, 0x20000, 0, 0, 0, true);
    auto m152 = makeMapper(152, 0x20000, 0x20000, 0, 0, 0, true);
    const uint8_t m152Effective = m152->resolveBusConflict(0x8000, 0xFF, 0x2B);
    m152->cpuWrite(0x8000, m152Effective, 0);
    uint32_t m152Prg = 0, m152Chr = 0;
    const bool m152Banks = m152->cpuMapRead(0x8000, m152Prg) &&
        m152->ppuMapRead(0x0000, m152Chr) && m152Prg == 0x8000 &&
        m152Chr == 0x16000;
    const bool mapper152 = !m70->hasBusConflicts() && m152->hasBusConflicts() &&
        m152Effective == 0x2B && m152Banks && m152->mirroring() == Mirror::OnescreenLo &&
        mapperImplementationSupported(152,0) && !mapperImplementationSupported(152,1);
    std::printf("phase36_mapper152 conflict=%s banks=%s mirror=%s gate=%s\n",
        mapper152 ? "PASS" : "FAIL", m152Banks ? "PASS" : "FAIL",
        m152->mirroring() == Mirror::OnescreenLo ? "PASS" : "FAIL",
        (mapperImplementationSupported(152,0) && !mapperImplementationSupported(152,1)) ? "PASS" : "FAIL");
    ok &= mapper152;

    // Mapper 78 has two incompatible mirroring wirings. NES 2.0 submapper 1
    // (Uchuusen/Cosmo Carrier) uses one-screen A/B, while submapper 3 (Holy
    // Diver) uses horizontal/vertical. Legacy images commonly distinguish
    // Holy Diver with the iNES alternative-nametables flag. All variants are
    // discrete UNROM+CNROM hardware and therefore retain bus conflicts.
    auto m78s1 = makeMapper(78, 0x20000, 0x20000, 0, 0, 1, true);
    auto m78s3 = makeMapper(78, 0x20000, 0x20000, 0, 0, 3, true);
    auto m78LegacyCosmo = makeMapper(78, 0x20000, 0x20000);
    auto m78LegacyHoly = makeMapper(78, 0x20000, 0x20000, 0, 0, 0, false,
                                    Mirror::FourScreen, false, 0, true);
    m78s1->cpuWrite(0x8000, 0xA3, 0);
    const bool m78s1Low = m78s1->mirroring() == Mirror::OnescreenLo;
    m78s1->cpuWrite(0x8000, 0xAB, 0);
    const bool m78s1High = m78s1->mirroring() == Mirror::OnescreenHi;
    m78s3->cpuWrite(0x8000, 0xA3, 0);
    const bool m78s3H = m78s3->mirroring() == Mirror::Horizontal;
    m78s3->cpuWrite(0x8000, 0xAB, 0);
    const bool m78s3V = m78s3->mirroring() == Mirror::Vertical;
    m78LegacyCosmo->cpuWrite(0x8000, 0xAB, 0);
    m78LegacyHoly->cpuWrite(0x8000, 0xAB, 0);
    uint32_t m78Prg=0, m78Chr=0;
    const bool m78Banks = m78s3->cpuMapRead(0x8000, m78Prg) && m78s3->ppuMapRead(0x0000, m78Chr) &&
        m78Prg == 0xC000 && m78Chr == 0x14000;
    const bool m78Legacy = m78LegacyCosmo->mirroring() == Mirror::OnescreenHi &&
        m78LegacyHoly->mirroring() == Mirror::Vertical;
    const bool m78Gate = mapperImplementationSupported(78,0) && mapperImplementationSupported(78,1) &&
        mapperImplementationSupported(78,3) && !mapperImplementationSupported(78,2);
    const bool mapper78 = m78s1Low && m78s1High && m78s3H && m78s3V && m78Legacy && m78Banks &&
        m78s1->hasBusConflicts() && m78s3->hasBusConflicts() && m78Gate;
    std::printf("mapper78_variants=%s s1=%s s3=%s legacy=%s banks=%s gate=%s\n",
        mapper78 ? "PASS" : "FAIL", (m78s1Low&&m78s1High)?"PASS":"FAIL",
        (m78s3H&&m78s3V)?"PASS":"FAIL", m78Legacy?"PASS":"FAIL",
        m78Banks?"PASS":"FAIL", m78Gate?"PASS":"FAIL");
    ok &= mapper78;

    // Mapper 71 uses different PRG latch widths on its two NES 2.0 boards.
    // Normal BF9093 boards expose four bank bits; Fire Hawk's BF9097 exposes
    // only three. Legacy iNES starts in the normal-board compatibility mode
    // and switches to the Fire Hawk interpretation after a $9000-$9FFF write.
    auto m71s0 = makeMapper(71, 0x40000, 0, 0, 0x2000, 0, true, Mirror::Vertical);
    auto m71s1 = makeMapper(71, 0x40000, 0, 0, 0x2000, 1, true, Mirror::Vertical);
    auto m71legacy = makeMapper(71, 0x40000, 0, 0, 0x2000, 0, false, Mirror::Vertical);
    m71s0->cpuWrite(0xC000, 0x0F, 0);
    m71s1->cpuWrite(0xC000, 0x0F, 0);
    m71legacy->cpuWrite(0xC000, 0x0F, 0);
    uint32_t m71s0Prg=0,m71s1Prg=0,m71legacyBefore=0,m71legacyAfter=0,m71statePrg=0;
    m71s0->cpuMapRead(0x8000,m71s0Prg);
    m71s1->cpuMapRead(0x8000,m71s1Prg);
    m71legacy->cpuMapRead(0x8000,m71legacyBefore);
    m71legacy->cpuWrite(0x9000,0x10,0);
    m71legacy->cpuWrite(0xC000,0x0F,0);
    m71legacy->cpuMapRead(0x8000,m71legacyAfter);
    std::vector<uint8_t> m71State;
    m71legacy->saveState(m71State);
    auto m71restored = makeMapper(71, 0x40000, 0, 0, 0x2000, 0, false, Mirror::Vertical);
    const uint8_t* m71StatePtr=m71State.data();
    const bool m71StateLoad=m71restored->loadState(m71StatePtr,m71StatePtr+m71State.size());
    m71restored->cpuWrite(0xC000,0x0F,0);
    m71restored->cpuMapRead(0x8000,m71statePrg);
    const bool mapper71 = m71s0Prg==0x3C000 && m71s1Prg==0x1C000 &&
        m71legacyBefore==0x3C000 && m71legacyAfter==0x1C000 &&
        m71legacy->mirroring()==Mirror::OnescreenHi && m71StateLoad && m71statePrg==0x1C000 &&
        mapperImplementationSupported(71,0) && mapperImplementationSupported(71,1) &&
        !mapperImplementationSupported(71,2);
    std::printf("mapper71_bank_width=%s s0=%05X s1=%05X legacy=%05X->%05X state=%05X\n",
        mapper71?"PASS":"FAIL",unsigned(m71s0Prg),unsigned(m71s1Prg),
        unsigned(m71legacyBefore),unsigned(m71legacyAfter),unsigned(m71statePrg));
    ok &= mapper71;

    // Mapper 68 / Sunsoft-4. Normal boards can expose the full 4-bit PRG
    // register and always fix $C000-$FFFF to the final PRG bank. NES 2.0
    // submapper 1 is Nantettatte!! Baseball's Dual Cartridge System: bit 3
    // selects internal/external ROM, the fixed bank remains the last 16 KiB
    // of the 128 KiB internal ROM, and writes to $6000-$7FFF reset the
    // licensing timer only while WRAM is disabled and the external ROM is
    // selected.
    auto m68normal = makeMapper(68, 0x40000, 0x20000, 0x2000, 0, 0, true);
    m68normal->cpuWrite(0xF000, 0x0A, 0);
    uint32_t m68NormLo=0,m68NormHi=0;
    const bool m68NormRead = m68normal->cpuMapRead(0x8000,m68NormLo) &&
        m68normal->cpuMapRead(0xC000,m68NormHi);
    const bool m68Normal = m68NormRead && m68NormLo==0x28000 && m68NormHi==0x3C000;

    // Append a 16 KiB synthetic option ROM after the 128 KiB internal image
    // so the mapper probe can exercise the DCS socket without a second file.
    auto m68dual = makeMapper(68, 0x24000, 0x20000, 0x2000, 0, 1, true);
    m68dual->cpuWrite(0xF000, 0x00, 0); // external bank 0, WRAM disabled
    uint32_t m68Ext=0,m68Fixed=0,m68Internal=0,m68Ram=0;
    const bool m68ExpiredInitially = !m68dual->cpuMapRead(0x8000,m68Ext);
    m68dual->cpuWrite(0x6000, 0x00, 0); // reset licensing timer
    const bool m68ExtLive = m68dual->cpuMapRead(0x8000,m68Ext) && m68Ext==0x20000;
    const bool m68FixedInternal = m68dual->cpuMapRead(0xC000,m68Fixed) && m68Fixed==0x1C000;

    std::vector<uint8_t> m68State;
    for (unsigned i=0;i<100;++i) m68dual->clockCpu();
    m68dual->saveState(m68State);
    for (unsigned i=100;i<107520;++i) m68dual->clockCpu();
    const bool m68Expires = !m68dual->cpuMapRead(0x8000,m68Ext);
    const uint8_t* m68StatePtr=m68State.data();
    const bool m68StateLoad=m68dual->loadState(m68StatePtr,m68StatePtr+m68State.size());
    const bool m68StateRestoresTimer=m68StateLoad && m68dual->cpuMapRead(0x8000,m68Ext) && m68Ext==0x20000;

    m68dual->cpuWrite(0xF000, 0x0B, 0); // internal bank 3
    const bool m68InternalRead=m68dual->cpuMapRead(0x8000,m68Internal) && m68Internal==0x0C000;
    m68dual->cpuWrite(0xF000, 0x10, 0); // external selected, WRAM enabled
    m68dual->cpuWrite(0x6000, 0x00, 0); // must not arm licensing timer
    const bool m68Wram=m68dual->mapPrgRam(0x6000,m68Ram,true) && m68Ram==0 &&
        !m68dual->cpuMapRead(0x8000,m68Ext);
    const bool m68Gate=mapperImplementationSupported(68,0) && mapperImplementationSupported(68,1) &&
        !mapperImplementationSupported(68,2);
    const bool mapper68=m68Normal && m68ExpiredInitially && m68ExtLive && m68FixedInternal &&
        m68Expires && m68StateRestoresTimer && m68InternalRead && m68Wram && m68Gate;
    std::printf("mapper68_sunsoft4=%s normal=%s dcs=%s timer=%s state=%s gate=%s\n",
        mapper68?"PASS":"FAIL",m68Normal?"PASS":"FAIL",
        (m68ExpiredInitially&&m68ExtLive&&m68FixedInternal&&m68InternalRead&&m68Wram)?"PASS":"FAIL",
        m68Expires?"PASS":"FAIL",m68StateRestoresTimer?"PASS":"FAIL",m68Gate?"PASS":"FAIL");
    ok &= mapper68;

    // MMC3: $C001 requests a reload on the next filtered A12 rising edge.
    // A short low pulse must not clock the counter; a qualified low period
    // followed by A12 rising decrements 1->0 and asserts IRQ.
    auto mmc3 = makeMapper(4);
    mmc3->cpuWrite(0xC000, 0x01, 0);
    mmc3->cpuWrite(0xC001, 0x00, 0);
    mmc3->cpuWrite(0xE001, 0x00, 0);
    mmc3->notifyPpuAddress(0x0000, 0);
    mmc3->notifyPpuAddress(0x1000, 8);
    const bool mmc3ReloadNoIrq = !mmc3->irqActive();
    mmc3->notifyPpuAddress(0x0000, 9);
    mmc3->notifyPpuAddress(0x1000, 13);
    const bool mmc3ShortFiltered = !mmc3->irqActive();
    mmc3->notifyPpuAddress(0x0000, 14);
    mmc3->notifyPpuAddress(0x1000, 22);
    const bool mmc3QualifiedIrq = mmc3->irqActive();
    // Hardware-verified blargg MMC3 details: the counter keeps running while
    // IRQ output is disabled; $C001 only marks the *next* qualified edge for
    // reload; and revision-B MMC3s assert IRQ when a zero reload leaves the
    // counter at zero. Also pin the A12 low-pass threshold itself at 8 PPU
    // clocks: seven low clocks must be rejected, eight must qualify.
    auto mmc3Disabled = makeMapper(4);
    mmc3Disabled->cpuWrite(0xC000, 0x02, 0);
    mmc3Disabled->cpuWrite(0xC001, 0x00, 0);
    mmc3Disabled->cpuWrite(0xE000, 0x00, 0);
    mmc3Disabled->notifyPpuAddress(0x0000, 100);
    mmc3Disabled->notifyPpuAddress(0x1000, 108); // reload 2, IRQ disabled
    mmc3Disabled->notifyPpuAddress(0x0000, 109);
    mmc3Disabled->notifyPpuAddress(0x1000, 117); // 2 -> 1 while disabled
    mmc3Disabled->cpuWrite(0xE001, 0x00, 0);
    mmc3Disabled->notifyPpuAddress(0x0000, 118);
    mmc3Disabled->notifyPpuAddress(0x1000, 126); // 1 -> 0, now IRQ
    const bool mmc3RunsDisabled = mmc3Disabled->irqActive();

    auto mmc3Deferred = makeMapper(4);
    mmc3Deferred->cpuWrite(0xC000, 0x00, 0);
    mmc3Deferred->cpuWrite(0xE001, 0x00, 0);
    mmc3Deferred->cpuWrite(0xC001, 0x00, 0);
    const bool mmc3C001NoImmediateIrq = !mmc3Deferred->irqActive();
    mmc3Deferred->notifyPpuAddress(0x0000, 200);
    mmc3Deferred->notifyPpuAddress(0x1000, 208);
    const bool mmc3RevBZeroIrq = mmc3Deferred->irqActive();

    auto mmc3Threshold = makeMapper(4);
    mmc3Threshold->cpuWrite(0xC000, 0x00, 0);
    mmc3Threshold->cpuWrite(0xC001, 0x00, 0);
    mmc3Threshold->cpuWrite(0xE001, 0x00, 0);
    mmc3Threshold->notifyPpuAddress(0x0000, 300);
    mmc3Threshold->notifyPpuAddress(0x1000, 307); // 7 clocks: reject
    const bool mmc3SevenRejected = !mmc3Threshold->irqActive();
    mmc3Threshold->notifyPpuAddress(0x0000, 308);
    mmc3Threshold->notifyPpuAddress(0x1000, 316); // 8 clocks: qualify
    const bool mmc3EightAccepted = mmc3Threshold->irqActive();

    std::printf("mmc3_reload=%s short_filter=%s qualified_irq=%s disabled_run=%s c001_deferred=%s revB_zero=%s a12_7=%s a12_8=%s\n",
        mmc3ReloadNoIrq ? "PASS" : "FAIL",
        mmc3ShortFiltered ? "PASS" : "FAIL",
        mmc3QualifiedIrq ? "PASS" : "FAIL",
        mmc3RunsDisabled ? "PASS" : "FAIL",
        mmc3C001NoImmediateIrq ? "PASS" : "FAIL",
        mmc3RevBZeroIrq ? "PASS" : "FAIL",
        mmc3SevenRejected ? "PASS" : "FAIL",
        mmc3EightAccepted ? "PASS" : "FAIL");
    ok &= mmc3ReloadNoIrq && mmc3ShortFiltered && mmc3QualifiedIrq &&
        mmc3RunsDisabled && mmc3C001NoImmediateIrq && mmc3RevBZeroIrq &&
        mmc3SevenRejected && mmc3EightAccepted;

    // Mapper 12.0 (Gouder SL-5020B): Huang-1 operates as an MMC3A and
    // an external GAL at $4132 independently supplies CHR A18 for the two
    // PPU pattern-table halves. The GAL is outside MMC3 CHR inversion.
    auto m12 = makeMapper(12, 0x40000, 0x80000, 0, 0, 0, true);
    uint32_t m12LoA=0,m12HiA=0,m12LoB=0,m12HiB=0;
    m12->cpuWrite(0x4132, 0x01, 0);
    m12->ppuMapRead(0x0000, m12LoA); m12->ppuMapRead(0x1000, m12HiA);
    m12->cpuWrite(0x4132, 0x10, 0);
    m12->ppuMapRead(0x0000, m12LoB); m12->ppuMapRead(0x1000, m12HiB);
    uint8_t language = 0xFF;
    const bool m12Read = m12->cpuReadRegister(0x4132, language) && language == 0;
    m12->cpuWrite(0xC000, 0x00, 0);
    m12->cpuWrite(0xC001, 0x00, 0);
    m12->cpuWrite(0xE001, 0x00, 0);
    m12->notifyPpuAddress(0x0000, 100);
    m12->notifyPpuAddress(0x1000, 108);
    const bool m12Mmc3aZero = !m12->irqActive();
    const bool m12Chr = m12LoA == 0x40000 && m12HiA == 0x00000 &&
        m12LoB == 0x00000 && m12HiB == 0x40000;
    const bool m12Gate = mapperImplementationSupported(12,0) && mapperImplementationSupported(12,1);
    const bool mapper12 = m12Chr && m12Read && m12Mmc3aZero && m12Gate;
    std::printf("mapper12_sl5020b=%s chr=%05X/%05X->%05X/%05X mmc3a0=%s gate=%s\n",
        mapper12?"PASS":"FAIL", unsigned(m12LoA),unsigned(m12HiA),unsigned(m12LoB),unsigned(m12HiB),
        m12Mmc3aZero?"PASS":"FAIL",m12Gate?"PASS":"FAIL");
    ok &= mapper12;

    // Front Fareast RAM-cartridge extracts. Mapper 6 selects one of the
    // latch-based Magic Card modes, mapper 8 is mapper 6 mode 4, mapper 12.1
    // powers up in 4M mode, and mapper 17 adds 1 KiB CHR banking + IRQ.
    auto m6 = makeMapper(6, 0x20000, 0, 0x29000, 0x8000, 0, true);
    uint32_t m6Lo0=0,m6Hi0=0,m6Lo3=0;
    m6->mapPrgRam(0x8000,m6Lo0,false); m6->mapPrgRam(0xC000,m6Hi0,false);
    m6->cpuWrite(0x8000,0x03,0); m6->mapPrgRam(0x8000,m6Lo3,false);
    const bool mapper6 = m6Lo0==0x00000 && m6Hi0==0x1C000 && m6Lo3==0x0C000 && !m6->irqActive();

    auto m8 = makeMapper(8, 0x20000, 0, 0x29000, 0x8000, 0, false);
    uint32_t m8Prg=0,m8Chr=0;
    m8->cpuWrite(0x8000,0x21,0); m8->mapPrgRam(0x8000,m8Prg,false); m8->ppuMapRead(0x0000,m8Chr);
    const bool mapper8 = m8Prg==0x10000 && m8Chr==0x02000;

    auto m12smc = makeMapper(12,0x80000,0,0x89000,0x8000,1,true);
    uint32_t m12p=0; m12smc->mapPrgRam(0x8000,m12p,false);
    const bool mapper12smc = m12p==0x78000;

    auto m17 = makeMapper(17,0x80000,0,0x89000,0x40000,0,true);
    uint32_t m17Prg=0,m17Chr=0;
    m17->cpuWrite(0x4504,0x02,0); m17->mapPrgRam(0x8000,m17Prg,false);
    m17->cpuWrite(0x4510,0x05,0); m17->ppuMapRead(0x0000,m17Chr);
    m17->cpuWrite(0x4502,0xFE,0); m17->cpuWrite(0x4503,0xFF,0);
    m17->clockCpu(); const bool m17Before=!m17->irqActive(); m17->clockCpu(); const bool m17Irq=m17->irqActive();
    m17->cpuWrite(0x4501,0,0); const bool m17Ack=!m17->irqActive();
    const bool mapper17 = m17Prg==0x04000 && m17Chr==0x01400 && m17Before && m17Irq && m17Ack;
    const bool smcGate = mapperImplementationSupported(6,7) && mapperImplementationSupported(8,0) &&
        mapperImplementationSupported(8,4) && mapperImplementationSupported(12,1) &&
        mapperImplementationSupported(17,3) && !mapperImplementationSupported(17,4);
    std::printf("super_magic_card m6=%s m8=%s m12.1=%s m17=%s gate=%s\n",
        mapper6?"PASS":"FAIL",mapper8?"PASS":"FAIL",mapper12smc?"PASS":"FAIL",mapper17?"PASS":"FAIL",smcGate?"PASS":"FAIL");
    ok &= mapper6 && mapper8 && mapper12smc && mapper17 && smcGate;

    // Nintendo MMC3 multicarts. Mappers 37 and 47 replace WRAM with an outer
    // bank latch; mapper 49 adds an outer block register plus a 32 KiB PRG
    // mode. All retain the underlying MMC3 banking/IRQ engine.
    auto m37 = makeMapper(37, 0x40000, 0x40000, 0, 0);
    uint32_t m37Prg0=0,m37Prg3=0,m37Prg4=0,m37Chr4=0;
    m37->cpuMapRead(0xE000,m37Prg0);
    m37->cpuWrite(0x6000,0x03,0); m37->cpuMapRead(0xE000,m37Prg3);
    m37->cpuWrite(0x6000,0x04,0); m37->cpuMapRead(0xE000,m37Prg4); m37->ppuMapRead(0x0000,m37Chr4);
    const bool mapper37 = m37Prg0 == 0x0E000 && m37Prg3 == 0x1E000 &&
        m37Prg4 == 0x3E000 && m37Chr4 == 0x20000;

    auto m47 = makeMapper(47, 0x40000, 0x40000, 0, 0);
    uint32_t m47Prg0=0,m47Prg1=0,m47Chr1=0;
    m47->cpuMapRead(0xE000,m47Prg0);
    m47->cpuWrite(0x6000,1,0); m47->cpuMapRead(0xE000,m47Prg1); m47->ppuMapRead(0x0000,m47Chr1);
    const bool mapper47 = m47Prg0 == 0x1E000 && m47Prg1 == 0x3E000 && m47Chr1 == 0x20000;

    auto m49 = makeMapper(49, 0x80000, 0x80000, 0, 0);
    uint32_t m49Mode32Lo=0,m49Mode32Hi=0,m49Blocked=0,m49Mmc3=0,m49Chr=0;
    m49->cpuWrite(0x6000,0x20,0); // P=2, 32 KiB mode
    m49->cpuMapRead(0x8000,m49Mode32Lo); m49->cpuMapRead(0xE000,m49Mode32Hi);
    m49->cpuWrite(0xA001,0x00,0); // disable outer-register writes
    m49->cpuWrite(0x6000,0xC1,0); m49->cpuMapRead(0x8000,m49Blocked);
    m49->cpuWrite(0xA001,0x80,0); // enable + writable
    m49->cpuWrite(0x6000,0xC1,0); m49->cpuMapRead(0xE000,m49Mmc3); m49->ppuMapRead(0x0000,m49Chr);
    const bool mapper49 = m49Mode32Lo == 0x10000 && m49Mode32Hi == 0x16000 &&
        m49Blocked == 0x10000 && m49Mmc3 == 0x7E000 && m49Chr == 0x60000;
    std::printf("mmc3_multicarts m37=%s m47=%s m49=%s\n",
        mapper37 ? "PASS" : "FAIL", mapper47 ? "PASS" : "FAIL", mapper49 ? "PASS" : "FAIL");
    ok &= mapper37 && mapper47 && mapper49;

    // Mapper 114/115: MMC3 clones with NROM override and outer CHR banking.
    // Mapper 114 additionally scrambles MMC3 register addresses and bank indices.
    auto m114 = makeMapper(114, 0x80000, 0x80000, 0, 0, 0, true, Mirror::Vertical);
    m114->cpuWrite(0xA000, 0x04, 0); // sub0: real $8000, index 4 -> MMC3 reg 6
    m114->cpuWrite(0xC000, 0x03, 0); // sub0: real $8001
    uint32_t m114Bank=0,m114NromLo=0,m114NromHi=0,m114Chr=0;
    m114->cpuMapRead(0x8000,m114Bank);
    m114->cpuWrite(0x8001, 0x01, 0); // sub0: real $A000 mirroring
    const bool m114Mirror = m114->mirroring() == Mirror::Horizontal;
    m114->cpuWrite(0x6000, 0x83, 0); // NROM-128, bank 3
    m114->cpuMapRead(0x8000,m114NromLo); m114->cpuMapRead(0xC000,m114NromHi);
    m114->cpuWrite(0x6001, 0x01, 0); m114->ppuMapRead(0x0000,m114Chr);
    m114->cpuWrite(0xE001, 0, 0); m114->scanlineTick();
    const bool m114IrqZero = !m114->irqActive();
    const bool mapper114 = m114Bank == 0x06000 && m114Mirror &&
        m114NromLo == 0x0C000 && m114NromHi == 0x0C000 && m114Chr == 0x40000 && m114IrqZero;

    auto m114s1 = makeMapper(114, 0x80000, 0x80000, 0, 0, 1, true, Mirror::Vertical);
    m114s1->cpuWrite(0x8001, 0x01, 0); // sub1: real $8001, not mirroring
    const bool mapper114Sub1 = m114s1->mirroring() == Mirror::Vertical;

    auto m115 = makeMapper(115, 0x80000, 0x80000, 0, 0, 0, true);
    uint32_t m115Low=0,m115Outer=0,m115Chr=0;
    m115->cpuMapRead(0xE000,m115Low);
    m115->cpuWrite(0x6000, 0x40, 0); // normal MMC3 + PRG A18
    m115->cpuMapRead(0xE000,m115Outer);
    m115->cpuWrite(0x6001, 1, 0); m115->ppuMapRead(0x0000,m115Chr);
    m115->cpuWrite(0xE001, 0, 0); m115->scanlineTick();
    const bool mapper115 = m115Low == 0x3E000 && m115Outer == 0x7E000 &&
        m115Chr == 0x40000 && m115->irqActive();
    std::printf("scrambled_mmc3 m114=%s m114s1=%s m115=%s\n",
        mapper114?"PASS":"FAIL", mapper114Sub1?"PASS":"FAIL", mapper115?"PASS":"FAIL");
    ok &= mapper114 && mapper114Sub1 && mapper115;

    // Mapper 197 changes only MMC3 CHR wiring. Submapper 0 exposes one 4 KiB
    // bank plus two 2 KiB banks; submapper 3 also has a WRAM-gated PRG A17 latch.
    auto m197 = makeMapper(197, 0x40000, 0x80000, 0, 0, 0, true);
    m197->cpuWrite(0x8000, 0x00, 0); m197->cpuWrite(0x8001, 0x02, 0);
    uint32_t m197c0=0,m197c3=0;
    m197->ppuMapRead(0x0000,m197c0); m197->ppuMapRead(0x0C00,m197c3);
    const bool mapper197Chr = m197c0 == 0x01000 && m197c3 == 0x01C00;

    auto m197s3 = makeMapper(197, 0x40000, 0x80000, 0, 0, 3, true);
    m197s3->cpuWrite(0x6000, 0x09, 0); // S=1, PRG A17=1
    uint32_t m197Prg=0; m197s3->cpuMapRead(0xE000,m197Prg);
    const bool mapper197Outer = m197Prg == 0x3E000;
    std::printf("mapper197_chr=%s outer=%s\n", mapper197Chr?"PASS":"FAIL", mapper197Outer?"PASS":"FAIL");
    ok &= mapper197Chr && mapper197Outer;

    // MMC6 (NES 2.0 mapper 4 submapper 1): 1 KiB internal RAM is mirrored
    // through $7000-$7FFF and split into independently readable/writable 512B
    // halves. $8000 bit 5 is the global RAM enable.
    auto mmc6 = makeMapper(4, 0x40000, 0x20000, 0, 0, 1, true);
    mmc6->cpuWrite(0x8000, 0x20, 0); // global RAM enable
    mmc6->cpuWrite(0xA001, 0x30, 0); // low half read+write only
    mmc6->cpuWrite(0x7000, 0xA5, 0);
    uint8_t mmc6Low=0, mmc6High=0, mmc6Mirror=0;
    const bool mmc6LowDriven = mmc6->cpuReadRegister(0x7000, mmc6Low);
    const bool mmc6HighDriven = mmc6->cpuReadRegister(0x7200, mmc6High);
    const bool mmc6MirrorDriven = mmc6->cpuReadRegister(0x7400, mmc6Mirror);
    mmc6->cpuWrite(0xA001, 0x20, 0); // low read only
    mmc6->cpuWrite(0x7000, 0x5A, 0);
    uint8_t mmc6Protected=0;
    mmc6->cpuReadRegister(0x7000, mmc6Protected);
    mmc6->cpuWrite(0x8000, 0x00, 0); // global disable also clears protections
    uint8_t mmc6Open=0;
    const bool mmc6DisabledDriven = mmc6->cpuReadRegister(0x7000, mmc6Open);
    const bool mmc6Ram = mmc6LowDriven && mmc6Low == 0xA5 && mmc6HighDriven && mmc6High == 0x00 &&
        mmc6MirrorDriven && mmc6Mirror == 0xA5 && mmc6Protected == 0xA5 && !mmc6DisabledDriven;
    std::printf("mmc6_ram_protection=%s low=%02X high=%02X mirror=%02X\n",
        mmc6Ram ? "PASS" : "FAIL", mmc6Low, mmc6High, mmc6Mirror);
    ok &= mmc6Ram;

    // MMC5 Phase 48: scanline/frame state must be inferred from the PPU bus,
    // not injected from the emulator's scanline counter. Two matching NT
    // reads are insufficient; three arm synchronization and the following PPU
    // read detects the scanline. Three CPU clocks without a PPU read leave the
    // frame and acknowledge pending IRQ state.
    auto mmc5 = makeMapper(5);
    auto mmc5Scanline = [&](uint16_t nt) {
        mmc5->notifyPpuAddressContext(nt, 0, 0, 337);
        mmc5->notifyPpuAddressContext(nt, 0, 0, 339);
        mmc5->notifyPpuAddressContext(nt, 0, 0, 1);
        mmc5->notifyPpuAddressContext(uint16_t((nt & 0x2C00) | 0x03C0), 0, 0, 3);
    };
    mmc5->cpuWrite(0x5204, 0x80, 0);
    mmc5->notifyPpuAddressContext(0x2000, 0, 0, 337);
    mmc5->notifyPpuAddressContext(0x2000, 0, 0, 339);
    uint8_t mmc5Status=0;
    mmc5->cpuReadRegister(0x5204, mmc5Status);
    const bool mmc5TwoReadsNoFrame = (mmc5Status & 0x40) == 0;
    mmc5->notifyPpuAddressContext(0x2000, 0, 0, 1);
    mmc5->cpuReadRegister(0x5204, mmc5Status);
    const bool mmc5ThirdArmsOnly = (mmc5Status & 0x40) == 0;
    mmc5->notifyPpuAddressContext(0x23C0, 0, 0, 3);
    mmc5->cpuReadRegister(0x5204, mmc5Status);
    const bool mmc5FourthStartsFrame = (mmc5Status & 0x40) != 0;

    mmc5->cpuWrite(0x5203, 0x00, 0);
    mmc5Scanline(0x2001);
    const bool mmc5ZeroNoIrq = !mmc5->irqActive();
    mmc5->cpuWrite(0x5203, 0x03, 0);
    mmc5Scanline(0x2002); // counter 2 after the earlier zero-compare line
    mmc5Scanline(0x2003); // counter 3 -> pending
    const bool mmc5IrqRaised = mmc5->irqActive();
    mmc5->clockCpu(); // consumes the PPU-read-seen latch
    mmc5->clockCpu(); mmc5->clockCpu(); mmc5->clockCpu(); // three idle M2 rises
    const bool mmc5FrameAck = !mmc5->irqActive();
    mmc5->cpuReadRegister(0x5204, mmc5Status);
    const bool mmc5IdleLeavesFrame = (mmc5Status & 0x40) == 0;
    std::printf("mmc5_bus_sync=%s third_arm=%s fourth_frame=%s irq_zero=%s raised=%s idle_ack=%s\n",
        mmc5TwoReadsNoFrame ? "PASS" : "FAIL",
        mmc5ThirdArmsOnly ? "PASS" : "FAIL",
        mmc5FourthStartsFrame ? "PASS" : "FAIL",
        mmc5ZeroNoIrq ? "PASS" : "FAIL",
        mmc5IrqRaised ? "PASS" : "FAIL",
        (mmc5FrameAck && mmc5IdleLeavesFrame) ? "PASS" : "FAIL");
    ok &= mmc5TwoReadsNoFrame && mmc5ThirdArmsOnly && mmc5FourthStartsFrame &&
        mmc5ZeroNoIrq && mmc5IrqRaised && mmc5FrameAck && mmc5IdleLeavesFrame;

    // MMC5 Phase 49: vertical split position is based on the 34 nametable
    // tile fetches, not on PPU dot/8. Two prefetched tiles mean scanline dot 1
    // corresponds to split column 2. With a left split threshold of 3, dot 1
    // must be inside the split and dot 9 (column 3) must be outside it.
    auto mmc5Split = makeMapper(5);
    mmc5Split->cpuWrite(0x5104, 0x00, 0);
    mmc5Split->cpuWrite(0x5200, 0x83, 0); // enable, left side, threshold 3
    mmc5Split->cpuWrite(0x5201, 0x00, 0);
    mmc5Split->cpuWrite(0x5202, 0x02, 0);
    auto syncMmc5Line = [&](uint16_t nt, int line) {
        mmc5Split->notifyPpuAddressContext(nt, 0, line - 1, 337);
        mmc5Split->notifyPpuAddressContext(nt, 0, line - 1, 339);
        mmc5Split->notifyPpuAddressContext(nt, 0, line, 1);
        mmc5Split->notifyPpuAddressContext(uint16_t((nt & 0x2C00) | 0x03C0), 0, line, 3);
    };
    syncMmc5Line(0x2000, 0); // enter frame
    mmc5Split->notifyPpuAddressContext(0x2000, 0, 0, 337);
    mmc5Split->notifyPpuAddressContext(0x2000, 0, 0, 339);
    mmc5Split->notifyPpuAddressContext(0x2000, 0, 1, 1); // column 2
    NametableSource splitSrc = NametableSource::Ciram;
    uint32_t splitMapped = 0;
    const bool splitDot1Mapped = mmc5Split->mapNametable(0x2000, splitSrc, splitMapped) &&
        splitSrc == NametableSource::MapperRam && splitMapped == 2;
    mmc5Split->notifyPpuAddressContext(0x23C0, 0, 1, 3);
    mmc5Split->notifyPpuAddressContext(0x2001, 0, 1, 9);
    splitSrc = NametableSource::MapperRam; splitMapped = 0xFFFFFFFFu;
    const bool splitDot9Normal = mmc5Split->mapNametable(0x2001, splitSrc, splitMapped) &&
        splitSrc == NametableSource::Ciram;
    const bool mmc5SplitFetchCounter = splitDot1Mapped && splitDot9Normal;
    std::printf("mmc5_split_fetch_counter=%s dot1=%s dot9=%s\n",
        mmc5SplitFetchCounter ? "PASS" : "FAIL",
        splitDot1Mapped ? "PASS" : "FAIL", splitDot9Normal ? "PASS" : "FAIL");
    ok &= mmc5SplitFetchCounter;


    // NES 2.0 submapper distinctions that represent genuinely different
    // boards must not be collapsed into the generic mapper behavior.
    auto g101Major = makeMapper(32, 0x20000, 0x20000, 0, 0, 1, true);
    g101Major->cpuWrite(0x8000, 1, 0);
    g101Major->cpuWrite(0x9000, 3, 0); // must be ignored on Major League board
    uint32_t g101Mapped = 0;
    g101Major->cpuMapRead(0x8000, g101Mapped);
    const bool g101MajorLeague = g101Mapped == 0x2000 && g101Major->mirroring() == Mirror::OnescreenHi;

    auto camHard = makeMapper(71, 0x20000, 0, 0, 0, 0, true, Mirror::Horizontal);
    camHard->cpuWrite(0x9000, 0x10, 0);
    auto camFire = makeMapper(71, 0x20000, 0, 0, 0, 1, true, Mirror::Horizontal);
    camFire->cpuWrite(0x9000, 0x10, 0);
    const bool camericaSubmaps = camHard->mirroring() == Mirror::Horizontal &&
        camFire->mirroring() == Mirror::OnescreenHi;

    auto cosmo = makeMapper(78, 0x20000, 0x20000, 0, 0, 1, true);
    cosmo->cpuWrite(0x8000, 0x08, 0);
    auto holy = makeMapper(78, 0x20000, 0x20000, 0, 0, 3, true);
    holy->cpuWrite(0x8000, 0x08, 0);
    const bool mapper78Submaps = cosmo->mirroring() == Mirror::OnescreenHi &&
        holy->mirroring() == Mirror::Vertical;

    auto vrc7b = makeMapper(85, 0x40000, 0x20000, 0x2000, 0, 1, true);
    // VRC7b uses CPU A3 as ASIC An. A4 is not a reject condition: it aliases
    // the same register, so $8010 -> $8000 and $8018 -> $8008.
    const bool vrc7bAliasA4Low = vrc7b->cpuWrite(0x8010, 3, 0);
    uint32_t vrc7bLowMapped = 0; vrc7b->cpuMapRead(0x8000, vrc7bLowMapped);
    const bool vrc7bAliasA4High = vrc7b->cpuWrite(0x8018, 5, 0);
    uint32_t vrc7bHighMapped = 0; vrc7b->cpuMapRead(0xA000, vrc7bHighMapped);

    auto vrc7a = makeMapper(85, 0x40000, 0x20000, 0x2000, 0, 2, true);
    // VRC7a uses CPU A4 as ASIC An. A3 is ignored, so $8008 -> $8000
    // while $8010 selects the canonical second PRG register.
    const bool vrc7aAliasA3Low = vrc7a->cpuWrite(0x8008, 3, 0);
    uint32_t vrc7aLowMapped = 0; vrc7a->cpuMapRead(0x8000, vrc7aLowMapped);
    const bool vrc7aSelectA4High = vrc7a->cpuWrite(0x8010, 5, 0);
    uint32_t vrc7aHighMapped = 0; vrc7a->cpuMapRead(0xA000, vrc7aHighMapped);
    const bool vrc7Submaps = vrc7bAliasA4Low && vrc7bLowMapped == 0x6000 &&
        vrc7bAliasA4High && vrc7bHighMapped == 0xA000 &&
        vrc7aAliasA3Low && vrc7aLowMapped == 0x6000 &&
        vrc7aSelectA4High && vrc7aHighMapped == 0xA000;

    std::printf("submappers_g101=%s camerica=%s m78=%s vrc7=%s\n",
        g101MajorLeague ? "PASS" : "FAIL", camericaSubmaps ? "PASS" : "FAIL",
        mapper78Submaps ? "PASS" : "FAIL", vrc7Submaps ? "PASS" : "FAIL");
    ok &= g101MajorLeague && camericaSubmaps && mapper78Submaps && vrc7Submaps;

    // VRC7 $E000 bit 6 is an audio reset, not a temporary mixer mute. A note
    // must not resume after reset is released, and register writes made while
    // reset is asserted must be ignored.
    auto vrc7Audio = makeMapper(85, 0x40000, 0x20000, 0x2000, 0, 0, false);
    auto vrc7WriteReg = [&](uint8_t reg, uint8_t value) {
        vrc7Audio->cpuWrite(0x9010, reg, 0);
        vrc7Audio->cpuWrite(0x9030, value, 0);
    };
    auto vrc7Peak = [&](int clocks) {
        float peak = 0.0f;
        for (int i = 0; i < clocks; ++i) {
            vrc7Audio->clockCpu();
            peak = std::max(peak, std::fabs(vrc7Audio->expansionAudioSample()));
        }
        return peak;
    };
    vrc7WriteReg(0x10, 0xFF);
    vrc7WriteReg(0x20, 0x19); // fnum high=1, octave=4, key on
    vrc7WriteReg(0x30, 0x10); // preset 1, maximum volume
    const bool vrc7SoundBeforeReset = vrc7Peak(20000) > 0.0001f;
    vrc7Audio->cpuWrite(0xE000, 0x40, 0);
    const bool vrc7MutedInReset = std::fabs(vrc7Audio->expansionAudioSample()) < 0.000001f;
    vrc7WriteReg(0x10, 0xFF); // all ignored while reset is asserted
    vrc7WriteReg(0x20, 0x19);
    vrc7WriteReg(0x30, 0x10);
    vrc7Audio->cpuWrite(0xE000, 0x00, 0);
    const bool vrc7StaysSilentAfterReset = vrc7Peak(20000) < 0.000001f;
    vrc7WriteReg(0x10, 0xFF);
    vrc7WriteReg(0x20, 0x19);
    vrc7WriteReg(0x30, 0x10);
    const bool vrc7CanRestart = vrc7Peak(20000) > 0.0001f;
    const bool vrc7AudioReset = vrc7SoundBeforeReset && vrc7MutedInReset &&
        vrc7StaysSilentAfterReset && vrc7CanRestart;
    std::printf("vrc7_audio_reset=%s before=%s held=%s release=%s restart=%s\n",
        vrc7AudioReset ? "PASS" : "FAIL",
        vrc7SoundBeforeReset ? "PASS" : "FAIL", vrc7MutedInReset ? "PASS" : "FAIL",
        vrc7StaysSilentAfterReset ? "PASS" : "FAIL", vrc7CanRestart ? "PASS" : "FAIL");
    ok &= vrc7AudioReset;


    // UNROM-512: NES 2.0 submappers define conflicts and mirroring/address
    // decode. Battery-backed variants expose the SST39SF040 flash at
    // $8000-$BFFF while the bank latch remains at $C000-$FFFF.
    auto u512Conflict = makeMapper(30, 0x80000, 0, 0, 0x8000, 2, true);
    auto u512NoConflict = makeMapper(30, 0x80000, 0, 0, 0x8000, 1, true);
    const bool u512ConflictModes = u512Conflict->hasBusConflicts() && !u512NoConflict->hasBusConflicts();
    auto u512Hv = makeMapper(30, 0x80000, 0, 0, 0x8000, 3, true, Mirror::Horizontal);
    u512Hv->cpuWrite(0xC000, 0x00, 0);
    const bool u512Vertical = u512Hv->mirroring() == Mirror::Vertical;
    u512Hv->cpuWrite(0xC000, 0x80, 0);
    const bool u512Horizontal = u512Hv->mirroring() == Mirror::Horizontal;
    auto u512Led = makeMapper(30, 0x80000, 0, 0, 0x8000, 4, true);
    u512Led->cpuWrite(0xC000, 0x02, 0);
    uint32_t u512Before=0,u512After=0;
    u512Led->cpuMapRead(0x8000,u512Before);
    u512Led->cpuWrite(0x8000,0x1F,0); // LED register, must not alter bank
    u512Led->cpuMapRead(0x8000,u512After);

    auto u512Flash = makeMapper(30, 0x80000, 0, 0, 0x8000, 1, true, Mirror::Horizontal, true);
    std::vector<uint8_t> u512Image(0x80000, 0xFF);
    u512Flash->initializePrgImage(u512Image.data(), u512Image.size());
    auto u512Select = [&](uint8_t bank) { u512Flash->cpuWrite(0xC000, bank, 0); };
    auto u512Unlock = [&](uint8_t command) {
        u512Select(1); u512Flash->cpuWrite(0x9555, 0xAA, 0);
        u512Select(0); u512Flash->cpuWrite(0xAAAA, 0x55, 0);
        u512Select(1); u512Flash->cpuWrite(0x9555, command, 0);
    };
    auto u512Read = [&](uint8_t bank, uint16_t addr) {
        uint8_t value = 0;
        u512Select(bank);
        const bool drove = u512Flash->cpuReadRegister(addr, value);
        return std::pair<bool,uint8_t>{drove, value};
    };

    // Byte program: AA@5555, 55@2AAA, A0@5555, then target write.
    u512Unlock(0xA0);
    u512Select(5); u512Flash->cpuWrite(0x8123, 0x5A, 0);
    const auto u512Programmed = u512Read(5, 0x8123);
    const bool u512Program = u512Programmed.first && u512Programmed.second == 0x5A;

    // NOR programming may only clear bits without an erase.
    u512Unlock(0xA0);
    u512Select(5); u512Flash->cpuWrite(0x8123, 0xF0, 0);
    const auto u512Anded = u512Read(5, 0x8123);
    const bool u512NorAnd = u512Anded.first && u512Anded.second == (0x5A & 0xF0);

    // 4 KiB sector erase: unlock 80, unlock again, then 30 in target sector.
    u512Unlock(0x80);
    u512Select(1); u512Flash->cpuWrite(0x9555, 0xAA, 0);
    u512Select(0); u512Flash->cpuWrite(0xAAAA, 0x55, 0);
    u512Select(5); u512Flash->cpuWrite(0x8000, 0x30, 0);
    const auto u512Erased = u512Read(5, 0x8123);
    const bool u512SectorErase = u512Erased.first && u512Erased.second == 0xFF;

    // Reprogram, then verify mapper-owned flash survives battery serialization.
    u512Unlock(0xA0);
    u512Select(5); u512Flash->cpuWrite(0x8123, 0xA5, 0);
    std::vector<uint8_t> u512Battery;
    u512Flash->saveMapperBattery(u512Battery);
    auto u512Reloaded = makeMapper(30, 0x80000, 0, 0, 0x8000, 1, true, Mirror::Horizontal, true);
    u512Reloaded->initializePrgImage(u512Image.data(), u512Image.size());
    const bool u512BatteryLoad = u512Battery.size() == 0x80000 &&
        u512Reloaded->loadMapperBattery(u512Battery.data(), u512Battery.size());
    u512Reloaded->cpuWrite(0xC000, 5, 0);
    uint8_t u512PersistedValue = 0;
    const bool u512BatteryRead = u512Reloaded->cpuReadRegister(0x8123, u512PersistedValue) &&
        u512PersistedValue == 0xA5;

    // Save states include the mutable flash image, not merely the latch.
    std::vector<uint8_t> u512State;
    u512Flash->saveState(u512State);
    u512Unlock(0x80);
    u512Select(1); u512Flash->cpuWrite(0x9555, 0xAA, 0);
    u512Select(0); u512Flash->cpuWrite(0xAAAA, 0x55, 0);
    u512Select(5); u512Flash->cpuWrite(0x8000, 0x30, 0);
    const uint8_t* u512StatePtr = u512State.data();
    const bool u512StateLoad = u512Flash->loadState(u512StatePtr, u512State.data() + u512State.size()) &&
        u512StatePtr == u512State.data() + u512State.size();
    const auto u512StateRestored = u512Read(5, 0x8123);
    const bool u512StatePersistence = u512StateLoad && u512StateRestored.first && u512StateRestored.second == 0xA5;

    const bool u512SupportTruth = u512NoConflict->implementationSupported() && u512Flash->implementationSupported();
    const bool u512 = u512ConflictModes && u512Vertical && u512Horizontal && u512Before == u512After &&
        u512SupportTruth && u512Program && u512NorAnd && u512SectorErase &&
        u512BatteryLoad && u512BatteryRead && u512StatePersistence;
    std::printf("unrom512_submappers=%s conflict=%s hv=%s led_decode=%s flash_claim=%s program=%s erase=%s battery=%s state=%s\n",
        u512 ? "PASS" : "FAIL", u512ConflictModes ? "PASS" : "FAIL",
        (u512Vertical&&u512Horizontal)?"PASS":"FAIL", (u512Before==u512After)?"PASS":"FAIL",
        u512SupportTruth?"PASS":"FAIL", (u512Program&&u512NorAnd)?"PASS":"FAIL",
        u512SectorErase?"PASS":"FAIL", (u512BatteryLoad&&u512BatteryRead)?"PASS":"FAIL",
        u512StatePersistence?"PASS":"FAIL");
    ok &= u512;

    // Mapper 15: exercise all four PRG banking modes and mirroring latch.
    auto m15 = makeMapper(15, 0x100000, 0, 0, 0x2000);
    uint32_t m15a=0,m15b=0,m15c=0,m15d=0;
    m15->cpuWrite(0x8000, 0x42, 0); // mode 0, P=2, H mirror
    m15->cpuMapRead(0x8000,m15a); m15->cpuMapRead(0xC000,m15b);
    m15->cpuWrite(0x8002, 0x82, 0); // mode 2, P=2, p=1
    m15->cpuMapRead(0x8000,m15c); m15->cpuMapRead(0xE000,m15d);
    const bool mapper15 = m15a == 0x8000 && m15b == 0xC000 && m15c == 0xA000 && m15d == 0xA000 &&
        m15->mirroring() == Mirror::Vertical; // second write clears D6
    std::printf("mapper15_modes=%s m0=%05X/%05X m2=%05X/%05X\n", mapper15?"PASS":"FAIL",
        unsigned(m15a),unsigned(m15b),unsigned(m15c),unsigned(m15d));
    ok &= mapper15;

    // Mapper 31: eight independent 4 KiB slots at $8000-$FFFF, selected by
    // writes to $5FF8-$5FFF; the final slot powers up to bank $FF.
    auto m31 = makeMapper(31, 0x100000, 0, 0, 0x2000);
    uint32_t m31Last=0,m31Slot=0;
    m31->cpuMapRead(0xF000,m31Last);
    m31->cpuWrite(0x5FFA, 0x12, 0);
    m31->cpuMapRead(0xA000,m31Slot);
    const bool mapper31 = m31Last == 0xFF000 && m31Slot == 0x12000;
    std::printf("mapper31_banking=%s last=%05X slot2=%05X\n", mapper31?"PASS":"FAIL",
        unsigned(m31Last),unsigned(m31Slot));
    ok &= mapper31;

    // Mapper 116 (Huang-1/Huang-2): three independent mapper personalities
    // selected by $4100. State must survive switching, VRC2 CHR registers
    // power up to $FF, and supervisor bit 2 supplies CHR A18.
    auto m116 = makeMapper(116, 0x40000, 0x80000, 0x2000, 0, 0, true);
    m116->cpuWrite(0x8000, 0x06, 0); m116->cpuWrite(0x8001, 0x03, 0); // MMC3 R6=3
    uint32_t m116Mmc3Before=0,m116Vrc=0,m116VrcChr=0,m116Mmc3After=0,m116OuterChr=0;
    m116->cpuMapRead(0x8000,m116Mmc3Before);
    m116->cpuWrite(0x4100,0x00,10); // VRC2
    m116->ppuMapRead(0x0000,m116VrcChr); // power-on CHR bank $FF
    m116->cpuWrite(0x8000,0x05,11); m116->cpuMapRead(0x8000,m116Vrc);
    m116->cpuWrite(0x4100,0x01,12); // MMC3 state must reappear
    m116->cpuMapRead(0x8000,m116Mmc3After);
    m116->cpuWrite(0x4100,0x05,13); // MMC3 + CHR A18
    m116->ppuMapRead(0x0000,m116OuterChr);
    const bool mapper116Modes = m116Mmc3Before == 0x06000 && m116Vrc == 0x0A000 &&
        m116VrcChr == 0x3FC00 && m116Mmc3After == 0x06000 && m116OuterChr == 0x40000;

    // Huang-1 -W (submapper 0) resets MMC1's serial latch when entering MMC1.
    // Start a partial serial transaction, leave MMC1, return, then write a full
    // PRG register. It must decode from a clean latch.
    auto m116w = makeMapper(116, 0x40000, 0x20000, 0x2000, 0, 0, true);
    m116w->cpuWrite(0x4100,0x02,20);
    m116w->cpuWrite(0xE000,1,22); m116w->cpuWrite(0xE000,1,24); // partial
    m116w->cpuWrite(0x4100,0x01,26); m116w->cpuWrite(0x4100,0x02,28);
    mmc1Serial(*m116w,0xE000,2,30);
    uint32_t m116wPrg=0; m116w->cpuMapRead(0x8000,m116wPrg);
    const bool mapper116WReset = m116wPrg == 0x08000;

    // Huang-2 changes the MMC1 PRG output wiring so register bank 2 selects
    // what corresponds to bank 4 on a normal 16 KiB MMC1 board.
    auto m116h2 = makeMapper(116, 0x40000, 0x20000, 0x2000, 0, 2, true);
    m116h2->cpuWrite(0x4100,0x02,40);
    mmc1Serial(*m116h2,0xE000,2,42);
    uint32_t m116h2Prg=0; m116h2->cpuMapRead(0x8000,m116h2Prg);
    const bool mapper116H2 = m116h2Prg == 0x10000;
    std::printf("mapper116_modes=%s w_reset=%s huang2=%s mmc3=%05X vrc=%05X chr=%05X\n",
        mapper116Modes?"PASS":"FAIL", mapper116WReset?"PASS":"FAIL", mapper116H2?"PASS":"FAIL",
        unsigned(m116Mmc3Before), unsigned(m116Vrc), unsigned(m116VrcChr));
    ok &= mapper116Modes && mapper116WReset && mapper116H2;

    // Mapper 116 submapper 3: Mario 5-in-1 uses the Reset button as an
    // outer selector. Game 0 occupies 256 KiB; games 1-4 occupy consecutive
    // 128 KiB windows. Internal mapper state remains live while Reset changes
    // only the outer ROM window, and a hard reset returns to game 0.
    auto m116multi = makeMapper(116, 0xC0000, 0xC0000, 0x2000, 0, 3, true);
    m116multi->cpuWrite(0x4100, 0x00, 100); // VRC2 mode
    m116multi->cpuWrite(0x8000, 0x01, 102); // deterministic 8 KiB inner bank
    uint32_t resetBanks[6] = {};
    m116multi->cpuMapRead(0x8000, resetBanks[0]);
    for (int i = 1; i < 6; ++i) {
        m116multi->reset(false);
        m116multi->cpuMapRead(0x8000, resetBanks[i]);
    }
    m116multi->reset(false); // game 1
    m116multi->reset(false); // game 2
    std::vector<uint8_t> m116State;
    m116multi->saveState(m116State);
    m116multi->reset(false); // game 3
    const uint8_t* m116StateP = m116State.data();
    const bool m116StateLoaded = m116multi->loadState(m116StateP, m116StateP + m116State.size()) &&
        m116StateP == m116State.data() + m116State.size();
    uint32_t restoredResetBank = 0;
    m116multi->cpuMapRead(0x8000, restoredResetBank);
    m116multi->reset(true);
    uint32_t hardResetBank = 0;
    m116multi->cpuMapRead(0x8000, hardResetBank);
    const bool mapper116Reset = resetBanks[0] == 0x02000 && resetBanks[1] == 0x42000 &&
        resetBanks[2] == 0x62000 && resetBanks[3] == 0x82000 && resetBanks[4] == 0xA2000 &&
        resetBanks[5] == 0x02000 && m116StateLoaded && restoredResetBank == 0x62000 &&
        hardResetBank == 0x02000;
    std::printf("mapper116_reset5in1=%s seq=%05X/%05X/%05X/%05X/%05X wrap=%05X state=%05X hard=%05X\n",
        mapper116Reset?"PASS":"FAIL", unsigned(resetBanks[0]), unsigned(resetBanks[1]),
        unsigned(resetBanks[2]), unsigned(resetBanks[3]), unsigned(resetBanks[4]), unsigned(resetBanks[5]),
        unsigned(restoredResetBank), unsigned(hardResetBank));
    ok &= mapper116Reset;


    // Mapper 45 / GA23C: four sequential outer registers mask and OR
    // the underlying MMC3 PRG/CHR bank numbers. Register #3 locks further
    // outer writes; any $6001-decode write clears the outer state and unlocks.
    auto m45 = makeMapper(45, 0x800000, 0x400000, 0x2000, 0);
    m45->cpuWrite(0x8000, 0x06, 0); m45->cpuWrite(0x8001, 0x2A, 0); // R6
    m45->cpuWrite(0x8000, 0x02, 0); m45->cpuWrite(0x8001, 0x55, 0); // R2
    uint32_t m45BasePrg=0,m45BaseChr=0;
    m45->cpuMapRead(0x8000,m45BasePrg); m45->ppuMapRead(0x1000,m45BaseChr);
    m45->cpuWrite(0x6000, 0x80, 0); // #0 CHR OR low
    m45->cpuWrite(0x6000, 0x41, 0); // #1 PRG OR low/A19-A20
    m45->cpuWrite(0x6000, 0x8E, 0); // #2 CHR mask/high + PRG A21-A22
    m45->cpuWrite(0x6000, 0x70, 0); // #3 PRG mask + lock
    uint32_t m45OuterPrg=0,m45OuterChr=0,m45LockedChr=0;
    m45->cpuMapRead(0x8000,m45OuterPrg); m45->ppuMapRead(0x1000,m45OuterChr);
    m45->cpuWrite(0x6000, 0x01, 0); // locked: must not replace register #0
    m45->ppuMapRead(0x1000,m45LockedChr);

    std::vector<uint8_t> m45State;
    m45->saveState(m45State);
    m45->cpuWrite(0x6001, 0x00, 0); // outer reset + unlock
    uint32_t m45ResetPrg=0,m45ResetChr=0,m45UnlockedChr=0;
    m45->cpuMapRead(0x8000,m45ResetPrg); m45->ppuMapRead(0x1000,m45ResetChr);
    m45->cpuWrite(0x6000, 0x20, 0); // must be register #0 after reset
    m45->ppuMapRead(0x1000,m45UnlockedChr);
    const uint8_t* m45StateP = m45State.data();
    const bool m45StateLoaded = m45->loadState(m45StateP, m45StateP + m45State.size()) &&
        m45StateP == m45State.data() + m45State.size();
    uint32_t m45RestoredPrg=0,m45RestoredChr=0;
    m45->cpuMapRead(0x8000,m45RestoredPrg); m45->ppuMapRead(0x1000,m45RestoredChr);
    const bool mapper45 = m45BasePrg == 0x54000 && m45BaseChr == 0x15400 &&
        m45OuterPrg == 0x496000 && m45OuterChr == 0x235400 && m45LockedChr == m45OuterChr &&
        m45ResetPrg == 0x54000 && m45ResetChr == 0x15400 && m45UnlockedChr == 0x1D400 &&
        m45StateLoaded && m45RestoredPrg == m45OuterPrg && m45RestoredChr == m45OuterChr &&
        mapperImplementationSupported(45, 0);
    std::printf("mapper45_ga23c=%s base=%05X/%05X outer=%06X/%06X reset=%05X/%05X lock=%s state=%s\n",
        mapper45 ? "PASS" : "FAIL", unsigned(m45BasePrg), unsigned(m45BaseChr),
        unsigned(m45OuterPrg), unsigned(m45OuterChr), unsigned(m45ResetPrg), unsigned(m45ResetChr),
        m45LockedChr == m45OuterChr ? "PASS" : "FAIL", m45StateLoaded ? "PASS" : "FAIL");
    ok &= mapper45;

    // Mapper 185: CNROM protection-only latch. NES 2.0 submappers 4-7
    // identify which low-two-bit latch value enables the single 8 KiB CHR ROM.
    auto m185s4 = makeMapper(185, 0x8000, 0x2000, 0, 0, 4, true);
    uint32_t m185Mapped = 0;
    const bool m185PowerEnabled = m185s4->ppuMapRead(0x0123, m185Mapped) && m185Mapped == 0x0123;
    m185s4->cpuWrite(0x8000, 0x01, 0);
    const bool m185Disabled = !m185s4->ppuMapRead(0x0123, m185Mapped);
    auto m185s6 = makeMapper(185, 0x8000, 0x2000, 0, 0, 6, true);
    const bool m185WrongInitially = !m185s6->ppuMapRead(0x0000, m185Mapped);
    m185s6->cpuWrite(0x8000, 0x02, 0);
    const bool m185Enabled2 = m185s6->ppuMapRead(0x1ABC, m185Mapped) && m185Mapped == 0x1ABC;
    // Legacy/iNES submapper 0 cannot identify the programmed CS value. The
    // compatibility rule is two CPU $2007 pattern reads of open bus ($FF)
    // after reset, without allowing renderer fetches to consume that window.
    auto m185legacy = makeMapper(185, 0x8000, 0x2000, 0, 0, 0, false);
    uint8_t legacyData = 0; uint32_t legacyMapped = 0;
    const bool legacyRenderMaps = !m185legacy->ppuReadOverride(0x0010, PpuFetchKind::Background, legacyData) &&
        m185legacy->ppuMapReadEx(0x0010, legacyMapped, PpuFetchKind::Background) && legacyMapped == 0x0010;
    const bool legacyRead1 = m185legacy->ppuReadOverride(0x0000, PpuFetchKind::Cpu, legacyData) && legacyData == 0xFF;
    legacyData = 0;
    const bool legacyRead2 = m185legacy->ppuReadOverride(0x1FFF, PpuFetchKind::Cpu, legacyData) && legacyData == 0xFF;
    legacyData = 0;
    const bool legacyRead3Maps = !m185legacy->ppuReadOverride(0x0123, PpuFetchKind::Cpu, legacyData) &&
        m185legacy->ppuMapReadEx(0x0123, legacyMapped, PpuFetchKind::Cpu) && legacyMapped == 0x0123;

    // Save state must preserve how many startup reads have already occurred.
    m185legacy->reset(true);
    m185legacy->ppuReadOverride(0x0000, PpuFetchKind::Cpu, legacyData);
    std::vector<uint8_t> m185State; m185legacy->saveState(m185State);
    m185legacy->ppuReadOverride(0x0000, PpuFetchKind::Cpu, legacyData);
    const uint8_t* m185StateP = m185State.data();
    const bool legacyStateLoad = m185legacy->loadState(m185StateP, m185StateP + m185State.size()) &&
        m185StateP == m185State.data() + m185State.size();
    const bool legacyStateSecond = m185legacy->ppuReadOverride(0x0000, PpuFetchKind::Cpu, legacyData);
    const bool legacyStateThird = !m185legacy->ppuReadOverride(0x0000, PpuFetchKind::Cpu, legacyData);
    m185legacy->reset(false);
    const bool legacySoftReset = m185legacy->ppuReadOverride(0x0000, PpuFetchKind::Cpu, legacyData);

    const bool mapper185Legacy = legacyRenderMaps && legacyRead1 && legacyRead2 && legacyRead3Maps &&
        legacyStateLoad && legacyStateSecond && legacyStateThird && legacySoftReset &&
        mapperImplementationSupported(185, 0);
    const bool mapper185 = m185PowerEnabled && m185Disabled && m185WrongInitially && m185Enabled2 &&
        m185s4->hasBusConflicts() && mapper185Legacy &&
        mapperImplementationSupported(185, 4) && mapperImplementationSupported(185, 7);
    std::printf("mapper185_protection=%s legacy=%s\n", mapper185 ? "PASS" : "FAIL", mapper185Legacy ? "PASS" : "FAIL");
    ok &= mapper185;

    // Mapper 210: Namco 175/340. Legacy submapper 0 is resolved from RAM/
    // battery metadata: RAM-bearing images use the 175, otherwise the 340.
    // The 175 has hardwired mirroring and only $C000-$C7FF controls WRAM;
    // the 340 has no WRAM and uses E000 bits 7-6 as 1A/V/1B/H.
    auto m210Legacy175 = makeMapper(210, 0x80000, 0x40000, 0x0800, 0, 0, true, Mirror::Vertical, false);
    uint32_t m210RamMap = 0;
    const bool m210RamInitiallyLocked = !m210Legacy175->mapPrgRam(0x6000, m210RamMap, true);
    m210Legacy175->cpuWrite(0xC800, 0x01, 0); // nonexistent on N175
    const bool m210C800Ignored = !m210Legacy175->mapPrgRam(0x6000, m210RamMap, true);
    m210Legacy175->cpuWrite(0xC000, 0x01, 0);
    const bool m210RamEnabled = m210Legacy175->mapPrgRam(0x6000, m210RamMap, true) && m210RamMap == 0;
    uint32_t m210RamMirror = 0;
    const bool m210RamMirrored = m210Legacy175->mapPrgRam(0x6800, m210RamMirror, false) && m210RamMirror == 0;
    m210Legacy175->cpuWrite(0xE000, 0xC0, 0);
    const bool m210175Hardwired = m210Legacy175->mirroring() == Mirror::Vertical;

    auto m210Legacy340 = makeMapper(210, 0x80000, 0x40000, 0, 0, 0, true, Mirror::Vertical, false);
    uint32_t m210NoRamMap = 0;
    const bool m210NoRam = !m210Legacy340->mapPrgRam(0x6000, m210NoRamMap, false);
    m210Legacy340->cpuWrite(0xE000, 0x00, 0); const bool m210M0 = m210Legacy340->mirroring() == Mirror::OnescreenLo;
    m210Legacy340->cpuWrite(0xE000, 0x40, 0); const bool m210M1 = m210Legacy340->mirroring() == Mirror::Vertical;
    m210Legacy340->cpuWrite(0xE000, 0x80, 0); const bool m210M2 = m210Legacy340->mirroring() == Mirror::OnescreenHi;
    m210Legacy340->cpuWrite(0xE000, 0xC0, 0); const bool m210M3 = m210Legacy340->mirroring() == Mirror::Horizontal;

    auto m210s1 = makeMapper(210, 0x80000, 0x40000, 0x2000, 0, 1, true, Mirror::Horizontal, false);
    m210s1->cpuWrite(0xE000, 0x40, 0);
    const bool m210Explicit175 = m210s1->mirroring() == Mirror::Horizontal;
    auto m210s2 = makeMapper(210, 0x80000, 0x40000, 0, 0, 2, true, Mirror::Vertical, false);
    m210s2->cpuWrite(0xE000, 0x80, 0);
    const bool m210Explicit340 = m210s2->mirroring() == Mirror::OnescreenHi;
    const bool m210Gate = mapperImplementationSupported(210,0) && mapperImplementationSupported(210,1) &&
        mapperImplementationSupported(210,2) && !mapperImplementationSupported(210,3);
    const bool mapper210 = m210RamInitiallyLocked && m210C800Ignored && m210RamEnabled && m210RamMirrored &&
        m210175Hardwired && m210NoRam && m210M0 && m210M1 && m210M2 && m210M3 &&
        m210Explicit175 && m210Explicit340 && m210Gate;
    std::printf("mapper210_namco=%s legacy175=%s legacy340=%s mirror=%s gate=%s\n",
        mapper210 ? "PASS" : "FAIL",
        (m210RamEnabled && m210C800Ignored && m210175Hardwired) ? "PASS" : "FAIL",
        m210NoRam ? "PASS" : "FAIL",
        (m210M0 && m210M1 && m210M2 && m210M3) ? "PASS" : "FAIL",
        m210Gate ? "PASS" : "FAIL");
    ok &= mapper210;

    // Mapper 228: Action 52/Cheetahmen II encode almost all banking in the
    // write address. Validate 32 KiB mode, mirrored-16 KiB mode, CHR banking,
    // mirroring, and the physically absent PRG chip-2 open-bus selection.
    auto m228 = makeMapper(228, 0x180000, 0x80000, 0, 0, 0, false, Mirror::Vertical);
    // chip 1, page 6, 32 KiB mode; CHR high nibble 9 and low bits 2.
    const uint16_t a228_32 = uint16_t(0x8000 | 0x0800 | (6u << 6) | 0x0009);
    m228->cpuWrite(a228_32, 0x02, 0);
    uint32_t m228Lo=0,m228Hi=0,m228Chr=0;
    const bool m228LoRead=m228->cpuMapRead(0x8000,m228Lo);
    const bool m228HiRead=m228->cpuMapRead(0xC000,m228Hi);
    const bool m228ChrRead=m228->ppuMapRead(0x0000,m228Chr);
    const bool m228Mode32 = m228LoRead && m228HiRead &&
        m228Lo == 0x98000 && m228Hi == 0x9C000 &&
        m228ChrRead && m228Chr == 0x4C000 && m228->mirroring() == Mirror::Vertical;

    // chip 3, page 3, mirrored 16 KiB mode, horizontal mirroring.
    const uint16_t a228_16 = uint16_t(0x8000 | 0x2000 | 0x1800 | (3u << 6) | 0x0020 | 0x0004);
    m228->cpuWrite(a228_16, 0x01, 0);
    uint32_t m228MirrorLo=0,m228MirrorHi=0;
    const bool m228Mirrored = m228->cpuMapRead(0x8000,m228MirrorLo) && m228->cpuMapRead(0xC000,m228MirrorHi) &&
        m228MirrorLo == 0x10C000 && m228MirrorHi == 0x10C000 && m228->mirroring() == Mirror::Horizontal;

    const uint16_t a228Missing = uint16_t(0x8000 | 0x1000); // chip 2
    m228->cpuWrite(a228Missing, 0, 0);
    uint32_t m228MissingMap=0;
    const bool m228OpenBus = !m228->cpuMapRead(0x8000,m228MissingMap);
    const bool mapper228 = m228Mode32 && m228Mirrored && m228OpenBus && mapperImplementationSupported(228,0);
    std::printf("mapper228_action52=%s 32=%05X/%05X chr=%05X mirror16=%05X open=%s\n",
        mapper228 ? "PASS" : "FAIL", unsigned(m228Lo), unsigned(m228Hi), unsigned(m228Chr),
        unsigned(m228MirrorLo), m228OpenBus ? "PASS" : "FAIL");
    ok &= mapper228;

    // J.Y. Company ASIC (mappers 90/209/211): verify the shared banking,
    // arithmetic/IRQ hardware and the PCB-specific nametable jumper behavior.
    auto jy90 = makeMapper(90, 0x200000, 0x200000, 0x2000, 0, 0, true);
    jy90->cpuWrite(0xD000, 0x1A, 0); // 8 KiB PRG, 1 KiB CHR
    jy90->cpuWrite(0x8000, 0x03, 0);
    jy90->cpuWrite(0x9000, 0x05, 0);
    jy90->cpuWrite(0xA000, 0x00, 0);
    jy90->cpuWrite(0xD003, 0x15, 0); // PRG outer=2, CHR outer=2 + A18
    uint32_t jyPrg=0,jyChr=0;
    jy90->cpuMapRead(0x8000,jyPrg); jy90->ppuMapRead(0x0000,jyChr);
    const bool jyOuter = jyPrg == 0x106000 && jyChr == 0x141400;

    jy90->cpuWrite(0xD000, 0x03, 0); // reversed-bit 8 KiB PRG mode
    jy90->cpuWrite(0xD003, 0x00, 0);
    jy90->cpuWrite(0x8000, 0x02, 0);
    jy90->cpuMapRead(0x8000,jyPrg);
    const bool jyReverse = jyPrg == 0x40000; // reverse7(2)=32 -> 32*8 KiB

    jy90->cpuWrite(0x5800, 12, 0); jy90->cpuWrite(0x5801, 13, 0);
    uint8_t jyMulLo=0,jyMulHi=0; jy90->cpuReadRegister(0x5800,jyMulLo); jy90->cpuReadRegister(0x5801,jyMulHi);
    const bool jyMulDeferred = (uint16_t(jyMulLo)|(uint16_t(jyMulHi)<<8)) != 156;
    for(int i=0;i<8;i++) jy90->clockCpu();
    jy90->cpuReadRegister(0x5800,jyMulLo); jy90->cpuReadRegister(0x5801,jyMulHi);
    const bool jyMul = jyMulDeferred && (uint16_t(jyMulLo)|(uint16_t(jyMulHi)<<8)) == 156;
    jy90->cpuWrite(0xC001,0x40,0); // CPU M2, increment
    jy90->cpuWrite(0xC004,0xFF,0); jy90->cpuWrite(0xC005,0xFF,0); jy90->cpuWrite(0xC003,0,0);
    jy90->clockCpu();
    const bool jyIrq = jy90->irqActive();
    jy90->cpuWrite(0xC002,0,0);
    const bool jyAck = !jy90->irqActive();

    jy90->cpuWrite(0xD001,0x08,0); jy90->cpuWrite(0xB000,1,0);
    NametableSource jyNtSource=NametableSource::Ciram; uint32_t jyNtMap=0;
    const bool jy90Suppress = !jy90->mapNametable(0x2000,jyNtSource,jyNtMap);

    auto jy209 = makeMapper(209, 0x200000, 0x200000, 0x2000, 0, 0, true);
    jy209->cpuWrite(0xD001,0x08,0); jy209->cpuWrite(0xB000,1,0);
    const bool jy209Ext = jy209->mapNametable(0x2000,jyNtSource,jyNtMap) &&
        jyNtSource==NametableSource::Ciram && jyNtMap==0x400;
    jy209->cpuWrite(0xD001,0x00,0); jy209->cpuWrite(0xD000,0x60,0); // ROM NT global
    jy209->cpuWrite(0xB000,0x02,0); jy209->cpuWrite(0xB004,0x00,0);
    const bool jyRomNt = jy209->mapNametable(0x2000,jyNtSource,jyNtMap) &&
        jyNtSource==NametableSource::ChrRom && jyNtMap==0x800;
    const bool jyWriteCiram = jy209->mapNametableWrite(0x2000,jyNtSource,jyNtMap) &&
        jyNtSource==NametableSource::Ciram && jyNtMap==0;
    jy209->cpuWrite(0xD000,0x0A,0); // 4 KiB CHR + 8 KiB PRG
    jy209->cpuWrite(0xD003,0x80,0); // enable MMC4-like latches
    jy209->cpuWrite(0x9000,0x00,0); jy209->cpuWrite(0x9002,0x04,0);
    uint32_t jyLatch0=0,jyLatch1=0; jy209->ppuMapRead(0x0000,jyLatch0);
    jy209->notifyPpuAddress(0x0FE8,0); jy209->ppuMapRead(0x0000,jyLatch1);
    const bool jyMmc4Latch = jyLatch0==0x0000 && jyLatch1==0x1000;
    auto jy35 = makeMapper(35,0x80000,0x80000,0x2000,0,0,true);
    uint32_t jy35Ram=0; const bool jy35Wram = jy35->mapPrgRam(0x6000,jy35Ram,false) && jy35Ram==0;
    const bool jySupport = mapperImplementationSupported(35,0) && mapperImplementationSupported(90,0) && mapperImplementationSupported(209,0) && mapperImplementationSupported(211,0);
    const bool mapperJY = jyOuter && jyReverse && jyMul && jyIrq && jyAck && jy90Suppress && jy209Ext && jyRomNt && jyWriteCiram && jyMmc4Latch && jy35Wram && jySupport;
    std::printf("jy_asic=%s outer=%s reverse=%s mul=%s irq=%s nt90=%s nt209=%s romnt=%s write_ciram=%s latch=%s m35=%s\n",
        mapperJY?"PASS":"FAIL", jyOuter?"PASS":"FAIL", jyReverse?"PASS":"FAIL", jyMul?"PASS":"FAIL",
        (jyIrq&&jyAck)?"PASS":"FAIL", jy90Suppress?"PASS":"FAIL", jy209Ext?"PASS":"FAIL",
        jyRomNt?"PASS":"FAIL", jyWriteCiram?"PASS":"FAIL", jyMmc4Latch?"PASS":"FAIL", jy35Wram?"PASS":"FAIL");
    ok &= mapperJY;

    // Mapper 176 / 8025-FK23C foundations. Submapper 0 is the six-bit
    // MMC3 variant; submapper 1 expands to eight MMC3 PRG bits and adds
    // extended MMC3 plus CNROM/NROM CHR modes.
    auto m176s0 = makeMapper(176, 0x200000, 0x100000, 0x2000, 0, 0, true);
    uint32_t m176s0Fixed=0,m176s0Outer=0;
    m176s0->cpuMapRead(0xE000,m176s0Fixed);
    m176s0->cpuWrite(0x5000,0x01,0); // 256 KiB MMC3 outer window
    m176s0->cpuWrite(0x5001,0x70,0);
    m176s0->cpuWrite(0x8000,0x06,0); m176s0->cpuWrite(0x8001,0x03,0);
    m176s0->cpuMapRead(0x8000,m176s0Outer);
    const bool m176SixBit = m176s0Fixed==0x7E000 && m176s0Outer==0x1C6000;

    auto m176s1 = makeMapper(176, 0x400000, 0x100000, 0x2000, 0, 1, true);
    uint32_t m176Ext=0,m176Ignored=0,m176Cnrom=0,m176Nrom=0;
    m176s1->cpuWrite(0x5003,0x02,0); // extended MMC3
    m176s1->cpuWrite(0x8000,0x08,0); m176s1->cpuWrite(0x8001,0x22,0);
    m176s1->cpuMapRead(0xC000,m176Ext);
    m176s1->cpuWrite(0x8002,0x07,0); // E003 decode: must not alias $8000/$8001
    m176s1->cpuMapRead(0xC000,m176Ignored);
    m176s1->cpuWrite(0x5003,0x00,0);
    m176s1->cpuWrite(0x5002,0x20,0);
    m176s1->cpuWrite(0x5000,0x40,0); // CNROM 32 KiB mode
    m176s1->cpuWrite(0x8000,0x02,0); m176s1->ppuMapRead(0x0000,m176Cnrom);
    m176s1->cpuWrite(0x5000,0x60,0); // NROM CHR mode
    m176s1->ppuMapRead(0x0000,m176Nrom);
    const bool m176Fk23 = m176Ext==0x44000 && m176Ignored==m176Ext &&
        m176Cnrom==0x44000 && m176Nrom==0x40000;
    // Submapper 2 / Waixing FS005: swapped MMC3 $46/$47 banks, 32 KiB
    // banked WRAM/protection aperture, mixed CHR RAM, and one-screen mirroring.
    auto m176s2 = makeMapper(176, 0x4000000, 0x200000, 0x8000, 0x2000, 2, true);
    m176s2->cpuWrite(0x8000,0x06,0); m176s2->cpuWrite(0x8001,0x03,0);
    uint32_t s2a=0; m176s2->cpuMapRead(0xA000,s2a); // swapped #6 write lands in #7
    m176s2->cpuWrite(0xA001,0xA1,0); // config on, bank 1, outer regs hidden
    uint32_t s2Prot=0,s2Ram1=0;
    const bool s2ProtMap=m176s2->mapPrgRam(0x5000,s2Prot,false);
    const bool s2RamMap=m176s2->mapPrgRam(0x6000,s2Ram1,false);
    m176s2->cpuWrite(0xA001,0xE8,0); // config + outer regs + CHR RAM
    m176s2->cpuWrite(0xA000,0x03,0);
    const bool s2Mirror=m176s2->mirroring()==Mirror::OnescreenHi;
    const bool s2ChrRam=m176s2->ppuUsesChrRam(0x0000);
    m176s2->cpuWrite(0x5000,0x88,0); // PRG A21/A22
    m176s2->cpuWrite(0x5002,0xE0,0); // PRG A23/A24/A25
    uint32_t s2High=0; m176s2->cpuMapRead(0xE000,s2High);
    const bool m176Fs005=s2a==0x06000 && s2ProtMap&&s2Prot==0x5000 &&
        s2RamMap&&s2Ram1==0x2000 && s2Mirror && s2ChrRam && s2High==0x3E7E000;

    // Submapper 3 / JX9003B adds independent PRG/CHR high registers.
    auto m176s3 = makeMapper(176, 0x2000000, 0x1000000, 0x2000, 0, 3, true);
    m176s3->cpuWrite(0x5005,0x02,0); // PRG A22
    m176s3->cpuWrite(0x5006,0x01,0); // CHR A21
    uint32_t s3Prg=0,s3Chr=0; m176s3->cpuMapRead(0xE000,s3Prg); m176s3->ppuMapRead(0x0000,s3Chr);
    const bool m176Jx9003=s3Prg==0x5FE000 && s3Chr==0x200000;

    // Submapper 4 sources PRG A21 from CHR-base bit 7.
    auto m176s4 = makeMapper(176, 0x400000, 0x200000, 0x2000, 0, 4, true);
    m176s4->cpuWrite(0x5002,0x80,0);
    uint32_t s4Prg=0,s4Chr=0; m176s4->cpuMapRead(0xE000,s4Prg); m176s4->ppuMapRead(0x0000,s4Chr);
    const bool m176Smart=s4Prg==0x27E000 && s4Chr==0x100000;

    // Submapper 5 / HST-162 sources PRG A24..A19 from $4800.
    auto m176s5 = makeMapper(176, 0x800000, 0x100000, 0x2000, 0, 5, true);
    m176s5->cpuWrite(0x4800,0x01,0);
    uint32_t s5Prg=0; m176s5->cpuMapRead(0xE000,s5Prg);
    const bool m176Hst=s5Prg==0x0FE000;

    const bool m176Support = mapperImplementationSupported(176,0) && mapperImplementationSupported(176,1) &&
        mapperImplementationSupported(176,2) && mapperImplementationSupported(176,3) &&
        mapperImplementationSupported(176,4) && mapperImplementationSupported(176,5) &&
        !mapperImplementationSupported(176,6);
    std::printf("mapper176_fk23c=%s sixbit=%s ext=%s cnrom=%s fs005=%s jx9003=%s sm4=%s hst=%s support_gate=%s\n",
        (m176SixBit&&m176Fk23&&m176Fs005&&m176Jx9003&&m176Smart&&m176Hst&&m176Support)?"PASS":"FAIL",
        m176SixBit?"PASS":"FAIL", (m176Ext==0x44000&&m176Ignored==m176Ext)?"PASS":"FAIL",
        (m176Cnrom==0x44000&&m176Nrom==0x40000)?"PASS":"FAIL",m176Fs005?"PASS":"FAIL",
        m176Jx9003?"PASS":"FAIL",m176Smart?"PASS":"FAIL",m176Hst?"PASS":"FAIL",m176Support?"PASS":"FAIL");
    ok &= m176SixBit && m176Fk23 && m176Fs005 && m176Jx9003 && m176Smart && m176Hst && m176Support;


    // Mapper 268 / AA6023 CoolBoy/MindKids. Submappers 0-3 share the ASIC
    // but move the six outer registers and, on AA6023B, alter register #1.
    auto m268s0 = makeMapper(268, 0x2000000, 0, 0x2000, 0x40000, 0, true);
    uint32_t c0Base=0,c0Outer=0,c0GnLo=0,c0GnHi=0,c0Chr=0;
    m268s0->cpuMapRead(0xE000,c0Base);
    m268s0->cpuWrite(0x6000,0x77,0); // A24/A23 + forced A19..A17
    m268s0->cpuWrite(0x6001,0x1C,0); // A22/A21/A20 offsets
    m268s0->cpuMapRead(0xE000,c0Outer);
    m268s0->reset(true);
    m268s0->cpuWrite(0x6001,0x02,0); // original revision: L=1 -> 32 KiB
    m268s0->cpuWrite(0x6002,0x0A,0); // GNROM CHR A16..A13
    m268s0->cpuWrite(0x6003,0x1C,0); // GNROM, QQ=3
    m268s0->cpuMapRead(0x8000,c0GnLo); m268s0->cpuMapRead(0xC000,c0GnHi); m268s0->ppuMapRead(0x0000,c0Chr);
    const bool coolboy0 = c0Base==0x7E000 && c0Outer==0x1FFE000 && c0GnLo==0x18000 && c0GnHi==0x7C000 && c0Chr==0x14000;

    // Lockout freezes all outer registers except #2 while in MMC3 mode.
    m268s0->reset(true); m268s0->cpuWrite(0x6000,0x40,0); m268s0->cpuWrite(0x6003,0x80,0);
    uint32_t c0LockedA=0,c0LockedB=0; m268s0->cpuMapRead(0xE000,c0LockedA);
    m268s0->cpuWrite(0x6000,0x00,0); m268s0->cpuMapRead(0xE000,c0LockedB);
    const bool coolboyLock = c0LockedA==c0LockedB;

    // MindKids moves the registers to $5000. AA6023B (submapper 2) moves
    // them to $7000 and flips the L control's sense/position.
    auto m268s1 = makeMapper(268, 0x2000000, 0, 0x2000, 0x40000, 1, true);
    m268s1->cpuWrite(0x5000,0x77,0); m268s1->cpuWrite(0x5001,0x1C,0);
    uint32_t c1Outer=0; m268s1->cpuMapRead(0xE000,c1Outer);
    auto m268s2 = makeMapper(268, 0x2000000, 0, 0x2000, 0x40000, 2, true);
    m268s2->cpuWrite(0x7002,0x0A,0); m268s2->cpuWrite(0x7003,0x1C,0); // L=0 => 32 KiB
    uint32_t c2GnLo=0,c2GnHi=0; m268s2->cpuMapRead(0x8000,c2GnLo); m268s2->cpuMapRead(0xC000,c2GnHi);
    const bool coolboyVariants = c1Outer==0x1FFE000 && c2GnLo==0x18000 && c2GnHi==0x7C000;
    // Later wiring families (submappers 4-11).
    // 4/5 return to the 6K/5K register windows and cap PRG addressing at 4 MiB.
    auto m268s4 = makeMapper(268, 0x800000, 0x400000, 0x2000, 0, 4, true);
    uint32_t s4Before=0,s4After=0,s4Wrong=0;
    m268s4->cpuMapRead(0xE000,s4Before);
    m268s4->cpuWrite(0x7000,0x70,0); m268s4->cpuMapRead(0xE000,s4Wrong); // must be ignored
    m268s4->cpuWrite(0x6000,0x70,0); m268s4->cpuMapRead(0xE000,s4After);
    const bool coolboy45 = s4Wrong==s4Before && s4After!=s4Before && s4After<0x400000;

    // 6/7 select between two equal PRG chips. With A=1, D directly chooses chip.
    auto m268s6 = makeMapper(268, 0x200000, 0, 0x2000, 0x20000, 6, true);
    uint32_t s6Chip0=0,s6Chip1=0;
    m268s6->cpuWrite(0x6000,0x80,0); m268s6->cpuMapRead(0xE000,s6Chip0);
    m268s6->cpuWrite(0x6000,0x88,0); m268s6->cpuMapRead(0xE000,s6Chip1);
    const bool coolboy67 = s6Chip0<0x100000 && s6Chip1>=0x100000;

    // 8/9 expose up to 256 KiB CHR-RAM and D ($xxx0.4) write-protects it.
    auto m268s8 = makeMapper(268, 0x200000, 0, 0x2000, 0x40000, 8, true);
    uint32_t s8Chr=0; const bool s8Writable=m268s8->ppuMapWrite(0x0000,s8Chr);
    m268s8->cpuWrite(0x6000,0x10,0);
    const bool s8Protected=!m268s8->ppuMapWrite(0x0000,s8Chr);
    const bool coolboy89=s8Writable&&s8Protected;

    // 10/11 borrow outer bits for one-screen mirroring. S=0 uses C as CIRAM A10;
    // S=1 reconnects the ordinary MMC3 H/V mirroring register.
    auto m268s10 = makeMapper(268, 0x800000, 0, 0x2000, 0x40000, 10, true);
    NametableSource ntSrc=NametableSource::MapperRam; uint32_t ntMap=0;
    m268s10->cpuWrite(0x6000,0x00,0); const bool ntLo=m268s10->mapNametable(0x2400,ntSrc,ntMap)&&ntSrc==NametableSource::Ciram&&ntMap==0;
    m268s10->cpuWrite(0x6000,0x10,0); const bool ntHi=m268s10->mapNametable(0x2400,ntSrc,ntMap)&&ntMap==0x400;
    m268s10->cpuWrite(0x6000,0x20,0); m268s10->cpuWrite(0xA000,1,0);
    const bool ntNormal=!m268s10->mapNametable(0x2400,ntSrc,ntMap)&&m268s10->mirroring()==Mirror::Horizontal;
    const bool coolboy1011=ntLo&&ntHi&&ntNormal;

    // Mixed CHR-ROM/RAM mode ($xxx4): D0 enables, D1-D7 select the MMC3 bank value.
    auto m268mix = makeMapper(268,0x200000,0x20000,0x2000,0x20000,0,true);
    const bool mixRom=!m268mix->ppuUsesChrRam(0x0000);
    m268mix->cpuWrite(0x6004,0x01,0);
    const bool mixRam=m268mix->ppuUsesChrRam(0x0000);
    const bool coolboyMixed=mixRom&&mixRam;

    const bool coolboySupport = mapperImplementationSupported(268,0) && mapperImplementationSupported(268,1) &&
        mapperImplementationSupported(268,2) && mapperImplementationSupported(268,3) && mapperImplementationSupported(268,4) &&
        mapperImplementationSupported(268,5) && mapperImplementationSupported(268,6) && mapperImplementationSupported(268,7) &&
        mapperImplementationSupported(268,8) && mapperImplementationSupported(268,9) && mapperImplementationSupported(268,10) &&
        mapperImplementationSupported(268,11) && !mapperImplementationSupported(268,12);
    const bool mapper268 = coolboy0 && coolboyLock && coolboyVariants && coolboy45 && coolboy67 && coolboy89 && coolboy1011 && coolboyMixed && coolboySupport;
    std::printf("mapper268_coolboy=%s base=%06X outer=%07X gn=%05X/%05X chr=%05X lock=%s variants=%s s45=%s s67=%s s89=%s s1011=%s mixed=%s support_gate=%s\n",
        mapper268?"PASS":"FAIL",unsigned(c0Base),unsigned(c0Outer),unsigned(c0GnLo),unsigned(c0GnHi),unsigned(c0Chr),
        coolboyLock?"PASS":"FAIL",coolboyVariants?"PASS":"FAIL",coolboy45?"PASS":"FAIL",coolboy67?"PASS":"FAIL",
        coolboy89?"PASS":"FAIL",coolboy1011?"PASS":"FAIL",coolboyMixed?"PASS":"FAIL",coolboySupport?"PASS":"FAIL");
    ok &= mapper268;

    // Color Dreams mapper 11 uses D1-D0 for PRG and D7-D4 for CHR. The old
    // implementation had those fields reversed.
    auto colorDreams = makeMapper(11, 0x20000, 0x20000, 0, 0);
    colorDreams->cpuWrite(0x8000, 0xA2, 0);
    uint32_t cdPrg = 0, cdChr = 0;
    colorDreams->cpuMapRead(0x8000, cdPrg);
    colorDreams->ppuMapRead(0x0000, cdChr);
    const bool colorDreamsBits = cdPrg == 0x10000 && cdChr == 0x14000;
    const bool supportedSubmaps = mapperImplementationSupported(32, 1) &&
        mapperImplementationSupported(78, 3) && mapperImplementationSupported(85, 2) &&
        mapperImplementationSupported(30, 3) && mapperImplementationSupported(30, 4) &&
        mapperImplementationSupported(15, 0) && mapperImplementationSupported(31, 0) &&
        mapperImplementationSupported(37, 0) && mapperImplementationSupported(47, 0) && mapperImplementationSupported(49, 0) &&
        mapperImplementationSupported(114, 1) && mapperImplementationSupported(115, 0) &&
        mapperImplementationSupported(116, 0) && mapperImplementationSupported(116, 1) && mapperImplementationSupported(116, 2) &&
        mapperImplementationSupported(116, 3) && mapperImplementationSupported(197, 3) &&
        !mapperImplementationSupported(78, 2) && !mapperImplementationSupported(30, 5) &&
        mapperImplementationSupported(268, 3) && mapperImplementationSupported(268, 11) && !mapperImplementationSupported(268, 12);
    std::printf("color_dreams_bits=%s submapper_reporting=%s\n",
        colorDreamsBits ? "PASS" : "FAIL", supportedSubmaps ? "PASS" : "FAIL");
    ok &= colorDreamsBits && supportedSubmaps;

    // Namco 163: address-port auto-increment saturates at $7F; it does not
    // wrap to $00. Two writes with $FF selected therefore both hit byte $7F.
    auto n163 = makeMapper(19);
    n163->cpuWrite(0xF800, 0xFF, 0);
    n163->cpuWrite(0x4800, 0xAA, 0);
    n163->cpuWrite(0x4800, 0xBB, 0);
    uint8_t n163Last = 0, n163Zero = 0;
    n163->cpuWrite(0xF800, 0x7F, 0);
    const bool n163ReadLast = n163->cpuReadRegister(0x4800, n163Last);
    n163->cpuWrite(0xF800, 0x00, 0);
    const bool n163ReadZero = n163->cpuReadRegister(0x4800, n163Zero);
    const bool n163Saturates = n163ReadLast && n163ReadZero && n163Last == 0xBB && n163Zero == 0x00;
    std::printf("n163_autoinc_saturate=%s last=%02X zero=%02X\n",
        n163Saturates ? "PASS" : "FAIL", n163Last, n163Zero);
    ok &= n163Saturates;

    // NES 2.0 Mapper 19 submappers encode the N163 cartridge mixing
    // resistor. Submappers 1/2 have no expansion sound, while 3/4/5 are
    // progressively louder hardware mixes. Program one active channel and
    // verify both the mute variants and relative resistor gains.
    auto n163Level = [](uint8_t submapper) {
        auto m = makeMapper(19, 0x20000, 0x20000, 0x2000, 0, submapper, true);
        auto writeRam = [&](uint8_t a, uint8_t d) {
            m->cpuWrite(0xF800, a, 0);
            m->cpuWrite(0x4800, d, 0);
        };
        writeRam(0x00, 0x00); // waveform samples 0/0 => positive full-scale
        writeRam(0x78, 0x01); // channel 7 frequency low
        writeRam(0x79, 0x00); // phase low
        writeRam(0x7A, 0x00); // frequency middle
        writeRam(0x7B, 0x00); // phase middle
        writeRam(0x7C, 0xFC); // 4-sample waveform, frequency high = 0
        writeRam(0x7D, 0x00); // phase high
        writeRam(0x7E, 0x00); // waveform address
        writeRam(0x7F, 0x0F); // one channel, maximum volume
        for (int i = 0; i < 15; ++i) m->clockCpu();
        return std::fabs(m->expansionAudioSample(false));
    };
    const float n163Sm1 = n163Level(1);
    const float n163Sm2 = n163Level(2);
    const float n163Sm3 = n163Level(3);
    const float n163Sm4 = n163Level(4);
    const float n163Sm5 = n163Level(5);
    const bool n163MutedMix = n163Sm1 < 0.000001f && n163Sm2 < 0.000001f;
    const bool n163MixOrder = n163Sm3 > 0.1f && n163Sm4 > n163Sm3 * 1.5f &&
        n163Sm5 > n163Sm4 * 1.2f;
    std::printf("n163_mix_submappers=%s mute=%s levels=%.4f/%.4f/%.4f\n",
        (n163MutedMix && n163MixOrder) ? "PASS" : "FAIL",
        n163MutedMix ? "PASS" : "FAIL", n163Sm3, n163Sm4, n163Sm5);
    ok &= n163MutedMix && n163MixOrder;

    // FME-7 IRQ phase: mapper clocks precede CPU bus accesses in Bus::clock(),
    // so a disabled->enabled control write consumes the counter edge belonging
    // to that CPU period. Starting from 3 therefore reaches IRQ after two more
    // mapper clocks. Counter-byte writes must not acknowledge a pending IRQ,
    // while any control-command $D write must acknowledge it.
    auto fme7 = makeMapper(69);
    fme7->cpuWrite(0x8000, 0x0E, 0); fme7->cpuWrite(0xA000, 0x03, 0);
    fme7->cpuWrite(0x8000, 0x0F, 0); fme7->cpuWrite(0xA000, 0x00, 0);
    fme7->cpuWrite(0x8000, 0x0D, 0); fme7->cpuWrite(0xA000, 0x81, 0);
    fme7->clockCpu();
    fme7->clockCpu();
    const bool fmeRaisedAtPollPhase = fme7->irqActive();
    fme7->cpuWrite(0x8000, 0x0E, 0); fme7->cpuWrite(0xA000, 0xFF, 0);
    const bool fmeLowWriteNoAck = fme7->irqActive();
    fme7->cpuWrite(0x8000, 0x0F, 0); fme7->cpuWrite(0xA000, 0xFF, 0);
    const bool fmeHighWriteNoAck = fme7->irqActive();
    fme7->cpuWrite(0x8000, 0x0D, 0); fme7->cpuWrite(0xA000, 0x81, 0);
    const bool fmeAckWhileEnabled = !fme7->irqActive();
    std::printf("fme7_irq_phase=%s counter_write_no_ack=%s ack_enabled=%s\n",
        fmeRaisedAtPollPhase ? "PASS" : "FAIL",
        (fmeLowWriteNoAck && fmeHighWriteNoAck) ? "PASS" : "FAIL",
        fmeAckWhileEnabled ? "PASS" : "FAIL");
    ok &= fmeRaisedAtPollPhase && fmeLowWriteNoAck && fmeHighWriteNoAck && fmeAckWhileEnabled;


    // FME-7 register 8 selects PRG-RAM in 8 KiB banks when bits 7/6 enable
    // RAM. NES 2.0 images can declare 32 KiB here, and the four banks must
    // remain distinct rather than aliasing a single $6000-$7FFF window.
    auto fme7Ram = makeMapper(69, 0x40000, 0x20000, 0x8000, 0, 0, true);
    bool fmeRamBanks = true;
    for (uint8_t bank = 0; bank < 4; ++bank) {
        fme7Ram->cpuWrite(0x8000, 0x08, 0);
        fme7Ram->cpuWrite(0xA000, uint8_t(0xC0 | bank), 0);
        uint32_t mapped = 0;
        fmeRamBanks &= fme7Ram->mapPrgRam(0x6000, mapped, false) && mapped == uint32_t(bank) * 0x2000;
    }
    std::printf("fme7_wram_banking=%s\n", fmeRamBanks ? "PASS" : "FAIL");
    ok &= fmeRamBanks;

    // VRC6 $B003 controls both pattern-table grouping and the nametable
    // register/A10 network. In mode 1 R4-R7 directly select four nametables;
    // mode 3 forces the low CHR/CIRAM address bit into 2 KiB spreads. Mapper
    // 26 must produce the same behavior after its A0/A1 register swap.
    auto checkVrc6Ppu = [&](uint16_t id) {
        auto v = makeMapper(id, 0x40000, 0x40000, 0x2000, 0, 0, true);
        const uint16_t r4 = id == 26 ? 0xE000 : 0xE000;
        const uint16_t r5 = id == 26 ? 0xE002 : 0xE001;
        const uint16_t r6 = id == 26 ? 0xE001 : 0xE002;
        const uint16_t r7 = 0xE003;
        v->cpuWrite(r4, 0x10, 0); v->cpuWrite(r5, 0x11, 0);
        v->cpuWrite(r6, 0x12, 0); v->cpuWrite(r7, 0x13, 0);
        // B003 mode 1 + CHR-ROM nametables + A10 override network enabled.
        v->cpuWrite(0xB003, 0x31, 0);
        bool romNt = true;
        for (uint8_t page = 0; page < 4; ++page) {
            NametableSource src = NametableSource::Ciram; uint32_t mapped = 0;
            romNt &= v->mapNametable(uint16_t(0x2000 + page * 0x400), src, mapped) &&
                src == NametableSource::ChrRom && mapped == uint32_t(0x10 + page) * 0x400;
        }
        // Mode 3 / $23 forces R6-even,R7-even,R6-odd,R7-odd on the existing PCB.
        v->cpuWrite(0xB003, 0x23, 0);
        static constexpr uint32_t expectedCiram[4] = {0x000,0x000,0x400,0x400};
        bool ciramNt = true;
        for (uint8_t page = 0; page < 4; ++page) {
            NametableSource src = NametableSource::ChrRom; uint32_t mapped = 0;
            ciramNt &= v->mapNametable(uint16_t(0x2000 + page * 0x400), src, mapped) &&
                src == NametableSource::Ciram && mapped == expectedCiram[page];
        }
        // PRG-RAM is gated by B003.7.
        uint32_t ram = 0;
        const bool ramOff = !v->mapPrgRam(0x6000, ram, false);
        v->cpuWrite(0xB003, 0xA0, 0);
        const bool ramOn = v->mapPrgRam(0x6000, ram, false) && ram == 0;
        return romNt && ciramNt && ramOff && ramOn;
    };
    const bool vrc6Ppu24 = checkVrc6Ppu(24);
    const bool vrc6Ppu26 = checkVrc6Ppu(26);
    std::printf("vrc6_ppu_banking=%s m24=%s m26=%s\n",
        (vrc6Ppu24 && vrc6Ppu26) ? "PASS" : "FAIL",
        vrc6Ppu24 ? "PASS" : "FAIL", vrc6Ppu26 ? "PASS" : "FAIL");
    ok &= vrc6Ppu24 && vrc6Ppu26;

    // MMC5 CHR set selection: in 8x8 sprite mode set A ($5120-$5127)
    // supplies normal rendering regardless of a later set-B write. In 8x16
    // mode sprites use A, backgrounds use B, and CPU $2007 accesses follow
    // whichever set was written most recently. Also pin the physical PRG-RAM
    // decode instead of modulo-wrapping nonexistent banks.
    auto mmc5Chr = makeMapper(5, 0x40000, 0x40000, 0x2000, 0, 0, true);
    mmc5Chr->cpuWrite(0x5101, 3, 0);
    mmc5Chr->cpuWrite(0x5120, 3, 0);
    mmc5Chr->cpuWrite(0x5128, 7, 0);
    mmc5Chr->observeCpuWrite(0x2000, 0x00);
    uint32_t m5bg8=0,m5cpu8=0;
    mmc5Chr->ppuMapReadEx(0x0000,m5bg8,PpuFetchKind::Background);
    mmc5Chr->ppuMapReadEx(0x0000,m5cpu8,PpuFetchKind::Cpu);
    mmc5Chr->observeCpuWrite(0x2000, 0x20);
    uint32_t m5bg16=0,m5spr16=0,m5cpu16b=0,m5cpu16a=0;
    mmc5Chr->ppuMapReadEx(0x0000,m5bg16,PpuFetchKind::Background);
    mmc5Chr->ppuMapReadEx(0x0000,m5spr16,PpuFetchKind::Sprite);
    mmc5Chr->ppuMapReadEx(0x0000,m5cpu16b,PpuFetchKind::Cpu);
    mmc5Chr->cpuWrite(0x5120, 4, 0);
    mmc5Chr->ppuMapReadEx(0x0000,m5cpu16a,PpuFetchKind::Cpu);
    const bool mmc5ChrSets = m5bg8==0x0C00 && m5cpu8==0x0C00 &&
        m5bg16==0x1C00 && m5spr16==0x0C00 && m5cpu16b==0x1C00 && m5cpu16a==0x1000;

    auto checkMmc5Ram = [&](size_t bytes, uint8_t bank, bool expect, uint32_t expected) {
        auto m = makeMapper(5, 0x40000, 0x2000, bytes, 0, 0, true);
        m->cpuWrite(0x5113, bank, 0); uint32_t mapped=0;
        const bool driven=m->mapPrgRam(0x6000,mapped,false);
        return driven==expect && (!expect || mapped==expected);
    };
    const bool mmc5RamDecode =
        checkMmc5Ram(0x2000,0x03,true,0x0000) && checkMmc5Ram(0x2000,0x04,false,0) &&
        checkMmc5Ram(0x4000,0x03,true,0x0000) && checkMmc5Ram(0x4000,0x04,true,0x2000) &&
        checkMmc5Ram(0x8000,0x03,true,0x6000) && checkMmc5Ram(0x8000,0x04,false,0) &&
        checkMmc5Ram(0x10000,0x07,true,0xE000) &&
        checkMmc5Ram(0x20000,0x0F,true,0x1E000);
    std::printf("mmc5_chr_sets=%s ram_decode=%s\n",
        mmc5ChrSets ? "PASS" : "FAIL", mmc5RamDecode ? "PASS" : "FAIL");
    ok &= mmc5ChrSets && mmc5RamDecode;

    // Mapper 215 / UNL-8237: selectable MMC3 address/index scrambling,
    // NROM override, and two distinct NES 2.0 outer-bank wirings.
    auto m215 = makeMapper(215, 0x100000, 0x100000, 0x2000, 0, 0, true);
    m215->cpuWrite(0x5001, 0x00, 0);
    m215->cpuWrite(0x8000, 0x06, 0);
    m215->cpuWrite(0x8001, 0x02, 0);
    uint32_t m215Base=0; m215->cpuMapRead(0x8000, m215Base);
    m215->cpuWrite(0x5000, 0x83, 0); // NROM-128 override, 16 KiB bank 3
    uint32_t m215N0=0,m215N1=0; m215->cpuMapRead(0x8000,m215N0); m215->cpuMapRead(0xC000,m215N1);
    m215->cpuWrite(0x5000, 0x00, 0);
    m215->cpuWrite(0x5007, 0x01, 0);
    m215->cpuWrite(0xA000, 0x06, 0); // scramble 1: real $8000, index 6 -> 4
    m215->cpuWrite(0xC000, 0x12, 0); // scramble 1: real $8001
    uint32_t m215Chr=0; m215->ppuMapRead(0x1800,m215Chr);
    m215->cpuWrite(0x5001, 0x00, 0);
    m215->reset(false); // outer register alone returns to power-up $xF
    uint32_t m215Reset=0; m215->cpuMapRead(0x8000,m215Reset);
    const bool mapper215s0 = m215Base == 0x04000 && m215N0 == 0x0C000 && m215N1 == 0x0C000 &&
        m215Chr == 0x04800 && m215Reset >= 0xC0000;

    auto m215a = makeMapper(215, 0x400000, 0x400000, 0x2000, 0, 1, true);
    m215a->cpuWrite(0x5001, 0x3B, 0);
    m215a->cpuWrite(0x5000, 0x40, 0); // replace inner A17 from outer register
    m215a->cpuWrite(0x8000, 0x06, 0); m215a->cpuWrite(0x8001, 0x02, 0);
    m215a->cpuWrite(0x8000, 0x02, 0); m215a->cpuWrite(0x8001, 0x04, 0);
    uint32_t m215aPrg=0,m215aChr=0; m215a->cpuMapRead(0x8000,m215aPrg); m215a->ppuMapRead(0x1000,m215aChr);
    const bool mapper215s1 = m215aPrg == 0x1E4000 && m215aChr == 0x161000;
    const bool mapper215Gate = mapperImplementationSupported(215,0) && mapperImplementationSupported(215,1) && !mapperImplementationSupported(215,2);
    std::printf("mapper215_8237=%s sub1=%s gate=%s base=%06X nrom=%05X/%05X chr=%06X\n",
        mapper215s0?"PASS":"FAIL", mapper215s1?"PASS":"FAIL", mapper215Gate?"PASS":"FAIL",
        m215Base,m215N0,m215N1,m215Chr);
    ok &= mapper215s0 && mapper215s1 && mapper215Gate;

    // Phase 25: older FDS-conversion/discrete boards.
    // Mapper 40 uses fixed banks 6/4/5/?/7 and a 4096-M2 one-shot IRQ.
    auto m40 = makeMapper(40, 0x10000, 0x2000, 0, 0, 0, true);
    uint32_t m40_6=0,m40_4=0,m40_5=0,m40_sw=0,m40_7=0;
    m40->cpuMapRead(0x6000,m40_6); m40->cpuMapRead(0x8000,m40_4);
    m40->cpuMapRead(0xA000,m40_5); m40->cpuWrite(0xE000,3,0);
    m40->cpuMapRead(0xC000,m40_sw); m40->cpuMapRead(0xE000,m40_7);
    m40->cpuWrite(0xA000,0,0); for(int i=0;i<4095;i++)m40->clockCpu();
    const bool m40Before=!m40->irqActive(); m40->clockCpu(); const bool m40Edge=m40->irqActive();
    m40->cpuWrite(0x8000,0,0); const bool m40Ack=!m40->irqActive();
    const bool mapper40base=m40_6==0xC000&&m40_4==0x8000&&m40_5==0xA000&&m40_sw==0x6000&&m40_7==0xE000&&m40Before&&m40Edge&&m40Ack&&mapperImplementationSupported(40,0);

    // NES 2.0 mapper 40 submapper 1 / NTDEC 2752 adds an address-latched
    // outer register at $C000-$DFFF: ...ppNPCCM. P=0 retains the 2722
    // SMB2J map. P=1 raises PRG A16 and selects regular NROM-128/256 mode.
    auto m40s1 = makeMapper(40, 0x20000, 0x8000, 0, 0, 1, true);
    m40s1->cpuWrite(0xC006,0,0); // P=0, CC=3, vertical
    uint32_t m40s1Smb=0,m40s1Chr=0; m40s1->cpuMapRead(0x8000,m40s1Smb); m40s1->ppuMapRead(0,m40s1Chr);
    const bool m40s1SmbOk = m40s1Smb==0x8000 && m40s1Chr==0x6000 && m40s1->mirroring()==Mirror::Vertical;

    m40s1->cpuWrite(0xC049,0,0); // pp=2, N=0, P=1, horizontal: NROM-128
    uint32_t m40n128a=0,m40n128b=0,m40n128c=0,m40n128d=0;
    m40s1->cpuMapRead(0x8000,m40n128a); m40s1->cpuMapRead(0xA000,m40n128b);
    m40s1->cpuMapRead(0xC000,m40n128c); m40s1->cpuMapRead(0xE000,m40n128d);
    const bool m40s1N128 = m40n128a==0x18000 && m40n128b==0x1A000 &&
        m40n128c==0x18000 && m40n128d==0x1A000 && m40s1->mirroring()==Mirror::Horizontal;

    m40s1->cpuWrite(0xC058,0,0); // pp=2, N=1, P=1: NROM-256
    uint32_t m40n256a=0,m40n256b=0,m40n256c=0,m40n256d=0;
    m40s1->cpuMapRead(0x8000,m40n256a); m40s1->cpuMapRead(0xA000,m40n256b);
    m40s1->cpuMapRead(0xC000,m40n256c); m40s1->cpuMapRead(0xE000,m40n256d);
    const bool m40s1N256 = m40n256a==0x18000 && m40n256b==0x1A000 &&
        m40n256c==0x1C000 && m40n256d==0x1E000;

    std::vector<uint8_t> m40state; m40s1->saveState(m40state);
    m40s1->cpuWrite(0xC008,0,0);
    const uint8_t* m40sp=m40state.data(); const uint8_t* m40se=m40sp+m40state.size();
    const bool m40Load=m40s1->loadState(m40sp,m40se); uint32_t m40Restored=0; m40s1->cpuMapRead(0xC000,m40Restored);
    m40s1->reset(true); uint32_t m40Hard=0; m40s1->cpuMapRead(0x8000,m40Hard);
    const bool mapper40s1 = m40s1SmbOk && m40s1N128 && m40s1N256 && m40Load &&
        m40Restored==0x1C000 && m40Hard==0x8000 && mapperImplementationSupported(40,1) &&
        !mapperImplementationSupported(40,2);
    const bool mapper40 = mapper40base && mapper40s1;

    // Mapper 41 / Caltron 6-in-1: address selects PRG, outer CHR and mirroring;
    // inner CHR writes become active only when address bit 2 enables them.
    auto m41=makeMapper(41,0x40000,0x20000,0,0);
    m41->cpuWrite(0x602D,0,0); // P=1, E=1, outer CHR=1, horizontal
    m41->cpuWrite(0x8000,2,0);
    uint32_t m41p=0,m41c=0;m41->cpuMapRead(0x8000,m41p);m41->ppuMapRead(0,m41c);
    m41->reset(false);uint32_t m41r=0;m41->cpuMapRead(0x8000,m41r);
    const bool mapper41=m41p==0x8000&&m41c==0xC000&&m41->hasBusConflicts()&&m41r==0&&mapperImplementationSupported(41,0);

    // Mapper 42: switchable $6000 PRG, fixed last 32 KiB, CHR/mirroring and
    // a 15-bit free-running IRQ whose output is high for counter $6000-$7FFF.
    auto m42=makeMapper(42,0x20000,0x10000,0,0);
    m42->cpuWrite(0xE000,3,0);m42->cpuWrite(0x8000,5,0);m42->cpuWrite(0xE001,8,0);
    uint32_t m42p=0,m42f=0,m42c=0;m42->cpuMapRead(0x6000,m42p);m42->cpuMapRead(0x8000,m42f);m42->ppuMapRead(0,m42c);
    m42->cpuWrite(0xE002,2,0);for(int i=0;i<0x5FFF;i++)m42->clockCpu();const bool m42Before=!m42->irqActive();m42->clockCpu();const bool m42Irq=m42->irqActive();m42->cpuWrite(0xE002,0,0);
    const bool mapper42=m42p==0x6000&&m42f==0x18000&&m42c==0xA000&&m42->mirroring()==Mirror::Horizontal&&m42Before&&m42Irq&&!m42->irqActive()&&mapperImplementationSupported(42,0);

    // Mapper 43: 80 KiB SMB2J layout, including the 2 KiB ROM visible in
    // expansion space, LUT-based $C000 banking, and 4096-cycle overflow IRQ.
    auto m43=makeMapper(43,0x14000,0x2000,0,0);
    uint32_t m43x=0,m43_6=0,m43_8=0,m43_a=0,m43_c=0,m43_e=0;
    m43->cpuMapRead(0x5000,m43x);m43->cpuMapRead(0x6000,m43_6);m43->cpuMapRead(0x8000,m43_8);m43->cpuMapRead(0xA000,m43_a);
    m43->cpuWrite(0x4022,5,0);m43->cpuMapRead(0xC000,m43_c);m43->cpuMapRead(0xE000,m43_e);
    m43->cpuWrite(0x4122,1,0);for(int i=0;i<4095;i++)m43->clockCpu();const bool m43Before=!m43->irqActive();m43->clockCpu();const bool m43Irq=m43->irqActive();m43->cpuWrite(0x8122,0,0);
    const bool mapper43=m43x==0x10000&&m43_6==0x4000&&m43_8==0x2000&&m43_a==0&&m43_c==0xE000&&m43_e==0x12000&&m43Before&&m43Irq&&!m43->irqActive()&&mapperImplementationSupported(43,0);
    std::printf("legacy_40_43 m40=%s m40.1=%s m41=%s m42=%s m43=%s\n",mapper40?"PASS":"FAIL",mapper40s1?"PASS":"FAIL",mapper41?"PASS":"FAIL",mapper42?"PASS":"FAIL",mapper43?"PASS":"FAIL");
    ok &= mapper40 && mapper41 && mapper42 && mapper43;

    // Phase 26: Action 53, TXC 01-22000-400, Super Big 7-in-1, and
    // Rumblestation 15-in-1.
    auto m28=makeMapper(28,0x800000,0x8000,0,0x8000);
    m28->cpuWrite(0x5000,0x80,0); m28->cpuWrite(0x8000,0x2E,0); // 128K outer, UNROM/fixed $C000, vertical
    m28->cpuWrite(0x5000,0x81,0); m28->cpuWrite(0x8000,0x12,0);
    m28->cpuWrite(0x5000,0x01,0); m28->cpuWrite(0x8000,0x02,0);
    m28->cpuWrite(0x5000,0x00,0); m28->cpuWrite(0x8000,0x03,0);
    uint32_t m28lo=0,m28hi=0,m28chr=0;m28->cpuMapRead(0x8000,m28lo);m28->cpuMapRead(0xC000,m28hi);m28->ppuMapRead(0,m28chr);
    const bool mapper28=m28lo==0x88000&&m28hi==0x94000&&m28chr==0x6000&&m28->mirroring()==Mirror::Vertical&&mapperImplementationSupported(28,0);

    auto m36=makeMapper(36,0x20000,0x8000,0,0);
    m36->cpuWrite(0x4102,0x20,0);m36->cpuWrite(0x4100,0,0);m36->cpuWrite(0x8000,0,0);m36->cpuWrite(0x4200,3,0);
    uint32_t m36p=0,m36c=0;m36->cpuMapRead(0x8000,m36p);m36->ppuMapRead(0,m36c);
    m36->cpuWrite(0x4103,0x10,0);m36->cpuWrite(0x4100,0,0);m36->cpuWrite(0x8000,0,0);uint32_t m36inc=0;m36->cpuMapRead(0x8000,m36inc);
    const bool mapper36=m36p==0x10000&&m36c==0x6000&&m36inc==0x18000&&mapperImplementationSupported(36,0);

    auto m44=makeMapper(44,0x100000,0x100000,0x2000,0);
    m44->cpuWrite(0x8000,0x06,0);m44->cpuWrite(0x8001,0x03,0);
    m44->cpuWrite(0x8000,0x02,0);m44->cpuWrite(0x8001,0x05,0);
    m44->cpuWrite(0xA001,0x82,0); // RAM enabled + block 2
    uint32_t m44p=0,m44c=0;m44->cpuMapRead(0x8000,m44p);m44->ppuMapRead(0x1000,m44c);
    m44->cpuWrite(0xA001,0x87,0);uint32_t m44b7=0;m44->cpuMapRead(0x8000,m44b7);
    const bool mapper44=m44p==0x46000&&m44c==0x41400&&m44b7==0xC6000&&mapperImplementationSupported(44,0);

    auto m46=makeMapper(46,0x100000,0x100000,0,0);
    m46->cpuWrite(0x6000,0x9A,0);m46->cpuWrite(0x8000,0x51,0);
    uint32_t m46p=0,m46c=0;m46->cpuMapRead(0x8000,m46p);m46->ppuMapRead(0,m46c);
    const bool mapper46=m46p==0xA8000&&m46c==0x9A000&&mapperImplementationSupported(46,0);
    std::printf("phase26_mappers m28=%s m36=%s m44=%s m46=%s\n",mapper28?"PASS":"FAIL",mapper36?"PASS":"FAIL",mapper44?"PASS":"FAIL",mapper46?"PASS":"FAIL");
    ok &= mapper28&&mapper36&&mapper44&&mapper46;


    // Phase 27: legacy discrete/multicart/FDS-conversion gaps.
    auto m38=makeMapper(38,0x20000,0x8000,0,0);
    m38->cpuWrite(0x7000,0x0B,0); uint32_t m38p=0,m38c=0; m38->cpuMapRead(0x8000,m38p); m38->ppuMapRead(0,m38c);
    const bool mapper38=m38p==0x18000&&m38c==0x4000&&mapperImplementationSupported(38,0);

    auto m39=makeMapper(39,0x40000,0x2000,0,0);
    m39->cpuWrite(0x8000,5,0); uint32_t m39p=0;m39->cpuMapRead(0x8000,m39p);m39->reset(false);uint32_t m39r=1;m39->cpuMapRead(0x8000,m39r);
    const bool mapper39=m39p==0x28000&&m39r==0&&mapperImplementationSupported(39,0);

    auto m50=makeMapper(50,0x20000,0x2000,0,0);
    m50->cpuWrite(0x4020,0x0B,0);uint32_t m50_6=0,m50_8=0,m50_a=0,m50_c=0,m50_e=0;
    m50->cpuMapRead(0x6000,m50_6);m50->cpuMapRead(0x8000,m50_8);m50->cpuMapRead(0xA000,m50_a);m50->cpuMapRead(0xC000,m50_c);m50->cpuMapRead(0xE000,m50_e);
    m50->cpuWrite(0x4120,1,0);for(int i=0;i<4095;i++)m50->clockCpu();bool m50before=!m50->irqActive();m50->clockCpu();bool m50irq=m50->irqActive();m50->cpuWrite(0x4120,0,0);
    const bool mapper50=m50_6==0x1E000&&m50_8==0x10000&&m50_a==0x12000&&m50_c==0x1A000&&m50_e==0x16000&&m50before&&m50irq&&!m50->irqActive()&&mapperImplementationSupported(50,0);

    auto m58=makeMapper(58,0x40000,0x10000,0,0);
    m58->cpuWrite(0x80D3,0,0);uint32_t m58lo=0,m58hi=0,m58c=0;m58->cpuMapRead(0x8000,m58lo);m58->cpuMapRead(0xC000,m58hi);m58->ppuMapRead(0,m58c);
    const bool mapper58=m58lo==0xC000&&m58hi==0xC000&&m58c==0x4000&&m58->mirroring()==Mirror::Horizontal&&mapperImplementationSupported(58,0);

    auto m60=makeMapper(60,0x10000,0x8000,0,0);uint32_t m60p0=0,m60p1=0,m60c=0;m60->cpuMapRead(0x8000,m60p0);m60->reset(false);m60->cpuMapRead(0x8000,m60p1);m60->ppuMapRead(0,m60c);m60->reset(true);uint32_t m60hard=1;m60->cpuMapRead(0x8000,m60hard);
    const bool mapper60=m60p0==0&&m60p1==0x4000&&m60c==0x2000&&m60hard==0&&mapperImplementationSupported(60,0);

    auto m62=makeMapper(62,0x200000,0x100000,0,0);
    m62->cpuWrite(0xA5E1,2,0);uint32_t m62lo=0,m62hi=0,m62c=0;m62->cpuMapRead(0x8000,m62lo);m62->cpuMapRead(0xC000,m62hi);m62->ppuMapRead(0,m62c);
    const bool mapper62=m62lo==0x194000&&m62hi==0x194000&&m62c==0xC000&&m62->mirroring()==Mirror::Horizontal&&mapperImplementationSupported(62,0);
    std::printf("phase27_mappers m38=%s m39=%s m50=%s m58=%s m60=%s m62=%s\n",mapper38?"PASS":"FAIL",mapper39?"PASS":"FAIL",mapper50?"PASS":"FAIL",mapper58?"PASS":"FAIL",mapper60?"PASS":"FAIL",mapper62?"PASS":"FAIL");
    ok &= mapper38&&mapper39&&mapper50&&mapper58&&mapper60&&mapper62;


    // Phase 28: mapper 51/52/61/63 multicart gap closure.
    auto m51=makeMapper(51,0x200000,0x2000,0,0);
    m51->reset(true);m51->cpuWrite(0x8000,3,0);uint32_t m51p=0,m51x=0;m51->cpuMapRead(0x8000,m51p);m51->cpuMapRead(0x6000,m51x);
    m51->cpuWrite(0x6000,0x12,0);uint32_t m51lo=0,m51hi=0;m51->cpuMapRead(0x8000,m51lo);m51->cpuMapRead(0xC000,m51hi);
    const bool mapper51=m51p==0x18000&&m51x==0x5E000&&m51lo==0x18000&&m51hi==0x1C000&&m51->mirroring()==Mirror::Horizontal&&mapperImplementationSupported(51,0);

    auto m52=makeMapper(52,0x100000,0x100000,0x2000,0);
    m52->cpuWrite(0x8000,0x06,0);m52->cpuWrite(0x8001,0x02,0);
    m52->cpuWrite(0x8000,0x02,0);m52->cpuWrite(0x8001,0x05,0);
    m52->cpuWrite(0x6000,0xFF,0);uint32_t m52p=0,m52c=0;m52->cpuMapRead(0x8000,m52p);m52->ppuMapRead(0x1000,m52c);
    m52->cpuWrite(0x6000,0x00,0);uint32_t m52locked=0;m52->cpuMapRead(0x8000,m52locked);
    m52->reset(false);uint32_t m52reset=0;m52->cpuMapRead(0x8000,m52reset);
    const bool mapper52=m52p==0xE4000&&m52c==0xE1400&&m52locked==m52p&&m52reset==0x04000&&mapperImplementationSupported(52,0);

    auto m61=makeMapper(61,0x100000,0x2000,0,0);
    m61->cpuWrite(0x8003,0,0);uint32_t m61_32=0;m61->cpuMapRead(0x8000,m61_32);
    m61->cpuWrite(0x8092,0,0);uint32_t m61lo=0,m61hi=0;m61->cpuMapRead(0x8000,m61lo);m61->cpuMapRead(0xC000,m61hi);
    const bool mapper61=m61_32==0x18000&&m61lo==0x10000&&m61hi==0x10000&&m61->mirroring()==Mirror::Horizontal&&mapperImplementationSupported(61,0);

    // Mapper 63: TH2291-3 (submapper 0) and 82AB (submapper 1) latch the
    // CPU write address. P selects NROM-128 vs NROM-256, b supplies A14 in
    // NROM-128 mode, C write-protects the unbanked CHR-RAM, and unpopulated
    // PRG selections must leave the CPU bus undriven rather than wrap.
    auto m63=makeMapper(63,0x400000,0,0,0x2000,0,true);
    m63->cpuWrite(0x8004,0,0); // NROM-128, b=1 -> 16 KiB bank 1 mirrored
    uint32_t m63lo=0,m63hi=0;
    const bool m63loOk=m63->cpuMapRead(0x8000,m63lo);
    const bool m63hiOk=m63->cpuMapRead(0xC000,m63hi);
    m63->cpuWrite(0x8002,0,0); // NROM-256, CPU A14 selects banks 0/1
    uint32_t m63n0=0,m63n1=0;
    const bool m63n0Ok=m63->cpuMapRead(0x8000,m63n0);
    const bool m63n1Ok=m63->cpuMapRead(0xC000,m63n1);
    m63->cpuWrite(0x8401,0,0); // submapper 0: A10 protects CHR, A0=H mirror
    uint32_t m63chr=0;
    const bool m63ChrRead=m63->ppuMapRead(0x1234,m63chr);
    const bool m63ChrProtected=!m63->ppuMapWrite(0x1234,m63chr);
    const bool m63sm0=m63loOk&&m63hiOk&&m63lo==0x4000&&m63hi==0x4000&&
        m63n0Ok&&m63n1Ok&&m63n0==0&&m63n1==0x4000&&m63ChrRead&&
        m63ChrProtected&&m63->mirroring()==Mirror::Horizontal;

    auto m63s1=makeMapper(63,0x200000,0,0,0x2000,1,true);
    m63s1->cpuWrite(0x8200,0,0); // submapper 1: A9 is CHR /WE protect
    uint32_t m63s1chr=0;
    const bool m63s1Protected=!m63s1->ppuMapWrite(0x0042,m63s1chr);
    m63s1->cpuWrite(0x8008,0,0); // outer bank 1, NROM-128, b=0 => bank 2
    uint32_t m63s1p=0;
    const bool m63s1Read=m63s1->cpuMapRead(0x8000,m63s1p);

    auto m63open=makeMapper(63,0x8000,0,0,0x2000,0,true);
    m63open->cpuWrite(0x8010,0,0); // select 16 KiB bank 4 on a 32 KiB ROM
    uint32_t m63unused=0;
    const bool m63Undriven=!m63open->cpuMapRead(0x8000,m63unused);
    uint8_t fake=0;
    const bool m63NoFakeRegister=!m63open->cpuReadRegister(0x8000,fake);
    const bool m63Gate=mapperImplementationSupported(63,0)&&mapperImplementationSupported(63,1)&&
        !mapperImplementationSupported(63,2);
    const bool mapper63=m63sm0&&m63s1Protected&&m63s1Read&&m63s1p==0x8000&&
        m63Undriven&&m63NoFakeRegister&&m63Gate;
    std::printf("phase28_mappers m51=%s [%05X %05X %05X %05X mir=%d] m52=%s m61=%s m63=%s\n",mapper51?"PASS":"FAIL",m51p,m51x,m51lo,m51hi,int(m51->mirroring()),mapper52?"PASS":"FAIL",mapper61?"PASS":"FAIL",mapper63?"PASS":"FAIL");
    std::printf("mapper63_latch sm0=%s sm1=%s openbus=%s gate=%s\n",
        m63sm0?"PASS":"FAIL",(m63s1Protected&&m63s1Read&&m63s1p==0x8000)?"PASS":"FAIL",
        (m63Undriven&&m63NoFakeRegister)?"PASS":"FAIL",m63Gate?"PASS":"FAIL");
    ok &= mapper51&&mapper52&&mapper61&&mapper63;



    // Phase 29: legacy mapper gaps 54/55/56/57/59.
    auto m54=makeMapper(54,0x20000,0x10000,0,0);
    m54->cpuWrite(0x8005,0,0); uint32_t m54p=0,m54c=0; m54->cpuMapRead(0x8000,m54p); m54->ppuMapRead(0,m54c);
    const bool mapper54=m54p==0x8000&&m54c==0xA000&&mapperImplementationSupported(54,0);

    auto m55=makeMapper(55,0xC000,0x2000,0x800,0);
    uint32_t m55x0=0,m55x1=0,m55p=0,m55r0=0,m55r1=0;
    const bool m55xok=m55->cpuMapRead(0x6000,m55x0)&&m55->cpuMapRead(0x6800,m55x1)&&m55->cpuMapRead(0x8000,m55p);
    const bool m55rok=m55->mapPrgRam(0x7000,m55r0,false)&&m55->mapPrgRam(0x7800,m55r1,false);
    const bool mapper55=m55xok&&m55rok&&m55x0==0x8000&&m55x1==0x8000&&m55p==0&&m55r0==0&&m55r1==0&&mapperImplementationSupported(55,0);

    auto m56=makeMapper(56,0x40000,0x20000,0x2000,0);
    m56->cpuWrite(0xE000,1,0);m56->cpuWrite(0xF000,3,0); // PRG0 low nibble=3
    m56->cpuWrite(0xE000,0,0); // no generic $Fxxx bank register selected
    m56->cpuWrite(0xF803,1,0); // vertical mirroring
    m56->cpuWrite(0xFC03,5,0); // CHR slot 3 = bank 5
    uint32_t m56p=0,m56c=0;m56->cpuMapRead(0x8000,m56p);m56->ppuMapRead(0x0C00,m56c);
    // IRQ reload FFFD, then two clocks reach FFFF and assert.
    m56->cpuWrite(0x8000,0xD,0);m56->cpuWrite(0x9000,0xF,0);m56->cpuWrite(0xA000,0xF,0);m56->cpuWrite(0xB000,0xF,0);m56->cpuWrite(0xC000,2,0);
    m56->clockCpu();const bool m56Before=!m56->irqActive();m56->clockCpu();const bool m56Irq=m56->irqActive();m56->cpuWrite(0xD000,0,0);
    m56->cpuWrite(0xE000,5,0);m56->cpuWrite(0xF000,4,0);uint32_t m56x=0;const bool m56rom=m56->cpuMapRead(0x6000,m56x);
    const bool mapper56=m56p==0x6000&&m56c==0x1400&&m56->mirroring()==Mirror::Vertical&&m56Before&&m56Irq&&!m56->irqActive()&&m56rom&&mapperImplementationSupported(56,0);

    auto m57=makeMapper(57,0x20000,0x20000,0x2000,0);
    m57->cpuWrite(0x8800,0xB8,0);m57->cpuWrite(0x8000,0x43,0);uint32_t m57lo=0,m57hi=0,m57c=0;m57->cpuMapRead(0x8000,m57lo);m57->cpuMapRead(0xC000,m57hi);m57->ppuMapRead(0,m57c);uint8_t m57dip=0xFF;const bool m57read=m57->cpuReadRegister(0x6000,m57dip);
    const bool mapper57=m57lo==0x10000&&m57hi==0x14000&&m57c==0x16000&&m57->mirroring()==Mirror::Horizontal&&m57read&&m57dip==0&&mapperImplementationSupported(57,0);

    auto m59=makeMapper(59,0x40000,0x10000,0,0);
    m59->cpuWrite(0x81D9,0,0);uint32_t m59lo=0,m59hi=0,m59c=0;m59->cpuMapRead(0x8000,m59lo);m59->cpuMapRead(0xC000,m59hi);m59->ppuMapRead(0,m59c);uint8_t m59dip=0xFF;const bool m59read=m59->cpuReadRegister(0x8000,m59dip);
    const bool mapper59=m59lo==0x14000&&m59hi==0x14000&&m59c==0x2000&&m59->mirroring()==Mirror::Horizontal&&m59read&&m59dip==0&&mapperImplementationSupported(59,0);
    const bool mapper53Gate=mapperImplementationSupported(53,0);
    std::printf("phase29_mappers m54=%s m55=%s m56=%s[p=%05X c=%05X mir=%d before=%d irq=%d ack=%d rom=%d x=%05X] m57=%s m59=%s m53_gate=%s\n",mapper54?"PASS":"FAIL",mapper55?"PASS":"FAIL",mapper56?"PASS":"FAIL",m56p,m56c,int(m56->mirroring()),int(m56Before),int(m56Irq),int(!m56->irqActive()),int(m56rom),m56x,mapper57?"PASS":"FAIL",mapper59?"PASS":"FAIL",mapper53Gate?"PASS":"FAIL");
    ok &= mapper54&&mapper55&&mapper56&&mapper57&&mapper59&&mapper53Gate;

    // Phase 30: mapper 53 supports both physical PRG dump orderings. The
    // hash database resolves boardVariant=1 for the EPROM-first layout.
    auto m53normal=makeMapper(53,0x220000,0x2000,0,0,0,false,Mirror::Vertical,false,0);
    auto m53eprom=makeMapper(53,0x220000,0x2000,0,0,0,false,Mirror::Vertical,false,1);
    uint32_t m53n=0,m53e=0,m53x=0; m53normal->cpuMapRead(0x8000,m53n); m53eprom->cpuMapRead(0x8000,m53e); m53eprom->cpuMapRead(0x6000,m53x);
    m53eprom->cpuWrite(0x6000,0x30,0); m53eprom->cpuWrite(0x8000,3,0); uint32_t m53sw=0; m53eprom->cpuMapRead(0x8000,m53sw);
    const bool mapper53=m53n==0x200000&&m53e==0&&m53x==0x26000&&m53sw==0x14000&&m53eprom->mirroring()==Mirror::Horizontal;
    std::printf("mapper53_supervision=%s normal=%06X eprom=%06X x=%06X switched=%06X\n",mapper53?"PASS":"FAIL",m53n,m53e,m53x,m53sw);
    ok &= mapper53;

    // Phase 32: legacy mapper gaps 81/103/107.
    auto m81=makeMapper(81,0x10000,0x8000,0,0);
    m81->cpuWrite(0x800B,0x00,0);uint32_t m81lo=0,m81hi=0,m81c=0;
    m81->cpuMapRead(0x8000,m81lo);m81->cpuMapRead(0xC000,m81hi);m81->ppuMapRead(0,m81c);
    const bool mapper81=m81lo==0x8000&&m81hi==0xC000&&m81c==0x6000&&mapperImplementationSupported(81,0);

    auto m103=makeMapper(103,0x22000,0x2000,0x2000,0);
    uint32_t m103r=0,m103ram0=0,m103ram1=0,m103fixed=0;
    const bool m103ramRead=m103->mapPrgRam(0x6000,m103ram0,false)&&m103->mapPrgRam(0xB800,m103ram1,false);
    m103->cpuWrite(0x8000,3,0);m103->cpuWrite(0xF000,0x10,0);
    const bool m103rom=m103->cpuMapRead(0x6000,m103r)&&m103->cpuMapRead(0xB800,m103fixed);
    uint32_t m103wr=0;const bool m103writeRam=m103->mapPrgRam(0x6000,m103wr,true);
    m103->cpuWrite(0xE000,8,0);
    const bool mapper103=m103ramRead&&m103ram0==0&&m103ram1==0&&m103rom&&m103r==0x6000&&m103fixed==0x1B800&&m103writeRam&&m103->mirroring()==Mirror::Horizontal&&mapperImplementationSupported(103,0);

    auto m107=makeMapper(107,0x20000,0x10000,0,0);
    m107->cpuWrite(0x8000,0x0D,0);uint32_t m107p=0,m107c=0;m107->cpuMapRead(0x8000,m107p);m107->ppuMapRead(0,m107c);
    const bool mapper107=m107p==0x10000&&m107c==0xA000&&mapperImplementationSupported(107,0);
    std::printf("phase32_mappers m81=%s m103=%s m107=%s\n",mapper81?"PASS":"FAIL",mapper103?"PASS":"FAIL",mapper107?"PASS":"FAIL");
    ok &= mapper81&&mapper103&&mapper107;


    // Phase 33: mapper 106/108/112 gap closure.
    auto m106=makeMapper(106,0x40000,0x20000,0x2000,0);
    m106->cpuWrite(0x8008,0x03,0);m106->cpuWrite(0x8009,0x05,0);m106->cpuWrite(0x800A,0x06,0);m106->cpuWrite(0x800B,0x02,0);
    m106->cpuWrite(0x8000,0x7F,0);m106->cpuWrite(0x8001,0x7E,0);m106->cpuWrite(0x8004,0x85,0);m106->cpuWrite(0x800C,1,0);
    uint32_t m106p0=0,m106p1=0,m106p2=0,m106p3=0,m106c0=0,m106c1=0,m106c4=0;
    m106->cpuMapRead(0x8000,m106p0);m106->cpuMapRead(0xA000,m106p1);m106->cpuMapRead(0xC000,m106p2);m106->cpuMapRead(0xE000,m106p3);
    m106->ppuMapRead(0x0000,m106c0);m106->ppuMapRead(0x0400,m106c1);m106->ppuMapRead(0x1000,m106c4);
    m106->cpuWrite(0x800E,0xFD,0);m106->cpuWrite(0x800F,0xFF,0);m106->clockCpu();const bool m106before=!m106->irqActive();m106->clockCpu();const bool m106irq=m106->irqActive();m106->cpuWrite(0x800D,0,0);
    const bool mapper106=m106p0==0x26000&&m106p1==0x0A000&&m106p2==0x0C000&&m106p3==0x24000&&
        m106c0==0x1F800&&m106c1==0x1FC00&&m106c4==0x01400&&m106->mirroring()==Mirror::Horizontal&&m106before&&m106irq&&!m106->irqActive()&&mapperImplementationSupported(106,0);

    auto m108s1=makeMapper(108,0x20000,0,0,0x2000,1,true,Mirror::Horizontal);
    m108s1->cpuWrite(0xE000,3,0);uint32_t m108ignore=0;m108s1->cpuMapRead(0x6000,m108ignore);
    m108s1->cpuWrite(0xF000,3,0);uint32_t m108p1=0,m108tail=0,m108c1=0;m108s1->cpuMapRead(0x6000,m108p1);m108s1->cpuMapRead(0x8000,m108tail);m108s1->ppuMapRead(0,m108c1);
    auto m108s2=makeMapper(108,0x20000,0x8000,0,0,2,true);m108s2->cpuWrite(0xE000,2,0);uint32_t m108p2=0,m108c2=0;m108s2->cpuMapRead(0x6000,m108p2);m108s2->ppuMapRead(0,m108c2);
    auto m108s4=makeMapper(108,0x20000,0x4000,0,0,4,true);m108s4->cpuWrite(0x8000,1,0);uint32_t m108p4=0,m108c4=0;m108s4->cpuMapRead(0x6000,m108p4);m108s4->ppuMapRead(0,m108c4);
    auto m108legacy=makeMapper(108,0x20000,0,0,0x2000,0,false,Mirror::Horizontal);m108legacy->cpuWrite(0xF000,4,0);uint32_t m108legacyP=0;m108legacy->cpuMapRead(0x6000,m108legacyP);
    const bool mapper108=m108ignore==0&&m108p1==0x6000&&m108tail==0x18000&&m108c1==0&&m108p2==0x4000&&m108c2==0x4000&&m108p4==0x1E000&&m108c4==0x2000&&m108legacyP==0x8000&&mapperImplementationSupported(108,0)&&mapperImplementationSupported(108,4)&&!mapperImplementationSupported(108,5);

    auto m112=makeMapper(112,0x40000,0x40000,0,0);
    m112->cpuWrite(0x8000,0,0);m112->cpuWrite(0xA000,3,0);m112->cpuWrite(0x8000,1,0);m112->cpuWrite(0xA000,4,0);
    m112->cpuWrite(0x8000,2,0);m112->cpuWrite(0xA000,6,0);m112->cpuWrite(0x8000,4,0);m112->cpuWrite(0xA000,9,0);m112->cpuWrite(0xE000,1,0);
    uint32_t m112p0=0,m112p1=0,m112p2=0,m112p3=0,m112c0=0,m112c4=0;m112->cpuMapRead(0x8000,m112p0);m112->cpuMapRead(0xA000,m112p1);m112->cpuMapRead(0xC000,m112p2);m112->cpuMapRead(0xE000,m112p3);m112->ppuMapRead(0,m112c0);m112->ppuMapRead(0x1000,m112c4);
    const bool mapper112=m112p0==0x6000&&m112p1==0x8000&&m112p2==0x3C000&&m112p3==0x3E000&&m112c0==0x1800&&m112c4==0x2400&&m112->mirroring()==Mirror::Horizontal&&mapperImplementationSupported(112,0);
    std::printf("phase33_mappers m106=%s m108=%s[%05X %05X %05X %05X %05X %05X %05X legacy=%05X] m112=%s\n",mapper106?"PASS":"FAIL",mapper108?"PASS":"FAIL",m108ignore,m108p1,m108tail,m108p2,m108c2,m108p4,m108c4,m108legacyP,mapper112?"PASS":"FAIL");
    ok &= mapper106&&mapper108&&mapper112;

    // Mapper 105 / NES-EVENT: cold boot hard-wires the first 32 KiB.
    // Starting the event timer through MMC1 CHR0 unlocks event banking.  The
    // official tournament DIP setting produces an IRQ at $28000000 M2 clocks;
    // load a near-edge state so the regression can verify the real 30-bit edge
    // without spending hundreds of millions of host iterations.
    auto m105=makeMapper(105,0x40000,0,0x2000,0x2000);
    uint32_t m105boot=0,m105bank=0;
    m105->cpuMapRead(0x8000,m105boot);
    mmc1Serial(*m105,0xA000,0x06,100); // event bank 3, bit4=0: run/unlock
    m105->cpuMapRead(0x8000,m105bank);
    std::vector<uint8_t> m105state;m105->saveState(m105state);
    const uint64_t nearEdge=0x27FFFFFFull;
    for(int i=0;i<8;++i)m105state[13+i]=uint8_t(nearEdge>>(i*8));
    m105state[21]=1;m105state[22]=0;m105state[23]=1;m105state[24]=4;
    const uint8_t* m105p=m105state.data();const uint8_t* m105e=m105p+m105state.size();
    const bool m105load=m105->loadState(m105p,m105e);const bool m105before=!m105->irqActive();m105->clockCpu();const bool m105irq=m105->irqActive();
    mmc1Serial(*m105,0xA000,0x10,200);const bool m105ack=!m105->irqActive();
    const bool mapper105=m105boot==0&&m105bank==0x18000&&m105load&&m105before&&m105irq&&m105ack&&mapperImplementationSupported(105,0);
    std::printf("phase34_mapper105 event=%s boot=%05X bank=%05X irq=%s\n",mapper105?"PASS":"FAIL",m105boot,m105bank,m105irq?"PASS":"FAIL");
    ok &= mapper105;

    // Phase 35: GTROM / mapper 111.  The mapper owns a writable 512 KiB
    // SST39SF040 image and banks a shared 32 KiB PPU RAM between pattern and
    // nametable regions.  Verify banking, flash byte-program + sector erase,
    // and mapper-owned battery persistence.
    auto m111=makeMapper(111,0x80000,0,0,0x8000);
    std::vector<uint8_t> gtromImage(0x80000,0xFF);
    gtromImage[3*0x8000+0x123]=0xA5;
    m111->initializePrgImage(gtromImage.data(),gtromImage.size());
    m111->cpuWrite(0x5000,0x33,0);
    uint8_t m111read=0;const bool m111bank=m111->cpuReadRegister(0x8123,m111read)&&m111read==0xA5;
    uint32_t m111pat=0,m111nt=0;NametableSource m111src=NametableSource::Ciram;
    const bool m111ppu=m111->ppuMapRead(0x0123,m111pat)&&m111->mapNametable(0x2123,m111src,m111nt)&&m111pat==0x4123&&m111src==NametableSource::ChrRam&&m111nt==0x4123;
    // Byte program: AA/55/A0, select bank 2, then program byte (1->0 only).
    m111->cpuWrite(0xD555,0xAA,0);m111->cpuWrite(0xAAAA,0x55,0);m111->cpuWrite(0xD555,0xA0,0);m111->cpuWrite(0x5000,0x02,0);m111->cpuWrite(0x8123,0x5A,0);
    uint8_t m111prog=0;m111->cpuReadRegister(0x8123,m111prog);
    const bool m111program=m111prog==0x5A;
    // Sector erase the programmed 4 KiB sector.
    m111->cpuWrite(0xD555,0xAA,0);m111->cpuWrite(0xAAAA,0x55,0);m111->cpuWrite(0xD555,0x80,0);m111->cpuWrite(0xD555,0xAA,0);m111->cpuWrite(0xAAAA,0x55,0);m111->cpuWrite(0x5000,0x02,0);m111->cpuWrite(0x8123,0x30,0);
    uint8_t m111erase=0;m111->cpuReadRegister(0x8123,m111erase);
    const bool m111sector=m111erase==0xFF;
    // Program again and confirm mapper-battery roundtrip preserves flash.
    m111->cpuWrite(0xD555,0xAA,0);m111->cpuWrite(0xAAAA,0x55,0);m111->cpuWrite(0xD555,0xA0,0);m111->cpuWrite(0x5000,0x02,0);m111->cpuWrite(0x8123,0x3C,0);
    std::vector<uint8_t> m111bat;m111->saveMapperBattery(m111bat);
    auto m111b=makeMapper(111,0x80000,0,0,0x8000);m111b->initializePrgImage(gtromImage.data(),gtromImage.size());
    const bool m111load=m111b->loadMapperBattery(m111bat.data(),m111bat.size());m111b->cpuWrite(0x5000,0x02,0);uint8_t m111persist=0;m111b->cpuReadRegister(0x8123,m111persist);
    const bool mapper111=m111bank&&m111ppu&&m111program&&m111sector&&m111load&&m111persist==0x3C&&m111->mapperBatterySize()==0x80000&&mapperImplementationSupported(111,0);
    std::printf("phase35_gtrom bank=%s ppu=%s program=%s sector=%s persist=%s\n",m111bank?"PASS":"FAIL",m111ppu?"PASS":"FAIL",m111program?"PASS":"FAIL",m111sector?"PASS":"FAIL",(m111load&&m111persist==0x3C)?"PASS":"FAIL");
    ok &= mapper111;

    // Phase 36: historical mapper-111 collision.  CHR-ROM identifies the
    // Chinese Ninja Ryukenden board, which is a non-serialized MMC1 variant
    // with 256 KiB CHR reach. Writes replace registers immediately.
    auto m111old=makeMapper(111,0x20000,0x40000,0,0);
    uint32_t oldPrg0=0,oldPrg1=0,oldChr0=0,oldChr1=0;
    m111old->cpuWrite(0x8000,0x1F,0); // 4 KiB CHR, PRG mode 3, H mirror
    m111old->cpuWrite(0xA000,0x21,0); // CHR bank 33 -> above normal MMC1 128 KiB limit
    m111old->cpuWrite(0xC000,0x3F,0); // CHR bank 63
    m111old->cpuWrite(0xE000,0x03,0); // PRG bank 3, direct (non-serial) write
    m111old->cpuMapRead(0x8000,oldPrg0); m111old->cpuMapRead(0xC000,oldPrg1);
    m111old->ppuMapRead(0x0000,oldChr0); m111old->ppuMapRead(0x1000,oldChr1);
    const bool m111oldOk=oldPrg0==0x0C000&&oldPrg1==0x1C000&&oldChr0==0x21000&&oldChr1==0x3F000&&
        m111old->mirroring()==Mirror::Horizontal&&m111old->mapperBatterySize()==0;
    std::printf("phase36_mapper111_collision oldmmc1=%s prg=%05X/%05X chr=%05X/%05X gtrom=%s\n",
        m111oldOk?"PASS":"FAIL",oldPrg0,oldPrg1,oldChr0,oldChr1,mapper111?"PASS":"FAIL");
    ok &= m111oldOk;


    // Phase 37: mapper 104/117/120 gap closure.
    auto m104=makeMapper(104,0x140000,0x2000,0,0);
    m104->cpuWrite(0x8000,0x02,0);m104->cpuWrite(0xC000,0x03,0);
    uint32_t m104lo=0,m104hi=0;m104->cpuMapRead(0x8000,m104lo);m104->cpuMapRead(0xC000,m104hi);
    m104->cpuWrite(0x8000,0x0A,0); // outer 2 + lock
    m104->cpuWrite(0x8000,0x01,0); // ignored while locked
    uint32_t m104lock=0;m104->cpuMapRead(0x8000,m104lock);
    m104->reset(false);uint32_t m104soft=0;m104->cpuMapRead(0x8000,m104soft);
    m104->reset(true);uint32_t m104hard=0;m104->cpuMapRead(0x8000,m104hard);
    const bool mapper104=m104lo==0x8C000&&m104hi==0xBC000&&m104lock==0x8C000&&m104soft==0x8C000&&m104hard==0&&mapperImplementationSupported(104,0);

    auto m117=makeMapper(117,0x20000,0x20000,0,0);
    m117->cpuWrite(0x8000,2,0);m117->cpuWrite(0x8001,3,0);m117->cpuWrite(0xA005,7,0);m117->cpuWrite(0xD000,1,0);
    uint32_t m117p0=0,m117p1=0,m117c=0;m117->cpuMapRead(0x8000,m117p0);m117->cpuMapRead(0xA000,m117p1);m117->ppuMapRead(0x1400,m117c);
    m117->cpuWrite(0xC001,2,0);m117->cpuWrite(0xC003,0,0);m117->cpuWrite(0xE000,1,0);
    m117->notifyPpuAddress(0x0000,0);m117->notifyPpuAddress(0x1000,8);const bool m117before=!m117->irqActive();
    m117->notifyPpuAddress(0x0000,9);m117->notifyPpuAddress(0x1000,17);const bool m117irq=m117->irqActive();m117->cpuWrite(0xC002,0,0);
    const bool mapper117=m117p0==0x4000&&m117p1==0x6000&&m117c==0x1C00&&m117->mirroring()==Mirror::Horizontal&&m117before&&m117irq&&!m117->irqActive()&&mapperImplementationSupported(117,0);

    auto m120=makeMapper(120,0x20000,0x2000,0,0);
    uint32_t m120base=0,m120fixed=0;m120->cpuMapRead(0x6000,m120base);m120->cpuMapRead(0x8000,m120fixed);
    m120->cpuWrite(0x41FF,3,0);uint32_t m120sw=0;m120->cpuMapRead(0x6000,m120sw);
    const bool mapper120=m120base==0&&m120fixed==0x10000&&m120sw==0x6000&&mapperImplementationSupported(120,0);
    std::printf("phase37_mappers m104=%s m117=%s m120=%s\n",mapper104?"PASS":"FAIL",mapper117?"PASS":"FAIL",mapper120?"PASS":"FAIL");
    ok &= mapper104&&mapper117&&mapper120;


    // Phase 38: mapper 123 / Kǎshèng H2288. The bank-index register is
    // scrambled and $5800 can replace MMC3 PRG banking with NROM-128/256.
    auto m123=makeMapper(123,0x80000,0x40000,0,0);
    // Written MMC3 index 1 routes to physical register 3.
    m123->cpuWrite(0x8000,0x01,0); m123->cpuWrite(0x8001,0x12,0);
    uint32_t m123chr=0; m123->ppuMapRead(0x1400,m123chr);
    // NROM override: D6 enable; bank bits D5,D2,D4,D0 = 1,1,0,1 -> bank 13.
    m123->cpuWrite(0x5800,0x65,0); // NROM-128
    uint32_t m123n0=0,m123n1=0; m123->cpuMapRead(0x8000,m123n0); m123->cpuMapRead(0xC000,m123n1);
    m123->cpuWrite(0x5800,0x67,0); // NROM-256: CPU A14 replaces bank bit 0
    uint32_t m123p0=0,m123p1=0; m123->cpuMapRead(0x8000,m123p0); m123->cpuMapRead(0xC000,m123p1);
    const bool mapper123=m123chr==0x4800&&m123n0==0x34000&&m123n1==0x34000&&
        m123p0==0x30000&&m123p1==0x34000&&mapperImplementationSupported(123,0);
    std::printf("phase38_mapper123 scramble=%s nrom=%s gate=%s chr=%05X n128=%05X/%05X n256=%05X/%05X\n",
        m123chr==0x4800?"PASS":"FAIL",(m123n0==0x34000&&m123n1==0x34000&&m123p0==0x30000&&m123p1==0x34000)?"PASS":"FAIL",
        mapperImplementationSupported(123,0)?"PASS":"FAIL",m123chr,m123n0,m123n1,m123p0,m123p1);
    ok &= mapper123;

    // Phase 39: mapper 121 / Kǎshèng A9711/A9713.  Verify the protection
    // array, the bit-reversed protection latch/PRG overrides, A9713 outer
    // 256 KiB selection, and A9711's PPU-A12-wired CHR A18.
    auto m121=makeMapper(121,0x80000,0x80000,0,0);
    uint8_t m121prot=0; m121->cpuWrite(0x5000,2,0); const bool m121protOk=m121->cpuReadRegister(0x5000,m121prot)&&m121prot==0x42;
    // A9713 powers up with outer bit set; $5180 can select either 256 KiB half.
    uint32_t m121boot=0,m121outer=0,m121chrOuter=0; m121->cpuMapRead(0x8000,m121boot);
    m121->cpuWrite(0x5180,0x00,0); m121->cpuMapRead(0x8000,m121outer); m121->ppuMapRead(0,m121chrOuter);
    // $8001 value 3 reverses to $30. Selecting protection index $26 applies
    // it immediately to the E000 8 KiB slot and keeps following $8001 writes live.
    m121->cpuWrite(0x8001,0x03,0); m121->cpuWrite(0x8003,0x26,0); uint32_t m121e0=0; m121->cpuMapRead(0xE000,m121e0);
    m121->cpuWrite(0x8001,0x01,0); uint32_t m121e1=0; m121->cpuMapRead(0xE000,m121e1);
    std::vector<uint8_t> m121state; m121->saveState(m121state);
    m121->cpuWrite(0x8003,0x00,0);
    const uint8_t* m121sp=m121state.data(); const uint8_t* m121se=m121sp+m121state.size();
    const bool m121stateOk=m121->loadState(m121sp,m121se); uint32_t m121restored=0; m121->cpuMapRead(0xE000,m121restored);
    // Invalid protection index expires overrides and restores normal MMC3 PRG.
    m121->cpuWrite(0x8003,0x00,0); uint32_t m121normal=0; m121->cpuMapRead(0xE000,m121normal);
    auto m121a=makeMapper(121,0x40000,0x80000,0,0);
    m121a->cpuWrite(0x8000,0x80,0); // flip MMC3 CHR inversion; A18 must still follow PPU A12
    uint32_t m121left=0,m121right=0; m121a->ppuMapRead(0x0000,m121left); m121a->ppuMapRead(0x1000,m121right);
    const bool mapper121=m121protOk&&m121boot==0x40000&&m121outer==0&&m121chrOuter==0&&
        m121e0==0x60000&&m121e1==0x40000&&m121stateOk&&m121restored==0x40000&&m121normal==0x3E000&&
        m121left<0x40000&&m121right>=0x40000&&mapperImplementationSupported(121,0);
    std::printf("phase39_mapper121 protection=%s outer=%s override=%s chrA18=%s boot=%05X e=%05X/%05X normal=%05X chr=%05X/%05X\n",
        m121protOk?"PASS":"FAIL",(m121boot==0x40000&&m121outer==0&&m121chrOuter==0)?"PASS":"FAIL",
        (m121e0==0x60000&&m121e1==0x40000&&m121stateOk&&m121restored==0x40000&&m121normal==0x3E000)?"PASS":"FAIL",
        (m121left<0x40000&&m121right>=0x40000)?"PASS":"FAIL",m121boot,m121e0,m121e1,m121normal,m121left,m121right);
    ok &= mapper121;


    // Phase 22: Mapper 136 / Sachen JV001. Verify the copy-protection
    // register's staged-load, inversion and increment modes, D6-D7 open-bus
    // preservation, and the separately latched PRG/CHR output.
    auto m136 = makeMapper(136, 0x10000, 0x10000, 0, 0);
    uint32_t m136p0=0,m136c0=0,m136p1=0,m136c1=0,m136restoredP=0,m136restoredC=0;
    m136->cpuMapRead(0x8000,m136p0); m136->ppuMapRead(0x0123,m136c0);
    m136->cpuWrite(0x4122,0x15,0); // $4102 alias: staged input
    m136->cpuWrite(0x4100,0x00,0); // mode 0: register := input
    uint8_t m136read0=0xC0; const bool m136r0=m136->cpuReadRegister(0x41A0,m136read0);
    m136->cpuWrite(0x8000,0xFF,0); // written value ignored; latch register to output
    m136->cpuMapRead(0x8000,m136p1); m136->ppuMapRead(0x0123,m136c1);

    m136->cpuWrite(0x4101,0x01,0); // inversion on
    m136->cpuWrite(0x4102,0x2A,0);
    m136->cpuWrite(0x4100,0x00,0); // register = $25 (low nibble inverted)
    uint8_t m136readInv=0x80; const bool m136ri=m136->cpuReadRegister(0x4103,m136readInv); // readback D4-D5 invert => $15
    m136->cpuWrite(0x4103,0x01,0); // increment mode
    m136->cpuWrite(0x4100,0x00,0); // low nibble $5 -> $6
    uint8_t m136readInc=0xC0; const bool m136rc=m136->cpuReadRegister(0x4101,m136readInc);
    m136->cpuWrite(0xFFFF,0x00,0); // latch $26 to output
    std::vector<uint8_t> m136state; m136->saveState(m136state);
    m136->cpuWrite(0x4103,0x00,0); m136->cpuWrite(0x4101,0x00,0);
    m136->cpuWrite(0x4102,0x00,0); m136->cpuWrite(0x4100,0x00,0); m136->cpuWrite(0x8000,0,0);
    const uint8_t* m136sp=m136state.data(); const uint8_t* m136se=m136sp+m136state.size();
    const bool m136load=m136->loadState(m136sp,m136se);
    m136->cpuMapRead(0x8000,m136restoredP); m136->ppuMapRead(0x0123,m136restoredC);
    const bool m136gate=mapperImplementationSupported(136,0)&&!mapperImplementationSupported(136,1);
    const bool mapper136=m136p0==0&&m136c0==0x0123&&m136r0&&m136read0==0xD5&&
        m136p1==0x8000&&m136c1==0xA123&&m136ri&&m136readInv==0x95&&m136rc&&m136readInc==0xD6&&
        m136load&&m136restoredP==0&&m136restoredC==0xC123&&m136gate;
    std::printf("phase22_mapper136 protection=%s openbus=%s banks=%s state=%s gate=%s\n",
        (m136r0&&m136ri&&m136rc&&m136read0==0xD5&&m136readInv==0x95&&m136readInc==0xD6)?"PASS":"FAIL",
        ((m136read0&0xC0)==0xC0&&(m136readInv&0xC0)==0x80&&(m136readInc&0xC0)==0xC0)?"PASS":"FAIL",
        (m136p1==0x8000&&m136c1==0xA123)?"PASS":"FAIL",
        (m136load&&m136restoredP==0&&m136restoredC==0xC123)?"PASS":"FAIL",m136gate?"PASS":"FAIL");
    ok &= mapper136;

    // Phase 21: Mapper 133 / Sachen SA-72008.  The 72-pin Jovial Race
    // board decodes $4100 under mask $E100; D2 selects one of two 32 KiB PRG
    // banks and D1-D0 select one of four 8 KiB CHR banks.
    auto m133 = makeMapper(133, 0x10000, 0x8000, 0, 0);
    uint32_t m133p0=0,m133p1=0,m133c0=0,m133c1=0,m133alias=0;
    m133->cpuMapRead(0x8000,m133p0); m133->ppuMapRead(0x0123,m133c0);
    const bool m133Ignored=!m133->cpuWrite(0x4000,0x07,0);
    m133->cpuWrite(0x4120,0x07,0); // alias of $4100 under mask $E100
    m133->cpuMapRead(0x8000,m133p1); m133->ppuMapRead(0x0123,m133c1);
    m133->cpuWrite(0x41FF,0x00,0); m133->ppuMapRead(0x0123,m133alias);
    std::vector<uint8_t> m133state; m133->cpuWrite(0x4100,0x06,0); m133->saveState(m133state);
    m133->cpuWrite(0x4100,0x00,0);
    const uint8_t* m133sp=m133state.data(); const uint8_t* m133se=m133sp+m133state.size();
    const bool m133load=m133->loadState(m133sp,m133se); uint32_t m133restoredP=0,m133restoredC=0;
    m133->cpuMapRead(0x8000,m133restoredP); m133->ppuMapRead(0x0123,m133restoredC);
    const bool m133gate=mapperImplementationSupported(133,0)&&!mapperImplementationSupported(133,1);
    const bool mapper133=m133p0==0&&m133c0==0x0123&&m133Ignored&&m133p1==0x8000&&m133c1==0x6123&&
        m133alias==0x0123&&m133load&&m133restoredP==0x8000&&m133restoredC==0x4123&&m133gate;
    std::printf("phase21_mapper133 prg=%s chr=%s decode=%s state=%s gate=%s\n",
        (m133p0==0&&m133p1==0x8000)?"PASS":"FAIL",(m133c0==0x0123&&m133c1==0x6123)?"PASS":"FAIL",
        (m133Ignored&&m133alias==0x0123)?"PASS":"FAIL",
        (m133load&&m133restoredP==0x8000&&m133restoredC==0x4123)?"PASS":"FAIL",m133gate?"PASS":"FAIL");
    ok &= mapper133;

    // Phase 20: Mapper 145 / Sachen SA-72007.  Fixed PRG and a single
    // $4100-decoded latch (mask $E100) whose D7 selects the 8 KiB CHR bank.
    auto m145 = makeMapper(145, 0x8000, 0x4000, 0, 0);
    uint32_t m145p0=0,m145p1=0,m145c0=0,m145c1=0,m145alias=0;
    m145->cpuMapRead(0x8000,m145p0); m145->cpuMapRead(0xFFFF,m145p1);
    m145->ppuMapRead(0x0123,m145c0);
    const bool m145Ignored = !m145->cpuWrite(0x4000,0x80,0);
    m145->cpuWrite(0x4120,0x80,0); // $4120 matches $4100 under mask $E100
    m145->ppuMapRead(0x0123,m145c1);
    m145->cpuWrite(0x41FF,0x00,0); // same decode, return to bank 0
    m145->ppuMapRead(0x0123,m145alias);
    std::vector<uint8_t> m145state; m145->cpuWrite(0x4100,0x80,0); m145->saveState(m145state);
    m145->cpuWrite(0x4100,0x00,0);
    const uint8_t* m145sp=m145state.data(); const uint8_t* m145se=m145sp+m145state.size();
    const bool m145load=m145->loadState(m145sp,m145se); uint32_t m145restored=0; m145->ppuMapRead(0x0123,m145restored);
    const bool m145gate=mapperImplementationSupported(145,0);
    const bool mapper145=m145p0==0x0000&&m145p1==0x7FFF&&m145c0==0x0123&&m145Ignored&&
        m145c1==0x2123&&m145alias==0x0123&&m145load&&m145restored==0x2123&&m145gate;
    std::printf("phase20_mapper145 fixed=%s chr=%s decode=%s state=%s gate=%s\n",
        (m145p0==0&&m145p1==0x7FFF)?"PASS":"FAIL",(m145c0==0x0123&&m145c1==0x2123)?"PASS":"FAIL",
        (m145Ignored&&m145alias==0x0123)?"PASS":"FAIL",(m145load&&m145restored==0x2123)?"PASS":"FAIL",m145gate?"PASS":"FAIL");
    ok &= mapper145;

    // Phase 19: Mapper 91. Submapper 0 (JY830623C) uses four 2 KiB
    // CHR banks, two 8 KiB PRG banks, an address-latched 128 KiB PRG /
    // 512 KiB CHR outer bank, and an IRQ after 64 unfiltered PPU-A12 rises.
    // Submapper 1 (EJ-006-1) removes the outer latch, adds H/V mirroring,
    // and clocks its programmable IRQ counter by 5 every fourth M2 cycle.
    auto m91s0 = makeMapper(91, 0x80000, 0x100000, 0, 0, 0, true, Mirror::Horizontal);
    m91s0->cpuWrite(0x7000, 2, 0); m91s0->cpuWrite(0x7001, 3, 0);
    m91s0->cpuWrite(0x6000, 5, 0);
    m91s0->cpuWrite(0x8007, 0, 0); // address bits: outer PRG=3, outer CHR=1
    uint32_t m91s0p0=0,m91s0p1=0,m91s0pf=0,m91s0chr=0;
    m91s0->cpuMapRead(0x8000,m91s0p0); m91s0->cpuMapRead(0xA000,m91s0p1);
    m91s0->cpuMapRead(0xC000,m91s0pf); m91s0->ppuMapRead(0x0000,m91s0chr);
    m91s0->cpuWrite(0x7007,0,0);
    for (unsigned i=0;i<63;++i) { m91s0->notifyPpuAddress(0x0000,i*2); m91s0->notifyPpuAddress(0x1000,i*2+1); }
    const bool m91s0Before = !m91s0->irqActive();
    m91s0->notifyPpuAddress(0x0000,126); m91s0->notifyPpuAddress(0x1000,127);
    const bool m91s0Irq = m91s0->irqActive();
    m91s0->cpuWrite(0x7006,0,0);
    const bool m91s0Ack = !m91s0->irqActive();
    std::vector<uint8_t> m91state; m91s0->saveState(m91state);
    m91s0->cpuWrite(0x8000,0,0);
    const uint8_t* m91sp=m91state.data(); const uint8_t* m91se=m91sp+m91state.size();
    const bool m91stateLoad=m91s0->loadState(m91sp,m91se);
    uint32_t m91restored=0; m91s0->cpuMapRead(0x8000,m91restored);

    auto m91s1 = makeMapper(91, 0x20000, 0x20000, 0, 0, 1, true, Mirror::Horizontal);
    m91s1->cpuWrite(0x7000,2,0); m91s1->cpuWrite(0x7001,3,0); m91s1->cpuWrite(0x6000,4,0);
    m91s1->cpuWrite(0x6005,0,0); const bool m91s1V=m91s1->mirroring()==Mirror::Vertical;
    m91s1->cpuWrite(0x6004,0,0); const bool m91s1H=m91s1->mirroring()==Mirror::Horizontal;
    uint32_t m91s1p=0,m91s1f=0,m91s1chr=0;
    m91s1->cpuMapRead(0x8000,m91s1p); m91s1->cpuMapRead(0xC000,m91s1f); m91s1->ppuMapRead(0,m91s1chr);
    m91s1->cpuWrite(0x6006,10,0); m91s1->cpuWrite(0x6007,0,0); m91s1->cpuWrite(0x7007,0,0);
    for(unsigned i=0;i<4;++i)m91s1->clockCpu();
    const bool m91s1Before=!m91s1->irqActive();
    for(unsigned i=0;i<4;++i)m91s1->clockCpu();
    const bool m91s1Irq=m91s1->irqActive();
    m91s1->cpuWrite(0x7006,0,0); const bool m91s1Ack=!m91s1->irqActive();
    const bool m91s0Banks=m91s0p0==0x64000&&m91s0p1==0x66000&&m91s0pf==0x7C000&&m91s0chr==0x82800;
    const bool m91s1Banks=m91s1p==0x4000&&m91s1f==0x1C000&&m91s1chr==0x2000;
    const bool m91gate=mapperImplementationSupported(91,0)&&mapperImplementationSupported(91,1)&&!mapperImplementationSupported(91,2);
    const bool mapper91=m91s0Banks&&m91s0Before&&m91s0Irq&&m91s0Ack&&m91stateLoad&&m91restored==0x64000&&
        m91s1Banks&&m91s1V&&m91s1H&&m91s1Before&&m91s1Irq&&m91s1Ack&&m91gate;
    std::printf("phase19_mapper91 s0=%s irq=%s state=%s s1=%s irq=%s mirror=%s gate=%s\n",
        mapper91?"PASS":"FAIL",(m91s0Before&&m91s0Irq&&m91s0Ack)?"PASS":"FAIL",
        (m91stateLoad&&m91restored==0x64000)?"PASS":"FAIL",m91s1Banks?"PASS":"FAIL",
        (m91s1Before&&m91s1Irq&&m91s1Ack)?"PASS":"FAIL",(m91s1V&&m91s1H)?"PASS":"FAIL",m91gate?"PASS":"FAIL");
    ok &= mapper91;

    // Phase 18: Mapper 206 / Namco 118 (DxROM). NES 2.0 submapper 1
    // identifies 3407/3417/3451 PCBs with 32 KiB PRG-ROM wired directly to
    // CPU A13/A14. Their nominal PRG bank-register writes must therefore be
    // ignored, while the ordinary submapper-0 board remains bankable and CHR
    // banking continues to operate on both variants.
    auto m206s0 = makeMapper(206, 0x10000, 0x10000, 0, 0, 0, true);
    auto m206s1 = makeMapper(206, 0x08000, 0x10000, 0, 0, 1, true);
    m206s0->cpuWrite(0x8000, 6, 0); m206s0->cpuWrite(0x8001, 5, 0);
    m206s1->cpuWrite(0x8000, 6, 0); m206s1->cpuWrite(0x8001, 5, 0);
    m206s1->cpuWrite(0x8000, 7, 0); m206s1->cpuWrite(0x8001, 6, 0);
    uint32_t m206banked0=0,m206unbank0=0,m206unbank1=0,m206unbank2=0,m206unbank3=0;
    m206s0->cpuMapRead(0x8000,m206banked0);
    m206s1->cpuMapRead(0x8000,m206unbank0); m206s1->cpuMapRead(0xA000,m206unbank1);
    m206s1->cpuMapRead(0xC000,m206unbank2); m206s1->cpuMapRead(0xE000,m206unbank3);
    m206s1->cpuWrite(0x8000, 2, 0); m206s1->cpuWrite(0x8001, 5, 0);
    uint32_t m206chr=0; m206s1->ppuMapRead(0x1000,m206chr);
    const bool m206NormalBanking = m206banked0 == 0xA000;
    const bool m206Unbanked = m206unbank0 == 0x0000 && m206unbank1 == 0x2000 &&
        m206unbank2 == 0x4000 && m206unbank3 == 0x6000;
    const bool m206ChrBanking = m206chr == 0x1400;
    const bool m206Gate = mapperImplementationSupported(206,0) && mapperImplementationSupported(206,1) &&
        !mapperImplementationSupported(206,2);
    const bool mapper206 = m206NormalBanking && m206Unbanked && m206ChrBanking && m206Gate;
    std::printf("phase18_mapper206 unbanked=%s normal=%s chr=%s gate=%s prg=%05X/%05X/%05X/%05X normal0=%05X chr=%05X\n",
        mapper206 ? "PASS" : "FAIL", m206NormalBanking ? "PASS" : "FAIL",
        m206ChrBanking ? "PASS" : "FAIL", m206Gate ? "PASS" : "FAIL",
        m206unbank0,m206unbank1,m206unbank2,m206unbank3,m206banked0,m206chr);
    ok &= mapper206;

    // Phase 23: Mapper 213 / BMC 9999999-in-1.  The board latches
    // CPU address lines rather than CPU data: A2-A1 choose the 32 KiB PRG
    // bank and A5-A3 choose the 8 KiB CHR bank.  Header mirroring is fixed.
    auto m213 = makeMapper(213, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Vertical);
    uint32_t m213p0=0,m213c0=0,m213p1=0,m213c1=0,m213restoredP=0,m213restoredC=0;
    m213->cpuMapRead(0x8000,m213p0); m213->ppuMapRead(0x0123,m213c0);
    const bool m213IgnoredLow = !m213->cpuWrite(0x7FFF,0xFF,0);
    m213->cpuWrite(0x801A,0x00,0); // PRG=(A2-A1)=1, CHR=(A5-A3)=3
    m213->cpuMapRead(0x8000,m213p1); m213->ppuMapRead(0x0123,m213c1);
    const bool m213Mirror = m213->mirroring() == Mirror::Vertical;
    std::vector<uint8_t> m213state; m213->saveState(m213state);
    m213->cpuWrite(0x8006,0xFF,0); // PRG=3, CHR=0; data must be ignored
    const uint8_t* m213sp=m213state.data(); const uint8_t* m213se=m213sp+m213state.size();
    const bool m213load=m213->loadState(m213sp,m213se);
    m213->cpuMapRead(0x8000,m213restoredP); m213->ppuMapRead(0x0123,m213restoredC);
    const bool m213gate=mapperImplementationSupported(213,0)&&!mapperImplementationSupported(213,1);
    const bool mapper213=m213p0==0&&m213c0==0x0123&&m213IgnoredLow&&m213p1==0x8000&&m213c1==0x6123&&
        m213Mirror&&m213load&&m213restoredP==0x8000&&m213restoredC==0x6123&&m213gate;
    std::printf("phase23_mapper213 prg=%s chr=%s decode=%s state=%s mirror=%s gate=%s\n",
        (m213p0==0&&m213p1==0x8000)?"PASS":"FAIL",(m213c0==0x0123&&m213c1==0x6123)?"PASS":"FAIL",
        m213IgnoredLow?"PASS":"FAIL",(m213load&&m213restoredP==0x8000&&m213restoredC==0x6123)?"PASS":"FAIL",
        m213Mirror?"PASS":"FAIL",m213gate?"PASS":"FAIL");
    ok &= mapper213;

    // Phase 24: Sachen 8259 A/B/C family (mappers 141/138/139).
    // All variants share the $4100/$4101 register interface and 32 KiB PRG
    // banking. Their CHR wiring differs by 0/1/2 shifted mapper address bits.
    auto write8259 = [](Mapper& m, uint8_t reg, uint8_t value) {
        m.cpuWrite(0x4120, reg, 0); // alias of $4100 under mask $C101
        m.cpuWrite(0x4121, value, 0); // alias of $4101
    };
    auto m138 = makeMapper(138, 0x20000, 0x20000, 0, 0, 0, false, Mirror::Horizontal);
    write8259(*m138,5,3); write8259(*m138,4,1);
    for (uint8_t r=0;r<4;++r) write8259(*m138,r,2);
    uint32_t m138p=0,m138c0=0,m138c1=0; m138->cpuMapRead(0x8000,m138p); m138->ppuMapRead(0x0000,m138c0); m138->ppuMapRead(0x0800,m138c1);

    auto m141 = makeMapper(141, 0x20000, 0x40000, 0, 0, 0, false, Mirror::Horizontal);
    write8259(*m141,4,1); for (uint8_t r=0;r<4;++r) write8259(*m141,r,2);
    uint32_t m141c0=0,m141c1=0; m141->ppuMapRead(0x0000,m141c0); m141->ppuMapRead(0x0800,m141c1);

    auto m139 = makeMapper(139, 0x20000, 0x80000, 0, 0, 0, false, Mirror::Horizontal);
    write8259(*m139,4,1); for (uint8_t r=0;r<4;++r) write8259(*m139,r,2);
    uint32_t m139c[4]={}; for (unsigned i=0;i<4;++i) m139->ppuMapRead(uint16_t(i*0x800),m139c[i]);

    // Normal mirroring modes plus the asymmetric (0,1,1,1) mode.
    NametableSource m8259NtSrc=NametableSource::ChrRom; uint32_t nt[4]={};
    write8259(*m138,7,4); // mm=2, simple=0 => (0,1,1,1)
    for(unsigned i=0;i<4;++i)m138->mapNametable(uint16_t(0x2000+i*0x400),m8259NtSrc,nt[i]);
    const bool m8259Asym = nt[0]==0x000 && nt[1]==0x400 && nt[2]==0x400 && nt[3]==0x400;
    write8259(*m138,7,1); // simple mode => vertical and all CHR slots use reg 0
    write8259(*m138,0,5); write8259(*m138,1,1);
    uint32_t simple0=0,simple1=0; m138->ppuMapRead(0x0000,simple0); m138->ppuMapRead(0x0800,simple1);
    NametableSource simpleSrc=NametableSource::ChrRom; uint32_t simpleNt0=0,simpleNt1=0;
    m138->mapNametable(0x2000,simpleSrc,simpleNt0); m138->mapNametable(0x2400,simpleSrc,simpleNt1);

    // Mapper 141 also has a known Q-Boy variant with unbanked 8 KiB CHR-RAM.
    auto m141ram = makeMapper(141, 0x8000, 0, 0, 0x2000, 0, false);
    write8259(*m141ram,4,7); write8259(*m141ram,0,7);
    uint32_t ramRead=0,ramWrite=0; const bool ramR=m141ram->ppuMapRead(0x1234,ramRead); const bool ramW=m141ram->ppuMapWrite(0x1234,ramWrite);

    std::vector<uint8_t> m8259state; m139->saveState(m8259state); write8259(*m139,4,0); write8259(*m139,0,0);
    const uint8_t* m8259sp=m8259state.data(); const uint8_t* m8259se=m8259sp+m8259state.size();
    const bool m8259load=m139->loadState(m8259sp,m8259se); uint32_t m139restored=0; m139->ppuMapRead(0x0000,m139restored);
    const bool m8259gate=mapperImplementationSupported(138,0)&&mapperImplementationSupported(139,0)&&mapperImplementationSupported(141,0)&&
        !mapperImplementationSupported(138,1)&&!mapperImplementationSupported(139,1)&&!mapperImplementationSupported(141,1);
    const bool mapper8259 = m138p==0x18000 && m138c0==0x5000 && m138c1==0x5000 &&
        m141c0==0xA000 && m141c1==0xA800 && m139c[0]==0x14000 && m139c[1]==0x14800 && m139c[2]==0x15000 && m139c[3]==0x15800 &&
        m8259Asym && simple0==0x6800 && simple1==0x6800 && simpleNt0==0x000 && simpleNt1==0x400 &&
        ramR && ramW && ramRead==0x1234 && ramWrite==0x1234 && m8259load && m139restored==0x14000 && m8259gate;
    std::printf("phase24_sachen8259 b=%s a=%s c=%s mirror=%s simple=%s chrram=%s state=%s gate=%s\n",
        (m138p==0x18000&&m138c0==0x5000&&m138c1==0x5000)?"PASS":"FAIL",
        (m141c0==0xA000&&m141c1==0xA800)?"PASS":"FAIL",
        (m139c[0]==0x14000&&m139c[1]==0x14800&&m139c[2]==0x15000&&m139c[3]==0x15800)?"PASS":"FAIL",
        m8259Asym?"PASS":"FAIL",(simple0==0x6800&&simple1==0x6800&&simpleNt0==0&&simpleNt1==0x400)?"PASS":"FAIL",
        (ramR&&ramW&&ramRead==0x1234&&ramWrite==0x1234)?"PASS":"FAIL",
        (m8259load&&m139restored==0x14000)?"PASS":"FAIL",m8259gate?"PASS":"FAIL");
    ok &= mapper8259;

    // Phase 34: Mapper 135 is the older iNES assignment for the Sachen
    // 8259A CHR-ROM board.  Modern documentation folds that hardware into
    // mapper 141, but legacy Super Pang-style dumps still use mapper 135.
    // It must therefore behave exactly like the banked-CHR 8259A variant,
    // while mapper 141 continues to retain its separate Q-Boy CHR-RAM path.
    auto m135 = makeMapper(135, 0x20000, 0x40000, 0, 0, 0, false, Mirror::Horizontal);
    write8259(*m135,5,3);
    write8259(*m135,4,1);
    for (uint8_t r=0;r<4;++r) write8259(*m135,r,2);
    uint32_t m135p=0,m135c0=0,m135c1=0,m135restored=0;
    m135->cpuMapRead(0x8000,m135p);
    m135->ppuMapRead(0x0000,m135c0);
    m135->ppuMapRead(0x0800,m135c1);
    write8259(*m135,7,4); // asymmetric (0,1,1,1) mode like mapper 141 8259A
    NametableSource m135src=NametableSource::ChrRom; uint32_t m135nt[4]={};
    for(unsigned i=0;i<4;++i) m135->mapNametable(uint16_t(0x2000+i*0x400),m135src,m135nt[i]);
    std::vector<uint8_t> m135state; m135->saveState(m135state);
    write8259(*m135,4,0); write8259(*m135,0,0);
    const uint8_t* m135sp=m135state.data(); const uint8_t* m135se=m135sp+m135state.size();
    const bool m135load=m135->loadState(m135sp,m135se);
    m135->ppuMapRead(0x0000,m135restored);
    const bool m135gate=mapperImplementationSupported(135,0)&&!mapperImplementationSupported(135,1);
    const bool mapper135=m135p==0x18000&&m135c0==0xA000&&m135c1==0xA800&&
        m135nt[0]==0x000&&m135nt[1]==0x400&&m135nt[2]==0x400&&m135nt[3]==0x400&&
        m135load&&m135restored==0xA000&&m135gate;
    std::printf("phase34_mapper135 alias=%s prg=%s chr=%s mirror=%s state=%s gate=%s\n",
        mapper135?"PASS":"FAIL",m135p==0x18000?"PASS":"FAIL",
        (m135c0==0xA000&&m135c1==0xA800)?"PASS":"FAIL",
        (m135nt[0]==0&&m135nt[1]==0x400&&m135nt[2]==0x400&&m135nt[3]==0x400)?"PASS":"FAIL",
        (m135load&&m135restored==0xA000)?"PASS":"FAIL",m135gate?"PASS":"FAIL");
    ok &= mapper135;

    // Phase 25: Mapper 148 moves the NINA-06/GNROM latch into $8000-$FFFF.
    // D3 selects the 32 KiB PRG bank, D2-D0 select the 8 KiB CHR bank, and
    // the overlapping PRG-ROM output means the board has wired-AND conflicts.
    auto m148 = makeMapper(148, 0x10000, 0x10000, 0, 0, 0, false, Mirror::Vertical);
    uint32_t m148p0=0,m148c0=0,m148p1=0,m148c1=0,m148restoredP=0,m148restoredC=0;
    m148->cpuMapRead(0x8000,m148p0); m148->ppuMapRead(0x0123,m148c0);
    const bool m148IgnoredLow = !m148->cpuWrite(0x7FFF,0x0F,0);
    m148->cpuWrite(0x9000,0x0D,0); // PRG 1, CHR 5
    m148->cpuMapRead(0x8000,m148p1); m148->ppuMapRead(0x0123,m148c1);
    const bool m148Mirror = m148->mirroring() == Mirror::Vertical;
    std::vector<uint8_t> m148state; m148->saveState(m148state);
    m148->cpuWrite(0xFFFF,0x00,0);
    const uint8_t* m148sp=m148state.data(); const uint8_t* m148se=m148sp+m148state.size();
    const bool m148load=m148->loadState(m148sp,m148se);
    m148->cpuMapRead(0x8000,m148restoredP); m148->ppuMapRead(0x0123,m148restoredC);
    const bool m148gate=mapperImplementationSupported(148,0)&&!mapperImplementationSupported(148,1);
    const bool mapper148=m148p0==0&&m148c0==0x0123&&m148IgnoredLow&&m148p1==0x8000&&m148c1==0xA123&&
        m148->hasBusConflicts()&&m148Mirror&&m148load&&m148restoredP==0x8000&&m148restoredC==0xA123&&m148gate;
    std::printf("phase25_mapper148 prg=%s chr=%s conflict=%s state=%s mirror=%s gate=%s\n",
        (m148p0==0&&m148p1==0x8000)?"PASS":"FAIL",(m148c0==0x0123&&m148c1==0xA123)?"PASS":"FAIL",
        m148->hasBusConflicts()?"PASS":"FAIL",(m148load&&m148restoredP==0x8000&&m148restoredC==0xA123)?"PASS":"FAIL",
        m148Mirror?"PASS":"FAIL",m148gate?"PASS":"FAIL");
    ok &= mapper148;

    // Phase 26: Mapper 150 / Sachen SA-015 (74LS374N).  $4100 selects one
    // of eight 3-bit registers and $4101 accesses it. R5 selects 32 KiB PRG,
    // R4/R6 compose 8 KiB CHR, and R7 controls H/V/one-screen/custom mirroring.
    auto m150 = makeMapper(150, 0x20000, 0x10000, 0, 0, 0, false);
    auto write150 = [](Mapper& m, uint8_t reg, uint8_t value) {
        m.cpuWrite(0x4120, reg, 0);  // $4100 alias under mask $C101
        m.cpuWrite(0x4121, value, 0); // $4101 alias
    };
    write150(*m150,5,3);
    write150(*m150,4,1);
    write150(*m150,6,2); // CHR bank = 4 | 2 = 6
    uint32_t m150p=0,m150c=0; m150->cpuMapRead(0x8000,m150p); m150->ppuMapRead(0x0123,m150c);

    // Register data reads drive only D0-D2; upper bits stay open bus.
    m150->cpuWrite(0x4100,6,0);
    uint8_t m150read=0xA8; const bool m150readOk=m150->cpuReadRegister(0x4101,m150read);
    uint8_t m150ignored=0x5A; const bool m150readIgnored=!m150->cpuReadRegister(0x4100,m150ignored);

    NametableSource m150src=NametableSource::ChrRom; uint32_t m150nt[4]={};
    write150(*m150,7,0);
    for(unsigned i=0;i<4;++i)m150->mapNametable(uint16_t(0x2000+i*0x400),m150src,m150nt[i]);
    const bool m150Custom=m150nt[0]==0x000&&m150nt[1]==0x000&&m150nt[2]==0x000&&m150nt[3]==0x400;
    write150(*m150,7,2); const bool m150H=m150->mirroring()==Mirror::Horizontal;
    write150(*m150,7,4); const bool m150V=m150->mirroring()==Mirror::Vertical;
    write150(*m150,7,6); const bool m150One=m150->mirroring()==Mirror::OnescreenHi;

    std::vector<uint8_t> m150state; m150->saveState(m150state);
    write150(*m150,5,0); write150(*m150,4,0); write150(*m150,6,0); write150(*m150,7,0);
    const uint8_t* m150sp=m150state.data(); const uint8_t* m150se=m150sp+m150state.size();
    const bool m150load=m150->loadState(m150sp,m150se); uint32_t m150rp=0,m150rc=0;
    m150->cpuMapRead(0x8000,m150rp); m150->ppuMapRead(0x0123,m150rc);
    const bool m150gate=mapperImplementationSupported(150,0)&&!mapperImplementationSupported(150,1);
    const bool mapper150=m150p==0x18000&&m150c==0xC123&&m150readOk&&m150read==0xAA&&m150readIgnored&&
        m150Custom&&m150H&&m150V&&m150One&&m150load&&m150rp==0x18000&&m150rc==0xC123&&
        m150->mirroring()==Mirror::OnescreenHi&&m150gate;
    std::printf("phase26_mapper150 prg=%s chr=%s read=%s mirror=%s state=%s gate=%s\n",
        m150p==0x18000?"PASS":"FAIL",m150c==0xC123?"PASS":"FAIL",
        (m150readOk&&m150read==0xAA&&m150readIgnored)?"PASS":"FAIL",
        (m150Custom&&m150H&&m150V&&m150One)?"PASS":"FAIL",
        (m150load&&m150rp==0x18000&&m150rc==0xC123&&m150->mirroring()==Mirror::OnescreenHi)?"PASS":"FAIL",
        m150gate?"PASS":"FAIL");
    ok &= mapper150;

    // Phase 27: Mapper 137 / Sachen 8259D. Unlike the A/B/C variants,
    // this board switches four 1 KiB banks in $0000-$0FFF and fixes the upper
    // pattern table to the last 4 KiB of CHR-ROM. Its glue logic provides
    // independent high bits to slots 1-3 and uses H/V/custom/one-screen-A
    // mirroring; simple mode fixes horizontal mirroring.
    auto m137 = makeMapper(137, 0x20000, 0x8000, 0, 0, 0, true);
    auto write137 = [](Mapper& m, uint8_t reg, uint8_t value) {
        m.cpuWrite(0x4120, reg, 0);   // $4100 alias under mask $C101
        m.cpuWrite(0x4121, value, 0); // $4101 alias
    };
    write137(*m137,0,1); write137(*m137,1,2); write137(*m137,2,3); write137(*m137,3,4);
    write137(*m137,4,7); write137(*m137,5,3); write137(*m137,6,1); write137(*m137,7,2);
    uint32_t m137p=0,m137c[5]={};
    m137->cpuMapRead(0x8000,m137p);
    m137->ppuMapRead(0x0123,m137c[0]); m137->ppuMapRead(0x0523,m137c[1]);
    m137->ppuMapRead(0x0923,m137c[2]); m137->ppuMapRead(0x0D23,m137c[3]);
    m137->ppuMapRead(0x1123,m137c[4]);
    const bool m137Banks = m137p==0x18000 && m137c[0]==0x0523 && m137c[1]==0x4923 &&
        m137c[2]==0x4D23 && m137c[3]==0x7123 && m137c[4]==0x7123;
    const bool m137V = m137->mirroring()==Mirror::Vertical;

    NametableSource m137src=NametableSource::ChrRom; uint32_t m137nt[4]={};
    write137(*m137,7,4); // custom (0,1,1,1)
    for(unsigned i=0;i<4;++i)m137->mapNametable(uint16_t(0x2000+i*0x400),m137src,m137nt[i]);
    const bool m137Asym=m137nt[0]==0&&m137nt[1]==0x400&&m137nt[2]==0x400&&m137nt[3]==0x400;
    write137(*m137,7,1); // simple mode: horizontal, all ccc bits from R0
    uint32_t m137simple1=0,m137simple3=0;
    m137->ppuMapRead(0x0523,m137simple1); m137->ppuMapRead(0x0D23,m137simple3);
    const bool m137Simple = m137->mirroring()==Mirror::Horizontal &&
        m137simple1==0x4523 && m137simple3==0x6523;
    write137(*m137,7,6); const bool m137One=m137->mirroring()==Mirror::OnescreenLo;

    std::vector<uint8_t> m137state; m137->saveState(m137state);
    write137(*m137,5,0); write137(*m137,7,0);
    const uint8_t* m137sp=m137state.data(); const uint8_t* m137se=m137sp+m137state.size();
    const bool m137load=m137->loadState(m137sp,m137se); uint32_t m137rp=0;
    m137->cpuMapRead(0x8000,m137rp);
    const bool m137gate=mapperImplementationSupported(137,0)&&!mapperImplementationSupported(137,1);
    const bool mapper137=m137Banks&&m137V&&m137Asym&&m137Simple&&m137One&&m137load&&
        m137rp==0x18000&&m137->mirroring()==Mirror::OnescreenLo&&m137gate;
    std::printf("phase27_mapper137 banks=%s mirror=%s simple=%s state=%s gate=%s\n",
        m137Banks?"PASS":"FAIL",(m137V&&m137Asym&&m137One)?"PASS":"FAIL",
        m137Simple?"PASS":"FAIL",(m137load&&m137rp==0x18000)?"PASS":"FAIL",m137gate?"PASS":"FAIL");
    ok &= mapper137;

    // Phase 28: Mapper 149 / Sachen SA-0036.  PRG is fixed, while D7 of
    // writes in $8000-$FFFF selects one of two 8 KiB CHR-ROM banks.  The
    // register overlaps PRG-ROM and therefore uses wired-AND bus conflicts.
    auto m149 = makeMapper(149, 0x8000, 0x4000, 0, 0, 0, false, Mirror::Horizontal);
    uint32_t m149p0=0,m149p1=0,m149c0=0,m149c1=0,m149cr=0;
    const bool m149pr0=m149->cpuMapRead(0x8000,m149p0);
    const bool m149pr1=m149->cpuMapRead(0xFFFF,m149p1);
    m149->ppuMapRead(0x0123,m149c0);
    const bool m149IgnoredLow=!m149->cpuWrite(0x7FFF,0x80,0);
    m149->cpuWrite(0x8000,0x80,0);
    m149->ppuMapRead(0x0123,m149c1);
    const bool m149Mirror=m149->mirroring()==Mirror::Horizontal;
    std::vector<uint8_t> m149state; m149->saveState(m149state);
    m149->cpuWrite(0xFFFF,0x00,0);
    const uint8_t* m149sp=m149state.data(); const uint8_t* m149se=m149sp+m149state.size();
    const bool m149load=m149->loadState(m149sp,m149se);
    m149->ppuMapRead(0x0123,m149cr);
    const bool m149gate=mapperImplementationSupported(149,0)&&!mapperImplementationSupported(149,1);
    const bool mapper149=m149pr0&&m149pr1&&m149p0==0&&m149p1==0x7FFF&&m149c0==0x0123&&
        m149IgnoredLow&&m149c1==0x2123&&m149->hasBusConflicts()&&m149Mirror&&
        m149load&&m149cr==0x2123&&m149gate;
    std::printf("phase28_mapper149 fixed=%s chr=%s conflict=%s state=%s mirror=%s gate=%s\n",
        (m149pr0&&m149pr1&&m149p0==0&&m149p1==0x7FFF)?"PASS":"FAIL",
        (m149c0==0x0123&&m149c1==0x2123)?"PASS":"FAIL",m149->hasBusConflicts()?"PASS":"FAIL",
        (m149load&&m149cr==0x2123)?"PASS":"FAIL",m149Mirror?"PASS":"FAIL",m149gate?"PASS":"FAIL");
    ok &= mapper149;


    // Phase 29: Mapper 143 / Sachen TCA-01.  The board is NROM-like but
    // exposes a read-only protection device in $4100-$5FFF.  On the decoded
    // $4100 + k*$200 aliases it returns (~address & $3F) | $40; intervening
    // addresses return zero. PRG/CHR remain fixed.
    auto m143 = makeMapper(143, 0x8000, 0x2000, 0, 0, 0, false, Mirror::Vertical);
    uint32_t m143p0=0,m143p1=0,m143c=0;
    const bool m143Pr0=m143->cpuMapRead(0x8000,m143p0);
    const bool m143Pr1=m143->cpuMapRead(0xFFFF,m143p1);
    m143->ppuMapRead(0x1234,m143c);
    uint8_t m143r0=0xAA,m143r1=0xAA,m143r2=0xAA,m143r3=0xAA,m143out=0xAA;
    const bool m143Read0=m143->cpuReadRegister(0x4100,m143r0); // $7F
    const bool m143Read1=m143->cpuReadRegister(0x413F,m143r1); // $40
    const bool m143Read2=m143->cpuReadRegister(0x4301,m143r2); // $7E alias
    const bool m143Read3=m143->cpuReadRegister(0x4200,m143r3); // undecoded -> 0
    const bool m143Outside=!m143->cpuReadRegister(0x4000,m143out) && !m143->cpuReadRegister(0x6000,m143out);
    const bool m143gate=mapperImplementationSupported(143,0)&&!mapperImplementationSupported(143,1);
    const bool mapper143=m143Pr0&&m143Pr1&&m143p0==0&&m143p1==0x7FFF&&m143c==0x1234&&
        m143Read0&&m143Read1&&m143Read2&&m143Read3&&m143r0==0x7F&&m143r1==0x40&&
        m143r2==0x7E&&m143r3==0x00&&m143Outside&&m143->mirroring()==Mirror::Vertical&&m143gate;
    std::printf("phase29_mapper143 fixed=%s protection=%s decode=%s mirror=%s gate=%s\n",
        (m143Pr0&&m143Pr1&&m143p0==0&&m143p1==0x7FFF&&m143c==0x1234)?"PASS":"FAIL",
        (m143Read0&&m143Read1&&m143Read2&&m143r0==0x7F&&m143r1==0x40&&m143r2==0x7E)?"PASS":"FAIL",
        (m143Read3&&m143r3==0&&m143Outside)?"PASS":"FAIL",
        m143->mirroring()==Mirror::Vertical?"PASS":"FAIL",m143gate?"PASS":"FAIL");
    ok &= mapper143;

    // Phase 30: Mapper 156 / DAOU DIS23C01.  Eight 1 KiB CHR slots use
    // split low/high register writes, the lower 16 KiB PRG window is
    // switchable, and the upper 16 KiB is fixed to the last bank.
    auto m156 = makeMapper(156, 0x40000, 0x80000, 0x2000, 0, 0, true);
    uint32_t m156pLo=0,m156pHi=0,m156c0=0,m156c7=0,m156ram=0;
    m156->cpuWrite(0xC010, 9, 0);
    m156->cpuMapRead(0x8123,m156pLo); m156->cpuMapRead(0xC123,m156pHi);
    // CHR slot 0 = $123, slot 7 = $1AB.
    m156->cpuWrite(0xC000,0x23,0); m156->cpuWrite(0xC004,0x01,0);
    m156->cpuWrite(0xC00B,0xAB,0); m156->cpuWrite(0xC00F,0x01,0);
    m156->ppuMapRead(0x0056,m156c0); m156->ppuMapRead(0x1C56,m156c7);
    const bool m156Ram=m156->mapPrgRam(0x6123,m156ram,false) && m156ram==0x0123;
    const bool m156Default=m156->mirroring()==Mirror::OnescreenLo;
    m156->cpuWrite(0xC014,0,0); const bool m156V=m156->mirroring()==Mirror::Vertical;
    m156->cpuWrite(0xC014,1,0); const bool m156H=m156->mirroring()==Mirror::Horizontal;
    m156->cpuWrite(0xC014,2,0); const bool m156One=m156->mirroring()==Mirror::OnescreenLo;
    const bool m156Ignored=!m156->cpuWrite(0xC011,0xFF,0) && !m156->cpuWrite(0xBFFF,0xFF,0);
    std::vector<uint8_t> m156state; m156->saveState(m156state);
    m156->cpuWrite(0xC010,0,0); m156->cpuWrite(0xC000,0,0); m156->cpuWrite(0xC004,0,0); m156->cpuWrite(0xC014,1,0);
    const uint8_t* m156sp=m156state.data(); const uint8_t* m156se=m156sp+m156state.size();
    const bool m156load=m156->loadState(m156sp,m156se); uint32_t m156rp=0,m156rc=0;
    m156->cpuMapRead(0x8123,m156rp); m156->ppuMapRead(0x0056,m156rc);
    const bool m156gate=mapperImplementationSupported(156,0)&&!mapperImplementationSupported(156,1);
    const bool mapper156=m156pLo==0x24123&&m156pHi==0x3C123&&m156c0==0x48C56&&m156c7==0x6AC56&&
        m156Ram&&m156Default&&m156V&&m156H&&m156One&&m156Ignored&&m156load&&m156rp==0x24123&&
        m156rc==0x48C56&&m156->mirroring()==Mirror::OnescreenLo&&m156gate;
    std::printf("phase30_mapper156 prg=%s chr=%s ram=%s mirror=%s state=%s gate=%s\n",
        (m156pLo==0x24123&&m156pHi==0x3C123)?"PASS":"FAIL",
        (m156c0==0x48C56&&m156c7==0x6AC56)?"PASS":"FAIL",m156Ram?"PASS":"FAIL",
        (m156Default&&m156V&&m156H&&m156One)?"PASS":"FAIL",
        (m156load&&m156rp==0x24123&&m156rc==0x48C56)?"PASS":"FAIL",m156gate?"PASS":"FAIL");
    ok &= mapper156;


    // Phase 31: Mapper 142 / Kaiser KS7032.  Four switchable 8 KiB PRG
    // windows live at $6000-$DFFF, $E000-$FFFF is fixed to the final bank,
    // CHR is unbanked RAM, and the KS202 ASIC supplies a one-shot 16-bit
    // incrementing IRQ whose reload value is written in four nibbles.
    auto m142 = makeMapper(142, 0x40000, 0, 0, 0x2000, 0, true, Mirror::Vertical);
    auto write142Bank = [](Mapper& m, uint8_t select, uint8_t bank) {
        m.cpuWrite(0xE000, select, 0);
        m.cpuWrite(0xF000, bank, 0);
    };
    write142Bank(*m142,4,6); write142Bank(*m142,1,3);
    write142Bank(*m142,2,4); write142Bank(*m142,3,5);
    uint32_t m142p6=0,m142p8=0,m142pA=0,m142pC=0,m142pE=0,m142chrR=0,m142chrW=0;
    const bool m142r6=m142->cpuMapRead(0x6123,m142p6);
    const bool m142r8=m142->cpuMapRead(0x8123,m142p8);
    const bool m142rA=m142->cpuMapRead(0xA123,m142pA);
    const bool m142rC=m142->cpuMapRead(0xC123,m142pC);
    const bool m142rE=m142->cpuMapRead(0xE123,m142pE);
    const bool m142chrRead=m142->ppuMapRead(0x1234,m142chrR);
    const bool m142chrWrite=m142->ppuMapWrite(0x1234,m142chrW);
    uint32_t m142ram=0;
    const bool m142NoWram=!m142->mapPrgRam(0x6000,m142ram,false);

    // Reload $FFFE, enable with bit 1, then require overflow on the second
    // CPU clock.  The IRQ is one-shot and $D000 acknowledges it.
    m142->cpuWrite(0x8000,0x0E,0); m142->cpuWrite(0x9000,0x0F,0);
    m142->cpuWrite(0xA000,0x0F,0); m142->cpuWrite(0xB000,0x0F,0);
    m142->cpuWrite(0xC000,0x02,0);
    m142->clockCpu(); const bool m142IrqBefore=!m142->irqActive();
    m142->clockCpu(); const bool m142IrqAtWrap=m142->irqActive();
    m142->clockCpu(); const bool m142OneShot=m142->irqActive();
    m142->cpuWrite(0xD000,0,0); const bool m142Ack=!m142->irqActive();

    std::vector<uint8_t> m142state; m142->saveState(m142state);
    write142Bank(*m142,1,0); write142Bank(*m142,4,0);
    const uint8_t* m142sp=m142state.data(); const uint8_t* m142se=m142sp+m142state.size();
    const bool m142load=m142->loadState(m142sp,m142se); uint32_t m142rp6=0,m142rp8=0;
    m142->cpuMapRead(0x6123,m142rp6); m142->cpuMapRead(0x8123,m142rp8);
    const bool m142gate=mapperImplementationSupported(142,0)&&!mapperImplementationSupported(142,1);
    const bool mapper142=m142r6&&m142r8&&m142rA&&m142rC&&m142rE&&
        m142p6==0x0C123&&m142p8==0x06123&&m142pA==0x08123&&m142pC==0x0A123&&m142pE==0x3E123&&
        m142chrRead&&m142chrWrite&&m142chrR==0x1234&&m142chrW==0x1234&&m142NoWram&&
        m142IrqBefore&&m142IrqAtWrap&&m142OneShot&&m142Ack&&m142load&&
        m142rp6==0x0C123&&m142rp8==0x06123&&m142gate;
    std::printf("phase31_mapper142 prg=%s chr=%s irq=%s state=%s gate=%s\n",
        (m142r6&&m142r8&&m142rA&&m142rC&&m142rE&&m142p6==0x0C123&&m142p8==0x06123&&
         m142pA==0x08123&&m142pC==0x0A123&&m142pE==0x3E123)?"PASS":"FAIL",
        (m142chrRead&&m142chrWrite&&m142chrR==0x1234&&m142chrW==0x1234&&m142NoWram)?"PASS":"FAIL",
        (m142IrqBefore&&m142IrqAtWrap&&m142OneShot&&m142Ack)?"PASS":"FAIL",
        (m142load&&m142rp6==0x0C123&&m142rp8==0x06123)?"PASS":"FAIL",m142gate?"PASS":"FAIL");
    ok &= mapper142;


    // Phase 32: Mapper 147 / Sachen TCU-01.  Writes at $4102 mirrored by
    // mask $E103 latch D7,D2 as the 32 KiB PRG bank and D7-D3 as the
    // 8 KiB CHR bank.  The data latch ignores all nonmatching addresses.
    auto m147 = makeMapper(147, 0x20000, 0x20000, 0, 0, 0, true, Mirror::Vertical);
    uint32_t m147p0=0,m147c0=0;
    const bool m147Default = m147->cpuMapRead(0x8123,m147p0) && m147p0==0x0123 &&
        m147->ppuMapRead(0x0456,m147c0) && m147c0==0x0456 && m147->mirroring()==Mirror::Vertical;
    // D7=1,D2=1 => PRG bank 3. D7-D3=10101b => bank 21, wrapping to bank 5
    // on the 128 KiB CHR image used by this probe.
    const bool m147Write = m147->cpuWrite(0x4102,0xAC,0);
    uint32_t m147p=0,m147c=0;
    m147->cpuMapRead(0x9234,m147p); m147->ppuMapRead(0x0567,m147c);
    const bool m147Banks = m147p==0x19234 && m147c==0x0A567;
    // $4302 is the documented mirror; $4100/$4103 and $4202 must not latch.
    const bool m147Alias = m147->cpuWrite(0x4302,0x20,0);
    uint32_t m147pa=0,m147ca=0; m147->cpuMapRead(0x8000,m147pa); m147->ppuMapRead(0x0000,m147ca);
    const bool m147AliasBanks = m147pa==0x00000 && m147ca==0x08000;
    const bool m147Ignored = !m147->cpuWrite(0x4100,0xFF,0) && !m147->cpuWrite(0x4103,0xFF,0) &&
        !m147->cpuWrite(0x4202,0xFF,0);
    uint32_t m147pi=0,m147ci=0; m147->cpuMapRead(0x8000,m147pi); m147->ppuMapRead(0x0000,m147ci);
    const bool m147NoRam = [&]{ uint32_t r=0; return !m147->mapPrgRam(0x6000,r,false); }();
    std::vector<uint8_t> m147state; m147->saveState(m147state);
    m147->cpuWrite(0x4102,0x00,0);
    const uint8_t* m147sp=m147state.data(); const uint8_t* m147se=m147sp+m147state.size();
    const bool m147load=m147->loadState(m147sp,m147se); uint32_t m147rp=0,m147rc=0;
    m147->cpuMapRead(0x8000,m147rp); m147->ppuMapRead(0x0000,m147rc);
    const bool m147gate=mapperImplementationSupported(147,0)&&!mapperImplementationSupported(147,1);
    const bool mapper147=m147Default&&m147Write&&m147Banks&&m147Alias&&m147AliasBanks&&m147Ignored&&
        m147pi==0x00000&&m147ci==0x08000&&m147NoRam&&m147load&&m147rp==0x00000&&m147rc==0x08000&&m147gate;
    std::printf("phase32_mapper147 prg=%s chr=%s decode=%s state=%s mirror=%s gate=%s\n",
        (m147Default&&m147Banks&&m147AliasBanks)?"PASS":"FAIL",
        (m147c==0x0A567&&m147ca==0x08000)?"PASS":"FAIL",
        (m147Write&&m147Alias&&m147Ignored&&m147NoRam)?"PASS":"FAIL",
        (m147load&&m147rp==0x00000&&m147rc==0x08000)?"PASS":"FAIL",
        m147->mirroring()==Mirror::Vertical?"PASS":"FAIL",m147gate?"PASS":"FAIL");
    ok &= mapper147;

    // Phase 33: Mapper 144 / AGCI 50282.  Banking uses the Color Dreams
    // bit layout, but a board resistor forces CPU D0 high before the
    // wired-AND bus conflict with PRG-ROM.  This makes the electrical
    // result observably different from Mapper 11 when CPU D0 is low.
    auto m144 = makeMapper(144, 0x20000, 0x20000, 0, 0, 0, true, Mirror::Horizontal);
    auto m11cmp = makeMapper(11, 0x20000, 0x20000, 0, 0, 0, true, Mirror::Horizontal);
    const bool m144ConflictFlag = m144->hasBusConflicts();
    const uint8_t m144Forced = m144->resolveBusConflict(0x8000, 0x02, 0x01);
    const uint8_t m11Normal = m11cmp->resolveBusConflict(0x8000, 0x02, 0x01);
    const uint8_t m144Resolved = m144->resolveBusConflict(0x8000, 0xA2, 0xF3);
    const bool m144Electrical = m144ConflictFlag && m144Forced == 0x01 && m11Normal == 0x00 && m144Resolved == 0xA3;
    const bool m144Write = m144->cpuWrite(0x8000, m144Resolved, 0);
    uint32_t m144p=0,m144c=0;
    const bool m144PrgRead=m144->cpuMapRead(0x8123,m144p);
    const bool m144ChrRead=m144->ppuMapRead(0x0456,m144c);
    const bool m144Banks=m144PrgRead&&m144ChrRead&&m144p==0x18123&&m144c==0x14000+0x0456;
    const bool m144Ignored=!m144->cpuWrite(0x7FFF,0xFF,0);
    std::vector<uint8_t> m144state; m144->saveState(m144state);
    m144->cpuWrite(0x8000,0x00,0);
    const uint8_t* m144sp=m144state.data(); const uint8_t* m144se=m144sp+m144state.size();
    const bool m144load=m144->loadState(m144sp,m144se); uint32_t m144rp=0,m144rc=0;
    m144->cpuMapRead(0x8000,m144rp); m144->ppuMapRead(0x0000,m144rc);
    const bool m144State=m144load&&m144rp==0x18000&&m144rc==0x14000;
    const bool m144Gate=mapperImplementationSupported(144,0)&&!mapperImplementationSupported(144,1);
    const bool mapper144=m144Electrical&&m144Write&&m144Banks&&m144Ignored&&m144State&&m144Gate&&
        m144->mirroring()==Mirror::Horizontal;
    std::printf("phase33_mapper144 conflict=%s prg=%s chr=%s state=%s gate=%s\n",
        m144Electrical?"PASS":"FAIL",(m144PrgRead&&m144p==0x18123)?"PASS":"FAIL",
        (m144ChrRead&&m144c==0x14456)?"PASS":"FAIL",m144State?"PASS":"FAIL",m144Gate?"PASS":"FAIL");
    ok &= mapper144;


    // Phase 35: Mapper 243 / Sachen SA-020A (74LS374NA).  The board uses
    // the same $4100/$4101 8-register ASIC interface as Mapper 150, but its
    // CHR wiring is R2->A13, R4->A14 and R6.1-0->A16-A15.
    auto m243 = makeMapper(243, 0x20000, 0x20000, 0, 0, 0, false);
    auto write243 = [](Mapper& m, uint8_t reg, uint8_t value) {
        m.cpuWrite(0x4120, reg, 0);   // $4100 alias under mask $C101
        m.cpuWrite(0x4121, value, 0); // $4101 alias
    };
    write243(*m243,5,3); // PRG bank 3
    write243(*m243,2,1); // CHR bit 0
    write243(*m243,4,1); // CHR bit 1
    write243(*m243,6,2); // CHR bits 3-2 = 10 -> bank 11
    uint32_t m243p=0,m243c=0;
    m243->cpuMapRead(0x8123,m243p); m243->ppuMapRead(0x0456,m243c);

    // All three data bits are readable; upper CPU-bus bits remain open bus.
    m243->cpuWrite(0x4100,6,0);
    uint8_t m243read=0xA8; const bool m243readOk=m243->cpuReadRegister(0x4101,m243read);
    const bool m243Ignored=!m243->cpuWrite(0x4200,0x07,0)&&!m243->cpuWrite(0x4201,0x07,0);

    NametableSource m243src=NametableSource::ChrRom; uint32_t m243nt[4]={};
    write243(*m243,7,0);
    for(unsigned i=0;i<4;++i)m243->mapNametable(uint16_t(0x2000+i*0x400),m243src,m243nt[i]);
    const bool m243Custom=m243nt[0]==0&&m243nt[1]==0&&m243nt[2]==0&&m243nt[3]==0x400;
    write243(*m243,7,2); const bool m243H=m243->mirroring()==Mirror::Horizontal;
    write243(*m243,7,4); const bool m243V=m243->mirroring()==Mirror::Vertical;
    write243(*m243,7,6); const bool m243One=m243->mirroring()==Mirror::OnescreenHi;

    std::vector<uint8_t> m243state; m243->saveState(m243state);
    write243(*m243,5,0); write243(*m243,2,0); write243(*m243,4,0); write243(*m243,6,0); write243(*m243,7,0);
    const uint8_t* m243sp=m243state.data(); const uint8_t* m243se=m243sp+m243state.size();
    const bool m243load=m243->loadState(m243sp,m243se); uint32_t m243rp=0,m243rc=0;
    m243->cpuMapRead(0x8000,m243rp); m243->ppuMapRead(0x0000,m243rc);
    const bool m243gate=mapperImplementationSupported(243,0)&&!mapperImplementationSupported(243,1);
    const bool mapper243=m243p==0x18123&&m243c==0x16456&&m243readOk&&m243read==0xAA&&m243Ignored&&
        m243Custom&&m243H&&m243V&&m243One&&m243load&&m243rp==0x18000&&m243rc==0x16000&&
        m243->mirroring()==Mirror::OnescreenHi&&m243gate;
    std::printf("phase35_mapper243 prg=%s chr=%s read=%s mirror=%s state=%s gate=%s\n",
        m243p==0x18123?"PASS":"FAIL",m243c==0x16456?"PASS":"FAIL",
        (m243readOk&&m243read==0xAA&&m243Ignored)?"PASS":"FAIL",
        (m243Custom&&m243H&&m243V&&m243One)?"PASS":"FAIL",
        (m243load&&m243rp==0x18000&&m243rc==0x16000&&m243->mirroring()==Mirror::OnescreenHi)?"PASS":"FAIL",
        m243gate?"PASS":"FAIL");
    ok &= mapper243;


    // Phase 44: Mapper 201 / BMC 21-in-1 is the standardized mapper number
    // for the same hardware historically assigned to Mapper 54.  Keep one
    // implementation and verify the two IDs remain behaviorally identical.
    auto m201 = makeMapper(201, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Horizontal);
    auto m54alias = makeMapper(54, 0x20000, 0x10000, 0, 0, 0, false, Mirror::Horizontal);
    const bool m201Boot = m201 && m54alias;
    uint32_t m201p0=0,m201c0=0,m54p0=0,m54c0=0;
    const bool m201BootMaps = m201Boot &&
        m201->cpuMapRead(0x8123,m201p0) && m54alias->cpuMapRead(0x8123,m54p0) &&
        m201->ppuMapRead(0x0456,m201c0) && m54alias->ppuMapRead(0x0456,m54c0) &&
        m201p0==m54p0 && m201c0==m54c0;

    // Address bits, not data, select both banks.  $8006 selects PRG bank 2
    // (A1:A0) and CHR bank 6 (A2:A0); the two mapper numbers must agree.
    const bool m201Write = m201Boot && m201->cpuWrite(0x8006,0x00,0) && m54alias->cpuWrite(0x8006,0xFF,0);
    uint32_t m201p=0,m201c=0,m54aliasp=0,m54aliasc=0;
    const bool m201Banks = m201Write &&
        m201->cpuMapRead(0x9234,m201p) && m54alias->cpuMapRead(0x9234,m54aliasp) &&
        m201->ppuMapRead(0x0567,m201c) && m54alias->ppuMapRead(0x0567,m54aliasc) &&
        m201p==0x11234 && m54aliasp==m201p && m201c==0x0C567 && m54aliasc==m201c;

    // Data is disconnected, so changing only the data byte at the same CPU
    // address cannot alter either bank selection.
    m201->cpuWrite(0x8003,0x00,0); m54alias->cpuWrite(0x8003,0xFF,0);
    uint32_t m201pd=0,m201cd=0,m54pd=0,m54cd=0;
    m201->cpuMapRead(0x8000,m201pd); m54alias->cpuMapRead(0x8000,m54pd);
    m201->ppuMapRead(0x0000,m201cd); m54alias->ppuMapRead(0x0000,m54cd);
    const bool m201DataIgnored = m201pd==m54pd && m201cd==m54cd && m201pd==0x18000 && m201cd==0x06000;

    std::vector<uint8_t> m201state; m201->saveState(m201state);
    m201->cpuWrite(0x8000,0x00,0);
    const uint8_t* m201sp=m201state.data(); const uint8_t* m201se=m201sp+m201state.size();
    const bool m201load=m201->loadState(m201sp,m201se); uint32_t m201rp=0,m201rc=0;
    m201->cpuMapRead(0x8000,m201rp); m201->ppuMapRead(0x0000,m201rc);
    const bool m201State=m201load&&m201rp==0x18000&&m201rc==0x06000;
    const bool m201Gate=mapperImplementationSupported(201,0)&&!mapperImplementationSupported(201,1);
    const bool mapper201=m201BootMaps&&m201Banks&&m201DataIgnored&&m201State&&m201Gate;
    std::printf("phase44_mapper201 alias=%s banks=%s data=%s state=%s gate=%s\n",
        m201BootMaps?"PASS":"FAIL",m201Banks?"PASS":"FAIL",m201DataIgnored?"PASS":"FAIL",
        m201State?"PASS":"FAIL",m201Gate?"PASS":"FAIL");
    ok &= mapper201;

    // Phase 45: Mapper 171 / Kaiser KS-7058.  PRG is fixed at 32 KiB;
    // $F000/$F080 aliases select the two 4 KiB CHR halves independently.
    auto m171 = makeMapper(171, 0x8000, 0x10000, 0, 0, 0, true, Mirror::Vertical);
    uint32_t m171p0=0,m171p1=0,m171c0=0,m171c1=0;
    const bool m171Fixed = m171->cpuMapRead(0x8000,m171p0) && m171->cpuMapRead(0xFFFF,m171p1) &&
        m171p0==0x0000 && m171p1==0x7FFF && m171->mirroring()==Mirror::Vertical;
    const bool m171NoRam = !m171->mapPrgRam(0x6000,m171p0,false);

    // Mask $F080: A7 chooses the CHR half; all other low address bits alias.
    const bool m171Write0 = m171->cpuWrite(0xF037,0x03,0);
    const bool m171Write1 = m171->cpuWrite(0xF1FF,0x05,0); // -> $F080 alias
    const bool m171Ignore = !m171->cpuWrite(0xE080,0x07,0);
    const bool m171Chr = m171Write0 && m171Write1 && m171Ignore &&
        m171->ppuMapRead(0x0123,m171c0) && m171->ppuMapRead(0x1456,m171c1) &&
        m171c0==0x03123 && m171c1==0x05456;

    std::vector<uint8_t> m171state; m171->saveState(m171state);
    m171->cpuWrite(0xF000,0x01,0); m171->cpuWrite(0xF080,0x02,0);
    const uint8_t* m171sp=m171state.data(); const uint8_t* m171se=m171sp+m171state.size();
    const bool m171load=m171->loadState(m171sp,m171se); uint32_t m171r0=0,m171r1=0;
    const bool m171State=m171load && m171->ppuMapRead(0x0000,m171r0) && m171->ppuMapRead(0x1000,m171r1) &&
        m171r0==0x03000 && m171r1==0x05000;
    m171->reset(true); uint32_t m171reset0=0,m171reset1=0;
    const bool m171Reset=m171->ppuMapRead(0x0000,m171reset0)&&m171->ppuMapRead(0x1000,m171reset1)&&
        m171reset0==0&&m171reset1==0;
    const bool m171Gate=mapperImplementationSupported(171,0)&&!mapperImplementationSupported(171,1);
    const bool mapper171=m171Fixed&&m171NoRam&&m171Chr&&m171State&&m171Reset&&m171Gate;
    std::printf("phase45_mapper171 fixed=%s chr=%s decode=%s state=%s reset=%s gate=%s\n",
        (m171Fixed&&m171NoRam)?"PASS":"FAIL",m171Chr?"PASS":"FAIL",m171Ignore?"PASS":"FAIL",
        m171State?"PASS":"FAIL",m171Reset?"PASS":"FAIL",m171Gate?"PASS":"FAIL");
    ok &= mapper171;


    // Phase 46: Mapper 193 / NTDEC TC-112 (Fighting Hero). Four registers
    // mirrored through $6000-$7FFF select one 8 KiB PRG bank and a
    // 4 KiB + 2 KiB + 2 KiB CHR layout. The final 24 KiB of PRG is fixed
    // and the physical board is hardwired to vertical mirroring.
    auto m193 = makeMapper(193, 0x20000, 0x20000, 0, 0, 0, true, Mirror::Horizontal);
    uint32_t m193p0=0,m193p1=0,m193p2=0,m193p3=0;
    const bool m193Boot = m193->cpuMapRead(0x8000,m193p0) && m193->cpuMapRead(0xA000,m193p1) &&
        m193->cpuMapRead(0xC000,m193p2) && m193->cpuMapRead(0xE000,m193p3) &&
        m193p0==0x00000 && m193p1==0x1A000 && m193p2==0x1C000 && m193p3==0x1E000 &&
        m193->mirroring()==Mirror::Vertical;
    const bool m193Writes = m193->cpuWrite(0x6003,0x03,0) &&
        m193->cpuWrite(0x7FFC,0x14,0) && // register 0 alias: 4 KiB CHR bank 5
        m193->cpuWrite(0x7FFD,0x0C,0) && // register 1 alias: 2 KiB CHR bank 6
        m193->cpuWrite(0x7FFE,0x12,0) && // register 2 alias: 2 KiB CHR bank 9
        !m193->cpuWrite(0x5FFF,0xFF,0) && !m193->cpuWrite(0x8000,0xFF,0);
    uint32_t m193pb=0,m193c0=0,m193c1=0,m193c2=0;
    const bool m193Banks = m193Writes && m193->cpuMapRead(0x8123,m193pb) &&
        m193->ppuMapRead(0x0123,m193c0) && m193->ppuMapRead(0x1456,m193c1) &&
        m193->ppuMapRead(0x1ABC,m193c2) &&
        m193pb==0x06123 && m193c0==0x05123 && m193c1==0x03456 && m193c2==0x04ABC;
    uint32_t m193ram=0;
    const bool m193NoRam = !m193->mapPrgRam(0x6000,m193ram,false);
    std::vector<uint8_t> m193state; m193->saveState(m193state);
    m193->cpuWrite(0x6003,0x01,0); m193->cpuWrite(0x6000,0x00,0);
    const uint8_t* m193sp=m193state.data(); const uint8_t* m193se=m193sp+m193state.size();
    const bool m193load=m193->loadState(m193sp,m193se); uint32_t m193rp=0,m193rc=0;
    const bool m193State=m193load && m193->cpuMapRead(0x8000,m193rp) && m193->ppuMapRead(0x0000,m193rc) &&
        m193rp==0x06000 && m193rc==0x05000;
    m193->reset(false); uint32_t m193soft=0; m193->cpuMapRead(0x8000,m193soft);
    m193->reset(true); uint32_t m193hard=0,m193hardChr=0;
    const bool m193Reset=m193soft==0x06000 && m193->cpuMapRead(0x8000,m193hard) &&
        m193->ppuMapRead(0x0000,m193hardChr) && m193hard==0 && m193hardChr==0 &&
        m193->mirroring()==Mirror::Vertical;
    const bool m193Gate=mapperImplementationSupported(193,0)&&!mapperImplementationSupported(193,1);
    const bool mapper193=m193Boot&&m193Banks&&m193NoRam&&m193State&&m193Reset&&m193Gate;
    std::printf("phase46_mapper193 fixed=%s banks=%s decode=%s state=%s reset=%s gate=%s\n",
        (m193Boot&&m193NoRam)?"PASS":"FAIL",m193Banks?"PASS":"FAIL",m193Writes?"PASS":"FAIL",
        m193State?"PASS":"FAIL",m193Reset?"PASS":"FAIL",m193Gate?"PASS":"FAIL");
    ok &= mapper193;


    // Phase 48: larger multicart bundle -- mappers 225, 226, 230, 235 and 236.
    // These cover five distinct discrete-latch families in one regression pass.
    bool phase48_225=false, phase48_226=false, phase48_227=false, phase48_230=false, phase48_235=false, phase48_236=false, phase48_237=false;
    {
        // Mapper 225 / BMC 72/110-in-1: address latch plus four 4-bit RAM cells.
        auto m = makeMapper(225, 0x200000, 0x100000, 0, 0, 0, false, Mirror::Horizontal);
        const bool wr = m->cpuWrite(0xE045,0x00,0); // o=$6045 -> banks $40/$41, CHR $45, H
        uint32_t p0=0,p1=0,c=0; uint8_t r=0xA0;
        const bool maps = wr && m->cpuMapRead(0x8123,p0) && m->cpuMapRead(0xC123,p1) &&
            m->ppuMapRead(0x0456,c) && p0==0x100123 && p1==0x104123 && c==0x8A456 &&
            m->mirroring()==Mirror::Horizontal;
        const bool ram = m->cpuWrite(0x5FFE,0xBC,0) && m->cpuReadRegister(0x5802,r) && r==0xAC;
        std::vector<uint8_t> st; m->saveState(st); m->cpuWrite(0x8000,0,0); m->cpuWrite(0x5802,1,0);
        const uint8_t* sp=st.data(); const uint8_t* se=sp+st.size(); uint32_t rp=0,rc=0; uint8_t rr=0xD0;
        const bool state=m->loadState(sp,se)&&m->cpuMapRead(0x8000,rp)&&m->ppuMapRead(0,rc)&&
            m->cpuReadRegister(0x5802,rr)&&rp==0x100000&&rc==0x8A000&&rr==0xDC;
        phase48_225=maps&&ram&&state&&mapperImplementationSupported(225,0)&&!mapperImplementationSupported(225,1);
    }
    {
        // Mapper 226 / 76-in-1: two data registers, CHR-RAM fixed.
        auto m = makeMapper(226, 0x200000, 0, 0, 0x2000);
        m->cpuWrite(0x8001,0x01,0); m->cpuWrite(0x8000,0x65,0); // bank 69, mirrored, vertical
        uint32_t p0=0,p1=0,c=0,w=0;
        const bool mirrored=m->cpuMapRead(0x8000,p0)&&m->cpuMapRead(0xC000,p1)&&p0==0x114000&&p1==0x114000&&
            m->mirroring()==Mirror::Vertical;
        m->cpuWrite(0x8000,0x05,0); // bank 69 in 32K mode -> 68/69, horizontal
        const bool split=m->cpuMapRead(0x8000,p0)&&m->cpuMapRead(0xC000,p1)&&p0==0x110000&&p1==0x114000&&
            m->mirroring()==Mirror::Horizontal;
        const bool chr=m->ppuMapRead(0x1234,c)&&m->ppuMapWrite(0x1234,w)&&c==0x1234&&w==0x1234;
        m->reset(false); uint32_t rr0=0,rr1=0;
        const bool soft=m->cpuMapRead(0x8000,rr0)&&m->cpuMapRead(0xC000,rr1)&&rr0==0&&rr1==0x4000;
        phase48_226=mirrored&&split&&chr&&soft&&mapperImplementationSupported(226,0)&&!mapperImplementationSupported(226,1);
    }
    {
        // Mapper 227 / BMC 1200-in-1 legacy path: address-latched UNROM/NROM modes, CHR-RAM.
        auto m = makeMapper(227, 0x100000, 0, 0, 0x2000);
        // $8382 => o=$0382: bank=((o>>2)&1F)|((o&100)>>3)=0x20, NROM-256, H.
        m->cpuWrite(0x8383,0,0); // bit0=1 -> adjacent 16K banks (NROM-256)
        uint32_t p0=0,p1=0,c=0,w=0;
        const bool nrom=m->cpuMapRead(0x8000,p0)&&m->cpuMapRead(0xC000,p1)&&
            p0==0x80000&&p1==0x84000&&m->mirroring()==Mirror::Horizontal;
        // UNROM mode with L=1 fixes the upper half to inner bank 7.
        m->cpuWrite(0x8200,0,0); uint32_t u0=0,u1=0;
        const bool unrom=m->cpuMapRead(0x8000,u0)&&m->cpuMapRead(0xC000,u1)&&u1==0x1C000;
        const bool chr=m->ppuMapRead(0x1234,c)&&m->ppuMapWrite(0x1234,w)&&c==0x1234&&w==0x1234;
        m->reset(false); uint32_t r0=0,r1=0;
        const bool reset=m->cpuMapRead(0x8000,r0)&&m->cpuMapRead(0xC000,r1)&&r0==0&&r1==0;
        phase48_227=nrom&&unrom&&chr&&reset&&mapperImplementationSupported(227,0)&&!mapperImplementationSupported(227,1);
    }
    {
        // Mapper 230 / Contra + 22-in-1: reset toggles between ROM chips/modes.
        auto m = makeMapper(230, 0xA0000, 0, 0, 0x2000);
        uint32_t a=0,b=0;
        const bool contraBoot=m->cpuMapRead(0x8000,a)&&m->cpuMapRead(0xC000,b)&&a==0&&b==0x1C000&&m->mirroring()==Mirror::Vertical;
        m->cpuWrite(0x8000,3,0); uint32_t c=0; m->cpuMapRead(0x8000,c);
        const bool contraBank=c==0x0C000;
        m->reset(false); uint32_t d=0,e=0; m->cpuMapRead(0x8000,d); m->cpuMapRead(0xC000,e);
        const bool multiBoot=d==0x20000&&e==0x24000&&m->mirroring()==Mirror::Horizontal;
        m->cpuWrite(0x8000,0x03,0); m->cpuMapRead(0x8000,d); m->cpuMapRead(0xC000,e);
        const bool multiBank=d==0x28000&&e==0x2C000;
        m->reset(false); m->cpuMapRead(0x8000,d); m->cpuMapRead(0xC000,e);
        const bool backToContra=d==0&&e==0x1C000&&m->mirroring()==Mirror::Vertical;
        phase48_230=contraBoot&&contraBank&&multiBoot&&multiBank&&backToContra&&
            mapperImplementationSupported(230,0)&&!mapperImplementationSupported(230,1);
    }
    {
        // Mapper 235 / Golden Game: address bits form PRG bank/mode/mirroring.
        auto m = makeMapper(235, 0x400000, 0, 0, 0x2000);
        m->cpuWrite(0xA003,0,0); // o=$2003 -> bank 6/7, horizontal
        uint32_t p0=0,p1=0,c=0,w=0;
        const bool bank=m->cpuMapRead(0x8000,p0)&&m->cpuMapRead(0xC000,p1)&&p0==0x18000&&p1==0x1C000&&
            m->mirroring()==Mirror::Horizontal;
        m->cpuWrite(0x8400,0,0); // o=$0400 -> one-screen low
        const bool one=m->mirroring()==Mirror::OnescreenLo;
        const bool chr=m->ppuMapRead(0x0456,c)&&m->ppuMapWrite(0x0456,w)&&c==0x0456&&w==0x0456;
        std::vector<uint8_t> st; m->saveState(st); m->cpuWrite(0x8000,0,0);
        const uint8_t* sp=st.data(); const uint8_t* se=sp+st.size();
        const bool state=m->loadState(sp,se)&&m->mirroring()==Mirror::OnescreenLo;
        phase48_235=bank&&one&&chr&&state&&mapperImplementationSupported(235,0)&&!mapperImplementationSupported(235,1);
    }
    {
        // Mapper 236 / Realtec: exercise both documented CHR-ROM and CHR-RAM wiring variants.
        auto rom = makeMapper(236, 0x20000, 0x10000, 0, 0);
        rom->cpuWrite(0x8025,0,0); // lower latch: CHR=5, horizontal
        rom->cpuWrite(0xE012,0,0); // upper latch: UNROM bank 2 / fixed 7
        uint32_t p0=0,p1=0,c=0;
        const bool romPath=rom->cpuMapRead(0x8000,p0)&&rom->cpuMapRead(0xC000,p1)&&rom->ppuMapRead(0,c)&&
            p0==0x08000&&p1==0x1C000&&c==0x0A000&&rom->mirroring()==Mirror::Horizontal;
        auto ram = makeMapper(236, 0x100000, 0, 0, 0x2000);
        ram->cpuWrite(0x8023,0,0); // outer PRG bits=3, H
        ram->cpuWrite(0xE035,0,0); // bank 29, NROM-128 mirrored
        uint32_t q0=0,q1=0,cr=0,cw=0;
        const bool ramPath=ram->cpuMapRead(0x8000,q0)&&ram->cpuMapRead(0xC000,q1)&&q0==0x74000&&q1==0x74000&&
            ram->ppuMapRead(0x1234,cr)&&ram->ppuMapWrite(0x1234,cw)&&cr==0x1234&&cw==0x1234&&
            ram->mirroring()==Mirror::Horizontal;
        phase48_236=romPath&&ramPath&&mapperImplementationSupported(236,0)&&!mapperImplementationSupported(236,1);
    }
    {
        // Mapper 237 / Teletubbies 420-in-1: address outer bits, data inner/mode/lock.
        auto m = makeMapper(237, 0x100000, 0, 0, 0x2000);
        // Unlocked write: latch A2 supplies outer bank bit; data selects inner=3, H mirroring.
        m->cpuWrite(0x8004,0x23,0);
        uint32_t p0=0,p1=0,c=0,w=0;
        const bool unrom=m->cpuMapRead(0x8000,p0)&&m->cpuMapRead(0xC000,p1)&&
            p0==0x8C000&&p1==0x9C000&&m->mirroring()==Mirror::Horizontal;
        // Lock via address A1. Subsequent writes may change only bbb (D2-D0).
        m->cpuWrite(0x8006,0xE5,0); // lock, NROM mode, bank outer|5
        uint32_t l0=0,l1=0; m->cpuMapRead(0x8000,l0); m->cpuMapRead(0xC000,l1);
        m->cpuWrite(0x8000,0x00,0); // locked: only low 3 bits change
        uint32_t q0=0,q1=0; m->cpuMapRead(0x8000,q0); m->cpuMapRead(0xC000,q1);
        const bool lock=(l0!=q0||l1!=q1)&&m->mirroring()==Mirror::Horizontal;
        const bool chr=m->ppuMapRead(0x0456,c)&&m->ppuMapWrite(0x0456,w)&&c==0x0456&&w==0x0456;
        m->reset(false); uint32_t r0=0,r1=0;
        const bool reset=m->cpuMapRead(0x8000,r0)&&m->cpuMapRead(0xC000,r1)&&r0==0&&r1==0x1C000&&m->mirroring()==Mirror::Vertical;
        phase48_237=unrom&&lock&&chr&&reset&&mapperImplementationSupported(237,0)&&!mapperImplementationSupported(237,1);
    }
    const bool phase48 = phase48_225&&phase48_226&&phase48_227&&phase48_230&&phase48_235&&phase48_236&&phase48_237;
    std::printf("phase48_multicart_bundle 225=%s 226=%s 227=%s 230=%s 235=%s 236=%s 237=%s\n",
        phase48_225?"PASS":"FAIL",phase48_226?"PASS":"FAIL",phase48_227?"PASS":"FAIL",
        phase48_230?"PASS":"FAIL",phase48_235?"PASS":"FAIL",phase48_236?"PASS":"FAIL",phase48_237?"PASS":"FAIL");
    ok &= phase48;

    // Phase 49 bundled compatibility pass: one new discrete mapper plus
    // three documented mapper-number aliases of existing hardware.
    bool p49_212=false,p49_179=false,p49_182=false,p49_255=false;
    {
        auto m=makeMapper(212,0x40000,0x20000,0,0,0,false,Mirror::Vertical);
        m->cpuWrite(0x800D,0x00,0); // A14=0: mirrored 16K bank 5, CHR 5, H mirror.
        uint32_t p0=0,p1=0,c=0;
        uint8_t ob=0x35, miss=0x66;
        const bool mode16=m->cpuMapRead(0x8123,p0)&&m->cpuMapRead(0xC456,p1)&&m->ppuMapRead(0x0456,c)&&
            p0==0x14123&&p1==0x14456&&c==0x0A456&&m->mirroring()==Mirror::Horizontal;
        const bool readback=m->cpuReadRegister(0x6000,ob)&&ob==0xB5&&!m->cpuReadRegister(0x6010,miss)&&miss==0x66;
        m->cpuWrite(0xC00B,0xFF,0); // A14=1: 32K bank 1, CHR 3, H mirror; data ignored.
        const bool mode32=m->cpuMapRead(0x8000,p0)&&m->cpuMapRead(0xC000,p1)&&m->ppuMapRead(0,c)&&
            p0==0x08000&&p1==0x0C000&&c==0x06000;
        std::vector<uint8_t> st;m->saveState(st);m->cpuWrite(0x8000,0,0);
        const uint8_t* sp=st.data();const uint8_t* se=sp+st.size();uint32_t rp=0;
        const bool state=m->loadState(sp,se)&&m->cpuMapRead(0x8000,rp)&&rp==0x08000&&m->mirroring()==Mirror::Horizontal;
        p49_212=mode16&&readback&&mode32&&state&&mapperImplementationSupported(212,0)&&!mapperImplementationSupported(212,1);
    }
    {
        auto a=makeMapper(176,0x200000,0x80000,0x2000,0,0,false,Mirror::Vertical);
        auto b=makeMapper(179,0x200000,0x80000,0x2000,0,0,false,Mirror::Vertical);
        const uint16_t wa[]={0x5000,0x5001,0x8000,0x8001,0xA000};
        const uint8_t wd[]={0x03,0x12,0x06,0x05,0x01};
        for(int i=0;i<5;++i){a->cpuWrite(wa[i],wd[i],i*2);b->cpuWrite(wa[i],wd[i],i*2);}
        uint32_t ap0=0,bp0=0,ac=0,bc=0;
        p49_179=a->cpuMapRead(0x8123,ap0)&&b->cpuMapRead(0x8123,bp0)&&a->ppuMapRead(0x0456,ac)&&b->ppuMapRead(0x0456,bc)&&
            ap0==bp0&&ac==bc&&a->mirroring()==b->mirroring()&&mapperImplementationSupported(179,0)&&mapperImplementationSupported(179,5)&&!mapperImplementationSupported(179,6);
    }
    {
        auto a=makeMapper(114,0x80000,0x40000,0x2000,0,0,false,Mirror::Vertical);
        auto b=makeMapper(182,0x80000,0x40000,0x2000,0,0,false,Mirror::Vertical);
        const uint16_t wa[]={0x8000,0x8001,0xA000,0xC000,0xC001,0xE001};
        const uint8_t wd[]={0x06,0x03,0x01,0x02,0x00,0x00};
        for(int i=0;i<6;++i){a->cpuWrite(wa[i],wd[i],i*2);b->cpuWrite(wa[i],wd[i],i*2);}
        uint32_t ap=0,bp=0,ac=0,bc=0;
        p49_182=a->cpuMapRead(0x8123,ap)&&b->cpuMapRead(0x8123,bp)&&a->ppuMapRead(0x0456,ac)&&b->ppuMapRead(0x0456,bc)&&
            ap==bp&&ac==bc&&a->mirroring()==b->mirroring()&&mapperImplementationSupported(182,0)&&mapperImplementationSupported(182,1)&&!mapperImplementationSupported(182,2);
    }
    {
        auto a=makeMapper(225,0x200000,0x100000,0,0,0,false,Mirror::Vertical);
        auto b=makeMapper(255,0x200000,0x100000,0,0,0,false,Mirror::Vertical);
        a->cpuWrite(0xE045,0,0);b->cpuWrite(0xE045,0,0);a->cpuWrite(0x5802,0x0C,0);b->cpuWrite(0x5802,0x0C,0);
        uint32_t ap=0,bp=0,ac=0,bc=0;uint8_t ar=0xA0,br=0xA0;
        const bool amap=a->cpuMapRead(0x8123,ap), bmap=b->cpuMapRead(0x8123,bp);
        const bool achr=a->ppuMapRead(0x0456,ac), bchr=b->ppuMapRead(0x0456,bc);
        const bool areg=a->cpuReadRegister(0x5802,ar), breg=b->cpuReadRegister(0x5802,br);
        const bool gate255=mapperImplementationSupported(255,0)&&!mapperImplementationSupported(255,1);
        p49_255=amap&&bmap&&achr&&bchr&&areg&&breg&&ap==bp&&ac==bc&&ar==br&&a->mirroring()==b->mirroring()&&gate255;
    }
    const bool phase49=p49_212&&p49_179&&p49_182&&p49_255;
    std::printf("phase49_mapper_bundle 212=%s 179=%s 182=%s 255=%s\n",p49_212?"PASS":"FAIL",p49_179?"PASS":"FAIL",p49_182?"PASS":"FAIL",p49_255?"PASS":"FAIL");
    ok &= phase49;


    // Action 53 / mapper 28 PRG decoder: exhaustively compare the mapper
    // implementation against the published reference formula for all 64
    // mode values, 256 outer-bank values, 16 inner-bank values, and both
    // CPU A14 states. Use an 8 MiB PRG size so no decoded address aliases.
    auto m28ref = makeMapper(28, 0x800000, 0, 0, 0);
    auto calcM28Ref = [](uint16_t address, uint8_t bankMode, uint8_t outerBank, uint8_t currentBank) -> uint16_t {
        static constexpr uint8_t masks[4] = {0x01, 0x03, 0x07, 0x0F};
        const uint8_t cpuA14 = uint8_t((address >> 14) & 1);
        uint16_t outer = uint16_t(outerBank) << 1;
        uint8_t mode = uint8_t(bankMode >> 2);
        if (((mode ^ cpuA14) & 0x03) == 0x02) mode = 0;
        uint16_t current = currentBank;
        if ((mode & 0x02) == 0) current = uint16_t((current << 1) | cpuA14);
        const uint16_t mask = masks[(mode >> 2) & 3];
        return uint16_t((current & mask) | (outer & ~mask));
    };
    bool m28Exact = true;
    for (unsigned mode = 0; mode < 64 && m28Exact; ++mode) {
        m28ref->cpuWrite(0x5000, 0x80, 0);
        m28ref->cpuWrite(0x8000, uint8_t(mode), 0);
        for (unsigned outer = 0; outer < 256 && m28Exact; ++outer) {
            m28ref->cpuWrite(0x5000, 0x81, 0);
            m28ref->cpuWrite(0x8000, uint8_t(outer), 0);
            for (unsigned inner = 0; inner < 16 && m28Exact; ++inner) {
                m28ref->cpuWrite(0x5000, 0x01, 0);
                m28ref->cpuWrite(0x8000, uint8_t(inner), 0);
                for (uint16_t address : {uint16_t(0x8000), uint16_t(0xC000)}) {
                    uint32_t mapped = 0;
                    const uint16_t expectedBank = calcM28Ref(address, uint8_t(mode), uint8_t(outer), uint8_t(inner));
                    const uint32_t expected = uint32_t(expectedBank) * 0x4000u;
                    if (!m28ref->cpuMapRead(address, mapped) || mapped != expected) {
                        m28Exact = false;
                        break;
                    }
                }
            }
        }
    }
    std::printf("mapper28_reference_exhaustive=%s combinations=%u\n",
        m28Exact?"PASS":"FAIL", 64u*256u*16u*2u);
    ok &= m28Exact;

    // Namco 163 external WRAM write-enable decode.  The four high bits of
    // $F800 must equal %0100 exactly; merely having bit 6 set is insufficient.
    // Low bits A-D independently protect the four 2 KiB WRAM windows.
    auto n163wp = makeMapper(19, 0x80000, 0x40000, 0x2000, 0);
    uint32_t n163Ram = 0;
    n163wp->cpuWrite(0xF800, 0x40, 0);
    const bool n163Write40 = n163wp->mapPrgRam(0x6000, n163Ram, true) && n163Ram == 0;
    n163wp->cpuWrite(0xF800, 0x50, 0);
    const bool n163Reject50 = !n163wp->mapPrgRam(0x6000, n163Ram, true);
    n163wp->cpuWrite(0xF800, 0x60, 0);
    const bool n163Reject60 = !n163wp->mapPrgRam(0x6000, n163Ram, true);
    n163wp->cpuWrite(0xF800, 0x70, 0);
    const bool n163Reject70 = !n163wp->mapPrgRam(0x6000, n163Ram, true);
    n163wp->cpuWrite(0xF800, 0x41, 0);
    const bool n163Protect0 = !n163wp->mapPrgRam(0x6000, n163Ram, true) &&
        n163wp->mapPrgRam(0x6800, n163Ram, true) && n163Ram == 0x0800;
    const bool n163RamProtect = n163Write40 && n163Reject50 && n163Reject60 && n163Reject70 && n163Protect0;
    std::printf("n163_wram_protect=%s exact_upper_nibble=%s segment_bits=%s\n",
        n163RamProtect?"PASS":"FAIL",
        (n163Write40&&n163Reject50&&n163Reject60&&n163Reject70)?"PASS":"FAIL",
        n163Protect0?"PASS":"FAIL");
    ok &= n163RamProtect;

    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runMapperConformanceProbe(); }
#endif
