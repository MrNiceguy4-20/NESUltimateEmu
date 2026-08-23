#include "Cartridge.hpp"
#include "Bus.hpp"
#include "RomDatabase.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {
std::filesystem::path tempRoot()
{
    std::error_code ec;
    auto root = std::filesystem::temp_directory_path(ec);
    if (ec) root = ".";
    root /= "nesultimate_cart_probe";
    std::filesystem::create_directories(root, ec);
    return root;
}

bool writeRom(const std::filesystem::path& path, std::array<uint8_t, 16> header,
              std::size_t prgSize, std::size_t chrSize,
              const std::vector<uint8_t>* trainer = nullptr,
              bool bankPattern = false)
{
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(header.data()), header.size());
    if (trainer) f.write(reinterpret_cast<const char*>(trainer->data()), trainer->size());
    std::vector<uint8_t> prg(prgSize, 0xEA);
    if (bankPattern) {
        for (std::size_t i = 0; i < prg.size(); ++i)
            prg[i] = static_cast<uint8_t>(i / 0x4000);
    }
    f.write(reinterpret_cast<const char*>(prg.data()), static_cast<std::streamsize>(prg.size()));
    std::vector<uint8_t> chr(chrSize, 0);
    if (!chr.empty()) f.write(reinterpret_cast<const char*>(chr.data()), static_cast<std::streamsize>(chr.size()));
    return bool(f);
}

std::array<uint8_t, 16> baseHeader(uint8_t prg16k, uint8_t chr8k)
{
    std::array<uint8_t, 16> h{};
    h[0] = 'N'; h[1] = 'E'; h[2] = 'S'; h[3] = 0x1A;
    h[4] = prg16k; h[5] = chr8k;
    return h;
}
}

int runCartridgeConformanceProbe()
{
    bool ok = true;
    const auto root = tempRoot();

    // Archaic/dirty iNES: classic post-header garbage must not contribute the
    // upper mapper nibble or byte-8 RAM extension.
    auto archaic = baseHeader(2, 0);
    archaic[6] = 0x20; // mapper 2 low nibble
    archaic[7] = 0x44; // 'D': garbage high mapper nibble + archaic version bits
    archaic[8] = 0x69; // 'i': must not mean 105 x 8 KiB PRG RAM
    archaic[12] = 'u'; archaic[13] = 'd'; archaic[14] = 'e'; archaic[15] = '!';
    const auto archaicPath = root / "archaic.nes";
    const bool archaicWritten = writeRom(archaicPath, archaic, 0x8000, 0);
    Cartridge archaicCart;
    const bool archaicLoaded = archaicWritten && archaicCart.loadFromFile(archaicPath.string());
    const bool archaicMapper = archaicLoaded && archaicCart.mapper() == 2;
    const bool archaicRam = archaicLoaded && archaicCart.prgRamSize() == 0x2000;
    std::printf("archaic_mapper=%s ram=%s mapper=%u prgram=%zu\n",
        archaicMapper ? "PASS" : "FAIL", archaicRam ? "PASS" : "FAIL",
        unsigned(archaicCart.mapper()), archaicCart.prgRamSize());
    ok &= archaicMapper && archaicRam;

    // Phase 30 hash database: mapper 53 has two incompatible physical PRG
    // dump orders. A CRC32 signature over the first 32 KiB selects the
    // EPROM-first wiring without relying on filenames.
    auto m53Header = baseHeader(0x88, 1); // 0x220000 PRG
    m53Header[6] = 0x50; m53Header[7] = 0x30;
    std::vector<uint8_t> m53Prg(0x220000);
    for (std::size_t i=0;i<m53Prg.size();++i) m53Prg[i]=static_cast<uint8_t>(i/0x4000);
    std::fill(m53Prg.begin(),m53Prg.begin()+0x8000,0);
    m53Prg[0x7FFC]=0xF1; m53Prg[0x7FFD]=0xEC; m53Prg[0x7FFE]=0x1A; m53Prg[0x7FFF]=0x21;
    const bool m53Crc = RomDatabase::crc32(m53Prg.data(),0x8000)==0x63794E25u;
    const auto m53Path=root/"supervision53.nes";
    bool m53Written=false; { std::ofstream f(m53Path,std::ios::binary|std::ios::trunc); if(f){ f.write(reinterpret_cast<const char*>(m53Header.data()),16); f.write(reinterpret_cast<const char*>(m53Prg.data()),static_cast<std::streamsize>(m53Prg.size())); std::vector<uint8_t> c(0x2000); f.write(reinterpret_cast<const char*>(c.data()),c.size()); m53Written=bool(f); } }
    Cartridge m53Cart; const bool m53Loaded=m53Written&&m53Cart.loadFromFile(m53Path.string());
    const bool m53Database=m53Crc&&m53Loaded&&m53Cart.mapper()==53&&m53Cart.mapperSupported()&&m53Cart.cpuRead(0x8000)==0x00;
    std::printf("rom_database_mapper53=%s crc=%08X boot=%02X\n",m53Database?"PASS":"FAIL",RomDatabase::crc32(m53Prg.data(),0x8000),m53Loaded?m53Cart.cpuRead(0x8000):0);
    ok &= m53Database;

    // Phase 31 database metadata corrections. These synthetic payloads have
    // the verified PRG+CHR CRCs of the real cartridges, while deliberately
    // carrying incomplete/wrong legacy iNES metadata.
    auto writeCrcPayload = [&](const std::filesystem::path& path, std::array<uint8_t,16> h,
                               std::size_t prg, std::size_t chr,
                               std::array<uint8_t,4> tail) {
        std::ofstream f(path,std::ios::binary|std::ios::trunc); if(!f) return false;
        f.write(reinterpret_cast<const char*>(h.data()),16);
        std::vector<uint8_t> payload(prg+chr,0);
        if(payload.size()<4) return false;
        std::copy(tail.begin(),tail.end(),payload.end()-4);
        f.write(reinterpret_cast<const char*>(payload.data()),static_cast<std::streamsize>(payload.size()));
        return bool(f);
    };

    auto hdHeader=baseHeader(8,16); hdHeader[6]=0xE0; hdHeader[7]=0x40;
    const auto hdPath=root/"db_holy_diver.nes";
    Cartridge hdCart;
    const bool hdLoaded=writeCrcPayload(hdPath,hdHeader,0x20000,0x20000,{0x11,0xD4,0x36,0xF8})&&hdCart.loadFromFile(hdPath.string());
    const bool hdDb=hdLoaded&&hdCart.mapper()==78&&hdCart.submapper()==3&&hdCart.prgRamSize()==0&&hdCart.timing()==ConsoleTiming::NTSC;

    auto ccHeader=baseHeader(8,16); ccHeader[6]=0xE0; ccHeader[7]=0x40;
    const auto ccPath=root/"db_cosmo_carrier.nes";
    Cartridge ccCart;
    const bool ccLoaded=writeCrcPayload(ccPath,ccHeader,0x20000,0x20000,{0xB9,0x45,0x28,0x89})&&ccCart.loadFromFile(ccPath.string());
    const bool ccDb=ccLoaded&&ccCart.mapper()==78&&ccCart.submapper()==1&&ccCart.prgRamSize()==0;

    auto fcHeader=baseHeader(32,16); fcHeader[6]=0x30; fcHeader[7]=0x10;
    const auto fcPath=root/"db_family_circuit91.nes";
    Cartridge fcCart;
    const bool fcLoaded=writeCrcPayload(fcPath,fcHeader,0x80000,0x20000,{0xC6,0x60,0x6A,0x16})&&fcCart.loadFromFile(fcPath.string());
    const bool fcDb=fcLoaded&&fcCart.mapper()==210&&fcCart.submapper()==1&&
        fcCart.mirroring()==Mirror::Vertical&&fcCart.prgRamSize()==0x0800&&
        fcCart.prgNvRamSize()==0x0800&&fcCart.hasBattery();
    std::printf("rom_database_metadata hd=%s cc=%s fc91=%s mapper=%u.%u ram=%zu\n",
        hdDb?"PASS":"FAIL",ccDb?"PASS":"FAIL",fcDb?"PASS":"FAIL",
        unsigned(fcCart.mapper()),unsigned(fcCart.submapper()),fcCart.prgRamSize());
    ok &= hdDb&&ccDb&&fcDb;

    // Trainers initialize CPU $7000-$71FF before execution.
    auto trainerHeader = baseHeader(1, 0);
    trainerHeader[6] = 0x04;
    std::vector<uint8_t> trainer(512);
    for (std::size_t i = 0; i < trainer.size(); ++i) trainer[i] = static_cast<uint8_t>(i ^ 0xA5);
    const auto trainerPath = root / "trainer.nes";
    const bool trainerWritten = writeRom(trainerPath, trainerHeader, 0x4000, 0, &trainer);
    Cartridge trainerCart;
    const bool trainerLoaded = trainerWritten && trainerCart.loadFromFile(trainerPath.string());
    const bool trainerMapped = trainerLoaded && trainerCart.cpuRead(0x7000) == 0xA5 &&
        trainerCart.cpuRead(0x71FF) == static_cast<uint8_t>(0x1FF ^ 0xA5);
    std::printf("trainer_7000_71ff=%s first=%02X last=%02X\n",
        trainerMapped ? "PASS" : "FAIL",
        trainerLoaded ? trainerCart.cpuRead(0x7000) : 0,
        trainerLoaded ? trainerCart.cpuRead(0x71FF) : 0);
    ok &= trainerMapped;

    // Front Fareast RAM-cartridge extracts use the iNES trainer as a boot
    // helper, but mapper 6/8/12.1 and mapper 17 have different conventions.
    auto mc6Header = baseHeader(8, 0); // 128 KiB game image
    mc6Header[6] = 0x64; // mapper 6 + trainer
    std::vector<uint8_t> mcTrainer(512, 0xEA); mcTrainer[3] = 0x60; // RTS at $7003
    const auto mc6Path = root / "magic_card6.nes";
    Cartridge mc6Cart;
    const bool mc6Loaded = writeRom(mc6Path, mc6Header, 0x20000, 0, &mcTrainer) && mc6Cart.loadFromFile(mc6Path.string());
    uint16_t mc6Entry=0; bool mc6Jsr=false;
    const bool mc6Boot = mc6Loaded && mc6Cart.hardResetBootstrap(0x8123,mc6Entry,mc6Jsr) &&
        mc6Entry==0x7003 && mc6Jsr && mc6Cart.cpuRead(0x7003)==0x60 &&
        mc6Cart.prgRamSize()>=0x29000 && mc6Cart.chrRamSize()>=0x8000 && !mc6Cart.hasBattery();

    auto smc17Header = baseHeader(0x20, 0); // 512 KiB PRG snapshot
    smc17Header[6] = 0x14; // mapper 17 low nibble + trainer
    smc17Header[7] = 0x18; // mapper high nibble 1 + NES2
    smc17Header[8] = 0x10; // submapper 1 -> trainer at $5D00
    const auto smc17Path = root / "super_magic17.nes";
    Cartridge smc17Cart;
    const bool smc17Loaded = writeRom(smc17Path,smc17Header,0x80000,0,&mcTrainer) && smc17Cart.loadFromFile(smc17Path.string());
    uint16_t smc17Entry=0; bool smc17Jsr=true;
    const bool smc17Boot = smc17Loaded && smc17Cart.hardResetBootstrap(0x8123,smc17Entry,smc17Jsr) &&
        smc17Entry==0x5D00 && !smc17Jsr && smc17Cart.cpuRead(0x5D00)==0xEA &&
        smc17Cart.prgRamSize()>=0x89000 && smc17Cart.chrRamSize()>=0x40000 && !smc17Cart.hasBattery();
    std::printf("magic_card_trainer_boot m6=%s m17=%s entry=%04X/%04X\n",
        mc6Boot?"PASS":"FAIL",smc17Boot?"PASS":"FAIL",unsigned(mc6Entry),unsigned(smc17Entry));
    ok &= mc6Boot && smc17Boot;

    // NES 2.0 cartridge metadata is part of save-state identity. Identical
    // PRG/CHR bytes with a different hard-wired mirroring configuration must
    // not accept each other's states.
    auto nes2a = baseHeader(1, 1);
    nes2a[7] = 0x08;
    auto nes2b = nes2a;
    nes2b[6] |= 0x01;
    const auto idAPath = root / "identity_a.nes";
    const auto idBPath = root / "identity_b.nes";
    Cartridge idA, idB;
    const bool idsLoaded = writeRom(idAPath, nes2a, 0x4000, 0x2000) &&
        writeRom(idBPath, nes2b, 0x4000, 0x2000) &&
        idA.loadFromFile(idAPath.string()) && idB.loadFromFile(idBPath.string());
    const bool identitySeparatesMetadata = idsLoaded && idA.romIdentity() != idB.romIdentity();
    std::printf("state_identity_metadata=%s\n", identitySeparatesMetadata ? "PASS" : "FAIL");
    ok &= identitySeparatesMetadata;

    // NES 2.0 PRG+CHR NVRAM round-trip through the versioned battery format.
    auto batteryHeader = baseHeader(1, 0);
    batteryHeader[6] = 0x02;
    batteryHeader[7] = 0x08;
    batteryHeader[10] = 0x70; // 8 KiB PRG NVRAM
    batteryHeader[11] = 0x70; // 8 KiB CHR NVRAM
    const auto batteryPath = root / "battery.nes";
    const auto savePath = root / "battery.sav";
    std::error_code ec;
    std::filesystem::remove(savePath, ec);
    Cartridge batteryA;
    const bool batteryLoadedA = writeRom(batteryPath, batteryHeader, 0x4000, 0) &&
        batteryA.loadFromFile(batteryPath.string());
    if (batteryLoadedA) {
        batteryA.cpuWrite(0x6000, 0xA5);
        batteryA.ppuWrite(0x0123, 0x5A);
        batteryA.saveBattery();
    }
    Cartridge batteryB;
    const bool batteryLoadedB = batteryB.loadFromFile(batteryPath.string());
    const bool batteryRoundTrip = batteryLoadedA && batteryLoadedB &&
        batteryB.cpuRead(0x6000) == 0xA5 && batteryB.ppuRead(0x0123) == 0x5A;
    std::printf("battery_prg_chr_roundtrip=%s prg=%02X chr=%02X\n",
        batteryRoundTrip ? "PASS" : "FAIL",
        batteryLoadedB ? batteryB.cpuRead(0x6000) : 0,
        batteryLoadedB ? batteryB.ppuRead(0x0123) : 0);
    ok &= batteryRoundTrip;


    // NES 2.0 submappers 1/2 for UxROM explicitly select no-conflict vs
    // AND-type bus-conflict hardware. The same CPU write must therefore pick
    // bank 3 on submapper 1 but be masked to bank 0 by ROM byte $00 on submapper 2.
    auto uxNoConflictHeader = baseHeader(4, 0);
    uxNoConflictHeader[6] = 0x20; uxNoConflictHeader[7] = 0x08; uxNoConflictHeader[8] = 0x10;
    auto uxConflictHeader = uxNoConflictHeader; uxConflictHeader[8] = 0x20;
    const auto uxNoConflictPath = root / "uxrom_noconflict.nes";
    const auto uxConflictPath = root / "uxrom_conflict.nes";
    Cartridge uxNoConflict, uxConflict;
    const bool uxLoaded = writeRom(uxNoConflictPath, uxNoConflictHeader, 0x10000, 0, nullptr, true) &&
        writeRom(uxConflictPath, uxConflictHeader, 0x10000, 0, nullptr, true) &&
        uxNoConflict.loadFromFile(uxNoConflictPath.string()) && uxConflict.loadFromFile(uxConflictPath.string());
    if (uxLoaded) {
        uxNoConflict.cpuWrite(0x8000, 3);
        uxConflict.cpuWrite(0x8000, 3);
    }
    const bool busConflictSubmaps = uxLoaded && uxNoConflict.cpuRead(0x8000) == 3 && uxConflict.cpuRead(0x8000) == 0;
    std::printf("uxrom_bus_conflict_submaps=%s no=%02X and=%02X\n",
        busConflictSubmaps ? "PASS" : "FAIL",
        uxLoaded ? uxNoConflict.cpuRead(0x8000) : 0, uxLoaded ? uxConflict.cpuRead(0x8000) : 0);
    ok &= busConflictSubmaps;

    // Reset lifecycle reaches cartridge hardware through Bus. Mapper 116.3 is
    // a five-game reset-select board: power-on selects game 0, soft Reset
    // advances the outer ROM window, and powerOn returns to game 0.
    auto reset116Header = baseHeader(0x30, 0x60); // 768 KiB PRG + 768 KiB CHR
    reset116Header[6] = 0x40;
    reset116Header[7] = 0x78; // mapper 116 + NES 2.0 marker
    reset116Header[8] = 0x30; // submapper 3
    const auto reset116Path = root / "mapper116_reset.nes";
    Cartridge reset116Cart;
    const bool reset116Loaded = writeRom(reset116Path, reset116Header, 0xC0000, 0xC0000, nullptr, true) &&
        reset116Cart.loadFromFile(reset116Path.string());
    Bus reset116Bus;
    reset116Bus.connectCartridge(&reset116Cart);
    uint8_t reset116Banks[6] = {};
    if (reset116Loaded) {
        reset116Bus.powerOn();
        reset116Banks[0] = reset116Cart.cpuRead(0xE000);
        for (int i = 1; i < 6; ++i) {
            reset116Bus.reset();
            reset116Banks[i] = reset116Cart.cpuRead(0xE000);
        }
        reset116Bus.powerOn();
    }
    const uint8_t reset116Hard = reset116Loaded ? reset116Cart.cpuRead(0xE000) : 0;
    const bool mapperResetLifecycle = reset116Loaded && reset116Cart.mapperSupported() &&
        reset116Banks[0] == 0x0F && reset116Banks[1] == 0x17 && reset116Banks[2] == 0x1F &&
        reset116Banks[3] == 0x27 && reset116Banks[4] == 0x2F && reset116Banks[5] == 0x0F &&
        reset116Hard == 0x0F;
    std::printf("mapper_reset_lifecycle=%s seq=%02X/%02X/%02X/%02X/%02X wrap=%02X hard=%02X\n",
        mapperResetLifecycle ? "PASS" : "FAIL", reset116Banks[0], reset116Banks[1], reset116Banks[2],
        reset116Banks[3], reset116Banks[4], reset116Banks[5], reset116Hard);
    ok &= mapperResetLifecycle;

    // Cartridge loadState is transactional even when a mapper payload parses
    // partially and then fails its exact-size check.
    auto uxromHeader = baseHeader(4, 0);
    uxromHeader[6] = 0x20;
    // This probe validates transactional state loading, not bus conflicts.
    // Declare NES 2.0 Mapper 2 submapper 1 so bank writes are explicitly
    // conflict-free and independent of the byte currently driven by PRG ROM.
    uxromHeader[7] = 0x08;
    uxromHeader[8] = 0x10;
    const auto uxromPath = root / "state_txn.nes";
    Cartridge stateCart;
    const bool stateLoaded = writeRom(uxromPath, uxromHeader, 0x10000, 0, nullptr, true) &&
        stateCart.loadFromFile(uxromPath.string());
    bool stateTransactional = false;
    if (stateLoaded) {
        stateCart.cpuWrite(0x8000, 2);
        std::vector<uint8_t> malformed;
        stateCart.saveState(malformed);
        // Cartridge state: mapper id[2], submapper[1], mapper-size[4], mapper data...
        if (malformed.size() > 8 && malformed[3] == 1 && malformed[4] == 0 &&
            malformed[5] == 0 && malformed[6] == 0) {
            malformed[3] = 2;
            malformed.insert(malformed.begin() + 8, 0xEE);
            stateCart.cpuWrite(0x8000, 3);
            const uint8_t before = stateCart.cpuRead(0x8000);
            const uint8_t* p = malformed.data();
            const bool rejected = !stateCart.loadState(p, malformed.data() + malformed.size());
            const uint8_t after = stateCart.cpuRead(0x8000);
            stateTransactional = rejected && before == 3 && after == before;
        }
    }
    std::printf("cartridge_state_transaction=%s bank=%02X\n",
        stateTransactional ? "PASS" : "FAIL", stateLoaded ? stateCart.cpuRead(0x8000) : 0);
    ok &= stateTransactional;

    // Archive loading feeds cartridge images from memory. The parsing and ROM
    // identity must be identical to loading the same iNES bytes from disk,
    // even when the archive member uses a nonstandard extension.
    auto memoryHeader = baseHeader(2, 1);
    std::vector<uint8_t> memoryImage(memoryHeader.begin(), memoryHeader.end());
    memoryImage.resize(16 + 0x8000 + 0x2000);
    for (std::size_t i = 16; i < memoryImage.size(); ++i)
        memoryImage[i] = static_cast<uint8_t>((i * 13u + 7u) & 0xFFu);
    const auto memoryPath = root / "memory_source.nes";
    {
        std::ofstream f(memoryPath, std::ios::binary);
        f.write(reinterpret_cast<const char*>(memoryImage.data()), static_cast<std::streamsize>(memoryImage.size()));
    }
    Cartridge memoryDiskCart;
    Cartridge memoryBufferCart;
    const bool memoryLoader = memoryDiskCart.loadFromFile(memoryPath.string()) &&
        memoryBufferCart.loadFromMemory(memoryImage, "renamed_game.rom", (root / "collection.zip").string()) &&
        memoryDiskCart.romIdentity() == memoryBufferCart.romIdentity() &&
        memoryBufferCart.fileName() == "renamed_game.rom" && memoryBufferCart.mapper() == memoryDiskCart.mapper();
    std::printf("cartridge_memory_loader=%s mapper=%u identity=%s\n",
        memoryLoader ? "PASS" : "FAIL", memoryBufferCart.mapper(),
        (memoryDiskCart.romIdentity() == memoryBufferCart.romIdentity()) ? "MATCH" : "MISMATCH");
    ok &= memoryLoader;

    std::filesystem::remove_all(root, ec);
    std::puts(ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

#ifndef NES_PROBE_SUITE
int main() { return runCartridgeConformanceProbe(); }
#endif
