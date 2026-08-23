#include "ProbeSuite.hpp"
#include "CheatSystem.hpp"
#include "Cartridge.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>

int runCheatConformanceProbe()
{
    int failures = 0;
    auto check = [&](bool ok, const char* label) {
        if (!ok) { std::printf("  FAIL: %s\n", label); ++failures; }
    };

    GameGenieCode gossip;
    check(CheatSystem::decodeGameGenie("GOSSIP", gossip), "decode six-letter Game Genie code");
    check(gossip.address == 0xD1DD && gossip.value == 0x14 && !gossip.hasCompare,
        "GOSSIP decodes to $D1DD -> $14");

    GameGenieCode invalid;
    check(!CheatSystem::decodeGameGenie("BAD123", invalid), "reject invalid Game Genie alphabet");
    check(!CheatSystem::decodeGameGenie("APZL", invalid), "reject invalid Game Genie length");

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "nesultimate_cheat_probe.cht";
    {
        std::ofstream out(path);
        out << "cheats = 3\n"
               "cheat0_desc = \"Known decode\"\n"
               "cheat0_code = \"GOSSIP\"\n"
               "cheat0_enable = false\n"
               "cheat1_desc = \"Multi-code entry\"\n"
               "cheat1_code = \"AEXTLIZA+IEXTLIZA\"\n"
               "cheat1_enable = false\n"
               "cheat2_desc = \"Raw RAM value\"\n"
               "cheat2_code = \"0025:63\"\n"
               "cheat2_enable = false\n";
    }

    CheatSystem cheats;
    check(cheats.loadFromFile(path.string()), "load RetroArch .cht syntax");
    check(cheats.entries().size() == 3, "parse Game Genie and raw cheat entries");
    check(cheats.entries().size() < 2 || cheats.entries()[1].codes.size() == 2,
        "parse plus-separated multi-code cheat");

    if (!cheats.entries().empty()) {
        check(cheats.applyGameGenie(0xD1DD, 0x99) == 0x99, "disabled cheat passes cartridge byte through");
        check(cheats.setEntryEnabled(0, true), "enable Game Genie entry");
        check(cheats.applyGameGenie(0xD1DD, 0x99) == 0x14, "enabled cheat replaces cartridge byte");
        check(cheats.applyGameGenie(0xD1DE, 0x99) == 0x99, "nonmatching address passes through");
        cheats.disableAll();
        check(cheats.applyGameGenie(0xD1DD, 0x99) == 0x99, "disable-all rebuilds active cheat cache");
        check(cheats.setEntryEnabled(0, true), "re-enable Game Genie entry");
    }


    if (cheats.entries().size() >= 3) {
        check(cheats.setEntryEnabled(2, true), "enable raw CPU entry");
        check(cheats.applyRawCpuRead(0x0025, 0x00) == 0x63, "raw address:value cheat overrides CPU read");
        check(cheats.applyRawCpuRead(0x0026, 0x44) == 0x44, "raw cheat leaves other addresses untouched");
    }

    // Verify the cartridge path itself applies Game Genie substitution after
    // mapper translation, which is required for bank-switched PRG reads.
    std::vector<uint8_t> rom(16 + 0x4000 + 0x2000, 0);
    rom[0] = 'N'; rom[1] = 'E'; rom[2] = 'S'; rom[3] = 0x1A;
    rom[4] = 1; rom[5] = 1;
    rom[16 + 0x11DD] = 0x99; // $D1DD in mirrored 16 KiB NROM PRG
    Cartridge cart;
    check(cart.loadFromMemory(rom, "CheatProbe.nes"), "load synthetic NROM for cartridge cheat path");
    check(cart.cheats().loadFromFile(path.string()), "attach cheat database to cartridge");
    if (!cart.cheats().entries().empty()) check(cart.cheats().setEntryEnabled(0, true), "enable cartridge Game Genie entry");
    check(cart.cpuRead(0xD1DD) == 0x14, "cartridge read path applies Game Genie after mapper mapping");

    std::error_code ec;
    std::filesystem::remove(path, ec);
    return failures == 0 ? 0 : 1;
}
