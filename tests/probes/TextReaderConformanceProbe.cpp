#include "ProbeSuite.hpp"
#include "TextReader.hpp"
#include <array>
#include <iostream>
#include <string>

int runTextReaderConformanceProbe()
{
    using namespace NesTextReader;
    if (decodeFinalFantasyTile(0x8A) != 'A' || decodeFinalFantasyTile(0xA4) != 'a' ||
        decodeFinalFantasyTile(0x80) != '0' || decodeFinalFantasyTile(0xC5) != '?') {
        std::cerr << "Final Fantasy tile decoder mapping failed\n";
        return 1;
    }

    std::array<uint8_t, 960> nt{};
    auto put = [&](int row, int col, const std::string& text) {
        for (std::size_t i = 0; i < text.size(); ++i) {
            const char ch = text[i];
            uint8_t tile = 0;
            if (ch >= 'A' && ch <= 'Z') tile = static_cast<uint8_t>(0x8A + ch - 'A');
            else if (ch >= 'a' && ch <= 'z') tile = static_cast<uint8_t>(0xA4 + ch - 'a');
            else if (ch == '!') tile = 0xC4;
            nt[static_cast<std::size_t>(row * 32 + col) + i] = tile;
        }
    };
    put(18, 3, "Welcome");
    put(19, 3, "Warrior!");
    const std::string got = extractFinalFantasyNametableText(nt);
    if (got != "Welcome Warrior!") {
        std::cerr << "Final Fantasy nametable extraction failed: '" << got << "'\n";
        return 2;
    }

    if (decodeTile(TileTextProfile::DragonWarrior, 0x24) != 'A' ||
        decodeTile(TileTextProfile::DragonWarrior, 0x0A) != 'a' ||
        decodeTile(TileTextProfile::DragonWarrior2, 0x69) != ',' ||
        decodeTile(TileTextProfile::DragonWarrior2, 0x6B) != '.' ||
        decodeTile(TileTextProfile::DragonWarrior3, 0x25) != 'A' ||
        decodeTile(TileTextProfile::DragonWarrior3, 0x69) != '.' ||
        decodeTile(TileTextProfile::DragonWarrior3, 0x6A) != ',' ||
        decodeTile(TileTextProfile::DragonWarrior3, 0x6F) != '?' ||
        decodeTile(TileTextProfile::DragonWarrior4, 0x25) != 'A' ||
        decodeTile(TileTextProfile::Zelda, 0x0A) != 'A' ||
        decodeTile(TileTextProfile::Zelda, 0x2E) != '?') {
        std::cerr << "RPG tile decoder mapping failed\n";
        return 3;
    }

    std::array<uint8_t, 960> dw{};
    dw.fill(0x5F);
    auto putDw = [&](int row, int col, const std::string& text) {
        for (std::size_t i = 0; i < text.size(); ++i) {
            const char ch = text[i];
            uint8_t tile = 0x5F;
            if (ch >= 'A' && ch <= 'Z') tile = static_cast<uint8_t>(0x24 + ch - 'A');
            else if (ch >= 'a' && ch <= 'z') tile = static_cast<uint8_t>(0x0A + ch - 'a');
            else if (ch == '!') tile = 0x4C;
            dw[static_cast<std::size_t>(row * 32 + col) + i] = tile;
        }
    };
    putDw(20, 4, "Welcome");
    putDw(21, 4, "Warrior!");
    const std::string dwGot = extractNametableText(TileTextProfile::DragonWarrior, dw);
    if (dwGot != "Welcome Warrior!") {
        std::cerr << "Dragon Warrior nametable extraction failed: '" << dwGot << "'\n";
        return 4;
    }

    std::array<uint8_t, 960> region{};
    region.fill(0x5F);
    auto putRegion = [&](int row, int col, const std::string& text) {
        for (std::size_t i = 0; i < text.size(); ++i) {
            const char ch = text[i];
            uint8_t tile = 0x5F;
            if (ch >= 'A' && ch <= 'Z') tile = static_cast<uint8_t>(0x24 + ch - 'A');
            else if (ch >= 'a' && ch <= 'z') tile = static_cast<uint8_t>(0x0A + ch - 'a');
            else if (ch == '!') tile = 0x4C;
            region[static_cast<std::size_t>(row * 32 + col) + i] = tile;
        }
    };
    putRegion(4, 2, "STATUS MENU");
    putRegion(20, 4, "Hello hero!");
    const std::string regionGot = extractRegionText(TileTextProfile::DragonWarrior, region, {1, 16, 30, 13}, 3);
    if (regionGot != "Hello hero!") {
        std::cerr << "Dialogue region extraction leaked non-dialogue text: '" << regionGot << "'\n";
        return 5;
    }

    std::array<uint8_t, 960> zelda{};
    zelda.fill(0x24);
    auto putZelda = [&](int row, int col, const std::string& text) {
        for (std::size_t i = 0; i < text.size(); ++i) {
            const char ch = text[i];
            uint8_t tile = 0x24;
            if (ch >= 'A' && ch <= 'Z') tile = static_cast<uint8_t>(0x0A + ch - 'A');
            else if (ch == '!') tile = 0x29;
            zelda[static_cast<std::size_t>(row * 32 + col) + i] = tile;
        }
    };
    putZelda(1, 2, "LIFE ITEMS");
    putZelda(7, 5, "TAKE THIS!");
    const std::string zeldaGot = extractRegionText(TileTextProfile::Zelda, zelda, {2, 4, 28, 10}, 4);
    if (zeldaGot != "TAKE THIS!") {
        std::cerr << "Zelda dialogue region extraction failed: '" << zeldaGot << "'\n";
        return 6;
    }

    if (hasDialogueLanguage("0000000000") || !hasDialogueLanguage("Welcome 000", 2) ||
        looksLikeSpokenDialogue("aa 0000000") || !looksLikeSpokenDialogue("Thou hast 120 gold.")) {
        std::cerr << "Dialogue language filter failed\n";
        return 7;
    }

    if (novelDialogueText("Brave warrior", "Brave warrior Welcome friend") != "Welcome friend" ||
        novelDialogueText("First line Second line", "Second line Third line") != "Third line" ||
        novelDialogueText("First line\nSecond line", "Second line\nThird line") != "Third line" ||
        !novelDialogueText("Same line", "Same line").empty() ||
        novelDialogueText("Old message", "A new message") != "A new message") {
        std::cerr << "Scrolling dialogue de-duplication failed\n";
        return 8;
    }

    return 0;
}
