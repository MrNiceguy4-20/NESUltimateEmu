#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct GameGenieCode {
    uint16_t address = 0;
    uint8_t value = 0;
    uint8_t compare = 0;
    bool hasCompare = false;
    bool rawCpuAddress = false;
    std::string text;
};

struct CheatEntry {
    std::string description;
    std::vector<GameGenieCode> codes;
    bool enabled = false;
};

class CheatSystem {
public:
    void clear();

    // Load RetroArch/libretro .cht entries for a ROM. Search order is the ROM
    // directory's cheats/nes folder, then ./cheats/nes. Exact basename matches
    // win; normalized title matching is a fallback for region-tagged databases.
    bool loadForRom(const std::string& romPath, const std::string& romFileName);
    bool loadFromFile(const std::string& path);

    // Game Genie is physically in the cartridge bus path and may replace only
    // reads in $8000-$FFFF. The mapper-provided byte is the compare source.
    uint8_t applyGameGenie(uint16_t address, uint8_t original) const;
    uint8_t applyRawCpuRead(uint16_t address, uint8_t original) const;

    const std::vector<CheatEntry>& entries() const { return m_entries; }
    bool setEntryEnabled(std::size_t index, bool enabled);
    void disableAll();
    const std::string& databasePath() const { return m_databasePath; }
    const std::string& gameTitle() const { return m_gameTitle; }
    const std::string& lastError() const { return m_lastError; }
    std::size_t enabledCount() const;

    static bool decodeGameGenie(const std::string& text, GameGenieCode& out);
    static bool decodeRawCpuCode(const std::string& text, GameGenieCode& out);

private:
    std::vector<CheatEntry> m_entries;
    std::string m_databasePath;
    std::string m_gameTitle;
    std::string m_lastError;
    std::vector<GameGenieCode> m_activeGameGenieCodes;
    std::vector<GameGenieCode> m_activeRawCpuCodes;

    void rebuildActiveCodes();

    static std::string normalizedTitle(std::string value);
    static std::string stripCheatSuffix(std::string value);
};
