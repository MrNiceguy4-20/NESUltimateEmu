#include "CheatSystem.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace {
std::string trimCopy(std::string value)
{
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string unquote(std::string value)
{
    value = trimCopy(std::move(value));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
        value = value.substr(1, value.size() - 2);
    return value;
}

bool parseIndexedKey(const std::string& key, const char* suffix, std::size_t& index)
{
    if (key.rfind("cheat", 0) != 0) return false;
    const std::size_t underscore = key.find('_', 5);
    if (underscore == std::string::npos || key.substr(underscore) != suffix) return false;
    const std::string digits = key.substr(5, underscore - 5);
    if (digits.empty() || !std::all_of(digits.begin(), digits.end(), [](unsigned char c) { return std::isdigit(c); }))
        return false;
    try { index = static_cast<std::size_t>(std::stoul(digits)); }
    catch (...) { return false; }
    return true;
}
}

void CheatSystem::clear()
{
    m_entries.clear();
    m_activeGameGenieCodes.clear();
    m_activeRawCpuCodes.clear();
    m_databasePath.clear();
    m_gameTitle.clear();
    m_lastError.clear();
}

bool CheatSystem::decodeGameGenie(const std::string& input, GameGenieCode& out)
{
    std::string code;
    code.reserve(input.size());
    for (unsigned char c : input) {
        if (c == '-' || std::isspace(c)) continue;
        code.push_back(static_cast<char>(std::toupper(c)));
    }
    if (code.size() != 6 && code.size() != 8) return false;

    static constexpr char alphabet[] = "APZLGITYEOXUKSVN";
    uint8_t n[8] = {};
    for (std::size_t i = 0; i < code.size(); ++i) {
        const char* p = std::find(std::begin(alphabet), std::end(alphabet) - 1, code[i]);
        if (p == std::end(alphabet) - 1) return false;
        n[i] = static_cast<uint8_t>(p - alphabet);
    }

    out = {};
    out.address = static_cast<uint16_t>(0x8000u |
        ((n[3] & 7u) << 12) |
        ((n[5] & 7u) << 8) |
        ((n[4] & 8u) << 8) |
        ((n[2] & 7u) << 4) |
        ((n[1] & 8u) << 4) |
        (n[4] & 7u) |
        (n[3] & 8u));

    if (code.size() == 6) {
        out.value = static_cast<uint8_t>(((n[1] & 7u) << 4) |
            ((n[0] & 8u) << 4) | (n[0] & 7u) | (n[5] & 8u));
    } else {
        out.value = static_cast<uint8_t>(((n[1] & 7u) << 4) |
            ((n[0] & 8u) << 4) | (n[0] & 7u) | (n[7] & 8u));
        out.compare = static_cast<uint8_t>(((n[7] & 7u) << 4) |
            ((n[6] & 8u) << 4) | (n[6] & 7u) | (n[5] & 8u));
        out.hasCompare = true;
    }
    out.text = code;
    return true;
}


bool CheatSystem::decodeRawCpuCode(const std::string& input, GameGenieCode& out)
{
    const std::string code = trimCopy(input);
    const std::size_t colon = code.find(':');
    if (colon == std::string::npos || code.find(':', colon + 1) != std::string::npos) return false;
    const std::string a = code.substr(0, colon);
    const std::string v = code.substr(colon + 1);
    if (a.empty() || a.size() > 4 || v.empty() || v.size() > 2) return false;
    auto hexOnly = [](const std::string& value) {
        return std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isxdigit(c); });
    };
    if (!hexOnly(a) || !hexOnly(v)) return false;
    try {
        const unsigned address = std::stoul(a, nullptr, 16);
        const unsigned value = std::stoul(v, nullptr, 16);
        if (address > 0xFFFFu || value > 0xFFu) return false;
        out = {};
        out.address = static_cast<uint16_t>(address);
        out.value = static_cast<uint8_t>(value);
        out.rawCpuAddress = true;
        out.text = code;
        return true;
    } catch (...) {
        return false;
    }
}

std::string CheatSystem::stripCheatSuffix(std::string value)
{
    const std::string suffix = " (Game Genie)";
    if (value.size() >= suffix.size() && value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0)
        value.resize(value.size() - suffix.size());
    return value;
}

std::string CheatSystem::normalizedTitle(std::string value)
{
    value = std::filesystem::path(value).stem().string();
    value = stripCheatSuffix(std::move(value));
    // ROM filenames often omit No-Intro region/revision tags while libretro's
    // files include them. Compare the stable title prefix before the first tag.
    const std::size_t tag = value.find(" (");
    if (tag != std::string::npos) value.resize(tag);
    std::string out;
    for (unsigned char c : value)
        if (std::isalnum(c)) out.push_back(static_cast<char>(std::tolower(c)));
    return out;
}

bool CheatSystem::loadForRom(const std::string& romPath, const std::string& romFileName)
{
    clear();
    if (romFileName.empty()) return false;

    const std::filesystem::path romFs(romPath);
    const std::filesystem::path logicalFile(romFileName);
    const std::string romStem = logicalFile.stem().string();
    const std::string normalizedRom = normalizedTitle(romStem);

    std::vector<std::filesystem::path> dirs;
    try {
        if (!romFs.empty() && romPath.find("::") == std::string::npos)
            dirs.push_back(romFs.parent_path() / "cheats" / "nes");
    } catch (...) {}
    dirs.emplace_back(std::filesystem::path("cheats") / "nes");

    std::filesystem::path best;
    for (const auto& dir : dirs) {
        std::error_code ec;
        if (!std::filesystem::is_directory(dir, ec)) continue;

        const std::filesystem::path exactA = dir / (romStem + ".cht");
        const std::filesystem::path exactB = dir / (romStem + " (Game Genie).cht");
        if (std::filesystem::is_regular_file(exactB, ec)) { best = exactB; break; }
        if (std::filesystem::is_regular_file(exactA, ec)) { best = exactA; break; }

        std::filesystem::path fallback;
        for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
            if (ec) break;
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ".cht") continue;
            if (normalizedTitle(entry.path().filename().string()) == normalizedRom) {
                const std::string name = entry.path().filename().string();
                if (name.find(" (Game Genie).cht") != std::string::npos) { best = entry.path(); break; }
                if (fallback.empty()) fallback = entry.path();
            }
        }
        if (best.empty()) best = fallback;
        if (!best.empty()) break;
    }

    if (best.empty()) {
        m_gameTitle = romStem;
        m_lastError = "No cheat database entry found for this game.";
        return false;
    }
    m_gameTitle = stripCheatSuffix(best.stem().string());
    return loadFromFile(best.string());
}

bool CheatSystem::loadFromFile(const std::string& path)
{
    m_entries.clear();
    m_databasePath.clear();
    m_lastError.clear();

    std::ifstream file(path);
    if (!file) {
        m_lastError = "Could not open cheat database file.";
        return false;
    }

    struct Pending { std::string desc; std::string code; };
    std::unordered_map<std::size_t, Pending> pending;
    std::size_t declaredCount = 0;
    std::string line;
    while (std::getline(file, line)) {
        line = trimCopy(std::move(line));
        if (line.empty() || line[0] == '#') continue;
        const std::size_t equals = line.find('=');
        if (equals == std::string::npos) continue;
        const std::string key = trimCopy(line.substr(0, equals));
        const std::string value = unquote(line.substr(equals + 1));
        if (key == "cheats") {
            try { declaredCount = static_cast<std::size_t>(std::stoul(value)); }
            catch (...) { declaredCount = 0; }
            continue;
        }
        std::size_t index = 0;
        if (parseIndexedKey(key, "_desc", index)) pending[index].desc = value;
        else if (parseIndexedKey(key, "_code", index)) pending[index].code = value;
    }

    std::size_t maxIndex = declaredCount;
    for (const auto& item : pending) maxIndex = std::max(maxIndex, item.first + 1);
    for (std::size_t i = 0; i < maxIndex; ++i) {
        const auto it = pending.find(i);
        if (it == pending.end() || it->second.code.empty()) continue;
        CheatEntry entry;
        entry.description = it->second.desc.empty() ? ("Cheat " + std::to_string(i + 1)) : it->second.desc;

        std::stringstream codes(it->second.code);
        std::string token;
        while (std::getline(codes, token, '+')) {
            GameGenieCode decoded;
            const std::string cleaned = trimCopy(token);
            if (decodeGameGenie(cleaned, decoded) || decodeRawCpuCode(cleaned, decoded))
                entry.codes.push_back(std::move(decoded));
        }
        if (!entry.codes.empty()) m_entries.push_back(std::move(entry));
    }

    m_databasePath = path;
    if (m_entries.empty()) {
        m_lastError = "Cheat file contains no valid NES Game Genie codes.";
        return false;
    }
    return true;
}

void CheatSystem::rebuildActiveCodes()
{
    m_activeGameGenieCodes.clear();
    m_activeRawCpuCodes.clear();
    for (const auto& entry : m_entries) {
        if (!entry.enabled) continue;
        for (const auto& code : entry.codes) {
            if (code.rawCpuAddress) m_activeRawCpuCodes.push_back(code);
            else m_activeGameGenieCodes.push_back(code);
        }
    }
}

bool CheatSystem::setEntryEnabled(std::size_t index, bool enabled)
{
    if (index >= m_entries.size()) return false;
    if (m_entries[index].enabled == enabled) return true;
    m_entries[index].enabled = enabled;
    rebuildActiveCodes();
    return true;
}

void CheatSystem::disableAll()
{
    bool changed = false;
    for (auto& entry : m_entries) {
        changed = changed || entry.enabled;
        entry.enabled = false;
    }
    if (changed) rebuildActiveCodes();
}

uint8_t CheatSystem::applyGameGenie(uint16_t address, uint8_t original) const
{
    // This function is on the hottest cartridge-read path in the emulator.
    // In particular, loading a large per-game cheat database must have zero
    // per-read cost while every cheat is disabled.
    if (address < 0x8000 || m_activeGameGenieCodes.empty()) return original;
    uint8_t value = original;
    for (const auto& code : m_activeGameGenieCodes) {
        if (code.address != address) continue;
        if (code.hasCompare && code.compare != value) continue;
        value = code.value;
    }
    return value;
}

uint8_t CheatSystem::applyRawCpuRead(uint16_t address, uint8_t original) const
{
    // Likewise, ordinary CPU RAM/I/O reads bypass cheat scanning entirely
    // unless at least one raw address:value code is actually enabled.
    if (m_activeRawCpuCodes.empty()) return original;
    uint8_t value = original;
    for (const auto& code : m_activeRawCpuCodes) {
        if (code.address == address) value = code.value;
    }
    return value;
}

std::size_t CheatSystem::enabledCount() const
{
    return static_cast<std::size_t>(std::count_if(m_entries.begin(), m_entries.end(),
        [](const CheatEntry& e) { return e.enabled; }));
}
