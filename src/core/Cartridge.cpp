#include "Cartridge.hpp"
#include "RomDatabase.hpp"
#include "AtomicFile.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>

namespace {

constexpr std::size_t kINesHeaderSize = 16;
constexpr std::size_t kTrainerSize = 512;

bool checkedAdd(std::size_t a, std::size_t b, std::size_t& out)
{
    if (a > std::numeric_limits<std::size_t>::max() - b)
        return false;
    out = a + b;
    return true;
}

bool decodeNes20RomSize(uint8_t lsb, uint8_t msbNibble, std::size_t linearUnit, std::size_t& out)
{
    if (msbNibble != 0x0F) {
        const uint64_t units = uint64_t(msbNibble) << 8 | lsb;
        const uint64_t bytes = units * linearUnit;
        if (bytes > std::numeric_limits<std::size_t>::max()) return false;
        out = static_cast<std::size_t>(bytes);
        return true;
    }

    const uint8_t exponent = lsb >> 2;
    const uint8_t multiplier = ((lsb & 3) << 1) | 1;
    if (exponent >= 64) return false;
    const uint64_t base = uint64_t(1) << exponent;
    if (base > std::numeric_limits<uint64_t>::max() / multiplier) return false;
    const uint64_t bytes = base * multiplier;
    if (bytes > std::numeric_limits<std::size_t>::max()) return false;
    out = static_cast<std::size_t>(bytes);
    return true;
}

std::size_t decodeNes20RamSize(uint8_t shift)
{

    return shift == 0 ? 0 : (std::size_t(64) << shift);
}

void put8(std::vector<uint8_t>& out, uint8_t value) { out.push_back(value); }
void put16(std::vector<uint8_t>& out, uint16_t value)
{
    put8(out, static_cast<uint8_t>(value));
    put8(out, static_cast<uint8_t>(value >> 8));
}
void put32(std::vector<uint8_t>& out, uint32_t value)
{
    for (int i = 0; i < 4; ++i) put8(out, static_cast<uint8_t>(value >> (i * 8)));
}

bool get8(const uint8_t*& p, const uint8_t* end, uint8_t& value)
{
    if (p >= end) return false;
    value = *p++;
    return true;
}
bool get16(const uint8_t*& p, const uint8_t* end, uint16_t& value)
{
    if (end - p < 2) return false;
    value = uint16_t(p[0]) | (uint16_t(p[1]) << 8);
    p += 2;
    return true;
}
bool get32(const uint8_t*& p, const uint8_t* end, uint32_t& value)
{
    if (end - p < 4) return false;
    value = uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    p += 4;
    return true;
}

uint32_t readLe32(const uint8_t* p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

struct UnifBoardInfo {
    uint16_t mapper = 0;
    uint8_t submapper = 0;
    std::size_t prgRam = 0;
    std::size_t chrRam = 0;
    bool recognized = false;
};

std::string canonicalUnifBoard(std::string board)
{
    while (!board.empty() && (board.back() == '\0' || std::isspace(static_cast<unsigned char>(board.back())))) board.pop_back();
    std::size_t first = 0;
    while (first < board.size() && std::isspace(static_cast<unsigned char>(board[first]))) ++first;
    board.erase(0, first);
    std::transform(board.begin(), board.end(), board.begin(), [](unsigned char c) { return char(std::toupper(c)); });
    for (const char* prefix : {"NES-", "HVC-", "UNL-", "BTL-", "BMC-"}) {
        const std::size_t n = std::strlen(prefix);
        if (board.size() >= n && board.compare(0, n, prefix) == 0) { board.erase(0, n); break; }
    }
    return board;
}

UnifBoardInfo resolveUnifBoard(const std::string& rawBoard)
{
    const std::string b = canonicalUnifBoard(rawBoard);
    auto exactOrVariant = [&](const char* stem) {
        const std::size_t n = std::strlen(stem);
        return b == stem || (b.size() > n && b.compare(0, n, stem) == 0 && b[n] == '-');
    };
    if (exactOrVariant("NROM") || exactOrVariant("HROM") || exactOrVariant("RROM") ||
        exactOrVariant("RTROM") || exactOrVariant("SROM") || exactOrVariant("STROM")) return {0,0,0,0,true};
    if (exactOrVariant("UNROM") || exactOrVariant("UOROM")) return {2,2,0,0x2000,true};
    if (exactOrVariant("CNROM")) return {3,2,0,0,true};
    if (exactOrVariant("CPROM")) return {13,0,0,0x4000,true};
    if (exactOrVariant("ANROM") || exactOrVariant("AN1ROM")) return {7,1,0,0x2000,true};
    if (exactOrVariant("AMROM")) return {7,2,0,0x2000,true};
    if (exactOrVariant("AOROM")) return {7,0,0,0x2000,true};
    if (exactOrVariant("BNROM")) return {34,2,0,0x2000,true};
    if (exactOrVariant("PEEOROM") || exactOrVariant("PNROM")) return {9,0,0,0,true};
    if (exactOrVariant("FJROM") || exactOrVariant("FKROM")) return {10,0,0x2000,0,true};
    if (exactOrVariant("SEROM") || exactOrVariant("SHROM") || exactOrVariant("SH1ROM")) return {1,5,0,0,true};
    if (exactOrVariant("SAROM") || exactOrVariant("SIROM") || exactOrVariant("SJROM") || exactOrVariant("SKROM")) return {1,0,0x2000,0,true};
    if (exactOrVariant("SGROM") || exactOrVariant("SMROM")) return {1,0,0,0x2000,true};
    if (exactOrVariant("SNROM") || exactOrVariant("SNWEPROM") || exactOrVariant("SUROM")) return {1,0,0x2000,0x2000,true};
    if (exactOrVariant("SOROM")) return {1,0,0x4000,0x2000,true};
    if (exactOrVariant("SXROM")) return {1,0,0x8000,0x2000,true};
    if (exactOrVariant("SBROM") || exactOrVariant("SCROM") || exactOrVariant("SC1ROM") ||
        exactOrVariant("SFROM") || exactOrVariant("SF1ROM") || exactOrVariant("SFEXPROM") ||
        exactOrVariant("SLROM") || exactOrVariant("SL1ROM") || exactOrVariant("SL2ROM") ||
        exactOrVariant("SL3ROM") || exactOrVariant("SLRROM")) return {1,0,0,0,true};
    if (exactOrVariant("TBROM") || exactOrVariant("TFROM") || exactOrVariant("TLROM") ||
        exactOrVariant("TL1ROM") || exactOrVariant("TL2ROM")) return {4,0,0,0,true};
    if (exactOrVariant("TGROM")) return {4,0,0,0x2000,true};
    if (exactOrVariant("TKROM") || exactOrVariant("TK1ROM") || exactOrVariant("TKEPROM") || exactOrVariant("TSROM")) return {4,0,0x2000,0,true};
    if (exactOrVariant("TNROM")) return {4,0,0x2000,0x2000,true};
    if (exactOrVariant("TKSROM")) return {118,0,0x2000,0,true};
    if (exactOrVariant("TLSROM")) return {118,0,0,0,true};
    if (exactOrVariant("TQROM")) return {119,0,0,0x2000,true};
    return {};
}

}

Cartridge::Cartridge() = default;
Cartridge::~Cartridge() = default;

void Cartridge::resetImage()
{
    m_cheats.clear();
    m_loaded = false;
    m_battery = false;
    m_nes20 = false;
    m_fds = false;
    m_timing = ConsoleTiming::NTSC;
    m_multiRegion = false;
    m_hasTrainer = false;
    m_trainerLoadAddress = 0x7000;
    m_prgRom.clear();
    m_chrRom.clear();
    m_chrRam.clear();
    m_prgRam.clear();
    m_identityData.clear();
    m_prgNvRamSize = 0;
    m_chrNvRamSize = 0;
    m_mapperId = 0;
    m_submapper = 0;
    m_headerMirror = Mirror::Horizontal;
    m_mapper.reset();
    m_path.clear();
    m_fileName.clear();
    m_batteryPath.clear();
}

void Cartridge::loadBattery()
{
    const std::size_t mapperSize = m_mapper ? m_mapper->mapperBatterySize() : 0;
    if (!m_battery || m_batteryPath.empty() || (m_prgNvRamSize == 0 && m_chrNvRamSize == 0 && mapperSize == 0))
        return;

    auto readFile = [](const std::string& path, std::vector<uint8_t>& data) -> bool {
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f) return false;
        const auto endPos = f.tellg();
        if (endPos < 0) return false;
        f.seekg(0);
        data.resize(static_cast<std::size_t>(endPos));
        if (!data.empty())
            f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
        return bool(f) || data.empty();
    };

    auto apply = [&](const std::vector<uint8_t>& data) -> bool {
        if (data.size() >= 13 && std::memcmp(data.data(), "NESB", 4) == 0) {
            const uint8_t version = data[4];
            uint32_t prgSize = 0, chrSize = 0, mapSize = 0;
            std::size_t headerSize = 0;
            if (version == 1) {
                prgSize = readLe32(data.data() + 5);
                chrSize = readLe32(data.data() + 9);
                headerSize = 13;
            }
            else if (version == 2 && data.size() >= 17) {
                prgSize = readLe32(data.data() + 5);
                chrSize = readLe32(data.data() + 9);
                mapSize = readLe32(data.data() + 13);
                headerSize = 17;
            }
            else {
                return false;
            }

            const uint64_t payload64 = uint64_t(prgSize) + uint64_t(chrSize) + uint64_t(mapSize);
            if (payload64 > std::numeric_limits<std::size_t>::max() ||
                data.size() != headerSize + static_cast<std::size_t>(payload64))
                return false;

            if (prgSize != m_prgNvRamSize || chrSize != m_chrNvRamSize ||
                (version == 2 && mapSize != mapperSize) ||
                prgSize > m_prgRam.size() || chrSize > m_chrRam.size())
                return false;

            const uint8_t* payload = data.data() + headerSize;
            if (prgSize) std::memcpy(m_prgRam.data(), payload, prgSize);
            payload += prgSize;
            if (chrSize) std::memcpy(m_chrRam.data(), payload, chrSize);
            payload += chrSize;
            if (version == 2 && m_mapper && mapSize)
                m_mapper->loadMapperBattery(payload, mapSize);
            return true;
        }

        if (m_chrNvRamSize != 0) return false;
        const std::size_t expected = m_prgNvRamSize + mapperSize;
        if (data.size() != expected) return false;
        if (m_prgNvRamSize && m_prgNvRamSize <= m_prgRam.size())
            std::memcpy(m_prgRam.data(), data.data(), m_prgNvRamSize);
        if (m_mapper && mapperSize)
            m_mapper->loadMapperBattery(data.data() + m_prgNvRamSize, mapperSize);
        return true;
    };

    std::vector<uint8_t> data;
    if (readFile(m_batteryPath, data) && apply(data))
        return;

    data.clear();
    const std::string backupPath = m_batteryPath + ".bak";
    if (readFile(backupPath, data))
        (void)apply(data);
}

void Cartridge::saveBattery() const
{
    const std::size_t mapperSize = m_mapper ? m_mapper->mapperBatterySize() : 0;
    if (!m_battery || m_batteryPath.empty() || (m_prgNvRamSize == 0 && m_chrNvRamSize == 0 && mapperSize == 0))
        return;

    const std::size_t prgAmount = std::min(m_prgNvRamSize, m_prgRam.size());
    const std::size_t chrAmount = std::min(m_chrNvRamSize, m_chrRam.size());
    std::vector<uint8_t> mapperData;
    if (m_mapper && mapperSize) {
        mapperData.reserve(mapperSize);
        m_mapper->saveMapperBattery(mapperData);
        if (mapperData.size() != mapperSize) mapperData.clear();
    }

    std::vector<uint8_t> data;
    data.reserve(17 + prgAmount + chrAmount + mapperData.size());
    data.insert(data.end(), {'N', 'E', 'S', 'B'});
    data.push_back(2);
    auto appendLe32 = [&](uint32_t value) {
        data.push_back(static_cast<uint8_t>(value));
        data.push_back(static_cast<uint8_t>(value >> 8));
        data.push_back(static_cast<uint8_t>(value >> 16));
        data.push_back(static_cast<uint8_t>(value >> 24));
    };
    appendLe32(static_cast<uint32_t>(prgAmount));
    appendLe32(static_cast<uint32_t>(chrAmount));
    appendLe32(static_cast<uint32_t>(mapperData.size()));
    data.insert(data.end(), m_prgRam.begin(), m_prgRam.begin() + static_cast<std::ptrdiff_t>(prgAmount));
    data.insert(data.end(), m_chrRam.begin(), m_chrRam.begin() + static_cast<std::ptrdiff_t>(chrAmount));
    data.insert(data.end(), mapperData.begin(), mapperData.end());

    nes::writeFileAtomically(m_batteryPath, data.data(), data.size(), true);
}

bool Cartridge::loadFromFile(const std::string& path)
{
    m_lastError.clear();
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) { m_lastError = "Could not open ROM file."; return false; }
    const auto end = file.tellg();
    if (end <= 0) { m_lastError = "ROM file is empty."; return false; }
    file.seekg(0);
    std::vector<uint8_t> raw(static_cast<std::size_t>(end));
    file.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));
    if (!file) { m_lastError = "Could not read the complete ROM file."; return false; }
    return loadFromMemory(raw, path);
}

bool Cartridge::loadFromMemory(const std::vector<uint8_t>& raw, const std::string& logicalPath,
    const std::string& containerPath)
{
    const std::string& path = logicalPath;
    const std::string backingPath = containerPath.empty() ? logicalPath : containerPath;
    saveBattery();
    resetImage();
    m_lastError.clear();

    std::string ext;
    try {
        ext = std::filesystem::path(path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    } catch (...) {}
    const bool headeredFds = raw.size() >= 16 && std::memcmp(raw.data(), "FDS\x1A", 4) == 0;
    if (ext == ".fds" || headeredFds) {
        const bool headered = raw.size() >= 16 && std::memcmp(raw.data(), "FDS\x1A", 4) == 0;
        const std::size_t sides = headered ? raw[4] : raw.size() / 65500;
        const std::size_t offset = headered ? 16 : 0;
        if (sides == 0 || offset + sides * 65500 > raw.size()) return false;

        std::vector<std::filesystem::path> biosCandidates;
        try {
            const auto rp = std::filesystem::path(backingPath);
            biosCandidates.push_back(rp.parent_path() / "disksys.rom");
            biosCandidates.push_back(rp.parent_path() / "DISKSYS.ROM");
        } catch (...) {}
        biosCandidates.emplace_back("disksys.rom");
        biosCandidates.emplace_back("DISKSYS.ROM");

        std::vector<uint8_t> bios;
        for (const auto& candidate : biosCandidates) {
            std::ifstream bf(candidate, std::ios::binary | std::ios::ate);
            if (!bf || bf.tellg() != std::streamoff(0x2000)) continue;
            bf.seekg(0);
            bios.resize(0x2000);
            bf.read(reinterpret_cast<char*>(bios.data()), 0x2000);
            if (bf) break;
            bios.clear();
        }
        if (bios.size() != 0x2000) return false;

        m_fds = true;
        m_mapperId = 20;
        m_submapper = 0;
        m_headerMirror = Mirror::Vertical;
        m_prgRom = std::move(bios);
        m_prgRam.assign(0x8000, 0);
        m_chrRam.assign(0x2000, 0);
        m_identityData = raw;
        const MapperConfig config { m_mapperId, m_submapper, m_prgRom.size(), 0, m_prgRam.size(),
            m_chrRam.size(), m_headerMirror, false, 0, 0, false, true };
        m_mapper = createMapper(config);
        if (!m_mapper || !m_mapper->loadDiskImage(raw)) { resetImage(); return false; }

        m_path = containerPath.empty() ? logicalPath : (containerPath + "::" + logicalPath);
        m_fileName = std::filesystem::path(logicalPath).filename().string();
        try {
            const auto rp = std::filesystem::path(backingPath);
            const std::string stem = containerPath.empty() ? rp.stem().string()
                : (rp.stem().string() + "." + std::filesystem::path(logicalPath).stem().string());
            m_batteryPath = (rp.parent_path() / (stem + ".sav")).string();
        } catch (...) { m_batteryPath = backingPath + ".sav"; }

        m_battery = true;
        loadBattery();
        m_loaded = true;
        return true;
    }

    const bool headeredUnif = raw.size() >= 32 && std::memcmp(raw.data(), "UNIF", 4) == 0;
    if (headeredUnif || ext == ".unf" || ext == ".unif") {
        if (!headeredUnif) { m_lastError = "Invalid UNIF header."; return false; }
        std::array<std::vector<uint8_t>, 16> prgChunks;
        std::array<std::vector<uint8_t>, 16> chrChunks;
        std::string board;
        int mirrorCode = -1;
        int tvci = -1;
        bool battery = false;
        bool vror = false;
        std::size_t pos = 32;
        auto hexIndex = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            return -1;
        };
        while (pos < raw.size()) {
            if (raw.size() - pos < 8) { m_lastError = "Truncated UNIF chunk header."; return false; }
            const char id0 = char(raw[pos]), id1 = char(raw[pos+1]), id2 = char(raw[pos+2]), id3 = char(raw[pos+3]);
            const uint32_t len32 = readLe32(raw.data() + pos + 4);
            pos += 8;
            if (std::size_t(len32) > raw.size() - pos) { m_lastError = "Truncated UNIF chunk payload."; return false; }
            const uint8_t* data = raw.data() + pos;
            if (id0=='M' && id1=='A' && id2=='P' && id3=='R') {
                board.assign(reinterpret_cast<const char*>(data), reinterpret_cast<const char*>(data) + len32);
                const auto zero = board.find('\0'); if (zero != std::string::npos) board.resize(zero);
            } else if (id0=='P' && id1=='R' && id2=='G') {
                const int idx = hexIndex(id3); if (idx >= 0) prgChunks[std::size_t(idx)].assign(data, data + len32);
            } else if (id0=='C' && id1=='H' && id2=='R') {
                const int idx = hexIndex(id3); if (idx >= 0) chrChunks[std::size_t(idx)].assign(data, data + len32);
            } else if (id0=='M' && id1=='I' && id2=='R' && id3=='R' && len32 >= 1) mirrorCode = data[0];
            else if (id0=='T' && id1=='V' && id2=='C' && id3=='I' && len32 >= 1) tvci = data[0];
            else if (id0=='B' && id1=='A' && id2=='T' && id3=='R') battery = true;
            else if (id0=='V' && id1=='R' && id2=='O' && id3=='R') vror = true;
            pos += len32;
        }
        if (board.empty() || prgChunks[0].empty()) { m_lastError = "UNIF requires MAPR and PRG0 chunks."; return false; }
        const UnifBoardInfo info = resolveUnifBoard(board);
        if (!info.recognized) { m_lastError = "Unsupported UNIF board " + board + "."; return false; }
        m_mapperId = info.mapper; m_submapper = info.submapper; m_nes20 = false;
        for (const auto& c : prgChunks) m_prgRom.insert(m_prgRom.end(), c.begin(), c.end());
        for (const auto& c : chrChunks) m_chrRom.insert(m_chrRom.end(), c.begin(), c.end());
        if (m_prgRom.empty()) { m_lastError = "UNIF contains no PRG ROM."; return false; }
        if (mirrorCode == 1) m_headerMirror = Mirror::Vertical;
        else if (mirrorCode == 2 || mirrorCode == 3) m_headerMirror = Mirror::OnescreenLo;
        else if (mirrorCode == 4) m_headerMirror = Mirror::FourScreen;
        else m_headerMirror = Mirror::Horizontal;
        if (tvci == 1) m_timing = ConsoleTiming::PAL;
        else if (tvci == 2) { m_timing = ConsoleTiming::NTSC; m_multiRegion = true; }
        else m_timing = ConsoleTiming::NTSC;
        std::size_t prgRam = info.prgRam;
        std::size_t chrRam = info.chrRam;
        if ((m_chrRom.empty() || vror) && chrRam == 0) chrRam = 0x2000;
        if (vror) m_chrRom.clear();
        if (battery && prgRam == 0) prgRam = 0x2000;
        m_prgNvRamSize = battery ? prgRam : 0;
        m_prgRam.assign(prgRam, 0);
        m_chrRam.assign(chrRam, 0);
        m_identityData = raw;
        const MapperConfig config { m_mapperId, m_submapper, m_prgRom.size(), m_chrRom.size(), m_prgRam.size(),
            m_chrRam.size(), m_headerMirror, m_headerMirror == Mirror::FourScreen, m_prgNvRamSize, 0, false, battery };
        m_mapper = createMapper(config);
        if (!m_mapper || !m_mapper->implementationSupported()) {
            m_lastError = "UNIF board " + board + " maps to unsupported mapper " + std::to_string(m_mapperId) + ".";
            resetImage(); return false;
        }
        m_mapper->initializePrgImage(m_prgRom.data(), m_prgRom.size());
        m_path = containerPath.empty() ? logicalPath : (containerPath + "::" + logicalPath);
        m_fileName = std::filesystem::path(logicalPath).filename().string();
        try {
            const auto rp = std::filesystem::path(backingPath);
            const std::string stem = containerPath.empty() ? rp.stem().string() :
                (rp.stem().string() + "." + std::filesystem::path(logicalPath).stem().string());
            m_batteryPath = (rp.parent_path() / (stem + ".sav")).string();
        } catch (...) { m_batteryPath = backingPath + ".sav"; }
        m_battery = battery && m_prgNvRamSize != 0;
        if (m_battery) loadBattery();
        m_loaded = true;
        return true;
    }

    if (raw.size() < kINesHeaderSize) return false;
    const std::size_t fileSize = raw.size();

    uint8_t header[kINesHeaderSize] = {};
    std::copy_n(raw.begin(), kINesHeaderSize, header);
    if (header[0] != 'N' || header[1] != 'E' || header[2] != 'S' || header[3] != 0x1A)
        return false;

    const uint8_t flags6 = header[6];
    const uint8_t flags7 = header[7];
    const bool hasTrainer = (flags6 & 0x04) != 0;
    m_nes20 = (flags7 & 0x0C) == 0x08;

    const bool dirtyLegacyTail = !m_nes20 &&
        (header[12] != 0 || header[13] != 0 || header[14] != 0 || header[15] != 0);
    const bool archaicINes = !m_nes20 && (((flags7 & 0x0C) == 0x04) || dirtyLegacyTail);

    m_mapperId = uint16_t(flags6 >> 4);
    if (!archaicINes) m_mapperId |= uint16_t(flags7 & 0xF0);
    m_submapper = 0;

    m_hasTrainer = hasTrainer;

    std::size_t prgSize = 0;
    std::size_t chrSize = 0;
    std::size_t prgRamVolatile = 0;
    std::size_t chrRamVolatile = 0;
    std::size_t headerPrgNvRamHint = 0;

    if (m_nes20) {
        m_mapperId |= uint16_t(header[8] & 0x0F) << 8;
        m_submapper = header[8] >> 4;
        if (!decodeNes20RomSize(header[4], header[9] & 0x0F, 0x4000, prgSize) ||
            !decodeNes20RomSize(header[5], header[9] >> 4, 0x2000, chrSize))
            return false;

        prgRamVolatile = decodeNes20RamSize(header[10] & 0x0F);
        m_prgNvRamSize = decodeNes20RamSize(header[10] >> 4);
        chrRamVolatile = decodeNes20RamSize(header[11] & 0x0F);
        m_chrNvRamSize = decodeNes20RamSize(header[11] >> 4);

        switch (header[12] & 0x03) {
        case 1: m_timing = ConsoleTiming::PAL; break;
        case 3: m_timing = ConsoleTiming::Dendy; break;
        case 2: m_multiRegion = true; [[fallthrough]];
        default: m_timing = ConsoleTiming::NTSC; break;
        }
    }
    else {
        prgSize = std::size_t(header[4]) * 0x4000;
        chrSize = std::size_t(header[5]) * 0x2000;

        if (m_mapperId != 30) {
            const uint8_t ramBanks = archaicINes ? 1 : (header[8] ? header[8] : 1);
            const std::size_t iNesRam = std::size_t(ramBanks) * 0x2000;
            if (flags6 & 0x02) m_prgNvRamSize = iNesRam;
            else prgRamVolatile = iNesRam;
        }
    }

    if (prgSize == 0) return false;

    if (m_mapperId == 30 && (flags6 & 0x08)) {

        m_headerMirror = (flags6 & 0x01) ? Mirror::FourScreen : Mirror::OnescreenLo;
    }
    else if (flags6 & 0x08) m_headerMirror = Mirror::FourScreen;
    else if (flags6 & 0x01) m_headerMirror = Mirror::Vertical;
    else m_headerMirror = Mirror::Horizontal;

    std::size_t dataOffset = kINesHeaderSize;
    if (hasTrainer) {
        if (!checkedAdd(dataOffset, kTrainerSize, dataOffset) || dataOffset > fileSize)
            return false;
    }
    std::size_t afterPrg = 0;
    std::size_t afterChr = 0;
    if (!checkedAdd(dataOffset, prgSize, afterPrg) || !checkedAdd(afterPrg, chrSize, afterChr) || afterChr > fileSize)
        return false;

    std::vector<uint8_t> trainer;
    if (hasTrainer) {
        trainer.assign(raw.begin() + kINesHeaderSize, raw.begin() + kINesHeaderSize + kTrainerSize);

        m_identityData = trainer;
    }

    m_prgRom.assign(raw.begin() + dataOffset, raw.begin() + afterPrg);

    if (chrSize)
        m_chrRom.assign(raw.begin() + afterPrg, raw.begin() + afterChr);

    const auto dbResolution = RomDatabase::resolve(m_mapperId,
        m_prgRom.empty() ? nullptr : m_prgRom.data(), m_prgRom.size(),
        m_chrRom.empty() ? nullptr : m_chrRom.data(), m_chrRom.size());

    if (dbResolution.matched) {
        if (dbResolution.overrideMapper) m_mapperId = dbResolution.mapper;
        if (dbResolution.overrideSubmapper) m_submapper = dbResolution.submapper;
        if (dbResolution.overrideMirror) m_headerMirror = dbResolution.mirror;
        if (dbResolution.overrideTiming) m_timing = dbResolution.timing;
        if (dbResolution.overrideMultiRegion) m_multiRegion = dbResolution.multiRegion;
        if (dbResolution.overridePrgRam) prgRamVolatile = dbResolution.prgRamSize;
        if (dbResolution.overridePrgNvRam) m_prgNvRamSize = dbResolution.prgNvRamSize;
        if (dbResolution.overrideChrRam) chrRamVolatile = dbResolution.chrRamSize;
        if (dbResolution.overrideChrNvRam) m_chrNvRamSize = dbResolution.chrNvRamSize;
    }

    headerPrgNvRamHint = m_prgNvRamSize;
    if (m_nes20 && m_mapperId == 4 && m_submapper == 1) {
        m_prgNvRamSize = 0;
        prgRamVolatile = 0;
    }
    const bool serialBandai =
        m_mapperId == 157 || m_mapperId == 159 ||
        (m_mapperId == 16 && (!m_nes20 ||
            (m_submapper == 5 && headerPrgNvRamHint == 256) ||
            (m_submapper == 0 && headerPrgNvRamHint == 256)));
    if (serialBandai) {
        m_prgNvRamSize = 0;
        if (!m_nes20) prgRamVolatile = 0;
    }

    std::size_t prgRamTotal = 0;
    std::size_t chrRamTotal = 0;
    if (!checkedAdd(m_prgNvRamSize, prgRamVolatile, prgRamTotal) ||
        !checkedAdd(m_chrNvRamSize, chrRamVolatile, chrRamTotal)) {
        resetImage();
        return false;
    }

    if (hasTrainer && prgRamTotal < 0x2000) prgRamTotal = 0x2000;

    if (m_mapperId == 198) {
        m_prgNvRamSize = std::max<std::size_t>(m_prgNvRamSize, 0x2000);
        prgRamVolatile = std::max<std::size_t>(prgRamVolatile, 0x1000);
        prgRamTotal = std::max<std::size_t>(prgRamTotal, 0x3000);
    } else if (m_mapperId == 199) {
        m_prgNvRamSize = std::max<std::size_t>(m_prgNvRamSize, 0x8000);
        prgRamTotal = std::max<std::size_t>(prgRamTotal, 0x8000);
    }

    if (m_mapperId == 111 && chrSize == 0) chrRamTotal = std::max<std::size_t>(chrRamTotal, 0x8000);

    if (chrSize == 0 && chrRamTotal == 0) {
        if (m_mapperId == 13) chrRamTotal = 0x4000;
        else if (m_mapperId == 30) chrRamTotal = 0x8000;
        else chrRamTotal = 0x2000;
    }
    if (chrRamTotal == 0 && chrSize != 0) {
        switch (m_mapperId) {
        case 74:  chrRamTotal = 0x0800; break;
        case 77:  chrRamTotal = 0x1800; break;
        case 119: chrRamTotal = 0x2000; break;
        case 191: chrRamTotal = 0x0800; break;
        case 192: chrRamTotal = 0x1000; break;
        case 194: chrRamTotal = 0x0800; break;
        case 195: chrRamTotal = 0x1000; break;
        default: break;
        }
    }

    const bool smcExtract = m_mapperId == 6 || m_mapperId == 8 || m_mapperId == 17 ||
        (m_mapperId == 12 && m_submapper == 1);
    if (smcExtract) {
        const std::size_t prgDram = (m_mapperId == 17 || (m_mapperId == 12 && m_submapper == 1))
            ? 0x80000 : std::max<std::size_t>(0x20000, prgSize);
        prgRamTotal = std::max(prgRamTotal, prgDram + 0x8000 + 0x1000);
        const std::size_t chrDram = m_mapperId == 17 ? 0x40000 : 0x8000;
        chrRamTotal = std::max(chrRamTotal, chrDram);
        m_prgNvRamSize = 0;
        m_chrNvRamSize = 0;
    }

    m_prgRam.assign(prgRamTotal, 0);
    m_chrRam.assign(chrRamTotal, 0);

    if (smcExtract) {
        const std::size_t n = std::min(m_prgRom.size(), m_prgRam.size());
        std::copy_n(m_prgRom.begin(), n, m_prgRam.begin());
        if (m_mapperId == 12 && m_submapper == 1) {

            if (m_prgRam.size() > 0x40000) {
                const std::size_t c = std::min(m_chrRom.size(), m_prgRam.size() - 0x40000);
                std::copy_n(m_chrRom.begin(), c, m_prgRam.begin() + 0x40000);
            }
        } else {
            const std::size_t c = std::min(m_chrRom.size(), m_chrRam.size());
            std::copy_n(m_chrRom.begin(), c, m_chrRam.begin());
        }
    }

    const MapperConfig config {
        m_mapperId,
        m_submapper,
        m_prgRom.size(),
        m_chrRom.size(),
        m_prgRam.size(),
        m_chrRam.size(),
        m_headerMirror,
        m_headerMirror == Mirror::FourScreen,
        headerPrgNvRamHint,
        m_chrNvRamSize,
        m_nes20,
        (flags6 & 0x02) != 0 || headerPrgNvRamHint != 0 || m_chrNvRamSize != 0,
        dbResolution.boardVariant
    };
    m_mapper = createMapper(config);
    if (!m_mapper || !m_mapper->implementationSupported()) {
        const uint16_t unsupportedMapper = m_mapperId;
        const uint8_t unsupportedSubmapper = m_submapper;
        const bool nes20 = m_nes20;
        resetImage();
        m_lastError = "Unsupported mapper " + std::to_string(unsupportedMapper);
        if (nes20)
            m_lastError += " / submapper " + std::to_string(static_cast<unsigned>(unsupportedSubmapper));
        m_lastError += ". This ROM needs a mapper implementation that is not present yet.";
        return false;
    }
    m_mapper->initializePrgImage(m_prgRom.data(), m_prgRom.size());

    if (hasTrainer) {
        if (m_mapperId == 17) {
            static const uint16_t kSmcTrainerAddress[4] = { 0x7000, 0x5D00, 0x5E00, 0x5F00 };
            m_trainerLoadAddress = kSmcTrainerAddress[std::min<unsigned>(m_submapper, 3)];
        } else {
            m_trainerLoadAddress = 0x7000;
        }
        for (std::size_t i = 0; i < trainer.size(); ++i) {
            const uint16_t addr = static_cast<uint16_t>(m_trainerLoadAddress + i);
            uint32_t mapped = 0;

            if (m_mapper->mapPrgRam(addr, mapped, false) && mapped < m_prgRam.size())
                m_prgRam[mapped] = trainer[i];
            else {
                mapped = static_cast<uint32_t>(addr - 0x6000);
                if (mapped < m_prgRam.size())
                    m_prgRam[mapped] = trainer[i];
            }
        }
    }

    m_path = containerPath.empty() ? logicalPath : (containerPath + "::" + logicalPath);
    m_fileName = std::filesystem::path(logicalPath).filename().string();
    try {
        const auto p = std::filesystem::path(backingPath);
        const std::string stem = containerPath.empty() ? p.stem().string()
            : (p.stem().string() + "." + std::filesystem::path(logicalPath).stem().string());
        m_batteryPath = (p.parent_path() / (stem + ".sav")).string();
    }
    catch (...) {
        m_batteryPath = backingPath + ".sav";
    }

    m_battery = (flags6 & 0x02) != 0 || m_prgNvRamSize != 0 || m_chrNvRamSize != 0 ||
        (m_mapper && m_mapper->mapperBatterySize() != 0);
    if (smcExtract) m_battery = false;
    if (m_battery) loadBattery();

    m_loaded = true;
    return true;
}

uint64_t Cartridge::romIdentity() const
{
    uint64_t hash = 1469598103934665603ull;
    auto mix = [&](uint8_t value) {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    auto mix64 = [&](uint64_t value) {
        for (int i = 0; i < 8; ++i) mix(static_cast<uint8_t>(value >> (i * 8)));
    };

    mix(static_cast<uint8_t>(m_mapperId));
    mix(static_cast<uint8_t>(m_mapperId >> 8));
    mix(m_submapper);
    mix(m_nes20 ? 1 : 0);
    mix(m_fds ? 1 : 0);
    mix(static_cast<uint8_t>(m_timing));
    mix(m_multiRegion ? 1 : 0);
    mix(static_cast<uint8_t>(m_headerMirror));
    mix(m_battery ? 1 : 0);
    mix64(m_prgRom.size());
    mix64(m_chrRom.size());
    mix64(m_prgRam.size());
    mix64(m_chrRam.size());
    mix64(m_prgNvRamSize);
    mix64(m_chrNvRamSize);
    for (uint8_t value : m_prgRom) mix(value);
    for (uint8_t value : m_chrRom) mix(value);
    for (uint8_t value : m_identityData) mix(value);
    return hash;
}

bool Cartridge::cpuRead(uint16_t addr, uint8_t& data)
{
    if (!m_loaded || !m_mapper) return false;

    uint8_t regData = data;
    if (addr >= 0x4020 && m_mapper->cpuReadRegister(addr, regData)) {
        data = m_cheats.applyGameGenie(addr, regData);
        return true;
    }

    uint32_t mapped = 0;
    if (addr >= 0x4020 && m_mapper->mapPrgRam(addr, mapped, false) && mapped < m_prgRam.size()) {
        const uint8_t ramData = m_mapper->transformPrgRamRead(addr, m_prgRam[mapped]);
        data = m_cheats.applyGameGenie(addr, ramData);
        m_mapper->observeCpuRead(addr, data);
        return true;
    }

    if (m_mapper->cpuMapRead(addr, mapped) && mapped < m_prgRom.size()) {
        data = m_cheats.applyGameGenie(addr, m_prgRom[mapped]);
        m_mapper->observeCpuRead(addr, data);
        return true;
    }
    return false;
}

bool Cartridge::debugCpuRead(uint16_t addr, uint8_t& data) const
{
    if (!m_loaded || !m_mapper) return false;
    uint32_t mapped = 0;
    if (addr >= 0x4020 && m_mapper->mapPrgRam(addr, mapped, false) && mapped < m_prgRam.size()) {
        data = m_cheats.applyGameGenie(addr, m_mapper->transformPrgRamRead(addr, m_prgRam[mapped]));
        return true;
    }
    if (m_mapper->cpuMapRead(addr, mapped) && mapped < m_prgRom.size()) {
        data = m_cheats.applyGameGenie(addr, m_prgRom[mapped]);
        return true;
    }
    return false;
}

uint8_t Cartridge::cpuRead(uint16_t addr)
{
    uint8_t data = 0;
    (void)cpuRead(addr, data);
    return data;
}

void Cartridge::observeCpuWrite(uint16_t addr, uint8_t data)
{
    if (m_loaded && m_mapper) m_mapper->observeCpuWrite(addr, data);
}

void Cartridge::cpuWrite(uint16_t addr, uint8_t data, uint64_t cpuCycle)
{
    if (!m_loaded || !m_mapper || addr < 0x4020) return;

    uint8_t mapperData = data;
    if (addr >= 0x8000 && m_mapper->hasBusConflicts()) {
        uint32_t mappedRom = 0;
        if (m_mapper->cpuMapRead(addr, mappedRom) && mappedRom < m_prgRom.size())
            mapperData = m_mapper->resolveBusConflict(addr, data, m_prgRom[mappedRom]);
    }

    m_mapper->cpuWrite(addr, mapperData, cpuCycle);

    if (addr >= 0x4020) {
        uint32_t mapped = 0;
        if (m_mapper->mapPrgRam(addr, mapped, true) && mapped < m_prgRam.size())
            m_prgRam[mapped] = data;
    }
}

uint8_t Cartridge::ppuRead(uint16_t addr, PpuFetchKind kind)
{
    if (!m_loaded || !m_mapper) return 0;
    addr &= 0x3FFF;
    if (addr >= 0x2000) return 0;

    uint8_t overrideData = 0;
    if (m_mapper->ppuReadOverride(addr, kind, overrideData)) return overrideData;

    uint32_t mapped = 0;
    if (!m_mapper->ppuMapReadEx(addr, mapped, kind)) return 0;
    if (m_mapper->ppuUsesChrRam(addr)) {
        if (!m_chrRam.empty()) return m_chrRam[mapped % m_chrRam.size()];
        return 0;
    }
    if (!m_chrRom.empty()) return m_chrRom[mapped % m_chrRom.size()];
    if (!m_chrRam.empty()) return m_chrRam[mapped % m_chrRam.size()];
    return 0;
}

bool Cartridge::mapPatternCiram(uint16_t addr, uint32_t& mapped) const
{
    return m_loaded && m_mapper && m_mapper->mapPatternCiram(addr & 0x1FFF, mapped);
}

void Cartridge::ppuWrite(uint16_t addr, uint8_t data)
{
    if (!m_loaded || !m_mapper || m_chrRam.empty()) return;
    addr &= 0x3FFF;
    if (addr >= 0x2000 || !m_mapper->ppuUsesChrRam(addr)) return;

    uint32_t mapped = 0;
    if (m_mapper->ppuMapWrite(addr, mapped) && mapped < m_chrRam.size())
        m_chrRam[mapped] = data;
}

bool Cartridge::mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const
{
    return m_loaded && m_mapper && m_mapper->mapNametable(addr, source, mapped);
}

bool Cartridge::mapNametableWrite(uint16_t addr, NametableSource& source, uint32_t& mapped) const
{
    return m_loaded && m_mapper && m_mapper->mapNametableWrite(addr, source, mapped);
}

uint8_t Cartridge::readNametableBacking(NametableSource source, uint32_t mapped) const
{
    if (source == NametableSource::ChrRom && !m_chrRom.empty())
        return m_chrRom[mapped % m_chrRom.size()];
    if (source == NametableSource::ChrRam && !m_chrRam.empty())
        return m_chrRam[mapped % m_chrRam.size()];
    if (source == NametableSource::MapperRam && m_mapper)
        return m_mapper->readMapperNametable(mapped);
    return 0;
}

void Cartridge::writeNametableBacking(NametableSource source, uint32_t mapped, uint8_t data)
{
    if (source == NametableSource::ChrRam && !m_chrRam.empty())
        m_chrRam[mapped % m_chrRam.size()] = data;
    else if (source == NametableSource::MapperRam && m_mapper)
        m_mapper->writeMapperNametable(mapped, data);
}

void Cartridge::notifyCpuAddress(uint16_t addr)
{
    if (m_mapper) m_mapper->notifyCpuAddress(addr);
}

void Cartridge::notifyPpuAddress(uint16_t addr, uint64_t ppuCycle, int scanline, int dot)
{
    if (m_mapper) m_mapper->notifyPpuAddressContext(addr, ppuCycle, scanline, dot);
}

void Cartridge::notifyPpuScanline(int scanline, bool rendering)
{
    if (m_mapper) m_mapper->notifyPpuScanline(scanline, rendering);
}

void Cartridge::clockCpu()
{
    if (m_mapper) m_mapper->clockCpu();
}

void Cartridge::scanlineTick()
{
    if (m_mapper) m_mapper->scanlineTick();
}

void Cartridge::resetMapper(bool hard)
{
    if (m_mapper) m_mapper->reset(hard);
}

bool Cartridge::hardResetBootstrap(uint16_t, uint16_t& entry, bool& jsr) const
{
    if (!m_hasTrainer) return false;
    if (m_mapperId == 6 || m_mapperId == 8 || (m_mapperId == 12 && m_submapper == 1)) {
        entry = 0x7003;
        jsr = true;
        return true;
    }
    if (m_mapperId == 17) {
        entry = m_trainerLoadAddress;
        jsr = false;
        return true;
    }
    return false;
}

void Cartridge::saveState(std::vector<uint8_t>& out) const
{
    put16(out, m_mapperId);
    put8(out, m_submapper);

    std::vector<uint8_t> mapperState;
    if (m_mapper) m_mapper->saveState(mapperState);
    put32(out, static_cast<uint32_t>(mapperState.size()));
    out.insert(out.end(), mapperState.begin(), mapperState.end());

    put32(out, static_cast<uint32_t>(m_prgRam.size()));
    out.insert(out.end(), m_prgRam.begin(), m_prgRam.end());
    put32(out, static_cast<uint32_t>(m_chrRam.size()));
    out.insert(out.end(), m_chrRam.begin(), m_chrRam.end());
}

bool Cartridge::loadState(const uint8_t*& p, const uint8_t* end)
{

    const uint8_t* q = p;
    uint16_t mapperId = 0;
    uint8_t submapper = 0;
    uint32_t mapperStateSize = 0;
    if (!get16(q, end, mapperId) || mapperId != m_mapperId ||
        !get8(q, end, submapper) || submapper != m_submapper ||
        !get32(q, end, mapperStateSize) || !m_mapper)
        return false;
    if (mapperStateSize > static_cast<uint32_t>(end - q))
        return false;

    const uint8_t* mapperBegin = q;
    const uint8_t* mapperEnd = mapperBegin + mapperStateSize;
    q = mapperEnd;

    uint32_t prgRamSize = 0;
    if (!get32(q, end, prgRamSize) || prgRamSize != m_prgRam.size() ||
        prgRamSize > static_cast<uint32_t>(end - q))
        return false;
    const uint8_t* prgData = q;
    q += prgRamSize;

    uint32_t chrRamSize = 0;
    if (!get32(q, end, chrRamSize) || chrRamSize != m_chrRam.size() ||
        chrRamSize > static_cast<uint32_t>(end - q))
        return false;
    const uint8_t* chrData = q;
    q += chrRamSize;

    std::vector<uint8_t> mapperBackup;
    m_mapper->saveState(mapperBackup);
    const uint8_t* mapperP = mapperBegin;
    if (!m_mapper->loadState(mapperP, mapperEnd) || mapperP != mapperEnd) {
        const uint8_t* restore = mapperBackup.data();
        const uint8_t* restoreEnd = restore + mapperBackup.size();
        (void)m_mapper->loadState(restore, restoreEnd);
        return false;
    }

    if (prgRamSize) std::memcpy(m_prgRam.data(), prgData, prgRamSize);
    if (chrRamSize) std::memcpy(m_chrRam.data(), chrData, chrRamSize);
    p = q;
    return true;
}
