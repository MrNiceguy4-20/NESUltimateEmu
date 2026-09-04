#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "Mapper.hpp"
#include "Timing.hpp"
#include "CheatSystem.hpp"

class CPU;

class Cartridge {
public:
    using Mirror = ::Mirror;

    Cartridge();
    ~Cartridge();

    bool loadFromFile(const std::string& path);
    bool loadFromMemory(const std::vector<uint8_t>& data, const std::string& logicalPath,
        const std::string& containerPath = std::string());
    void connectCPU(CPU* cpu) { m_cpu = cpu; }

    void saveBattery() const;

    bool isLoaded() const { return m_loaded; }
    const std::string& path() const { return m_path; }
    const std::string& fileName() const { return m_fileName; }
    uint32_t prgBanks() const { return static_cast<uint32_t>((m_prgRom.size() + 0x3FFF) / 0x4000); }
    uint32_t chrBanks() const { return static_cast<uint32_t>((m_chrRom.size() + 0x1FFF) / 0x2000); }
    std::size_t prgRomSize() const { return m_prgRom.size(); }
    std::size_t chrRomSize() const { return m_chrRom.size(); }
    std::size_t prgRamSize() const { return m_prgRam.size(); }
    std::size_t chrRamSize() const { return m_chrRam.size(); }
    std::size_t prgNvRamSize() const { return m_prgNvRamSize; }
    std::size_t chrNvRamSize() const { return m_chrNvRamSize; }
    uint16_t mapper() const { return m_mapperId; }
    uint8_t submapper() const { return m_submapper; }
    bool isNes20() const { return m_nes20; }
    bool isFds() const { return m_fds; }
    ConsoleTiming timing() const { return m_timing; }
    bool isMultiRegion() const { return m_multiRegion; }
    bool mapperSupported() const { return m_mapper && m_mapper->implementationSupported(); }
    const std::string& lastError() const { return m_lastError; }
    Mirror mirroring() const { return m_mapper ? m_mapper->mirroring() : m_headerMirror; }
    bool hasChrRam() const { return !m_chrRam.empty(); }
    bool hasBattery() const { return m_battery; }
    bool irqActive() const { return m_mapper && m_mapper->irqActive(); }
    float expansionAudioSample(bool chipMod = false) const { return m_mapper ? m_mapper->expansionAudioSample(chipMod) : 0.0f; }
    uint64_t romIdentity() const;
    CheatSystem& cheats() { return m_cheats; }
    const CheatSystem& cheats() const { return m_cheats; }
    std::size_t diskSideCount() const { return m_mapper ? m_mapper->diskSideCount() : 0; }
    int currentDiskSide() const { return m_mapper ? m_mapper->currentDiskSide() : -1; }
    bool diskInserted() const { return m_mapper && m_mapper->diskInserted(); }
    bool setDiskSide(std::size_t side) { return m_mapper && m_mapper->setDiskSide(side); }
    void ejectDisk() { if (m_mapper) m_mapper->ejectDisk(); }

    uint8_t cpuRead(uint16_t addr);

    bool cpuRead(uint16_t addr, uint8_t& data);
    bool debugCpuRead(uint16_t addr, uint8_t& data) const;
    void    cpuWrite(uint16_t addr, uint8_t data, uint64_t cpuCycle = ~uint64_t(0));
    void    observeCpuWrite(uint16_t addr, uint8_t data);
    uint8_t ppuRead(uint16_t addr, PpuFetchKind kind = PpuFetchKind::Cpu);
    void    ppuWrite(uint16_t addr, uint8_t data);
    bool    mapPatternCiram(uint16_t addr, uint32_t& mapped) const;
    bool    mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const;
    bool    mapNametableWrite(uint16_t addr, NametableSource& source, uint32_t& mapped) const;
    uint8_t readNametableBacking(NametableSource source, uint32_t mapped) const;
    void    writeNametableBacking(NametableSource source, uint32_t mapped, uint8_t data);

    void notifyCpuAddress(uint16_t addr);
    void notifyPpuAddress(uint16_t addr, uint64_t ppuCycle, int scanline, int dot);
    void notifyPpuScanline(int scanline, bool rendering);
    void clockCpu();
    void scanlineTick();
    void resetMapper(bool hard);

    bool hardResetBootstrap(uint16_t normalVector, uint16_t& entry, bool& jsr) const;

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

private:
    CPU* m_cpu = nullptr;
    bool m_loaded = false;
    bool m_battery = false;
    bool m_nes20 = false;
    bool m_fds = false;
    ConsoleTiming m_timing = ConsoleTiming::NTSC;
    bool m_multiRegion = false;
    bool m_hasTrainer = false;
    uint16_t m_trainerLoadAddress = 0x7000;

    std::vector<uint8_t> m_prgRom;
    std::vector<uint8_t> m_chrRom;
    std::vector<uint8_t> m_chrRam;
    std::vector<uint8_t> m_prgRam;
    std::vector<uint8_t> m_identityData;

    std::size_t m_prgNvRamSize = 0;
    std::size_t m_chrNvRamSize = 0;

    uint16_t m_mapperId = 0;
    uint8_t m_submapper = 0;
    Mirror m_headerMirror = Mirror::Horizontal;
    std::unique_ptr<Mapper> m_mapper;
    CheatSystem m_cheats;

    std::string m_path;
    std::string m_fileName;
    std::string m_batteryPath;
    std::string m_lastError;

    void resetImage();
    void loadBattery();
};
