#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

enum class Mirror {
    Horizontal,
    Vertical,
    OnescreenLo,
    OnescreenHi,
    FourScreen
};

enum class NametableSource {
    Ciram,
    ChrRom,
    ChrRam,
    MapperRam
};

enum class PpuFetchKind {
    Cpu,
    Background,
    Sprite
};

struct MapperConfig {
    uint16_t id = 0;
    uint8_t submapper = 0;
    std::size_t prgRomSize = 0;
    std::size_t chrRomSize = 0;
    std::size_t prgRamSize = 0;
    std::size_t chrRamSize = 0;
    Mirror headerMirror = Mirror::Horizontal;
    bool fourScreen = false;
    std::size_t headerPrgNvRamSize = 0;
    std::size_t headerChrNvRamSize = 0;
    bool nes20 = false;
    bool hasBattery = false;
    // Hash/database-resolved physical board variant for ambiguous legacy dumps.
    uint8_t boardVariant = 0;
};

class Mapper {
public:
    explicit Mapper(const MapperConfig& config);
    virtual ~Mapper() = default;

    virtual bool cpuMapRead(uint16_t addr, uint32_t& mapped) const = 0;
    virtual bool cpuReadRegister(uint16_t addr, uint8_t& data);
    // Observe a normal CPU memory read after the cartridge has supplied the
    // byte. Boards such as MMC5 use reads from PRG space as an audio side
    // effect (PCM read mode).
    virtual void observeCpuRead(uint16_t addr, uint8_t data);
    virtual void observeCpuWrite(uint16_t addr, uint8_t data);
    virtual bool cpuWrite(uint16_t addr, uint8_t data, uint64_t cpuCycle);
    virtual bool hasBusConflicts() const { return false; }
    virtual uint8_t resolveBusConflict(uint16_t addr, uint8_t cpuData, uint8_t romData) const;
    virtual bool ppuMapRead(uint16_t addr, uint32_t& mapped);
    virtual bool ppuMapReadEx(uint16_t addr, uint32_t& mapped, PpuFetchKind kind);
    // Optional direct-value path for mapper hardware that deliberately leaves
    // CHR disabled/open-bus for specific PPU accesses. Returning true supplies
    // data directly and bypasses CHR ROM/RAM mapping for this read.
    virtual bool ppuReadOverride(uint16_t addr, PpuFetchKind kind, uint8_t& data);
    virtual bool ppuMapWrite(uint16_t addr, uint32_t& mapped);
    virtual bool ppuUsesChrRam(uint16_t addr) const;
    virtual bool mapPatternCiram(uint16_t addr, uint32_t& mapped) const;
    virtual bool mapNametable(uint16_t addr, NametableSource& source, uint32_t& mapped) const;
    virtual bool mapNametableWrite(uint16_t addr, NametableSource& source, uint32_t& mapped) const;
    virtual uint8_t readMapperNametable(uint32_t mapped) const;
    virtual void writeMapperNametable(uint32_t mapped, uint8_t data);
    virtual bool mapPrgRam(uint16_t addr, uint32_t& mapped, bool write) const;

    virtual void notifyPpuAddress(uint16_t addr, uint64_t ppuCycle);
    virtual void notifyPpuAddressContext(uint16_t addr, uint64_t ppuCycle, int scanline, int dot);
    virtual void notifyPpuScanline(int scanline, bool rendering);
    virtual void clockCpu();
    virtual void scanlineTick();
    // Cartridge reset lifecycle. hard=true is a cold power-on; hard=false is
    // the console Reset button. Most boards ignore reset, while reset-based
    // multicarts use it to advance an outer selector.
    virtual void reset(bool hard);
    // Initialize mapper-owned writable PRG devices (flash carts) from the ROM image.
    virtual void initializePrgImage(const uint8_t* data, std::size_t size);
    virtual bool irqActive() const;
    virtual float expansionAudioSample(bool chipMod = false) const;

    virtual std::size_t mapperBatterySize() const;
    virtual void saveMapperBattery(std::vector<uint8_t>& out) const;
    virtual bool loadMapperBattery(const uint8_t* data, std::size_t size);

    // Optional disk-system hooks. Cartridge images ignore these; Mapper 20/FDS
    // implements them so the frontend does not need to know mapper internals.
    virtual bool loadDiskImage(const std::vector<uint8_t>& rawImage);
    virtual std::size_t diskSideCount() const;
    virtual int currentDiskSide() const;
    virtual bool diskInserted() const;
    virtual bool setDiskSide(std::size_t side);
    virtual void ejectDisk();

    virtual void saveState(std::vector<uint8_t>& out) const;
    virtual bool loadState(const uint8_t*& p, const uint8_t* end);

    virtual bool implementationSupported() const { return true; }

    Mirror mirroring() const { return m_mirror; }
    uint16_t id() const { return m_config.id; }
    uint8_t submapper() const { return m_config.submapper; }

protected:
    MapperConfig m_config;
    Mirror m_mirror;

    std::size_t prgBanks(std::size_t bankSize) const;
    std::size_t chrBanks(std::size_t bankSize) const;
};

std::unique_ptr<Mapper> createMapper(const MapperConfig& config);
bool mapperImplementationSupported(uint16_t mapper, uint8_t submapper = 0);
