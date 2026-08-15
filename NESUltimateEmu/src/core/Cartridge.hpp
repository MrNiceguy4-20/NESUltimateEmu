#pragma once
#include <cstdint>
#include <vector>
#include <string>

class CPU;

class Cartridge {
public:
    enum class Mirror {
        Horizontal,
        Vertical,
        OnescreenLo,
        OnescreenHi
    };

    Cartridge();

    bool loadFromFile(const std::string& path);
    void connectCPU(CPU* cpu) { m_cpu = cpu; }

    // Call on unload / exit to persist battery RAM
    void saveBattery() const;

    bool isLoaded() const { return m_loaded; }
    const std::string& path() const { return m_path; }
    const std::string& fileName() const { return m_fileName; }
    uint8_t prgBanks() const { return m_prgBanks; }
    uint8_t chrBanks() const { return m_chrBanks; }
    uint8_t mapper() const { return m_mapper; }
    Mirror mirroring() const { return m_mirror; }
    bool hasChrRam() const { return m_chrBanks == 0; }
    bool hasBattery() const { return m_battery; }

    uint8_t cpuRead(uint16_t addr) const;
    void    cpuWrite(uint16_t addr, uint8_t data);
    uint8_t ppuRead(uint16_t addr) const;
    void    ppuWrite(uint16_t addr, uint8_t data);

    void scanlineTick();

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

private:
    CPU* m_cpu = nullptr;
    bool m_loaded = false;
    bool m_battery = false;

    std::vector<uint8_t> m_prgRom;
    std::vector<uint8_t> m_chrRom;
    std::vector<uint8_t> m_chrRam;
    std::vector<uint8_t> m_prgRam;

    uint8_t m_prgBanks = 0;
    uint8_t m_chrBanks = 0;
    uint8_t m_mapper = 0;
    Mirror  m_mirror = Mirror::Horizontal;

    std::string m_path;
    std::string m_fileName;
    std::string m_batteryPath;

    // Mapper 1
    uint8_t m_shiftReg = 0x10;
    uint8_t m_mmc1Ctrl = 0x0C;
    uint8_t m_mmc1Chr0 = 0;
    uint8_t m_mmc1Chr1 = 0;
    uint8_t m_mmc1Prg = 0;

    // Mapper 2
    uint8_t m_unromBank = 0;

    // Mapper 3
    uint8_t m_cnromBank = 0;

    // Mapper 4 MMC3
    uint8_t  m_mmc3BankSelect = 0;
    uint8_t  m_mmc3Regs[8] = {};
    uint8_t  m_mmc3PrgMode = 0;
    uint8_t  m_mmc3ChrMode = 0;
    uint8_t  m_mmc3IrqLatch = 0;
    uint8_t  m_mmc3IrqCounter = 0;
    bool     m_mmc3IrqEnabled = false;
    bool     m_mmc3IrqReload = false;

    // Mapper 7 AxROM
    uint8_t m_axromBank = 0;

    void mmc1Write(uint16_t addr, uint8_t data);
    void applyMmc1Mirroring();
    void mmc3Write(uint16_t addr, uint8_t data);
    uint32_t mmc3MapPrg(uint16_t addr) const;
    uint32_t mmc3MapChr(uint16_t addr) const;

    void loadBattery();
};







