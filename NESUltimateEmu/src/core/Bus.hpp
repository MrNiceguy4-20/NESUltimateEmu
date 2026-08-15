#pragma once
#include <cstdint>
#include <vector>

class CPU;
class PPU;
class APU;
class Cartridge;

class Bus {
public:
    Bus();

    void connectCPU(CPU* cpu);
    void connectPPU(PPU* ppu);
    void connectAPU(APU* apu);
    void connectCartridge(Cartridge* cart);

    void clock();

    // OAM DMA: writing $4014 starts a 256-byte CPU-stalling transfer.
    bool dmaActive() const { return m_dmaActive; }

    uint8_t read(uint16_t addr) const;
    void    write(uint16_t addr, uint8_t data);

    // Controller: bit0=A bit1=B bit2=Select bit3=Start bit4=Up bit5=Down bit6=Left bit7=Right
    void setController1(uint8_t state) { m_controller1 = state; }
    void setController2(uint8_t state) { m_controller2 = state; }

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

private:
    CPU* m_cpu = nullptr;
    PPU* m_ppu = nullptr;
    APU* m_apu = nullptr;
    Cartridge* m_cart = nullptr;

    uint8_t m_ram[2048] = {};

    uint8_t m_controller1 = 0;
    uint8_t m_controller2 = 0;
    mutable uint8_t m_controller1Shift = 0;
    mutable uint8_t m_controller2Shift = 0;
    bool    m_strobe = false;

    // CPU-cycle counter used for OAM DMA alignment. It advances once per
    // master CPU clock, including cycles stolen by DMA.
    uint64_t m_cpuCycleCounter = 0;

    bool    m_dmaActive = false;
    bool    m_dmaDummy = false;
    bool    m_dmaReadPhase = true;
    uint16_t m_dmaPage = 0;
    uint8_t  m_dmaAddress = 0;
    uint8_t  m_dmaData = 0;
    uint8_t  m_dmaDummyCycles = 0;

    void startOamDma(uint8_t page);
    void clockOamDma();
};



