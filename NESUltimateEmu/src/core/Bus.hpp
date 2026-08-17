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

    // Console Reset button: cancel in-flight DMA and reset CPU/APU/PPU live
    // state, but preserve CPU/PPU memory and the running master-cycle epoch.
    void reset();

    // Cold boot used after loading a new cartridge. Clears console-owned RAM,
    // controller/DMA/timing state, powers on the PPU/APU, then resets the CPU
    // so its reset vector is read from the newly loaded cartridge.
    void powerOn();

    void clock();

    // OAM DMA: writing $4014 starts a 256-byte CPU-stalling transfer.
    bool dmaActive() const { return m_dmaActive; }

    // DMC DMA is requested by the APU when its sample buffer is empty.
    // Returns false only if a DMC transfer is already in flight.
    bool requestDmcDma(uint16_t addr);

    uint8_t read(uint16_t addr) const;
    void    write(uint16_t addr, uint8_t data);

    // Side-effect-free CPU address-space read for debugger/disassembly tools.
    uint8_t debugRead(uint16_t addr) const;
    uint64_t cpuCycleCounter() const { return m_cpuCycleCounter; }
    bool dmcDmaActive() const { return m_dmcDmaActive; }

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

    // Value currently present on the CPU data bus. Reads from write-only or
    // physically unconnected address ranges return this value (open bus), and
    // partially-driven registers replace only the bits they actually drive.
    mutable uint8_t m_cpuDataBus = 0;

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

    bool     m_dmcDmaActive = false;
    uint8_t  m_dmcDmaPhase = 0; // halt, dummy, align, read
    uint16_t m_dmcDmaAddress = 0;

    uint8_t driveCpuDataBus(uint8_t value, uint8_t drivenMask = 0xFF) const;
    void startOamDma(uint8_t page);
    void clearDmaState();
    void clockOamDma();
    bool clockDmcDma(); // true when the DMC used the memory bus this cycle
};







