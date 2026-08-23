#pragma once
#include <cstdint>
#include <vector>
#include "Timing.hpp"

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

    void reset();

    void powerOn();

    void clock();
    void setTiming(ConsoleTiming timing);

    void applyCartridgeResetBootstrap(uint16_t& pc, uint8_t& sp);
    ConsoleTiming timing() const { return m_timing; }

    bool dmaActive() const { return m_dmaActive; }

    bool requestDmcDma(uint16_t addr, bool abortAfterHalt = false);

    bool cancelDmcDma();
    void stopDmcDma();

    uint8_t read(uint16_t addr) const;
    void    write(uint16_t addr, uint8_t data);

    uint64_t cpuCycleCounter() const { return m_cpuCycleCounter; }

    bool dmaGetCycle() const { return (m_cpuCycleCounter & 1u) != 0; }
    bool dmcDmaActive() const { return m_dmcDmaActive; }
#ifdef NES_HEADLESS

    void testStartOamDma(uint8_t page, uint16_t haltedCpuAddress = 0xFFFF) {
        m_dmaCpuReadAddress = haltedCpuAddress;
        m_dmaCpuReadAddressValid = haltedCpuAddress != 0xFFFF;
        startOamDma(page);
    }
    uint8_t testReadOamDmaSource(uint16_t addr, uint16_t haltedCpuAddress) const {
        m_dmaCpuReadAddress = haltedCpuAddress;
        m_dmaCpuReadAddressValid = true;
        return readOamDmaSource(addr);
    }
    uint8_t testDmcDmaPhase() const { return static_cast<uint8_t>(m_dmcDmaPhase); }
    uint8_t testExternalDataBus() const { return m_cpuDataBus; }
    uint8_t testInternalDataBus() const { return m_cpuInternalDataBus; }
    void testSetInternalDataBus(uint8_t value) { m_cpuInternalDataBus = value; }
#endif

    void setController1(uint8_t state) { m_controller1 = state; }
    void setController2(uint8_t state) { m_controller2 = state; }
#ifdef NES_HEADLESS

    void testLatchControllers() {
        m_controller1Shift = m_controller1;
        m_controller2Shift = m_controller2;
    }

    uint8_t testPeekCpuRam(uint16_t addr) const { return m_ram[addr & 0x07FF]; }
#endif

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

    mutable uint8_t m_controllerReadActivePort = 0;
    mutable uint8_t m_controllerReadLatched1 = 0;
    mutable uint8_t m_controllerReadLatched2 = 0;

    mutable uint8_t m_cpuDataBus = 0;

    mutable uint8_t m_cpuInternalDataBus = 0;

    uint64_t m_cpuCycleCounter = 0;
    ConsoleTiming m_timing = ConsoleTiming::NTSC;
    uint8_t m_ppuClockAccumulator = 0;
    bool m_hardResetBootstrapPending = false;

    bool    m_dmaRequestPending = false;
    uint8_t m_dmaPendingPage = 0;
    bool    m_dmaActive = false;
    bool    m_dmaDummy = false;
    bool    m_dmaReadPhase = true;
    uint16_t m_dmaPage = 0;
    uint8_t  m_dmaAddress = 0;
    uint8_t  m_dmaData = 0;
    uint8_t  m_dmaDummyCycles = 0;

    mutable uint16_t m_dmaCpuReadAddress = 0;
    mutable bool m_dmaCpuReadAddressValid = false;

    enum class DmcDmaPhase : uint8_t {
        Halt = 0,
        Dummy,
        Align,
        Get
    };
    bool        m_dmcDmaActive = false;

    DmcDmaPhase m_dmcDmaPhase = DmcDmaPhase::Halt;
    uint16_t m_dmcDmaAddress = 0;
    uint16_t m_dmcDmaCpuReadAddress = 0;
    bool     m_dmcDmaNeedsAlign = false;
    bool     m_dmcDmaAbortAfterHalt = false;

    uint8_t driveCpuDataBus(uint8_t value, uint8_t drivenMask = 0xFF) const;
    uint8_t driveExternalCpuDataBus(uint8_t value, uint8_t drivenMask = 0xFF) const;
    uint8_t driveInternalCpuDataBus(uint8_t value, uint8_t drivenMask = 0xFF) const;
    uint8_t readControllerPort(uint8_t port) const;
    uint8_t repeatDmcStalledCpuRead() const;
    uint8_t readDmcExternalSample() const;
    uint8_t readDmcSampleWithCpuConflict() const;
    uint8_t readOamDmaSource(uint16_t addr) const;
    void releaseControllerReadLine() const;
    void startOamDma(uint8_t page);
    void clearDmaState();
    void clockOamDma();
    bool clockDmcDma();
    void traceDmc(const char* event) const;
    void resetDmcTrace() const;
};
