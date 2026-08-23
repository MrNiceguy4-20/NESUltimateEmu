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

    // Console Reset button: cancel in-flight DMA and reset CPU/APU/PPU live
    // state, but preserve CPU/PPU memory and the running master-cycle epoch.
    void reset();

    // Cold boot used after loading a new cartridge. Clears console-owned RAM,
    // controller/DMA/timing state, powers on the PPU/APU, then powers on the
    // CPU so its architectural power values and reset vector are initialized.
    void powerOn();

    void clock();
    void setTiming(ConsoleTiming timing);
    // Called by the CPU after fetching the reset vector. RAM-cartridge
    // extracts can redirect a cold boot through their trainer bootstrap.
    void applyCartridgeResetBootstrap(uint16_t& pc, uint8_t& sp);
    ConsoleTiming timing() const { return m_timing; }

    // OAM DMA: writing $4014 starts a 256-byte CPU-stalling transfer.
    bool dmaActive() const { return m_dmaActive; }

    // DMC DMA is requested by the APU when its sample buffer is empty.
    // Returns false only if a DMC transfer is already in flight.
    bool requestDmcDma(uint16_t addr, bool abortAfterHalt = false);
    // Cancel a request that has not yet acquired RDY. Used by an explicit
    // $4015 DMC stop that lands before the DMA halt cycle.
    bool cancelDmcDma();
    void stopDmcDma();

    uint8_t read(uint16_t addr) const;
    void    write(uint16_t addr, uint8_t data);

    uint64_t cpuCycleCounter() const { return m_cpuCycleCounter; }
    // Canonical DMA cadence. GET/PUT is an APU-divider phase, not an
    // intrinsic even/odd CPU-cycle property. This baseline selects the
    // odd-counter alignment so every DMA user shares one phase definition.
    bool dmaGetCycle() const { return (m_cpuCycleCounter & 1u) != 0; }
    bool dmcDmaActive() const { return m_dmcDmaActive; }
#ifdef NES_HEADLESS
    // Test-only DMA hooks. They expose scheduler state without bypassing the
    // production clock paths, allowing deterministic arbitration probes.
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

    // Controller: bit0=A bit1=B bit2=Select bit3=Start bit4=Up bit5=Down bit6=Left bit7=Right
    void setController1(uint8_t state) { m_controller1 = state; }
    void setController2(uint8_t state) { m_controller2 = state; }
#ifdef NES_HEADLESS
    // Test-only hook: emulate a controller parallel-load event without
    // advancing the CPU scheduler. Production code samples the strobe on PUT.
    void testLatchControllers() {
        m_controller1Shift = m_controller1;
        m_controller2Shift = m_controller2;
    }
    // Side-effect-free CPU RAM inspection for external conformance harnesses.
    // This deliberately bypasses the CPU data-bus/open-bus latches so observing
    // a test ROM cannot change the behavior being measured.
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

    // US NES / AV Famicom controller clocking is edge-like: consecutive
    // read cycles from the same controller port keep the CLK line asserted,
    // so the serial register advances only once and both reads return the
    // same bit. 0 = inactive, 1 = $4016, 2 = $4017.
    mutable uint8_t m_controllerReadActivePort = 0;
    mutable uint8_t m_controllerReadLatched1 = 0;
    mutable uint8_t m_controllerReadLatched2 = 0;

    // Value currently present on the CPU data bus. Reads from write-only or
    // physically unconnected address ranges return this value (open bus), and
    // partially-driven registers replace only the bits they actually drive.
    mutable uint8_t m_cpuDataBus = 0;
    // Internal 2A03 data latch. Ordinary CPU bus accesses feed both latches,
    // but a DMC sample GET drives only the external pins. $4015 is the inverse:
    // its status bits are placed only on this internal path.
    mutable uint8_t m_cpuInternalDataBus = 0;

    // CPU-cycle counter used for OAM DMA alignment. It advances once per
    // master CPU clock, including cycles stolen by DMA.
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
    // OAM DMA holds the 6502 core on the read cycle where RDY was acquired.
    // The APU/I/O decoder continues seeing A15-A5 from this frozen address
    // throughout the transfer, while A4-A0 come from the OAM address bus.
    mutable uint16_t m_dmaCpuReadAddress = 0;
    mutable bool m_dmaCpuReadAddressValid = false;

    enum class DmcDmaPhase : uint8_t {
        Halt = 0,
        Dummy,
        Align,
        Get
    };
    bool        m_dmcDmaActive = false;
    // A DMC request first attempts an RDY halt. After that the DMA unit
    // performs one dummy cycle, an optional alignment cycle, and one GET.
    // Keeping these phases explicit is important when OAM DMA overlaps: only
    // the GET contends for the shared memory bus.
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
    bool clockDmcDma(); // true when the DMC used the memory bus this cycle
    void traceDmc(const char* event) const;
    void resetDmcTrace() const;
};







