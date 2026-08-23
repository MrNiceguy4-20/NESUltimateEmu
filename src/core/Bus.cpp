#include "Bus.hpp"
#include <cstring>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include "CPU.hpp"
#include "PPU.hpp"
#include "APU.hpp"
#include "Cartridge.hpp"

Bus::Bus() {}

namespace {
[[maybe_unused]] const char* dmcPhaseName(uint8_t phase)
{
    switch (phase) {
    case 0: return "HALT";
    case 1: return "DUMMY";
    case 2: return "ALIGN";
    case 3: return "GET";
    default: return "?";
    }
}

[[maybe_unused]] const char* cpuCycleTypeName(CPU::BusCycleType type)
{
    switch (type) {
    case CPU::BusCycleType::Read: return "R";
    case CPU::BusCycleType::Write: return "W";
    default: return "-";
    }
}
}

void Bus::resetDmcTrace() const
{
}

void Bus::traceDmc(const char* event) const
{
    (void)event;
}

void Bus::connectCPU(CPU* cpu) { m_cpu = cpu; }
void Bus::connectPPU(PPU* ppu) { m_ppu = ppu; }
void Bus::connectAPU(APU* apu) { m_apu = apu; }
void Bus::connectCartridge(Cartridge* cart) { m_cart = cart; }
void Bus::setTiming(ConsoleTiming timing)
{
    m_timing = timing;
    m_ppuClockAccumulator = 0;
    if (m_ppu) m_ppu->setTiming(timing);
    if (m_apu) m_apu->setTiming(timing);
}

void Bus::clearDmaState()
{
    m_dmaRequestPending = false;
    m_dmaPendingPage = 0;
    m_dmaActive = false;
    m_dmaDummy = false;
    m_dmaReadPhase = true;
    m_dmaPage = 0;
    m_dmaAddress = 0;
    m_dmaData = 0;
    m_dmaDummyCycles = 0;
    m_dmaCpuReadAddress = 0;
    m_dmaCpuReadAddressValid = false;

    m_dmcDmaActive = false;
    m_dmcDmaPhase = DmcDmaPhase::Halt;
    m_dmcDmaAddress = 0;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
}

void Bus::reset()
{
    clearDmaState();

    m_controller1Shift = m_controller1;
    m_controller2Shift = m_controller2;
    m_strobe = false;
    m_controllerReadActivePort = 0;
    m_controllerReadLatched1 = 0;
    m_controllerReadLatched2 = 0;

    m_hardResetBootstrapPending = false;
    if (m_cart) m_cart->resetMapper(false);
    if (m_ppu) m_ppu->reset();
    if (m_apu) m_apu->reset();
    if (m_cpu) m_cpu->reset();
}

void Bus::powerOn()
{
    resetDmcTrace();
    std::memset(m_ram, 0, sizeof(m_ram));
    m_controller1 = 0;
    m_controller2 = 0;
    m_controller1Shift = 0;
    m_controller2Shift = 0;
    m_strobe = false;
    m_controllerReadActivePort = 0;
    m_controllerReadLatched1 = 0;
    m_controllerReadLatched2 = 0;
    m_cpuDataBus = 0;
    m_cpuInternalDataBus = 0;
    m_cpuCycleCounter = 0;
    m_ppuClockAccumulator = 0;
    clearDmaState();

    m_hardResetBootstrapPending = true;

    if (m_cart) m_cart->resetMapper(true);
    if (m_ppu) m_ppu->powerOn();
    if (m_apu) m_apu->powerOn();

    if (m_cpu) m_cpu->powerOn();
}

void Bus::applyCartridgeResetBootstrap(uint16_t& pc, uint8_t& sp)
{
    if (!m_hardResetBootstrapPending) return;
    m_hardResetBootstrapPending = false;
    if (!m_cart) return;

    uint16_t entry = 0;
    bool jsr = false;
    if (!m_cart->hardResetBootstrap(pc, entry, jsr)) return;

    if (jsr) {

        const uint16_t ret = static_cast<uint16_t>(pc - 1);
        write(static_cast<uint16_t>(0x0100 | sp), static_cast<uint8_t>(ret >> 8)); --sp;
        write(static_cast<uint16_t>(0x0100 | sp), static_cast<uint8_t>(ret)); --sp;
    }
    pc = entry;
}

void Bus::clock()
{

    int ppuClocksThisCpu = 3;
    if (m_timing == ConsoleTiming::PAL) {
        m_ppuClockAccumulator = static_cast<uint8_t>(m_ppuClockAccumulator + 16);
        ppuClocksThisCpu = m_ppuClockAccumulator / 5;
        m_ppuClockAccumulator = static_cast<uint8_t>(m_ppuClockAccumulator % 5);
    }
    const int ppuBeforeCpu = std::min(2, ppuClocksThisCpu);
    if (m_ppu) {
        for (int i = 0; i < ppuBeforeCpu; ++i) {
            m_ppu->clock();
            if (i == 0 && m_cpu)
                m_cpu->sampleNmiInput();
        }
    }
    else if (m_cpu) {
        m_cpu->sampleNmiInput();
    }

    if (m_apu) {
        m_apu->clock();

        m_apu->clockFrameCounterPreCpuPhase();
    }
    if (m_cart)
        m_cart->clockCpu();

    if (m_cpu) {
        const bool apuIrq = m_apu && m_apu->irqActive();
        const bool cartIrq = m_cart && m_cart->irqActive();
        m_cpu->setIrqLine(apuIrq || cartIrq);
    }

    const bool dmcWasActive = m_dmcDmaActive;
    if (m_dmcDmaActive) traceDmc("CLOCK_BEGIN");
    const CPU::BusCycle cpuBusCycle = m_cpu ? m_cpu->nextBusCycle() : CPU::BusCycle{};

    if (m_dmaRequestPending && !m_dmaActive && m_cpu &&
        cpuBusCycle.type == CPU::BusCycleType::Read) {
        const uint8_t page = m_dmaPendingPage;
        m_dmaRequestPending = false;
        m_dmaCpuReadAddress = cpuBusCycle.address;
        m_dmaCpuReadAddressValid = true;
        startOamDma(page);
    }

    const bool dmcBlockedByCpuWrite =
        !m_dmaActive && dmcWasActive && m_dmcDmaPhase == DmcDmaPhase::Halt &&
        m_cpu && cpuBusCycle.type == CPU::BusCycleType::Write;

    if (dmcBlockedByCpuWrite && m_dmcDmaAbortAfterHalt) {
        traceDmc("ABORT_HALT_SUPPRESSED_BY_WRITE");
        m_dmcDmaActive = false;
        m_dmcDmaPhase = DmcDmaPhase::Halt;
        m_dmcDmaAddress = 0;
        m_dmcDmaCpuReadAddress = 0;
        m_dmcDmaNeedsAlign = false;
        m_dmcDmaAbortAfterHalt = false;
        if (m_apu)
            m_apu->abortDmcDma();
    }

    const bool oamGetCycle = dmaGetCycle();
    const bool oamNeedsBus = m_dmaActive && !m_dmaDummy &&
        (m_dmaReadPhase == oamGetCycle);

    bool dmcUsedBus = false;
    if (dmcWasActive && !dmcBlockedByCpuWrite)
        dmcUsedBus = clockDmcDma();

    if (m_dmaActive && (!dmcUsedBus || !oamNeedsBus))
        clockOamDma();
    else if (!m_dmaActive && (!dmcWasActive || dmcBlockedByCpuWrite) && m_cpu)
        m_cpu->clock();

    if (dmaGetCycle() && m_strobe) {
        m_controller1Shift = m_controller1;
        m_controller2Shift = m_controller2;
    }

    if (m_apu)
        m_apu->clockFrameCounterPhase();

    if (m_ppu)
        for (int i = ppuBeforeCpu; i < ppuClocksThisCpu; ++i) m_ppu->clock();

    ++m_cpuCycleCounter;
}

bool Bus::requestDmcDma(uint16_t addr, bool abortAfterHalt)
{
    traceDmc(abortAfterHalt ? "REQUEST_ABORT1" : "REQUEST_NORMAL");
    if (m_dmcDmaActive)
        return false;

    m_dmcDmaActive = true;
    m_dmcDmaPhase = DmcDmaPhase::Halt;
    m_dmcDmaAddress = addr;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = abortAfterHalt;
    traceDmc("REQUEST_ACCEPTED");
    return true;
}

bool Bus::cancelDmcDma()
{
    traceDmc("CANCEL_REQUEST");
    if (!m_dmcDmaActive || m_dmcDmaPhase != DmcDmaPhase::Halt)
        return false;

    m_dmcDmaActive = false;
    m_dmcDmaPhase = DmcDmaPhase::Halt;
    m_dmcDmaAddress = 0;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
    return true;
}

void Bus::stopDmcDma()
{
    traceDmc("STOP_REQUEST");
    if (!m_dmcDmaActive)
        return;

    if (m_dmcDmaPhase == DmcDmaPhase::Halt) {
        m_dmcDmaAbortAfterHalt = true;
        traceDmc("STOP_ARM_ABORT1");
        return;
    }

    m_dmcDmaActive = false;
    m_dmcDmaPhase = DmcDmaPhase::Halt;
    m_dmcDmaAddress = 0;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
    if (m_apu)
        m_apu->abortDmcDma();
}

bool Bus::clockDmcDma()
{
    if (!m_dmcDmaActive)
        return false;

    if (m_dmcDmaPhase == DmcDmaPhase::Halt) {
        traceDmc("HALT_ATTEMPT");
        m_dmcDmaNeedsAlign = !dmaGetCycle();

        if (!m_dmaActive) {
            if (m_cpu) {
                const CPU::BusCycle cpuCycle = m_cpu->nextBusCycle();
                if (cpuCycle.type == CPU::BusCycleType::Read) {
                    m_dmcDmaCpuReadAddress = cpuCycle.address;
                    m_cpu->notifyRdyReadStall();
                    traceDmc("HALT_ACQUIRED");
                }
            }

            (void)repeatDmcStalledCpuRead();
        }

        if (m_dmcDmaAbortAfterHalt) {
            traceDmc("ABORT_AFTER_HALT");
            m_dmcDmaActive = false;
            m_dmcDmaPhase = DmcDmaPhase::Halt;
            m_dmcDmaAddress = 0;
            m_dmcDmaCpuReadAddress = 0;
            m_dmcDmaNeedsAlign = false;
            m_dmcDmaAbortAfterHalt = false;
            if (m_apu)
                m_apu->abortDmcDma();
            return false;
        }

        m_dmcDmaPhase = DmcDmaPhase::Dummy;
        traceDmc("ENTER_DUMMY");
        return false;
    }

    if (m_dmcDmaPhase == DmcDmaPhase::Dummy) {
        traceDmc("DUMMY");

        if (!m_dmaActive)
            (void)repeatDmcStalledCpuRead();
        m_dmcDmaPhase = DmcDmaPhase::Align;
        traceDmc("ENTER_ALIGN");
        return false;
    }

    if (m_dmcDmaPhase == DmcDmaPhase::Align && m_dmcDmaNeedsAlign) {
        traceDmc("ALIGN");

        if (!m_dmaActive)
            (void)repeatDmcStalledCpuRead();
        m_dmcDmaPhase = DmcDmaPhase::Get;
        traceDmc("ENTER_GET");
        return false;
    }

    traceDmc("GET_COMMIT");
    const uint8_t data = readDmcSampleWithCpuConflict();
    m_dmcDmaActive = false;
    m_dmcDmaPhase = DmcDmaPhase::Halt;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
    if (m_apu)
        m_apu->completeDmcDma(data);
    traceDmc("GET_COMPLETE");
    return true;
}

uint8_t Bus::repeatDmcStalledCpuRead() const
{

    return read(m_dmcDmaCpuReadAddress);
}

uint8_t Bus::readDmcExternalSample() const
{

    uint8_t data = m_cpuDataBus;
    if (m_cart && m_cart->cpuRead(m_dmcDmaAddress, data))
        return driveExternalCpuDataBus(data);
    return m_cpuDataBus;
}

uint8_t Bus::readDmcSampleWithCpuConflict() const
{

    const uint8_t externalData = readDmcExternalSample();

    const bool cpuInApuIoRegion =
        (m_dmcDmaCpuReadAddress & 0xFFE0u) == 0x4000u;
    if (!cpuInApuIoRegion) {

        releaseControllerReadLine();
        return externalData;
    }

    const uint16_t activated = static_cast<uint16_t>(
        0x4000u | (m_dmcDmaAddress & 0x001Fu));

    if (activated != 0x4016 && activated != 0x4017)
        releaseControllerReadLine();

    if (activated == 0x4015) {

        return m_apu ? m_apu->cpuRead(0x4015) : 0;
    }

    if (activated == 0x4016)
        return readControllerPort(1);
    if (activated == 0x4017)
        return readControllerPort(2);

    return externalData;
}

uint8_t Bus::readOamDmaSource(uint16_t addr) const
{

    uint8_t external = m_cpuDataBus;
    bool externalDriven = false;

    if (addr <= 0x1FFF) {
        external = driveExternalCpuDataBus(m_ram[addr & 0x07FF]);
        externalDriven = true;
    }
    else if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) {
            external = driveExternalCpuDataBus(m_ppu->cpuRead(addr));
            externalDriven = true;
        }
    }
    else if (addr >= 0x4020) {
        uint8_t data = m_cpuDataBus;
        if (m_cart && m_cart->cpuRead(addr, data)) {
            external = driveExternalCpuDataBus(data);
            externalDriven = true;
        }
    }

    if (!m_dmaCpuReadAddressValid && m_cpu) {
        const CPU::BusCycle held = m_cpu->nextBusCycle();
        if (held.type == CPU::BusCycleType::Read) {
            m_dmaCpuReadAddress = held.address;
            m_dmaCpuReadAddressValid = true;
        }
    }
    const bool cpuEnablesApuIo = m_dmaCpuReadAddressValid &&
        (m_dmaCpuReadAddress & 0xFFE0u) == 0x4000u;

    if (!cpuEnablesApuIo) {
        releaseControllerReadLine();
        return driveInternalCpuDataBus(external);
    }

    const uint16_t activated = static_cast<uint16_t>(0x4000u | (addr & 0x001Fu));
    if (activated == 0x4015) {
        const uint8_t status = m_apu ? m_apu->cpuRead(0x4015) : 0;

        const uint8_t result = static_cast<uint8_t>((external & 0x20) |
                                                    (status & 0xDF));

        if (!externalDriven) {

            const uint8_t driven = 0xDF;
            m_cpuDataBus = static_cast<uint8_t>((external & ~driven) |
                                                (external & status & driven));
        }
        return driveInternalCpuDataBus(result);
    }

    if (activated == 0x4016 || activated == 0x4017) {
        const uint8_t port = activated == 0x4016 ? 1 : 2;
        if (externalDriven) {

            (void)readControllerPort(port);
            return driveInternalCpuDataBus(external);
        }

        const uint8_t oldExternal = external;
        const uint8_t portRead = readControllerPort(port);
        const uint8_t bit = static_cast<uint8_t>(portRead & 0x01);
        const uint8_t result = static_cast<uint8_t>((oldExternal & 0xFE) | bit);
        m_cpuDataBus = static_cast<uint8_t>((oldExternal & 0xFE) |
                                            ((oldExternal & bit) & 0x01));
        return driveInternalCpuDataBus(result);
    }

    releaseControllerReadLine();
    return driveInternalCpuDataBus(external);
}

void Bus::startOamDma(uint8_t page)
{
    m_dmaActive = true;
    m_dmaPage = static_cast<uint16_t>(page) << 8;
    m_dmaAddress = 0;
    m_dmaData = 0;
    m_dmaReadPhase = true;

    m_dmaDummy = true;

    const bool haltIsGet = dmaGetCycle();
    m_dmaDummyCycles = static_cast<uint8_t>(haltIsGet ? 2 : 1);
}

void Bus::clockOamDma()
{
    if (!m_ppu) {
        m_dmaActive = false;
        return;
    }

    if (m_dmaDummy) {
        if (m_dmaDummyCycles > 0)
            --m_dmaDummyCycles;
        if (m_dmaDummyCycles == 0) {
            m_dmaDummy = false;
            m_dmaReadPhase = true;
        }
        return;
    }

    const bool getCycle = dmaGetCycle();
    if (m_dmaReadPhase != getCycle)
        return;

    if (m_dmaReadPhase) {
        m_dmaData = readOamDmaSource(static_cast<uint16_t>(m_dmaPage | m_dmaAddress));
        m_dmaReadPhase = false;
    }
    else {
        m_ppu->oamDmaWrite(m_dmaData);
        m_dmaAddress++;
        m_dmaReadPhase = true;

        if (m_dmaAddress == 0) {
            m_dmaActive = false;
            m_dmaCpuReadAddress = 0;
            m_dmaCpuReadAddressValid = false;
        }
    }
}

uint8_t Bus::driveExternalCpuDataBus(uint8_t value, uint8_t drivenMask) const
{
    m_cpuDataBus = static_cast<uint8_t>((m_cpuDataBus & ~drivenMask) |
                                        (value & drivenMask));
    return m_cpuDataBus;
}

uint8_t Bus::driveInternalCpuDataBus(uint8_t value, uint8_t drivenMask) const
{
    m_cpuInternalDataBus = static_cast<uint8_t>((m_cpuInternalDataBus & ~drivenMask) |
                                                (value & drivenMask));
    return m_cpuInternalDataBus;
}

uint8_t Bus::driveCpuDataBus(uint8_t value, uint8_t drivenMask) const
{

    driveExternalCpuDataBus(value, drivenMask);
    return driveInternalCpuDataBus(value, drivenMask);
}

void Bus::releaseControllerReadLine() const
{
    m_controllerReadActivePort = 0;
}

uint8_t Bus::readControllerPort(uint8_t port) const
{
    const uint8_t active = port == 1 ? 1 : 2;
    uint8_t& shift = port == 1 ? m_controller1Shift : m_controller2Shift;
    uint8_t& latched = port == 1 ? m_controllerReadLatched1 : m_controllerReadLatched2;
    const uint8_t live = port == 1 ? m_controller1 : m_controller2;

    if (m_strobe) {
        latched = live & 0x01;
        m_controllerReadActivePort = active;
        return driveCpuDataBus(latched, 0x01);
    }

    if (m_controllerReadActivePort == active)
        return driveCpuDataBus(latched, 0x01);

    latched = shift & 0x01;
    shift = static_cast<uint8_t>((shift >> 1) | 0x80);
    m_controllerReadActivePort = active;
    return driveCpuDataBus(latched, 0x01);
}

uint8_t Bus::read(uint16_t addr) const
{
    if (addr == 0x4016) {
        const uint8_t value = readControllerPort(1);
        return m_cart ? driveCpuDataBus(m_cart->cheats().applyRawCpuRead(addr, value)) : value;
    }
    if (addr == 0x4017) {
        const uint8_t value = readControllerPort(2);
        return m_cart ? driveCpuDataBus(m_cart->cheats().applyRawCpuRead(addr, value)) : value;
    }

    releaseControllerReadLine();

    if (addr <= 0x1FFF) {
        uint8_t value = m_ram[addr & 0x07FF];
        if (m_cart) value = m_cart->cheats().applyRawCpuRead(addr, value);
        return driveCpuDataBus(value);
    }

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) {
            uint8_t value = m_ppu->cpuRead(addr);
            if (m_cart) value = m_cart->cheats().applyRawCpuRead(addr, value);
            return driveCpuDataBus(value);
        }
        return m_cpuDataBus;
    }

    if (addr == 0x4015) {

        if (m_apu) {
            const uint8_t status = m_apu->cpuRead(addr);
            const uint8_t internal = static_cast<uint8_t>((m_cpuInternalDataBus & 0x20) |
                                                           (status & 0xDF));

            return driveInternalCpuDataBus(internal);
        }
        return m_cpuInternalDataBus;
    }

    if (addr >= 0x4020 && m_cart) {

        uint8_t data = m_cpuDataBus;
        if (m_cart->cpuRead(addr, data)) {
            data = m_cart->cheats().applyRawCpuRead(addr, data);
            return driveCpuDataBus(data);
        }
    }

    return m_cpuDataBus;
}

void Bus::write(uint16_t addr, uint8_t data)
{
    releaseControllerReadLine();

    m_cpuDataBus = data;
    m_cpuInternalDataBus = data;

    if (m_cart) m_cart->observeCpuWrite(addr, data);

    if (addr <= 0x1FFF) {
        m_ram[addr & 0x07FF] = data;
        return;
    }
    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) m_ppu->cpuWrite(addr, data);
        return;
    }
    if (addr == 0x4014) {

        m_dmaPendingPage = data;
        m_dmaRequestPending = true;
        return;
    }
    if (addr == 0x4016) {

        m_strobe = (data & 1) != 0;
        return;
    }

    if ((addr >= 0x4000 && addr <= 0x4013) || addr == 0x4015 || addr == 0x4017) {
        if (addr == 0x4015) traceDmc((data & 0x10) ? "WRITE_4015_ENABLE" : "WRITE_4015_DISABLE");
        if (m_apu) m_apu->cpuWrite(addr, data);
        return;
    }
    if (addr >= 0x4020) {
        if (m_cart) m_cart->cpuWrite(addr, data, m_cpuCycleCounter);
        return;
    }
}

void Bus::saveState(std::vector<uint8_t>& out) const
{
    out.insert(out.end(), m_ram, m_ram + 2048);
    out.push_back(m_controller1);
    out.push_back(m_controller2);
    out.push_back(m_controller1Shift);
    out.push_back(m_controller2Shift);
    out.push_back(m_strobe ? 1 : 0);
    out.push_back(m_controllerReadActivePort);
    out.push_back(m_controllerReadLatched1);
    out.push_back(m_controllerReadLatched2);
    out.push_back(m_cpuDataBus);
    out.push_back(m_cpuInternalDataBus);

    for (int i = 0; i < 8; ++i) out.push_back((m_cpuCycleCounter >> (i * 8)) & 0xFF);
    out.push_back(m_ppuClockAccumulator);
    out.push_back(m_hardResetBootstrapPending ? 1 : 0);
    out.push_back(m_dmaRequestPending ? 1 : 0);
    out.push_back(m_dmaPendingPage);
    out.push_back(m_dmaActive ? 1 : 0);
    out.push_back(m_dmaDummy ? 1 : 0);
    out.push_back(m_dmaReadPhase ? 1 : 0);
    out.push_back(static_cast<uint8_t>(m_dmaPage >> 8));
    out.push_back(m_dmaAddress);
    out.push_back(m_dmaData);
    out.push_back(m_dmaDummyCycles);

    out.push_back(m_dmcDmaActive ? 1 : 0);
    out.push_back(static_cast<uint8_t>(m_dmcDmaPhase));
    out.push_back(static_cast<uint8_t>(m_dmcDmaAddress & 0xFF));
    out.push_back(static_cast<uint8_t>(m_dmcDmaAddress >> 8));
    out.push_back(static_cast<uint8_t>(m_dmcDmaCpuReadAddress & 0xFF));
    out.push_back(static_cast<uint8_t>(m_dmcDmaCpuReadAddress >> 8));
    out.push_back(m_dmcDmaNeedsAlign ? 1 : 0);
    out.push_back(m_dmcDmaAbortAfterHalt ? 1 : 0);
}

bool Bus::loadState(const uint8_t*& p, const uint8_t* end)
{
    if (p + 2048 + 10 + 12 + 7 + 8 > end) return false;
    memcpy(m_ram, p, 2048); p += 2048;
    m_controller1 = *p++;
    m_controller2 = *p++;
    m_controller1Shift = *p++;
    m_controller2Shift = *p++;
    m_strobe = (*p++) != 0;
    m_controllerReadActivePort = *p++;
    m_controllerReadLatched1 = *p++;
    m_controllerReadLatched2 = *p++;
    m_cpuDataBus = *p++;
    m_cpuInternalDataBus = *p++;
    if (m_controllerReadActivePort > 2) return false;

    m_cpuCycleCounter = 0;
    for (int i = 0; i < 8; ++i)
        m_cpuCycleCounter |= (uint64_t(*p++) << (i * 8));
    m_ppuClockAccumulator = *p++;
    if (m_ppuClockAccumulator >= 5) return false;
    m_hardResetBootstrapPending = (*p++) != 0;
    m_dmaRequestPending = (*p++) != 0;
    m_dmaPendingPage = *p++;
    m_dmaActive = (*p++) != 0;
    m_dmaDummy = (*p++) != 0;
    m_dmaReadPhase = (*p++) != 0;
    m_dmaPage = static_cast<uint16_t>(*p++) << 8;
    m_dmaAddress = *p++;
    m_dmaData = *p++;
    m_dmaDummyCycles = *p++;
    m_dmaCpuReadAddress = 0;
    m_dmaCpuReadAddressValid = false;

    m_dmcDmaActive = (*p++) != 0;
    m_dmcDmaPhase = static_cast<DmcDmaPhase>(*p++);
    m_dmcDmaAddress = p[0] | (uint16_t(p[1]) << 8);
    p += 2;
    m_dmcDmaCpuReadAddress = p[0] | (uint16_t(p[1]) << 8);
    p += 2;
    m_dmcDmaNeedsAlign = (*p++) != 0;
    m_dmcDmaAbortAfterHalt = (*p++) != 0;
    if (static_cast<uint8_t>(m_dmcDmaPhase) > static_cast<uint8_t>(DmcDmaPhase::Get)) return false;
    return true;
}
