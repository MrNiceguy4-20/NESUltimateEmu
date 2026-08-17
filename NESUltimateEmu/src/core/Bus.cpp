#include "Bus.hpp"
#include <cstring>
#include "CPU.hpp"
#include "PPU.hpp"
#include "APU.hpp"
#include "Cartridge.hpp"

Bus::Bus() {}

void Bus::connectCPU(CPU* cpu) { m_cpu = cpu; }
void Bus::connectPPU(PPU* ppu) { m_ppu = ppu; }
void Bus::connectAPU(APU* apu) { m_apu = apu; }
void Bus::connectCartridge(Cartridge* cart) { m_cart = cart; }

void Bus::clearDmaState()
{
    m_dmaActive = false;
    m_dmaDummy = false;
    m_dmaReadPhase = true;
    m_dmaPage = 0;
    m_dmaAddress = 0;
    m_dmaData = 0;
    m_dmaDummyCycles = 0;

    m_dmcDmaActive = false;
    m_dmcDmaPhase = 0;
    m_dmcDmaAddress = 0;
}

void Bus::reset()
{
    clearDmaState();

    // The external controller state represents the buttons currently being
    // held, so preserve it. Only reset the serial latch/shift transaction.
    m_controller1Shift = m_controller1;
    m_controller2Shift = m_controller2;
    m_strobe = false;

    if (m_ppu) m_ppu->reset();
    if (m_apu) m_apu->reset();
    if (m_cpu) m_cpu->reset();
}

void Bus::powerOn()
{
    std::memset(m_ram, 0, sizeof(m_ram));
    m_controller1 = 0;
    m_controller2 = 0;
    m_controller1Shift = 0;
    m_controller2Shift = 0;
    m_strobe = false;
    m_cpuDataBus = 0;
    m_cpuCycleCounter = 0;
    clearDmaState();

    if (m_ppu) m_ppu->powerOn();
    if (m_apu) m_apu->reset();

    // CPU reset must be last: it reads $FFFC/$FFFD through this bus, which
    // must already be connected to the newly loaded cartridge.
    if (m_cpu) m_cpu->reset();
}

void Bus::clock()
{
    // One Bus clock represents one CPU clock. The PPU runs at 3x the CPU
    // clock and the APU advances once per CPU clock. DMA can steal the CPU
    // slot without stopping either device.
    if (m_ppu) {
        m_ppu->clock();
        m_ppu->clock();
        m_ppu->clock();
    }
    if (m_apu)
        m_apu->clock();
    if (m_cart)
        m_cart->clockCpu();

    // IRQ is a shared level-sensitive line. Recompute it every CPU clock so
    // clearing one source does not accidentally clear another active source.
    if (m_cpu) {
        const bool apuIrq = m_apu && m_apu->irqActive();
        const bool cartIrq = m_cart && m_cart->irqActive();
        m_cpu->setIrqLine(apuIrq || cartIrq);
    }

    // DMC DMA always stalls the CPU. Its first three setup cycles can overlap
    // OAM DMA in this timing model; only the actual DMC memory read takes
    // precedence over the OAM transfer bus cycle.
    const bool dmcWasActive = m_dmcDmaActive;
    bool dmcUsedBus = false;
    if (dmcWasActive)
        dmcUsedBus = clockDmcDma();

    if (m_dmaActive && !dmcUsedBus)
        clockOamDma();
    else if (!m_dmaActive && !dmcWasActive && m_cpu)
        m_cpu->clock();

    // CPU register accesses occur while M2 is high; the frame sequencer's
    // low-frequency APU clocks occur afterward. Keeping this phase after the
    // CPU slot is required for $4015 polling and length reload/halt collision
    // behavior measured by the hardware APU tests. DMA does not stop it.
    if (m_apu)
        m_apu->clockFrameCounterPhase();

    ++m_cpuCycleCounter;
}

bool Bus::requestDmcDma(uint16_t addr)
{
    if (m_dmcDmaActive)
        return false;

    m_dmcDmaActive = true;
    m_dmcDmaPhase = 0;
    m_dmcDmaAddress = addr;
    return true;
}

bool Bus::clockDmcDma()
{
    if (!m_dmcDmaActive)
        return false;

    // Coarse RDY model: halt, dummy, alignment, then the sample read. This
    // gives the common 4-cycle reload stall while keeping the transfer
    // centralized in the Bus instead of letting the APU read CPU memory.
    if (m_dmcDmaPhase < 3) {
        ++m_dmcDmaPhase;
        return false;
    }

    const uint8_t data = read(m_dmcDmaAddress);
    m_dmcDmaActive = false;
    m_dmcDmaPhase = 0;
    if (m_apu)
        m_apu->completeDmcDma(data);
    return true;
}

void Bus::startOamDma(uint8_t page)
{
    m_dmaActive = true;
    m_dmaPage = static_cast<uint16_t>(page) << 8;
    m_dmaAddress = 0;
    m_dmaData = 0;
    m_dmaReadPhase = true;

    // DMA begins on the next CPU cycle. The CPU performs one alignment
    // cycle when the current CPU cycle is odd, giving the documented
    // 513/514-cycle total transfer time.
    m_dmaDummy = true;
    m_dmaDummyCycles = static_cast<uint8_t>(1 + (m_cpuCycleCounter & 1));
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

    if (m_dmaReadPhase) {
        m_dmaData = read(static_cast<uint16_t>(m_dmaPage | m_dmaAddress));
        m_dmaReadPhase = false;
    }
    else {
        m_ppu->oamDmaWrite(m_dmaData);
        m_dmaAddress++;
        m_dmaReadPhase = true;

        // uint8_t wrap marks completion after byte $FF has been written.
        if (m_dmaAddress == 0)
            m_dmaActive = false;
    }
}

uint8_t Bus::driveCpuDataBus(uint8_t value, uint8_t drivenMask) const
{
    m_cpuDataBus = static_cast<uint8_t>((m_cpuDataBus & ~drivenMask) |
                                        (value & drivenMask));
    return m_cpuDataBus;
}

uint8_t Bus::read(uint16_t addr) const
{
    if (addr <= 0x1FFF)
        return driveCpuDataBus(m_ram[addr & 0x07FF]);

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) return driveCpuDataBus(m_ppu->cpuRead(addr));
        return m_cpuDataBus;
    }

    if (addr == 0x4015) {
        // APU status drives D0-D4 and D6-D7. D5 is physically open bus.
        if (m_apu) return driveCpuDataBus(m_apu->cpuRead(addr), 0xDF);
        return m_cpuDataBus;
    }

    if (addr == 0x4016) {
        uint8_t data = 0;
        if (m_strobe) {
            data = m_controller1 & 0x01;
        }
        else {
            data = m_controller1Shift & 0x01;
            m_controller1Shift = static_cast<uint8_t>((m_controller1Shift >> 1) | 0x80);
        }
        // A standard controller drives only D0.
        return driveCpuDataBus(data, 0x01);
    }
    if (addr == 0x4017) {
        uint8_t data = 0;
        if (m_strobe) {
            data = m_controller2 & 0x01;
        }
        else {
            data = m_controller2Shift & 0x01;
            m_controller2Shift = static_cast<uint8_t>((m_controller2Shift >> 1) | 0x80);
        }
        return driveCpuDataBus(data, 0x01);
    }

    if (addr >= 0x4020 && m_cart) {
        uint8_t data = 0;
        if (m_cart->cpuRead(addr, data))
            return driveCpuDataBus(data);
    }

    // $4000-$4014, $4018-$401F, and any unhandled cartridge expansion
    // address have no readable device driving D0-D7. Keep the previous bus.
    return m_cpuDataBus;
}

uint8_t Bus::debugRead(uint16_t addr) const
{
    if (addr <= 0x1FFF)
        return m_ram[addr & 0x07FF];

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) return m_ppu->debugCpuRead(addr);
        return 0;
    }

    if (addr == 0x4015) {
        if (m_apu) return m_apu->debugStatus();
        return 0;
    }

    // Debug reads must not advance the serial controller shift registers.
    if (addr == 0x4016) {
        const uint8_t bit = m_strobe ? (m_controller1 & 1) : (m_controller1Shift & 1);
        return static_cast<uint8_t>((m_cpuDataBus & 0xFE) | bit);
    }
    if (addr == 0x4017) {
        const uint8_t bit = m_strobe ? (m_controller2 & 1) : (m_controller2Shift & 1);
        return static_cast<uint8_t>((m_cpuDataBus & 0xFE) | bit);
    }

    if (addr >= 0x4020 && m_cart)
        return m_cart->debugCpuRead(addr);

    return m_cpuDataBus;
}

void Bus::write(uint16_t addr, uint8_t data)
{
    // The CPU drives D0-D7 during every write, including writes to unmapped
    // space; a following open-bus read therefore observes this value.
    m_cpuDataBus = data;

    if (addr <= 0x1FFF) {
        m_ram[addr & 0x07FF] = data;
        return;
    }
    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) m_ppu->cpuWrite(addr, data);
        return;
    }
    if (addr == 0x4014) {
        startOamDma(data);
        return;
    }
    if (addr == 0x4016) {
        const bool newStrobe = (data & 1) != 0;

        // While strobe is high the controllers continuously present the A
        // button. When it falls, latch the complete current button state for
        // the subsequent serial reads.
        if (newStrobe || (m_strobe && !newStrobe)) {
            m_controller1Shift = m_controller1;
            m_controller2Shift = m_controller2;
        }
        m_strobe = newStrobe;
        return;
    }
    // APU
    if ((addr >= 0x4000 && addr <= 0x4013) || addr == 0x4015 || addr == 0x4017) {
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

    // DMA state is serialized so save states taken during OAM DMA resume
    // deterministically.
    for (int i = 0; i < 8; ++i) out.push_back((m_cpuCycleCounter >> (i * 8)) & 0xFF);
    out.push_back(m_dmaActive ? 1 : 0);
    out.push_back(m_dmaDummy ? 1 : 0);
    out.push_back(m_dmaReadPhase ? 1 : 0);
    out.push_back(static_cast<uint8_t>(m_dmaPage >> 8));
    out.push_back(m_dmaAddress);
    out.push_back(m_dmaData);
    out.push_back(m_dmaDummyCycles);

    out.push_back(m_dmcDmaActive ? 1 : 0);
    out.push_back(m_dmcDmaPhase);
    out.push_back(static_cast<uint8_t>(m_dmcDmaAddress & 0xFF));
    out.push_back(static_cast<uint8_t>(m_dmcDmaAddress >> 8));
}

bool Bus::loadState(const uint8_t*& p, const uint8_t* end)
{
    if (p + 2048 + 5 + 8 + 7 + 4 > end) return false;
    memcpy(m_ram, p, 2048); p += 2048;
    m_controller1 = *p++;
    m_controller2 = *p++;
    m_controller1Shift = *p++;
    m_controller2Shift = *p++;
    m_strobe = (*p++) != 0;

    m_cpuCycleCounter = 0;
    for (int i = 0; i < 8; ++i)
        m_cpuCycleCounter |= (uint64_t(*p++) << (i * 8));
    m_dmaActive = (*p++) != 0;
    m_dmaDummy = (*p++) != 0;
    m_dmaReadPhase = (*p++) != 0;
    m_dmaPage = static_cast<uint16_t>(*p++) << 8;
    m_dmaAddress = *p++;
    m_dmaData = *p++;
    m_dmaDummyCycles = *p++;

    m_dmcDmaActive = (*p++) != 0;
    m_dmcDmaPhase = *p++;
    m_dmcDmaAddress = p[0] | (uint16_t(p[1]) << 8);
    p += 2;
    if (m_dmcDmaPhase > 3) return false;
    return true;
}





























