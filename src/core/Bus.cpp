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
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
}

void Bus::reset()
{
    clearDmaState();

    // The external controller state represents the buttons currently being
    // held, so preserve it. Only reset the serial latch/shift transaction.
    m_controller1Shift = m_controller1;
    m_controller2Shift = m_controller2;
    m_strobe = false;
    m_controllerReadActivePort = 0;
    m_controllerReadLatched1 = 0;
    m_controllerReadLatched2 = 0;

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
    m_controllerReadActivePort = 0;
    m_controllerReadLatched1 = 0;
    m_controllerReadLatched2 = 0;
    m_cpuDataBus = 0;
    m_cpuCycleCounter = 0;
    clearDmaState();

    if (m_ppu) m_ppu->powerOn();
    if (m_apu) m_apu->powerOn();

    // CPU reset must be last: it reads $FFFC/$FFFD through this bus, which
    // must already be connected to the newly loaded cartridge.
    if (m_cpu) m_cpu->powerOn();
}

void Bus::clock()
{
    // One Bus clock represents one CPU clock. The PPU runs at 3x the CPU
    // clock and the APU advances once per CPU clock. DMA can steal the CPU
    // slot without stopping either device.
    // Keep a defined NTSC PPU/CPU synchronization phase. Real consoles can
    // power up in one of three divider alignments, and cpu_interrupts_v2
    // deliberately distinguishes these near VBlank/NMI boundaries. Use two
    // PPU dots before the CPU/APU slot and one after while preserving the
    // exact 3:1 frequency ratio.
    if (m_ppu) {
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

    // A pending DMC DMA first has to acquire RDY on a CPU read cycle when the
    // CPU owns the slot. During OAM DMA the CPU is already halted, so DMC's
    // halt/dummy/alignment cycles may advance in parallel with OAM DMA. Only
    // the DMC sample-memory get actually arbitrates for the shared CPU bus.
    const bool dmcWasActive = m_dmcDmaActive;
    const CPU::BusCycle cpuBusCycle = m_cpu ? m_cpu->nextBusCycle() : CPU::BusCycle{};
    const bool dmcBlockedByCpuWrite =
        !m_dmaActive && dmcWasActive && m_dmcDmaPhase == 0 &&
        m_cpu && cpuBusCycle.type == CPU::BusCycleType::Write;

    // An explicit-stop aborted DMA is only a one-shot halt attempt. Unlike a
    // normal DMA, it does not retry RDY after a CPU write; hardware simply
    // suppresses the aborted DMA in that case.
    if (dmcBlockedByCpuWrite && m_dmcDmaAbortAfterHalt) {
        m_dmcDmaActive = false;
        m_dmcDmaPhase = 0;
        m_dmcDmaAddress = 0;
        m_dmcDmaCpuReadAddress = 0;
        m_dmcDmaNeedsAlign = false;
        m_dmcDmaAbortAfterHalt = false;
        if (m_apu)
            m_apu->abortDmcDma();
    }

    // Record whether OAM actually needs the shared bus on this slot. Its
    // initial/alignment no-op cycles can overlap even with DMC's memory get.
    const bool oamGetCycle = dmaGetCycle();
    const bool oamNeedsBus = m_dmaActive && !m_dmaDummy &&
        (m_dmaReadPhase == oamGetCycle);

    bool dmcUsedBus = false;
    if (dmcWasActive && !dmcBlockedByCpuWrite)
        dmcUsedBus = clockDmcDma();

    // DMC wins only a same-cycle memory-access collision. If OAM is in a
    // no-op slot, both DMA engines advance together. On a real collision,
    // leaving OAM's transfer phase untouched makes it retry after parity
    // realignment on the following slot.
    if (m_dmaActive && (!dmcUsedBus || !oamNeedsBus))
        clockOamDma();
    else if (!m_dmaActive && (!dmcWasActive || dmcBlockedByCpuWrite) && m_cpu)
        m_cpu->clock();

    // CPU register accesses occur while M2 is high; the frame sequencer's
    // low-frequency APU clocks occur afterward. Keeping this phase after the
    // CPU slot is required for $4015 polling and length reload/halt collision
    // behavior measured by the hardware APU tests. DMA does not stop it.
    if (m_apu)
        m_apu->clockFrameCounterPhase();

    // Finish this CPU period with the remaining PPU dot.
    if (m_ppu)
        m_ppu->clock();

    ++m_cpuCycleCounter;
}

bool Bus::requestDmcDma(uint16_t addr, bool abortAfterHalt)
{
    if (m_dmcDmaActive)
        return false;

    m_dmcDmaActive = true;
    m_dmcDmaPhase = 0;
    m_dmcDmaAddress = addr;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = abortAfterHalt;
    return true;
}

bool Bus::cancelDmcDma()
{
    if (!m_dmcDmaActive || m_dmcDmaPhase != 0)
        return false;

    m_dmcDmaActive = false;
    m_dmcDmaPhase = 0;
    m_dmcDmaAddress = 0;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
    return true;
}

bool Bus::clockDmcDma()
{
    if (!m_dmcDmaActive)
        return false;

    // The 2A03 DMA unit alternates PUT/GET bus slots. Sample memory can only
    // be fetched on a GET slot. GET/PUT comes from the APU divider phase, not
    // from an architectural even/odd CPU-cycle rule, so use dmaGetCycle()
    // everywhere. A halt accepted on a PUT slot therefore needs one
    // extra alignment cycle, yielding the hardware 3-or-4-cycle DMC DMA.
    if (m_dmcDmaPhase == 0) {
        m_dmcDmaNeedsAlign = !dmaGetCycle();

        if (!m_dmaActive) {
            if (m_cpu) {
                const CPU::BusCycle cpuCycle = m_cpu->nextBusCycle();
                if (cpuCycle.type == CPU::BusCycleType::Read)
                    m_dmcDmaCpuReadAddress = cpuCycle.address;
            }

            // RDY stretches the CPU read that was in progress. Re-issuing it
            // is observable for PPU/APU/controller registers. During OAM DMA
            // there is no CPU read to repeat; this DMC no-op overlaps OAM.
            (void)repeatDmcStalledCpuRead();
        }
        // The explicit-stop DMC bug performs only the halt cycle. It does
        // not execute the normal dummy/alignment/get portion of the transfer.
        if (m_dmcDmaAbortAfterHalt) {
            m_dmcDmaActive = false;
            m_dmcDmaPhase = 0;
            m_dmcDmaAddress = 0;
            m_dmcDmaCpuReadAddress = 0;
            m_dmcDmaNeedsAlign = false;
            m_dmcDmaAbortAfterHalt = false;
            if (m_apu)
                m_apu->abortDmcDma();
            return false;
        }

        m_dmcDmaPhase = 1;
        return false;
    }

    if (m_dmcDmaPhase == 1) {
        // Mandatory DMC dummy/put cycle. If OAM DMA already owns the CPU, the
        // no-op overlaps its access rather than creating a second bus access.
        if (!m_dmaActive)
            (void)repeatDmcStalledCpuRead();
        m_dmcDmaPhase = 2;
        return false;
    }

    if (m_dmcDmaPhase == 2 && m_dmcDmaNeedsAlign) {
        // Optional alignment/no-op cycle so the DMC sample fetch lands on an
        // GET cycle. This also overlaps OAM DMA when OAM is active.
        if (!m_dmaActive)
            (void)repeatDmcStalledCpuRead();
        m_dmcDmaPhase = 3;
        return false;
    }

    const uint8_t data = readDmcSampleWithCpuConflict();
    m_dmcDmaActive = false;
    m_dmcDmaPhase = 0;
    m_dmcDmaCpuReadAddress = 0;
    m_dmcDmaNeedsAlign = false;
    m_dmcDmaAbortAfterHalt = false;
    if (m_apu)
        m_apu->completeDmcDma(data);
    return true;
}


uint8_t Bus::repeatDmcStalledCpuRead() const
{
    // RDY does not advance the CPU; each halt/dummy/alignment slot repeats
    // the exact read cycle that was in progress when DMC acquired the bus.
    // Route the access through the ordinary CPU read decoder so all mapped
    // side effects remain observable: $2002 clears VBlank/w, $2007 updates
    // the PPU read buffer and increments v, $4015 clears frame IRQ, and
    // $4016/$4017 obey the controller clock/select rules.
    return read(m_dmcDmaCpuReadAddress);
}


uint8_t Bus::readDmcSampleWithCpuConflict() const
{
    // A DMA transfer drives the 2A03's external address bus, but the APU/I/O
    // register decoder is built from two different address sources: A15-A5
    // come from the halted 6502 core while A4-A0 come from the DMA address.
    // If RDY stopped the CPU on a read anywhere in $4000-$401F, a DMC GET can
    // therefore accidentally activate an internal register selected by the
    // sample address's low five bits.
    //
    // Always perform the external cartridge read first. This is still visible
    // on the CPU pins/open-bus latch even when an internal $4015 activation
    // later isolates that value from the 2A03's internal data bus.
    const uint8_t externalData = read(m_dmcDmaAddress);

    const bool cpuInApuIoRegion =
        (m_dmcDmaCpuReadAddress & 0xFFE0u) == 0x4000u;
    if (!cpuInApuIoRegion)
        return externalData;

    const uint16_t activated = static_cast<uint16_t>(
        0x4000u | (m_dmcDmaAddress & 0x001Fu));

    if (activated == 0x4015) {
        // $4015 is internal-only on a retail 2A03. Its status byte replaces
        // the external DMA byte on the internal data path and the read clears
        // the frame-counter IRQ flag. The external sample byte remains on the
        // external/open-bus latch, exactly as with an ordinary $4015 read.
        return m_apu ? m_apu->cpuRead(0x4015) : 0;
    }

    if (activated == 0x4016)
        return readControllerPort(1);
    if (activated == 0x4017)
        return readControllerPort(2);

    // Other retail APU registers in $4000-$401F are not readable. Their
    // accidental activation does not replace the externally fetched sample.
    return externalData;
}

void Bus::startOamDma(uint8_t page)
{
    m_dmaActive = true;
    m_dmaPage = static_cast<uint16_t>(page) << 8;
    m_dmaAddress = 0;
    m_dmaData = 0;
    m_dmaReadPhase = true;

    // DMA begins on the next CPU cycle. There is always one halt cycle,
    // followed by an alignment cycle only when the first data-read slot
    // would otherwise be PUT. Compute this from the same canonical DMA
    // cadence used by DMC rather than raw CPU-counter parity.
    m_dmaDummy = true;
    const bool dataSlotAfterOneHaltIsGet = dmaGetCycle();
    // With this build's odd-counter GET alignment, N+2 has the same parity as
    // the $4014 write slot N. One dummy cycle is enough iff that slot is GET.
    m_dmaDummyCycles = static_cast<uint8_t>(dataSlotAfterOneHaltIsGet ? 1 : 2);
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

    // OAM DMA reads occur on GET slots and writes on PUT slots.
    // A DMC sample fetch can steal an OAM get. Because the OAM transfer phase
    // is left pending, the immediately following odd slot becomes an implicit
    // alignment cycle and the stolen read is retried on the next GET slot.
    const bool getCycle = dmaGetCycle();
    if (m_dmaReadPhase != getCycle)
        return;

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

    // While strobe is high the 4021 is continuously reloaded and reads always
    // expose A without advancing the serial register.
    if (m_strobe) {
        latched = live & 0x01;
        m_controllerReadActivePort = active;
        return driveCpuDataBus(latched, 0x01);
    }

    // On a US NES / AV Famicom, keeping the same controller read-select
    // asserted across consecutive CPU read cycles does not create another CLK
    // edge. Return the bit captured by the first cycle without shifting again.
    if (m_controllerReadActivePort == active)
        return driveCpuDataBus(latched, 0x01);

    latched = shift & 0x01;
    shift = static_cast<uint8_t>((shift >> 1) | 0x80);
    m_controllerReadActivePort = active;
    return driveCpuDataBus(latched, 0x01);
}

uint8_t Bus::read(uint16_t addr) const
{
    if (addr == 0x4016)
        return readControllerPort(1);
    if (addr == 0x4017)
        return readControllerPort(2);

    // Any other bus access deasserts the controller read-select line. A later
    // $4016/$4017 read can then generate a fresh clock edge.
    releaseControllerReadLine();

    if (addr <= 0x1FFF)
        return driveCpuDataBus(m_ram[addr & 0x07FF]);

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) return driveCpuDataBus(m_ppu->cpuRead(addr));
        return m_cpuDataBus;
    }

    if (addr == 0x4015) {
        // $4015 is special on the 2A03: the APU status bits are placed on the
        // CPU's *internal* data bus, not driven onto the external cartridge /
        // memory data bus. D5 remains open bus. This distinction is observable
        // during DMC DMA bus conflicts: repeated $4015 reads may clear the
        // frame IRQ flag, while the DMC sample fetch is still free to update
        // the external data-bus latch. Do not call driveCpuDataBus() here.
        if (m_apu) {
            const uint8_t status = m_apu->cpuRead(addr);
            return static_cast<uint8_t>((m_cpuDataBus & 0x20) | (status & 0xDF));
        }
        return m_cpuDataBus;
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

void Bus::write(uint16_t addr, uint8_t data)
{
    releaseControllerReadLine();

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
    out.push_back(m_controllerReadActivePort);
    out.push_back(m_controllerReadLatched1);
    out.push_back(m_controllerReadLatched2);

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
    out.push_back(static_cast<uint8_t>(m_dmcDmaCpuReadAddress & 0xFF));
    out.push_back(static_cast<uint8_t>(m_dmcDmaCpuReadAddress >> 8));
    out.push_back(m_dmcDmaNeedsAlign ? 1 : 0);
    out.push_back(m_dmcDmaAbortAfterHalt ? 1 : 0);
}

bool Bus::loadState(const uint8_t*& p, const uint8_t* end)
{
    if (p + 2048 + 8 + 8 + 7 + 8 > end) return false;
    memcpy(m_ram, p, 2048); p += 2048;
    m_controller1 = *p++;
    m_controller2 = *p++;
    m_controller1Shift = *p++;
    m_controller2Shift = *p++;
    m_strobe = (*p++) != 0;
    m_controllerReadActivePort = *p++;
    m_controllerReadLatched1 = *p++;
    m_controllerReadLatched2 = *p++;
    if (m_controllerReadActivePort > 2) return false;

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
    m_dmcDmaCpuReadAddress = p[0] | (uint16_t(p[1]) << 8);
    p += 2;
    m_dmcDmaNeedsAlign = (*p++) != 0;
    m_dmcDmaAbortAfterHalt = (*p++) != 0;
    if (m_dmcDmaPhase > 3) return false;
    return true;
}





























