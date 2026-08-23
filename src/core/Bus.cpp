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

    // The external controller state represents the buttons currently being
    // held, so preserve it. Only reset the serial latch/shift transaction.
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

    // Give reset-sensitive cartridge hardware its cold-start edge before the
    // CPU fetches the reset vector from the newly selected PRG bank.
    if (m_cart) m_cart->resetMapper(true);
    if (m_ppu) m_ppu->powerOn();
    if (m_apu) m_apu->powerOn();

    // CPU reset must be last: it reads $FFFC/$FFFD through this bus, which
    // must already be connected to the newly loaded cartridge.
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
        // Mapper-6/8/12.1 extracts specify an implicit JSR $7003 before the
        // game's original reset handler. Establish the exact architectural
        // stack result of JSR so the trainer's RTS lands on the original
        // vector without inventing cartridge-visible bootstrap ROM.
        const uint16_t ret = static_cast<uint16_t>(pc - 1);
        write(static_cast<uint16_t>(0x0100 | sp), static_cast<uint8_t>(ret >> 8)); --sp;
        write(static_cast<uint16_t>(0x0100 | sp), static_cast<uint8_t>(ret)); --sp;
    }
    pc = entry;
}

void Bus::clock()
{
    // One Bus clock represents one CPU clock. NTSC and Dendy run the PPU at
    // exactly 3 dots per CPU clock; licensed PAL uses the 16:5 (3.2:1)
    // divider. A small integer accumulator preserves that fractional ratio
    // without floating-point drift: PAL emits 3,3,3,3,4 PPU dots repeatedly.
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
        // A delayed $4017 reset/5-step immediate clock is sampled before the
        // CPU bus access on the cycle where its 3/4-clock delay expires.
        // Regular frame-sequencer events are still clocked after the CPU.
        m_apu->clockFrameCounterPreCpuPhase();
    }
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
    if (m_dmcDmaActive) traceDmc("CLOCK_BEGIN");
    const CPU::BusCycle cpuBusCycle = m_cpu ? m_cpu->nextBusCycle() : CPU::BusCycle{};

    // A write to $4014 only requests OAM DMA. RDY can halt the CPU on read
    // cycles, not on writes. This is essential for RMW instructions such as
    // INC $4014: both writes must reach the register, the second write replaces
    // the requested page, and only then does one DMA begin on the following
    // haltable read cycle.
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

    // An explicit-stop aborted DMA is only a one-shot halt attempt. Unlike a
    // normal DMA, it does not retry RDY after a CPU write; hardware simply
    // suppresses the aborted DMA in that case.
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

    // The controller 4021 latch sees the $4016 strobe on the APU PUT phase,
    // not combinationally on every CPU write. A one-cycle high pulse that
    // exists only across GET->PUT ordering can therefore be missed, while a
    // pulse sampled on PUT reloads both controller shift registers. Sample
    // after the CPU slot so a $4016 write performed on this PUT is visible.
    // AccuracyCoin's DEC $4016 phase test identifies the sampled controller
    // PUT half-cycle relative to the DMA cadence. With Bus::clock() sampling
    // after the CPU write, this corresponds to dmaGetCycle()==true in the
    // current scheduler alignment (the previous Phase 69 polarity was
    // exactly reversed, turning failure 4 into failure 3).
    if (dmaGetCycle() && m_strobe) {
        m_controller1Shift = m_controller1;
        m_controller2Shift = m_controller2;
    }

    // CPU register accesses occur while M2 is high; the frame sequencer's
    // low-frequency APU clocks occur afterward. Keeping this phase after the
    // CPU slot is required for $4015 polling and length reload/halt collision
    // behavior measured by the hardware APU tests. DMA does not stop it.
    if (m_apu)
        m_apu->clockFrameCounterPhase();

    // Finish this CPU period with the remaining PPU dots. The two-before
    // placement retains the established NTSC synchronization phase while the
    // PAL accumulator occasionally contributes a second trailing dot.
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

    // If RDY has not yet been acquired, an aborted reload still attempts the
    // HALT once.  If the current CPU cycle is a write, the existing Bus::clock
    // write-block path suppresses that one-cycle DMA entirely.
    if (m_dmcDmaPhase == DmcDmaPhase::Halt) {
        m_dmcDmaAbortAfterHalt = true;
        traceDmc("STOP_ARM_ABORT1");
        return;
    }

    // Halt/dummy/alignment cycles already spent remain observable, but no
    // sample GET occurs after the delayed $4015 disable reaches the reader.
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

    // The 2A03 DMA unit alternates PUT/GET bus slots. Sample memory can only
    // be fetched on a GET slot. GET/PUT comes from the APU divider phase, not
    // from an architectural even/odd CPU-cycle rule, so use dmaGetCycle()
    // everywhere. A halt accepted on a PUT slot therefore needs one
    // extra alignment cycle, yielding the hardware 3-or-4-cycle DMC DMA.
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

            // RDY stretches the CPU read that was in progress. Re-issuing it
            // is observable for PPU/APU/controller registers. During OAM DMA
            // there is no CPU read to repeat; this DMC no-op overlaps OAM.
            (void)repeatDmcStalledCpuRead();
        }
        // The explicit-stop DMC bug performs only the halt cycle. It does
        // not execute the normal dummy/alignment/get portion of the transfer.
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
        // Mandatory DMC dummy/put cycle. If OAM DMA already owns the CPU, the
        // no-op overlaps its access rather than creating a second bus access.
        if (!m_dmaActive)
            (void)repeatDmcStalledCpuRead();
        m_dmcDmaPhase = DmcDmaPhase::Align;
        traceDmc("ENTER_ALIGN");
        return false;
    }

    if (m_dmcDmaPhase == DmcDmaPhase::Align && m_dmcDmaNeedsAlign) {
        traceDmc("ALIGN");
        // Optional alignment/no-op cycle so the DMC sample fetch lands on an
        // GET cycle. This also overlaps OAM DMA when OAM is active.
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
    // RDY does not advance the CPU; each halt/dummy/alignment slot repeats
    // the exact read cycle that was in progress when DMC acquired the bus.
    // Route the access through the ordinary CPU read decoder so all mapped
    // side effects remain observable: $2002 clears VBlank/w, $2007 updates
    // the PPU read buffer and increments v, $4015 clears frame IRQ, and
    // $4016/$4017 obey the controller clock/select rules.
    return read(m_dmcDmaCpuReadAddress);
}


uint8_t Bus::readDmcExternalSample() const
{
    // DMC sample fetches are external cartridge-bus reads. They must update
    // the external/open-bus latch without deasserting the 2A03's internal
    // $4016/$4017 read-select line; that distinction is observable when the
    // DMA GET itself conflicts with a controller read. DMC addresses are
    // always in $8000-$FFFF, so no internal CPU/APU device is selected here.
    uint8_t data = m_cpuDataBus;
    if (m_cart && m_cart->cpuRead(m_dmcDmaAddress, data))
        return driveExternalCpuDataBus(data);
    return m_cpuDataBus;
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
    const uint8_t externalData = readDmcExternalSample();

    const bool cpuInApuIoRegion =
        (m_dmcDmaCpuReadAddress & 0xFFE0u) == 0x4000u;
    if (!cpuInApuIoRegion) {
        // The sample-memory GET is a different external bus access from the
        // stalled CPU read. It breaks a run of consecutive $4016/$4017 reads.
        releaseControllerReadLine();
        return externalData;
    }

    const uint16_t activated = static_cast<uint16_t>(
        0x4000u | (m_dmcDmaAddress & 0x001Fu));

    // Only an actual internal controller-port activation keeps the 4021 read
    // select asserted. Ordinary DMC sample GETs (including conflicts with
    // open-bus APU registers) release it, so the resumed LDA $4016 creates a
    // fresh clock edge. This distinguishes AccuracyCoin controller tests 6/7.
    if (activated != 0x4016 && activated != 0x4017)
        releaseControllerReadLine();

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

uint8_t Bus::readOamDmaSource(uint16_t addr) const
{
    // OAM DMA drives the external address bus, while the internal APU/I/O
    // decoder keeps A15-A5 from the 6502 read cycle on which RDY was acquired.
    // This permits $4015/$4016/$4017 (and their $20-byte mirrors) only when the
    // frozen CPU address itself is in $4000-$401F.
    uint8_t external = m_cpuDataBus;
    bool externalDriven = false;

    // Read the OAM source from the external side first. $4000-$401F is open
    // bus unless an internal register is selected by the split address decode.
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

    // A4-A0 come from the OAM DMA address, so the readable APU/I/O registers
    // repeat every $20 bytes for as long as the 6502 high-address gate stays
    // asserted. This is normally unreachable to CPU software but observable
    // during OAM DMA.
    const uint16_t activated = static_cast<uint16_t>(0x4000u | (addr & 0x001Fu));
    if (activated == 0x4015) {
        const uint8_t status = m_apu ? m_apu->cpuRead(0x4015) : 0;
        // $4015 is an internal 2A03 source. For the byte being transferred,
        // its driven status bits replace open bus while D5 remains supplied by
        // the OAM/external side.
        const uint8_t result = static_cast<uint8_t>((external & 0x20) |
                                                    (status & 0xDF));

        if (!externalDriven) {
            // The OAM-activation test exposes an asymmetric bus-keeper detail:
            // an internal APU read can discharge bits that were already high
            // on the external/open-bus latch, but a high internal status bit
            // does not charge a previously-low external bit. This is why the
            // first mirrored $4015 read returns $44 yet following open-bus
            // bytes remain $40, while the later $04 status read clears D6 and
            // all subsequent open-bus bytes become $00.
            const uint8_t driven = 0xDF;
            m_cpuDataBus = static_cast<uint8_t>((external & ~driven) |
                                                (external & status & driven));
        }
        return driveInternalCpuDataBus(result);
    }

    if (activated == 0x4016 || activated == 0x4017) {
        const uint8_t port = activated == 0x4016 ? 1 : 2;
        if (externalDriven) {
            // The controller read-select can still clock as a side effect, but
            // an externally-driven OAM source dominates the returned DMA byte.
            // This is the bus-conflict behavior used by AccuracyCoin test 7.
            (void)readControllerPort(port);
            return driveInternalCpuDataBus(external);
        }

        // Controller ports drive only D0. Preserve the open-bus value for the
        // current OAM byte, but apply the same one-way discharge behavior to
        // the persistent external/open-bus latch. Calling readControllerPort()
        // performs the real 4021 edge/shift side effect; restore the external
        // latch afterward instead of letting its helper charge D0 high.
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

    // DMA begins on the next CPU cycle. There is always one halt cycle,
    // followed by an alignment cycle only when the first data-read slot
    // would otherwise be PUT. Compute this from the same canonical DMA
    // cadence used by DMC rather than raw CPU-counter parity.
    m_dmaDummy = true;
    // startOamDma() is entered on the halt cycle itself. If that cycle is
    // PUT, the immediately following cycle is GET and the first source read
    // can proceed. If the halt is GET, the following PUT must be consumed as
    // an alignment cycle before the first source read.
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

    // OAM DMA reads occur on GET slots and writes on PUT slots.
    // A DMC sample fetch can steal an OAM get. Because the OAM transfer phase
    // is left pending, the immediately following odd slot becomes an implicit
    // alignment cycle and the stolen read is retried on the next GET slot.
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

        // uint8_t wrap marks completion after byte $FF has been written.
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
    // A normal CPU read connects the external bus to the core's internal data
    // path, so both latches see the same driven bits. DMC sample fetches call
    // driveExternalCpuDataBus() directly and therefore cannot contaminate the
    // internal latch used by $4015.
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
            const uint8_t internal = static_cast<uint8_t>((m_cpuInternalDataBus & 0x20) |
                                                           (status & 0xDF));
            // $4015 does not drive the external CPU data pins. It does update
            // the internal 2A03 latch, with D5 remaining whatever was already
            // on that internal path.
            return driveInternalCpuDataBus(internal);
        }
        return m_cpuInternalDataBus;
    }

    if (addr >= 0x4020 && m_cart) {
        // Seed cartridge register reads with the current CPU data-bus value.
        // Some mapper registers drive only a subset of D0-D7 and deliberately
        // preserve the remaining open-bus bits.
        uint8_t data = m_cpuDataBus;
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
    m_cpuInternalDataBus = data;

    // Some cartridge ASICs (notably J.Y. Company) can clock an IRQ from
    // every CPU write, including writes outside cartridge address space.
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
        // Defer acquisition of RDY until a CPU read cycle. A second write from
        // an RMW instruction updates the page before the single DMA starts.
        m_dmaPendingPage = data;
        m_dmaRequestPending = true;
        return;
    }
    if (addr == 0x4016) {
        // The CPU-visible strobe level changes immediately, but the external
        // controller's parallel-load action is sampled on the APU PUT phase
        // in Bus::clock(). This reproduces the one-cycle RMW strobe asymmetry
        // measured by AccuracyCoin instead of latching on every edge here.
        m_strobe = (data & 1) != 0;
        return;
    }
    // APU
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

    // DMA state is serialized so save states taken during OAM DMA resume
    // deterministically.
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





























