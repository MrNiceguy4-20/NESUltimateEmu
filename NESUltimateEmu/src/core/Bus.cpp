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

void Bus::clock()
{
    // One Bus clock represents one CPU clock. The PPU runs at 3x the CPU
    // clock and the APU is clocked once here as well. OAM DMA steals the CPU
    // slot but does not stop the PPU/APU.
    if (m_ppu) {
        m_ppu->clock();
        m_ppu->clock();
        m_ppu->clock();
    }
    if (m_apu)
        m_apu->clock();

    if (m_dmaActive)
        clockOamDma();
    else if (m_cpu)
        m_cpu->clock();

    ++m_cpuCycleCounter;
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
        m_ppu->oamWrite(m_dmaAddress, m_dmaData);
        m_dmaAddress++;
        m_dmaReadPhase = true;

        // uint8_t wrap marks completion after byte $FF has been written.
        if (m_dmaAddress == 0)
            m_dmaActive = false;
    }
}

uint8_t Bus::read(uint16_t addr) const
{
    if (addr <= 0x1FFF)
        return m_ram[addr & 0x07FF];

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        if (m_ppu) return m_ppu->cpuRead(addr);
        return 0;
    }

    if (addr == 0x4015) {
        if (m_apu) return m_apu->cpuRead(addr);
        return 0;
    }

    if (addr == 0x4016) {
        if (m_strobe) {
            return m_controller1 & 0x01;
        }
        uint8_t data = m_controller1Shift & 0x01;
        m_controller1Shift = static_cast<uint8_t>((m_controller1Shift >> 1) | 0x80);
        return data;
    }
    if (addr == 0x4017) {
        if (m_strobe) {
            return m_controller2 & 0x01;
        }
        uint8_t data = m_controller2Shift & 0x01;
        m_controller2Shift = static_cast<uint8_t>((m_controller2Shift >> 1) | 0x80);
        return data;
    }

    if (addr >= 0x4020) {
        if (m_cart) return m_cart->cpuRead(addr);
    }
    return 0x00;
}

void Bus::write(uint16_t addr, uint8_t data)
{
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
        m_strobe = (data & 1) != 0;
        if (m_strobe) {
            m_controller1Shift = m_controller1;
            m_controller2Shift = m_controller2;
        }
        return;
    }
    // APU
    if ((addr >= 0x4000 && addr <= 0x4013) || addr == 0x4015 || addr == 0x4017) {
        if (m_apu) m_apu->cpuWrite(addr, data);
        return;
    }
    if (addr >= 0x4020) {
        if (m_cart) m_cart->cpuWrite(addr, data);
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
}

bool Bus::loadState(const uint8_t*& p, const uint8_t* end)
{
    if (p + 2048 + 5 + 8 + 7 > end) return false;
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
    return true;
}














