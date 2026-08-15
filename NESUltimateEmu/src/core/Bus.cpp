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
    if (m_ppu) {
        m_ppu->clock();
        m_ppu->clock();
        m_ppu->clock();
    }
    if (m_apu)
        m_apu->clock();
    if (m_cpu)
        m_cpu->clock();
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
            m_controller1Shift = m_controller1;
            m_controller2Shift = m_controller2;
        }
        uint8_t data = (m_controller1Shift & 0x80) ? 1 : 0;
        m_controller1Shift <<= 1;
        m_controller1Shift |= 0x01;
        return data;
    }
    if (addr == 0x4017) {
        if (m_strobe) {
            m_controller1Shift = m_controller1;
            m_controller2Shift = m_controller2;
        }
        uint8_t data = (m_controller2Shift & 0x80) ? 1 : 0;
        m_controller2Shift <<= 1;
        m_controller2Shift |= 0x01;
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
        if (m_ppu) {
            uint16_t page = (uint16_t)data << 8;
            for (uint16_t i = 0; i < 256; i++)
                m_ppu->oamWrite((uint8_t)i, read(page + i));
        }
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
}

bool Bus::loadState(const uint8_t*& p, const uint8_t* end)
{
    if (p + 2048 + 5 > end) return false;
    memcpy(m_ram, p, 2048); p += 2048;
    m_controller1 = *p++;
    m_controller2 = *p++;
    m_controller1Shift = *p++;
    m_controller2Shift = *p++;
    m_strobe = (*p++) != 0;
    return true;
}







