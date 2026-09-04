#include "Bus.hpp"
#include "Mapper.hpp"
#include <cstdint>
#include <cstdio>
#include <memory>

static uint32_t readSerial(Bus& bus, uint16_t addr, int count) {
    uint32_t bits = 0;
    for (int i = 0; i < count; ++i) {
        const uint8_t v = bus.read(addr);
        bits |= uint32_t(v & 1) << i;
        (void)bus.read(0x0000);
    }
    return bits;
}

int main() {
    bool ok = true;

    Bus bus;
    bus.powerOn();
    bus.setController1(0xA5); bus.setController2(0x3C);
    bus.setController3(0x5A); bus.setController4(0xC3);
    bus.setFourScoreEnabled(true);
    bus.write(0x4016, 1); bus.write(0x4016, 0);
    const uint32_t fs1 = readSerial(bus, 0x4016, 24);
    const uint32_t fs2 = readSerial(bus, 0x4017, 24);
    const bool fourScore = fs1 == 0x085AA5u && fs2 == 0x04C33Cu && (bus.read(0x4016) & 1) == 1;
    std::printf("four_score=%s p1=%06X p2=%06X\n", fourScore ? "PASS" : "FAIL", fs1, fs2);
    ok &= fourScore;

    bus.setFourScoreEnabled(false);
    bus.setZapper(true, 100, 100, true);
    (void)bus.read(0x0000);
    const uint8_t zap = bus.read(0x4017);
    const bool zapper = (zap & 0x18) == 0x18;
    std::printf("zapper_bits=%s value=%02X\n", zapper ? "PASS" : "FAIL", zap);
    ok &= zapper;

    MapperConfig cfg{};
    cfg.id = 487; cfg.nes20 = true; cfg.prgRomSize = 1536u * 1024u; cfg.chrRomSize = 1536u * 1024u;
    auto mapper = createMapper(cfg);
    uint32_t prg = 0, chr = 0;
    bool m487 = mapper && mapper->cpuMapRead(0x8000, prg) && prg == 0 && mapper->mirroring() == Mirror::Vertical;
    m487 &= mapper->cpuWrite(0x4100, 0x0F, 0);
    m487 &= mapper->cpuWrite(0x4180, 0x40, 0);
    m487 &= mapper->cpuMapRead(0x8000, prg) && prg == 0x8000;
    m487 &= mapper->ppuMapRead(0x0000, chr) && chr == 0xE000;
    m487 &= mapper->cpuWrite(0x4180, 0xE2, 0);
    m487 &= mapper->cpuWrite(0x8000, 0x71, 0);
    m487 &= mapper->cpuMapRead(0x8000, prg) && prg == 0x18000;
    m487 &= mapper->ppuMapRead(0x0000, chr) && chr == 0x1E000;
    m487 &= mapper->mirroring() == Mirror::Horizontal;
    std::printf("mapper487=%s prg=%05X chr=%05X\n", m487 ? "PASS" : "FAIL", prg, chr);
    ok &= m487;

    return ok ? 0 : 1;
}
