#pragma once
#include <cstddef>
#include <cstdint>
#include "Mapper.hpp"
#include "Timing.hpp"

namespace RomDatabase {

inline uint32_t crc32Update(uint32_t state, const uint8_t* data, std::size_t size)
{
    uint32_t crc = state;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & uint32_t(0u - (crc & 1u)));
    }
    return crc;
}

inline uint32_t crc32(const uint8_t* data, std::size_t size)
{
    return crc32Update(0xFFFFFFFFu, data, size) ^ 0xFFFFFFFFu;
}

inline uint32_t payloadCrc32(const uint8_t* prg, std::size_t prgSize,
                             const uint8_t* chr, std::size_t chrSize)
{
    uint32_t state = 0xFFFFFFFFu;
    if (prg && prgSize) state = crc32Update(state, prg, prgSize);
    if (chr && chrSize) state = crc32Update(state, chr, chrSize);
    return state ^ 0xFFFFFFFFu;
}

struct Resolution {
    bool matched = false;

    bool overrideMapper = false;
    uint16_t mapper = 0;
    bool overrideSubmapper = false;
    uint8_t submapper = 0;
    bool overrideMirror = false;
    Mirror mirror = Mirror::Horizontal;
    bool overrideTiming = false;
    ConsoleTiming timing = ConsoleTiming::NTSC;
    bool overrideMultiRegion = false;
    bool multiRegion = false;
    bool overridePrgRam = false;
    std::size_t prgRamSize = 0;
    bool overridePrgNvRam = false;
    std::size_t prgNvRamSize = 0;
    bool overrideChrRam = false;
    std::size_t chrRamSize = 0;
    bool overrideChrNvRam = false;
    std::size_t chrNvRamSize = 0;

    uint8_t boardVariant = 0;
};

inline Resolution resolveByCrc(uint16_t headerMapper, uint32_t payloadCrc,
                               uint32_t first32kPrgCrc = 0)
{
    Resolution r{};

    if (headerMapper == 53 && first32kPrgCrc == 0x63794E25u) {
        r.matched = true;
        r.boardVariant = 1;
        return r;
    }

    switch (payloadCrc) {
    case 0xBA51AC6Fu:
        r.matched = true;
        r.overrideMapper = true; r.mapper = 78;
        r.overrideSubmapper = true; r.submapper = 3;
        r.overrideMirror = true; r.mirror = Mirror::Horizontal;
        r.overrideTiming = true; r.timing = ConsoleTiming::NTSC;
        r.overridePrgRam = true; r.prgRamSize = 0;
        r.overridePrgNvRam = true; r.prgNvRamSize = 0;
        return r;

    case 0x3D1C3137u:
        r.matched = true;
        r.overrideMapper = true; r.mapper = 78;
        r.overrideSubmapper = true; r.submapper = 1;
        r.overrideMirror = true; r.mirror = Mirror::Horizontal;
        r.overrideTiming = true; r.timing = ConsoleTiming::NTSC;
        r.overridePrgRam = true; r.prgRamSize = 0;
        r.overridePrgNvRam = true; r.prgNvRamSize = 0;
        return r;

    case 0xC247CC80u:
        r.matched = true;
        r.overrideMapper = true; r.mapper = 210;
        r.overrideSubmapper = true; r.submapper = 1;
        r.overrideMirror = true; r.mirror = Mirror::Vertical;
        r.overrideTiming = true; r.timing = ConsoleTiming::NTSC;
        r.overridePrgRam = true; r.prgRamSize = 0;
        r.overridePrgNvRam = true; r.prgNvRamSize = 0x0800;
        return r;

    default:
        return r;
    }
}

inline Resolution resolve(uint16_t mapper, const uint8_t* prg, std::size_t prgSize,
                          const uint8_t* chr, std::size_t chrSize)
{
    const uint32_t payload = payloadCrc32(prg, prgSize, chr, chrSize);
    const uint32_t first32 = (prg && prgSize >= 0x8000) ? crc32(prg, 0x8000) : 0;
    return resolveByCrc(mapper, payload, first32);
}

}
