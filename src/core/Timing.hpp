#pragma once
#include <cstdint>

enum class ConsoleTiming : uint8_t {
    NTSC = 0,
    PAL = 1,
    Dendy = 2,
};

inline constexpr int consoleCpuClockHz(ConsoleTiming timing)
{
    switch (timing) {
    case ConsoleTiming::PAL:   return 1662607;
    case ConsoleTiming::Dendy: return 1773448;
    default:                   return 1789773;
    }
}

inline constexpr int consolePpuClockHz(ConsoleTiming timing)
{
    switch (timing) {
    case ConsoleTiming::PAL:   return 5320342;
    case ConsoleTiming::Dendy: return 5320344;
    default:                   return 5369318;
    }
}

inline constexpr int consoleScanlines(ConsoleTiming timing)
{
    return timing == ConsoleTiming::NTSC ? 262 : 312;
}

inline constexpr int consoleVblankStartScanline(ConsoleTiming timing)
{
    return timing == ConsoleTiming::Dendy ? 291 : 241;
}

inline constexpr bool consoleHasOddFrameSkip(ConsoleTiming timing)
{
    return timing == ConsoleTiming::NTSC;
}

inline constexpr double consoleFrameRate(ConsoleTiming timing)
{
    switch (timing) {
    case ConsoleTiming::PAL:   return 50.00698;
    case ConsoleTiming::Dendy: return 50.00698;
    default:                   return 60.0988;
    }
}

inline constexpr const char* consoleTimingName(ConsoleTiming timing)
{
    switch (timing) {
    case ConsoleTiming::PAL: return "PAL";
    case ConsoleTiming::Dendy: return "Dendy";
    default: return "NTSC";
    }
}
