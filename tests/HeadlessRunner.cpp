#include "../src/core/CPU.hpp"
#include "../src/core/Bus.hpp"
#include "../src/core/PPU.hpp"
#include "../src/core/APU.hpp"
#include "../src/core/Cartridge.hpp"
#include "probes/ProbeSuite.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr uint64_t kDefaultMaxCycles = 150000000ULL;
constexpr uint64_t kResetDelayCycles = 180000ULL; // >= 100 ms at NTSC CPU rate
constexpr uint64_t kPollInterval = 256ULL;
constexpr uint16_t kStatus = 0x6000;
constexpr uint16_t kSignature = 0x6001;
constexpr uint16_t kMessage = 0x6004;
constexpr std::size_t kMaxMessage = 4096;

struct Options {
    std::string romPath;
    uint64_t maxCycles = kDefaultMaxCycles;
    bool quiet = false;
    APU::DmcCpuRevision dmcRevision = APU::DmcCpuRevision::Mid1990OrLater;
    bool forceTiming = false;
    ConsoleTiming timing = ConsoleTiming::NTSC;
    bool accuracyCoin = false;
    bool legacyApu = false;
    bool legacyShell = false;
    bool scanlineVisual = false;
    uint16_t legacyResultAddress = 0x00F0;
    uint8_t expectedResult = 1;
    bool expectedResultSet = false;
    bool json = false;
    int accuracyCoinMinPass = -1;
};

void usage(const char* exe)
{
    std::cerr
        << "NES Ultimate Emulator - headless regression runner\n\n"
        << "Usage:\n  " << exe << " <test.nes> [--max-cycles N] [--timing auto|ntsc|pal|dendy] [--dmc-revision early|late] [--quiet] [--json]\n"
        << "  " << exe << " <AccuracyCoin.nes> --accuracycoin [--max-cycles N] [--min-pass N] [--json]\n"
        << "  " << exe << " <legacy-rom.nes> --legacy-apu [--legacy-result-address N] [--expected-result N] [--max-cycles N] [--json]\n"
        << "  " << exe << " <legacy-shell.nes> --legacy-shell [--expected-result N] [--max-cycles N] [--json]\n"
        << "  " << exe << " <scanline.nes> --scanline-visual [--max-cycles N] [--json]\n"
        << "  " << exe << " --self-test\n\n"
        << "Exit codes:\n"
        << "  0  test passed\n"
        << "  1  test reported failure\n"
        << "  2  ROM could not be loaded\n"
        << "  3  timed out after detecting the test protocol\n"
        << "  4  no supported test protocol was detected\n"
        << "  5  AccuracyCoin automation did not complete\n"
        << "  6  legacy APU automation did not reach its terminal loop\n";
}

bool parseU64(const char* text, uint64_t& out)
{
    if (!text || !*text) return false;
    char* end = nullptr;
    const unsigned long long value = std::strtoull(text, &end, 10);
    if (!end || *end != '\0') return false;
    out = static_cast<uint64_t>(value);
    return true;
}

bool parseArgs(int argc, char** argv, Options& out)
{
    if (argc < 2) return false;
    out.romPath = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--quiet") {
            out.quiet = true;
        } else if (arg == "--accuracycoin") {
            out.accuracyCoin = true;
        } else if (arg == "--legacy-apu") {
            out.legacyApu = true;
        } else if (arg == "--legacy-shell") {
            out.legacyShell = true;
        } else if (arg == "--scanline-visual") {
            out.scanlineVisual = true;
        } else if (arg == "--json") {
            out.json = true;
        } else if (arg == "--legacy-result-address" && i + 1 < argc) {
            const char* text = argv[++i];
            char* end = nullptr;
            const unsigned long value = std::strtoul(text, &end, 0);
            if (!end || *end != '\0' || value > 0x07FF)
                return false;
            out.legacyResultAddress = static_cast<uint16_t>(value);
        } else if (arg == "--expected-result" && i + 1 < argc) {
            uint64_t value = 0;
            if (!parseU64(argv[++i], value) || value > 255)
                return false;
            out.expectedResult = static_cast<uint8_t>(value);
            out.expectedResultSet = true;
        } else if (arg == "--min-pass" && i + 1 < argc) {
            uint64_t value = 0;
            if (!parseU64(argv[++i], value) || value > 255)
                return false;
            out.accuracyCoinMinPass = static_cast<int>(value);
        } else if (arg == "--max-cycles" && i + 1 < argc) {
            if (!parseU64(argv[++i], out.maxCycles) || out.maxCycles == 0)
                return false;
        } else if (arg == "--timing" && i + 1 < argc) {
            const std::string timing = argv[++i];
            if (timing == "auto") {
                out.forceTiming = false;
            } else if (timing == "ntsc") {
                out.forceTiming = true; out.timing = ConsoleTiming::NTSC;
            } else if (timing == "pal") {
                out.forceTiming = true; out.timing = ConsoleTiming::PAL;
            } else if (timing == "dendy") {
                out.forceTiming = true; out.timing = ConsoleTiming::Dendy;
            } else {
                return false;
            }
        } else if (arg == "--dmc-revision" && i + 1 < argc) {
            const std::string revision = argv[++i];
            if (revision == "early")
                out.dmcRevision = APU::DmcCpuRevision::PreMid1990;
            else if (revision == "late")
                out.dmcRevision = APU::DmcCpuRevision::Mid1990OrLater;
            else
                return false;
        } else {
            return false;
        }
    }
    return true;
}

bool hasBlarggSignature(const Bus& bus)
{
    return bus.read(kSignature + 0) == 0xDE &&
           bus.read(kSignature + 1) == 0xB0 &&
           bus.read(kSignature + 2) == 0x61;
}

std::string readBlarggMessage(const Bus& bus)
{
    std::string text;
    text.reserve(256);
    for (std::size_t i = 0; i < kMaxMessage; ++i) {
        const uint8_t c = bus.read(static_cast<uint16_t>(kMessage + i));
        if (c == 0) break;
        if (c == '\r') continue;
        if (c == '\n' || c == '\t' || (c >= 32 && c <= 126))
            text.push_back(static_cast<char>(c));
        else
            text.push_back('.');
    }
    return text;
}

struct Machine {
    std::unique_ptr<Bus> bus = std::make_unique<Bus>();
    std::unique_ptr<CPU> cpu = std::make_unique<CPU>(*bus);
    std::unique_ptr<PPU> ppu = std::make_unique<PPU>();
    std::unique_ptr<APU> apu = std::make_unique<APU>();
    std::unique_ptr<Cartridge> cart = std::make_unique<Cartridge>();

    Machine()
    {
        bus->connectCPU(cpu.get());
        bus->connectPPU(ppu.get());
        bus->connectAPU(apu.get());
        bus->connectCartridge(cart.get());
        ppu->connectCartridge(cart.get());
        ppu->connectCPU(cpu.get());
        cart->connectCPU(cpu.get());
        apu->connectBus(bus.get());
        apu->connectCPU(cpu.get());
        apu->connectCartridge(cart.get());
    }
};


struct AccuracyCoinFailure {
    uint16_t address = 0;
    uint8_t raw = 0;
    uint8_t errorCode = 0;
};

int runScanlineVisual(Machine& m, const Options& options)
{
    // Quietust's scanline test starts with six '*' tiles in columns 25-30 of
    // each test row. Correct mid-scanline register timing erases those stars;
    // any surviving lit pixel in those cells denotes an error. Let the ROM
    // run for many frames, then inspect only the documented error cells.
    const uint64_t targetCycles = std::min<uint64_t>(options.maxCycles, 5000000ULL);
    while (m.bus->cpuCycleCounter() < targetCycles)
        m.bus->clock();

    const uint32_t* fb = m.ppu->framebuffer();
    const uint32_t background = fb[0] & 0x00FFFFFFu;
    struct Range { int y0, y1; };
    constexpr Range ranges[] = {{6 * 8, 12 * 8}, {15 * 8, 21 * 8}, {24 * 8, 28 * 8}};
    unsigned badPixels = 0;
    for (const auto& r : ranges) {
        for (int y = r.y0; y < r.y1; ++y) {
            for (int x = 25 * 8; x < 31 * 8; ++x) {
                if ((fb[y * 256 + x] & 0x00FFFFFFu) != background)
                    ++badPixels;
            }
        }
    }

    if (options.json) {
        std::cout << "{\"protocol\":\"scanline-visual\",\"error_pixels\":"
                  << badPixels << ",\"cycles\":" << m.bus->cpuCycleCounter() << "}\n";
    } else if (!options.quiet || badPixels != 0) {
        std::cout << (badPixels == 0 ? "PASS: " : "FAIL: ") << m.cart->fileName()
                  << " (scanline visual error pixels " << badPixels << ", "
                  << m.bus->cpuCycleCounter() << " CPU cycles)\n";
    }
    return badPixels == 0 ? 0 : 1;
}

int runLegacyApu(Machine& m, const Options& options)
{
    // blargg_apu_2005.07.30 predates the later $6000 protocol. The suite
    // stores its current result code in zero-page $F0; success is 1. Each ROM
    // calls report_final_result and then enters a permanent one-instruction
    // loop. Detect that terminal state from repeated instruction-boundary
    // opcode fetches before observing $F0, so a long intermediate test cannot
    // be mistaken for completion.
    const uint16_t kResult = options.legacyResultAddress;
    // Some legacy ROMs intentionally spin in a one-instruction loop while
    // waiting for the next NMI. Require a repeat run longer than one NTSC
    // frame so those wait loops cannot be mistaken for the final report loop.
    constexpr uint32_t kTerminalBoundaryRepeats = 50000;

    uint16_t lastFetch = 0xFFFF;
    uint32_t sameFetchCount = 0;

    while (m.bus->cpuCycleCounter() < options.maxCycles) {
        m.bus->clock();
        if (!m.cpu->atInstructionBoundary())
            continue;

        const CPU::BusCycle next = m.cpu->nextBusCycle();
        if (!next.exact || next.type != CPU::BusCycleType::Read) {
            sameFetchCount = 0;
            lastFetch = 0xFFFF;
            continue;
        }

        if (next.address == lastFetch) {
            ++sameFetchCount;
        } else {
            lastFetch = next.address;
            sameFetchCount = 1;
        }

        if (sameFetchCount < kTerminalBoundaryRepeats)
            continue;

        const uint8_t result = m.bus->testPeekCpuRam(kResult);
        const uint64_t now = m.bus->cpuCycleCounter();
        if (options.json) {
            std::cout << "{\"protocol\":\"blargg-apu-2005\",\"result\":"
                      << unsigned(result) << ",\"cycles\":" << now << "}\n";
        }
        const uint8_t expected = options.expectedResultSet ? options.expectedResult : 1;
        if (result == expected) {
            if (!options.quiet && !options.json)
                std::cout << "PASS: " << m.cart->fileName() << " (legacy result "
                          << unsigned(result) << ", " << now << " CPU cycles)\n";
            return 0;
        }
        if (!options.json)
            std::cout << "FAIL: " << m.cart->fileName() << " returned legacy code "
                      << unsigned(result) << " (expected "
                      << unsigned(expected) << ", " << now << " CPU cycles)\n";
        return 1;
    }

    std::cerr << "LEGACY APU TIMEOUT: terminal report loop was not detected within "
              << options.maxCycles << " CPU cycles.\n";
    return 6;
}


int runLegacyShell(Machine& m, const Options& options)
{
    // Older blargg shells predate the $6000 protocol. Their common exit
    // routine begins with the distinctive sequence:
    //   LDX #$FF / TXS / SEI / PHA / JSR nmi_off / PLA / JMP exit_
    // A contains the final result code on entry (0 = pass). Locate that
    // routine in PRG and observe A at the instruction boundary before LDX
    // executes, so later printing/beeping/idle code cannot destroy it.
    std::ifstream f(options.romPath, std::ios::binary);
    std::vector<uint8_t> image((std::istreambuf_iterator<char>(f)), {});
    if (image.size() < 16 || image[0] != 'N' || image[1] != 'E' || image[2] != 'S' || image[3] != 0x1A) {
        std::cerr << "LEGACY SHELL ERROR: expected iNES ROM.\n";
        return 4;
    }
    const std::size_t trainer = (image[6] & 0x04) ? 512 : 0;
    const std::size_t prgSize = std::size_t(image[4]) * 16384;
    const std::size_t prgOff = 16 + trainer;
    if (prgSize == 0 || image.size() < prgOff + prgSize) {
        std::cerr << "LEGACY SHELL ERROR: invalid PRG image.\n";
        return 4;
    }

    uint16_t exitPc = 0;
    int matches = 0;
    for (std::size_t i = 0; i + 12 <= prgSize; ++i) {
        const uint8_t* q = image.data() + prgOff + i;
        if (q[0] != 0xA2 || q[1] != 0xFF || q[2] != 0x9A || q[3] != 0x78 ||
            q[4] != 0x48 || q[5] != 0x20 || q[8] != 0x68 || q[9] != 0x4C)
            continue;
        // NROM-128 is mirrored at $8000/$C000; the shell links at $E000, so
        // use the upper mapping for 16 KiB PRG. NROM-256 maps linearly at $8000.
        const uint16_t mapped = static_cast<uint16_t>((prgSize == 16384 ? 0xC000 : 0x8000) + i);
        exitPc = mapped;
        ++matches;
    }
    if (matches != 1) {
        std::cerr << "LEGACY SHELL ERROR: exit routine was not uniquely identified (matches="
                  << matches << ").\n";
        return 4;
    }

    constexpr uint64_t kStartupGuardCycles = 10000;
    while (m.bus->cpuCycleCounter() < options.maxCycles) {
        if (m.bus->cpuCycleCounter() >= kStartupGuardCycles && m.cpu->atInstructionBoundary()) {
            const CPU::BusCycle next = m.cpu->nextBusCycle();
            if (next.exact && next.type == CPU::BusCycleType::Read && next.address == exitPc) {
                const uint8_t result = m.cpu->testAccumulator();
                const uint8_t expected = options.expectedResultSet ? options.expectedResult : 0;
                const uint64_t now = m.bus->cpuCycleCounter();
                if (options.json)
                    std::cout << "{\"protocol\":\"legacy-shell\",\"result\":" << unsigned(result)
                              << ",\"cycles\":" << now << "}\n";
                else if (!options.quiet)
                    std::cout << (result == expected ? "PASS: " : "FAIL: ") << m.cart->fileName()
                              << " (legacy shell result " << unsigned(result) << ", " << now << " CPU cycles)\n";
                return result == expected ? 0 : 1;
            }
        }
        m.bus->clock();
    }
    std::cerr << "LEGACY SHELL TIMEOUT: exit routine was not reached within "
              << options.maxCycles << " CPU cycles; PC=$" << std::hex << std::setw(4)
              << std::setfill('0') << m.cpu->testProgramCounter() << std::dec << ".\n";
    return 6;
}

int runAccuracyCoin(Machine& m, const Options& options)
{
    // AccuracyCoin's own engine uses these RAM locations:
    // $35 RunningAllTests, $37 total tally, $38 pass tally, and page $04 for
    // encoded results.  Passing results have bit 0 set; failures are encoded
    // as (errorCode << 2) | 2.  Reads here must be observational only.
    constexpr uint16_t kMenuTabX = 0x0014;
    constexpr uint16_t kMenuCursorY = 0x0016;
    constexpr uint16_t kRunningAll = 0x0035;
    constexpr uint16_t kTotalTally = 0x0037;
    constexpr uint16_t kPassTally = 0x0038;
    constexpr uint64_t kStartupGuardCycles = 250000;
    constexpr uint64_t kButtonHoldCycles = 60000;
    constexpr uint8_t kStartButton = 0x08; // Bus representation: bit3 = Start.

    bool startPressed = false;
    bool startReleased = false;
    bool runSeen = false;
    uint8_t observedTestCount = 0;
    uint64_t startPressAt = 0;

    while (m.bus->cpuCycleCounter() < options.maxCycles) {
        m.bus->clock();
        const uint64_t now = m.bus->cpuCycleCounter();

        if (!startPressed && now >= kStartupGuardCycles &&
            m.bus->testPeekCpuRam(kMenuCursorY) == 0xFF) {
            m.bus->setController1(kStartButton);
            startPressed = true;
            startPressAt = now;
            if (!options.quiet && !options.json)
                std::cout << "[AccuracyCoin] main menu detected; pressing Start\n";
        }

        if (startPressed && !startReleased && now - startPressAt >= kButtonHoldCycles) {
            m.bus->setController1(0);
            startReleased = true;
        }

        const uint8_t running = m.bus->testPeekCpuRam(kRunningAll);
        if (running != 0) {
            runSeen = true;
            observedTestCount = std::max(observedTestCount, m.bus->testPeekCpuRam(kTotalTally));
        }

        const uint8_t total = m.bus->testPeekCpuRam(kTotalTally);
        if (runSeen && running == 0 && observedTestCount != 0 && total == observedTestCount &&
            m.bus->testPeekCpuRam(kMenuTabX) == 0) {
            const uint8_t passed = m.bus->testPeekCpuRam(kPassTally);
            std::vector<AccuracyCoinFailure> failures;
            for (uint16_t addr = 0x0400; addr <= 0x04FF; ++addr) {
                const uint8_t raw = m.bus->testPeekCpuRam(addr);
                if ((raw & 0x03) == 0x02)
                    failures.push_back({addr, raw, static_cast<uint8_t>(raw >> 2)});
            }

            if (options.json) {
                std::ostringstream os;
                os << "{\"protocol\":\"accuracycoin\",\"passed\":" << unsigned(passed)
                   << ",\"total\":" << unsigned(total)
                   << ",\"failed\":" << (unsigned(total) - unsigned(passed))
                   << ",\"cycles\":" << now << ",\"failures\":[";
                for (std::size_t i = 0; i < failures.size(); ++i) {
                    if (i) os << ',';
                    os << "{\"address\":\"$" << std::hex << std::uppercase
                       << std::setw(4) << std::setfill('0') << failures[i].address
                       << "\",\"raw\":\"$" << std::setw(2) << unsigned(failures[i].raw)
                       << "\",\"error_code\":" << std::dec << unsigned(failures[i].errorCode) << '}';
                }
                os << "]}";
                std::cout << os.str() << '\n';
            } else {
                std::cout << "AccuracyCoin: " << unsigned(passed) << "/" << unsigned(total)
                          << " passed (" << (unsigned(total) - unsigned(passed)) << " failed)\n";
                for (const auto& f : failures) {
                    std::cout << "  FAIL RAM $" << std::hex << std::uppercase << std::setw(4)
                              << std::setfill('0') << f.address << " raw=$" << std::setw(2)
                              << unsigned(f.raw) << std::dec << " error=" << unsigned(f.errorCode) << "\n";
                }
            }
            if (options.accuracyCoinMinPass >= 0)
                return passed >= static_cast<unsigned>(options.accuracyCoinMinPass) ? 0 : 1;
            return passed == total ? 0 : 1;
        }
    }

    std::cerr << "ACCURACYCOIN TIMEOUT: automated full-suite run did not complete within "
              << options.maxCycles << " CPU cycles.\n";
    return 5;
}
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--self-test")
        return runBuiltInProbeSuite(argc > 0 ? argv[0] : nullptr);

    Options options;
    if (!parseArgs(argc, argv, options)) {
        usage(argc > 0 ? argv[0] : "NESUltimateEmu.Tests.exe");
        return 2;
    }

    Machine m;
    m.apu->setDmcCpuRevision(options.dmcRevision);
    if (!m.cart->loadFromFile(options.romPath)) {
        std::cerr << "ERROR: unable to load ROM: " << options.romPath << "\n";
        return 2;
    }
    if (!m.cart->mapperSupported()) {
        std::cerr << "ERROR: mapper " << m.cart->mapper()
                  << " submapper " << unsigned(m.cart->submapper())
                  << " is not supported by this build.\n";
        return 2;
    }

    const ConsoleTiming selectedTiming = options.forceTiming ? options.timing : m.cart->timing();
    m.bus->setTiming(selectedTiming);
    m.bus->powerOn();

    if (!options.quiet) {
        std::cout << "ROM: " << m.cart->fileName()
                  << " | mapper " << m.cart->mapper()
                  << "." << unsigned(m.cart->submapper())
                  << " | timing " << consoleTimingName(selectedTiming)
                  << (options.forceTiming ? " (forced)" : " (ROM)") << "\n";
    }

    if (options.accuracyCoin)
        return runAccuracyCoin(m, options);
    if (options.legacyApu)
        return runLegacyApu(m, options);
    if (options.legacyShell)
        return runLegacyShell(m, options);
    if (options.scanlineVisual)
        return runScanlineVisual(m, options);

    bool protocolSeen = false;
    bool resetPending = false;
    bool resetRequestServiced = false;
    uint64_t resetAt = 0;
    std::string lastMessage;

    while (m.bus->cpuCycleCounter() < options.maxCycles) {
        m.bus->clock();
        const uint64_t now = m.bus->cpuCycleCounter();

        if (resetPending && now >= resetAt) {
            if (!options.quiet)
                std::cout << "[test] applying requested console reset\n";
            m.bus->reset();
            resetPending = false;
        }

        if ((now % kPollInterval) != 0)
            continue;

        if (!hasBlarggSignature(*m.bus))
            continue;

        protocolSeen = true;
        const uint8_t status = m.bus->read(kStatus);
        const std::string message = readBlarggMessage(*m.bus);
        if (!options.quiet && !message.empty() && message != lastMessage) {
            std::cout << message;
            if (message.back() != '\n') std::cout << '\n';
            lastMessage = message;
        }

        if (status == 0x80) {
            resetRequestServiced = false;
            continue;
        }
        if (status == 0x81) {
            // A reset-test ROM can leave $6000 at $81 for a short time after
            // RESET has been asserted. Treat the entire contiguous $81 period
            // as one request; otherwise the polling loop can schedule a second
            // reset before the ROM's reset handler has a chance to update the
            // status byte.
            if (!resetPending && !resetRequestServiced) {
                resetPending = true;
                resetRequestServiced = true;
                resetAt = now + kResetDelayCycles;
                if (!options.quiet)
                    std::cout << "[test] reset requested; delaying >=100 ms of emulated CPU time\n";
            }
            continue;
        }

        resetRequestServiced = false;
        if (status <= 0x7F) {
            if (message.empty()) {
                const std::string finalMessage = readBlarggMessage(*m.bus);
                if (!finalMessage.empty()) std::cout << finalMessage << '\n';
            }
            const uint8_t expected = options.expectedResultSet ? options.expectedResult : 0;
            if (status == expected) {
                std::cout << "PASS: " << m.cart->fileName();
                if (options.expectedResultSet && expected != 0)
                    std::cout << " (pinned expected result " << unsigned(expected) << ")";
                std::cout << " (" << now << " CPU cycles)\n";
                return 0;
            }
            std::cout << "FAIL: " << m.cart->fileName()
                      << " returned code " << unsigned(status);
            if (options.expectedResultSet)
                std::cout << ", expected " << unsigned(expected);
            std::cout << " (" << now << " CPU cycles)\n";
            return 1;
        }
    }

    if (protocolSeen) {
        std::cerr << "TIMEOUT: test protocol detected but did not finish within "
                  << options.maxCycles << " CPU cycles.\n";
        return 3;
    }

    std::cerr << "UNSUPPORTED/NO PROTOCOL: no Blargg $6000 signature was detected within "
              << options.maxCycles << " CPU cycles.\n";
    return 4;
}
