#include "../src/core/CPU.hpp"
#include "../src/core/Bus.hpp"
#include "../src/core/PPU.hpp"
#include "../src/core/APU.hpp"
#include "../src/core/Cartridge.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace {
constexpr uint64_t kDefaultMaxCycles = 120000000ULL;
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
};

void usage(const char* exe)
{
    std::cerr
        << "NES Ultimate Emulator - headless regression runner\n\n"
        << "Usage:\n  " << exe << " <test.nes> [--max-cycles N] [--dmc-revision early|late] [--quiet]\n\n"
        << "Exit codes:\n"
        << "  0  test passed\n"
        << "  1  test reported failure\n"
        << "  2  ROM could not be loaded\n"
        << "  3  timed out after detecting the test protocol\n"
        << "  4  no supported test protocol was detected\n";
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
        } else if (arg == "--max-cycles" && i + 1 < argc) {
            if (!parseU64(argv[++i], out.maxCycles) || out.maxCycles == 0)
                return false;
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
}

int main(int argc, char** argv)
{
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

    m.bus->powerOn();

    if (!options.quiet) {
        std::cout << "ROM: " << m.cart->fileName()
                  << " | mapper " << m.cart->mapper()
                  << "." << unsigned(m.cart->submapper()) << "\n";
    }

    bool protocolSeen = false;
    bool resetPending = false;
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
            continue;
        }
        if (status == 0x81) {
            if (!resetPending) {
                resetPending = true;
                resetAt = now + kResetDelayCycles;
                if (!options.quiet)
                    std::cout << "[test] reset requested; delaying >=100 ms of emulated CPU time\n";
            }
            continue;
        }

        if (status <= 0x7F) {
            if (message.empty()) {
                const std::string finalMessage = readBlarggMessage(*m.bus);
                if (!finalMessage.empty()) std::cout << finalMessage << '\n';
            }
            if (status == 0) {
                std::cout << "PASS: " << m.cart->fileName()
                          << " (" << now << " CPU cycles)\n";
                return 0;
            }
            std::cout << "FAIL: " << m.cart->fileName()
                      << " returned code " << unsigned(status)
                      << " (" << now << " CPU cycles)\n";
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
