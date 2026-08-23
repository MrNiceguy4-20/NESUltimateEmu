#include "ProbeSuite.hpp"

#include <filesystem>
#include <iostream>
#include <string>

namespace {
struct Result {
    const char* name;
    int code;
};

std::filesystem::path probeAsset(const char* executablePath, const char* fileName)
{
    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);

    std::filesystem::path exeDir;
    if (executablePath && *executablePath) {
        std::filesystem::path exe = std::filesystem::absolute(executablePath, ec);
        if (!ec) exeDir = exe.parent_path();
    }

    auto findFrom = [&](std::filesystem::path base) -> std::filesystem::path {
        for (unsigned depth = 0; !base.empty() && depth < 5; ++depth) {
            const std::filesystem::path beside = base / fileName;
            if (std::filesystem::is_regular_file(beside, ec)) return beside;
            ec.clear();

            const std::filesystem::path sourceTree = base / "tests" / "probes" / fileName;
            if (std::filesystem::is_regular_file(sourceTree, ec)) return sourceTree;
            ec.clear();

            const std::filesystem::path parent = base.parent_path();
            if (parent == base) break;
            base = parent;
        }
        return {};
    };

    if (!exeDir.empty()) {
        if (const std::filesystem::path found = findFrom(exeDir); !found.empty())
            return found;
    }
    if (!cwd.empty()) {
        if (const std::filesystem::path found = findFrom(cwd); !found.empty())
            return found;
    }

    return (exeDir.empty() ? cwd : exeDir) / fileName;
}
}

int runBuiltInProbeSuite(const char* executablePath)
{
    const std::string dmcApu = probeAsset(executablePath, "dmc_apu_conflict.nes").string();
    const std::string dmcPpu = probeAsset(executablePath, "dmc_ppu_conflict_probe.nes").string();
    const std::string dmcPpuAlign = probeAsset(executablePath, "dmc_ppu_conflict_probe_align.nes").string();

    std::cout << "NES Ultimate Emulator - built-in regression probes\n";

    Result results[] = {
        {"CPU conformance", runCpuConformanceProbe()},
        {"Timing/region conformance", runTimingConformanceProbe()},
        {"APU conformance", runApuConformanceProbe()},
        {"Cartridge conformance", runCartridgeConformanceProbe()},
        {"Cheat/Game Genie conformance", runCheatConformanceProbe()},
        {"Interrupt hijack", runInterruptHijackProbe()},
        {"Mapper conformance", runMapperConformanceProbe()},
        {"DMC load start", runDmcLoadStartProbe()},
        {"DMA arbitration", runDmaArbitrationProbe()},
        {"PPU conformance", runPpuConformanceProbe()},
        {"PPU open bus/OAM", runPpuOpenBusOamProbe()},
        {"DMC/PPU conflict", runDmcPpuConflictProbe(dmcPpu)},
        {"DMC/PPU conflict (aligned)", runDmcPpuConflictProbe(dmcPpuAlign)},
        {"DMC/APU conflict", runDmcApuConflictProbe(dmcApu)},
    };

    unsigned passed = 0;
    std::cout << "\n=== BUILT-IN PROBE SUMMARY ===\n";
    for (const Result& result : results) {
        const bool ok = result.code == 0;
        passed += ok ? 1u : 0u;
        std::cout << (ok ? "PASS" : "FAIL") << " [" << result.code << "] " << result.name << "\n";
    }
    const unsigned total = static_cast<unsigned>(sizeof(results) / sizeof(results[0]));
    std::cout << passed << "/" << total << " probes passed\n";
    return passed == total ? 0 : 1;
}
