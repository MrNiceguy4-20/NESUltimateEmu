#pragma once

#include <string>

int runCpuConformanceProbe();
int runApuConformanceProbe();
int runCartridgeConformanceProbe();
int runCheatConformanceProbe();
int runDmcApuConflictProbe(const std::string& romPath);
int runDmcLoadStartProbe();
int runDmaArbitrationProbe();
int runDmcPpuConflictProbe(const std::string& romPath);
int runInterruptHijackProbe();
int runMapperConformanceProbe();
int runPpuOpenBusOamProbe();
int runPpuConformanceProbe();
int runTimingConformanceProbe();

int runBuiltInProbeSuite(const char* executablePath);
