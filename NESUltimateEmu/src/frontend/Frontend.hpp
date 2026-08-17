#pragma once
#include <SDL.h>
#include "imgui.h"
#include "Debugger.hpp"
#include <deque>
#include <fstream>
#include <set>
#include <string>

class CPU;
class Bus;
class Cartridge;
class PPU;
class APU;

class Frontend {
public:
    Frontend(SDL_Window* window, SDL_Renderer* renderer,
        CPU& cpu, Bus& bus, Cartridge& cart, PPU& ppu, APU& apu);
    ~Frontend();

    void run();

private:
    SDL_Window* m_window;
    SDL_Renderer* m_renderer;
    CPU& m_cpu;
    Bus& m_bus;
    Cartridge& m_cart;
    PPU& m_ppu;
    APU& m_apu;
    Debugger m_debugger;
    bool          m_running;

    SDL_Texture* m_nesTexture = nullptr;
    SDL_GameController* m_gamepad = nullptr;

    std::string m_statusMessage;
    bool        m_statusIsError = false;

    bool m_paused = false;
    int  m_scale = 3;
    bool m_fullscreen = false;
    bool m_ntscAspect = true;
    int  m_saveSlot = 0; // 0..9

    // Debugger state is frontend-only and never affects save-state compatibility.
    bool m_debuggerOpen = false;
    bool m_traceEnabled = false;
    bool m_traceToFile = false;
    bool m_followPc = true;
    bool m_skipBreakpointOnce = false;
    uint16_t m_skipBreakpointPc = 0;
    uint64_t m_skipBreakpointInstructionCount = 0;
    std::set<uint16_t> m_breakpoints;
    std::deque<std::string> m_traceLines;
    std::ofstream m_traceFile;
    char m_breakpointText[8] = "8000";
    char m_disasmAddressText[8] = "8000";
    char m_cpuMemoryAddressText[8] = "0000";
    char m_ppuMemoryAddressText[8] = "2000";
    uint16_t m_disasmAddress = 0x8000;
    uint16_t m_cpuMemoryAddress = 0x0000;
    uint16_t m_ppuMemoryAddress = 0x2000;

    void processEvents();
    void updateControllers();
    void drawUI();
    void drawDebugger();
    void runFrame();
    void clockSystem();
    bool checkBreakpoint();
    void stepCpuCycle();
    void stepInstruction();
    void stepFrame();
    void captureTrace(uint64_t cpuCycle, int ppuScanline, int ppuCycle);
    void setTraceFileEnabled(bool enabled);
    void resetDebugSessionForRom();
    bool openRomDialog();
    void updateTexture();
    void openGamepad();
    void toggleFullscreen();
    void saveState();
    void loadState();
    std::string statePath(int slot) const;
};







