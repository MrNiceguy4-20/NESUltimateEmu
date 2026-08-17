#include "Frontend.hpp"
#include <fstream>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <sstream>
#include <iomanip>
#include "../core/CPU.hpp"
#include "../core/Bus.hpp"
#include "../core/Cartridge.hpp"
#include "../core/PPU.hpp"
#include "../core/APU.hpp"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#endif

namespace {

bool parseHex16(const char* text, uint16_t& value)
{
    if (!text || !*text) return false;
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 16);
    if (end == text || *end != '\0' || parsed > 0xFFFFul) return false;
    value = static_cast<uint16_t>(parsed);
    return true;
}

std::string cpuFlagString(uint8_t status)
{
    const char* names = "NVUBDIZC";
    std::string out;
    for (int bit = 7; bit >= 0; --bit) {
        const char c = names[7 - bit];
        out.push_back((status & (1u << bit)) ? c : '-');
    }
    return out;
}

std::string mapperBytesString(const std::vector<uint8_t>& bytes)
{
    if (bytes.empty()) return "(no mapper registers serialized)";
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) out << ' ';
        out << std::setw(2) << static_cast<unsigned>(bytes[i]);
    }
    return out.str();
}

} // namespace

Frontend::Frontend(SDL_Window* window, SDL_Renderer* renderer,
    CPU& cpu, Bus& bus, Cartridge& cart, PPU& ppu, APU& apu)
    : m_window(window)
    , m_renderer(renderer)
    , m_cpu(cpu)
    , m_bus(bus)
    , m_cart(cart)
    , m_ppu(ppu)
    , m_apu(apu)
    , m_debugger(bus)
    , m_running(true)
    , m_statusMessage("No ROM loaded. Click \"Load ROM...\" to choose a .nes or .fds file.")
{
    m_nesTexture = SDL_CreateTexture(
        m_renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        256, 240
    );
    SDL_SetTextureScaleMode(m_nesTexture, SDL_ScaleModeNearest);
    openGamepad();
}

void Frontend::openGamepad()
{
    for (int i = 0; i < SDL_NumJoysticks(); i++) {
        if (SDL_IsGameController(i)) {
            m_gamepad = SDL_GameControllerOpen(i);
            if (m_gamepad) break;
        }
    }
}

Frontend::~Frontend()
{
    m_cart.saveBattery();
    if (m_gamepad) {
        SDL_GameControllerClose(m_gamepad);
        m_gamepad = nullptr;
    }
    if (m_nesTexture)
        SDL_DestroyTexture(m_nesTexture);
}

void Frontend::clockSystem()
{
    if (!m_traceEnabled) {
        m_bus.clock();
        return;
    }

    const uint64_t beforeInstruction = m_cpu.instructionCount();
    const uint64_t cpuCycle = m_bus.cpuCycleCounter();
    const int ppuScanline = m_ppu.scanline();
    const int ppuCycle = m_ppu.cycle();

    m_bus.clock();

    if (m_cpu.instructionCount() != beforeInstruction)
        captureTrace(cpuCycle, ppuScanline, ppuCycle);
}

bool Frontend::checkBreakpoint()
{
    if (m_breakpoints.empty() || !m_cpu.atInstructionBoundary())
        return false;

    const uint16_t pc = m_cpu.debugState().pc;
    if (m_skipBreakpointOnce) {
        if (pc == m_skipBreakpointPc &&
            m_cpu.instructionCount() == m_skipBreakpointInstructionCount) {
            return false;
        }
        m_skipBreakpointOnce = false;
    }

    if (m_breakpoints.find(pc) == m_breakpoints.end())
        return false;

    m_paused = true;
    m_skipBreakpointOnce = true;
    m_skipBreakpointPc = pc;
    m_skipBreakpointInstructionCount = m_cpu.instructionCount();

    char message[64];
    std::snprintf(message, sizeof(message), "Breakpoint hit at $%04X.", static_cast<unsigned>(pc));
    m_statusMessage = message;
    m_statusIsError = false;
    return true;
}

void Frontend::runFrame()
{
    const int kMaxCycles = 100000;
    int guard = 0;
    m_ppu.clearFrameComplete();
    while (!m_ppu.frameComplete() && guard < kMaxCycles) {
        if (checkBreakpoint())
            break;
        clockSystem();
        ++guard;
    }
}

void Frontend::stepCpuCycle()
{
    if (!m_cart.isLoaded()) return;
    m_paused = true;
    clockSystem();
    updateTexture();
}

void Frontend::stepInstruction()
{
    if (!m_cart.isLoaded()) return;
    m_paused = true;

    const uint64_t startInstruction = m_cpu.instructionCount();
    int guard = 0;
    constexpr int kMaxStepCycles = 100000;
    do {
        clockSystem();
        ++guard;
    } while (guard < kMaxStepCycles &&
        (m_cpu.instructionCount() == startInstruction || !m_cpu.atInstructionBoundary()));

    if (guard >= kMaxStepCycles) {
        m_statusMessage = "Instruction step did not reach a boundary (CPU may be JAMmed).";
        m_statusIsError = true;
    }
    updateTexture();
}

void Frontend::stepFrame()
{
    if (!m_cart.isLoaded()) return;
    m_paused = true;
    runFrame();
    updateTexture();
}

void Frontend::captureTrace(uint64_t cpuCycle, int ppuScanline, int ppuCycle)
{
    if (!m_traceEnabled)
        return;

    const CPU::TraceState state = m_cpu.lastTraceState();
    std::string line = m_debugger.formatTrace(
        state.pc, state.opcode, state.operand1, state.operand2,
        state.a, state.x, state.y, state.sp, state.status,
        cpuCycle, ppuScanline, ppuCycle);

    constexpr std::size_t kMaxTraceLines = 1000;
    m_traceLines.push_back(line);
    while (m_traceLines.size() > kMaxTraceLines)
        m_traceLines.pop_front();

    if (m_traceToFile) {
        if (!m_traceFile.is_open())
            setTraceFileEnabled(true);
        if (m_traceFile.is_open())
            m_traceFile << line << '\n';
    }
}

void Frontend::setTraceFileEnabled(bool enabled)
{
    if (enabled) {
        m_traceEnabled = true;
        m_cpu.setTraceCaptureEnabled(true);
    }
    m_traceToFile = enabled;
    if (m_traceFile.is_open())
        m_traceFile.close();

    if (!enabled || !m_cart.isLoaded())
        return;

    m_traceFile.open(m_cart.path() + ".trace.txt", std::ios::out | std::ios::trunc);
    if (!m_traceFile) {
        m_traceToFile = false;
        m_statusMessage = "Failed to open trace output file.";
        m_statusIsError = true;
    }
}

void Frontend::resetDebugSessionForRom()
{
    m_traceLines.clear();
    m_skipBreakpointOnce = false;
    const uint16_t pc = m_cpu.debugState().pc;
    std::snprintf(m_disasmAddressText, sizeof(m_disasmAddressText), "%04X", static_cast<unsigned>(pc));
    m_disasmAddress = pc;
    if (m_traceToFile)
        setTraceFileEnabled(true);
}

void Frontend::updateTexture()
{
    if (!m_nesTexture) return;
    void* pixels = nullptr;
    int pitch = 0;
    if (SDL_LockTexture(m_nesTexture, nullptr, &pixels, &pitch) == 0) {
        const uint32_t* src = m_ppu.framebuffer();
        uint8_t* dst = static_cast<uint8_t*>(pixels);
        for (int y = 0; y < 240; y++) {
            memcpy(dst + y * pitch, src + y * 256, 256 * sizeof(uint32_t));
        }
        SDL_UnlockTexture(m_nesTexture);
    }
}

void Frontend::run()
{
    const double targetFrameMs = 1000.0 / 60.0988; // NTSC
    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 last = SDL_GetPerformanceCounter();

    while (m_running) {
        processEvents();
        updateControllers();

        if (m_cart.isLoaded() && !m_paused) {
            runFrame();
            updateTexture();
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        drawUI();
        ImGui::Render();

        SDL_SetRenderDrawColor(m_renderer, 20, 20, 30, 255);
        SDL_RenderClear(m_renderer);

        if (m_cart.isLoaded() && m_nesTexture) {
            int winW = 0, winH = 0;
            SDL_GetWindowSize(m_window, &winW, &winH);
            int tw = 256 * m_scale;
            int th = 240 * m_scale;
            if (m_ntscAspect) {
                // NTSC pixel aspect ~8:7
                tw = (256 * m_scale * 8) / 7;
            }
            SDL_Rect dst{ (winW - tw) / 2, (winH - th) / 2, tw, th };
            SDL_RenderCopy(m_renderer, m_nesTexture, nullptr, &dst);
        }

        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), m_renderer);
        SDL_RenderPresent(m_renderer);

        Uint64 now = SDL_GetPerformanceCounter();
        double elapsedMs = (double)(now - last) / (double)freq * 1000.0;
        if (elapsedMs < targetFrameMs) {
            SDL_Delay((Uint32)(targetFrameMs - elapsedMs));
        }
        last = SDL_GetPerformanceCounter();
    }
}

void Frontend::updateControllers()
{
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    // Bus expects: bit0=A bit1=B bit2=Select bit3=Start bit4=Up bit5=Down bit6=Left bit7=Right
    auto pack = [](bool a, bool b, bool sel, bool st, bool u, bool d, bool l, bool r) -> uint8_t {
        uint8_t v = 0;
        if (a)   v |= 0x01;
        if (b)   v |= 0x02;
        if (sel) v |= 0x04;
        if (st)  v |= 0x08;
        if (u)   v |= 0x10;
        if (d)   v |= 0x20;
        if (l)   v |= 0x40;
        if (r)   v |= 0x80;
        return v;
        };

    bool p1a = keys[SDL_SCANCODE_Z] || keys[SDL_SCANCODE_A];
    bool p1b = keys[SDL_SCANCODE_X] || keys[SDL_SCANCODE_S];
    bool p1sel = keys[SDL_SCANCODE_RSHIFT] || keys[SDL_SCANCODE_BACKSPACE];
    bool p1st = keys[SDL_SCANCODE_RETURN];
    bool p1u = keys[SDL_SCANCODE_UP];
    bool p1d = keys[SDL_SCANCODE_DOWN];
    bool p1l = keys[SDL_SCANCODE_LEFT];
    bool p1r = keys[SDL_SCANCODE_RIGHT];

    if (m_gamepad) {
        p1a = p1a || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_A);
        p1b = p1b || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_B);
        p1sel = p1sel || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_BACK);
        p1st = p1st || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_START);
        p1u = p1u || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_UP);
        p1d = p1d || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        p1l = p1l || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        p1r = p1r || SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        Sint16 lx = SDL_GameControllerGetAxis(m_gamepad, SDL_CONTROLLER_AXIS_LEFTX);
        Sint16 ly = SDL_GameControllerGetAxis(m_gamepad, SDL_CONTROLLER_AXIS_LEFTY);
        const Sint16 dead = 16000;
        if (ly < -dead) p1u = true;
        if (ly > dead) p1d = true;
        if (lx < -dead) p1l = true;
        if (lx > dead) p1r = true;
    }

    uint8_t p1 = pack(p1a, p1b, p1sel, p1st, p1u, p1d, p1l, p1r);

    // Player 2: H=A G=B T=Select Y=Start IJKL=D-pad
    uint8_t p2 = pack(
        keys[SDL_SCANCODE_H],
        keys[SDL_SCANCODE_G],
        keys[SDL_SCANCODE_T],
        keys[SDL_SCANCODE_Y],
        keys[SDL_SCANCODE_I],
        keys[SDL_SCANCODE_K],
        keys[SDL_SCANCODE_J],
        keys[SDL_SCANCODE_L]
    );

    m_bus.setController1(p1);
    m_bus.setController2(p2);
}

void Frontend::processEvents()
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        ImGui_ImplSDL2_ProcessEvent(&e);
        if (e.type == SDL_QUIT)
            m_running = false;
        if (e.type == SDL_CONTROLLERDEVICEADDED && !m_gamepad) {
            openGamepad();
        }
        if (e.type == SDL_CONTROLLERDEVICEREMOVED && m_gamepad) {
            SDL_GameControllerClose(m_gamepad);
            m_gamepad = nullptr;
        }
        if (e.type == SDL_KEYDOWN) {
            switch (e.key.keysym.sym) {
            case SDLK_ESCAPE: m_running = false; break;
            case SDLK_p: m_paused = !m_paused; break;
            case SDLK_F8: m_debuggerOpen = !m_debuggerOpen; break;
            case SDLK_F10:
                if (m_cart.isLoaded()) stepInstruction();
                break;
            case SDLK_r:
                if (m_cart.isLoaded()) {
                    m_bus.reset();
                    resetDebugSessionForRom();
                    m_statusMessage = "System reset.";
                    m_statusIsError = false;
                }
                break;
            case SDLK_1: m_scale = 1; break;
            case SDLK_2: m_scale = 2; break;
            case SDLK_3: m_scale = 3; break;
            case SDLK_4: m_scale = 4; break;
            case SDLK_F5: saveState(); break;
            case SDLK_F9: loadState(); break;
            case SDLK_F6:
                m_saveSlot = (m_saveSlot + 1) % 10;
                m_statusMessage = "Save slot: " + std::to_string(m_saveSlot);
                m_statusIsError = false;
                break;
            case SDLK_F7:
                m_saveSlot = (m_saveSlot + 9) % 10;
                m_statusMessage = "Save slot: " + std::to_string(m_saveSlot);
                m_statusIsError = false;
                break;
            case SDLK_a:
                m_ntscAspect = !m_ntscAspect;
                m_statusMessage = m_ntscAspect ? "NTSC aspect ON (8:7)" : "Square pixels";
                m_statusIsError = false;
                break;
            case SDLK_c:
                m_apu.setChipMod(!m_apu.chipMod());
                m_statusMessage = m_apu.chipMod()
                    ? "Chip Mod ON (KYLXBN-style 2A03/VRC6/VRC7/N163)"
                    : "Chip Mod OFF (accurate nonlinear mix)";
                m_statusIsError = false;
                break;
            case SDLK_F11:
                toggleFullscreen();
                break;
            }
        }
    }
}

void Frontend::drawUI()
{
    ImGui::Begin("NES Ultimate Emulator", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::Button("Load ROM...")) {
        openRomDialog();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset") && m_cart.isLoaded()) {
        m_bus.reset();
        resetDebugSessionForRom();
        m_statusMessage = "System reset.";
        m_statusIsError = false;
    }
    ImGui::SameLine();
    if (ImGui::Button(m_paused ? "Resume" : "Pause")) {
        m_paused = !m_paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Debugger (F8)")) {
        m_debuggerOpen = !m_debuggerOpen;
    }

    ImGui::Separator();

    if (m_statusIsError)
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s", m_statusMessage.c_str());
    else
        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%s", m_statusMessage.c_str());

    if (m_cart.isLoaded()) {
        ImGui::Separator();
        ImGui::Text("File   : %s", m_cart.fileName().c_str());
        if (m_cart.isFds()) {
            ImGui::Text("System : Famicom Disk System");
            const int side = m_cart.currentDiskSide();
            ImGui::Text("Disk   : %s  (%zu side%s)", m_cart.diskInserted() ? "inserted" : "ejected",
                m_cart.diskSideCount(), m_cart.diskSideCount() == 1 ? "" : "s");
            if (side >= 0)
                ImGui::Text("Side   : %d%c", side / 2 + 1, (side & 1) ? 'B' : 'A');
            if (m_cart.diskInserted()) {
                if (ImGui::Button("Eject Disk")) {
                    m_cart.ejectDisk();
                    m_statusMessage = "FDS disk ejected.";
                    m_statusIsError = false;
                }
            }
            for (std::size_t i = 0; i < m_cart.diskSideCount(); ++i) {
                if (i || m_cart.diskInserted()) ImGui::SameLine();
                char label[32];
                std::snprintf(label, sizeof(label), "Insert %zu%c##fds%zu", i / 2 + 1,
                    (i & 1) ? 'B' : 'A', i);
                if (ImGui::Button(label)) {
                    m_cart.setDiskSide(i);
                    m_statusMessage = "Inserted FDS side " + std::to_string(i / 2 + 1) + ((i & 1) ? "B" : "A") + ".";
                    m_statusIsError = false;
                }
            }
        } else {
            if (m_cart.isNes20())
                ImGui::Text("Header : NES 2.0");
            else
                ImGui::Text("Header : iNES");
            if (m_cart.isNes20())
                ImGui::Text("Mapper : %u  Submapper: %u%s", static_cast<unsigned>(m_cart.mapper()),
                    static_cast<unsigned>(m_cart.submapper()),
                    m_cart.mapperSupported() ? " (implemented)" : " (fallback – may not run)");
            else
                ImGui::Text("Mapper : %u%s", static_cast<unsigned>(m_cart.mapper()),
                    m_cart.mapperSupported() ? " (implemented)" : " (fallback – may not run)");
        }
        ImGui::Text("PRG ROM: %zu KB", m_cart.prgRomSize() / 1024);
        ImGui::Text("CHR ROM: %zu KB", m_cart.chrRomSize() / 1024);
        if (m_cart.prgRamSize())
            ImGui::Text("PRG RAM: %zu KB%s", m_cart.prgRamSize() / 1024,
                m_cart.prgNvRamSize() ? " (includes NVRAM)" : "");
        if (m_cart.chrRamSize())
            ImGui::Text("CHR RAM: %zu KB%s", m_cart.chrRamSize() / 1024,
                m_cart.chrNvRamSize() ? " (includes NVRAM)" : "");

        const char* mirrorStr = "Horizontal";
        switch (m_cart.mirroring()) {
        case Cartridge::Mirror::Vertical:    mirrorStr = "Vertical"; break;
        case Cartridge::Mirror::OnescreenLo: mirrorStr = "One-screen LO"; break;
        case Cartridge::Mirror::OnescreenHi: mirrorStr = "One-screen HI"; break;
        case Cartridge::Mirror::FourScreen:   mirrorStr = "Four-screen"; break;
        default: break;
        }
        ImGui::Text("Mirror : %s", mirrorStr);
        if (m_cart.hasBattery())
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "Battery RAM: yes (auto-saves .sav)");

        ImGui::Text("Scale  : %dx  (keys 1-4)", m_scale);
        ImGui::Text("Paused : %s  (P to toggle)", m_paused ? "yes" : "no");
        ImGui::Text("Slot   : %d  (F6/F7)", m_saveSlot);
        ImGui::Text("Aspect : %s  (A to toggle)", m_ntscAspect ? "NTSC 8:7" : "Square");

        ImGui::Separator();
        ImGui::Text("P1: Z/A=A  X/S=B  Enter=Start  RShift=Select  Arrows");
        ImGui::Text("P2: H=A  G=B  Y=Start  T=Select  IJKL=D-pad");
        ImGui::Text("          Gamepad → Player 1");
        ImGui::Text("          F5=Save  F9=Load  F6/F7=Slot  F11=Fullscreen");
        ImGui::Text("          F8=Debugger  F10=Step instruction (paused)");
        ImGui::Text("          C=Chip Mod  A=Aspect");

        if (m_apu.chipMod())
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.7f, 1.0f), "Chip Mod: ON");
        else
            ImGui::Text("Chip Mod: OFF");
    }

    ImGui::End();

    if (m_debuggerOpen)
        drawDebugger();
}

void Frontend::drawDebugger()
{
    ImGui::Begin("NES Debugger", &m_debuggerOpen);

    if (!m_cart.isLoaded()) {
        ImGui::Text("Load a ROM to use the debugger.");
        ImGui::End();
        return;
    }

    const CPU::DebugState cpu = m_cpu.debugState();
    const PPU::DebugState ppu = m_ppu.debugState();

    if (ImGui::Button(m_paused ? "Run" : "Pause"))
        m_paused = !m_paused;
    ImGui::SameLine();
    if (ImGui::Button("CPU Cycle")) stepCpuCycle();
    ImGui::SameLine();
    if (ImGui::Button("Instruction (F10)")) stepInstruction();
    ImGui::SameLine();
    if (ImGui::Button("Frame")) stepFrame();

    ImGui::Text("Execution: %s", m_paused ? "paused" : "running");
    ImGui::Text("CPU cycle: %llu   Instructions: %llu",
        static_cast<unsigned long long>(m_bus.cpuCycleCounter()),
        static_cast<unsigned long long>(cpu.instructionCount));

    if (ImGui::CollapsingHeader("CPU", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("PC:$%04X  A:$%02X  X:$%02X  Y:$%02X  SP:$%02X",
            static_cast<unsigned>(cpu.pc), static_cast<unsigned>(cpu.a),
            static_cast<unsigned>(cpu.x), static_cast<unsigned>(cpu.y),
            static_cast<unsigned>(cpu.sp));
        const std::string flags = cpuFlagString(cpu.status);
        ImGui::Text("P:$%02X  [%s]  cycles remaining:%d",
            static_cast<unsigned>(cpu.status), flags.c_str(), cpu.cyclesRemaining);
        ImGui::Text("NMI pending:%s  IRQ line:%s  OAM DMA:%s  DMC DMA:%s",
            cpu.nmiPending ? "yes" : "no", cpu.irqLine ? "high" : "low",
            m_bus.dmaActive() ? "active" : "idle", m_bus.dmcDmaActive() ? "active" : "idle");
    }

    if (ImGui::CollapsingHeader("Breakpoints", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputText("Address##breakpoint", m_breakpointText, sizeof(m_breakpointText),
            ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
        ImGui::SameLine();
        if (ImGui::Button("Add")) {
            uint16_t address = 0;
            if (parseHex16(m_breakpointText, address)) {
                m_breakpoints.insert(address);
                m_statusMessage = "Breakpoint added.";
                m_statusIsError = false;
            }
            else {
                m_statusMessage = "Invalid breakpoint address.";
                m_statusIsError = true;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear all")) m_breakpoints.clear();

        if (m_breakpoints.empty()) {
            ImGui::Text("No execution breakpoints.");
        }
        else {
            for (auto it = m_breakpoints.begin(); it != m_breakpoints.end();) {
                char label[16];
                std::snprintf(label, sizeof(label), "$%04X", static_cast<unsigned>(*it));
                ImGui::Text("%s", label);
                ImGui::SameLine();
                char button[32];
                std::snprintf(button, sizeof(button), "Remove##%04X", static_cast<unsigned>(*it));
                if (ImGui::Button(button))
                    it = m_breakpoints.erase(it);
                else
                    ++it;
            }
        }
    }

    if (ImGui::CollapsingHeader("Disassembly", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Follow PC", &m_followPc);
        if (!m_followPc) {
            ImGui::InputText("Start##disasm", m_disasmAddressText, sizeof(m_disasmAddressText),
                ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
            ImGui::SameLine();
            if (ImGui::Button("Go##disasm")) {
                uint16_t address = 0;
                if (parseHex16(m_disasmAddressText, address))
                    m_disasmAddress = address;
            }
        }

        uint16_t address = m_followPc ? cpu.pc : m_disasmAddress;
        for (int i = 0; i < 24; ++i) {
            const DisassembledInstruction inst = m_debugger.disassemble(address);
            char line[128];
            std::snprintf(line, sizeof(line), "%c $%04X  %-9s %s",
                address == cpu.pc ? '>' : ' ', static_cast<unsigned>(address),
                inst.bytes.c_str(), inst.text.c_str());
            if (address == cpu.pc)
                ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "%s", line);
            else
                ImGui::TextUnformatted(line);
            address = static_cast<uint16_t>(address + inst.length);
        }
    }

    if (ImGui::CollapsingHeader("Trace")) {
        if (ImGui::Checkbox("Capture instruction trace", &m_traceEnabled)) {
            m_cpu.setTraceCaptureEnabled(m_traceEnabled);
            if (!m_traceEnabled && m_traceToFile)
                setTraceFileEnabled(false);
        }
        bool fileTrace = m_traceToFile;
        if (ImGui::Checkbox("Write trace to ROM .trace.txt", &fileTrace))
            setTraceFileEnabled(fileTrace);
        ImGui::SameLine();
        if (ImGui::Button("Clear trace")) m_traceLines.clear();
        ImGui::Text("Buffered lines: %zu / 1000", m_traceLines.size());
        if (m_traceToFile)
            ImGui::Text("File: %s.trace.txt", m_cart.path().c_str());

        const std::size_t show = 24;
        const std::size_t start = m_traceLines.size() > show ? m_traceLines.size() - show : 0;
        for (std::size_t i = start; i < m_traceLines.size(); ++i)
            ImGui::TextUnformatted(m_traceLines[i].c_str());
    }

    if (ImGui::CollapsingHeader("CPU Memory")) {
        ImGui::InputText("Start##cpumem", m_cpuMemoryAddressText, sizeof(m_cpuMemoryAddressText),
            ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
        ImGui::SameLine();
        if (ImGui::Button("Go##cpumem")) {
            uint16_t address = 0;
            if (parseHex16(m_cpuMemoryAddressText, address))
                m_cpuMemoryAddress = address & 0xFFF0;
        }
        ImGui::Text("Side-effect-free CPU bus view (256 bytes)");
        for (int row = 0; row < 16; ++row) {
            const uint16_t base = static_cast<uint16_t>(m_cpuMemoryAddress + row * 16);
            std::ostringstream line;
            line << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
                << static_cast<unsigned>(base) << ": ";
            for (int col = 0; col < 16; ++col)
                line << std::setw(2) << static_cast<unsigned>(m_bus.debugRead(static_cast<uint16_t>(base + col))) << ' ';
            ImGui::TextUnformatted(line.str().c_str());
        }
    }

    if (ImGui::CollapsingHeader("PPU", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Scanline:%d  Dot:%d  master:%llu  odd:%s",
            ppu.scanline, ppu.cycle, static_cast<unsigned long long>(ppu.masterClock), ppu.oddFrame ? "yes" : "no");
        ImGui::Text("CTRL:$%02X MASK:$%02X STATUS:$%02X OAMADDR:$%02X",
            static_cast<unsigned>(ppu.ctrl), static_cast<unsigned>(ppu.mask),
            static_cast<unsigned>(ppu.status), static_cast<unsigned>(ppu.oamAddr));
        ImGui::Text("v:$%04X t:$%04X fineX:%u w:%u buffer:$%02X bus:$%02X",
            static_cast<unsigned>(ppu.v), static_cast<unsigned>(ppu.t),
            static_cast<unsigned>(ppu.fineX), ppu.writeToggle ? 1u : 0u,
            static_cast<unsigned>(ppu.dataBuffer), static_cast<unsigned>(ppu.busLatch));
        ImGui::Text("Sprites:%u  NMI line:%s  NMI delay:%u",
            static_cast<unsigned>(ppu.spriteCount), ppu.nmiLine ? "high" : "low",
            static_cast<unsigned>(ppu.nmiDelay));
        ImGui::Text("VBlank:%s Sprite0Hit:%s Overflow:%s",
            (ppu.status & 0x80) ? "yes" : "no", (ppu.status & 0x40) ? "yes" : "no",
            (ppu.status & 0x20) ? "yes" : "no");

        ImGui::InputText("VRAM start##ppumem", m_ppuMemoryAddressText, sizeof(m_ppuMemoryAddressText),
            ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);
        ImGui::SameLine();
        if (ImGui::Button("Go##ppumem")) {
            uint16_t address = 0;
            if (parseHex16(m_ppuMemoryAddressText, address)) {
                address &= 0x3FFF;
                if (address < 0x2000) address = 0x2000;
                m_ppuMemoryAddress = address & 0xFFF0;
            }
        }
        ImGui::Text("Nametable/palette VRAM only; pattern reads are omitted to avoid mapper latch side effects.");
        for (int row = 0; row < 8; ++row) {
            const uint16_t base = static_cast<uint16_t>((m_ppuMemoryAddress + row * 16) & 0x3FFF);
            std::ostringstream line;
            line << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
                << static_cast<unsigned>(base) << ": ";
            for (int col = 0; col < 16; ++col)
                line << std::setw(2) << static_cast<unsigned>(m_ppu.debugVramRead(static_cast<uint16_t>(base + col))) << ' ';
            ImGui::TextUnformatted(line.str().c_str());
        }

        ImGui::Text("OAM $00-$3F:");
        for (int row = 0; row < 4; ++row) {
            std::ostringstream line;
            line << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << row * 16 << ": ";
            for (int col = 0; col < 16; ++col)
                line << std::setw(2) << static_cast<unsigned>(m_ppu.debugOamRead(static_cast<uint8_t>(row * 16 + col))) << ' ';
            ImGui::TextUnformatted(line.str().c_str());
        }
    }

    if (ImGui::CollapsingHeader("APU")) {
        const APU::DebugState apu = m_apu.debugState();
        auto drawChannel = [](const char* name, const APU::ChannelDebug& ch) {
            ImGui::Text("%-8s %s timer:%u period:%u length:%u value:%u", name,
                ch.enabled ? "ON " : "OFF", static_cast<unsigned>(ch.timer),
                static_cast<unsigned>(ch.period), static_cast<unsigned>(ch.length),
                static_cast<unsigned>(ch.volume));
        };
        drawChannel("Pulse 1", apu.pulse1);
        drawChannel("Pulse 2", apu.pulse2);
        drawChannel("Triangle", apu.triangle);
        drawChannel("Noise", apu.noise);
        drawChannel("DMC", apu.dmc);
        ImGui::Text("Triangle linear:%u  Noise LFSR:$%04X",
            static_cast<unsigned>(apu.triangleLinear), static_cast<unsigned>(apu.noiseShift));
        ImGui::Text("DMC addr:$%04X bytes:%u bits:%u DMA:%s",
            static_cast<unsigned>(apu.dmcAddress), static_cast<unsigned>(apu.dmcBytesRemaining),
            static_cast<unsigned>(apu.dmcBitsRemaining), apu.dmcDmaPending ? "pending" : "idle");
        ImGui::Text("Frame:%s-step cycle:%u IRQ inhibit:%s frame IRQ:%s DMC IRQ:%s",
            apu.frameMode5 ? "5" : "4", static_cast<unsigned>(apu.frameCycles),
            apu.irqInhibit ? "yes" : "no", apu.frameIrq ? "yes" : "no", apu.dmcIrq ? "yes" : "no");
    }

    if (ImGui::CollapsingHeader("Mapper / Cartridge")) {
        ImGui::Text("Mapper:%u  Submapper:%u  %s", static_cast<unsigned>(m_cart.mapper()),
            static_cast<unsigned>(m_cart.submapper()), m_cart.mapperSupported() ? "implemented" : "fallback");
        ImGui::Text("IRQ:%s  PRG ROM:%zu KB  CHR ROM:%zu KB  PRG RAM:%zu KB  CHR RAM:%zu KB",
            m_cart.irqActive() ? "active" : "idle", m_cart.prgRomSize() / 1024,
            m_cart.chrRomSize() / 1024, m_cart.prgRamSize() / 1024, m_cart.chrRamSize() / 1024);
        std::vector<uint8_t> mapperState;
        m_cart.mapperDebugState(mapperState);
        const std::string raw = mapperBytesString(mapperState);
        ImGui::TextWrapped("Raw mapper state (%zu bytes): %s", mapperState.size(), raw.c_str());
    }

    ImGui::End();
}

bool Frontend::openRomDialog()
{
#ifdef _WIN32
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = "NES / FDS Images (*.nes;*.fds)\0*.nes;*.fds\0NES ROMs (*.nes)\0*.nes\0FDS Images (*.fds)\0*.fds\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = "Select NES / FDS Image";

    if (!GetOpenFileNameA(&ofn))
        return false;

    m_cart.saveBattery();
    if (m_cart.loadFromFile(filename)) {
        m_bus.powerOn();
        resetDebugSessionForRom();
        m_statusMessage = "Loaded: " + m_cart.fileName();
        m_statusIsError = false;
        m_paused = false;
        return true;
    }
    else {
        {
            std::string failedPath = filename;
            for (char& c : failedPath) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (failedPath.size() >= 4 && failedPath.substr(failedPath.size() - 4) == ".fds")
                m_statusMessage = "Failed to load FDS image. Place a valid 8 KiB disksys.rom beside the .fds file.";
            else
                m_statusMessage = "Failed to load ROM (invalid iNES/NES 2.0 image).";
        }
        m_statusIsError = true;
        return false;
    }
#else
    m_statusMessage = "Native file dialog only on Windows.";
    m_statusIsError = true;
    return false;
#endif
}

std::string Frontend::statePath(int slot) const
{
    return m_cart.path() + ".state" + std::to_string(slot);
}

namespace {
constexpr uint8_t kSaveStateVersion = 13;

void appendU32(std::vector<uint8_t>& out, uint32_t value)
{
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
}

void appendU64(std::vector<uint8_t>& out, uint64_t value)
{
    for (int i = 0; i < 8; ++i)
        out.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
}

uint32_t readU32(const uint8_t* p)
{
    return uint32_t(p[0]) |
        (uint32_t(p[1]) << 8) |
        (uint32_t(p[2]) << 16) |
        (uint32_t(p[3]) << 24);
}

uint64_t readU64(const uint8_t* p)
{
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i)
        value |= uint64_t(p[i]) << (i * 8);
    return value;
}

uint32_t stateChecksum(const uint8_t* data, size_t size)
{
    // FNV-1a checksum catches truncation/corruption before live emulator
    // state is modified during a load.
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < size; ++i) {
        hash ^= data[i];
        hash *= 16777619u;
    }
    return hash;
}
}

void Frontend::saveState()
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "No ROM loaded – cannot save state.";
        m_statusIsError = true;
        return;
    }

    std::vector<uint8_t> payload;
    m_cpu.saveState(payload);
    m_ppu.saveState(payload);
    m_bus.saveState(payload);
    m_apu.saveState(payload);
    m_cart.saveState(payload);

    std::vector<uint8_t> buf;
    buf.reserve(21 + payload.size());
    buf.push_back('N'); buf.push_back('E'); buf.push_back('S'); buf.push_back('U');
    buf.push_back(kSaveStateVersion);
    appendU64(buf, m_cart.romIdentity());
    appendU32(buf, static_cast<uint32_t>(payload.size()));
    appendU32(buf, stateChecksum(payload.data(), payload.size()));
    buf.insert(buf.end(), payload.begin(), payload.end());

    std::string path = statePath(m_saveSlot);
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        m_statusMessage = "Failed to write save state.";
        m_statusIsError = true;
        return;
    }
    f.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());
    m_statusMessage = "State saved → slot " + std::to_string(m_saveSlot);
    m_statusIsError = false;
}

void Frontend::loadState()
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "No ROM loaded – cannot load state.";
        m_statusIsError = true;
        return;
    }

    std::string path = statePath(m_saveSlot);
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        m_statusMessage = "No save state in slot " + std::to_string(m_saveSlot);
        m_statusIsError = true;
        return;
    }
    auto sz = f.tellg();
    if (sz <= 0) {
        m_statusMessage = "Save state file is empty or unreadable.";
        m_statusIsError = true;
        return;
    }
    f.seekg(0);
    std::vector<uint8_t> buf((size_t)sz);
    f.read(reinterpret_cast<char*>(buf.data()), sz);
    if (!f) {
        m_statusMessage = "Failed to read save state file.";
        m_statusIsError = true;
        return;
    }

    constexpr size_t kHeaderSize = 21;
    if (buf.size() < kHeaderSize || buf[0] != 'N' || buf[1] != 'E' || buf[2] != 'S' || buf[3] != 'U') {
        m_statusMessage = "Invalid save state file.";
        m_statusIsError = true;
        return;
    }

    if (buf[4] != kSaveStateVersion) {
        m_statusMessage = "Unsupported save state version (create a new state with this build).";
        m_statusIsError = true;
        return;
    }

    const uint64_t savedRomIdentity = readU64(buf.data() + 5);
    if (savedRomIdentity != m_cart.romIdentity()) {
        m_statusMessage = "Save state belongs to a different ROM.";
        m_statusIsError = true;
        return;
    }

    const uint32_t payloadSize = readU32(buf.data() + 13);
    const uint32_t savedChecksum = readU32(buf.data() + 17);
    if (payloadSize != buf.size() - kHeaderSize) {
        m_statusMessage = "Save state is truncated or has an invalid size.";
        m_statusIsError = true;
        return;
    }

    const uint8_t* payload = buf.data() + kHeaderSize;
    if (stateChecksum(payload, payloadSize) != savedChecksum) {
        m_statusMessage = "Save state checksum failed (file is corrupt).";
        m_statusIsError = true;
        return;
    }

    const uint8_t* p = payload;
    const uint8_t* end = payload + payloadSize;
    if (!m_cpu.loadState(p, end) || !m_ppu.loadState(p, end) ||
        !m_bus.loadState(p, end) || !m_apu.loadState(p, end) ||
        !m_cart.loadState(p, end) || p != end) {
        m_statusMessage = "Save state load failed (corrupt or wrong ROM).";
        m_statusIsError = true;
        return;
    }
    m_skipBreakpointOnce = false;
    m_statusMessage = "State loaded ← slot " + std::to_string(m_saveSlot);
    m_statusIsError = false;
}

void Frontend::toggleFullscreen()
{
    m_fullscreen = !m_fullscreen;
    SDL_SetWindowFullscreen(m_window, m_fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
    m_statusMessage = m_fullscreen ? "Fullscreen ON" : "Fullscreen OFF";
    m_statusIsError = false;
}
































