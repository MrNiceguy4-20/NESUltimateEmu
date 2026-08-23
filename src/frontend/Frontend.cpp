#include "Frontend.hpp"
#include <algorithm>
#include <fstream>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cctype>
#include <filesystem>
#include <sstream>
#include <limits>
#include <cmath>
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

#ifdef _WIN32
namespace {
std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool endsWithI(const std::string& value, const char* suffix)
{
    const std::string a = lowerCopy(value);
    const std::string b = lowerCopy(suffix);
    return a.size() >= b.size() && a.compare(a.size() - b.size(), b.size(), b) == 0;
}

bool isArchivePath(const std::string& path)
{
    static const char* kArchiveExtensions[] = {
        ".zip", ".7z", ".rar", ".tar", ".tgz", ".tar.gz", ".gz",
        ".tbz", ".tbz2", ".tar.bz2", ".bz2", ".txz", ".tar.xz", ".xz"
    };
    for (const char* ext : kArchiveExtensions)
        if (endsWithI(path, ext)) return true;
    return false;
}

bool hasNesExtension(const std::string& path)
{
    return endsWithI(path, ".nes");
}

bool hasFdsExtension(const std::string& path)
{
    return endsWithI(path, ".fds");
}

bool hasNesSignature(const std::vector<uint8_t>& data)
{
    return data.size() >= 16 && data[0] == 'N' && data[1] == 'E' &&
        data[2] == 'S' && data[3] == 0x1A;
}

bool hasHeaderedFdsSignature(const std::vector<uint8_t>& data)
{
    return data.size() >= 16 && data[0] == 'F' && data[1] == 'D' &&
        data[2] == 'S' && data[3] == 0x1A;
}

std::string quoteWindowsArg(const std::string& arg)
{
    std::string out = "\"";
    unsigned slashes = 0;
    for (char ch : arg) {
        if (ch == '\\') { ++slashes; continue; }
        if (ch == '"') {
            out.append(slashes * 2 + 1, '\\');
            out.push_back('"');
            slashes = 0;
            continue;
        }
        out.append(slashes, '\\');
        slashes = 0;
        out.push_back(ch);
    }
    out.append(slashes * 2, '\\');
    out.push_back('"');
    return out;
}

bool runTarCapture(const std::vector<std::string>& args, std::vector<uint8_t>& output,
    std::size_t maxBytes, DWORD& exitCode)
{
    output.clear();
    exitCode = ERROR_GEN_FAILURE;

    SECURITY_ATTRIBUTES sa{};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    HANDLE readPipe = nullptr;
    HANDLE writePipe = nullptr;
    if (!CreatePipe(&readPipe, &writePipe, &sa, 0)) return false;
    SetHandleInformation(readPipe, HANDLE_FLAG_INHERIT, 0);

    HANDLE nullErr = CreateFileA("NUL", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
        &sa, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput = writePipe;
    si.hStdError = (nullErr != INVALID_HANDLE_VALUE) ? nullErr : writePipe;

    PROCESS_INFORMATION pi{};
    std::string command = "tar.exe";
    for (const auto& arg : args) {
        command.push_back(' ');
        command += quoteWindowsArg(arg);
    }
    std::vector<char> mutableCommand(command.begin(), command.end());
    mutableCommand.push_back('\0');

    const BOOL created = CreateProcessA(nullptr, mutableCommand.data(), nullptr, nullptr, TRUE,
        CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    CloseHandle(writePipe);
    if (nullErr != INVALID_HANDLE_VALUE) CloseHandle(nullErr);
    if (!created) { CloseHandle(readPipe); return false; }

    bool withinLimit = true;
    uint8_t buffer[16384];
    for (;;) {
        DWORD got = 0;
        if (!ReadFile(readPipe, buffer, sizeof(buffer), &got, nullptr) || got == 0) break;
        if (output.size() + got > maxBytes) {
            withinLimit = false;
            TerminateProcess(pi.hProcess, ERROR_FILE_TOO_LARGE);
            break;
        }
        output.insert(output.end(), buffer, buffer + got);
    }
    CloseHandle(readPipe);
    WaitForSingleObject(pi.hProcess, INFINITE);
    GetExitCodeProcess(pi.hProcess, &exitCode);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return withinLimit && exitCode == 0;
}

bool listArchiveEntries(const std::string& archive, std::vector<std::string>& entries)
{
    std::vector<uint8_t> text;
    DWORD code = 0;
    if (!runTarCapture({ "-tf", archive }, text, 4u * 1024u * 1024u, code)) return false;
    std::string listing(text.begin(), text.end());
    std::istringstream lines(listing);
    std::string line;
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line.back() == '/' || line.back() == '\\') continue;
        entries.push_back(line);
    }
    return !entries.empty();
}

bool extractArchiveEntry(const std::string& archive, const std::string& entry,
    std::vector<uint8_t>& data)
{
    DWORD code = 0;

    return runTarCapture({ "-xOf", archive, "--", entry }, data,
        512u * 1024u * 1024u, code);
}
}
#endif

Frontend::Frontend(SDL_Window* window, SDL_Renderer* renderer,
    CPU& cpu, Bus& bus, Cartridge& cart, PPU& ppu, APU& apu)
    : m_window(window)
    , m_renderer(renderer)
    , m_cpu(cpu)
    , m_bus(bus)
    , m_cart(cart)
    , m_ppu(ppu)
    , m_apu(apu)
    , m_running(true)
    , m_statusMessage("No ROM loaded. Click \"Load ROM...\" to choose a ROM or archive.")
{
    m_nesTexture = SDL_CreateTexture(
        m_renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        256, 240
    );
    SDL_SetTextureScaleMode(m_nesTexture, SDL_ScaleModeNearest);
    setDefaultBindings();
    loadFrontendConfig();
    m_apu.setMasterVolume(m_masterVolume);
    m_apu.setHostAudioEnabled(m_emulationSpeed == EmulationSpeed::Normal);
    m_apu.setAudioPlaybackPaused(true);
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
#ifdef _WIN32
    if (m_systemSleepSuppressed)
        SetThreadExecutionState(ES_CONTINUOUS);
#endif
    saveFrontendConfig();
    m_cart.saveBattery();
    if (m_gamepad) {
        SDL_GameControllerClose(m_gamepad);
        m_gamepad = nullptr;
    }
    if (m_nesTexture)
        SDL_DestroyTexture(m_nesTexture);
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

SDL_Rect Frontend::gameDestinationRect() const
{
    int winW = 0, winH = 0;
    SDL_GetWindowSize(m_window, &winW, &winH);

    if (m_fullscreen) {
        const double displayW = m_ntscAspect ? (256.0 * 8.0 / 7.0) : 256.0;
        const double displayH = 240.0;
        const double aspect = displayW / displayH;

        int dstW = winW;
        int dstH = static_cast<int>(dstW / aspect + 0.5);
        if (dstH > winH) {
            dstH = winH;
            dstW = static_cast<int>(dstH * aspect + 0.5);
        }
        return SDL_Rect{ (winW - dstW) / 2, (winH - dstH) / 2, dstW, dstH };
    }

    int dstW = 256 * m_scale;
    const int dstH = 240 * m_scale;
    if (m_ntscAspect)
        dstW = (256 * m_scale * 8) / 7;
    return SDL_Rect{ (winW - dstW) / 2, (winH - dstH) / 2, dstW, dstH };
}

void Frontend::runFrame()
{
    if (!m_cart.isLoaded()) return;

    constexpr int kMaxCycles = 100000;
    int guard = 0;
    m_ppu.clearFrameComplete();
    while (!m_ppu.frameComplete() && guard < kMaxCycles) {
        m_bus.clock();
        ++guard;
    }
}

void Frontend::stepInstruction()
{
    if (!m_cart.isLoaded()) return;
    m_paused = true;

    const uint64_t startInstruction = m_cpu.instructionCount();
    int guard = 0;
    constexpr int kMaxStepCycles = 100000;
    do {
        m_bus.clock();
        ++guard;
    } while (guard < kMaxStepCycles &&
        (m_cpu.instructionCount() == startInstruction || !m_cpu.atInstructionBoundary()));

    if (guard >= kMaxStepCycles) {
        m_statusMessage = "Instruction step did not reach a boundary (CPU may be JAMmed).";
        m_statusIsError = true;
    }
    updateTexture();
}

void Frontend::run()
{
    resetPacing();

    while (m_running) {
        processEvents();
        updateControllers();
        updateSystemSleepSuppression();

        const bool activelyEmulating = m_cart.isLoaded() && !m_paused;
        const bool rewinding = activelyEmulating && rewindHeld();
        if (!activelyEmulating || rewinding || m_emulationSpeed != EmulationSpeed::Normal)
            m_apu.setAudioPlaybackPaused(true);

        if (activelyEmulating) {
            if (rewinding) {

                rewindStep();
            } else {
                const int frameBatch = framesPerPresentation();
                for (int i = 0; i < frameBatch; ++i) {
                    runFrame();
                    captureRewindState();
                }
            }
            updateTexture();
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        drawUI();
        ImGui::Render();

        if (m_fullscreen)
            SDL_SetRenderDrawColor(m_renderer, 0, 0, 0, 255);
        else
            SDL_SetRenderDrawColor(m_renderer, 20, 20, 30, 255);
        SDL_RenderClear(m_renderer);

        if (m_cart.isLoaded() && m_nesTexture) {
            const SDL_Rect dst = gameDestinationRect();
            SDL_RenderCopy(m_renderer, m_nesTexture, nullptr, &dst);
        }

        if (!m_fullscreen)
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), m_renderer);
        SDL_RenderPresent(m_renderer);

        if (!m_paused && m_cart.isLoaded())
            pacePresentation(!rewinding);
        else
            resetPacing();
    }

#ifdef _WIN32
    if (m_systemSleepSuppressed) {
        SetThreadExecutionState(ES_CONTINUOUS);
        m_systemSleepSuppressed = false;
    }
#endif
    m_apu.setAudioPlaybackPaused(true);
}

void Frontend::updateSystemSleepSuppression()
{
#ifdef _WIN32
    const bool shouldSuppress = m_cart.isLoaded() && !m_paused;
    if (shouldSuppress == m_systemSleepSuppressed)
        return;

    if (shouldSuppress) {
        const EXECUTION_STATE result = SetThreadExecutionState(
            ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED);
        if (result != 0)
            m_systemSleepSuppressed = true;
    }
    else {
        SetThreadExecutionState(ES_CONTINUOUS);
        m_systemSleepSuppressed = false;
    }
#endif
}

int Frontend::framesPerPresentation() const
{
    switch (m_emulationSpeed) {

    case EmulationSpeed::Double: return 4;
    case EmulationSpeed::Quadruple: return 8;
    case EmulationSpeed::Uncapped: return 16;
    case EmulationSpeed::Normal:
    default: return 1;
    }
}

double Frontend::emulationSpeedMultiplier() const
{
    switch (m_emulationSpeed) {
    case EmulationSpeed::Double: return 2.0;
    case EmulationSpeed::Quadruple: return 4.0;
    case EmulationSpeed::Uncapped: return 0.0;
    case EmulationSpeed::Normal:
    default: return 1.0;
    }
}

const char* Frontend::emulationSpeedName() const
{
    switch (m_emulationSpeed) {
    case EmulationSpeed::Double: return "2x";
    case EmulationSpeed::Quadruple: return "4x";
    case EmulationSpeed::Uncapped: return "Uncapped";
    case EmulationSpeed::Normal:
    default: return "1x";
    }
}

void Frontend::setEmulationSpeed(EmulationSpeed speed)
{
    if (m_emulationSpeed == speed) return;
    m_emulationSpeed = speed;

    m_apu.setHostAudioEnabled(speed == EmulationSpeed::Normal);
    m_apu.setAudioPlaybackPaused(true);
    resetPacing();
    m_statusMessage = std::string("Emulation speed: ") + emulationSpeedName();
    m_statusIsError = false;
}

void Frontend::resetPacing()
{
    m_pacingDeadline = 0;
    m_audioPacingPrimed = false;
}

void Frontend::pacePresentation(bool allowAudioSync)
{
    if (m_emulationSpeed == EmulationSpeed::Uncapped)
        return;

    const uint64_t freq = SDL_GetPerformanceFrequency();
    const uint64_t now = SDL_GetPerformanceCounter();
    const double frameSeconds = 1.0 / consoleFrameRate(m_bus.timing());
    const double speed = std::max(1.0, emulationSpeedMultiplier());
    const double presentationSeconds = frameSeconds * double(framesPerPresentation()) / speed;

    if (m_pacingDeadline == 0)
        m_pacingDeadline = now + uint64_t(presentationSeconds * double(freq));
    else
        m_pacingDeadline += uint64_t(presentationSeconds * double(freq));

    if (allowAudioSync && m_emulationSpeed == EmulationSpeed::Normal && m_apu.audioOpen()) {
        const size_t queued = m_apu.queuedAudioSamples();
        const int sampleRate = std::max(1, m_apu.outputSampleRate());
        constexpr size_t targetSamples = 1536;
        constexpr size_t primeSamples = 1024;

        if (!m_audioPacingPrimed) {
            if (queued < primeSamples) {
                m_pacingDeadline = SDL_GetPerformanceCounter();
                return;
            }
            m_audioPacingPrimed = true;
            m_apu.setAudioPlaybackPaused(false);
            m_pacingDeadline = SDL_GetPerformanceCounter() + uint64_t(presentationSeconds * double(freq));
        }

        const double errorSeconds = (double(queued) - double(targetSamples)) / double(sampleRate);
        const double correction = std::clamp(errorSeconds * 0.12, -0.0015, 0.0030);
        const int64_t correctionTicks = static_cast<int64_t>(correction * double(freq));
        if (correctionTicks >= 0)
            m_pacingDeadline += static_cast<uint64_t>(correctionTicks);
        else {
            const uint64_t magnitude = static_cast<uint64_t>(-correctionTicks);
            m_pacingDeadline = magnitude < m_pacingDeadline ? m_pacingDeadline - magnitude : 0;
        }
    }

    uint64_t current = SDL_GetPerformanceCounter();
    const uint64_t maxLag = uint64_t(presentationSeconds * 4.0 * double(freq));
    if (current > m_pacingDeadline + maxLag) {

        m_pacingDeadline = current;
        return;
    }

    while (current < m_pacingDeadline) {
        const double remainingMs = double(m_pacingDeadline - current) * 1000.0 / double(freq);
        if (remainingMs > 2.0)
            SDL_Delay(static_cast<Uint32>(remainingMs - 1.0));
        else
            SDL_Delay(0);
        current = SDL_GetPerformanceCounter();
    }
}

bool Frontend::rewindHeld() const
{
    if (!m_cart.isLoaded()) return false;
    const SDL_Keycode key = m_hotkeys[static_cast<int>(HotkeyAction::Rewind)];
    if (key == SDLK_UNKNOWN) return false;
    const SDL_Scancode sc = SDL_GetScancodeFromKey(key);
    if (sc == SDL_SCANCODE_UNKNOWN) return false;
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    return keys && keys[sc] != 0 && !ImGui::GetIO().WantCaptureKeyboard;
}

std::vector<uint8_t> Frontend::captureMachinePayload() const
{
    std::vector<uint8_t> payload;
    m_cpu.saveState(payload);
    m_ppu.saveState(payload);
    m_bus.saveState(payload);
    m_apu.saveState(payload);
    m_cart.saveState(payload);
    return payload;
}

bool Frontend::loadMachinePayload(const std::vector<uint8_t>& payload)
{
    if (payload.empty()) return false;
    const uint8_t* cursor = payload.data();
    const uint8_t* finish = payload.data() + payload.size();
    return m_cpu.loadState(cursor, finish) && m_ppu.loadState(cursor, finish) &&
        m_bus.loadState(cursor, finish) && m_apu.loadState(cursor, finish) &&
        m_cart.loadState(cursor, finish) && cursor == finish;
}

void Frontend::clearRewindHistory()
{
    m_rewindStates.clear();
    m_rewindBytes = 0;
    m_rewindFrameCounter = 0;
    m_rewindRomIdentity = m_cart.isLoaded() ? m_cart.romIdentity() : 0;
}

void Frontend::captureRewindState()
{
    if (!m_cart.isLoaded()) return;
    if (m_rewindRomIdentity != m_cart.romIdentity())
        clearRewindHistory();

    if ((++m_rewindFrameCounter & 1u) != 0)
        return;

    auto payload = captureMachinePayload();
    m_rewindBytes += payload.size();
    m_rewindStates.emplace_back(std::move(payload));

    while (!m_rewindStates.empty() &&
        (m_rewindStates.size() > kMaxRewindSnapshots || m_rewindBytes > kMaxRewindBytes)) {
        m_rewindBytes -= m_rewindStates.front().size();
        m_rewindStates.pop_front();
    }
}

bool Frontend::rewindStep()
{
    if (!m_cart.isLoaded()) return false;
    if (m_rewindRomIdentity != m_cart.romIdentity()) {
        clearRewindHistory();
        return false;
    }
    if (m_rewindStates.empty())
        return false;

    std::vector<uint8_t> payload = std::move(m_rewindStates.back());
    m_rewindBytes -= payload.size();
    m_rewindStates.pop_back();
    if (!loadMachinePayload(payload)) {
        clearRewindHistory();
        m_statusMessage = "Rewind state rejected; rewind history cleared.";
        m_statusIsError = true;
        return false;
    }

    const bool wantHostAudio = m_emulationSpeed == EmulationSpeed::Normal;
    m_apu.setHostAudioEnabled(false);
    if (wantHostAudio) m_apu.setHostAudioEnabled(true);
    m_apu.setAudioPlaybackPaused(true);
    resetPacing();
    m_statusMessage = "Rewinding...";
    m_statusIsError = false;
    return true;
}

void Frontend::setDefaultBindings()
{
    using NB = NesButton;
    m_p1Keys = { SDL_SCANCODE_Z, SDL_SCANCODE_X, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_RETURN,
        SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT };
    m_p2Keys = { SDL_SCANCODE_H, SDL_SCANCODE_G, SDL_SCANCODE_T, SDL_SCANCODE_Y,
        SDL_SCANCODE_I, SDL_SCANCODE_K, SDL_SCANCODE_J, SDL_SCANCODE_L };
    m_p1PadButtons = { SDL_CONTROLLER_BUTTON_A, SDL_CONTROLLER_BUTTON_B, SDL_CONTROLLER_BUTTON_BACK,
        SDL_CONTROLLER_BUTTON_START, SDL_CONTROLLER_BUTTON_DPAD_UP, SDL_CONTROLLER_BUTTON_DPAD_DOWN,
        SDL_CONTROLLER_BUTTON_DPAD_LEFT, SDL_CONTROLLER_BUTTON_DPAD_RIGHT };
    m_hotkeys = { SDLK_p, SDLK_F10, SDLK_r, SDLK_1, SDLK_2, SDLK_3, SDLK_4,
        SDLK_F5, SDLK_F9, SDLK_F6, SDLK_F7, SDLK_a, SDLK_c, SDLK_F11, SDLK_COMMA, SDLK_PERIOD, SDLK_ESCAPE, SDLK_BACKSPACE };
    m_masterVolume = 1.50f;
}

void Frontend::loadFrontendConfig()
{
    std::ifstream in("nesultimate.cfg");
    if (!in) return;

    std::string key;
    int index = 0;
    int value = 0;
    int configVersion = 1;
    while (in >> key) {
        if (key == "config_version") {
            in >> configVersion;
        } else if (key == "volume") {
            in >> m_masterVolume;
            m_masterVolume = std::clamp(m_masterVolume, 0.0f, 2.5f);
        } else if (key == "timing_override") {
            int timing = 0;
            in >> timing;
            if (timing >= int(TimingOverride::Auto) && timing <= int(TimingOverride::Dendy))
                m_timingOverride = static_cast<TimingOverride>(timing);
        } else if (key == "emulation_speed") {
            int speed = 0;
            in >> speed;
            if (speed >= int(EmulationSpeed::Normal) && speed <= int(EmulationSpeed::Uncapped))
                m_emulationSpeed = static_cast<EmulationSpeed>(speed);
        } else if ((key == "p1key" || key == "p2key" || key == "p1pad" || key == "hotkey") && (in >> index >> value)) {
            if (key == "p1key" && index >= 0 && index < int(m_p1Keys.size()))
                m_p1Keys[index] = static_cast<SDL_Scancode>(value);
            else if (key == "p2key" && index >= 0 && index < int(m_p2Keys.size()))
                m_p2Keys[index] = static_cast<SDL_Scancode>(value);
            else if (key == "p1pad" && index >= 0 && index < int(m_p1PadButtons.size()))
                m_p1PadButtons[index] = static_cast<SDL_GameControllerButton>(value);
            else if (key == "hotkey" && configVersion >= 2) {
                if (configVersion >= 4 && index >= 0 && index < int(m_hotkeys.size()))
                    m_hotkeys[index] = static_cast<SDL_Keycode>(value);
                else if (configVersion < 4 && index >= 0 && index < 14)
                    m_hotkeys[index] = static_cast<SDL_Keycode>(value);
                else if (configVersion < 4 && index == 14)
                    m_hotkeys[static_cast<int>(HotkeyAction::Quit)] = static_cast<SDL_Keycode>(value);
            }
        } else {
            std::string ignored;
            std::getline(in, ignored);
        }
    }
}

void Frontend::saveFrontendConfig() const
{
    std::ofstream out("nesultimate.cfg", std::ios::out | std::ios::trunc);
    if (!out) return;
    out << "config_version 4\n";
    out << "emulation_speed " << int(m_emulationSpeed) << '\n';
    out << "volume " << m_masterVolume << '\n';
    out << "timing_override " << int(m_timingOverride) << '\n';
    for (int i = 0; i < int(m_p1Keys.size()); ++i) out << "p1key " << i << ' ' << int(m_p1Keys[i]) << '\n';
    for (int i = 0; i < int(m_p2Keys.size()); ++i) out << "p2key " << i << ' ' << int(m_p2Keys[i]) << '\n';
    for (int i = 0; i < int(m_p1PadButtons.size()); ++i) out << "p1pad " << i << ' ' << int(m_p1PadButtons[i]) << '\n';
    for (int i = 0; i < int(m_hotkeys.size()); ++i) out << "hotkey " << i << ' ' << int(m_hotkeys[i]) << '\n';
}

ConsoleTiming Frontend::effectiveTiming() const
{
    switch (m_timingOverride) {
    case TimingOverride::NTSC:  return ConsoleTiming::NTSC;
    case TimingOverride::PAL:   return ConsoleTiming::PAL;
    case TimingOverride::Dendy: return ConsoleTiming::Dendy;
    case TimingOverride::Auto:
    default:                    return m_cart.timing();
    }
}

void Frontend::applyTimingOverride(bool resetSystem)
{
    if (!m_cart.isLoaded()) return;
    m_bus.setTiming(effectiveTiming());
    if (resetSystem) m_bus.powerOn();
}

bool Frontend::handleBindingCapture(const SDL_Event& e)
{
    if (m_captureTarget == CaptureTarget::None) return false;

    if (e.type == SDL_KEYDOWN) {
        if (e.key.keysym.sym == SDLK_ESCAPE) {
            m_captureTarget = CaptureTarget::None;
            m_captureIndex = -1;
            return true;
        }
        if (m_captureIndex >= 0) {
            if (m_captureTarget == CaptureTarget::P1Keyboard)
                m_p1Keys[m_captureIndex] = e.key.keysym.scancode;
            else if (m_captureTarget == CaptureTarget::P2Keyboard)
                m_p2Keys[m_captureIndex] = e.key.keysym.scancode;
            else if (m_captureTarget == CaptureTarget::Hotkey)
                m_hotkeys[m_captureIndex] = e.key.keysym.sym;
            else
                return false;
            saveFrontendConfig();
            m_captureTarget = CaptureTarget::None;
            m_captureIndex = -1;
            return true;
        }
    }

    if (e.type == SDL_CONTROLLERBUTTONDOWN && m_captureTarget == CaptureTarget::P1Gamepad && m_captureIndex >= 0) {
        m_p1PadButtons[m_captureIndex] = static_cast<SDL_GameControllerButton>(e.cbutton.button);
        saveFrontendConfig();
        m_captureTarget = CaptureTarget::None;
        m_captureIndex = -1;
        return true;
    }
    return false;
}

bool Frontend::handleHotkey(SDL_Keycode key)
{
    auto match = [&](HotkeyAction action) { return key == m_hotkeys[static_cast<int>(action)]; };
    if (match(HotkeyAction::Quit)) { m_running = false; return true; }
    if (match(HotkeyAction::Pause)) { m_paused = !m_paused; return true; }
    if (match(HotkeyAction::StepInstruction)) { if (m_cart.isLoaded()) stepInstruction(); return true; }
    if (match(HotkeyAction::Reset)) {
        if (m_cart.isLoaded()) { m_bus.reset(); clearRewindHistory(); m_statusMessage = "System reset."; m_statusIsError = false; }
        return true;
    }
    if (match(HotkeyAction::Scale1)) { m_scale = 1; return true; }
    if (match(HotkeyAction::Scale2)) { m_scale = 2; return true; }
    if (match(HotkeyAction::Scale3)) { m_scale = 3; return true; }
    if (match(HotkeyAction::Scale4)) { m_scale = 4; return true; }
    if (match(HotkeyAction::SaveState)) { saveState(); return true; }
    if (match(HotkeyAction::LoadState)) { loadState(); return true; }
    if (match(HotkeyAction::SlotNext)) { m_saveSlot = (m_saveSlot + 1) % 10; m_statusMessage = "Save slot: " + std::to_string(m_saveSlot); m_statusIsError = false; return true; }
    if (match(HotkeyAction::SlotPrevious)) { m_saveSlot = (m_saveSlot + 9) % 10; m_statusMessage = "Save slot: " + std::to_string(m_saveSlot); m_statusIsError = false; return true; }
    if (match(HotkeyAction::Aspect)) { m_ntscAspect = !m_ntscAspect; m_statusMessage = m_ntscAspect ? "NTSC aspect ON (8:7)" : "Square pixels"; m_statusIsError = false; return true; }
    if (match(HotkeyAction::ChipMod)) { m_apu.setChipMod(!m_apu.chipMod()); m_statusMessage = m_apu.chipMod() ? "Chip Mod ON (KYLXBN-style 2A03/VRC6/VRC7/N163)" : "Chip Mod OFF (accurate nonlinear mix)"; m_statusIsError = false; return true; }
    if (match(HotkeyAction::Fullscreen)) { toggleFullscreen(); return true; }
    if (match(HotkeyAction::SpeedPrevious)) {
        int speed = std::max(0, int(m_emulationSpeed) - 1);
        setEmulationSpeed(static_cast<EmulationSpeed>(speed));
        saveFrontendConfig();
        return true;
    }
    if (match(HotkeyAction::SpeedNext)) {
        int speed = std::min(int(EmulationSpeed::Uncapped), int(m_emulationSpeed) + 1);
        setEmulationSpeed(static_cast<EmulationSpeed>(speed));
        saveFrontendConfig();
        return true;
    }
    if (match(HotkeyAction::Rewind)) {

        if (m_cart.isLoaded() && rewindStep()) updateTexture();
        return true;
    }
    return false;
}

void Frontend::updateControllers()
{
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    auto pressed = [&](const std::array<SDL_Scancode, 8>& map, NesButton button) {
        const SDL_Scancode sc = map[static_cast<int>(button)];
        return sc != SDL_SCANCODE_UNKNOWN && keys[sc] != 0;
    };
    auto pack = [](bool a, bool b, bool sel, bool st, bool u, bool d, bool l, bool r) -> uint8_t {
        uint8_t v = 0;
        if (a) v |= 0x01; if (b) v |= 0x02; if (sel) v |= 0x04; if (st) v |= 0x08;
        if (u) v |= 0x10; if (d) v |= 0x20; if (l) v |= 0x40; if (r) v |= 0x80;
        return v;
    };

    bool p1[8]{};
    bool p2[8]{};
    for (int i = 0; i < 8; ++i) {
        p1[i] = pressed(m_p1Keys, static_cast<NesButton>(i));
        p2[i] = pressed(m_p2Keys, static_cast<NesButton>(i));
    }

    if (m_gamepad) {
        for (int i = 0; i < 8; ++i)
            p1[i] = p1[i] || SDL_GameControllerGetButton(m_gamepad, m_p1PadButtons[i]);
        Sint16 lx = SDL_GameControllerGetAxis(m_gamepad, SDL_CONTROLLER_AXIS_LEFTX);
        Sint16 ly = SDL_GameControllerGetAxis(m_gamepad, SDL_CONTROLLER_AXIS_LEFTY);
        const Sint16 dead = 16000;
        if (ly < -dead) p1[4] = true;
        if (ly > dead) p1[5] = true;
        if (lx < -dead) p1[6] = true;
        if (lx > dead) p1[7] = true;
    }

    m_bus.setController1(pack(p1[0], p1[1], p1[2], p1[3], p1[4], p1[5], p1[6], p1[7]));
    m_bus.setController2(pack(p2[0], p2[1], p2[2], p2[3], p2[4], p2[5], p2[6], p2[7]));
}

void Frontend::processEvents()
{
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        ImGui_ImplSDL2_ProcessEvent(&e);
        if (e.type == SDL_QUIT) { m_running = false; continue; }
        if (e.type == SDL_CONTROLLERDEVICEADDED && !m_gamepad) openGamepad();
        if (e.type == SDL_CONTROLLERDEVICEREMOVED && m_gamepad) {
            SDL_GameControllerClose(m_gamepad);
            m_gamepad = nullptr;
        }
        if (handleBindingCapture(e)) continue;
        if (e.type == SDL_KEYDOWN && !e.key.repeat) {
            if (ImGui::GetIO().WantCaptureKeyboard) continue;
            handleHotkey(e.key.keysym.sym);
        }
    }
}

void Frontend::drawUI()
{
    ImGui::Begin("NES Ultimate Emulator", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::Button("Load ROM / State..."))
        ImGui::OpenPopup("LoadSaveMenu");
    if (ImGui::BeginPopup("LoadSaveMenu")) {
        if (ImGui::MenuItem("Load ROM..."))
            openRomDialog();
        if (ImGui::MenuItem("Load State...", nullptr, false, m_cart.isLoaded()))
            openStateLoadDialog();
        if (ImGui::MenuItem("Save State...", nullptr, false, m_cart.isLoaded()))
            openStateSaveDialog();
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset") && m_cart.isLoaded()) {
        m_bus.reset();
        clearRewindHistory();
        m_statusMessage = "System reset.";
        m_statusIsError = false;
    }
    ImGui::SameLine();
    if (ImGui::Button(m_paused ? "Resume" : "Pause"))
        m_paused = !m_paused;

    ImGui::SameLine();
    if (ImGui::Button("Rewind") && m_cart.isLoaded()) {
        if (rewindStep()) updateTexture();
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!m_cart.isLoaded());
    if (ImGui::Button("Cheats"))
        ImGui::OpenPopup("CheatsMenu");
    ImGui::EndDisabled();
    if (ImGui::BeginPopup("CheatsMenu")) {
        auto& cheatSystem = m_cart.cheats();
        auto& cheats = cheatSystem.entries();
        if (cheats.empty()) {
            ImGui::TextDisabled("No cheats found for this game.");
            ImGui::TextDisabled("Expected database: cheats/nes/*.cht");
        } else {
            ImGui::Text("%s", cheatSystem.gameTitle().empty() ? m_cart.fileName().c_str() : cheatSystem.gameTitle().c_str());
            ImGui::TextDisabled("%zu available, %zu enabled", cheats.size(), cheatSystem.enabledCount());
            ImGui::Separator();
            for (std::size_t i = 0; i < cheats.size(); ++i) {
                const CheatEntry& cheat = cheats[i];
                std::string label = cheat.description + "##cheat" + std::to_string(i);
                bool enabled = cheat.enabled;
                if (ImGui::Checkbox(label.c_str(), &enabled))
                    cheatSystem.setEntryEnabled(i, enabled);
                if (ImGui::IsItemHovered()) {
                    std::string codes;
                    for (std::size_t c = 0; c < cheat.codes.size(); ++c) {
                        if (c) codes += " + ";
                        codes += cheat.codes[c].text;
                    }
                    ImGui::SetTooltip("%s", codes.c_str());
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Disable All"))
                cheatSystem.disableAll();
        }
        if (ImGui::MenuItem("Reload Game Database")) {
            reloadCheatsForLoadedRom();
            m_statusMessage = m_cart.cheats().entries().empty()
                ? "No cheat database entry found for " + m_cart.fileName() + "."
                : "Reloaded " + std::to_string(m_cart.cheats().entries().size()) + " cheats for " + m_cart.fileName() + ".";
            m_statusIsError = m_cart.cheats().entries().empty();
        }
        if (ImGui::MenuItem("Load Cheat File..."))
            openCheatFileDialog();
        if (!cheatSystem.databasePath().empty()) {
            ImGui::Separator();
            ImGui::TextDisabled("Source: %s", cheatSystem.databasePath().c_str());
        }
        ImGui::EndPopup();
    }

    if (ImGui::Button("Settings"))
        ImGui::OpenPopup("SettingsMenu");
    if (ImGui::BeginPopup("SettingsMenu")) {
        if (ImGui::MenuItem("Controller..."))
            m_controllerConfigOpen = true;
        if (ImGui::MenuItem("Hotkeys..."))
            m_settingsOpen = true;
        if (ImGui::BeginMenu("Console Timing")) {
            struct TimingChoice { const char* label; TimingOverride value; };
            static constexpr TimingChoice choices[] = {
                { "Auto (ROM header/database)", TimingOverride::Auto },
                { "NTSC", TimingOverride::NTSC },
                { "PAL", TimingOverride::PAL },
                { "Dendy", TimingOverride::Dendy },
            };
            for (const auto& choice : choices) {
                if (ImGui::MenuItem(choice.label, nullptr, m_timingOverride == choice.value)) {
                    m_timingOverride = choice.value;
                    saveFrontendConfig();
                    if (m_cart.isLoaded()) {
                        applyTimingOverride(true);
                        m_paused = false;
                        m_statusMessage = std::string("Console timing: ") + consoleTimingName(m_bus.timing()) +
                            (m_timingOverride == TimingOverride::Auto ? " (auto)." : " (forced override). System restarted.");
                        m_statusIsError = false;
                    }
                }
            }
            ImGui::EndMenu();
        }
        ImGui::Separator();
        char fullscreenLabel[96];
        std::snprintf(fullscreenLabel, sizeof(fullscreenLabel), "Borderless Fullscreen    [%s]",
            SDL_GetKeyName(m_hotkeys[static_cast<int>(HotkeyAction::Fullscreen)]));
        if (ImGui::MenuItem(fullscreenLabel, nullptr, m_fullscreen))
            toggleFullscreen();
        ImGui::EndPopup();
    }

    ImGui::SameLine();
    ImGui::TextUnformatted("Speed");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(95.0f);
    int speedIndex = int(m_emulationSpeed);
    const char* speedItems[] = { "1x", "2x", "4x", "Uncapped" };
    if (ImGui::Combo("##EmulationSpeed", &speedIndex, speedItems, 4)) {
        setEmulationSpeed(static_cast<EmulationSpeed>(speedIndex));
        saveFrontendConfig();
    }

    ImGui::SameLine();
    ImGui::TextUnformatted("Volume");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(155.0f);
    float mainVolumePercent = m_masterVolume * 100.0f;
    if (ImGui::SliderFloat("##MainVolume", &mainVolumePercent, 0.0f, 250.0f, "%.0f%%")) {
        m_masterVolume = mainVolumePercent / 100.0f;
        m_apu.setMasterVolume(m_masterVolume);
    }
    if (ImGui::IsItemDeactivatedAfterEdit())
        saveFrontendConfig();

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
        ImGui::Text("Timing : %s%s", consoleTimingName(m_bus.timing()),
            m_timingOverride == TimingOverride::Auto ? " (auto)" : " (forced)");
        if (m_cart.isMultiRegion() && m_timingOverride == TimingOverride::Auto)
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f),
                "ROM is multi-region; use Settings > Console Timing to choose PAL/Dendy when needed.");
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

        ImGui::Text("Scale  : %dx", m_scale);
        ImGui::Text("Paused : %s", m_paused ? "yes" : "no");
        ImGui::Text("Speed  : %s", emulationSpeedName());
        if (m_apu.audioOpen() && m_emulationSpeed == EmulationSpeed::Normal)
            ImGui::Text("Audio  : %zu queued / %llu underruns / %llu overruns", m_apu.queuedAudioSamples(),
                static_cast<unsigned long long>(m_apu.audioUnderrunCount()),
                static_cast<unsigned long long>(m_apu.audioOverrunCount()));
        ImGui::Text("Slot   : %d", m_saveSlot);
        ImGui::Text("Aspect : %s", m_ntscAspect ? "NTSC 8:7" : "Square");
        ImGui::Text("Volume : %.0f%%", m_masterVolume * 100.0f);
        ImGui::Text("Fullscreen: borderless fit-to-display (F11 by default)");

        ImGui::Separator();
        ImGui::Text("Use Settings > Controller to remap P1/P2 keyboard and P1 gamepad buttons.");
        ImGui::Text("Use Settings > Hotkeys to remap emulator hotkeys, including fullscreen.");

        if (m_apu.chipMod())
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.7f, 1.0f), "Chip Mod: ON");
        else
            ImGui::Text("Chip Mod: OFF");
    }

    ImGui::End();

    drawControllerConfig();
    drawSettings();
}

void Frontend::drawControllerConfig()
{
    if (!m_controllerConfigOpen) return;
    static const char* names[] = { "A", "B", "Select", "Start", "Up", "Down", "Left", "Right" };
    ImGui::Begin("Controller Configuration", &m_controllerConfigOpen, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Click a binding, then press a key or controller button. Escape cancels capture.");
    ImGui::Separator();
    ImGui::Columns(4, "controllerBindings", false);
    ImGui::Text("NES"); ImGui::NextColumn(); ImGui::Text("P1 Keyboard"); ImGui::NextColumn(); ImGui::Text("P1 Gamepad"); ImGui::NextColumn(); ImGui::Text("P2 Keyboard"); ImGui::NextColumn();
    for (int i = 0; i < 8; ++i) {
        ImGui::TextUnformatted(names[i]); ImGui::NextColumn();
        std::string p1 = std::string(SDL_GetScancodeName(m_p1Keys[i])) + "##p1key" + std::to_string(i);
        if (ImGui::Button(p1.c_str(), ImVec2(120, 0))) { m_captureTarget = CaptureTarget::P1Keyboard; m_captureIndex = i; }
        ImGui::NextColumn();
        const char* padName = SDL_GameControllerGetStringForButton(m_p1PadButtons[i]);
        std::string gp = std::string(padName ? padName : "Unknown") + "##p1pad" + std::to_string(i);
        if (ImGui::Button(gp.c_str(), ImVec2(120, 0))) { m_captureTarget = CaptureTarget::P1Gamepad; m_captureIndex = i; }
        ImGui::NextColumn();
        std::string p2 = std::string(SDL_GetScancodeName(m_p2Keys[i])) + "##p2key" + std::to_string(i);
        if (ImGui::Button(p2.c_str(), ImVec2(120, 0))) { m_captureTarget = CaptureTarget::P2Keyboard; m_captureIndex = i; }
        ImGui::NextColumn();
    }
    ImGui::Columns(1);
    if (m_captureTarget == CaptureTarget::P1Keyboard || m_captureTarget == CaptureTarget::P2Keyboard || m_captureTarget == CaptureTarget::P1Gamepad)
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "Waiting for input...");
    if (ImGui::Button("Restore Controller Defaults")) {
        m_p1Keys = { SDL_SCANCODE_Z, SDL_SCANCODE_X, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_RETURN,
            SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT };
        m_p2Keys = { SDL_SCANCODE_H, SDL_SCANCODE_G, SDL_SCANCODE_T, SDL_SCANCODE_Y,
            SDL_SCANCODE_I, SDL_SCANCODE_K, SDL_SCANCODE_J, SDL_SCANCODE_L };
        m_p1PadButtons = { SDL_CONTROLLER_BUTTON_A, SDL_CONTROLLER_BUTTON_B, SDL_CONTROLLER_BUTTON_BACK,
            SDL_CONTROLLER_BUTTON_START, SDL_CONTROLLER_BUTTON_DPAD_UP, SDL_CONTROLLER_BUTTON_DPAD_DOWN,
            SDL_CONTROLLER_BUTTON_DPAD_LEFT, SDL_CONTROLLER_BUTTON_DPAD_RIGHT };
        saveFrontendConfig();
    }
    ImGui::End();
}

void Frontend::drawSettings()
{
    if (!m_settingsOpen) return;
    static const char* names[] = { "Pause / Resume", "Step Instruction", "Reset", "Scale 1x", "Scale 2x", "Scale 3x", "Scale 4x",
        "Save State", "Load State", "Next Save Slot", "Previous Save Slot", "Toggle Aspect", "Chip Mod", "Fullscreen",
        "Speed Down", "Speed Up", "Quit", "Rewind" };
    ImGui::Begin("Settings - Hotkeys", &m_settingsOpen, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Every emulator hotkey can be reassigned. Click one and press the new key.");
    ImGui::Separator();
    for (int i = 0; i < int(m_hotkeys.size()); ++i) {
        ImGui::Text("%-20s", names[i]);
        ImGui::SameLine(190.0f);
        std::string label = std::string(SDL_GetKeyName(m_hotkeys[i])) + "##hotkey" + std::to_string(i);
        if (ImGui::Button(label.c_str(), ImVec2(140, 0))) { m_captureTarget = CaptureTarget::Hotkey; m_captureIndex = i; }
    }
    if (m_captureTarget == CaptureTarget::Hotkey)
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "Waiting for a key...");
    if (ImGui::Button("Restore Hotkey Defaults")) {
        const float volume = m_masterVolume;
        const auto p1 = m_p1Keys; const auto p2 = m_p2Keys; const auto gp = m_p1PadButtons;
        setDefaultBindings(); m_masterVolume = volume; m_p1Keys = p1; m_p2Keys = p2; m_p1PadButtons = gp; saveFrontendConfig();
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
    ofn.lpstrFilter = "NES ROMs / Archives (*.nes;*.nes2;*.fds;*.bin;*.rom;*.zip;*.7z;*.rar;*.tar;*.gz)\0*.nes;*.nes2;*.fds;*.bin;*.rom;*.zip;*.7z;*.rar;*.tar;*.tgz;*.gz;*.bz2;*.xz\0NES / FDS Images (*.nes;*.nes2;*.fds;*.bin;*.rom)\0*.nes;*.nes2;*.fds;*.bin;*.rom\0Archives (*.zip;*.7z;*.rar;*.tar;*.gz)\0*.zip;*.7z;*.rar;*.tar;*.tgz;*.gz;*.bz2;*.xz\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = "Select NES ROM, FDS Image, or Archive";

    if (!GetOpenFileNameA(&ofn))
        return false;

    m_cart.saveBattery();

    if (!isArchivePath(filename) && m_cart.loadFromFile(filename)) {
        applyTimingOverride(true);
        reloadCheatsForLoadedRom();
        m_statusMessage = "Loaded: " + m_cart.fileName();
        m_statusIsError = false;
        m_paused = false;
        return true;
    }

    std::vector<std::string> entries;
    if (listArchiveEntries(filename, entries)) {
        std::vector<std::size_t> order;
        order.reserve(entries.size());
        for (std::size_t i = 0; i < entries.size(); ++i)
            if (hasNesExtension(entries[i])) order.push_back(i);
        for (std::size_t i = 0; i < entries.size(); ++i)
            if (hasFdsExtension(entries[i])) order.push_back(i);

        if (order.empty()) {
            const std::size_t inspectCount = std::min<std::size_t>(entries.size(), 64);
            for (std::size_t i = 0; i < inspectCount; ++i) order.push_back(i);
        }

        for (std::size_t index : order) {
            std::vector<uint8_t> image;
            if (!extractArchiveEntry(filename, entries[index], image)) continue;
            const bool namedMedia = hasNesExtension(entries[index]) || hasFdsExtension(entries[index]);
            if (!namedMedia && !hasNesSignature(image) && !hasHeaderedFdsSignature(image)) continue;
            if (!m_cart.loadFromMemory(image, entries[index], filename)) continue;

            applyTimingOverride(true);
            reloadCheatsForLoadedRom();
            m_statusMessage = "Loaded from archive: " + m_cart.fileName();
            m_statusIsError = false;
            m_paused = false;
            return true;
        }

        m_statusMessage = "Archive opened, but it contains no loadable iNES/NES 2.0 or FDS image.";
        m_statusIsError = true;
        return false;
    }

    std::string failedPath = lowerCopy(filename);
    if (endsWithI(failedPath, ".fds"))
        m_statusMessage = "Failed to load FDS image. Place a valid 8 KiB disksys.rom beside the image/archive.";
    else if (isArchivePath(failedPath))
        m_statusMessage = "Failed to open archive. The Windows archive backend may not support this file or it may be encrypted/corrupt.";
    else
        m_statusMessage = "Failed to load ROM/archive (invalid or unsupported image).";
    m_statusIsError = true;
    return false;
#else
    m_statusMessage = "Native file dialog only on Windows.";
    m_statusIsError = true;
    return false;
#endif
}

void Frontend::reloadCheatsForLoadedRom()
{
    if (!m_cart.isLoaded()) {
        m_cart.cheats().clear();
        return;
    }
    (void)m_cart.cheats().loadForRom(m_cart.path(), m_cart.fileName());
}

bool Frontend::openCheatFileDialog()
{
#ifdef _WIN32
    if (!m_cart.isLoaded()) return false;
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = "NES Cheat Files (*.cht)\0*.cht\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = "Load NES Cheat File";
    if (!GetOpenFileNameA(&ofn)) return false;

    if (!m_cart.cheats().loadFromFile(filename)) {
        m_statusMessage = "Failed to load cheat file: " + m_cart.cheats().lastError();
        m_statusIsError = true;
        return false;
    }
    m_statusMessage = "Loaded " + std::to_string(m_cart.cheats().entries().size()) + " cheats for " + m_cart.fileName() + ".";
    m_statusIsError = false;
    return true;
#else
    m_statusMessage = "Native cheat-file dialog is only available on Windows.";
    m_statusIsError = true;
    return false;
#endif
}

bool Frontend::openStateLoadDialog()
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "Load a ROM before loading a save state.";
        m_statusIsError = true;
        return false;
    }
#ifdef _WIN32
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = "NES Ultimate Save States (*.nesstate;*.state*)\0*.nesstate;*.state*\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = "Load NES Save State";

    if (!GetOpenFileNameA(&ofn))
        return false;
    return loadStateFromPath(filename);
#else
    m_statusMessage = "Native save-state file dialog is only available on Windows.";
    m_statusIsError = true;
    return false;
#endif
}

bool Frontend::openStateSaveDialog()
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "Load a ROM before saving a state.";
        m_statusIsError = true;
        return false;
    }
#ifdef _WIN32
    std::string suggested = m_cart.path() + ".nesstate";
    char filename[MAX_PATH] = {};
    std::snprintf(filename, sizeof(filename), "%s", suggested.c_str());

    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = "NES Ultimate Save State (*.nesstate)\0*.nesstate\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "nesstate";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | OFN_OVERWRITEPROMPT;
    ofn.lpstrTitle = "Save NES State As";

    if (!GetSaveFileNameA(&ofn))
        return false;
    return saveStateToPath(filename);
#else
    m_statusMessage = "Native save-state file dialog is only available on Windows.";
    m_statusIsError = true;
    return false;
#endif
}

std::string Frontend::statePath(int slot) const
{
    return m_cart.path() + ".state" + std::to_string(slot);
}

namespace {
constexpr uint8_t kSaveStateVersion = 74;

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
    saveStateToPath(statePath(m_saveSlot));
}

bool Frontend::saveStateToPath(const std::string& path)
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "No ROM loaded – cannot save state.";
        m_statusIsError = true;
        return false;
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

    std::ofstream f(path, std::ios::binary);
    if (!f) {
        m_statusMessage = "Failed to write save state.";
        m_statusIsError = true;
        return false;
    }
    f.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());
    if (!f) {
        m_statusMessage = "Failed while writing save state.";
        m_statusIsError = true;
        return false;
    }
    m_statusMessage = "State saved: " + path;
    m_statusIsError = false;
    return true;
}

void Frontend::loadState()
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "No ROM loaded – cannot load state.";
        m_statusIsError = true;
        return;
    }
    loadStateFromPath(statePath(m_saveSlot));
}

bool Frontend::loadStateFromPath(const std::string& path)
{
    if (!m_cart.isLoaded()) {
        m_statusMessage = "No ROM loaded – cannot load state.";
        m_statusIsError = true;
        return false;
    }

    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        m_statusMessage = "Could not open save state: " + path;
        m_statusIsError = true;
        return false;
    }
    auto sz = f.tellg();
    if (sz <= 0) {
        m_statusMessage = "Save state file is empty or unreadable.";
        m_statusIsError = true;
        return false;
    }
    f.seekg(0);
    std::vector<uint8_t> buf((size_t)sz);
    f.read(reinterpret_cast<char*>(buf.data()), sz);
    if (!f) {
        m_statusMessage = "Failed to read save state file.";
        m_statusIsError = true;
        return false;
    }

    constexpr size_t kHeaderSize = 21;
    if (buf.size() < kHeaderSize || buf[0] != 'N' || buf[1] != 'E' || buf[2] != 'S' || buf[3] != 'U') {
        m_statusMessage = "Invalid save state file.";
        m_statusIsError = true;
        return false;
    }

    if (buf[4] != kSaveStateVersion) {
        m_statusMessage = "Unsupported save state version (create a new state with this build).";
        m_statusIsError = true;
        return false;
    }

    const uint64_t savedRomIdentity = readU64(buf.data() + 5);
    if (savedRomIdentity != m_cart.romIdentity()) {
        m_statusMessage = "Save state belongs to a different ROM.";
        m_statusIsError = true;
        return false;
    }

    const uint32_t payloadSize = readU32(buf.data() + 13);
    const uint32_t savedChecksum = readU32(buf.data() + 17);
    if (payloadSize != buf.size() - kHeaderSize) {
        m_statusMessage = "Save state is truncated or has an invalid size.";
        m_statusIsError = true;
        return false;
    }

    const uint8_t* payload = buf.data() + kHeaderSize;
    if (stateChecksum(payload, payloadSize) != savedChecksum) {
        m_statusMessage = "Save state checksum failed (file is corrupt).";
        m_statusIsError = true;
        return false;
    }

    std::vector<uint8_t> rollback;
    m_cpu.saveState(rollback);
    m_ppu.saveState(rollback);
    m_bus.saveState(rollback);
    m_apu.saveState(rollback);
    m_cart.saveState(rollback);

    auto loadPayload = [&](const uint8_t* begin, const uint8_t* finish) {
        const uint8_t* cursor = begin;
        return m_cpu.loadState(cursor, finish) && m_ppu.loadState(cursor, finish) &&
            m_bus.loadState(cursor, finish) && m_apu.loadState(cursor, finish) &&
            m_cart.loadState(cursor, finish) && cursor == finish;
    };

    if (!loadPayload(payload, payload + payloadSize)) {
        (void)loadPayload(rollback.data(), rollback.data() + rollback.size());
        m_statusMessage = "Save state load failed (corrupt or wrong ROM); current state was preserved.";
        m_statusIsError = true;
        return false;
    }
    clearRewindHistory();
    m_statusMessage = "State loaded: " + path;
    m_statusIsError = false;
    return true;
}

void Frontend::toggleFullscreen()
{
    m_fullscreen = !m_fullscreen;
    SDL_SetWindowFullscreen(m_window, m_fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
    SDL_ShowCursor(m_fullscreen ? SDL_DISABLE : SDL_ENABLE);
    m_statusMessage = m_fullscreen
        ? "Borderless fullscreen ON - press the fullscreen hotkey to return."
        : "Borderless fullscreen OFF";
    m_statusIsError = false;
}
