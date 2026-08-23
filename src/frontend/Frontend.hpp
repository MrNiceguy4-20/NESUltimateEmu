#pragma once
#include <SDL.h>
#include "imgui.h"
#include <array>
#include <deque>
#include <string>
#include <vector>
#include "../core/Timing.hpp"

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
    bool          m_running;

    SDL_Texture* m_nesTexture = nullptr;
    SDL_GameController* m_gamepad = nullptr;

    std::string m_statusMessage;
    bool        m_statusIsError = false;

    bool m_paused = false;
    int  m_scale = 3;
    bool m_fullscreen = false;
    bool m_ntscAspect = true;
    int  m_saveSlot = 0;

    enum class NesButton : int { A, B, Select, Start, Up, Down, Left, Right, Count };
    enum class HotkeyAction : int {
        Pause, StepInstruction, Reset, Scale1, Scale2, Scale3, Scale4,
        SaveState, LoadState, SlotNext, SlotPrevious, Aspect, ChipMod, Fullscreen, SpeedPrevious, SpeedNext, Quit, Rewind, Count
    };
    enum class CaptureTarget { None, P1Keyboard, P2Keyboard, P1Gamepad, Hotkey };

    std::array<SDL_Scancode, static_cast<int>(NesButton::Count)> m_p1Keys{};
    std::array<SDL_Scancode, static_cast<int>(NesButton::Count)> m_p2Keys{};
    std::array<SDL_GameControllerButton, static_cast<int>(NesButton::Count)> m_p1PadButtons{};
    std::array<SDL_Keycode, static_cast<int>(HotkeyAction::Count)> m_hotkeys{};
    CaptureTarget m_captureTarget = CaptureTarget::None;
    int m_captureIndex = -1;
    bool m_controllerConfigOpen = false;
    bool m_settingsOpen = false;
    float m_masterVolume = 1.50f;

    enum class EmulationSpeed : int { Normal = 0, Double = 1, Quadruple = 2, Uncapped = 3 };
    EmulationSpeed m_emulationSpeed = EmulationSpeed::Normal;
    uint64_t m_pacingDeadline = 0;
    bool m_audioPacingPrimed = false;
    bool m_systemSleepSuppressed = false;

    std::deque<std::vector<uint8_t>> m_rewindStates;
    size_t m_rewindBytes = 0;
    uint64_t m_rewindFrameCounter = 0;
    uint64_t m_rewindRomIdentity = 0;
    static constexpr size_t kMaxRewindSnapshots = 900;
    static constexpr size_t kMaxRewindBytes = 128u * 1024u * 1024u;

    enum class TimingOverride : int { Auto = 0, NTSC = 1, PAL = 2, Dendy = 3 };
    TimingOverride m_timingOverride = TimingOverride::Auto;

    void processEvents();
    void updateControllers();
    void drawUI();
    void runFrame();
    int framesPerPresentation() const;
    double emulationSpeedMultiplier() const;
    const char* emulationSpeedName() const;
    void setEmulationSpeed(EmulationSpeed speed);
    void resetPacing();
    void pacePresentation(bool allowAudioSync = true);
    void updateSystemSleepSuppression();
    bool rewindHeld() const;
    void captureRewindState();
    bool rewindStep();
    void clearRewindHistory();
    std::vector<uint8_t> captureMachinePayload() const;
    bool loadMachinePayload(const std::vector<uint8_t>& payload);
    void stepInstruction();
    bool openRomDialog();
    bool openStateLoadDialog();
    bool openStateSaveDialog();
    bool openCheatFileDialog();
    void reloadCheatsForLoadedRom();
    void updateTexture();
    SDL_Rect gameDestinationRect() const;
    void openGamepad();
    void toggleFullscreen();
    void drawControllerConfig();
    void drawSettings();
    void setDefaultBindings();
    void loadFrontendConfig();
    void saveFrontendConfig() const;
    ConsoleTiming effectiveTiming() const;
    void applyTimingOverride(bool resetSystem);
    bool handleBindingCapture(const SDL_Event& e);
    bool handleHotkey(SDL_Keycode key);
    void saveState();
    void loadState();
    bool saveStateToPath(const std::string& path);
    bool loadStateFromPath(const std::string& path);
    std::string statePath(int slot) const;
};
