#include "Frontend.hpp"
#include "../core/CPU.hpp"
#include "../core/Bus.hpp"
#include "../core/Cartridge.hpp"
#include "../core/PPU.hpp"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#endif

Frontend::Frontend(SDL_Window* window, SDL_Renderer* renderer,
    CPU& cpu, Bus& bus, Cartridge& cart, PPU& ppu)
    : m_window(window)
    , m_renderer(renderer)
    , m_cpu(cpu)
    , m_bus(bus)
    , m_cart(cart)
    , m_ppu(ppu)
    , m_running(true)
    , m_statusMessage("No ROM loaded. Click \"Load ROM...\" to choose a .nes file.")
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

void Frontend::runFrame()
{
    // Run CPU/PPU until a full frame is produced
    // Safety limit prevents infinite loop if something goes wrong
    const int kMaxCycles = 100000;
    int guard = 0;
    m_ppu.clearFrameComplete();
    while (!m_ppu.frameComplete() && guard < kMaxCycles) {
        m_bus.clock();
        guard++;
    }
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

        // ---- ImGui ----
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        drawUI();
        ImGui::Render();

        // ---- Draw ----
        SDL_SetRenderDrawColor(m_renderer, 20, 20, 30, 255);
        SDL_RenderClear(m_renderer);

        // NES screen (centered, scaled)
        if (m_cart.isLoaded() && m_nesTexture) {
            int winW = 0, winH = 0;
            SDL_GetWindowSize(m_window, &winW, &winH);
            int tw = 256 * m_scale;
            int th = 240 * m_scale;
            SDL_Rect dst{ (winW - tw) / 2, (winH - th) / 2, tw, th };
            SDL_RenderCopy(m_renderer, m_nesTexture, nullptr, &dst);
        }

        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), m_renderer);
        SDL_RenderPresent(m_renderer);

        // Simple frame limiter
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
    // Keyboard
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    uint8_t state = 0;
    if (keys[SDL_SCANCODE_Z] || keys[SDL_SCANCODE_A])     state |= 0x80; // A
    if (keys[SDL_SCANCODE_X] || keys[SDL_SCANCODE_S])     state |= 0x40; // B
    if (keys[SDL_SCANCODE_RSHIFT] || keys[SDL_SCANCODE_BACKSPACE]) state |= 0x20; // Select
    if (keys[SDL_SCANCODE_RETURN])                         state |= 0x10; // Start
    if (keys[SDL_SCANCODE_UP])                             state |= 0x08;
    if (keys[SDL_SCANCODE_DOWN])                           state |= 0x04;
    if (keys[SDL_SCANCODE_LEFT])                           state |= 0x02;
    if (keys[SDL_SCANCODE_RIGHT])                          state |= 0x01;

    // Gamepad (OR with keyboard)
    if (m_gamepad) {
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_A)) state |= 0x80;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_B)) state |= 0x40;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_BACK)) state |= 0x20;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_START)) state |= 0x10;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_UP)) state |= 0x08;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) state |= 0x04;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) state |= 0x02;
        if (SDL_GameControllerGetButton(m_gamepad, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) state |= 0x01;

        // Left stick as D-pad
        Sint16 lx = SDL_GameControllerGetAxis(m_gamepad, SDL_CONTROLLER_AXIS_LEFTX);
        Sint16 ly = SDL_GameControllerGetAxis(m_gamepad, SDL_CONTROLLER_AXIS_LEFTY);
        const Sint16 dead = 16000;
        if (ly < -dead) state |= 0x08;
        if (ly > dead) state |= 0x04;
        if (lx < -dead) state |= 0x02;
        if (lx > dead) state |= 0x01;
    }

    m_bus.setController1(state);
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
            case SDLK_p:      m_paused = !m_paused; break;
            case SDLK_r:
                if (m_cart.isLoaded()) {
                    m_cpu.reset();
                    m_statusMessage = "CPU reset.";
                    m_statusIsError = false;
                }
                break;
            case SDLK_1: m_scale = 1; break;
            case SDLK_2: m_scale = 2; break;
            case SDLK_3: m_scale = 3; break;
            case SDLK_4: m_scale = 4; break;
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
        m_cpu.reset();
        m_statusMessage = "CPU reset.";
        m_statusIsError = false;
    }
    ImGui::SameLine();
    if (ImGui::Button(m_paused ? "Resume" : "Pause")) {
        m_paused = !m_paused;
    }

    ImGui::Separator();

    if (m_statusIsError)
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s", m_statusMessage.c_str());
    else
        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%s", m_statusMessage.c_str());

    if (m_cart.isLoaded()) {
        ImGui::Separator();
        ImGui::Text("File   : %s", m_cart.fileName().c_str());
        ImGui::Text("Mapper : %u%s", m_cart.mapper(),
            (m_cart.mapper() <= 4 || m_cart.mapper() == 7) ? " (supported)" : " (unsupported – may not run)");
        ImGui::Text("PRG    : %u x 16KB", m_cart.prgBanks());
        ImGui::Text("CHR    : %u x  8KB%s", m_cart.chrBanks(),
            m_cart.hasChrRam() ? " (CHR RAM)" : "");
        const char* mirrorStr = "Horizontal";
        switch (m_cart.mirroring()) {
        case Cartridge::Mirror::Vertical:     mirrorStr = "Vertical"; break;
        case Cartridge::Mirror::OnescreenLo:  mirrorStr = "One-screen LO"; break;
        case Cartridge::Mirror::OnescreenHi:  mirrorStr = "One-screen HI"; break;
        default: break;
        }
        ImGui::Text("Mirror : %s", mirrorStr);
        if (m_cart.hasBattery())
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.3f, 1.0f), "Battery RAM: yes (auto-saves .sav)");
        ImGui::Text("Scale  : %dx  (keys 1-4)", m_scale);
        ImGui::Text("Paused : %s  (P to toggle)", m_paused ? "yes" : "no");
        ImGui::Separator();
        ImGui::Text("Controls: Z/A=A  X/S=B  Enter=Start  RShift=Select");
        ImGui::Text("          Arrow keys = D-Pad  |  Gamepad supported");
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
    ofn.lpstrFilter = "NES ROMs (*.nes)\0*.nes\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = "Select NES ROM";

    if (!GetOpenFileNameA(&ofn))
        return false;

    m_cart.saveBattery();
    if (m_cart.loadFromFile(filename)) {
        m_cpu.reset();
        m_statusMessage = "Loaded: " + m_cart.fileName();
        m_statusIsError = false;
        m_paused = false;
        return true;
    }
    else {
        m_statusMessage = "Failed to load ROM (invalid iNES / unsupported).";
        m_statusIsError = true;
        return false;
    }
#else
    m_statusMessage = "Native file dialog only on Windows.";
    m_statusIsError = true;
    return false;
#endif
}







