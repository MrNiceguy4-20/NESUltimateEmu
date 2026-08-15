#pragma once
#include <SDL.h>
#include "imgui.h"
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
    bool          m_running;

    SDL_Texture* m_nesTexture = nullptr;
    SDL_GameController* m_gamepad = nullptr;

    std::string   m_statusMessage;
    bool          m_statusIsError = false;

    bool          m_paused = false;
    int           m_scale = 3;

    void processEvents();
    void updateControllers();
    void drawUI();
    void runFrame();
    bool openRomDialog();
    void updateTexture();
    void openGamepad();
    void saveState();
    void loadState();
};






