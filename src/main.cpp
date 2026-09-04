#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlrenderer3.h"

#include "frontend/Frontend.hpp"
#include "core/CPU.hpp"
#include "core/Bus.hpp"
#include "core/PPU.hpp"
#include "core/APU.hpp"
#include "core/Cartridge.hpp"

#include <memory>

int main(int argc, char** argv)
{
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMEPAD))
        return 1;

    SDL_Window* window = SDL_CreateWindow(
        "NES Ultimate Emulator",
        1280, 720,
        SDL_WINDOW_RESIZABLE
    );
    if (!window) {
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    auto bus = std::make_unique<Bus>();
    auto cpu = std::make_unique<CPU>(*bus);
    auto ppu = std::make_unique<PPU>();
    auto apu = std::make_unique<APU>();
    auto cart = std::make_unique<Cartridge>();

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
    apu->initAudio();

    Frontend frontend(window, renderer, *cpu, *bus, *cart, *ppu, *apu);
    if (argc > 1 && argv[1] && *argv[1])
        frontend.loadRomPath(argv[1]);
    frontend.run();

    cart->saveBattery();
    apu->shutdownAudio();

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
