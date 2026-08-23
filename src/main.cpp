#include <SDL.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "frontend/Frontend.hpp"
#include "core/CPU.hpp"
#include "core/Bus.hpp"
#include "core/PPU.hpp"
#include "core/APU.hpp"
#include "core/Cartridge.hpp"

#include <memory>

int main(int, char**)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0)
        return 1;

    SDL_Window* window = SDL_CreateWindow(
        "NES Ultimate Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1,
        SDL_RENDERER_ACCELERATED);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

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
    frontend.run();

    cart->saveBattery();
    apu->shutdownAudio();

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
