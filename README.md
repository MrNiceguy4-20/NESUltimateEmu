# NES Ultimate Emulator

A work-in-progress NTSC Nintendo Entertainment System / Famicom emulator written in C++17 with an SDL2 + Dear ImGui frontend.

This build focuses on CPU/PPU timing accuracy, mapper coverage, save-state integrity, Famicom Disk System support, NES expansion audio, and an optional **KYLXBN-style Chip Mod** for an alternate idealized sound path.

> **Current project build:** Phase 11 — Audio Output Hotfix  
> **Current save-state format:** Version 13  
> **Primary target:** Windows / Visual Studio / NTSC NES

---

## Features

### Core emulation

- 6502/2A03 CPU core with all 256 opcode slots populated, including unofficial opcodes
- Cycle-aware CPU bus activity for interrupt entry and dummy reads
- IRQ/NMI handling including BRK/IRQ vector hijacking behavior
- CPU open-bus behavior for I/O and expansion space
- PPU rendering with NTSC even/odd frame timing
- VBlank/NMI timing and suppression handling
- Dot-timed sprite overflow evaluation, including the diagonal sprite-overflow bug
- Sprite DMA and DMC DMA support
- APU emulation for:
  - Pulse 1
  - Pulse 2
  - Triangle
  - Noise
  - DMC
- Battery-backed RAM support
- iNES and NES 2.0 loading
- 512-byte iNES trainer support
- Famicom Disk System image support
- Save states with ROM identity validation and checksums
- Player 1 and Player 2 input
- SDL game-controller support for Player 1
- Integrated CPU/PPU/APU/mapper debugger and trace output

### Expansion audio

The cartridge/mapper layer includes expansion-audio support for several NES/Famicom sound chips, including:

- **MMC5** — pulse channels and PCM
- **VRC6** — pulse channels and saw
- **Namco 163 / N163** — wavetable audio
- **Sunsoft 5B** — tone, noise, mixer, and envelope paths
- **VRC7** — project-local six-channel OPLL-style FM implementation
- **FDS** — wavetable audio

### KYLXBN-style Chip Mod

Press **C** to toggle the optional Chip Mod.

**Chip Mod OFF** is the hardware-oriented playback path and uses the normal nonlinear NES mixer.

**Chip Mod ON** enables the project's KYLXBN-style idealized audio path where implemented, including:

- floating-point / forced-linear 2A03 mixing
- smoothed triangle waveform
- triangle loudness compensation
- smoothed VRC6 saw output
- floating-point VRC7 mixing
- continuous N163 channel output intended to reduce the normal multiplexing/aliasing character

The Chip Mod is intentionally an alternate sound mode. Hardware-accuracy testing should be performed with **Chip Mod OFF**.

---

## Current accuracy work

This source contains the cumulative core fixes from the project's accuracy phases, including:

- full cold-power state when changing ROMs
- correct iNES trainer installation
- MMC3 A12 observation on `$2006` and `$2007` activity
- CPU I/O open-bus behavior
- one-byte opcode dummy reads
- indexed load/store provisional dummy reads
- cycle-aware BRK/IRQ/NMI entry
- correct NTSC PPU frame length and odd-frame skip
- VBlank/NMI edge and cancellation behavior
- dot-timed sprite overflow and obscure overflow evaluation behavior
- rebuilt base APU/output path
- expansion-audio corrections
- SDL2 output-device compatibility hotfix

### Internal regression status

The current source has passed the project's targeted regression probes for the completed phases, including:

- reset / trainer behavior
- MMC3 A12 behavior
- CPU execution-space / open-bus behavior
- CPU dummy reads
- interrupt vectoring behavior
- NTSC PPU frame timing
- VBlank/NMI edge behavior
- sprite-overflow behavior
- base APU and expansion-audio behavior
- SDL audio-device output compatibility

The deep audio regression suite passes **46/46 checks**, and the Phase 11 host-output regression passes **10/10 checks**.

External NES test ROMs are still the final authority for hardware-conformance work.

---

## Building

### Requirements

This project currently targets the author's Windows development setup:

- Visual Studio with C++17 support
- SDL2 2.32.8
- Dear ImGui 1.92.9b
- x64 build

The Visual Studio project currently references:

```text
C:\kevin\SDL2-2.32.8
C:\kevin\imgui-1.92.9b
```

SDL2 libraries are expected under:

```text
C:\kevin\SDL2-2.32.8\lib\x64
```

The project links against:

```text
SDL2.lib
SDL2main.lib
comdlg32.lib
```

### Build steps

1. Make sure SDL2 and Dear ImGui exist at the paths above, or change the paths in `NESUltimateEmu.vcxproj` to match your machine.
2. Open `NESUltimateEmu.slnx` in Visual Studio.
3. Select an **x64** configuration.
4. Build the solution.
5. Run `NESUltimateEmu.exe`.
6. Click **Load ROM...** and select a `.nes` or `.fds` image.

ROM images and the FDS BIOS are not included with the emulator.

---

## Controls

### Emulator controls

| Key | Action |
|---|---|
| `P` | Pause / resume |
| `R` | Soft reset |
| `1`–`4` | Video scale 1x–4x |
| `A` | Toggle NTSC 8:7 aspect / square pixels |
| `C` | Toggle KYLXBN-style Chip Mod |
| `F5` | Save state |
| `F6` | Next save-state slot |
| `F7` | Previous save-state slot |
| `F8` | Open/close debugger |
| `F9` | Load state |
| `F10` | Step one CPU instruction while paused |
| `F11` | Toggle fullscreen |
| `Esc` | Exit |

### Player 1 keyboard

| NES input | Key |
|---|---|
| A | `Z` or `A` |
| B | `X` or `S` |
| Start | `Enter` |
| Select | `Right Shift` or `Backspace` |
| D-pad | Arrow keys |

A compatible SDL game controller is automatically used for Player 1 when available. The left stick and D-pad are supported.

### Player 2 keyboard

| NES input | Key |
|---|---|
| A | `H` |
| B | `G` |
| Start | `Y` |
| Select | `T` |
| Up | `I` |
| Down | `K` |
| Left | `J` |
| Right | `L` |

---

## ROM formats

### NES cartridges

Supported image formats:

- iNES
- NES 2.0

The loader handles PRG ROM, CHR ROM/RAM, PRG RAM/NVRAM, mirroring, battery-backed storage, and iNES trainers.

When a mapper is not implemented, the frontend marks it as a fallback and the game may not run correctly.

### Implemented mapper IDs

The current mapper factory reports implementation support for:

```text
0, 1, 2, 3, 4, 5, 7, 9, 10, 11, 13,
16, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
30, 32, 33, 34, 48, 64, 65, 66, 67, 68, 69,
70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 82,
85, 86, 87, 88, 89, 92, 93, 94, 95, 96, 97, 101,
113, 118, 119, 140, 144, 146, 151, 152, 153, 154,
155, 157, 158, 159, 180, 184, 191, 192, 194, 195,
206, 207, 232, 240, 241, 242, 246
```

Mapper 210 is supported for the implemented submapper variants.

Mapper support does not automatically mean every board revision or obscure hardware quirk is fully cycle-perfect.

---

## Famicom Disk System

`.fds` images are supported.

A valid **8 KiB `disksys.rom`** BIOS must be placed beside the `.fds` image being loaded.

When an FDS title is loaded, the frontend provides controls to:

- eject the disk
- insert a disk side
- switch between available A/B sides

FDS wavetable audio is implemented.

---

## Save data

### Battery saves

Battery-backed cartridges automatically use a `.sav` file beside the ROM.

Battery RAM is saved when changing ROMs and when the emulator exits.

### Save states

There are ten save-state slots: `0` through `9`.

Save-state files are stored beside the loaded image using names such as:

```text
game.nes.state0
game.nes.state1
...
game.nes.state9
```

The current save-state format is **version 13**. States include a ROM identity check and checksum. Incompatible older state layouts are intentionally rejected rather than silently loaded with the wrong structure.

---

## Debugger

Press **F8** to open the integrated debugger.

Current debugger functionality includes:

- CPU register/status display
- disassembly
- breakpoints
- instruction stepping
- trace capture
- optional trace-file output
- side-effect-free CPU memory view
- PPU register/timing state
- nametable/palette VRAM inspection
- OAM inspection
- APU channel/state information
- mapper/cartridge state information

Pattern-table reads are intentionally omitted from the debugger's PPU memory view where they could trigger mapper latch side effects.

---

## Audio architecture

The NES APU is clocked from the emulated CPU timeline and integrated into the host sample rate using fractional CPU-clock-to-output-sample accumulation.

Phase 11 uses SDL2's audio-device API so the emulator callback remains **float32 mono** while SDL can convert to the physical output device format when necessary. This avoids the complete-silence failure that occurred when Windows exposed a different hardware format/channel count.

The final output contains:

- base 2A03 APU mix
- mapper expansion audio when present
- optional KYLXBN-style processing when Chip Mod is enabled

---

## Known limitations / remaining work

This emulator is still under active development. Important remaining accuracy areas include:

1. **DMC DMA bus-cycle interactions**  
   DMC DMA is implemented, but exact RDY/collision/sub-instruction timing still needs dedicated conformance work.

2. **VRC7 bit-level fidelity**  
   The current implementation is a substantially expanded project-local OPLL-style FM core, but it is not yet claimed to be bit-for-bit equivalent to mature OPLL reference cores.

3. **NTSC focus**  
   Timing is currently designed around the NTSC NES. PAL/Dendy timing is not the current target.

4. **Mapper edge cases**  
   Many mapper families are implemented, but obscure board revisions and mapper-specific electrical/timing behavior may still need game/test-specific validation.

5. **Unofficial 6502 edge behavior**  
   Several unstable illegal-opcode behaviors remain approximations rather than transistor-level models.

---

## Project layout

```text
src/
├── core/
│   ├── CPU.cpp / CPU.hpp
│   ├── Bus.cpp / Bus.hpp
│   ├── PPU.cpp / PPU.hpp
│   ├── APU.cpp / APU.hpp
│   ├── Cartridge.cpp / Cartridge.hpp
│   ├── Mapper.cpp / Mapper.hpp
│   ├── MapperFds.inc
│   ├── MapperHard.inc
│   └── MapperMore.inc
├── frontend/
│   ├── Frontend.cpp / Frontend.hpp
│   └── Debugger.cpp / Debugger.hpp
└── main.cpp
```

### Core responsibilities

- **CPU** — 2A03/6502 execution, interrupts, bus-cycle-visible dummy accesses
- **Bus** — CPU memory map, RAM, controllers, DMA, component scheduling
- **PPU** — graphics pipeline, VRAM/OAM/registers, VBlank/NMI, sprite evaluation
- **APU** — NES audio channels, frame counter, DMC, SDL output path
- **Cartridge** — ROM/FDS loading, RAM/NVRAM, persistence, mapper ownership
- **Mapper** — PRG/CHR banking, IRQs, board-specific behavior, expansion audio
- **Frontend** — SDL window/input, ImGui UI, ROM selection, state management
- **Debugger** — disassembly, tracing, breakpoints, CPU/PPU/APU inspection

---

## Testing recommendations

When checking hardware accuracy, keep **Chip Mod OFF**.

Useful external validation areas include:

- CPU instruction and dummy-read tests
- interrupt timing tests
- APU and DMC timing tests
- PPU VBlank/NMI timing tests
- sprite overflow tests
- MMC3 IRQ/A12 tests
- mapper-specific test ROMs
- expansion-audio NSF/test programs

When a test suite contains ordered subtests, run them in order and resolve the first failure before relying on later results.

---

## Legal / ROM notice

This repository contains emulator source code only. Commercial NES ROMs, copyrighted game data, and the Famicom Disk System BIOS are not included. Use software images and BIOS files that you are legally permitted to use.

---

## Development status

This project is being improved incrementally using targeted test ROM results and regression probes. Accuracy changes should be kept isolated, tested against earlier fixed behavior, and committed in phases so regressions can be identified quickly.
