# NES Ultimate Emulator

> **Project history / modernization:** Development on this project originally began in **2017**. The current modernization effort is updating the codebase toward the newest practical **C++** standard and **SDL** release, while preserving emulator accuracy and compatibility. The **Dear ImGui** integration has already been updated.

A cycle-focused Nintendo Entertainment System / Famicom emulator written in C++17, with an SDL2 + Dear ImGui desktop frontend and a headless hardware-conformance test runner.

This project goes beyond basic NROM-era emulation: it includes iNES and NES 2.0 parsing, Famicom Disk System support, region-aware timing, save states, battery persistence, controller remapping, Game Genie/raw CPU cheats, expansion audio, a ROM metadata database, and a large mapper implementation set.

> **Project status:** actively accuracy-oriented. The repository contains extensive regression probes and hardware-timing audit notes, but broad mapper presence does not imply perfect compatibility with every dump, board revision, submapper, homebrew, or test ROM.

## Highlights

- **6502/RP2A03 CPU core** with bus-cycle-oriented timing behavior and coverage for interrupt, branch, DMA, dummy-read/write, unstable-opcode, and BRK/NMI edge cases.
- **2C02-style PPU emulation** including per-dot rendering behavior, sprite evaluation/overflow behavior, sprite-0 hit rules, open-bus behavior, `$2007` access conflicts, VBlank/NMI race handling, odd-frame timing, and NTSC/PAL/Dendy frame timing.
- **NES APU emulation** for pulse 1/2, triangle, noise, and DMC, including frame-counter and DMC DMA edge cases.
- **Expansion audio paths** used by supported cartridge hardware such as VRC6, VRC7, Namco 163, FDS, MMC5, and Sunsoft-family boards.
- **iNES + NES 2.0** cartridge parsing, trainer handling, RAM/NVRAM sizing, submapper gating, mirroring, and ROM-database metadata corrections.
- **Famicom Disk System** image loading, side insertion/ejection, writable media persistence, and FDS audio. A valid 8 KiB `disksys.rom` BIOS is required.
- **Large mapper set:** the mapper factory contains implementations/aliases for 189 mapper IDs, with NES 2.0 submapper validation where board variants materially differ.
- **Save states** with ROM identity checking, payload sizing, checksum validation, and transactional rollback if state loading fails.
- **Battery-backed saves** written automatically as `.sav`, including mapper-owned persistent memory where applicable.
- **Cheat system** with 6/8-character Game Genie decoding plus raw CPU-address codes and a bundled `.cht` database.
- **Two keyboard players + SDL game controller support**, configurable at runtime.
- **NTSC, PAL, and Dendy timing**, selectable automatically from ROM metadata or forced in the UI.
- **Archive loading on Windows** through the system `tar.exe`/libarchive backend for formats such as ZIP, 7z, RAR, tar, gzip, bzip2, and xz when supported by the installed Windows backend.
- **Headless regression runner** and optional integration with external NES test-ROM suites through CMake/CTest.

## Screens / Frontend

The desktop frontend is implemented with **SDL2** and **Dear ImGui**. It provides:

- ROM/FDS/archive loading
- Reset, pause, and instruction stepping
- Save-state slots and explicit save/load dialogs
- Cheat database browsing and manual cheat-file loading
- Controller and hotkey remapping
- Automatic or forced console timing
- Integer display scaling (1x-4x)
- NTSC-aspect toggle
- Borderless fullscreen
- Master volume up to 250%
- FDS side selection/ejection controls
- Mapper/header/mirroring/timing status information

The renderer uses a 256x240 streaming texture with nearest-neighbor scaling.

## Default Controls

### Player 1 - Keyboard

| NES button | Key |
| --- | --- |
| A | `Z` |
| B | `X` |
| Select | `Right Shift` |
| Start | `Enter` |
| Up | `Up Arrow` |
| Down | `Down Arrow` |
| Left | `Left Arrow` |
| Right | `Right Arrow` |

### Player 2 - Keyboard

| NES button | Key |
| --- | --- |
| A | `H` |
| B | `G` |
| Select | `T` |
| Start | `Y` |
| Up | `I` |
| Down | `K` |
| Left | `J` |
| Right | `L` |

### Game Controller - Player 1

SDL's standard game-controller mapping is used by default:

- A / B -> controller A / B
- Select -> Back
- Start -> Start
- D-pad -> D-pad
- Left analog stick also drives the NES D-pad with a dead zone

### Emulator Hotkeys

| Action | Default |
| --- | --- |
| Pause / Resume | `P` |
| Step instruction | `F10` |
| Reset | `R` |
| Scale 1x / 2x / 3x / 4x | `1` / `2` / `3` / `4` |
| Save state | `F5` |
| Load state | `F9` |
| Next save slot | `F6` |
| Previous save slot | `F7` |
| Toggle aspect | `A` |
| Toggle Chip Mod audio mode | `C` |
| Borderless fullscreen | `F11` |
| Quit | `Esc` |

Keyboard, gamepad, and hotkey mappings are configurable in the frontend and persisted to `nesultimate.cfg`.

## Supported Media

### Cartridge images

The cartridge loader recognizes standard **iNES** and **NES 2.0** headers. The Windows file picker advertises `.nes`, `.nes2`, `.bin`, and `.rom`; actual native cartridge loading is signature-validated, so a valid iNES/NES 2.0 image may still load with a non-standard extension.

Implemented cartridge handling includes:

- NES 2.0 mapper/submapper parsing
- NES 2.0 PRG/CHR exponent-multiplier sizes
- PRG RAM / PRG NVRAM / CHR RAM / CHR NVRAM sizing
- archaic iNES header cleanup
- 512-byte trainer preload
- horizontal, vertical, one-screen, and four-screen mirroring
- hash/CRC-backed ROM metadata corrections for known ambiguous legacy dumps

### Famicom Disk System

Both headered and raw FDS images are supported. FDS loading requires a valid **8 KiB** BIOS named either:

```text
disksys.rom
DISKSYS.ROM
```

The emulator searches beside the loaded image/archive and then in the current working directory.

FDS media are treated as writable. Disk changes are persisted to a `.sav` file, and the UI exposes side insertion and ejection.

### Archives - Windows frontend

The Windows frontend can inspect supported archives without extracting the selected ROM to a temporary file. It invokes the `tar.exe` supplied by modern Windows and attempts to read NES/FDS members from formats supported by that backend, including:

```text
.zip .7z .rar .tar .tgz .tar.gz .gz .tbz .tbz2
.tar.bz2 .bz2 .txz .tar.xz .xz
```

Archive support therefore depends on the capabilities of the `tar.exe`/libarchive installation on the host system.

## Mapper Coverage

The mapper factory currently contains implementations or shared hardware aliases for **189 mapper IDs**:

```text
0-13, 15-28, 30-82, 85-97, 101, 103-108, 111-123, 125,
132-133, 135-159, 171, 174, 176, 179-180, 182, 184-185,
191-195, 197, 200-204, 206-207, 209-215, 217, 225-232,
235-237, 240-243, 246, 255, 268
```

That list is **factory coverage, not a blanket compatibility guarantee**. NES 2.0 submapper checks deliberately reject variants that are known to represent materially different hardware unless that variant has an implementation. Unsupported images fall back/reject according to the cartridge support checks rather than silently claiming full board support.

Notable board families covered in the source and regression suite include MMC1/MMC3/MMC5/MMC6, VRC2/4/6/7, Namco 163/175/340 variants, Sunsoft/FME-7 families, J.Y. Company ASICs, UNROM-512/GTROM flash boards, multicarts, FDS conversions, and numerous discrete/unlicensed boards.

For board-level details, see `tests/README.md` and the audit documents under `docs/`.

## Cheats

The repository includes a large NES `.cht` database under:

```text
cheats/nes/
```

The supplied tree contains **2,262 cheat files**.

The parser supports:

- standard 6-character Game Genie codes
- standard 8-character Game Genie codes with compare values
- raw CPU-address cheat codes supported by the project's `.cht` format
- multi-code cheat entries

When a ROM is loaded, the frontend attempts to locate a matching cheat file by ROM name. Cheats can be enabled individually, disabled globally, reloaded from the bundled database, or loaded manually from another `.cht` file.

See [`cheats/README.md`](cheats/README.md) for database-format notes.

## Save Data and Save States

### Battery saves

Battery-backed cartridge data is saved automatically as `.sav` beside the ROM/archive backing path. The persistence layer supports ordinary PRG NVRAM as well as CHR NVRAM and mapper-owned persistent memory when required.

Legacy/raw `.sav` layouts remain supported for compatible boards.

### Save states

Quick-state slots use:

```text
<rom path>.state0
...
<rom path>.state9
```

The explicit file dialog uses `.nesstate` by default.

A save-state file contains:

- a `NESU` magic header
- a state-format version
- ROM identity metadata
- payload length
- FNV-1a payload checksum
- serialized CPU, PPU, bus, APU, cartridge, and mapper state

Loading validates the ROM identity and checksum before committing the state. If a component rejects the payload, the emulator restores the pre-load machine state instead of leaving a partially loaded state behind.

## Region / Timing Support

The timing layer supports:

- **NTSC** - 262 PPU scanlines
- **PAL** - 312 PPU scanlines and PAL CPU/APU timing
- **Dendy** - 312 PPU scanlines with Dendy-specific CPU/APU relationships and VBlank placement

NES 2.0 region metadata is parsed automatically. The frontend also allows `Auto`, `NTSC`, `PAL`, or `Dendy` to be forced manually; changing the timing mode restarts the emulated system.

## Building

### Requirements

Core/test code:

- CMake 3.20+
- a C++17 compiler

Desktop Windows frontend:

- Visual Studio / MSVC with a compatible C++ toolset
- Windows SDK
- SDL2
- Dear ImGui with the SDL2 + SDL_Renderer2 backends

The checked-in Visual Studio project currently references these developer-local dependency paths:

```text
C:\kevin\SDL2-2.32.8
C:\kevin\imgui-1.92.9b
```

It also declares Windows SDK `10.0.26100.0` and MSVC platform toolset `v145`.

**You will need to update the SDL2/ImGui include, source, and library paths in `NESUltimateEmu.vcxproj` (or replace them with your own property-sheet/package-manager setup) before building the GUI on another machine.**

### Windows GUI - Visual Studio

1. Install SDL2 and Dear ImGui.
2. Update the hard-coded paths in `NESUltimateEmu.vcxproj` to your installations.
3. Open `NESUltimateEmu.slnx` in Visual Studio.
4. Select `x64` and either `Debug` or `Release`.
5. Build `NESUltimateEmu`.
6. Make sure `SDL2.dll` is beside `NESUltimateEmu.exe` at runtime.

The archive already contains a prebuilt `x64/Release/NESUltimateEmu.exe` and `SDL2.dll`, but rebuilding from source is recommended when changing the core.

### Headless regression runner - CMake

The current `CMakeLists.txt` builds the **headless test runner**, not the SDL2/ImGui desktop executable.

```bash
cmake -S . -B build -DNES_BUILD_HEADLESS_TESTS=ON -DBUILD_TESTING=ON
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

To run the built-in probes directly:

```bash
./build/NESUltimateEmu.Tests --self-test
```

On multi-config Windows generators the executable may instead be under `build/Release/`.

### Sanitizers

On non-MSVC compilers, AddressSanitizer and UndefinedBehaviorSanitizer can be enabled for the headless test target:

```bash
cmake -S . -B build-san \
  -DNES_BUILD_HEADLESS_TESTS=ON \
  -DNES_ENABLE_SANITIZERS=ON
cmake --build build-san
ctest --test-dir build-san --output-on-failure
```

## Testing and Accuracy Work

The built-in probe suite covers CPU, PPU, APU, cartridge, mapper, DMA, interrupt, open-bus, cheat, and timing behavior.

A clean build of this source tree was verified with the repository's CMake configuration, and the built-in regression suite reports:

```text
14/14 probes passed
```

The CMake configuration can also register optional external ROM suites when paths are supplied, including:

- AccuracyCoin
- blargg APU tests
- legacy blargg APU tests
- blargg PPU tests
- branch timing tests
- sprite hit / sprite overflow / VBlank-NMI suites
- an extended NES hardware-test corpus

Example:

```bash
cmake -S . -B build \
  -DNES_ACCURACYCOIN_ROM=/path/to/AccuracyCoin.nes \
  -DNES_BLARGG_APU_ROM_DIR=/path/to/apu_tests \
  -DNES_BLARGG_PPU_ROM_DIR=/path/to/ppu_tests

cmake --build build
ctest --test-dir build --output-on-failure
```

External test ROMs are optional and are not implied to be distributed with the emulator.

### Accuracy notes

The repository includes focused engineering notes for several difficult NES timing areas:

- `docs/PHASE2_CPU_BUS_CYCLE_AUDIT.md`
- `docs/PHASE3_DMC_CPU_REVISION.md`
- `docs/PHASE4_PPU_VBLANK_NMI_RACE.md`
- `docs/TIER0_EXTERNAL_CONFORMANCE.md`
- `docs/TIER0_HARDENING.md`
- `docs/VOICE_PROFILE_AUDIT.md`

`tests/README.md` contains the most detailed inventory of regression coverage and mapper-specific behavior.

## Project Layout

```text
.
├── src/
│   ├── core/
│   │   ├── CPU.*          # RP2A03/6502 CPU core
│   │   ├── Bus.*          # CPU bus, controllers, DMA/arbitration
│   │   ├── PPU.*          # Picture Processing Unit
│   │   ├── APU.*          # Audio Processing Unit + host audio
│   │   ├── Cartridge.*    # iNES/NES 2.0/FDS loading + persistence
│   │   ├── Mapper.*       # mapper implementations + expansion audio
│   │   ├── CheatSystem.*  # Game Genie/raw cheats + .cht parser
│   │   ├── RomDatabase.hpp
│   │   └── Timing.hpp
│   ├── frontend/
│   │   └── Frontend.*     # SDL2 + Dear ImGui desktop UI
│   └── main.cpp
├── tests/
│   ├── HeadlessRunner.cpp
│   ├── probes/            # built-in conformance/regression probes
│   └── README.md
├── docs/                  # accuracy/hardening audit notes
├── cheats/nes/            # bundled NES cheat database
├── CMakeLists.txt         # headless test/conformance build
├── NESUltimateEmu.vcxproj # Windows GUI project
└── NESUltimateEmu.slnx
```

## Configuration Files

The frontend writes `nesultimate.cfg` in the current working directory. It stores:

- master volume
- console timing override
- Player 1 keyboard bindings
- Player 2 keyboard bindings
- Player 1 gamepad bindings
- emulator hotkeys

Dear ImGui may also write `imgui.ini` for UI layout state.

## Known Limitations / Packaging Notes

- The desktop file dialogs and archive loader are currently **Windows-specific**. On non-Windows builds, the frontend reports that native file dialogs are unavailable.
- The top-level CMake project currently builds the headless regression executable only; it does **not** configure SDL2/ImGui or build the GUI application.
- The checked-in Visual Studio GUI project contains **absolute local dependency paths** and must be adjusted on another development machine.
- FDS emulation requires a separately supplied `disksys.rom` BIOS. Do not distribute Nintendo BIOS or copyrighted game ROMs unless you have the legal right to do so.
- Mapper coverage is broad, but compatibility should be evaluated per game/board/submapper. The code intentionally gates some NES 2.0 variants rather than pretending incompatible hardware is supported.
- No license file is present in the supplied repository snapshot. Add a license before publishing the source if you want others to know their rights to use, modify, or redistribute it.

## ROM Legality

This emulator does not need commercial ROMs in the source tree. Use ROM and disk images that you are legally permitted to possess and use. The Famicom Disk System BIOS is copyrighted software and is not included by this project.

## Acknowledgements

This project owes a significant debt to the wider NES development and emulation community. In particular, sincere thanks go to **NESdev.org**, its wiki, forums, hardware researchers, emulator authors, test-ROM developers, and contributors who have spent decades documenting the behavior of the NES and Famicom. That body of shared technical knowledge has been invaluable when investigating difficult CPU, PPU, APU, DMA, mapper, timing, and hardware-compatibility problems.

I would also like to recognize the developers and maintainers of the many **open-source NES/Famicom emulators** that helped pioneer accurate software emulation and made high-quality reference implementations available to the community. Studying established approaches, comparing behavior, and learning from prior implementations has been extremely valuable while tracking down bugs, correcting inaccurate behavior, resolving performance bottlenecks, and improving this emulator's architecture and compatibility.

Notable projects and emulator lineages that have contributed to the broader body of open NES emulation knowledge include:

- **Mesen / Mesen2 / Mesen Community Edition**
- **FCE Ultra / FCEUX**
- **Nestopia / Nestopia UE / Nestopia JG**
- **Nintendulator**
- **puNES**
- **BizHawk** and its NES-related cores
- **QuickNES**
- **higan / ares**
- **FCE Ultra GX**
- **Nesalizer**
- **Fergulator**
- **Nintengo**
- **NESICIDE** and the many other research, debugging, and emulator projects catalogued by the NESdev community

This acknowledgement is intended as recognition of the community's collective research, documentation, testing, and open-source work. It does **not** imply that code from every project listed above is incorporated into this repository. Where third-party code is directly incorporated, its original license and attribution requirements should be preserved in the source tree.

Most importantly, thank you to the developers, reverse engineers, hardware testers, documentation authors, and enthusiasts who pioneered NES emulation and continue to publish their findings openly. Their work has greatly reduced the amount of duplicated research required to understand the platform and has helped make difficult accuracy and performance problems far more approachable.

## Contributing

Accuracy regressions are easiest to review when accompanied by a focused probe. A useful contribution flow is:

1. Reproduce the hardware or compatibility issue.
2. Add or extend a deterministic headless probe when possible.
3. Make the smallest core/mapper change that fixes the behavior.
4. Run `NESUltimateEmu.Tests --self-test`.
5. Run relevant external test ROMs if available.
6. Document any intentional compatibility tradeoff or unsupported submapper.

## License

No license file was found in this repository snapshot. Until a license is added, copyright law generally reserves reuse/modification/distribution rights to the copyright holder.
