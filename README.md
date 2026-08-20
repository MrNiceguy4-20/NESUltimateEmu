# Phase 27D4B - Unified DMA GET/PUT cadence

This phase replaces scattered raw CPU-counter parity checks with one canonical Bus DMA cadence shared by DMC scheduling, DMC alignment, OAM DMA reads/writes, and OAM startup alignment. The selected deterministic NTSC alignment uses odd `cpuCycleCounter` slots as GET and even slots as PUT. This targets the persistent one-slot-late `dma_4016_read` result while keeping the DMA engines phase-locked to each other. Save-state format is Version 43.

# NES Ultimate Emulator — Phase 27D4A Hotfix 11: BRK/NMI Hijack Boundary
## Hotfix 11 — BRK-specific NMI takeover boundary

`cpu_interrupts_v2/3-nmi_and_irq` and `4-irq_and_dma` are confirmed passing. The remaining `2-nmi_and_brk` mismatch exposed a BRK-specific takeover boundary: a fetched BRK may still have its vector replaced by a late NMI at the low-vector boundary while retaining the BRK-origin status push (B=1), whereas a hardware IRQ has already committed its vector at that point. The CPU now models those two cutoffs separately. Save-state format is Version 42.


The CPU interrupt-entry fixes from Hotfixes 6–8 are retained. `3-nmi_and_irq` and `4-irq_and_dma` are confirmed passing on hardware-test ROMs. The remaining `2-nmi_and_brk` mismatch is the synchronization-sensitive case documented by the test itself. Bus scheduling now advances one PPU dot before each CPU/APU slot and two PPU dots afterward, rather than all three PPU dots before the CPU. This preserves the NTSC 3:1 clock ratio while selecting the reference PPU/CPU phase at the NMI sampling boundary. Save-state layout remains Version 42.

## Hotfix 8 — NMI vector-commit boundary

Historical Hotfix 8 note: hardware IRQ commits before the low-vector fetch. Hotfix 11 refines this by allowing fetched BRK, specifically, one later NMI takeover boundary while preserving B=1. Save-state version 42.

Phase 27D4A Hotfix 6 - Immediate IRQ Poll Timing

# Phase 27D4A Hotfix 5 — BRK/IRQ NMI Hijack Window

This focused timing hotfix extends NMI takeover of an in-progress BRK/IRQ entry through the low-vector fetch boundary. An NMI pending when the `$FFFE` low-vector cycle is about to begin now redirects that access to `$FFFA`; an NMI arriving after the low-vector fetch remains pending for the following handler instruction boundary. This matches the five-clock BRK hijack window exercised by `cpu_interrupts_v2`.

A dedicated headless probe was added at `tests/probes/InterruptHijackProbe.cpp`. The old Hotfix 4 core fails the probe (`after_low_vector=FFFF`), while this build passes (`after_low_vector=FFFB`). Save-state format: **Version 42**.

# Phase 27D4A Hotfix 4 — Correct DMC Load Start Window

This build replaces the ineffective one/two-clock DMC load-delay tuning with the hardware 3/4-cycle `$4015` load-start rule: a new load waits three CPU clocks and then starts on the next DMA GET phase. Save-state format: **Version 38**.

# Phase 27D4A Hotfix 3 Hotfix 2 — PPU open bus, OAM attribute RAM, DMA 4016 alignment

This build fixes three hardware-test regressions observed on the real frontend:

- PPU I/O open-bus bits now decay independently toward 0 and only bits actively driven by a PPU register read are refreshed.
- OAM sprite-attribute bytes physically mask bits 2-4 (`& 0xE3`) on CPU and DMA writes, and `$2004` reads expose those bits as zero.
- `$4015` DMC load startup alignment is shifted one CPU clock earlier so `dma_4016_read` targets the documented `08 08 07 08 08` pattern.

Save-state format: **Version 38**.

# NES Ultimate Emulator — Phase 27D4A Hotfix: DMC $4016 Timing

## Phase 27D4A Hotfix — DMC load startup timing

This hotfix corrects the `$4015`-initiated DMC load startup delay from an incorrectly modeled 2 CPU clocks to the hardware 3/4 CPU-clock window. The DMC request now waits at least three CPU clocks after the `$4015` write and, when required by the alternating DMA phase, waits one additional clock before it can acquire RDY. This directly targets `dmc_dma_during_read4/dma_4016_read.nes`, whose source expects `08 08 07 08 08`; an all-`08` result means the DMA did not land on the intended `$4016` read.

The hotfix also removes an accidental duplicate local `uint8_t data` declaration in `Bus.cpp`.

Save-state format advances to **Version 35** because an in-flight DMC startup-delay state from Version 34 has different timing semantics.

The Phase 27D4A APU-register activation behavior, Phase 27D3 PPU conflicts, Phase 27D2A/D2B controller behavior, and Phase 27A–27C DMC work remain present.

> **Current phase:** 27D4A Hotfix — DMC `$4016` timing conformance
> **Current save-state format:** Version 38
> **Primary target:** `dmc_dma_during_read4/dma_4016_read.nes`

## Phase 26H1 — Immediate + JMP bus cycles

All official/unofficial immediate operand reads now occur on their real cycle-2 bus slot, and JMP absolute/indirect expose each operand and pointer read explicitly. JMP indirect preserves the NMOS page-wrap bug. Save-state format is Version 28.

A work-in-progress NTSC Nintendo Entertainment System / Famicom emulator written in C++17 with an SDL2 + Dear ImGui frontend.

This build focuses on CPU/PPU timing accuracy, mapper coverage, save-state integrity, Famicom Disk System support, NES expansion audio, and an optional **KYLXBN-style Chip Mod** for an alternate idealized sound path.

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
- Remappable P1/P2 keyboard controls and P1 SDL game-controller buttons
- Remappable emulator hotkeys with persistent frontend preferences
- Master volume control from 0–250% with 150% default gain
- SDL game-controller support for Player 1

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

## Frontend configuration

The main window exposes a **Load ROM / State...** dropdown, a **Settings** dropdown, and an always-visible master **Volume** slider.

- **Load ROM / State...** opens a dropdown with **Load ROM...**, **Load State...**, and **Save State...**. Load/Save State use native Windows file pickers so states can be stored anywhere. Manual state files default to `.nesstate`; legacy `.state0`–`.state9` files remain loadable.
- The existing ten-slot save-state hotkeys remain available for quick saves/loads, but the slot row is no longer shown in the main frontend.
- **Borderless Fullscreen** uses SDL desktop fullscreen, fits the game to the display while preserving the selected NES aspect, hides frontend overlays/cursor, and uses black aspect bars where necessary. F11 is the default fullscreen hotkey.
- **Controller** remaps all eight NES buttons for Player 1 keyboard, Player 1 gamepad, and Player 2 keyboard.
- **Settings** remaps every existing emulator hotkey, including pause, reset, save/load state, slot selection, scale, aspect, Chip Mod, fullscreen, and quit.
- **Volume** controls frontend master output gain from 0% to 250%, with quick 100%, 150%, 200%, and mute buttons. The default is 150% to address the previous low host-output level.
- Preferences are stored in `nesultimate.cfg` beside the emulator working directory and are intentionally separate from save states.


### Phase 21 cleanup

The integrated debugger has been removed from the frontend and project build. This removes the debugger button, F8 binding, trace capture, breakpoints, disassembly/memory inspection UI, debugger-only source files, and side-effect-free inspection APIs that were no longer needed. **Step Instruction** remains available as a normal emulator control.

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
- DMC RDY gating on known CPU write cycles
- parity-sensitive 3/4-cycle DMC fetch timing with stretched CPU reads
- DMC/OAM DMA collision arbitration with OAM retry and parity realignment
- common 2-APU-cycle DMC load delay after `$4015` enable, plus pre-halt explicit DMA cancellation
- canonical `CPU::BusCycle` preview used by DMA arbitration, exposing read/write, address, write data, dummy status, and whether the cycle is exact
- DMC RDY arbitration now consumes the canonical CPU bus-cycle record instead of DMC-specific CPU heuristics


### Phase 26 CPU bus conversion status

Phase 26 is intentionally being converted incrementally. `CPU::nextBusCycle()` is now the single source of truth for the CPU slot that DMA sees. Cycles already modeled explicitly (opcode boundaries, interrupt entry, one-byte discarded reads, indexed provisional reads, scheduled final reads/stores, and unstable high-byte stores) are marked `exact=true`. Instruction families still executed atomically by the legacy core are returned as `exact=false` rather than being silently presented as hardware-accurate.

The next Phase 26 subphases should convert reset sequencing, stack instructions, read-modify-write instructions, branches/addressing operand fetches, and remaining ALU/load/store families into scheduled per-cycle micro-operations until the synthesized fallback disappears entirely.

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

The current save-state format is **version 28**. States include a ROM identity check and checksum. Incompatible older state layouts are intentionally rejected rather than silently loaded with the wrong structure.

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

1. **DMC DMA revision-specific abort behavior**  
   Load delay, RDY gating, repeated reads, and OAM collisions are modeled, but the early-vs-late 2A03 implicit-abort quirk still needs a selectable CPU-revision model and dedicated conformance coverage.

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


## Phase 23 frontend cleanup

The main Settings button now opens a dropdown containing Controller configuration, Hotkeys, Volume, and Borderless Fullscreen. The fullscreen menu item shows the currently configured fullscreen hotkey; standalone Controller, Volume, and Fullscreen buttons were removed.

## Phase 24 frontend repair
Restored the ROM/save-state file-dialog methods and Version 17 save-state helper functions that were accidentally omitted from Frontend.cpp during the main-volume-slider edit.


## Phase 25 — Headless Regression Harness

A second Visual Studio target, `NESUltimateEmu.Tests`, now runs the emulator core without SDL/ImGui host I/O. It recognizes the standard Blargg `$6000` status/signature/text protocol, supports reset-request tests, has deterministic emulated-cycle timeouts, and returns machine-readable exit codes. See `tests/README.md` for commands and suite usage. Third-party test ROMs are not bundled.


## Phase 26A — CPU Bus-Cycle Contract

Introduced `CPU::BusCycle` as the single read/write/address/data/dummy/exact contract consumed by DMA arbitration. Exact bus-visible instruction cycles are identified explicitly; remaining instruction-atomic slots are tagged synthesized rather than hidden behind a PC fallback assumption.

## Phase 26B — RESET + Stack/Subroutine Bus Cycles

Converted RESET and the stack-control family to explicit per-clock bus sequences. RESET now performs two discarded PC reads, three stack-page reads while decrementing S, and the `$FFFC/$FFFD` vector fetch on their real cycles. PHA/PHP/PLA/PLP plus JSR/RTS/RTI now expose their dummy reads, stack reads/writes, operand fetches, and return-vector activity through `CPU::BusCycle`, allowing RDY/DMC arbitration to observe their real read/write slots. The serialized CPU payload layout is unchanged, so save-state format remains Version 17.

## Phase 26C — memory RMW bus scheduling

Memory read/modify/write instructions now expose the NMOS 6502's two final write cycles explicitly: the original byte is written on the penultimate cycle and the modified byte on the final cycle. This applies to ASL/LSR/ROL/ROR/INC/DEC and unofficial SLO/RLA/SRE/RRA/DCP/ISC. Save-state format was version 18 for the Phase 26C pending RMW sequence. Phase 26D1 advances the format to version 19 because indexed zero-page load/store instructions now have new serialized mid-instruction pending states.

## Phase 26D1 — Indexed zero-page load/store bus cycles

The official indexed zero-page load/store family is now microcoded across its real four CPU cycles instead of resolving the address and data access atomically during opcode dispatch. `LDA zp,X`, `LDX zp,Y`, `LDY zp,X`, `STA zp,X`, `STX zp,Y`, and `STY zp,X` now expose: cycle 2 operand fetch from PC, cycle 3 mandatory dummy read from the unindexed zero-page address, and cycle 4 final indexed read/write with zero-page wrap. These slots are reported as exact through `CPU::nextBusCycle()`, allowing DMC RDY arbitration to see the real read/write type and address.

This is intentionally Phase 26D1, not the whole addressing-mode conversion. Indexed ALU/RMW families and `(indirect,X)` / `(indirect),Y` pointer-fetch sequences remain targets for the next subphases. Save-state format is version 19.


## Phase 26D2 — Indexed zero-page ALU/compare bus cycles

The indexed zero-page AND/ORA/EOR/ADC/SBC/CMP family uses the explicit four-cycle sequence introduced in Phase 26D1: operand fetch, mandatory unindexed zero-page dummy read, then final indexed data read. Save-state format advanced to version 20.

## Phase 26D3 — Indexed zero-page RMW bus cycles

The official and unofficial `zp,X` RMW family now exposes operand fetch, unindexed dummy read, indexed data read, old-value write, and modified-value write as exact CPU bus slots. This covers ASL/LSR/ROL/ROR/INC/DEC plus SLO/RLA/SRE/RRA/DCP/ISC. Save-state format advanced to version 21.

## Phase 26E1 — Absolute-indexed read bus cycles

Absolute-indexed read instructions now fetch both address operand bytes on their real CPU cycles. `abs,X` / `abs,Y` reads use the original high byte for the provisional access and, on a page crossing, perform the corrected effective-address read on the following cycle. LDA/LDX/LDY, AND/ORA/EOR/ADC/SBC/CMP, unofficial LAX/LAS, and absolute-X NOP reads use this shared microsequence. These slots are reported `exact=true` through `CPU::nextBusCycle()` for RDY/DMC arbitration. Save-state format is version 22 because the new absolute-indexed pending operations can exist in a mid-instruction state.


## Phase 26E2 — Absolute-Indexed Store Bus Cycles

Absolute-indexed stores now fetch both address bytes on their real CPU cycles, perform the mandatory provisional read using the original high byte plus indexed low byte, and drive the final write on cycle 5. `STA abs,X` and `STA abs,Y` use this exact sequence. The absolute unstable high-byte stores `SHY abs,X`, `SHX abs,Y`, `AHX abs,Y`, and `TAS abs,Y` now use the same explicit addressing sequence while retaining their NMOS page-cross address/data corruption model. `(zp),Y` AHX remains for the indirect-addressing phase. These bus slots are exposed as `exact=true` through `CPU::nextBusCycle()`. Save-state format is version 23 because the new pending store micro-operations can exist in a mid-instruction state.


## Phase 26F — indirect addressing bus cycles

`($nn,X)` and `($nn),Y` are now explicit CPU bus microsequences instead of atomic addressing helpers. Indexed-indirect reads/stores expose the operand fetch, unindexed zero-page dummy read, wrapped pointer-low/pointer-high reads, and final data transfer. Indirect-indexed reads expose pointer fetches plus the provisional old-high-byte read only when a page crossing occurs; stores always perform that provisional read. The unofficial indirect RMW families (`SLO/RLA/SRE/RRA/DCP/ISC`) now expose the full pointer walk, provisional/effective read, old-value write, and modified-value write. `AHX ($nn),Y` is also on the exact indirect-store path, including page-cross high-byte address corruption. All converted slots report `exact=true` through `CPU::nextBusCycle()`. Save-state format advances to version 25 because the new indirect pending micro-operations can exist mid-instruction.


## Phase 26E3 — Absolute-Indexed RMW Bus Cycles

Absolute-indexed memory RMW instructions now use an explicit seven-cycle NMOS 6502 sequence. The official `ASL/LSR/ROL/ROR/INC/DEC abs,X` family and unofficial `SLO/RLA/SRE/RRA/DCP/ISC abs,X/abs,Y` forms fetch both operand bytes on their real cycles, perform the mandatory provisional read using the original high byte plus indexed low byte, read the corrected effective address, write the original value, then write the modified value. All of these slots are exposed as `exact=true` through `CPU::nextBusCycle()` so RDY/DMC arbitration sees the real bus type and address. Save-state format advances to version 24 because these new pending RMW states can exist mid-instruction.

## Phase 26G1 — direct zero-page/absolute read and store bus cycles

Non-indexed zero-page and absolute loads, stores, ALU/compare reads, BIT, LAX/SAX, and memory NOPs now use explicit CPU microsequences. Zero-page forms expose the operand fetch and final data transfer; absolute forms expose low-byte fetch, high-byte fetch, and final transfer. Store cycles expose exact write data through `CPU::nextBusCycle()`, and all converted slots report `exact=true` for DMC/RDY arbitration. Non-indexed RMW was completed in Phase 26G2. Save-state format advanced to version 26 in Phase 26G1 because these direct-memory pending operations can exist mid-instruction.


## Hotfix 7 - NMI polling alignment

NMI is now sampled alongside IRQ at the normal second-to-last-cycle interrupt polling point for finite instruction microsequences. The final-cycle late-edge path remains in place for NMI edges that arrive after the normal poll but before instruction completion. This targets the one-slot-late patterns observed in cpu_interrupts_v2/rom_singles/2-nmi_and_brk and 3-nmi_and_irq. Save-state format is Version 43.


## Hotfix 11 - BRK hijack window + NTSC divider phase
- Restored BRK/NMI vector hijacking to the five-clock window used by cpu_interrupts_v2.
- Selected the next valid NTSC PPU/CPU divider alignment: 2 PPU dots before the CPU slot, 1 after.
- No serialized fields changed; save-state version remains 42.
