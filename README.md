## Phase 90 - RP2C02 sprite-evaluation overflow chain and $2004 stress

AccuracyCoin's `$2004 Stress Test` now passes. After secondary OAM fills with eight sprites, the visible OAM bus enters the diagonal overflow scan. An in-range comparison now feeds a short +1 primary-OAM increment latch chain; when that chain expires, the address realigns and resumes +4 Y-byte stepping while OAM2 remains in read mode. The eighth sprite's tile/attribute/X bytes are also copied during the final three evaluation writes instead of being dropped when the Y byte raises the found count to eight. CPU-visible `$2004` reads therefore match the measured OAM1/OAM2 bus handoff through evaluation, primary-OAM wrap, and all eight sprite-fetch slots. AccuracyCoin improves from **137/141 to 138/141**. Built-in probes remain **13/13**, and the configured non-AccuracyCoin external corpus remains **73/73**. The headless AccuracyCoin default cycle ceiling is now 150,000,000 to avoid false timeouts around the previous 120M boundary. Save-state version is **69**.

## Phase 87 - PPU external fetch phases and X=0 sprite-zero collision

Rendering fetches now model the RP2C02 external memory transaction as separate address/ALE and /RD dots instead of performing the memory read on the address-setup dot. Mapper address notification remains on the ALE phase so MMC3/A12 observers do not see a duplicate edge on /RD. Sprite-zero collision now permits X=0 (PPU dot 1) when both PPUMASK left-edge enables expose the pixels, while X=255 remains excluded. This fixes AccuracyCoin's Run-All `Stale BG Shift Registers` prerequisite and raises the external baseline from **136/141 to 137/141**. The older blargg sprite-hit left-clipping ROM also continues to pass. Save-state version is **68** because an in-flight two-dot rendering fetch is now serialized.

# Phase 89 - Late RP2A03G Implicit DMC Abort Timing

AccuracyCoin's Implicit DMA Abort matrix now passes for the default late RP2A03G profile. The one-byte load-completion race is classified from the DMC output-unit state *after* the APU phase but before the DMA GET completes: a load ending at `bitsRemaining == 0 && timer == 0` performs the late-revision full duplicate reload, while a load ending two CPU clocks earlier keeps a three-clock implicit-stop window that produces the shared one-cycle aborted reload. The one-cycle RDY pulse is submitted in the same CPU period that the output unit consumes the buffer, so the following JSR write suppresses only the hardware-observed matrix position. AccuracyCoin improves from **135/141 to 136/141**; built-in and external regression suites remain green. Save-state version is **67**.

# Phase 85 - Implicit DMC Abort Instrumentation

See `README_PHASE85.md`. Phase 84 behavior is preserved; this build only adds a focused implicit-DMA trace.

# Phase 84 - Pre-mid-1990 Implicit DMC Abort Window

- Preserved the Phase 83 DMA core: normal DMC/OAM overlap, explicit DMA abort, internal/external CPU data-bus separation, and all existing CPU/PPU/APU behavior are unchanged.
- Matched AccuracyCoin's accepted pre-mid-1990 implicit-abort behavior more precisely. Its first implicit-abort matrix expects two adjacent one-cycle aborted reload opportunities; the second matrix suppresses the latter when it lands on a CPU write; the looping-control matrix remains normal four-cycle reload DMA.
- The one-byte, non-looping load completion now keeps the implicit-stop opportunity alive for two CPU/APU clocks instead of one, allowing both hardware-observed adjacent positions while retaining the existing one-shot write suppression in Bus.
- No changes were made to explicit-stop timing or ordinary reload scheduling.

# Phase 80 - DMC DMA abort timing

Phase 80 keeps the Phase 79 CPU/DMA and internal-data-bus fixes and refines the two remaining AccuracyCoin DMC abort cases. $4015 disable is now sampled after the DMC reader's APU-phase-dependent 2/3 CPU-clock delay instead of cancelling the memory reader immediately. If a DMC request is already in flight, the DMA is stopped at its current micro-phase: a request still at HALT becomes the one-cycle abort pulse (and is suppressed if that halt lands on a CPU write), while halt/dummy/alignment cycles already spent remain visible. The pre-mid-1990 one-byte implicit-stop race is also armed at the load-DMA completion boundary when the bit counter/timer are in the hardware race window, rather than much later when the sample buffer is consumed.

Built-in regression gate: **13/13 PASS**.

## Phase 78 DMC abort-timing checkpoint

Phase 78 branches from the validated Phase 77 DMA-arbitration baseline. It preserves the now-passing DMC+OAM overlap model and changes only the DMC stop/abort lifecycle. The implicit-stop quirk is no longer scheduled when the one-byte load DMA completes; it is scheduled when the output unit later consumes the sample buffer, which is the hardware event that creates the reload request. The default accepted AccuracyCoin behavior is the pre-mid-1990 RP2A03G one-cycle aborted reload, and one-cycle aborts remain suppressed rather than delayed if RDY would land on a CPU write. Explicit-stop detection continues to use the buffer-empty edge after $4015 disables the memory reader. Built-in regression gate: 13/13 PASS. External AccuracyCoin validation is still required for the Explicit DMA Abort and Implicit DMA Abort answer-key matrices.


## Phase 77 DMA arbitration checkpoint

Phase 77 branches from the Phase 75 baseline. It keeps the Phase 75 unstable-store RDY fix and does not include Phase 76's experimental internal/external bus-latch split. The DMA scheduler now names DMC phases explicitly (halt, dummy, align, get), corrects OAM halt/alignment polarity, and includes a deterministic headless arbitration probe verifying a 513-cycle OAM transfer and the common +2-cycle DMC/OAM overlap. External AccuracyCoin validation is still required for the explicit/implicit abort answer-key cases.

## Phase 75: CPU RDY / unstable-store accuracy

Phase 75 fixes the RP2A03/NMOS RDY interaction used by AccuracyCoin's `$93/$9B/$9C/$9E/$9F` tests. When DMC DMA acquires RDY on the provisional indexed read immediately before an unstable SH*/SHA write, the CPU now latches that condition and drops the usual `(H+1)` mask for the final transfer. This reproduces the observed DMA-stalled behavior where SHX degenerates to STX, SHY to STY, and SHA/SHS store their underlying source value without high-byte corruption.

The CPU conformance probe now drives this through the real Bus/DMC scheduler and also verifies that `ASL A` advertises the exact post-opcode PC dummy read on cycle 2. The full built-in regression gate remains 12/12 passing.

# NESUltimateEmu Phase 73

Phase 73 is an AccuracyCoin-driven stability rollback. The Phase 72 experimental split-time PPUSTATUS/sprite-counter/serial-input model caused new external regressions without fixing its target cases, so those changes were removed. This phase restores the Phase 71 emulation core as the known-good baseline while preserving all confirmed fixes through Phase 71. Save-state format remains version 64.

## Phase 73 AccuracyCoin rollback

- Removed the Phase 72 CPU-M2-rise PPUSTATUS snapshot model that regressed VBlank Beginning and NMI Suppression.
- Removed the Phase 72 explicit sprite-counter active-state experiment, which did not fix AccuracyCoin stale-sprite behavior and changed unrelated raster behavior.
- Removed the Phase 72 background serial-input experiment, which did not fix BG Serial In and regressed Hybrid Addresses.
- Restored the Phase 71 PPU/Bus timing paths exactly, including the confirmed PPUMASK delay, $2006/$2007 propagation/conflict work, OAM behavior, and INC $4014 fix.
- No speculative replacement behavior was added; future fixes will be isolated against individual AccuracyCoin failures.
- Full built-in regression gate remains the required baseline.

## Phase 70 AccuracyCoin corrections

- `$4015` frame-IRQ acknowledge now clears on the correct PUT-to-GET APU transition. Phase 69 had the polarity reversed, which changed AccuracyCoin Frame Counter IRQ failure 7 into failure 6 rather than eliminating it.
- Controller strobe sampling now uses the opposite APU half-cycle, matching AccuracyCoin's `DEC $4016` one-cycle pulse tests. Phase 69 similarly flipped failure 4 into failure 3; this corrects that inversion.
- A normal DMC sample-memory GET now releases the controller `$4016/$4017` read-select line before the halted CPU read resumes. A DMC address-bus conflict that actually selects `$4016` or `$4017` keeps the line asserted. This distinguishes AccuracyCoin Controller Clocking tests 6 and 7 and the DMA + `$4016` test.
- The DMC/APU built-in regression was updated to observe the deferred `$4015` acknowledge on the following APU transition instead of requiring an impossible combinational clear.
- Full built-in regression gate: **12/12 PASS**.


## AccuracyCoin corrections

- APU `$4015` frame-IRQ acknowledge is now deferred to the next APU GET phase instead of clearing combinationally inside the CPU read. Frame IRQ output is gated separately from the internal terminal flag pulses.
- Consecutive `$4017` writes no longer overwrite an older delayed reset that is due on the same CPU clock; this preserves distinct immediate quarter/half clocks in 5-step mode.
- Controller `$4016` strobe loading is sampled on the APU PUT phase, allowing the documented one-cycle RMW strobe asymmetry.
- DMC cartridge reads no longer deassert the internal controller read-select line, preserving the NES consecutive-read behavior during `$4016` DMA conflicts.
- Palette RAM reads honor PPUMASK greyscale by exposing only bits 4-5 of the stored 6-bit palette value while writes remain unaffected.
- Background shift registers clock whenever either rendering layer is active, including sprite-only rendering.
- Rendering-time `$2004` writes do not modify primary OAM and advance `OAMADDR` by four with the low two bits cleared.
- Corrected the Phase 63 OAM-corruption model: disabling rendering captures the current secondary-OAM address; re-enabling rendering on NTSC/Dendy copies primary OAM row 0 into the captured destination row. PAL 2C07 remains exempt. This supersedes the older OAMADDR-selected row-to-row0 approximation documented below.
- Added/updated headless regressions for delayed frame-IRQ acknowledge, controller DMA read-select behavior, and corrected OAM corruption direction.
- Full built-in regression gate remains **12/12 PASS**. External AccuracyCoin results must be rerun to verify these fixes on that suite.


Phase 68 extends the rendering-time PPUDATA (`$2007`) increment pipeline with explicit CPU/PPU I/O alignment state. The established deterministic alignment keeps the five-dot propagation used by Phase 67, while the alternate hardware-observed alignment applies the same increment after six PPU dots. Scroll-reload conflicts from Phase 67 remain intact on whichever dot the delayed increment actually lands. The alignment class is serialized so save states restore the same timing relationship. Save-state format is version 62.

# Phase 63 — Rendering-start OAM row-copy behavior


## Phase 66 — PPU internal scroll-bus conflicts

Phase 66 models the 2C02 internal bus conflict that occurs when the delayed second `$2006` (`t -> v`) transfer lands on the same PPU dot as an automatic rendering scroll increment. For the X and/or Y component being incremented, the two competing inputs are combined with a bitwise AND; that corrupted component is written into both `v` and `t`, while unaffected components still load normally from the pending `$2006` address. The implementation suppresses the normal increment later in the same dot so the conflicted component is not advanced twice. New PPU conformance coverage checks a horizontal collision and the combined horizontal+vertical collision at dot 256. The built-in regression suite remains 12/12 passing. Save-state format remains version 60 because the new one-dot suppression flags are transient within a single PPU clock and never persist across a completed clock boundary.

Phase 63 adds the documented later-2C02 rendering-start OAM corruption rule without attempting to model the still poorly characterized analog mid-access corruption cases. When active rendering begins on the NTSC 2C02-family baseline with **OAMADDR >= $08**, the 8-byte row at `OAMADDR & $F8` is copied over OAM row 0. The effect is triggered when rendering actually becomes active, not merely when `$2001` is written, so enabling rendering during vblank does not corrupt OAM prematurely. Dendy follows the 2C02-compatible behavior; PAL 2C07 is explicitly exempt.

The PPU OAM regression now verifies NTSC row-copy behavior, no corruption when OAMADDR is below $08, PAL exemption, and Dendy compatibility. The sprite-overflow regression explicitly restores OAMADDR to zero before normal rendering so it remains isolated from this separate silicon bug. A new rendering-active edge latch is serialized for deterministic mid-frame states, so save-state version is now **58**. Full built-in regression suite: **12/12 passing**.

# Phase 62 - Sprite Overflow Bug Conformance

- Added timed PPU regression coverage for the RP2C02 sprite-overflow diagonal OAM scan bug.
- Verifies reliable overflow when the ninth sprite immediately following the first eight is in range.
- Verifies a hardware-style false positive where an out-of-range ninth candidate advances the diagonal scan and a later tile byte is interpreted as a Y coordinate.
- Verifies a hardware-style false negative where a real ninth in-range sprite is skipped because the diagonal scan evaluates a non-Y byte instead.
- No PPU core behavior change was required; the existing per-dot evaluator already reproduced these cases correctly.
- Full built-in regression gate remains 12/12 PASS.
- Save-state format unchanged.

# Phase 61 - Renderer-Level Sprite-0 Hit Conformance

- Added headless regression coverage that drives the real pixel renderer rather than only the sprite-zero eligibility helper.
- Verified left-edge clipping suppresses sprite-0 hit through pixels 1-8 unless both PPUMASK background-left and sprite-left bits are enabled.
- Verified enabling only one left-edge layer still suppresses the collision.
- Verified cycle 2 and cycle 8 can set sprite-0 hit when both left-edge layers are visible.
- Verified cycle 9 is outside the left-edge clipping window and can hit even when both left-edge enable bits are clear.
- Verified transparent background or sprite pixels do not set the hit flag through the actual renderer.
- Verified PPUSTATUS bit 6 is clear immediately before the collision dot and visible immediately after that dot executes.
- No core behavior change was required; Phase 60 already implemented these renderer rules correctly.
- Full built-in regression suite: **12/12 passing**.
- Save-state format unchanged.

# Phase 60 - Sprite-0 Hit Earliest-Dot Accuracy

- Corrected sprite-0 hit so PPU cycle 1 cannot set PPUSTATUS bit 6; cycle 2 is the earliest hardware-valid collision point.
- Preserved the existing X=255 exclusion and opaque background/sprite requirements.
- Kept sprite priority out of the hit condition, matching hardware: a sprite-zero pixel can hit even when it is behind the background.
- Left-edge clipping still suppresses hits because clipping is applied before the collision rule and converts the clipped layer to transparent.
- Added headless regression coverage for cycle 1, cycle 2, X=255, transparent foreground/background, and sprite-zero identity gating.
- Full built-in regression suite: **12/12 passing**.
- Save-state format unchanged.

# NES Ultimate Emulator — Phase 59

## Phase 55 - External hardware-conformance dashboard

- Upgraded the headless test runner with `--timing auto|ntsc|pal|dendy`, allowing external hardware tests to force the console region independently of ROM header metadata.
- Upgraded `tests/run_suite.py` to consume the public `nes-test-roms/test_roms.xml` manifest and honor each ROM's declared NTSC/PAL system.
- Added repeatable `--suite` filters so focused families such as `ppu_vbl_nmi`, `vbl_nmi_timing`, `sprite_hit_tests`, `sprite_overflow_tests`, `ppu_open_bus`, `pal_apu_tests`, `mmc3_irq_tests`, and `mmc5test` can be run independently.
- Added explicit result categories for pass, test failure, timeout, unsupported protocol, and ROM/load errors instead of collapsing every nonzero result together.
- Added optional JSON result output for an accuracy dashboard and regression history.
- Added `--list`, `--fail-fast`, and `--rom-root` controls for large external corpora.
- External ROMs are intentionally not redistributed with NESUltimate; point the harness at a local copy of the hardware-test corpus.
- The built-in regression gate remains **12/12 PASS** after the harness changes. No emulation behavior or save-state format changed in this phase.

### External suite examples

```bash
# Built-ins plus every manifest test
python3 tests/run_suite.py build/NESUltimateEmu.Tests \
  --manifest /path/to/nes-test-roms/test_roms.xml \
  --json build/accuracy-report.json

# PPU VBlank/NMI families only
python3 tests/run_suite.py build/NESUltimateEmu.Tests \
  --manifest /path/to/nes-test-roms/test_roms.xml \
  --suite ppu_vbl_nmi --suite vbl_nmi_timing

# Run one ROM explicitly as PAL regardless of header
python3 tests/run_suite.py build/NESUltimateEmu.Tests /path/to/test.nes --timing pal
```


## Phase 54 - PPU reset warm-up and regional emphasis correctness

- Added the RP2C02/RP2C07 post-reset register-write inhibit: `$2000`, `$2001`, `$2005`, and `$2006` ignore writes until the internal PPU reset signal releases. Ignored `$2005/$2006` writes do not advance the shared write toggle.
- `$2002`, `$2003`, `$2004`, `$2007`, and OAM DMA remain usable during the warm-up interval. Ignored writes still drive the PPU I/O/open-bus latch.
- NTSC release timing is modeled at 88,974 PPU clocks (~29,658 CPU clocks); PAL at 106,023 PPU clocks (~33,132 CPU clocks). Dendy uses the corresponding 312-line internal-reset release point.
- PAL and Dendy now swap PPUMASK emphasis bits 5/6 relative to NTSC: bit 5 controls green emphasis and bit 6 controls red emphasis; blue remains bit 7.
- Added PPU regressions for NTSC/PAL warm-up boundaries, write-toggle inhibition, live OAM access during warm-up, and regional red/green emphasis mapping.
- PPU warm-up timing state is serialized; save-state version is now **57**.
- Full built-in regression suite remains **12/12 PASS**.

## Phase 53 - Archive and signature-aware ROM loading

- Added direct loading of ROM images from archive members without writing temporary extracted ROM files.
- The Windows frontend uses the built-in `tar.exe`/libarchive backend, covering ZIP, 7-Zip, RAR where supported by the installed Windows build, TAR, TGZ/GZip, BZip2 and XZ-family archives.
- Archive members ending in `.nes` or `.fds` are preferred automatically; if none use a conventional extension, up to 64 members are inspected for iNES/NES 2.0 or headered FDS signatures.
- Native iNES/NES 2.0 loading is now signature-aware, so correctly formatted ROMs renamed to `.nes2`, `.bin`, `.rom`, or another extension can still load. Raw headerless FDS images still need the `.fds` extension.
- Archived FDS images look for `disksys.rom` beside the archive rather than in a temporary directory.
- Battery-backed archived games save beside the archive with an archive/member-specific `.sav` name, avoiding collisions between multiple games stored in one directory.
- Added `Cartridge::loadFromMemory()` so archive loading uses the exact same cartridge parser, mapper setup, database corrections, identity hashing, and battery logic as disk loading.
- Added a cartridge regression proving memory-loaded and disk-loaded copies of the same iNES image produce the same ROM identity.
- Built-in regression suite remains 12/12 PASS.


Phase 49 replaces MMC5 vertical-split horizontal positioning based on the emulator's PPU dot number with the hardware's nametable-fetch counter. MMC5 locates the split by counting background nametable tile fetches: the two end-of-line prefetched tiles occupy columns 0 and 1, the scanline dot-1 nametable fetch corresponds to screen column 2, and the sprite-fetch garbage nametable reads advance the counter through positions 32-39. The dots 337/339/1 synchronization sequence resets this counter for the next line. This makes the $5200 split threshold independent of coarse PPU scroll addresses and correctly preserves the up-to-7-pixel fine-scroll offset inherent to the hardware.

The mapper regression probe now verifies the prefetch offset directly: with a left split threshold of 3, dot 1 resolves to split column 2 while dot 9 resolves to column 3 and is outside the split. The new fetch-counter state is serialized, so save-state version is now **55**. Full built-in regression suite: **12/12 passing**.

# Phase 48 — MMC5 PPU-bus scanline/frame detector

Phase 48 removes the mapper-5 shortcut that drove MMC5 scanline IRQ and in-frame state directly from the emulator's known PPU scanline number. MMC5 now synchronizes from the PPU read stream itself: three consecutive reads of the same $2000-$2FFF nametable address arm synchronization, and the following PPU read detects the next scanline. The detector consumes the real dots 337/339/1 nametable sequence established by Phases 46-47, so scanline counter/IRQ timing is now derived from mapper-visible bus activity instead of hidden emulator state.

The MMC5 in-frame detector now also times out after three CPU clocks with no PPU read, immediately resets when rendering is disabled through $2001, and resets on NMI-vector reads ($FFFA/$FFFB) and OAM DMA writes ($4014). $5203 value zero remains the hardware special case that never creates a new IRQ pending condition, while $5204 reads still acknowledge pending IRQs. The mapper regression probe verifies two reads are insufficient, the third only arms synchronization, the fourth PPU read starts the frame, scanline compare IRQ generation, and the three-idle-CPU-clock frame teardown. Because the new detector has persistent mid-frame state, save-state version is now **54** and serializes the MMC5 synchronization fields. Full built-in regression suite: **12/12 passing**.

# Phase 47 — End-of-scanline PPU bus sequencing

Phase 47 completes the mapper-visible rendering fetch sequence at the end of each rendered scanline. Dots **337-340** are two unused nametable fetches; the existing background pipeline already emitted the first access at dot 337, and the PPU now emits the second access at dot 339 as well. The fetched bytes are intentionally discarded, but the addresses are still reported through the normal cartridge/mapper PPU-address path, which matters for hardware that synchronizes itself from the PPU fetch cadence (especially MMC5-style fetch-phase detection). The NTSC odd-frame skip still executes the dot-339 access before advancing directly to scanline 0.

The PPU conformance probe now records mapper-visible PPU bus addresses in headless builds and verifies exactly one nametable access at dot 337 and one at dot 339, at the same current nametable address, with no synthetic extra read on dots 338/340. No save-state fields changed.

# Phase 46 — Dot-timed sprite pattern fetch pipeline

Phase 46 replaces the remaining dot-260 batch sprite-pattern loader with an eight-slot fetch pipeline spanning PPU dots **257–320**. Each 8-dot sprite slot now performs the two garbage nametable accesses followed by low/high sprite pattern-table accesses on its own fetch phases, while attribute and X state are loaded from secondary OAM in parallel. This means mapper-visible PPU addresses (including MMC3-style A12 observation and other latch hardware) occur at the appropriate per-slot dots instead of all 16 sprite CHR reads sharing one synthetic timestamp. Dummy slots still perform tile-$FF pattern fetches but load transparent output state.

The PPU OAM regression now verifies that slot-0 attribute and X values become visible on their individual fetch phases rather than being preloaded by a batch call. The Phase 45 timed secondary-OAM evaluator remains the sole source of sprite selection/order. No save-state fields changed. Full built-in regression suite: **12/12 passing**.

# Phase 45 — Timed secondary-OAM evaluator unification

Phase 45 removes the renderer's separate dot-257 primary-OAM rescan. During dots 65-256, the existing per-dot sprite evaluator now writes accepted bytes directly into secondary OAM on the same even-dot cadence used by the hardware-visible evaluation bus. The sprite-fetch path therefore consumes the exact secondary-OAM contents produced by timed evaluation, keeping sprite selection/order, arbitrary OAMADDR starts, sprite-0 identity, overflow behavior, and `$2004` reads under one source of truth. Secondary OAM is committed to `$FF` at evaluation start to represent the completed dots 1-64 clear, while the current scanline's already-loaded sprite shifters/count remain untouched until the next sprite-fetch boundary.

The PPU OAM regression now proves the sprite list persists across dot 257 without a hidden batch rescan: rendering is disabled after timed evaluation has copied a sprite, then re-enabled only for the sprite-fetch bus read, which must still expose the copied tile byte. No save-state fields changed; evaluator state continues to be deterministically rebuilt from the serialized PPU state. Full built-in regression suite: **12/12 passing**.

# Phase 44 — Arbitrary OAMADDR sprite evaluation and sprite-0 identity

Phase 44 makes visible-scanline sprite evaluation begin from the live **OAMADDR ($2003)** value sampled at dot 65 instead of always assuming primary OAM address `$00`. Misaligned starting addresses are now interpreted as candidate Y bytes, out-of-range candidates use the PPU's realigning `+4` increment, and in-range candidates use sequential `+1` increments. The renderer's secondary-OAM build follows the same starting-address order so pixels and the timed evaluation bus remain consistent.

Sprite-0 identity is also no longer hard-wired to primary OAM entry 0. The dot-66 in-range result is latched for the next scanline and transferred to the first sprite output slot during sprite fetch, matching current RP2C02 hardware research. This means a sprite reached first from a nonzero OAMADDR can behave as sprite 0 for hit detection/priority. Regression coverage now verifies arbitrary evaluation start, dot-66 sprite-0 identity, and misaligned `+4` realignment. Full built-in regression suite: **12/12 passing**. Save-state version remains 53; the new evaluator fields are reconstructed from existing serialized PPU state.

# Phase 43 — Rendering-time OAM write protection and OAMADDR reset

Phase 43 tightens RP2C02 OAM behavior during rendering. Writes to **OAMDATA ($2004)** on visible and pre-render scanlines while background or sprite rendering is enabled no longer modify primary OAM; OAM DMA inherits the same behavior because it writes through $2004. During sprite-fetch dots **257–320**, **OAMADDR ($2003)** is now continuously driven back to `$00`, matching the hardware-visible post-render address behavior. The existing Phase 42 rendering-time OAMDATA read-bus model remains intact, and the OAM regression probe now verifies write suppression and sprite-fetch OAMADDR reset. The built-in regression suite remains **12/12 passing**.

# Phase 42 — Rendering-time OAMDATA read bus

Phase 42 made **OAMDATA ($2004)** reads during active rendering expose the PPU's internal sprite OAM bus instead of `OAM[OAMADDR]`: secondary-OAM clear returns `$FF`, sprite evaluation exposes the primary-OAM read latch, sprite fetch exposes selected secondary-OAM bytes, and the background-prefetch interval exposes secondary OAM byte 0. Outside rendering, normal OAMADDR reads and attribute-bit masking remain unchanged.

# Phase 41 — Console timing override and multi-region handling

Phase 41 adds a persistent frontend console-timing selector with **Auto / NTSC / PAL / Dendy** modes. Auto continues to honor cartridge/header/database timing, while forced modes apply the selected timing coherently to the Bus, PPU, and APU and restart the loaded machine so no mixed-region state survives a switch. NES 2.0 multi-region images are now called out in the frontend instead of silently appearing as ordinary NTSC titles, and the shipped configuration records the timing override explicitly. The built-in regression suite remains 12/12 passing.

# Phase 40 — Audio output + debugger removal

Phase 40 removes the dormant debugger source files completely and raises final host-audio output without changing NES channel or expansion-chip balance. The APU output-conditioning headroom was increased from 0.60 to 0.85, while the shipped frontend configuration now starts at 150% master volume. Final float output remains clamped to the valid -1.0..1.0 range. The built-in regression suite remains 12/12 passing.

# Phase 39 — Mapper 121 / Kǎshèng A9711/A9713

Phase 39 adds iNES mapper 121 on top of the existing MMC3 core. The implementation covers the four-byte `$5000-$5FFF` protection array, the `$8001/$8003` bit-reversed protection latch and direct `$A000/$C000/$E000` PRG overrides, A9713 `$5180` outer 256 KiB PRG/CHR selection, and A9711 512 KiB CHR wiring where PPU A12 directly selects CHR A18. Mapper 121 reset and save-state state are included. The built-in regression suite remains 12/12 passing. Save-state version remains 53 because mapper 121 was previously unsupported and no existing mapper payload changed.

## Phase 38 — Mapper 123 / Kǎshèng H2288

Phase 38 added iNES mapper 123 using the existing MMC3 core plus the H2288-specific `$5800` NROM override and scrambled MMC3 bank-register indices.

## Phase 36 - Mapper 111 collision completion

- Mapper 111 now dispatches by cartridge CHR memory type: CHR-ROM images use the historical Ninja Ryukenden non-serialized MMC1 variant, while CHR-RAM images use GTROM/Cheapocabra.
- The historical branch accepts direct mapper-register writes (no MMC1 serial shift sequence), preserves standard MMC1 PRG/mirroring modes, and exposes the full 256 KiB CHR-ROM range.
- The GTROM flash and 32 KiB cartridge PPU-RAM implementation from Phase 35 remains unchanged.
- Mapper regression coverage now verifies immediate PRG banking, CHR banks above the normal MMC1 128 KiB ceiling, mirroring, and continued GTROM dispatch.
- Built-in regression suite remains 12/12 passing.
- Save-state version remains 53 because the historical mapper-111 branch was previously unsupported and the existing GTROM serialized payload did not change.


## Phase 30 - Hash database + Mapper 53

- Added `src/core/RomDatabase.hpp`, a conservative CRC32-based cartridge-resolution layer for ambiguous legacy dumps. It is intentionally payload-driven (no filename heuristics) and carries resolved physical-board variants through `MapperConfig`.
- Added Mapper 53 / Supervision 16-in-1, including its `$6000` expansion PRG window, outer/inner 16 KiB banking, H/V mirroring control, reset/state support, and both known PRG dump orders.
- The EPROM-first Supervision ordering is detected by CRC32 `0x63794E25` over the first 32 KiB, matching the established Nestopia identification rule.
- Mapper/cartridge regression coverage now verifies both physical orderings and an end-to-end synthetic ROM whose first 32 KiB matches the known CRC signature.
- Built-in regression suite remains 12/12 passing.

## Phase 29 mapper compatibility

Added mapper 54 (Novel Diamond), mapper 55 (BTL-MARIO1-MALEE2), mapper 56 (Kaiser KS-202), mapper 57 (GK-54/GK-L01A family), and mapper 59 (UNL-D1038). Mapper 56 includes its VRC3-like nibble IRQ, four 8 KiB PRG registers, eight 1 KiB CHR registers, optional PRG-ROM window at $6000, and mirroring control. Mapper 55 models the extra 2 KiB PRG-ROM and mirrored 2 KiB RAM windows. Mapper 57/59 currently expose mapper DIP switches at their hardware-default value (0); frontend DIP-switch selection remains a future peripheral/UI item. Mapper 53 remains deliberately unsupported because known Supervision dumps use incompatible PRG orderings that require ROM-hash/database identification for reliable selection. The built-in regression suite remains 12/12 passing.


## Phase 28 mapper compatibility

Added mapper 51 (Ball Games 11-in-1), mapper 52 (Realtec 8213/Mario Party 7-in-1), mapper 61 (20-in-1), and mapper 63 (CH-001). Mapper 52 uses the MMC3 core plus its locked outer PRG/CHR bank register and WRAM overlap. Mapper 63 models its address-selected open-bus window. All additions are covered by the built-in mapper conformance probe.
# Phase 19 — Mapper 176 completion

- Mapper 176 NES 2.0 submappers 0-5 are now implemented.
- FS005/FS006 (submapper 2): swapped MMC3 bank registers 6/7, banked 32 KiB WRAM/protection aperture, mixed CHR-ROM/CHR-RAM selection, one-screen mirroring, extended MMC3, and PRG A21-A25 outer wiring.
- JX9003B (submapper 3): independent PRG A21-A24 and CHR A21-A24 high-bank registers with the Fxx7 outer-register decode.
- Submapper 4: PRG A21 sourced from CHR-base bit 7.
- HST-162 (submapper 5): upper PRG A19-A24 latch at $4800-$4FFF.
- Cartridge RAM routing now permits mapper-defined RAM windows below $6000, required by FS005 protection software.
- Save-state version is 50.
- Built-in regression suite remains 12/12 passing.

# Phase 17 - J.Y. Company ASIC (Mappers 35/90/209/211)

Added a shared J.Y. Company ASIC mapper core for iNES mappers 35, 90, 209, and 211. The implementation covers 32/16/8 KiB PRG modes (including reversed-bit 8 KiB mode), 8/4/2/1 KiB CHR banking, mapper-90 outer 512 KiB PRG and CHR regions, MMC4-like CHR latches, hardware multiplier/accumulator registers, flexible IRQ counting from CPU M2 / PPU A12 / PPU reads / CPU writes, $6000 PRG-ROM mapping, and mapper-controlled mirroring. Mapper 90 suppresses ROM/extended nametables as required by its PCB jumper; mapper 209/211 support ROM nametable reads with writes correctly landing in CIRAM. Mapper 35 is treated as the WRAM-declared duplicate of mapper 209. The built-in regression suite remains 12/12 passing. Save-state version remains 49 because no previously-supported mapper payload changed.

# Phase 16 - Mapper 45 / GA23C Multicart Support

Mapper 45 is now implemented as an MMC3-derived GA23C multicart board. The four sequential outer registers at `$6000` apply the documented PRG/CHR AND/OR masks around normal MMC3 banking, register #3 locks further outer writes, and `$6001` resets/unlocks the outer layer. Mapper 45 state is serialized without changing the save-state format of previously supported mapper families. The mapper conformance probe now covers outer PRG/CHR banking, register locking, reset/unlock behavior, and save-state restoration.

# Phase 15 — Mapper 185 protection + Mapper 228 Action 52

- Added NES 2.0 mapper 185 submappers 4-7 with CNROM protection-latch CHR enable behavior and mandatory AND bus conflicts. Legacy submapper 0 remains explicitly unsupported until its PPU open-bus heuristic can be modeled faithfully.
- Added mapper 228 for Action 52 / Cheetahmen II with address-encoded PRG/CHR banking, 16/32 KiB PRG modes, H/V mirroring, and the physically absent Action 52 PRG chip-2 open-bus selection.
- Extended mapper conformance coverage for both boards.
- Built-in regression suite remains 12/12 passing.
- Save-state version remains 49; these are newly supported mapper payloads and do not change serialization for previously supported cartridges.

# Phase 14 — Reset lifecycle + Mapper 116.3

- Added mapper-visible hard/soft reset lifecycle through Bus -> Cartridge -> Mapper.
- Implemented NES 2.0 mapper 116 submapper 3 (Mario 5-in-1) reset-based outer ROM selection.
- Soft Reset advances 5 game windows (256 KiB first game, then four 128 KiB games); cold power-on returns to game 0.
- Mapper 116.3 reset selector is included in save states.
- Save-state version: 49.
- Built-in regression suite remains 12/12 passing.

# Phase 13 — Mapper 116 Huang-1/Huang-2 composite support

Mapper 116 is now implemented as a true three-personality board with independent MMC1, MMC3, and VRC2b state selected through the $4100 supervisor register. Submapper 0 resets only the MMC1 serial latch when entering MMC1 mode, submapper 1 preserves it, and submapper 2 implements Huang-2's altered MMC1 PRG-bank wiring. The VRC2 CHR registers power up to $FF and supervisor bit 2 supplies CHR A18. Submapper 3 remains explicitly unsupported until reset-count multicart selection is wired into the mapper reset lifecycle. The mapper conformance probe covers mode-state preservation, VRC2 power-up banking, outer CHR selection, Huang-1 latch reset, and Huang-2 PRG wiring. The full built-in regression gate remains 12/12 passing.


## Phase 11 - MMC3 multicart expansion

Added hardware-specific support for iNES mappers 37, 47, and 49 on top of the existing MMC3 core. Mapper 37 implements its 3-bit outer latch and asymmetric 64/128 KiB PRG windows, mapper 47 selects 128 KiB PRG/CHR halves, and mapper 49 implements both its 32 KiB PRG mode and MMC3 mode with WRAM-gated outer-register writes. Mapper-state serialization was extended for the outer latch; frontend save-state version is now 47. The mapper conformance probe verifies PRG/CHR outer banking and mapper-49 write gating.
# Phase 10 — Mapper completeness: MMC6, UNROM-512 variants, Mapper 15/31

This phase extends the Phase 9 mapper compatibility work with NES 2.0 MMC6 support (mapper 4 submapper 1), UNROM-512 submapper 0-4 banking/mirroring/LED address decoding and bus-conflict selection, plus new mapper 15 and mapper 31 implementations. Flash-enabled UNROM-512 images are deliberately reported as unsupported until SST39SF040 erase/program persistence is implemented. MMC6 changes the mapper save-state payload, so the frontend save-state format is Version 46.

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

### External APU conformance gate
The headless runner now supports both generations of blargg APU tests: the later `$6000` protocol and the 2005-07-30 frame-counter suite via `--legacy-apu`. CMake options `NES_BLARGG_APU_ROM_DIR` and `NES_BLARGG_LEGACY_APU_ROM_DIR` register these third-party ROMs with CTest when paths are supplied; ROMs are never bundled. The legacy adapter waits for the ROM's terminal report loop before reading its zero-page result code, avoiding false completion on intermediate diagnostic values.


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

## Phase 8: regional timing

The emulator now derives console timing from NES 2.0 byte 12 and propagates the selected mode through the cartridge, bus, PPU, APU, headless runner, and frontend frame pacing. Supported timing modes are NTSC, licensed PAL (RP2A07/RP2C07), and Dendy-style PAL famiclone timing. NES 2.0 multi-region images currently default to NTSC while retaining a multi-region flag for a later user override.

### Phase 9 - NES 2.0 submappers and bus conflicts

Mapper writes now support cartridge-level AND bus conflicts where the selected board requires them. NES 2.0 mapper 2/3/7 submapper 2 explicitly enables conflicts and submapper 1 disables them, while legacy iNES behavior is preserved. Fixed board distinctions include mapper 32 submapper 1 (Major League), mapper 71 submapper 1 (Fire Hawk), mapper 78 submappers 1/3 (Cosmo Carrier/Holy Diver), and mapper 85 submappers 1/2 (VRC7b/VRC7a address-line decoding; VRC7b expansion audio disabled). Color Dreams mapper 11 PRG/CHR select bitfields were also corrected. The built-in regression gate remains 12/12 passing.

### Phase 18 - Mapper 176 / FK23C foundation

Mapper 176 now supports the foundational 8025/FK23C variants represented by
NES 2.0 submappers 0 and 1. This includes MMC3-compatible IRQ timing, 128/256
KiB and large MMC3 PRG windows, NROM/UNROM modes, FK23C extended MMC3 bank
registers, and submapper-1 CNROM/NROM CHR modes. Later Waixing/JX9003/HST
submappers 2-5 remain deliberately gated until their additional RAM and high
address wiring is implemented.

## Phase 20 - Mapper 268 CoolBoy / MindKids foundation

Added NES 2.0 mapper 268 support for submappers 0-3 (AA6023/AA6023B): oversize MMC3 and GNROM/NROM banking, six outer registers, submapper-dependent $5000/$6000/$7000 register decode, lockout, WRAM overlay, CHR-RAM banking, mirroring, MMC3 IRQs, reset, and save-state support. Submappers 4-11 remain deliberately unsupported pending their incompatible high-address, CHR-ROM/RAM, protection, and mirroring wiring.

## Phase 22 - Mapper 215 / UNL-8237

Added NES 2.0 mapper 215 submappers 0 and 1 (UNL-8237 / UNL-8237A). The MMC3-derived core now supports the $5000 NROM override, $5001 outer PRG/CHR banking, $5007-selectable address and bank-index scrambling patterns, submapper-specific overlapping outer-bank wiring, and reset of the outer bank register on an M2 interruption/reset. Unsupported submapper values remain gated.


## Phase 23 - SL-5020B / mapper 12.0

Mapper 12 submapper 0 is supported as the Gouder SL-5020B board: MMC3A-compatible IRQ behavior plus the external `$4132` CHR-A18 selector. The Front Fareast Magic Card variants (mapper 6, mapper 8, mapper 12.1, and mapper 17) remain deliberately gated until their RAM-cartridge memory and trainer startup model is implemented.

## Phase 24 - Front Fareast Magic / Super Magic Card RAM cartridges

Mapper 6, mapper 8, mapper 12 submapper 1, and mapper 17 now share a Front Fareast RAM-cartridge implementation instead of being treated as ordinary ROM boards. Extracted PRG/CHR payloads are copied into writable cartridge DRAM, the documented $42FC-$43FF/$4500-$451B banking registers are initialized for each image type, and latch, 2M/4M, 1 KiB CHR, MMC4-style CHR latch, mirroring, WRAM, scratch RAM, and mapper-17 IRQ paths are represented. Mapper 8 remains the mapper-6 mode-4 synonym. Hard-reset trainer startup is also represented: mapper 6/8/12.1 performs the specified implicit JSR $7003 before the original reset handler, while mapper 17 jumps to the trainer address selected by NES 2.0 submapper 0-3. These copier RAM cartridges are deliberately excluded from battery persistence.

The Bus save-state payload now includes whether a cold-reset trainer bootstrap is still pending, so the frontend save-state version is 52.


## Phase 25 - legacy mapper 40-43 compatibility

Added mapper 40 (NTDEC 2722 base board), mapper 41 (Caltron 6-in-1), mapper 42 FDS conversions, and mapper 43 TONY-I/YS-612. Regression coverage verifies banking, mirroring, reset behavior, and M2 IRQ timing. Cartridge ROM mapping can now extend into $4020-$5FFF after mapper-register/RAM priority, which is required by mapper 43's $5000 expansion ROM. Mapper 40 submapper 1 remains deliberately unsupported pending fuller NTDEC 2752 outer-board validation.


## Phase 26 - Action 53 and conventional mapper expansion

Added mapper 28 (Action 53), mapper 36 (TXC 01-22000-400), mapper 44 (Super Big 7-in-1 MMC3 multicart), and mapper 46 (Rumblestation 15-in-1). Mapper 28 implements the user/supervisor register model, 32/64/128/256 KiB outer PRG windows, 32 KiB and fixed-half PRG modes, CHR-RAM banking, and H/V/one-screen mirroring. Mapper 36 implements the TXC PP/RR register machine, inversion/increment modes, readable RR bits, explicit PRG latch, and CHR banking. Mapper 44 reuses the MMC3 core with its seven outer PRG/CHR blocks (block 7 aliases block 6), and mapper 46 implements its two-stage outer/inner PRG+CHR registers. The built-in regression suite remains 12/12 passing.

## Phase 27 mapper additions

Added mapper support for iNES 38 (UNL-PCI556), 39 (Study and Game 32-in-1), 50 (N-32 SMB2J conversion), 58, 60 (reset-select 4-in-1), and 62. These are covered by the built-in mapper conformance probe.

## Phase 31 - hash-resolved legacy cartridge metadata

The ROM database now resolves verified PRG+CHR payload CRCs before mapper construction and RAM allocation.  Entries can override mapper/submapper, mirroring, timing, volatile/nonvolatile PRG/CHR RAM sizes, and mapper-specific board variants.  The initial expanded set covers Holy Diver (mapper 78.3), Uchuusen: Cosmo Carrier (78.1), and Family Circuit '91 (Namco 175 / mapper 210.1 with 2 KiB battery RAM), while retaining the mapper-53 physical-order signature from Phase 30.  Filenames are never used.

## Phase 32 - Legacy mapper gap closure

Added and regression-tested mapper support for:

- Mapper 81 (NTDEC N715021 / Super Gun): address-latched 16 KiB PRG and 8 KiB CHR banking.
- Mapper 103 (Doki Doki Panic FDS conversion): split fixed/switchable PRG-ROM layout plus read-selectable 8 KiB RAM windows at $6000-$7FFF and $B800-$D7FF; writes always reach RAM.
- Mapper 107: simple 32 KiB PRG + 8 KiB CHR latch mapper.

The built-in regression suite remains 12/12 passing.

## Phase 33 - mapper 106 / 108 / 112 compatibility

Phase 33 closes three more legacy mapper gaps:

- Mapper 106 (discrete SMB3 bootleg): eight 1 KiB CHR banks, four 8 KiB PRG banks, H/V mirroring, and the hardware 16-bit M2 up-counter IRQ.
- Mapper 108 (Whirlwind Manu FDS conversions): NES 2.0 submappers 1-4, plus conservative legacy submapper-0 PCB inference from CHR type/size and mirroring.
- Mapper 112 (NTDEC/Asder): two switchable 8 KiB PRG banks, fixed final 16 KiB, 2K+2K+1K+1K+1K+1K CHR layout, and H/V mirroring.

The built-in regression suite remains 12/12 passing. Mapper 105 (NES-EVENT timer/DIP hardware) and mapper 111 (GTROM self-flash persistence) remain intentionally unsupported for a dedicated implementation phase.

## Phase 34 - NES-EVENT / mapper 105

Phase 34 adds mapper 105 (Nintendo NES-EVENT / Nintendo World Championships 1990) as a dedicated MMC1-derived board implementation rather than treating it as ordinary SxROM.

Implemented behavior includes the cold-boot fixed first 32 KiB window, event-bank selection through the MMC1 CHR0 register, normal MMC1 banking in the upper 128 KiB region, MMC1 mirroring/CHR-RAM/PRG-RAM behavior, the board's 30-bit CPU-M2 event timer, IRQ acknowledge/reset through CHR0 bit 4, and the official tournament DIP default ($28000000 timer edge). Mapper state includes the serial MMC1 state and event timer so save states resume deterministically.

The mapper regression suite now verifies mapper-105 boot mapping, event-bank unlock, a near-threshold 30-bit IRQ edge, and IRQ acknowledge. The full built-in suite remains 12/12 passing.

Mapper 111 (GTROM) remains intentionally unsupported. Correct GTROM support requires not only its $5000/$7000 bank latch and 32 KiB PPU RAM arrangement, but SST39SF040 self-flash program/erase command handling and persistent modified PRG contents. It will be enabled only when that persistence path is implemented.


## Phase 35 - Mapper 111 / GTROM

Mapper 111 now supports GTROM/Cheapocabra cartridges with 512 KiB banked PRG flash, the 32 KiB shared cartridge PPU RAM used for pattern tables and four-screen/bonus nametable RAM, and the SST39SF040 software command sequences for byte programming, 4 KiB sector erase, chip erase, and software ID mode. Mapper-owned flash contents are persisted in the cartridge save file and included in save states. The historical unrelated mapper-111 Chinese MMC1 variant is deliberately not identified as GTROM when CHR-ROM is present.

## Phase 37 - Mapper 104 / 117 / 120 gap closure

- Mapper 104 (Pegasus 5-in-1): 256 KiB outer PRG selector, 16 KiB inner selector, outer-register lock, and power-on-only register clearing.
- Mapper 117 (Future Media): four independent 8 KiB PRG banks, eight 1 KiB CHR banks, H/V mirroring, and qualified PPU-A12 IRQ counting.
- Mapper 120 (FDS conversion): $41FF-selected 8 KiB PRG window at $6000-$7FFF with the upper 32 KiB PRG area fixed.
- Mapper regression coverage added for banking, mirroring, reset/lock behavior, and mapper-117 IRQ timing.
- Full built-in regression suite: 12/12 passing.

## Phase 42 - rendering-time OAMDATA bus accuracy

Phase 42 improves later-2C02 OAMDATA ($2004) read behavior during rendering. Instead of always returning primary OAM at OAMADDR, rendering-time reads now expose the PPU's internal sprite OAM bus: dots 1-64 return the secondary-OAM clear value ($FF), dots 65-256 expose the sprite-evaluation read latch, dots 257-320 expose the secondary-OAM sprite-fetch sequence, and dots 321-340/0 expose the first secondary-OAM byte during background prefetch. Outside rendering, normal OAMADDR reads and attribute-bit masking are preserved.

The PPU open-bus/OAM regression probe now covers clear, evaluation, sprite-fetch, and prefetch OAMDATA bus phases. No serialized fields changed, so the save-state version remains 53. Full built-in regression suite: 12/12 passing.


## Phase 51 — APU write-relative frame sequencer timing

Phase 51 fixes the APU frame-counter timing exposed by the hardware-verified `blargg_apu_2005.07.30` results reported against Phase 50: `01.len_ctr=04`, `04.clock_jitter=03`, `05.len_timing_mode0=03`, `06.len_timing_mode1=03`, `07.irq_flag_timing=03`, `08.irq_timing=03`, `10.len_halt_timing=03`, and `11.len_reload_timing=03`.

The Phase 50 `$4017` parity inversion was reverted because it broke the previously passing `$4017=$80` immediate half-frame behavior. More importantly, the frame sequencer no longer treats the delayed 3/4-clock divider reset as time zero for the new sequence. Hardware-observable frame events are timed from the CPU write to `$4017`; when the delayed divider reset lands, the sequencer now preserves the 1 or 2 write-relative clocks already elapsed. This produces the NTSC hardware-test boundaries of quarter-frame 7459, half-frame 14915, quarter-frame 22373, and frame-IRQ assertions at 29830/29831/29832 clocks after the write while retaining the delayed mode-switch visibility.

The APU regression probe now checks the specific 3/4-clock divider polarity, the 14915-clock first half-frame boundary, and the 29830-clock first IRQ-flag edge. Length halt/reload collision ordering remains CPU write first, frame clock second, deferred length-control application last, so once the frame edge is on the correct CPU cycle the documented same-cycle collision semantics remain intact. Save-state version is 56 because a new pending write-relative frame-sequencer field is serialized. Full built-in regression suite: **12/12 passing**.

## Phase 50 — APU $4017 frame-counter parity correction

Phase 50 addresses the blargg APU frame-counter failures reported against Phase 49 (`04.clock_jitter=05`, mode 0/1 first length clock late, frame IRQ flag/IRQ late, and length halt/reload collision timing late).

The root issue was not the frame-step interval table itself. The emulator already generated a 3-or-4 CPU-clock delayed $4017 reset, but assigned those two delays to the wrong CPU/APU divider phase. That makes one jitter polarity correct and the opposite polarity one clock late, which then shifts every test synchronized to that phase.

Changes:
- Corrected the $4017 delayed-reset parity mapping while preserving the documented 3/4 CPU-clock behavior.
- Kept IRQ-inhibit (`$4017` bit 6) immediate.
- Kept the existing NTSC/PAL frame-step decode positions unchanged rather than compensating with globally shifted quarter/half-frame clocks.
- Strengthened `ApuConformanceProbe` to require the correct delay for each specific divider phase instead of accepting the unordered pair `{3,4}`.
- No save-state payload change was required.

Built-in regression result after the correction: **12/12 PASS**.

External blargg APU ROMs should be rerun against this phase. The reported failure cluster is expected to improve together because all seven failures were downstream of the same $4017 parity alignment.

## Phase 52 - APU frame-counter jitter and 5-step wrap timing

Phase 52 fixes the two remaining failures reported by the hardware-verified
blargg APU frame-counter ROMs after Phase 51:

- `$4017` frame-counter jitter is no longer canceled by compensating the slower
  divider-reset phase with a different starting counter value. The 3/4-clock
  reset alignment now remains visible as the expected one-CPU-clock difference
  in subsequent frame events.
- The NTSC 5-step sequencer now preserves the extra clock at the sequence wrap.
  This places the third regular length-counter clock on the CPU-read boundary
  expected by `06.len_timing_mode1` (playing at 52197, silent at 52198).
- The internal APU probe now verifies both opposite-phase frame IRQ timing and
  the 5-step third-length boundary, preventing either bug from regressing.

No save-state layout change was required. The complete built-in regression suite
passes 12/12 after these changes.



## Phase 57 - Complete $2002 VBlank/NMI suppression window

- Completed the NTSC `$2002` race model around the VBlank-set edge at scanline 241 dot 1.
- A status read one PPU dot before VBlank still reads bit 7 clear and suppresses the flag/NMI for that frame.
- A status read on the exact VBlank-set dot now reads bit 7 set, immediately clears it, and suppresses the too-short NMI pulse. This scheduler can service the CPU read before the PPU dot body, so the returned VBlank bit is synthesized for that simultaneous hardware case without leaving the internal flag asserted.
- A status read one PPU dot after VBlank reads bit 7 set and can still cancel an NMI edge that has not yet been sampled by the CPU.
- Two or more PPU dots after the set edge are now outside the cancelable-NMI window; the previous implementation left that window open one dot too long.
- Tightened the internal NMI cancellation delay from two following PPU dots to one.
- Expanded `PpuConformanceProbe` to distinguish exact-dot, +1-dot, and +2-dot behavior with NMI output enabled, protecting both the status-bit result and interrupt cancellation boundary.
- No save-state layout change was required; the existing serialized NMI-delay byte remains compatible.
- Full built-in regression suite remains **12/12 PASS**.

## Phase 56 - PPU VBlank race accuracy

- `$2002` reads on the PPU dot immediately before VBlank assertion now suppress that frame's VBlank flag/NMI, matching the documented hardware race.
- Added a headless PPU regression that compares normal VBlank entry with the dot-0 suppression case.
- No save-state format change.

## Phase 58 - PPUCTRL/NMI edge conformance

Phase 58 strengthens VBlank/NMI regression coverage around writes to PPUCTRL ($2000) while VBlank is active. The PPU's NMI output is modeled as the edge-sensitive conjunction of the VBlank flag and PPUCTRL bit 7, so enabling NMI during an already-active VBlank creates an edge, disabling it inside the short cancellation window can withdraw an asynchronous edge, and disabling/re-enabling can create a fresh edge in the same VBlank.

The headless CPU exposes its NMI pending/polled latches only under NES_HEADLESS so the PPU conformance probe can verify CPU-visible interrupt behavior without adding debugger functionality to the normal build. The new probe covers disabled-at-VBlank, enable-during-VBlank, immediate cancel, re-enable, +1-dot disable cancellation, and +2-dot committed-edge behavior. No save-state format change was required. Built-in regression gate: 12/12 passing.


## Phase 59 - Pre-render status-clear boundary regression

- Added single-dot regression coverage for the pre-render PPUSTATUS clear event.
- Verifies VBlank, sprite-0 hit, and sprite-overflow remain latched through dot 0 and clear together when pre-render dot 1 executes.
- Verifies a `$2002` read before the hardware clear removes only VBlank; sprite hit/overflow remain set until the dot-1 clear.
- No emulation behavior or save-state format changed in this phase; the existing core already matched the documented boundary.

# Phase 64 - PPUMASK Rendering Propagation Delay

Phase 64 separates the CPU-visible PPUMASK register from the rendering pipeline's effective background/sprite enable signals. Writes to PPUMASK bits 3-4 are accepted immediately, but their rendering effect propagates through a short PPU-dot pipeline before background/sprite fetch, pixel generation, rendering-time PPUDATA behavior, internal OAM-bus behavior, odd-frame rendering decisions, and rendering-start OAM corruption see the new state. External `ppu_vbl_nmi/10-even_odd_timing.nes` validation later refined the deterministic model from four dots to three.

The left-edge clip, greyscale, and color-emphasis bits remain tied to the register value; only background/sprite rendering enable is delayed. A new PPU conformance regression verifies both enable and disable remain at the old effective state for three dots and change on the fourth. Existing PPU/OAM tests were updated where they intentionally require a settled rendering state rather than an in-flight PPUMASK transition.

The effective render mask, pending target, and propagation countdown are serialized for deterministic mid-transition save states. Save-state version is now **59**. Full built-in regression suite: **12/12 passing**.

Hardware observations indicate the visible rendering-toggle delay is only a few pixels and can vary with CPU/PPU alignment. The original Phase 64 four-dot baseline was subsequently refined to three dots after the hardware-validated `10-even_odd_timing.nes` test showed the odd-frame skip threshold one dot early under the four-dot model. Alignment-dependent variation remains future work where hardware tests require it.


## Phase 65 - delayed PPUADDR transfer

- The second `$2006` write now captures the completed temporary VRAM address and transfers it to active `v` after a deterministic three-PPU-dot propagation delay instead of updating `v` immediately.
- Mapper-visible PPU address observation occurs when the delayed transfer reaches `v`, not at the earlier CPU register write.
- The first `$2006` write still modifies only `t`/the shared write toggle.
- Added a PPU conformance boundary test proving two dots of hold followed by application on the third dot.
- Same-dot `$2006`/rendering increment bus-conflict hybrids remain intentionally deferred until a hardware-ROM test can define the bit-level collision behavior.
- Save-state version 60 serializes the in-flight PPUADDR transfer.


## Phase 68 - CPU/PPU I/O alignment

- Preserves the established 5-dot rendering-time PPUDATA increment alignment as the deterministic default.
- Adds an explicit alternate CPU/PPU master-clock I/O phase observed by hardware tests, where the same `$2007` increment reaches `v` after 6 PPU dots.
- The alignment class is serialized in PPU save states so an in-flight timing relationship restores deterministically.
- PPU conformance now verifies both the 5-dot and 6-dot paths.
- Save-state version is 62.

## Phase 71 - AccuracyCoin RMW DMA and hidden sprite pipeline

Phase 71 is driven by the Phase 70 AccuracyCoin results.

- `$4014` writes now create a pending OAM-DMA request instead of halting the CPU immediately. RDY acquisition waits for a CPU read cycle, so read-modify-write instructions such as `INC $4014` complete both writes; the second write replaces the requested page and exactly one OAM DMA starts afterward.
- Sprite X counters and sprite pattern shifters now advance through visible dots whenever either rendering layer is active. Disabling sprite output in PPUMASK hides sprites from the compositor but does not freeze the internal sprite pipeline, matching AccuracyCoin's background-only rendering test.
- Save-state version is 64 because pending OAM-DMA request/page state is serialized.
- Built-in regression gate remains 12/12 PASS.

## Phase 79 - 2A03 internal/external data-bus isolation

Phase 79 keeps the Phase 78 DMC/OAM arbitration baseline and splits the CPU data
path into the external pin/open-bus latch and the 2A03 internal data latch.
Normal CPU reads/writes feed both latches. DMC sample GETs drive only the
external latch, while $4015 status reads drive only the internal latch and leave
D5 open from the previous internal value. This targets AccuracyCoin CPU Behavior
2 / Internal Data Bus without disturbing the now-passing DMC+OAM overlap or
implied-dummy-read behavior.

The DMC/APU conflict regression now explicitly checks that a $C015 DMA byte with
D5=1 remains visible externally while D5 stays clear on a preloaded internal
latch. Save states serialize both latches.

## Phase 82 - DMC DMA abort cycle instrumentation

Phase 82 intentionally does not change DMC abort behavior. It instruments the
working Phase 81/79 DMA core so the remaining AccuracyCoin Explicit DMA Abort
and Implicit DMA Abort code-2 failures can be diagnosed from exact CPU-cycle
traces instead of further timing heuristics.

During each cold boot, the emulator creates/truncates `dmc_dma_trace.log` in
the process working directory. The trace records DMC request/stop events,
$4015 enable/disable writes, HALT acquisition, dummy/alignment/GET transitions,
CPU read/write phase and address, OAM-DMA state, and the external/internal CPU
data-bus latches. The logger flushes each event so a trace survives even if the
emulator is stopped immediately after the AccuracyCoin test.

For AccuracyCoin diagnosis, run Page 13 through Explicit DMA Abort and Implicit
DMA Abort, exit the emulator, and inspect/share `dmc_dma_trace.log`. Phase 82
keeps the existing DMA scheduler behavior unchanged; the built-in regression
gate remains 13/13 PASS.

## Phase 83 - DMC abort matrix correction

Phase 83 uses the Phase 82 cycle trace to distinguish three DMC stop outcomes that were previously conflated: an already-completed normal reload, a committed reload that still performs the full four-cycle DMA, and the one-cycle aborted reload pulse. The explicit-abort trace showed the first six matrix entries already produced full four-cycle DMAs, entry 6 produced the required three-cycle write-delayed DMA, entries 8-9 produced the required one-cycle pulses, and entries 10-15 produced zero cycles; only entry 7 was misclassified as a one-cycle abort. Phase 83 preserves that entry as a normal reload. The implicit pre-mid-1990 abort window is also narrowed from the four observed positions in Phase 82 to the two positions required by AccuracyCoin's behavior-2 matrix.


## External conformance gate

The headless runner supports AccuracyCoin directly with `--accuracycoin`. The current 141-test baseline is **141/141**. Use `--json` for machine-readable results and `--min-pass 141` to make the current score a non-regression threshold. See `docs/TIER0_EXTERNAL_CONFORMANCE.md`.

### Tier 0 external conformance expansion

The optional CMake test gate can now register legacy blargg NTSC PPU tests,
branch timing, sprite-0 hit, sprite overflow, and VBL/NMI timing suites in
addition to AccuracyCoin and the APU suites. External ROMs remain unbundled.
See `docs/TIER0_EXTERNAL_CONFORMANCE.md` for cache variables and the current
baseline, including the explicitly tracked one-PPU-clock-early NMI discrepancy.

### AccuracyCoin milestone: 141/141

The default NTSC RP2C02G profile now passes all 141 tests in AccuracyCoin. The
final PPU bus corrections include a six-dot rendering-time PPUDATA refill phase
for this scheduler alignment, six-dot rendering-time PPUDATA increment phase,
dot-257 dummy nametable ALE before horizontal t->v reload, explicit AD0-AD7
external-latch behavior for ALE+/RD collisions, and five-dot PPUADDR t->v
propagation for hybrid-address behavior. The optional CTest AccuracyCoin floor
is 141.


### DMC DMA external conformance

The extended external gate now includes blargg's `dmc_dma_during_read4` suite and both `sprdma_and_dmc_dma` variants when present under `NES_EXTENDED_TEST_ROOT`. The DMC memory reader keeps the delayed `$4015` enable boundary separate from load-vs-reload classification. A reload that becomes necessary while that enable is still propagating waits for the boundary and then acquires on the next GET phase; ordinary reloads remain PUT-aligned. This satisfies AccuracyCoin DMC timing tests L/M/N while preserving the older `$2007`/`$4016` DMA-conflict behavior.
