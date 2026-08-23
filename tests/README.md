## Phase 75 CPU RDY regression

`CpuConformanceProbe.cpp` now verifies the AccuracyCoin unstable-store RDY quirk through the real DMC scheduler: an SHX that would normally apply `(H+1)` must become an ordinary STX-style write when DMC acquires RDY on its provisional indexed read. The same probe also checks the exact cycle-2 PC bus address for `ASL A`.

# NESUltimate Headless Regression Harness

## Reproducible headless build

The headless regression runner can be built without SDL or ImGui on Windows, Linux, or macOS with CMake 3.20+ and a C++17 compiler:

```bash
cmake -S . -B build -DNES_BUILD_HEADLESS_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

`ctest` runs `NESUltimateEmu.Tests --self-test` and succeeds only when every built-in probe passes. CMake copies the three bundled DMC probe ROMs beside the runner. The runner also searches `tests/probes` through the executable/current-directory ancestry, so direct source-tree builds are not dependent on one exact output folder.


## Phase 55 external conformance support

The runner supports both the built-in probe gate and Blargg-style external ROMs that publish the `$6000-$6003` status/signature protocol. `HeadlessRunner` accepts `--timing auto|ntsc|pal|dendy`; this is important for hardware-test corpora whose manifest declares PAL/NTSC separately from the ROM header.

`run_suite.py` can consume `test_roms.xml` from the public `christopherpow/nes-test-roms` corpus. It honors each test's `system` field, supports repeatable path filters through `--suite`, and can emit JSON suitable for tracking accuracy over time. External ROMs are not redistributed in this repository.

Examples:

```bash
python3 tests/run_suite.py ./NESUltimateEmu.Tests
# The script runs built-ins by default; use a corpus manifest for external tests:
python3 tests/run_suite.py ./NESUltimateEmu.Tests --manifest /path/to/nes-test-roms/test_roms.xml --json accuracy.json
python3 tests/run_suite.py ./NESUltimateEmu.Tests --manifest /path/to/nes-test-roms/test_roms.xml --suite ppu_vbl_nmi --suite vbl_nmi_timing
python3 tests/run_suite.py ./NESUltimateEmu.Tests /path/to/one-test.nes --timing pal
```

Exit classification used by the dashboard: 0=pass, 1=test failure, 2=load error, 3=timeout after protocol detection, 4=unsupported/no `$6000` protocol.


`NESUltimateEmu.Tests` is a windowless console target that runs the normal CPU/Bus/PPU/APU/Cartridge/Mapper core without SDL or ImGui host I/O.

## Built-in regression gate

Phase 2 folds the focused hardware probes into `NESUltimateEmu.Tests`. Build the test project, then run:

```bat
x64\Release\NESUltimateEmu.Tests.exe --self-test
```

The command runs the CPU conformance smoke probe, BRK/NMI boundary, DMC load-start, PPU open-bus/OAM, both DMC/PPU conflict alignments, and DMC/APU activation-conflict probes. The three small probe ROM assets are copied beside the test executable automatically by MSBuild. The process exits `0` only when every built-in probe passes, making `--self-test` suitable as a pre-commit or CI regression gate.

The individual probe sources retain standalone `main` wrappers when compiled without `NES_PROBE_SUITE`, so they can still be built and debugged independently.

## Build

Open `NESUltimateEmu.slnx` in Visual Studio and build `NESUltimateEmu.Tests` for x64. The test target defines `NES_HEADLESS`; `APU.cpp` still clocks normally, but its SDL audio-device functions become no-ops.

## Run one ROM

```bat
x64\Release\NESUltimateEmu.Tests.exe C:\tests\instr_test-v5\official_only.nes
```

Optional timeout:

```bat
x64\Release\NESUltimateEmu.Tests.exe C:\tests\test.nes --max-cycles 200000000
```

The runner understands the standard Blargg memory protocol used by many NES conformance ROMs: status at `$6000`, signature `$DE $B0 $61` at `$6001-$6003`, and zero-terminated text starting at `$6004`. It also honors status `$81` by waiting at least 100 ms of emulated NTSC CPU time before issuing console RESET.

Exit codes are machine-readable: `0` pass, `1` test failure, `2` load/setup error, `3` timeout after protocol detection, `4` unsupported/no recognized protocol.

## Run the regression suite

Run only the built-in hardware regressions:

```bat
py tests\run_suite.py x64\Release\NESUltimateEmu.Tests.exe
```

Or run the built-ins first and then recursively execute every `.nes` test ROM in a directory:

```bat
py tests\run_suite.py x64\Release\NESUltimateEmu.Tests.exe C:\tests\nes-test-roms
```

Use `--skip-builtins` when you intentionally want to run only the external ROM set. Tests using a different completion protocol currently report exit code 4; add protocol adapters to `HeadlessRunner.cpp` rather than special-casing the emulation core.

## Test ROMs

Test ROMs are intentionally not bundled with the emulator. Keep third-party test suites in a separate directory and respect their licenses. Good early targets are CPU instruction/timing suites, PPU VBlank/NMI suites, MMC3 IRQ suites, and DMC/DMA suites.

## DMC CPU revision profile

AccuracyCoin accepts two observed implicit DMC-stop behaviors. The runner defaults to the mid-1990-or-later profile. The core now models the late RP2A03G/RP2A03H same-cycle unexpected reload as well as the one-cycle abort window shared by both revisions. Use:

```bat
NESUltimateEmu.Tests.exe test.nes --dmc-revision late
NESUltimateEmu.Tests.exe test.nes --dmc-revision early
```

This lets the same regression harness verify both known hardware families.

## DMC/PPU conflict probe

Phase 27D3 includes `probes/DmcPpuConflictProbe.cpp` plus two generated NROM probe images. The probe checks that DMC-stretched `$2007` reads increment PPU `v` once per repeated CPU read (3 total increments without alignment, 4 with alignment) and that the first DMC halt read of `$2002` clears the PPU write toggle. These test-only PPU accessors are compiled only when `NES_HEADLESS` is defined.

## DMC/APU register-activation conflict probe

Phase 27D4A adds `probes/DmcApuConflictProbe.cpp` and `probes/dmc_apu_conflict.nes`. It verifies that a DMC GET with low address bits `$15`, while the CPU is halted on a `$4000-$401F` read, activates `$4015`, clears the frame IRQ flag, and leaves the external DMC byte on the external bus latch. It also verifies low bits `$16` activate controller port 1 and clock its serial register.

## Phase 27D4A Hotfix 4: DMC load-start probe

`DmcLoadStartProbe.cpp` verifies that a `$4015`-initiated DMC load begins 3 or 4 CPU clocks after the write depending on CPU/APU phase. The load remains GET-scheduled so its transfer sequence is halt/dummy/get rather than a reload-style aligned transfer.


## BRK/NMI boundary regression

`tests/probes/InterruptHijackProbe.cpp` verifies the fetched-BRK late-hijack boundary: an NMI arriving as the BRK low-vector cycle begins must redirect the vector to `$FFFA/$FFFB` while the already-pushed status remains BRK-originated. Hardware IRQ uses the stricter vector-commit boundary validated by `cpu_interrupts_v2/3-nmi_and_irq`.

## Phase 3 CPU conformance smoke probe

`tests/probes/CpuConformanceProbe.cpp` protects the first CPU-conformance conversion. It verifies that one-byte implied/accumulator instructions perform their discarded PC read on cycle 2 and do not update registers or flags during the opcode-fetch cycle. It also checks the NMOS `JMP ($xxFF)` page-wrap behavior and the three-cycle taken-branch bus sequence.

The implied/accumulator family is now scheduled explicitly in `CPU.cpp`, so DMC RDY stretching can hold the real second cycle without architectural state changing early.

## Phase 4 PPU conformance probe

`PpuConformanceProbe.cpp` locks down two timing-sensitive 2C02 behaviors:

- `$2007` reads/writes increment `v` linearly by 1 or 32 outside rendering, but on visible/pre-render scanlines with rendering enabled they increment coarse X and Y together using the rendering scroll wrap rules.
- NTSC odd-frame shortening skips the final pre-render idle dot: an odd pre-render contains 340 clocks and advances from dot 339 directly to visible scanline 0 dot 0; an even pre-render contains all 341 clocks.

These checks are part of `--self-test` and therefore run before any optional external ROM suite.


## Phase 5 APU/frame-counter conformance probe

`ApuConformanceProbe.cpp` verifies the timing-sensitive `$4017` control path. The
sequencer mode/reset is deferred by CPU/APU alignment (3 or 4 CPU clocks), while
IRQ inhibit remains immediate. Entering 5-step mode generates its initial
quarter+half clock only when that delayed reset occurs. The probe also locks the
4-step frame IRQ edge to frame count 29828 and verifies that a `$4015` status read
reports and clears the frame IRQ flag.

The existing DMC load-start, DMC/APU conflict, and DMC/PPU conflict probes remain
in the same `--self-test` gate, so frame-counter changes cannot silently regress
DMA timing.

### Mapper conformance

`MapperConformanceProbe.cpp` provides focused regression coverage for mapper hardware rules that are easy to regress without a full game ROM:

- MMC1 consecutive-cycle write suppression, including the exception that D7 reset is never ignored.
- MMC3 filtered A12 IRQ reload/decrement behavior and rejection of short A12-low pulses.
- MMC5 scanline IRQ compare value 0 and frame-end IRQ acknowledgement.
- Namco 163 sound-RAM address auto-increment saturation at `$7F`.
- Sunsoft FME-7 IRQ acknowledgement on every command `$D` write.
- MMC6 internal 1 KiB RAM mirroring, global enable, and independent 512-byte read/write protection halves.
- UNROM-512 NES 2.0 submapper conflict selection, submapper 3 H/V mirroring, submapper 4 LED-register address decoding, and truthful rejection of flash-enabled images until flash emulation is present.
- Mapper 15 PRG banking modes and mapper 31 eight-slot 4 KiB PRG banking.
- Mapper 116 Huang-1/Huang-2 composite mode switching (MMC1/MMC3/VRC2b), state preservation, CHR A18, VRC2 power-up CHR state, and submapper-specific MMC1 behavior.
- Mapper 215 UNL-8237/8237A outer PRG/CHR banking, NROM override, selectable MMC3 register/index scrambling, reset-sensitive outer bank state, and NES 2.0 submapper gating.


### Cartridge conformance

The built-in suite validates archaic iNES header cleanup, trainer preload, NES 2.0 save-state identity metadata, PRG/CHR NVRAM battery round-trips, and transactional cartridge state loading.

## Timing / region conformance

`TimingConformanceProbe.cpp` validates the regional timing layer introduced in Phase 8:

- NES 2.0 byte 12 parsing for NTSC, PAL, multi-region, and Dendy images.
- 262-scanline NTSC and 312-scanline PAL/Dendy PPU frame lengths.
- Dendy VBlank/NMI start at scanline 291.
- Licensed PAL 16:5 PPU/CPU divider (3.2 PPU dots per CPU clock).
- Regional CPU clocks and PAL-vs-NTSC/Dendy APU noise/DMC period tables.
- PAL frame-counter cadence versus the NTSC-cycle-count cadence used by Dendy.

## Phase 9 mapper/submapper coverage

The mapper conformance probe now also checks NES 2.0 board distinctions for UxROM bus conflicts, Irem G-101/Major League, Camerica/Fire Hawk, mapper 78 Cosmo Carrier vs. Holy Diver, and VRC7a/VRC7b address decoding. It also verifies Color Dreams mapper 11 PRG/CHR register bit assignments. Unsupported NES 2.0 variants are no longer reported as implemented for mapper 30 and deprecated mapper 78 submapper 2.
## Phase 14 reset lifecycle coverage

The cartridge and mapper probes now verify that the console Reset button is propagated through `Bus -> Cartridge -> Mapper`, while cold `powerOn()` is delivered as a distinct hard-reset event. Mapper 116 submapper 3 uses this hook to cycle the Mario 5-in-1 outer ROM selector and save/restore the selected game.
### Phase 17 J.Y. Company ASIC coverage

The mapper conformance probe now also checks mappers 35/90/209/211: outer PRG/CHR banking, reversed PRG-bit mode, delayed multiplier completion, CPU-source IRQ and acknowledge, mapper-90 nametable suppression, mapper-209 extended/ROM nametables with CIRAM write-through, MMC4-like CHR latches, and mapper-35 WRAM mapping.


## Mapper 176 / FK23C regression coverage

The mapper conformance probe now validates the foundational 8025/FK23C variants:

- NES 2.0 submapper 0: six-bit MMC3 PRG banking and outer-bank modes.
- NES 2.0 submapper 1: eight-bit MMC3 PRG banking, `$E003` register decode,
  extended MMC3 bank registers, and CNROM/NROM CHR modes.
- Submappers 2-5 remain explicitly unsupported until their extra WRAM,
  high-address and board-specific wiring is implemented.

Phase 19 extends mapper 176 coverage through NES 2.0 submappers 0-5, including FS005 banked WRAM/protection, JX9003B high address registers, Smart Genius PRG A21 wiring, and HST-162 $4800 high PRG banking.

### Phase 23 mapper coverage

The mapper conformance probe now covers NES 2.0 mapper 12 submapper 0 (Gouder SL-5020B): its external `$4132` GAL independently supplies CHR A18 for PPU A12=0/1 and the Huang-1 core uses MMC3A-style zero-latch IRQ behavior. Mapper 12 submapper 1 and mappers 6/8/17 remain intentionally unsupported because those formats represent Front Fareast Magic/Super Magic Card RAM cartridges and require writable PRG/CHR memory plus trainer boot semantics rather than only static ROM banking.

### Phase 24 RAM-cartridge coverage

The mapper regression now covers the shared Front Fareast Magic/Super Magic Card implementation: mapper 6 latch banking, mapper 8's mapper-6 mode-4 alias, mapper 12.1 4M startup, mapper 17 1 KiB CHR banking and signed 16-bit overflow IRQ, plus support gating. Cartridge conformance additionally verifies writable RAM-image allocation, no battery persistence, mapper-6 implicit JSR $7003 trainer boot metadata, and mapper-17 submapper-selected trainer entry addresses.


## Phase 25 - legacy mapper 40-43 compatibility

Added mapper 40 (NTDEC 2722 base board), mapper 41 (Caltron 6-in-1), mapper 42 FDS conversions, and mapper 43 TONY-I/YS-612. Regression coverage verifies banking, mirroring, reset behavior, and M2 IRQ timing. Cartridge ROM mapping can now extend into $4020-$5FFF after mapper-register/RAM priority, which is required by mapper 43's $5000 expansion ROM. Mapper 40 submapper 1 remains deliberately unsupported pending fuller NTDEC 2752 outer-board validation.


## Phase 26 - Action 53 and conventional mapper expansion

Added mapper 28 (Action 53), mapper 36 (TXC 01-22000-400), mapper 44 (Super Big 7-in-1 MMC3 multicart), and mapper 46 (Rumblestation 15-in-1). Mapper 28 implements the user/supervisor register model, 32/64/128/256 KiB outer PRG windows, 32 KiB and fixed-half PRG modes, CHR-RAM banking, and H/V/one-screen mirroring. Mapper 36 implements the TXC PP/RR register machine, inversion/increment modes, readable RR bits, explicit PRG latch, and CHR banking. Mapper 44 reuses the MMC3 core with its seven outer PRG/CHR blocks (block 7 aliases block 6), and mapper 46 implements its two-stage outer/inner PRG+CHR registers. The built-in regression suite remains 12/12 passing.

### Phase 28 mapper coverage

The mapper conformance probe now also checks mappers 51, 52, 61, and 63, including mapper 52 outer-bank locking and mapper 63 open-bus behavior.


### Phase 29 mapper coverage

The mapper conformance probe now checks mappers 54, 55, 56, 57, and 59. Coverage includes address-driven Novel Diamond PRG/CHR selection, mapper 55's expansion-ROM/RAM mirrors, KS-202 PRG/CHR registers and cycle IRQ, mapper 57's two-register PRG/CHR/mirroring modes, and mapper 59's address-selected PRG/CHR/mirroring plus DIP-read mode. Mapper 53 remains support-gated pending a ROM database/hash-based PRG-order override.

### Phase 30-31 ROM database coverage

The cartridge probe also validates the hash-driven metadata layer. Mapper 53's EPROM-first physical ordering is selected by its verified first-32-KiB CRC signature. Phase 31 adds verified full PRG+CHR payload signatures for Holy Diver (mapper 78 submapper 3), Uchuusen: Cosmo Carrier (mapper 78 submapper 1), and Family Circuit '91 (Namco 175 / mapper 210 submapper 1 with 2 KiB battery RAM). Database corrections are applied before mapper construction and RAM allocation; filenames are never consulted.

### Phase 34 mapper coverage

The mapper conformance probe now includes mapper 105 / NES-EVENT: cold-boot fixed mapping, event-bank unlock, official NWC 30-bit M2 timer edge, and IRQ acknowledge/reset. Mapper 111 / GTROM remains support-gated pending complete self-flash persistence.


### Phase 35 GTROM regression
`MapperConformanceProbe` validates mapper 111 PRG banking, shared pattern/nametable PPU RAM banking, SST39SF040 byte programming and sector erase, and mapper-owned persistent flash round-tripping.

- Phase 38 mapper coverage: mapper 123 / Kǎshèng H2288 scrambled MMC3 index routing and `$5800` NROM override.
- Phase 39 mapper coverage: mapper 121 / Kǎshèng A9711/A9713 protection array, bit-reversed protection latch, direct PRG overrides, outer banking, CHR A18 wiring, and state restoration.

## Phase 74 BRK/NMI hijack boundary

AccuracyCoin's NMI-over-BRK test exposed that the previous internal regression
kept software BRK hijackable one CPU tick too long. The vector is selected at
the status-push boundary: an NMI recognized before that boundary may redirect
BRK to $FFFA/$FFFB while preserving BRK's B=1 stack image, but an NMI arriving
when the $FFFE low-vector read is already due cannot redirect the in-flight
BRK and remains pending for a later interrupt opportunity.

`InterruptHijackProbe` now verifies both sides of that boundary as well as the
existing hardware-IRQ commit case.

## Phase 79 internal data-bus regression

`DmcApuConflictProbe.cpp` now primes the 2A03 internal data latch with D5 clear,
performs a DMC GET whose external byte is `$A5`, and verifies that accidental
`$4015` activation cannot copy the DMA byte's D5 into the internal latch while
the external latch still retains `$A5`.

## Phase 81 - DMC stop-DMA timing

Phase 81 replaces the Phase 80 delayed-$4015 cancellation heuristic with an
output-unit-driven model of the documented DMC stop-DMA bug:

- Clearing DMC enable immediately sets the reader's remaining-byte count to 0.
- A DMC DMA that has already been scheduled is allowed to finish; it is not
  retroactively cancelled by the $4015 write.
- If the output unit empties the sample buffer in the short stop window, the
  otherwise-suppressed reload survives as a one-cycle HALT attempt.
- That one-cycle aborted reload is one-shot: a CPU write on its halt slot
  suppresses it instead of delaying it.
- The pre-mid-1990 one-byte implicit-stop case uses the same one-cycle reload
  mechanism, armed by completion of the one-byte load and committed only if
  the output unit empties the buffer immediately afterward.

The existing DMC/OAM arbitration and internal/external CPU data-bus behavior
are unchanged. The Linux self-test build passes 13/13 regression probes.
## CPU bus-cycle completeness gate

Phase 2 adds an exhaustive CPU audit to the normal `--self-test` run. The CPU conformance probe executes all 256 opcode values from deterministic internal RAM and requires every non-JAM opcode bus slot reported by `CPU::nextBusCycle()` to be exact. The 12 JAM/KIL opcodes are separately checked as the intentional halted-state exception. See `docs/PHASE2_CPU_BUS_CYCLE_AUDIT.md` for the conversion matrix and audit result.


### Phase 8 - Mapper 40 submapper 1 / NTDEC 2752

Mapper 40 NES 2.0 submapper 1 is now enabled and regression-tested. The
address-latched `$C000-$DFFF` outer register follows the documented
`...ppNPCCM` wiring: P selects the original SMB2J map versus regular NROM,
N selects NROM-128/NROM-256 behavior, `pp` supplies the applicable outer PRG
address bits, `CC` selects the 8 KiB CHR bank, and M controls mirroring.
Coverage checks SMB2J compatibility, NROM-128 mirroring, NROM-256 banking,
CHR/mirroring selection, hard-reset defaults, save-state restoration, and the
submapper support gate.

## Legacy blargg APU frame-counter suite (2005-07-30)

The older `blargg_apu_2005.07.30` ROMs predate the later `$6000` memory-status protocol. Run them explicitly with `--legacy-apu`. The adapter waits until the ROM reaches its terminal one-instruction report loop, then reads the suite's zero-page `result` byte at `$00F0`; result `1` is success. This observation is side-effect-free and compiled only into the headless target.

```bash
NESUltimateEmu.Tests 01.len_ctr.nes --legacy-apu
```

To register the complete ordered 11-ROM suite with CTest:

```bash
cmake -S . -B build \
  -DNES_BUILD_HEADLESS_TESTS=ON \
  -DNES_BLARGG_LEGACY_APU_ROM_DIR=/path/to/blargg_apu_2005.07.30
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The ROMs are intentionally not redistributed with the emulator.

## Tier 0 legacy result protocol

Some older blargg ROMs predate the `$6000` protocol. Run them through the
legacy adapter and supply the suite's result byte address when necessary:

```bat
NESUltimateEmu.Tests.exe test.nes --legacy-apu --legacy-result-address 0xF0
NESUltimateEmu.Tests.exe test.nes --legacy-apu --legacy-result-address 0xF8
```

The adapter considers result code 1 a hardware pass by default. The
`--expected-result N` option exists only to pin a documented known discrepancy
in CI. It should not be used to relabel a failing hardware test as passing.
Terminal self-loop confirmation is longer than one NTSC frame so tight loops
that wait for NMI are not mistaken for the final report loop.

## Extended hardware-test corpus

The `branch_timing_tests.zip` corpus used during Tier-0 validation contains far more than the three historical branch ROMs. It also includes CPU interrupt/reset/instruction suites and detailed PPU VBlank/NMI tests. Configure its extracted root with:

```bash
cmake -S . -B build \
  -DNES_BUILD_HEADLESS_TESTS=ON \
  -DNES_WARNINGS_AS_ERRORS=ON \
  -DNES_EXTENDED_TEST_ROOT=/path/to/extracted/corpus
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The extended CTest subset deliberately separates hardware passes from pinned known discrepancies. A `*-known-failure` entry only succeeds when the ROM still returns its documented current failure code; this prevents a regression from silently changing behavior while also preventing the dashboard from misreporting that ROM as a hardware pass.

Current Tier-0 observations from this corpus:

- CPU dummy writes to OAM and PPU memory: pass.
- CPU interrupts: CLI latency, NMI+IRQ, IRQ+DMA, and branch-delayed IRQ pass; NMI+BRK collision now passes after modeling the CPU /NMI input-sampling phase.
- CPU reset RAM/register behavior: pass.
- Official instruction functional tests, miscellaneous instruction tests, and instruction timing: pass.
- PPU open bus and PPU read buffer: pass.
- `ppu_vbl_nmi`: tests 01-04, 06, 08, and 09 pass; tests 05 and 07 now pass with phase-aware CPU /NMI sampling; test 10 still reports that the odd-frame clock is skipped too soon relative to enabling background rendering.

Third-party ROMs remain external and are never redistributed with the emulator source archive.

## Tier-0 CPU/PPU NMI sampling phase

The RP2A03 now keeps three distinct NMI states: an asynchronous PPU edge, an
edge sampled at the CPU input phase, and an interrupt committed by the
instruction poll. `Bus::clock()` samples the asynchronous edge after the first
PPU dot of each CPU period. This preserves the hardware distinction between an
NMI edge that arrives early enough in the period to affect the current CPU
poll and one arriving on the later PPU subphase that must wait for the next
period.

This fixes both `cpu_interrupts_v2/2-nmi_and_brk.nes` and
`ppu_vbl_nmi/05-nmi_timing.nes` without regressing
`cpu_interrupts_v2/3-nmi_and_irq.nes` or the built-in interrupt-hijack probe.
The BRK vector remains hijackable through the T4/status-push boundary; the
fix is the missing CPU input-sampling stage, not an opcode-specific BRK rule.

## Scanline visual protocol

Quietust's `scanline.nes` has no `$6000` result protocol. Use:

```sh
NESUltimateEmu.Tests scanline.nes --scanline-visual
```

The runner executes 5,000,000 CPU cycles and checks only the ROM's documented
error-star cells (tile columns 25-30 in the three test regions). Any lit pixel
there is a failure; the normal separator column is outside the checked area.
When `NES_EXTENDED_TEST_ROOT` contains `scanline/scanline.nes`, CMake registers
this automatically as `extended-ppu-scanline-visual`.
