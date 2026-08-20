# Phase 25 Headless Regression Harness

`NESUltimateEmu.Tests` is a windowless console target that runs the normal CPU/Bus/PPU/APU/Cartridge/Mapper core without SDL or ImGui host I/O.

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

## Run a directory

```bat
py tests\run_suite.py x64\Release\NESUltimateEmu.Tests.exe C:\tests\nes-test-roms
```

The suite script recursively finds `.nes` files and prints a final pass/fail summary. Tests using a different completion protocol will currently report exit code 4; add protocol adapters to `HeadlessRunner.cpp` rather than special-casing the emulation core.

## Test ROMs

Test ROMs are intentionally not bundled with the emulator. Keep third-party test suites in a separate directory and respect their licenses. Good early targets are CPU instruction/timing suites, PPU VBlank/NMI suites, MMC3 IRQ suites, and DMC/DMA suites.

## DMC CPU revision profile

AccuracyCoin accepts two observed implicit DMC-stop behaviors. The runner defaults to the mid-1990-or-later profile. Use:

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
