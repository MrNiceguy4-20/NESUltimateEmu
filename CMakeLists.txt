cmake_minimum_required(VERSION 3.20)

project(NESUltimateEmu LANGUAGES CXX)

option(NES_BUILD_HEADLESS_TESTS "Build the headless emulator regression runner" ON)
option(NES_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(NES_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(NES_ACCURACYCOIN_ROM "" CACHE FILEPATH "Path to AccuracyCoin.nes for optional external conformance testing")
set(NES_ACCURACYCOIN_MIN_PASS "141" CACHE STRING "Minimum AccuracyCoin pass count accepted by CTest")
set(NES_BLARGG_APU_ROM_DIR "" CACHE PATH "Directory containing optional blargg APU/reset ROMs")
set(NES_BLARGG_LEGACY_APU_ROM_DIR "" CACHE PATH "Directory containing blargg_apu_2005.07.30 ROMs")
set(NES_BLARGG_PPU_ROM_DIR "" CACHE PATH "Directory containing blargg_ppu_tests_2005.09.15b ROMs")
set(NES_BRANCH_TIMING_ROM_DIR "" CACHE PATH "Directory containing blargg branch_timing_tests ROMs")
set(NES_SPRITE_TEST_ROOT "" CACHE PATH "Root directory extracted from sprite_hit_tests_2005.10.05.zip")
set(NES_EXTENDED_TEST_ROOT "" CACHE PATH "Root directory of the extended NES hardware-test corpus")

if(NES_BUILD_HEADLESS_TESTS)
    add_executable(NESUltimateEmu.Tests
        tests/HeadlessRunner.cpp
        tests/probes/ProbeSuite.cpp
        tests/probes/InterruptHijackProbe.cpp
        tests/probes/DmcLoadStartProbe.cpp
        tests/probes/DmaArbitrationProbe.cpp
        tests/probes/PpuOpenBusOamProbe.cpp
        tests/probes/PpuConformanceProbe.cpp
        tests/probes/DmcPpuConflictProbe.cpp
        tests/probes/DmcApuConflictProbe.cpp
        tests/probes/CpuConformanceProbe.cpp
        tests/probes/TimingConformanceProbe.cpp
        tests/probes/ApuConformanceProbe.cpp
        tests/probes/MapperConformanceProbe.cpp
        tests/probes/CartridgeConformanceProbe.cpp
        src/core/CPU.cpp
        src/core/Bus.cpp
        src/core/PPU.cpp
        src/core/APU.cpp
        src/core/Cartridge.cpp
        src/core/Mapper.cpp
    )

    target_compile_features(NESUltimateEmu.Tests PRIVATE cxx_std_17)
    target_compile_definitions(NESUltimateEmu.Tests PRIVATE NES_HEADLESS NES_PROBE_SUITE)
    target_include_directories(NESUltimateEmu.Tests PRIVATE src/core)

    if(MSVC)
        target_compile_options(NESUltimateEmu.Tests PRIVATE /W4 /permissive-)
        if(NES_WARNINGS_AS_ERRORS)
            target_compile_options(NESUltimateEmu.Tests PRIVATE /WX)
        endif()
    else()
        target_compile_options(NESUltimateEmu.Tests PRIVATE -Wall -Wextra -Wpedantic)
        if(NES_WARNINGS_AS_ERRORS)
            target_compile_options(NESUltimateEmu.Tests PRIVATE -Werror)
        endif()
        if(NES_ENABLE_SANITIZERS)
            target_compile_options(NESUltimateEmu.Tests PRIVATE
                -fno-omit-frame-pointer -fsanitize=address,undefined)
            target_link_options(NESUltimateEmu.Tests PRIVATE
                -fno-omit-frame-pointer -fsanitize=address,undefined)
        endif()
    endif()

    add_custom_target(NESUltimateEmu.ProbeAssets ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/probes/dmc_apu_conflict.nes"
            "$<TARGET_FILE_DIR:NESUltimateEmu.Tests>/dmc_apu_conflict.nes"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/probes/dmc_ppu_conflict_probe.nes"
            "$<TARGET_FILE_DIR:NESUltimateEmu.Tests>/dmc_ppu_conflict_probe.nes"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/probes/dmc_ppu_conflict_probe_align.nes"
            "$<TARGET_FILE_DIR:NESUltimateEmu.Tests>/dmc_ppu_conflict_probe_align.nes"
        DEPENDS NESUltimateEmu.Tests
        COMMENT "Copying built-in probe ROMs"
        VERBATIM
    )

    include(CTest)
    if(BUILD_TESTING)
        add_test(NAME builtin-regression-probes
                 COMMAND NESUltimateEmu.Tests --self-test)
        if(NES_ACCURACYCOIN_ROM)
            add_test(NAME accuracycoin
                     COMMAND NESUltimateEmu.Tests "${NES_ACCURACYCOIN_ROM}" --accuracycoin --max-cycles 500000000 --min-pass "${NES_ACCURACYCOIN_MIN_PASS}")
            set_tests_properties(accuracycoin PROPERTIES TIMEOUT 180)
        endif()

        if(NES_BLARGG_LEGACY_APU_ROM_DIR)
            set(_nes_blargg_legacy_apu_roms
                01.len_ctr.nes
                02.len_table.nes
                03.irq_flag.nes
                04.clock_jitter.nes
                05.len_timing_mode0.nes
                06.len_timing_mode1.nes
                07.irq_flag_timing.nes
                08.irq_timing.nes
                09.reset_timing.nes
                10.len_halt_timing.nes
                11.len_reload_timing.nes
            )
            foreach(_rom IN LISTS _nes_blargg_legacy_apu_roms)
                if(EXISTS "${NES_BLARGG_LEGACY_APU_ROM_DIR}/${_rom}")
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-legacy-apu-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${NES_BLARGG_LEGACY_APU_ROM_DIR}/${_rom}" --legacy-apu --max-cycles 500000000 --quiet)
                    set_tests_properties("blargg-legacy-apu-${_test_name}" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional legacy blargg APU ROM: ${NES_BLARGG_LEGACY_APU_ROM_DIR}/${_rom}")
                endif()
            endforeach()
        endif()

        if(NES_BLARGG_APU_ROM_DIR)
            set(_nes_blargg_apu_roms
                4015_cleared.nes
                4017_timing.nes
                4017_written.nes
                irq_flag_cleared.nes
                len_ctrs_enabled.nes
                works_immediately.nes
                apu_test.nes
            )
            foreach(_rom IN LISTS _nes_blargg_apu_roms)
                if(EXISTS "${NES_BLARGG_APU_ROM_DIR}/${_rom}")
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-apu-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${NES_BLARGG_APU_ROM_DIR}/${_rom}" --max-cycles 500000000 --quiet)
                    set_tests_properties("blargg-apu-${_test_name}" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional blargg APU ROM: ${NES_BLARGG_APU_ROM_DIR}/${_rom}")
                endif()
            endforeach()
        endif()

        if(NES_BLARGG_PPU_ROM_DIR)
            set(_nes_blargg_ppu_roms
                palette_ram.nes
                sprite_ram.nes
                vbl_clear_time.nes
                vram_access.nes
            )
            foreach(_rom IN LISTS _nes_blargg_ppu_roms)
                if(EXISTS "${NES_BLARGG_PPU_ROM_DIR}/${_rom}")
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-ppu-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${NES_BLARGG_PPU_ROM_DIR}/${_rom}" --legacy-apu --legacy-result-address 0xF0 --max-cycles 250000000 --quiet)
                    set_tests_properties("blargg-ppu-${_test_name}" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional blargg PPU ROM: ${NES_BLARGG_PPU_ROM_DIR}/${_rom}")
                endif()
            endforeach()
            if(EXISTS "${NES_BLARGG_PPU_ROM_DIR}/power_up_palette.nes")
                message(STATUS "power_up_palette.nes detected but not gated: its expected power-up bytes are console-specific")
            endif()
        endif()

        if(NES_BRANCH_TIMING_ROM_DIR)
            set(_nes_branch_roms
                1.Branch_Basics.nes
                2.Backward_Branch.nes
                3.Forward_Branch.nes
            )
            foreach(_rom IN LISTS _nes_branch_roms)
                if(EXISTS "${NES_BRANCH_TIMING_ROM_DIR}/${_rom}")
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-branch-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${NES_BRANCH_TIMING_ROM_DIR}/${_rom}" --legacy-apu --legacy-result-address 0xF8 --max-cycles 250000000 --quiet)
                    set_tests_properties("blargg-branch-${_test_name}" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional branch timing ROM: ${NES_BRANCH_TIMING_ROM_DIR}/${_rom}")
                endif()
            endforeach()
        endif()

        if(NES_SPRITE_TEST_ROOT)
            set(_nes_sprite_hit_dir "${NES_SPRITE_TEST_ROOT}/sprite_hit_tests_2005.10.05")
            foreach(_i RANGE 1 11)
                if(_i LESS 10)
                    set(_prefix "0${_i}")
                else()
                    set(_prefix "${_i}")
                endif()
                file(GLOB _matches "${_nes_sprite_hit_dir}/${_prefix}.*.nes")
                foreach(_rom IN LISTS _matches)
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-sprite-hit-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${_rom}" --legacy-apu --legacy-result-address 0xF8 --max-cycles 250000000 --quiet)
                    set_tests_properties("blargg-sprite-hit-${_test_name}" PROPERTIES TIMEOUT 120)
                endforeach()
            endforeach()

            set(_nes_sprite_overflow_dir "${NES_SPRITE_TEST_ROOT}/sprite_overflow_tests")
            foreach(_i RANGE 1 5)
                file(GLOB _matches "${_nes_sprite_overflow_dir}/${_i}.*.nes")
                foreach(_rom IN LISTS _matches)
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-sprite-overflow-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${_rom}" --legacy-apu --legacy-result-address 0xF8 --max-cycles 250000000 --quiet)
                    set_tests_properties("blargg-sprite-overflow-${_test_name}" PROPERTIES TIMEOUT 120)
                endforeach()
            endforeach()

            set(_nes_vbl_nmi_dir "${NES_SPRITE_TEST_ROOT}/vbl_nmi_timing")
            foreach(_i RANGE 1 7)
                file(GLOB _matches "${_nes_vbl_nmi_dir}/${_i}.*.nes")
                foreach(_rom IN LISTS _matches)
                    get_filename_component(_test_name "${_rom}" NAME_WE)
                    add_test(NAME "blargg-vbl-nmi-${_test_name}"
                             COMMAND NESUltimateEmu.Tests "${_rom}" --legacy-apu --legacy-result-address 0xF8 --max-cycles 250000000 --quiet)
                    set_tests_properties("blargg-vbl-nmi-${_test_name}" PROPERTIES TIMEOUT 120)
                endforeach()
            endforeach()
        endif()

        if(NES_EXTENDED_TEST_ROOT)
            function(nes_add_blargg_test _name _relative)
                set(_rom "${NES_EXTENDED_TEST_ROOT}/${_relative}")
                if(EXISTS "${_rom}")
                    add_test(NAME "extended-${_name}"
                             COMMAND NESUltimateEmu.Tests "${_rom}" --max-cycles 150000000 --quiet)
                    set_tests_properties("extended-${_name}" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional extended ROM: ${_rom}")
                endif()
            endfunction()

            function(nes_add_legacy_shell_test _name _relative)
                set(_rom "${NES_EXTENDED_TEST_ROOT}/${_relative}")
                if(EXISTS "${_rom}")
                    add_test(NAME "extended-${_name}"
                             COMMAND NESUltimateEmu.Tests "${_rom}" --legacy-shell --max-cycles 150000000 --quiet)
                    set_tests_properties("extended-${_name}" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional extended ROM: ${_rom}")
                endif()
            endfunction()

            function(nes_add_known_failure _name _relative _result)
                set(_rom "${NES_EXTENDED_TEST_ROOT}/${_relative}")
                if(EXISTS "${_rom}")
                    add_test(NAME "extended-${_name}-known-failure"
                             COMMAND NESUltimateEmu.Tests "${_rom}" --expected-result "${_result}" --max-cycles 150000000 --quiet)
                    set_tests_properties("extended-${_name}-known-failure" PROPERTIES TIMEOUT 120)
                else()
                    message(WARNING "Missing optional extended ROM: ${_rom}")
                endif()
            endfunction()

            # CPU bus/interrupt/reset/instruction coverage.
            nes_add_blargg_test(cpu-dummy-writes-oam "cpu_dummy_writes/cpu_dummy_writes_oam.nes")
            nes_add_blargg_test(cpu-dummy-writes-ppumem "cpu_dummy_writes/cpu_dummy_writes_ppumem.nes")
            nes_add_blargg_test(cpu-int-cli-latency "cpu_interrupts_v2/rom_singles/1-cli_latency.nes")
            nes_add_blargg_test(cpu-int-nmi-brk "cpu_interrupts_v2/rom_singles/2-nmi_and_brk.nes")
            nes_add_blargg_test(cpu-int-nmi-irq "cpu_interrupts_v2/rom_singles/3-nmi_and_irq.nes")
            nes_add_blargg_test(cpu-int-irq-dma "cpu_interrupts_v2/rom_singles/4-irq_and_dma.nes")
            nes_add_blargg_test(cpu-int-branch-delay "cpu_interrupts_v2/rom_singles/5-branch_delays_irq.nes")
            nes_add_blargg_test(cpu-reset-ram "cpu_reset/ram_after_reset.nes")
            nes_add_blargg_test(cpu-reset-registers "cpu_reset/registers.nes")
            nes_add_blargg_test(instr-misc "instr_misc/instr_misc.nes")
            nes_add_blargg_test(instr-official "instr_test-v5/official_only.nes")
            nes_add_blargg_test(instr-timing "instr_timing/instr_timing.nes")

            # DMC DMA read-conflict and sprite-DMA arbitration coverage.
            nes_add_legacy_shell_test(dmc-2007-read "dmc_dma_during_read4/dma_2007_read.nes")
            nes_add_legacy_shell_test(dmc-2007-write "dmc_dma_during_read4/dma_2007_write.nes")
            nes_add_legacy_shell_test(dmc-4016-read "dmc_dma_during_read4/dma_4016_read.nes")
            nes_add_legacy_shell_test(dmc-double-2007-read "dmc_dma_during_read4/double_2007_read.nes")
            nes_add_legacy_shell_test(dmc-read-write-2007 "dmc_dma_during_read4/read_write_2007.nes")
            nes_add_blargg_test(sprdma-dmc "sprdma_and_dmc_dma/sprdma_and_dmc_dma.nes")
            nes_add_blargg_test(sprdma-dmc-512 "sprdma_and_dmc_dma/sprdma_and_dmc_dma_512.nes")

            # PPU open-bus/read-buffer and detailed VBlank/NMI timing.
            nes_add_blargg_test(ppu-open-bus "ppu_open_bus/ppu_open_bus.nes")
            nes_add_blargg_test(ppu-read-buffer "ppu_read_buffer/test_ppu_read_buffer.nes")
            nes_add_blargg_test(ppu-vbl-basics "ppu_vbl_nmi/rom_singles/01-vbl_basics.nes")
            nes_add_blargg_test(ppu-vbl-set-time "ppu_vbl_nmi/rom_singles/02-vbl_set_time.nes")
            nes_add_blargg_test(ppu-vbl-clear-time "ppu_vbl_nmi/rom_singles/03-vbl_clear_time.nes")
            nes_add_blargg_test(ppu-nmi-control "ppu_vbl_nmi/rom_singles/04-nmi_control.nes")
            nes_add_blargg_test(ppu-nmi-timing "ppu_vbl_nmi/rom_singles/05-nmi_timing.nes")
            nes_add_blargg_test(ppu-nmi-suppression "ppu_vbl_nmi/rom_singles/06-suppression.nes")
            nes_add_blargg_test(ppu-nmi-on-timing "ppu_vbl_nmi/rom_singles/07-nmi_on_timing.nes")
            nes_add_blargg_test(ppu-nmi-off-timing "ppu_vbl_nmi/rom_singles/08-nmi_off_timing.nes")
            nes_add_blargg_test(ppu-even-odd-frames "ppu_vbl_nmi/rom_singles/09-even_odd_frames.nes")
            nes_add_blargg_test(ppu-even-odd-timing "ppu_vbl_nmi/rom_singles/10-even_odd_timing.nes")

            set(_scanline_rom "${NES_EXTENDED_TEST_ROOT}/scanline/scanline.nes")
            if(EXISTS "${_scanline_rom}")
                add_test(NAME extended-ppu-scanline-visual
                         COMMAND NESUltimateEmu.Tests "${_scanline_rom}" --scanline-visual --max-cycles 5000000 --quiet)
                set_tests_properties(extended-ppu-scanline-visual PROPERTIES TIMEOUT 30)
            endif()
        endif()
    endif()
endif()
