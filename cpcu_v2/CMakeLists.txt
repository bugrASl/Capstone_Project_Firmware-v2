##
##  CMakeLists.txt — CPCU v2.3 Build System
##  Author: bugrASl
##  Date:   April 2026
##
##  Changes v2.2 -> v2.3:
##      - cpcu_tui split into three source files (cpcu_tui.c +
##        cpcu_tui_render.c + cpcu_tui_data.c sharing cpcu_tui.h).
##        The monolithic 2900-line file was getting unmanageable;
##        the split groups main / render / data layers so each
##        file is ~500-1800 lines.
##      - cpcu_safety v2.3: ring-overflow fault is now recoverable
##        (delta-based threshold + 5 s quiescence timer). Public
##        API unchanged so cpcu_io and safety_testbench did not
##        need to change.
##
##  Changes v2.1 -> v2.2:
##      - pca_testbench now depends on cpcu_smooth.c and links -lrt
##        (smoother uses clock_gettime for dt)
##      - cpcu_io and cpcu_kernel both keep cpcu_log linkage; logging
##        is now actually wired up through LOG_I/W/E/D/F macros with
##        optional per-module CSV sinks (--log flag).
##
##  Targets:
##      cpcu_io         Core 3: Real-time I/O controller
##      cpcu_kernel     Core 0: Process supervisor
##      cpcu_tui        Core 0/SSH: ncurses live dashboard (3-file)
##      test_codec      Test: Codec + IPC unit tests (no hardware)
##      pca_testbench   Test: Interactive PCA9685 servo test (with slew)
##      signal_testbench Test: End-to-end signal integrity TUI
##
##  The DSP pipeline is Python (cpcu_dsp.py) — not compiled, just installed.
##

cmake_minimum_required(VERSION 3.16)
project(cpcu VERSION 2.3 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic -O2 -D_GNU_SOURCE)

include_directories(${CMAKE_SOURCE_DIR}/include)

##============= SHARED STATIC LIBRARIES ====================================================

## Codec: wireless packet pack/unpack (platform-independent, shared with BSAU)
add_library(cpcu_codec STATIC src/wireless_packet.c)

## IPC: POSIX shared memory ring buffer + SeqLock
add_library(cpcu_ipc STATIC src/cpcu_ipc.c)
target_link_libraries(cpcu_ipc PUBLIC rt)

## Log: structured colored logging + optional per-module CSV file sinks
add_library(cpcu_log STATIC src/cpcu_log.c)

## Config (v2.3.3): runtime.json loader -> IPC_RuntimeConfig
add_library(cpcu_config STATIC src/cpcu_config.c)
target_link_libraries(cpcu_config PUBLIC cpcu_ipc)

##============= EXECUTABLES ================================================================

## Core 3: Real-time I/O (NRF + PCA9685 + Safety + Servo Smoother)
add_executable(cpcu_io
    src/cpcu_io.c
    src/nrf24l01_linux.c
    src/cpcu_pca9685.c
    src/cpcu_safety.c
    src/cpcu_smooth.c
)
target_link_libraries(cpcu_io PRIVATE cpcu_codec cpcu_ipc cpcu_log cpcu_config rt m)

## Core 0: Supervisor (spawns cpcu_io + cpcu_dsp.py). Owns runtime config.
add_executable(cpcu_kernel src/cpcu_kernel.c)
target_link_libraries(cpcu_kernel PRIVATE cpcu_config cpcu_ipc cpcu_log rt)

## TUI: ncurses live dashboard (optional, needs ncurses)
##
## CMake's bundled FindCurses.cmake module is broken on Debian bookworm / Pi
## OS — even with CURSES_NEED_WIDE=TRUE it stops at the first libcurses.so
## symlink it finds, which is the narrow 8-bit variant. Narrow ncurses
## garbles multi-byte UTF-8 (prints each byte of a 3-byte glyph as a
## separate cell with "M-x" meta notation).
##
## We work around this with a direct find_library() probe for libncursesw,
## falling back to the standard FindCurses.cmake if the wide variant isn't
## available. Install with: ./setup_pi.sh    (or: sudo apt install libncurses-dev)
find_library(NCURSESW_LIB  NAMES ncursesw)
find_path(NCURSESW_INCLUDE NAMES ncursesw/ncurses.h curses.h)

if(NCURSESW_LIB)
    message(STATUS "Using wide ncurses:  ${NCURSESW_LIB}")
    set(CURSES_FOUND TRUE)
    set(CURSES_LIBRARIES  ${NCURSESW_LIB})
    set(CURSES_INCLUDE_DIRS ${NCURSESW_INCLUDE})
    # _XOPEN_SOURCE_EXTENDED tells <ncurses.h> to expose wide-char APIs like
    # wadd_wch, cchar_t. Harmless when using narrow APIs too.
    add_compile_definitions(_XOPEN_SOURCE_EXTENDED=1 NCURSES_WIDECHAR=1)
else()
    message(STATUS "libncursesw not found — falling back to narrow ncurses")
    find_package(Curses)
endif()

if(CURSES_FOUND)
    message(STATUS "Curses library:      ${CURSES_LIBRARIES}")
    ## v3.4 (2026-04): cpcu_tui split into three .c files for readability.
    ##   src/cpcu_tui.c          main() + key dispatch + splash + boot
    ##   src/cpcu_tui_render.c   drawing primitives, helpers, page draws
    ##   src/cpcu_tui_data.c     demo synthesis, dataset capture, wave ring
    ## All three include the shared header include/cpcu_tui.h.
    add_executable(cpcu_tui
        src/cpcu_tui.c
        src/cpcu_tui_render.c
        src/cpcu_tui_data.c
    )
    target_link_libraries(cpcu_tui PRIVATE cpcu_codec cpcu_ipc cpcu_log ${CURSES_LIBRARIES} rt m)
    target_include_directories(cpcu_tui PRIVATE ${CURSES_INCLUDE_DIRS})
    message(STATUS "ncurses found — cpcu_tui will be built (3-file)")
else()
    message(STATUS "ncurses not found — cpcu_tui will NOT be built")
endif()

##============= TEST TARGETS ===============================================================

## Codec unit test (always buildable, no hardware needed)
add_executable(test_codec test/test_codec.c)
target_link_libraries(test_codec PRIVATE cpcu_codec m)

## Safety module testbench (always buildable, no hardware needed).
## Drives cpcu_safety directly with synthetic inputs and simulated time —
## verifies all 5 fault paths transition at correct thresholds.
add_executable(safety_testbench
    test/safety_testbench.c
    src/cpcu_safety.c
)
target_link_libraries(safety_testbench PRIVATE cpcu_codec m)

## Smoother testbench (always buildable, no hardware needed).
## v2.3.2 — exercises the new hold-pose deadband logic plus the
## existing trapezoidal-motion behaviour. See cpcu_smooth.h v2.1.
add_executable(smooth_testbench
    test/smooth_testbench.c
    src/cpcu_smooth.c
)
target_link_libraries(smooth_testbench PRIVATE m)

## Config-loader testbench (v2.3.3) — exercises the JSON parser and
## validation rules without touching shared memory. See RUNTIME_CONFIG.md.
add_executable(config_testbench
    test/config_testbench.c
)
target_link_libraries(config_testbench PRIVATE cpcu_config cpcu_ipc m)

## PCA9685 servo testbench (interactive TUI, needs ncurses + I2C hardware)
## v2.2: now depends on cpcu_smooth.c (slew/smoother feature exposed via 's' / --smooth)
if(CURSES_FOUND)
    add_executable(pca_testbench
        test/pca_testbench.c
        src/cpcu_pca9685.c
        src/cpcu_smooth.c
    )
    target_link_libraries(pca_testbench PRIVATE cpcu_config cpcu_ipc ${CURSES_LIBRARIES} rt m)
    target_include_directories(pca_testbench PRIVATE ${CURSES_INCLUDE_DIRS})
    message(STATUS "ncurses found — pca_testbench will be built")

    ## Signal integrity testbench (interactive TUI, needs cpcu_kernel + cpcu_io running)
    add_executable(signal_testbench test/signal_testbench.c)
    target_link_libraries(signal_testbench PRIVATE cpcu_codec cpcu_ipc ${CURSES_LIBRARIES} rt m)
    target_include_directories(signal_testbench PRIVATE ${CURSES_INCLUDE_DIRS})
    message(STATUS "ncurses found — signal_testbench will be built")
endif()

## Enable CTest
enable_testing()
add_test(NAME codec_roundtrip COMMAND test_codec)
add_test(NAME safety_fsm      COMMAND safety_testbench)
add_test(NAME smoother_unit   COMMAND smooth_testbench)
add_test(NAME config_loader   COMMAND config_testbench)

##============= INSTALL ====================================================================

## C binaries -> /opt/cpcu/bin
install(TARGETS cpcu_io cpcu_kernel RUNTIME DESTINATION /opt/cpcu/bin)
if(CURSES_FOUND)
    install(TARGETS cpcu_tui RUNTIME DESTINATION /opt/cpcu/bin)
endif()

## Python scripts -> /opt/cpcu/scripts
install(FILES
    scripts/cpcu_dsp.py
    scripts/cpcu_ipc_bridge.py
    DESTINATION /opt/cpcu/scripts
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

## Launch scripts -> /opt/cpcu/scripts
install(FILES
    scripts/launch.sh
    DESTINATION /opt/cpcu/scripts
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

## Python tests -> /opt/cpcu/test (optional)
install(FILES
    test/test_ipc_bridge.py
    test/test_dsp_pipeline.py
    DESTINATION /opt/cpcu/test
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

## Model placeholder directory
install(DIRECTORY DESTINATION /opt/cpcu/models)

## Systemd service
install(FILES
    scripts/cpcu.service
    DESTINATION /etc/systemd/system
)
