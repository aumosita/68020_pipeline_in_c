cmake_minimum_required(VERSION 3.20)
project(m68020_emulator C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Strict warnings — emulator correctness requires catching all UB
add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wshadow
    -Wno-unused-parameter
    -fno-strict-aliasing   # needed for memory modelling (u32* / u16* aliasing)
)

# -----------------------------------------------------------------------
# Core emulator library
# -----------------------------------------------------------------------
add_library(m68020 STATIC
    src/cpu/m68020_core.c
    src/cpu/m68020_state.c
    src/cpu/m68020_pipeline.c
    src/cpu/m68020_pipeline_stages.c
    src/cpu/m68020_cycle_timing.c
    src/cpu/m68020_cache.c
    src/cpu/m68020_ea.c
    src/cpu/m68020_exceptions.c
    src/cpu/m68020_privileged.c
    src/bus/m68020_bus.c
    src/bus/m68020_bus_error.c
    src/instructions/m68020_move.c
    src/instructions/m68020_branch.c
    src/instructions/m68020_jump.c
    src/instructions/m68020_alu.c
    src/instructions/m68020_shift.c
    src/instructions/m68020_mul_div.c
    src/instructions/m68020_load_store.c
    src/instructions/m68020_compare.c
    src/instructions/m68020_cas.c
    src/instructions/m68020_misc.c
    src/instructions/m68020_bitfield.c
    src/instructions/m68020_callm_rtm.c
    src/instructions/m68020_coproc.c
    src/debug/m68020_disasm.c
    src/debug/m68020_trace.c
)

target_include_directories(m68020
    PUBLIC  include
    PRIVATE src
)

# -----------------------------------------------------------------------
# Integration test runner
# -----------------------------------------------------------------------
add_executable(test_runner tests/integration/test_runner.c)
target_link_libraries(test_runner m68020)
target_include_directories(test_runner PRIVATE include)

# -----------------------------------------------------------------------
# Systematic validation test
# -----------------------------------------------------------------------
add_executable(test_systematic tests/validation/test_systematic.c)
target_link_libraries(test_systematic m68020)
target_include_directories(test_systematic PRIVATE include)

# -----------------------------------------------------------------------
# CTest integration
# -----------------------------------------------------------------------
enable_testing()
add_test(NAME integration COMMAND test_runner)
add_test(NAME systematic  COMMAND test_systematic)
