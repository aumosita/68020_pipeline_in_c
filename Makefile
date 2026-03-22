CC      = cc
CFLAGS  = -std=c11 -Wall -Wextra -Wpedantic -Wshadow \
          -Wno-unused-parameter -fno-strict-aliasing \
          -g -O0 -Iinclude -MMD -MP

SRCS = \
    src/cpu/m68020_core.c \
    src/cpu/m68020_state.c \
    src/cpu/m68020_pipeline.c \
    src/cpu/m68020_pipeline_stages.c \
    src/cpu/m68020_cycle_timing.c \
    src/cpu/m68020_cache.c \
    src/cpu/m68020_ea.c \
    src/cpu/m68020_exceptions.c \
    src/cpu/m68020_privileged.c \
    src/bus/m68020_bus.c \
    src/bus/m68020_bus_error.c \
    src/bus/m68020_memmap.c \
    src/instructions/m68020_move.c \
    src/instructions/m68020_branch.c \
    src/instructions/m68020_jump.c \
    src/instructions/m68020_alu.c \
    src/instructions/m68020_shift.c \
    src/instructions/m68020_mul_div.c \
    src/instructions/m68020_load_store.c \
    src/instructions/m68020_compare.c \
    src/instructions/m68020_cas.c \
    src/instructions/m68020_misc.c \
    src/instructions/m68020_bitfield.c \
    src/instructions/m68020_callm_rtm.c \
    src/instructions/m68020_coproc.c \
    src/debug/m68020_disasm.c \
    src/debug/m68020_trace.c

OBJS = $(SRCS:.c=.o)

LIB  = libm68020.a
TEST = test_runner
TEST_SYS = test_systematic
TEST_VEC = test_vector_runner

.PHONY: all clean test test-all test-vectors

all: $(TEST) $(TEST_SYS) $(TEST_VEC)

$(LIB): $(OBJS)
	ar rcs $@ $^

$(TEST): tests/integration/test_runner.c $(LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -lm68020

$(TEST_SYS): tests/validation/test_systematic.c $(LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -lm68020

$(TEST_VEC): tests/vectors/test_vector_runner.c $(LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -lm68020

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

test: $(TEST) $(TEST_SYS)
	./$(TEST)
	./$(TEST_SYS)

test-vectors: $(TEST_VEC)
	python3 tests/vectors/run_vectors.py

DEPS = $(OBJS:.o=.d)
-include $(DEPS)

clean:
	rm -f $(OBJS) $(DEPS) $(LIB) $(TEST)
