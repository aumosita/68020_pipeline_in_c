/*
 * test_runner.c — Integration test harness for the MC68020 emulator.
 *
 * Provides a simple flat-memory bus, loads a hand-coded 68020 program,
 * runs it, and checks that the final register state matches expectations.
 *
 * Test program (assembled inline as bytes):
 *
 *   ; Reset vectors at 0x0000:
 *   DC.L  0x00008000    ; Initial SSP
 *   DC.L  0x00000008    ; Initial PC
 *
 *   ; Code starts at 0x0008:
 *   MOVEQ  #100, D0     ; D0 = 100  (countdown counter)
 *   MOVEQ  #0,   D1     ; D1 = 0    (accumulator)
 * loop:
 *   ADD.L  D0, D1       ; D1 += D0
 *   SUBQ.L #1, D0       ; D0--
 *   BNE.B  loop         ; branch if D0 != 0
 *   ILLEGAL             ; halt (triggers illegal instruction exception)
 *
 * Expected result after ILLEGAL:
 *   D0 = 0
 *   D1 = 5050  (sum of 1..100)
 */

#include "m68020.h"
#include "m68020_internal.h"  /* for cycle_count / instr_count inspection */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Flat-memory bus implementation                                      */
/* ------------------------------------------------------------------ */

#define MEM_SIZE (1u << 20)   /* 1 MB flat RAM */

typedef struct {
    uint8_t mem[MEM_SIZE];
} FlatBus;

static BusResult flat_read(void *ctx, u32 addr, BusSize size,
                            FunctionCode fc, u32 *val, u32 *cycles_out) {
    FlatBus *b = (FlatBus *)ctx;
    *cycles_out = 0;
    (void)fc;

    if (addr >= MEM_SIZE) { *val = 0xFFFFFFFFu; return BUS_ERROR; }

    switch (size) {
        case SIZE_BYTE: *val = b->mem[addr]; break;
        case SIZE_WORD:
            *val = ((u32)b->mem[addr] << 8) | b->mem[addr + 1];
            break;
        case SIZE_LONG:
            *val = ((u32)b->mem[addr    ] << 24)
                 | ((u32)b->mem[addr + 1] << 16)
                 | ((u32)b->mem[addr + 2] <<  8)
                 |  (u32)b->mem[addr + 3];
            break;
    }
    return BUS_OK;
}

static BusResult flat_write(void *ctx, u32 addr, BusSize size,
                             FunctionCode fc, u32 val, u32 *cycles_out) {
    FlatBus *b = (FlatBus *)ctx;
    *cycles_out = 0;
    (void)fc;

    if (addr >= MEM_SIZE) return BUS_ERROR;

    switch (size) {
        case SIZE_BYTE: b->mem[addr] = (uint8_t)val; break;
        case SIZE_WORD:
            b->mem[addr    ] = (uint8_t)(val >> 8);
            b->mem[addr + 1] = (uint8_t)(val     );
            break;
        case SIZE_LONG:
            b->mem[addr    ] = (uint8_t)(val >> 24);
            b->mem[addr + 1] = (uint8_t)(val >> 16);
            b->mem[addr + 2] = (uint8_t)(val >>  8);
            b->mem[addr + 3] = (uint8_t)(val      );
            break;
    }
    return BUS_OK;
}

static u8 flat_iack(void *ctx, u8 level) {
    (void)ctx; (void)level;
    return 0xFF;  /* autovector */
}

/* ------------------------------------------------------------------ */
/* Test program                                                        */
/* ------------------------------------------------------------------ */

/*
 * 68020 machine code (big-endian):
 *
 * 0x0000: DC.L 0x00008000    ; reset SSP
 * 0x0004: DC.L 0x00000008    ; reset PC
 * 0x0008: MOVEQ #100, D0     ; 70 64
 * 0x000A: MOVEQ #0, D1       ; 72 00
 * 0x000C: ADD.L D0,D1        ; D200 = ADD.L D0,D1  <- actually wrong
 *         ; ADD Dn,<ea>: 1101 DDD0 ss ea
 *         ;   1101 000 0 10 000 001 = D080 + (ea for D1) = D081
 *         ; Wait: ADD.L D0,D1 — source=D0, dest=D1
 *         ; Encoding: 1101 001 0 10 000 000 = D280
 *         ;   (dn=D1, dir=0 meaning ea→Dn, sz=10=LONG, ea=000 000=D0)
 * 0x000C: D2 80               ; ADD.L D0,D1
 * 0x000E: 53 80               ; SUBQ.L #1,D0  (0101 001 1 10 000 000)
 *                             ; Wait: SUBQ #1,D0
 *                             ; 0101 DDD1 ss ea, DDD=001(=#1), ss=10(=.L), ea=D0
 *                             ; = 0101 001 1 10 000 000 = 5380
 * 0x0010: 66 FA               ; BNE.B -6 (back to 0x000C)
 *                             ; 0110 0110 1111 1010
 *                             ; BNE = 0x66xx, disp = -6 = 0xFA
 * 0x0012: 4A FC               ; ILLEGAL
 *
 * Vector for ILLEGAL (vector 4): at 0x0010 = 4*4 = 0x0010... but 0x0010 is code!
 * We need the vector table at 0x0000. Let's put the illegal handler at 0x0100.
 * Vector 4 address: VBR + 4*4 = 0 + 16 = 0x0010. Hmm conflict.
 *
 * Better: put code at 0x0100, illegal handler at 0x0200.
 */

static const uint8_t test_program[] = {
    /* 0x0000: Reset SSP = 0x00008000 */
    0x00, 0x00, 0x80, 0x00,
    /* 0x0004: Reset PC  = 0x00000100 */
    0x00, 0x00, 0x01, 0x00,

    /* 0x0008: Vector 2 (bus error)      → 0x0200 */
    0x00, 0x00, 0x02, 0x00,
    /* 0x000C: Vector 3 (address error)  → 0x0200 */
    0x00, 0x00, 0x02, 0x00,
    /* 0x0010: Vector 4 (illegal insn)   → 0x0200 */
    0x00, 0x00, 0x02, 0x00,
};

/* Code at 0x0100 */
static const uint8_t test_code[] = {
    /* 0x0100: MOVEQ #100, D0   = 70 64 */
    0x70, 0x64,
    /* 0x0102: MOVEQ #0, D1    = 72 00 */
    0x72, 0x00,
    /* 0x0104: ADD.L D0,D1     = D2 80
     *   1101 001 0 10 000 000  = D280
     *   dn=D1, dir=0, size=10(LONG), ea=000 000(D0) */
    0xD2, 0x80,
    /* 0x0106: SUBQ.L #1,D0    = 53 80
     *   0101 001 1 10 000 000 = 5380
     *   imm=1(001), dir=1, sz=10(LONG), ea=D0 */
    0x53, 0x80,
    /* 0x0108: BNE.B -6         = 66 FA
     *   target = 0x010A + (-6) = 0x0104 ✓ */
    0x66, 0xFA,
    /* 0x010A: ILLEGAL          = 4A FC */
    0x4A, 0xFC,
};

/* Illegal instruction handler at 0x0200 — just halts via STOP #0x2700 */
static const uint8_t illegal_handler[] = {
    /* 0x0200: STOP #0x2700 = 4E 72 27 00 */
    0x4E, 0x72, 0x27, 0x00,
};

/* ------------------------------------------------------------------ */
/* Test infrastructure                                                 */
/* ------------------------------------------------------------------ */

static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, fmt, ...)                                               \
    do {                                                                    \
        tests_run++;                                                        \
        if (cond) {                                                         \
            tests_passed++;                                                 \
            printf("  PASS: " fmt "\n", ##__VA_ARGS__);                    \
        } else {                                                            \
            tests_failed++;                                                 \
            printf("  FAIL: " fmt "\n", ##__VA_ARGS__);                    \
        }                                                                   \
    } while (0)

/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

static void test_countdown_loop(void) {
    printf("\n=== Test: countdown loop (sum 1..100) ===\n");

    FlatBus bus_data;
    memset(&bus_data, 0, sizeof bus_data);

    /* Load vectors and code */
    memcpy(bus_data.mem + 0x0000, test_program,   sizeof test_program);
    memcpy(bus_data.mem + 0x0100, test_code,       sizeof test_code);
    memcpy(bus_data.mem + 0x0200, illegal_handler, sizeof illegal_handler);

    M68020BusInterface bus = {
        .read             = flat_read,
        .write            = flat_write,
        .iack             = flat_iack,
        .reset_peripherals = NULL,
        .ctx              = &bus_data,
    };

    M68020State *cpu = m68020_create(&bus);
    if (!cpu) { printf("  FAIL: could not create CPU\n"); return; }

    m68020_reset(cpu);

    /* Run for up to 100 000 cycles */
    m68020_run(cpu, 100000);

    u32 d0 = m68020_get_reg(cpu, REG_D0);
    u32 d1 = m68020_get_reg(cpu, REG_D1);
    u32 pc = m68020_get_reg(cpu, REG_PC);

    printf("  D0 = %u (expected 0)\n",    d0);
    printf("  D1 = %u (expected 5050)\n", d1);
    printf("  PC = 0x%08X\n", pc);
    printf("  cycles = %llu\n", (unsigned long long)cpu->cycle_count);
    printf("  instructions = %u\n", cpu->instr_count);

    CHECK(d0 == 0,    "D0 = 0 after loop");
    CHECK(d1 == 5050, "D1 = 5050 (sum 1..100)");

    m68020_destroy(cpu);
}

static void test_addx_multiword(void) {
    printf("\n=== Test: ADDX multi-word arithmetic ===\n");

    FlatBus bus_data;
    memset(&bus_data, 0, sizeof bus_data);
    uint8_t *m = bus_data.mem;

    /* Reset vectors */
    m[0] = 0x00; m[1] = 0x00; m[2] = 0x80; m[3] = 0x00;  /* SSP */
    m[4] = 0x00; m[5] = 0x00; m[6] = 0x01; m[7] = 0x00;  /* PC  */
    /* Vector 4 (illegal) → 0x0200 */
    m[16] = 0x00; m[17] = 0x00; m[18] = 0x02; m[19] = 0x00;

    /*
     * Code at 0x0100:
     *   MOVEQ #0, D0        ; D0 = 0 (high word of 64-bit sum)
     *   MOVEQ #0, D1        ; D1 = 0 (high word of 64-bit sum, extended)
     *   MOVEQ #-1, D2       ; D2 = 0xFFFFFFFF
     *   MOVEQ #-1, D3       ; D3 = 0xFFFFFFFF
     *   ADD.L  D3, D2       ; D2 = 0xFFFFFFFE, X=1 (carry)
     *   ADDX.L D1, D0       ; D0 = 0 + 0 + X = 1
     *   ILLEGAL
     *
     * Expected: D0 = 1, D2 = 0xFFFFFFFE
     */
    m[0x100] = 0x70; m[0x101] = 0x00;  /* MOVEQ #0,  D0 */
    m[0x102] = 0x72; m[0x103] = 0x00;  /* MOVEQ #0,  D1 */
    m[0x104] = 0x74; m[0x105] = 0xFF;  /* MOVEQ #-1, D2 */
    m[0x106] = 0x76; m[0x107] = 0xFF;  /* MOVEQ #-1, D3 */
    m[0x108] = 0xD4; m[0x109] = 0x83;  /* ADD.L  D3,D2  → 1101 010 0 10 000 011 = D483 */
    m[0x10A] = 0xD1; m[0x10B] = 0x81;  /* ADDX.L D1,D0  → 1101 000 1 10 000 001 = D181 */
    m[0x10C] = 0x4A; m[0x10D] = 0xFC;  /* ILLEGAL */

    /* Handler at 0x0200: STOP */
    m[0x200] = 0x4E; m[0x201] = 0x72; m[0x202] = 0x27; m[0x203] = 0x00;

    M68020BusInterface bus = {
        .read = flat_read, .write = flat_write, .iack = flat_iack,
        .reset_peripherals = NULL, .ctx = &bus_data
    };
    M68020State *cpu = m68020_create(&bus);
    m68020_reset(cpu);
    m68020_run(cpu, 10000);

    u32 d0 = m68020_get_reg(cpu, REG_D0);
    u32 d2 = m68020_get_reg(cpu, REG_D2);

    CHECK(d0 == 1u,          "D0 = 1 after ADDX carry");
    CHECK(d2 == 0xFFFFFFFEu, "D2 = 0xFFFFFFFE after ADD.L");

    m68020_destroy(cpu);
}

static void test_move_immediate(void) {
    printf("\n=== Test: MOVE immediate and MOVEQ ===\n");

    FlatBus bus_data;
    memset(&bus_data, 0, sizeof bus_data);

    /* Build test:
     *   0x0000: SSP = 0x8000
     *   0x0004: PC  = 0x0100
     *   0x0010: vec 4 (illegal) → 0x0200
     */
    uint8_t *m = bus_data.mem;

    /* Reset vectors */
    m[0] = 0x00; m[1] = 0x00; m[2] = 0x80; m[3] = 0x00;  /* SSP */
    m[4] = 0x00; m[5] = 0x00; m[6] = 0x01; m[7] = 0x00;  /* PC */
    /* Vector 4 */
    m[16] = 0x00; m[17] = 0x00; m[18] = 0x02; m[19] = 0x00;

    /* Code at 0x0100:
     *   MOVEQ #42, D3   = 76 2A
     *   MOVEQ #-1, D5   = 7A FF
     *   ILLEGAL         = 4A FC
     */
    m[0x100] = 0x76; m[0x101] = 0x2A;
    m[0x102] = 0x7A; m[0x103] = 0xFF;
    m[0x104] = 0x4A; m[0x105] = 0xFC;

    /* Handler at 0x0200: STOP */
    m[0x200] = 0x4E; m[0x201] = 0x72; m[0x202] = 0x27; m[0x203] = 0x00;

    M68020BusInterface bus = {
        .read = flat_read, .write = flat_write, .iack = flat_iack,
        .reset_peripherals = NULL, .ctx = &bus_data
    };
    M68020State *cpu = m68020_create(&bus);
    m68020_reset(cpu);
    m68020_run(cpu, 10000);

    u32 d3 = m68020_get_reg(cpu, REG_D3);
    u32 d5 = m68020_get_reg(cpu, REG_D5);

    CHECK(d3 == 42,          "D3 = 42 after MOVEQ #42,D3");
    CHECK(d5 == 0xFFFFFFFFu, "D5 = -1 after MOVEQ #-1,D5");

    m68020_destroy(cpu);
}

/* ------------------------------------------------------------------ */
/* Phase 3 Tests                                                       */
/* ------------------------------------------------------------------ */

/*
 * test_trapcc: Verify TRAPcc (68020) fires only when condition is true.
 *
 * Code:
 *   MOVEQ #5, D0       ; D0 = 5
 *   TRAPF              ; cc=F (always false) — must NOT trap
 *   MOVEQ #10, D0      ; D0 = 10
 *   TRAPT              ; cc=T (always true)  — MUST trap → 0x0300 handler
 *   MOVEQ #99, D0      ; must NOT execute
 *   ILLEGAL            ; must NOT execute
 *
 * TRAPV handler at 0x0300:
 *   MOVEQ #0x55, D7   ; mark handler was reached
 *   ILLEGAL           ; terminate via illegal-insn handler → STOP
 *
 * Expected: D0=10, D7=0x55
 */
static void test_trapcc(void) {
    printf("\n=== Test: TRAPcc (68020) ===\n");

    FlatBus bus_data;
    memset(&bus_data, 0, sizeof bus_data);
    uint8_t *m = bus_data.mem;

    /* Reset vectors */
    m[0] = 0x00; m[1] = 0x00; m[2] = 0x80; m[3] = 0x00;  /* SSP */
    m[4] = 0x00; m[5] = 0x00; m[6] = 0x01; m[7] = 0x00;  /* PC  */
    /* Vector 4 (illegal) → 0x0200 */
    m[16] = 0x00; m[17] = 0x00; m[18] = 0x02; m[19] = 0x00;
    /* Vector 7 (TRAPV) → 0x0300 */
    m[28] = 0x00; m[29] = 0x00; m[30] = 0x03; m[31] = 0x00;

    /* Code at 0x0100 */
    m[0x100] = 0x70; m[0x101] = 0x05;  /* MOVEQ #5, D0      */
    m[0x102] = 0x51; m[0x103] = 0xFC;  /* TRAPF (cc=F)      */
    m[0x104] = 0x70; m[0x105] = 0x0A;  /* MOVEQ #10, D0     */
    m[0x106] = 0x50; m[0x107] = 0xFC;  /* TRAPT (cc=T)      */
    m[0x108] = 0x70; m[0x109] = 0x63;  /* MOVEQ #99, D0 (should not run) */
    m[0x10A] = 0x4A; m[0x10B] = 0xFC;  /* ILLEGAL (should not run)       */

    /* Illegal handler at 0x0200: STOP #0x2700 */
    m[0x200] = 0x4E; m[0x201] = 0x72; m[0x202] = 0x27; m[0x203] = 0x00;

    /* TRAPV handler at 0x0300 */
    m[0x300] = 0x7E; m[0x301] = 0x55;  /* MOVEQ #0x55, D7 */
    m[0x302] = 0x4A; m[0x303] = 0xFC;  /* ILLEGAL         */

    M68020BusInterface bus = {
        .read = flat_read, .write = flat_write, .iack = flat_iack,
        .reset_peripherals = NULL, .ctx = &bus_data
    };
    M68020State *cpu = m68020_create(&bus);
    m68020_reset(cpu);
    m68020_run(cpu, 10000);

    u32 d0 = m68020_get_reg(cpu, REG_D0);
    u32 d7 = m68020_get_reg(cpu, REG_D7);

    CHECK(d0 == 10u,   "D0 = 10 (TRAPT fired, MOVEQ #99 skipped)");
    CHECK(d7 == 0x55u, "D7 = 0x55 (TRAPV handler executed)");

    m68020_destroy(cpu);
}

/*
 * test_cmp2: Verify CMP2.W bounds check (68020).
 *
 * Memory at 0x0400: lower=10 (word), upper=20 (word).
 *
 * Code:
 *   MOVEQ #15, D4      ; in range [10..20] → C should be 0
 *   CMP2.W $0400.L, D4
 *   SCS D6             ; D6 = 0xFF if C=1 (out of range)
 *   MOVEQ #5, D4       ; out of range (< 10) → C should be 1
 *   CMP2.W $0400.L, D4
 *   SCS D7             ; D7 = 0xFF if C=1
 *   ILLEGAL
 *
 * CMP2.W (abs.L), D4 encoding:
 *   opword : 0x02F9  (0000 0010 11 111 001 — word size, abs.L EA)
 *   extword: 0x4000  (D4 = reg 4, is_an=0, is_chk=0)
 *   address: 0x00000400
 *
 * SCS (set if carry set, cc=5=0101) Dn: 0101 0101 11 000 rrr = 0x55C0|r
 *
 * Expected: D6=0x00 (in range), D7=0xFF (out of range)
 */
static void test_cmp2(void) {
    printf("\n=== Test: CMP2.W bounds check (68020) ===\n");

    FlatBus bus_data;
    memset(&bus_data, 0, sizeof bus_data);
    uint8_t *m = bus_data.mem;

    /* Reset vectors */
    m[0] = 0x00; m[1] = 0x00; m[2] = 0x80; m[3] = 0x00;  /* SSP */
    m[4] = 0x00; m[5] = 0x00; m[6] = 0x01; m[7] = 0x00;  /* PC  */
    /* Vector 4 (illegal) → 0x0200 */
    m[16] = 0x00; m[17] = 0x00; m[18] = 0x02; m[19] = 0x00;

    /* Bounds at 0x0400: lower=10, upper=20 (big-endian words) */
    m[0x400] = 0x00; m[0x401] = 0x0A;  /* lower = 10 */
    m[0x402] = 0x00; m[0x403] = 0x14;  /* upper = 20 */

    /* Code at 0x0100 */
    uint32_t p = 0x100;
    m[p++] = 0x78; m[p++] = 0x0F;  /* MOVEQ #15, D4 */
    /* CMP2.W $0400.L, D4 */
    m[p++] = 0x02; m[p++] = 0xF9;  /* opword  */
    m[p++] = 0x40; m[p++] = 0x00;  /* extword */
    m[p++] = 0x00; m[p++] = 0x00; m[p++] = 0x04; m[p++] = 0x00; /* addr */
    m[p++] = 0x55; m[p++] = 0xC6;  /* SCS D6 */
    m[p++] = 0x78; m[p++] = 0x05;  /* MOVEQ #5, D4 */
    /* CMP2.W $0400.L, D4 */
    m[p++] = 0x02; m[p++] = 0xF9;
    m[p++] = 0x40; m[p++] = 0x00;
    m[p++] = 0x00; m[p++] = 0x00; m[p++] = 0x04; m[p++] = 0x00;
    m[p++] = 0x55; m[p++] = 0xC7;  /* SCS D7 */
    m[p++] = 0x4A; m[p++] = 0xFC;  /* ILLEGAL */

    /* Illegal handler at 0x0200: STOP #0x2700 */
    m[0x200] = 0x4E; m[0x201] = 0x72; m[0x202] = 0x27; m[0x203] = 0x00;

    M68020BusInterface bus = {
        .read = flat_read, .write = flat_write, .iack = flat_iack,
        .reset_peripherals = NULL, .ctx = &bus_data
    };
    M68020State *cpu = m68020_create(&bus);
    m68020_reset(cpu);
    m68020_run(cpu, 10000);

    u32 d6 = m68020_get_reg(cpu, REG_D6);
    u32 d7 = m68020_get_reg(cpu, REG_D7);

    CHECK(d6 == 0x00u, "D6 = 0x00 (15 in bounds [10..20], C=0)");
    CHECK(d7 == 0xFFu, "D7 = 0xFF (5 out of bounds [10..20], C=1)");

    m68020_destroy(cpu);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void) {
    printf("MC68020 Emulator — Integration Tests\n");
    printf("=====================================\n");

    test_countdown_loop();
    test_addx_multiword();
    test_move_immediate();
    test_trapcc();
    test_cmp2();

    printf("\n=====================================\n");
    printf("Results: %d/%d passed", tests_passed, tests_run);
    if (tests_failed)
        printf(" (%d FAILED)", tests_failed);
    printf("\n");

    return tests_failed ? 1 : 0;
}
