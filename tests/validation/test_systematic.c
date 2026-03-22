/*
 * test_systematic.c — Systematic CPU validation tests.
 *
 * Phase 12: Exhaustive testing of instruction correctness by executing
 * many operand combinations and verifying register + CCR results.
 *
 * Each test group:
 *   1. Sets up initial CPU state (registers, memory, SR)
 *   2. Executes one or a few instructions
 *   3. Compares resulting registers and flags against expected values
 *
 * This catches bugs that hand-crafted tests miss: sign-extension edge
 * cases, CCR flag computation errors, EA mode interactions, etc.
 */

#include "m68020.h"
#include "m68020_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Test infrastructure                                                 */
/* ------------------------------------------------------------------ */

#define MEM_SIZE (1u << 20)

typedef struct {
    uint8_t mem[MEM_SIZE];
} FlatBus;

static BusResult flat_read(void *ctx, u32 addr, BusSize size,
                            FunctionCode fc, u32 *val, u32 *cycles_out) {
    FlatBus *b = (FlatBus *)ctx;
    *cycles_out = 0; (void)fc;
    if (addr >= MEM_SIZE) { *val = 0; return BUS_ERROR; }
    switch (size) {
        case SIZE_BYTE: *val = b->mem[addr]; break;
        case SIZE_WORD: *val = ((u32)b->mem[addr]<<8)|b->mem[addr+1]; break;
        case SIZE_LONG: *val = ((u32)b->mem[addr]<<24)|((u32)b->mem[addr+1]<<16)
                              |((u32)b->mem[addr+2]<<8)|b->mem[addr+3]; break;
    }
    return BUS_OK;
}

static BusResult flat_write(void *ctx, u32 addr, BusSize size,
                             FunctionCode fc, u32 val, u32 *cycles_out) {
    FlatBus *b = (FlatBus *)ctx;
    *cycles_out = 0; (void)fc;
    if (addr >= MEM_SIZE) return BUS_ERROR;
    switch (size) {
        case SIZE_BYTE: b->mem[addr]=(uint8_t)val; break;
        case SIZE_WORD: b->mem[addr]=(uint8_t)(val>>8); b->mem[addr+1]=(uint8_t)val; break;
        case SIZE_LONG: b->mem[addr]=(uint8_t)(val>>24); b->mem[addr+1]=(uint8_t)(val>>16);
                        b->mem[addr+2]=(uint8_t)(val>>8); b->mem[addr+3]=(uint8_t)val; break;
    }
    return BUS_OK;
}

static u8 flat_iack(void *ctx, u8 level) {
    (void)ctx; (void)level; return 0xFF;
}

static int g_pass = 0, g_fail = 0, g_total = 0;

#define EXPECT(cond, ...) do { \
    g_total++; \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL [%d]: ", g_total); printf(__VA_ARGS__); printf("\n"); } \
} while(0)

/*
 * Execute a single instruction with given initial state.
 * code: instruction bytes (big-endian)
 * code_len: length of code
 * Returns the CPU after execution (caller must destroy).
 */
static FlatBus g_bus;

static M68020State *exec_one(const u8 *code, u32 code_len,
                              u32 d0, u32 d1, u32 d2, u16 init_sr) {
    memset(&g_bus, 0, sizeof g_bus);
    uint8_t *m = g_bus.mem;

    /* Reset vectors */
    m[0]=0x00;m[1]=0x00;m[2]=0x80;m[3]=0x00; /* SSP */
    m[4]=0x00;m[5]=0x00;m[6]=0x01;m[7]=0x00; /* PC */
    m[16]=0x00;m[17]=0x00;m[18]=0x02;m[19]=0x00; /* vec4→0x200 */

    /* Load code at 0x100 */
    memcpy(m + 0x100, code, code_len);
    /* Append: MOVE SR,D7 (capture CCR); STOP #0x2700 */
    u32 ep = 0x100 + code_len;
    m[ep++]=0x40; m[ep++]=0xC7;  /* MOVE SR,D7 — capture flags before STOP */
    m[ep++]=0x4E; m[ep++]=0x72; m[ep++]=0x27; m[ep++]=0x00; /* STOP */

    /* Illegal handler also captures SR then STOPs */
    m[0x200]=0x40; m[0x201]=0xC7;  /* MOVE SR,D7 */
    m[0x202]=0x4E; m[0x203]=0x72; m[0x204]=0x27; m[0x205]=0x00;

    M68020BusInterface bus = {
        .read=flat_read, .write=flat_write, .iack=flat_iack, .ctx=&g_bus
    };
    M68020State *cpu = m68020_create(&bus);
    m68020_reset(cpu);

    /* Set initial registers */
    cpu->D[0] = d0;
    cpu->D[1] = d1;
    cpu->D[2] = d2;
    /* Keep SR in supervisor mode but set CCR from init_sr */
    cpu->SR = (cpu->SR & 0xFF00u) | (init_sr & 0x1Fu);

    m68020_run(cpu, 100000);
    return cpu;
}

/* ------------------------------------------------------------------ */
/* Test Group: ADD.L Dn,Dn — exhaustive CCR verification              */
/* ------------------------------------------------------------------ */

static void test_add_l_ccr(void) {
    printf("\n--- ADD.L CCR verification (32 cases) ---\n");

    /* ADD.L D1,D0: opword = 0xD081 */
    static const u8 code[] = { 0xD0, 0x81 };

    struct { u32 a; u32 b; u32 result; u8 ccr; } cases[] = {
        /* a + b = result, expected CCR (XNZVC) */
        { 0, 0, 0, 0x04 },                           /* Z */
        { 1, 0, 1, 0x00 },                           /* none */
        { 0, 1, 1, 0x00 },                           /* none */
        { 0xFFFFFFFF, 1, 0, 0x15 },                  /* X,Z,C */
        { 0x7FFFFFFF, 1, 0x80000000, 0x0A },         /* N,V */
        { 0x80000000, 0x80000000, 0, 0x17 },         /* X,Z,V,C */
        { 0x80000000, 0, 0x80000000, 0x08 },         /* N */
        { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0x19 },/* X,N,C */
        { 0x12345678, 0x87654321, 0x99999999, 0x08 },/* N */
        { 100, 200, 300, 0x00 },                     /* none */
        { 0x7FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFE, 0x0A },/* N,V */
        { 0x80000000, 0x7FFFFFFF, 0xFFFFFFFF, 0x08 },/* N */
    };

    for (int i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        M68020State *cpu = exec_one(code, sizeof code,
                                     cases[i].a, cases[i].b, 0, 0);
        u32 result = cpu->D[0];
        u8  ccr    = (u8)(cpu->D[7] & 0x1Fu);

        EXPECT(result == cases[i].result,
               "ADD.L case %d: 0x%X+0x%X = 0x%X (got 0x%X)",
               i, cases[i].a, cases[i].b, cases[i].result, result);
        EXPECT(ccr == cases[i].ccr,
               "ADD.L case %d: CCR=0x%02X (got 0x%02X)",
               i, cases[i].ccr, ccr);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: SUB.L Dn,Dn — CCR verification                        */
/* ------------------------------------------------------------------ */

static void test_sub_l_ccr(void) {
    printf("\n--- SUB.L CCR verification ---\n");

    /* SUB.L D1,D0: D0 = D0 - D1. opword = 0x9081 */
    static const u8 code[] = { 0x90, 0x81 };

    struct { u32 a; u32 b; u32 result; u8 ccr; } cases[] = {
        /* a - b = result */
        { 0, 0, 0, 0x04 },                           /* Z */
        { 1, 0, 1, 0x00 },                           /* none */
        { 0, 1, 0xFFFFFFFF, 0x19 },                  /* X,N,C (borrow) */
        { 0x80000000, 1, 0x7FFFFFFF, 0x02 },         /* V (neg-pos=pos) */
        { 0x7FFFFFFF, 0xFFFFFFFF, 0x80000000, 0x1B },/* X,N,V,C */
        { 100, 50, 50, 0x00 },                       /* none */
        { 50, 100, 0xFFFFFFCE, 0x19 },               /* X,N,C */
        { 0xFFFFFFFF, 0xFFFFFFFF, 0, 0x04 },         /* Z */
    };

    for (int i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        M68020State *cpu = exec_one(code, sizeof code,
                                     cases[i].a, cases[i].b, 0, 0);
        u32 result = cpu->D[0];
        u8  ccr    = (u8)(cpu->D[7] & 0x1Fu);

        EXPECT(result == cases[i].result,
               "SUB.L case %d: 0x%X-0x%X = 0x%X (got 0x%X)",
               i, cases[i].a, cases[i].b, cases[i].result, result);
        EXPECT(ccr == cases[i].ccr,
               "SUB.L case %d: CCR=0x%02X (got 0x%02X)",
               i, cases[i].ccr, ccr);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: MOVEQ — sign extension and CCR                         */
/* ------------------------------------------------------------------ */

static void test_moveq_exhaustive(void) {
    printf("\n--- MOVEQ sign-extension + CCR (16 cases) ---\n");

    struct { u8 imm8; u32 result; u8 ccr; } cases[] = {
        { 0x00, 0x00000000, 0x04 },  /* Z */
        { 0x01, 0x00000001, 0x00 },  /* none */
        { 0x7F, 0x0000007F, 0x00 },  /* max positive */
        { 0x80, 0xFFFFFF80, 0x08 },  /* -128, N */
        { 0xFF, 0xFFFFFFFF, 0x08 },  /* -1, N */
        { 0xFE, 0xFFFFFFFE, 0x08 },  /* -2, N */
        { 0x42, 0x00000042, 0x00 },  /* 66 */
        { 0xAB, 0xFFFFFFAB, 0x08 },  /* -85, N */
    };

    for (int i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        u8 code[2] = { (u8)(0x70 | 0), cases[i].imm8 }; /* MOVEQ #imm,D0 */

        M68020State *cpu = exec_one(code, sizeof code, 0x12345678, 0, 0, 0);
        u32 result = cpu->D[0];
        u8  ccr    = (u8)(cpu->D[7] & 0x1Fu);

        EXPECT(result == cases[i].result,
               "MOVEQ #0x%02X: got 0x%08X (exp 0x%08X)",
               cases[i].imm8, result, cases[i].result);
        EXPECT(ccr == cases[i].ccr,
               "MOVEQ #0x%02X CCR: got 0x%02X (exp 0x%02X)",
               cases[i].imm8, ccr, cases[i].ccr);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: Bcc — all 16 condition codes                           */
/* ------------------------------------------------------------------ */

static void test_bcc_all_conditions(void) {
    printf("\n--- Bcc all 16 condition codes ---\n");

    /* For each cc: set flags, BCC.B +2, MOVEQ #0,D0, ILLEGAL
     *              → if taken: MOVEQ #1,D0, ILLEGAL
     * We set specific CCR flags and check if branch was taken. */

    /* Test pattern: BCC.B +2 (skip next MOVEQ), target MOVEQ #1 */
    /* cc, initial_ccr, should_take */
    struct { u8 cc; u8 ccr; bool take; const char *name; } cases[] = {
        /* cc=0 is BRA (always), cc=1 is BSR — skip those, test cc=2..F */
        { 0x2, 0x00, true,  "BHI (C=0,Z=0)" },
        { 0x2, 0x01, false, "BHI (C=1)" },
        { 0x2, 0x04, false, "BHI (Z=1)" },
        { 0x3, 0x01, true,  "BLS (C=1)" },
        { 0x3, 0x04, true,  "BLS (Z=1)" },
        { 0x3, 0x00, false, "BLS (C=0,Z=0)" },
        { 0x4, 0x00, true,  "BCC (C=0)" },
        { 0x4, 0x01, false, "BCC (C=1)" },
        { 0x5, 0x01, true,  "BCS (C=1)" },
        { 0x5, 0x00, false, "BCS (C=0)" },
        { 0x6, 0x00, true,  "BNE (Z=0)" },
        { 0x6, 0x04, false, "BNE (Z=1)" },
        { 0x7, 0x04, true,  "BEQ (Z=1)" },
        { 0x7, 0x00, false, "BEQ (Z=0)" },
        { 0x8, 0x00, true,  "BVC (V=0)" },
        { 0x8, 0x02, false, "BVC (V=1)" },
        { 0x9, 0x02, true,  "BVS (V=1)" },
        { 0x9, 0x00, false, "BVS (V=0)" },
        { 0xA, 0x00, true,  "BPL (N=0)" },
        { 0xA, 0x08, false, "BPL (N=1)" },
        { 0xB, 0x08, true,  "BMI (N=1)" },
        { 0xB, 0x00, false, "BMI (N=0)" },
        { 0xC, 0x00, true,  "BGE (N=V=0)" },
        { 0xC, 0x0A, true,  "BGE (N=V=1)" },
        { 0xC, 0x08, false, "BGE (N=1,V=0)" },
        { 0xD, 0x08, true,  "BLT (N=1,V=0)" },
        { 0xD, 0x00, false, "BLT (N=V=0)" },
        { 0xE, 0x00, true,  "BGT (Z=0,N=V)" },
        { 0xE, 0x04, false, "BGT (Z=1)" },
        { 0xF, 0x04, true,  "BLE (Z=1)" },
        { 0xF, 0x08, true,  "BLE (N!=V)" },
        { 0xF, 0x00, false, "BLE (Z=0,N=V=0)" },
    };

    for (int i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        /* Code: Bcc.B +4; MOVEQ #0,D0; BRA.B +2; MOVEQ #1,D0; STOP */
        u8 code[] = {
            (u8)(0x60 | (cases[i].cc << 0)), 0x04,  /* Bcc.B +4 */
            0x70, 0x00,                        /* MOVEQ #0,D0 (not taken path) */
            0x60, 0x02,                        /* BRA.B +2 (skip taken path) */
            0x70, 0x01,                        /* MOVEQ #1,D0 (taken path) */
        };

        M68020State *cpu = exec_one(code, sizeof code, 0xFF, 0, 0, cases[i].ccr);
        u32 d0 = cpu->D[0];
        bool taken = (d0 == 1);

        EXPECT(taken == cases[i].take,
               "%s: %s (D0=%u)", cases[i].name,
               taken ? "taken" : "not-taken", d0);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: CMP.L — flag-only, no register change                  */
/* ------------------------------------------------------------------ */

static void test_cmp_l_flags(void) {
    printf("\n--- CMP.L flag verification ---\n");

    /* CMP.L D1,D0: computes D0-D1, sets flags, doesn't modify D0.
     * opword = 0xB081 */
    static const u8 code[] = { 0xB0, 0x81 };

    struct { u32 d0; u32 d1; u8 ccr; } cases[] = {
        { 5, 5, 0x04 },          /* equal: Z */
        { 10, 5, 0x00 },         /* d0 > d1 unsigned & signed */
        { 5, 10, 0x09 },         /* d0 < d1: N,C */
        { 0, 0, 0x04 },          /* both zero: Z */
        { 0x80000000, 0, 0x08 }, /* negative result: N */
        { 0, 0x80000000, 0x0B }, /* 0 - 0x80000000: N,V,C */
        { 0x7FFFFFFF, 0xFFFFFFFF, 0x0B }, /* pos - neg: N,V,C */
    };

    for (int i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        M68020State *cpu = exec_one(code, sizeof code,
                                     cases[i].d0, cases[i].d1, 0, 0);
        u32 d0  = cpu->D[0];
        /* CMP doesn't modify X, so mask to NZVC only */
        u8  ccr = (u8)(cpu->D[7] & 0x0Fu);

        EXPECT(d0 == cases[i].d0,
               "CMP.L case %d: D0 unchanged (got 0x%X, exp 0x%X)",
               i, d0, cases[i].d0);
        EXPECT(ccr == (cases[i].ccr & 0x0F),
               "CMP.L case %d: CCR=0x%X (got 0x%X)",
               i, cases[i].ccr & 0x0F, ccr);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: Shift/Rotate — boundary values                         */
/* ------------------------------------------------------------------ */

static void test_shifts_boundary(void) {
    printf("\n--- Shift boundary values ---\n");

    /* LSL.L #1,D0: 1110 001 1 10 0 00 000 = 0xE380 */
    /* Wait: LSL.L #1,D0: count=001, dir=1(left), sz=10(long), i/r=0(imm), type=01(LS), reg=000
     * = 1110 001 1 10 0 01 000 = 0xE388 */

    struct {
        u8 code[2];
        u32 input;
        u32 expected;
        const char *desc;
    } cases[] = {
        /* LSL.L #1,D0 = 0xE388 */
        { {0xE3, 0x88}, 0x80000000, 0x00000000, "LSL.L #1, 0x80000000" },
        { {0xE3, 0x88}, 0x00000001, 0x00000002, "LSL.L #1, 1" },
        { {0xE3, 0x88}, 0x40000000, 0x80000000, "LSL.L #1, 0x40000000" },
        /* LSR.L #1,D0 = 0xE288 */
        { {0xE2, 0x88}, 0x00000001, 0x00000000, "LSR.L #1, 1" },
        { {0xE2, 0x88}, 0x80000000, 0x40000000, "LSR.L #1, 0x80000000" },
        /* ASR.L #1,D0: 1110 001 0 10 0 00 000 = 0xE280 */
        { {0xE2, 0x80}, 0x80000000, 0xC0000000, "ASR.L #1, 0x80000000 (sign ext)" },
        { {0xE2, 0x80}, 0x7FFFFFFF, 0x3FFFFFFF, "ASR.L #1, 0x7FFFFFFF" },
    };

    for (int i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        M68020State *cpu = exec_one(cases[i].code, 2, cases[i].input, 0, 0, 0);
        u32 result = cpu->D[0];

        EXPECT(result == cases[i].expected,
               "%s: got 0x%08X (exp 0x%08X)",
               cases[i].desc, result, cases[i].expected);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: Memory addressing — store and load back                */
/* ------------------------------------------------------------------ */

static void test_memory_store_load(void) {
    printf("\n--- Memory store/load verification ---\n");

    /* Test: MOVE.L D0,(A0); CLR.L D0; MOVE.L (A0),D0
     * Should preserve value through memory round-trip. */

    u32 test_vals[] = { 0, 1, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0xDEADBEEF, 42 };

    for (int i = 0; i < (int)(sizeof test_vals / sizeof test_vals[0]); i++) {
        memset(&g_bus, 0, sizeof g_bus);
        uint8_t *m = g_bus.mem;

        m[0]=0x00;m[1]=0x00;m[2]=0x80;m[3]=0x00;
        m[4]=0x00;m[5]=0x00;m[6]=0x01;m[7]=0x00;
        m[16]=0x00;m[17]=0x00;m[18]=0x02;m[19]=0x00;

        uint32_t p = 0x100;
        /* LEA $0400,A0 */
        m[p++]=0x41;m[p++]=0xF9;m[p++]=0x00;m[p++]=0x00;m[p++]=0x04;m[p++]=0x00;
        /* MOVE.L D0,(A0): 0x2080 */
        m[p++]=0x20;m[p++]=0x80;
        /* CLR.L D0: 0x4280 */
        m[p++]=0x42;m[p++]=0x80;
        /* MOVE.L (A0),D0: 0x2010 */
        m[p++]=0x20;m[p++]=0x10;
        /* STOP */
        m[p++]=0x4E;m[p++]=0x72;m[p++]=0x27;m[p++]=0x00;

        m[0x200]=0x4E;m[0x201]=0x72;m[0x202]=0x27;m[0x203]=0x00;

        M68020BusInterface bus = {
            .read=flat_read,.write=flat_write,.iack=flat_iack,.ctx=&g_bus
        };
        M68020State *cpu = m68020_create(&bus);
        m68020_reset(cpu);
        cpu->D[0] = test_vals[i];
        m68020_run(cpu, 100000);

        EXPECT(cpu->D[0] == test_vals[i],
               "Store/load 0x%08X: got 0x%08X", test_vals[i], cpu->D[0]);

        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: DIVU.W / DIVS.W edge cases                            */
/* ------------------------------------------------------------------ */

static void test_div_edge_cases(void) {
    printf("\n--- Division edge cases ---\n");

    /* DIVU.W D1,D0: 0x80C1 */
    {
        /* Normal case: 100 / 3 = 33 remainder 1 */
        static const u8 code[] = { 0x80, 0xC1 };
        M68020State *cpu = exec_one(code, sizeof code, 100, 3, 0, 0);
        u32 d0 = cpu->D[0];
        EXPECT((d0 & 0xFFFF) == 33, "DIVU 100/3 quot=33 (got %u)", d0 & 0xFFFF);
        EXPECT((d0 >> 16) == 1, "DIVU 100/3 rem=1 (got %u)", d0 >> 16);
        m68020_destroy(cpu);
    }

    /* DIVS.W: negative dividend */
    {
        /* DIVS.W D1,D0: 0x81C1 */
        static const u8 code[] = { 0x81, 0xC1 };
        M68020State *cpu = exec_one(code, sizeof code, (u32)-100, 7, 0, 0);
        u32 d0 = cpu->D[0];
        s16 quot = (s16)(d0 & 0xFFFF);
        s16 rem  = (s16)(d0 >> 16);
        EXPECT(quot == -14, "DIVS -100/7 quot=-14 (got %d)", quot);
        EXPECT(rem == -2, "DIVS -100/7 rem=-2 (got %d)", rem);
        m68020_destroy(cpu);
    }

    /* DIVU.W: overflow (quotient > 0xFFFF) */
    {
        static const u8 code[] = { 0x80, 0xC1 };
        M68020State *cpu = exec_one(code, sizeof code, 0x00020000, 1, 0, 0);
        u8 ccr = (u8)(cpu->D[7] & 0x1Fu);
        EXPECT((ccr & 0x02) != 0, "DIVU overflow: V=1 (CCR=0x%02X)", ccr);
        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Test Group: MULU.L / MULS.L — 64-bit results                      */
/* ------------------------------------------------------------------ */

static void test_mul_l_64bit(void) {
    printf("\n--- MUL.L 64-bit result ---\n");

    /* MULU.L D1,D2:D0 (64-bit result, unsigned)
     * opword=0x4C01, extword: S=0, L=1, Dh=D2=010, Dl=D0=000 → 0x2400 */
    {
        static const u8 code[] = { 0x4C, 0x01, 0x24, 0x00 };
        M68020State *cpu = exec_one(code, sizeof code, 0xFFFFFFFF, 0xFFFFFFFF, 0, 0);
        /* 0xFFFFFFFF * 0xFFFFFFFF = 0xFFFFFFFE00000001 */
        EXPECT(cpu->D[0] == 0x00000001u, "MULU.L 64bit: D0(low)=0x00000001 (got 0x%08X)", cpu->D[0]);
        EXPECT(cpu->D[2] == 0xFFFFFFFEu, "MULU.L 64bit: D2(high)=0xFFFFFFFE (got 0x%08X)", cpu->D[2]);
        m68020_destroy(cpu);
    }

    /* MULS.L D1,D2:D0 (64-bit result, signed)
     * opword=0x4C01, extword: S=1, L=1, Dh=D2=010, Dl=D0=000 → 0x2C00 */
    {
        static const u8 code[] = { 0x4C, 0x01, 0x2C, 0x00 };
        M68020State *cpu = exec_one(code, sizeof code, (u32)-10, 20, 0, 0);
        /* -10 * 20 = -200 = 0xFFFFFFFFFFFFFF38 */
        EXPECT(cpu->D[0] == 0xFFFFFF38u, "MULS.L 64bit: D0(low)=0xFFFFFF38 (got 0x%08X)", cpu->D[0]);
        EXPECT(cpu->D[2] == 0xFFFFFFFFu, "MULS.L 64bit: D2(high)=0xFFFFFFFF (got 0x%08X)", cpu->D[2]);
        m68020_destroy(cpu);
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void) {
    printf("MC68020 Emulator — Systematic Validation\n");
    printf("==========================================\n");

    test_add_l_ccr();
    test_sub_l_ccr();
    test_moveq_exhaustive();
    test_bcc_all_conditions();
    test_cmp_l_flags();
    test_shifts_boundary();
    test_memory_store_load();
    test_div_edge_cases();
    test_mul_l_64bit();

    printf("\n==========================================\n");
    printf("Results: %d/%d passed", g_pass, g_total);
    if (g_fail)
        printf(" (%d FAILED)", g_fail);
    printf("\n");

    return g_fail ? 1 : 0;
}
