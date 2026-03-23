/*
 * ab_sequence.c — Multi-instruction sequence A/B comparison.
 *
 * Runs small programs (10-100 instructions) on both our emulator and
 * Musashi, then compares the final CPU state and memory.
 *
 * Test programs:
 *   1. Fibonacci sequence
 *   2. Function call chain (BSR/RTS + LINK/UNLK)
 *   3. Memory copy loop
 *   4. Bubble sort (small array)
 *   5. MOVEM save/restore cycle
 *   6. Conditional branching maze
 *   7. Stack frame manipulation
 *   8. Randomized instruction sequences
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "m68020.h"
#include "m68020_internal.h"
#include "m68k.h"

/* ------------------------------------------------------------------ */
/* Shared memory                                                       */
/* ------------------------------------------------------------------ */

#define MEM_SIZE (16 * 1024 * 1024)
static uint8_t g_mem[MEM_SIZE];

/* Musashi bus */
unsigned int m68k_read_memory_8(unsigned int a)  { return g_mem[a&0xFFFFFF]; }
unsigned int m68k_read_memory_16(unsigned int a) { a&=0xFFFFFF; return (g_mem[a]<<8)|g_mem[a+1]; }
unsigned int m68k_read_memory_32(unsigned int a) { a&=0xFFFFFF; return (g_mem[a]<<24)|(g_mem[a+1]<<16)|(g_mem[a+2]<<8)|g_mem[a+3]; }
void m68k_write_memory_8(unsigned int a, unsigned int v)  { g_mem[a&0xFFFFFF]=v; }
void m68k_write_memory_16(unsigned int a, unsigned int v) { a&=0xFFFFFF; g_mem[a]=v>>8; g_mem[a+1]=v; }
void m68k_write_memory_32(unsigned int a, unsigned int v) { a&=0xFFFFFF; g_mem[a]=v>>24; g_mem[a+1]=v>>16; g_mem[a+2]=v>>8; g_mem[a+3]=v; }
unsigned int m68k_read_disassembler_16(unsigned int a) { return m68k_read_memory_16(a); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return m68k_read_memory_32(a); }

/* Our bus */
static BusResult our_read(void *c, u32 a, BusSize s, FunctionCode f, u32 *v, u32 *cy) {
    (void)c;(void)f; *cy=0; a&=0xFFFFFF;
    switch(s) {
        case SIZE_BYTE: *v=g_mem[a]; break;
        case SIZE_WORD: *v=(g_mem[a]<<8)|g_mem[a+1]; break;
        case SIZE_LONG: *v=(g_mem[a]<<24)|(g_mem[a+1]<<16)|(g_mem[a+2]<<8)|g_mem[a+3]; break;
    } return BUS_OK;
}
static BusResult our_write(void *c, u32 a, BusSize s, FunctionCode f, u32 v, u32 *cy) {
    (void)c;(void)f; *cy=0; a&=0xFFFFFF;
    switch(s) {
        case SIZE_BYTE: g_mem[a]=v; break;
        case SIZE_WORD: g_mem[a]=v>>8; g_mem[a+1]=v; break;
        case SIZE_LONG: g_mem[a]=v>>24; g_mem[a+1]=v>>16; g_mem[a+2]=v>>8; g_mem[a+3]=v; break;
    } return BUS_OK;
}
static u8 our_iack(void *c, u8 l) { (void)c;(void)l; return 0xFF; }

/* ------------------------------------------------------------------ */
/* Helper: write big-endian values to memory                           */
/* ------------------------------------------------------------------ */

static void w16(uint32_t addr, uint16_t v) { g_mem[addr]=v>>8; g_mem[addr+1]=v; }
static void w32(uint32_t addr, uint32_t v) { g_mem[addr]=v>>24; g_mem[addr+1]=v>>16; g_mem[addr+2]=v>>8; g_mem[addr+3]=v; }

/* ------------------------------------------------------------------ */
/* Test framework                                                      */
/* ------------------------------------------------------------------ */

static int g_pass = 0, g_fail = 0;

typedef struct {
    const char *name;
    void (*setup)(void);     /* Load program into g_mem */
    int  max_steps;          /* Max instructions to execute */
} SeqTest;

/*
 * Run one sequence test: execute on both emulators, compare.
 * Returns 1 on match, 0 on mismatch.
 */
static int run_sequence(SeqTest *test) {
    /* Setup memory */
    memset(g_mem, 0, MEM_SIZE);
    /* Vectors: SSP=0x8000, PC=0x1000 */
    w32(0x000000, 0x00008000);
    w32(0x000004, 0x00001000);
    /* Vector 4 (illegal) → STOP at 0x0F00 */
    w32(0x000010, 0x00000F00);
    w16(0x000F00, 0x4E72); w16(0x000F02, 0x2700); /* STOP #$2700 */

    test->setup();

    /* Save initial memory */
    static uint8_t mem_init[MEM_SIZE];
    memcpy(mem_init, g_mem, MEM_SIZE);

    /* ---- Run Musashi ---- */
    m68k_set_cpu_type(M68K_CPU_TYPE_68020);
    m68k_pulse_reset();
    /* Clear all registers (reset doesn't zero D0-D7, A0-A6) */
    for (int i = 0; i < 8; i++) m68k_set_reg(M68K_REG_D0+i, 0);
    for (int i = 0; i < 7; i++) m68k_set_reg(M68K_REG_A0+i, 0);

    for (int i = 0; i < test->max_steps; i++) {
        m68k_execute(1);
        /* Check if stopped or halted */
        unsigned int sr = m68k_get_reg(NULL, M68K_REG_SR);
        (void)sr;
        /* Musashi doesn't expose stopped/halted easily;
         * we detect it by PC not changing */
    }

    /* Save Musashi state */
    uint32_t m_d[8], m_a[8], m_pc;
    uint16_t m_sr;
    for (int i = 0; i < 8; i++) m_d[i] = m68k_get_reg(NULL, M68K_REG_D0+i);
    for (int i = 0; i < 8; i++) m_a[i] = m68k_get_reg(NULL, M68K_REG_A0+i);
    m_pc = m68k_get_reg(NULL, M68K_REG_PC);
    m_sr = (uint16_t)m68k_get_reg(NULL, M68K_REG_SR);
    static uint8_t musashi_mem[MEM_SIZE];
    memcpy(musashi_mem, g_mem, MEM_SIZE);

    /* ---- Run our emulator ---- */
    memcpy(g_mem, mem_init, MEM_SIZE);

    M68020BusInterface bus = { .read=our_read, .write=our_write, .iack=our_iack, .ctx=NULL };
    M68020State *cpu = m68020_create(&bus);
    m68020_reset(cpu);

    for (int i = 0; i < test->max_steps; i++) {
        if (cpu->halted || cpu->stopped) break;
        m68020_step(cpu);
    }

    /* ---- Compare ---- */
    int mismatch = 0;
    char errbuf[2048] = "";
    int ep = 0;

    for (int i = 0; i < 8; i++) {
        if (cpu->D[i] != m_d[i]) {
            ep += snprintf(errbuf+ep, sizeof(errbuf)-ep, "D%d:0x%X!=0x%X ", i, cpu->D[i], m_d[i]);
            mismatch++;
        }
    }
    for (int i = 0; i < 7; i++) {
        if (cpu->A[i] != m_a[i]) {
            ep += snprintf(errbuf+ep, sizeof(errbuf)-ep, "A%d:0x%X!=0x%X ", i, cpu->A[i], m_a[i]);
            mismatch++;
        }
    }
    if ((cpu->SR & 0x1F) != (m_sr & 0x1F)) {
        ep += snprintf(errbuf+ep, sizeof(errbuf)-ep, "SR:0x%04X!=0x%04X ", cpu->SR, m_sr);
        mismatch++;
    }

    /* Compare data area (0x2000-0x3000) */
    for (uint32_t j = 0x2000; j < 0x3000; j++) {
        if (g_mem[j] != musashi_mem[j]) {
            ep += snprintf(errbuf+ep, sizeof(errbuf)-ep, "MEM[%X]:0x%02X!=0x%02X ", j, g_mem[j], musashi_mem[j]);
            mismatch++;
            if (mismatch > 8) break;
        }
    }

    m68020_destroy(cpu);

    if (mismatch) {
        printf("  FAIL: %s — %s\n", test->name, errbuf);
        g_fail++;
        return 0;
    }
    printf("  PASS: %s (D0=0x%X)\n", test->name, m_d[0]);
    g_pass++;
    return 1;
}

/* ================================================================== */
/* Test Programs                                                       */
/* ================================================================== */

/* --- Test 1: Fibonacci(10) --- */
static void setup_fibonacci(void) {
    /*
     *   MOVEQ #1, D0       ; fib(0) = 1
     *   MOVEQ #1, D1       ; fib(1) = 1
     *   MOVEQ #8, D7       ; counter (compute fib(2)..fib(9))
     * loop:
     *   MOVE.L D1, D2      ; tmp = D1
     *   ADD.L  D0, D1      ; D1 = D0 + D1
     *   MOVE.L D2, D0      ; D0 = tmp
     *   SUBQ.L #1, D7
     *   BNE.B  loop
     *   ILLEGAL
     */
    uint32_t p = 0x1000;
    w16(p, 0x7001); p+=2;  /* MOVEQ #1,D0 */
    w16(p, 0x7201); p+=2;  /* MOVEQ #1,D1 */
    w16(p, 0x7E08); p+=2;  /* MOVEQ #8,D7 */
    /* loop: */
    w16(p, 0x2401); p+=2;  /* MOVE.L D1,D2 */
    w16(p, 0xD280); p+=2;  /* ADD.L D0,D1 */
    w16(p, 0x2002); p+=2;  /* MOVE.L D2,D0 */
    w16(p, 0x5387); p+=2;  /* SUBQ.L #1,D7 */
    w16(p, 0x66F6); p+=2;  /* BNE.B -10 */
    w16(p, 0x4AFC); p+=2;  /* ILLEGAL */
}

/* --- Test 2: Function call chain --- */
static void setup_call_chain(void) {
    /*
     * main:
     *   MOVEQ #5, D0
     *   BSR   func_double     ; D0 = D0 * 2 = 10
     *   BSR   func_double     ; D0 = 10 * 2 = 20
     *   BSR   func_add10      ; D0 = 20 + 10 = 30
     *   ILLEGAL
     *
     * func_double:
     *   ADD.L D0, D0
     *   RTS
     *
     * func_add10:
     *   LINK A6, #0
     *   ADDI.L #10, D0        ; 0x0680 0x0000000A
     *   UNLK A6
     *   RTS
     */
    uint32_t p = 0x1000;
    w16(p, 0x7005); p+=2;  /* MOVEQ #5,D0 */
    w16(p, 0x6108); p+=2;  /* BSR.B +8 → 0x100C */
    w16(p, 0x6106); p+=2;  /* BSR.B +6 → 0x100C */
    w16(p, 0x610A); p+=2;  /* BSR.B +10 → 0x1014 */
    w16(p, 0x4AFC); p+=2;  /* ILLEGAL */
    /* padding */
    w16(p, 0x4E71); p+=2;  /* NOP */
    /* func_double at 0x100C: */
    w16(p, 0xD080); p+=2;  /* ADD.L D0,D0 */
    w16(p, 0x4E75); p+=2;  /* RTS */
    /* padding */
    w16(p, 0x4E71); p+=2;  /* NOP */
    /* func_add10 at 0x1014: */
    w16(p, 0x4E56); w16(p+2, 0x0000); p+=4;  /* LINK A6,#0 */
    w16(p, 0x0680); w32(p+2, 0x0000000A); p+=6; /* ADDI.L #10,D0 */
    w16(p, 0x4E5E); p+=2;  /* UNLK A6 */
    w16(p, 0x4E75); p+=2;  /* RTS */
}

/* --- Test 3: Memory copy (8 longs) --- */
static void setup_memcopy(void) {
    /*
     *   LEA $2000,A0         ; source
     *   LEA $2100,A1         ; dest
     *   MOVEQ #7, D7         ; counter (8 longs)
     * loop:
     *   MOVE.L (A0)+,(A1)+
     *   DBF D7, loop
     *   ILLEGAL
     */
    /* Source data at 0x2000 */
    for (int i = 0; i < 8; i++)
        w32(0x2000 + i*4, 0xDEAD0000 + i);

    uint32_t p = 0x1000;
    w16(p, 0x41F9); w32(p+2, 0x00002000); p+=6; /* LEA $2000,A0 */
    w16(p, 0x43F9); w32(p+2, 0x00002100); p+=6; /* LEA $2100,A1 */
    w16(p, 0x7E07); p+=2;  /* MOVEQ #7,D7 */
    /* loop: */
    w16(p, 0x22D8); p+=2;  /* MOVE.L (A0)+,(A1)+ */
    w16(p, 0x51CF); w16(p+2, 0xFFFC); p+=4; /* DBF D7,loop */
    w16(p, 0x4AFC); p+=2;  /* ILLEGAL */
}

/* --- Test 4: Bubble sort (6 words) --- */
static void setup_bubblesort(void) {
    /* Data at 0x2000: 6 words descending */
    uint16_t data[] = { 6, 5, 4, 3, 2, 1 };
    for (int i = 0; i < 6; i++)
        w16(0x2000 + i*2, data[i]);

    /*
     *   LEA $2000,A0
     *   MOVEQ #4,D7          ; outer = n-2
     * outer:
     *   MOVE.L A0,A1
     *   MOVE.L D7,D6
     * inner:
     *   MOVE.W (A1),D0
     *   CMP.W  (2,A1),D0
     *   BLE.B  noswap
     *   MOVE.W (2,A1),D1
     *   MOVE.W D1,(A1)
     *   MOVE.W D0,(2,A1)
     * noswap:
     *   ADDQ.L #2,A1
     *   DBF    D6,inner
     *   DBF    D7,outer
     *   ILLEGAL
     */
    uint32_t p = 0x1000;
    w16(p, 0x41F9); w32(p+2, 0x00002000); p+=6; /* LEA $2000,A0 */
    w16(p, 0x7E04); p+=2;                        /* MOVEQ #4,D7 */
    /* outer: */
    uint32_t outer = p;
    w16(p, 0x2248); p+=2;                        /* MOVEA.L A0,A1 */
    w16(p, 0x2C07); p+=2;                        /* MOVE.L D7,D6 */
    /* inner: */
    uint32_t inner = p;
    w16(p, 0x3011); p+=2;                        /* MOVE.W (A1),D0 */
    w16(p, 0x3229); w16(p+2, 0x0002); p+=4;      /* MOVE.W (2,A1),D1 */
    w16(p, 0xB041); p+=2;                        /* CMP.W D1,D0 */
    w16(p, 0x6F06); p+=2;                        /* BLE.B +6 (skip swap) */
    w16(p, 0x3281); p+=2;                        /* MOVE.W D1,(A1) */
    w16(p, 0x3340); w16(p+2, 0x0002); p+=4;      /* MOVE.W D0,(2,A1) */
    /* noswap: */
    w16(p, 0x5489); p+=2;                        /* ADDQ.L #2,A1 */
    /* DBF D6,inner */
    int16_t id = (int16_t)(inner - (p + 2));
    w16(p, 0x51CE); w16(p+2, (uint16_t)id); p+=4;
    /* DBF D7,outer */
    int16_t od = (int16_t)(outer - (p + 2));
    w16(p, 0x51CF); w16(p+2, (uint16_t)od); p+=4;
    w16(p, 0x4AFC); p+=2;                        /* ILLEGAL */
}

/* --- Test 5: MOVEM save/restore --- */
static void setup_movem_cycle(void) {
    /*
     *   MOVEQ #1,D0; MOVEQ #2,D1; MOVEQ #3,D2; MOVEQ #4,D3
     *   MOVEQ #5,D4; MOVEQ #6,D5; MOVEQ #7,D6
     *   MOVEM.L D0-D6,-(A7)     ; save 7 registers
     *   MOVEQ #0,D0 through D6  ; clear them
     *   MOVEM.L (A7)+,D0-D6     ; restore
     *   ILLEGAL
     */
    uint32_t p = 0x1000;
    for (int i = 0; i < 7; i++) {
        w16(p, 0x7000 | (i << 9) | (i + 1)); p += 2; /* MOVEQ #(i+1),Di */
    }
    /* MOVEM.L D0-D6,-(A7): opword=0x48E7, mask for predec: bit15=D0..bit9=D6 = 0xFE00 */
    w16(p, 0x48E7); w16(p+2, 0xFE00); p+=4;
    /* Clear D0-D6 */
    for (int i = 0; i < 7; i++) {
        w16(p, 0x7000 | (i << 9)); p += 2; /* MOVEQ #0,Di */
    }
    /* MOVEM.L (A7)+,D0-D6: opword=0x4CDF, mask=0x007F (D0-D6) */
    w16(p, 0x4CDF); w16(p+2, 0x007F); p+=4;
    w16(p, 0x4AFC); p+=2;
}

/* --- Test 6: Conditional maze --- */
static void setup_cond_maze(void) {
    /*
     *   MOVEQ #42, D0
     *   CMP.L  #42, D0
     *   BNE.B  bad
     *   ADDQ.L #1, D0       ; D0=43
     *   CMP.L  #50, D0
     *   BGT.B  bad
     *   SUBQ.L #3, D0       ; D0=40
     *   TST.L  D0
     *   BEQ.B  bad
     *   MOVEQ  #100, D1     ; success marker
     *   ILLEGAL
     * bad:
     *   MOVEQ  #-1, D1      ; failure marker
     *   ILLEGAL
     */
    uint32_t p = 0x1000;
    w16(p, 0x702A); p+=2;            /* MOVEQ #42,D0 */
    /* CMPI.L #42,D0: 0x0C80 + long imm */
    w16(p, 0x0C80); w32(p+2, 42); p+=6;
    w16(p, 0x6610); p+=2;            /* BNE.B +16 → bad */
    w16(p, 0x5280); p+=2;            /* ADDQ.L #1,D0 */
    w16(p, 0x0C80); w32(p+2, 50); p+=6; /* CMPI.L #50,D0 */
    w16(p, 0x6E06); p+=2;            /* BGT.B +6 → bad */
    w16(p, 0x5780); p+=2;            /* SUBQ.L #3,D0 */
    w16(p, 0x4A80); p+=2;            /* TST.L D0 */
    w16(p, 0x6704); p+=2;            /* BEQ.B +4 → bad */
    w16(p, 0x7264); p+=2;            /* MOVEQ #100,D1 */
    w16(p, 0x4AFC); p+=2;            /* ILLEGAL */
    /* bad: */
    w16(p, 0x72FF); p+=2;            /* MOVEQ #-1,D1 */
    w16(p, 0x4AFC); p+=2;            /* ILLEGAL */
}

/* --- Test 7: Stack frame manipulation --- */
static void setup_stack_frames(void) {
    /*
     * Nested function calls with LINK/UNLK:
     *   BSR func_a
     *   ILLEGAL
     *
     * func_a:
     *   LINK A6,#-8
     *   MOVE.L #0xDEAD,(A6)        ; write to frame
     *   BSR func_b
     *   UNLK A6
     *   RTS
     *
     * func_b:
     *   LINK A5,#-4
     *   MOVE.L #0xBEEF,(A5)
     *   UNLK A5
     *   RTS
     */
    uint32_t p = 0x1000;
    w16(p, 0x6106); p+=2;  /* BSR.B +6 → func_a at 0x1008 */
    w16(p, 0x4AFC); p+=2;  /* ILLEGAL */
    /* padding */
    w16(p, 0x4E71); p+=2; w16(p, 0x4E71); p+=2;

    /* func_a at 0x1008: */
    w16(p, 0x4E56); w16(p+2, 0xFFF8); p+=4;  /* LINK A6,#-8 */
    w16(p, 0x2CBC); w32(p+2, 0x0000DEAD); p+=6; /* MOVE.L #$DEAD,(A6)... wait.
     * MOVE.L #imm,(A6): dst=(A6)=010 110, src=#imm=111 100
     * 0010 110 010 111 100 = 0x2CBC */
    w16(p, 0x6108); p+=2;  /* BSR.B +8 → func_b */
    w16(p, 0x4E5E); p+=2;  /* UNLK A6 */
    w16(p, 0x4E75); p+=2;  /* RTS */
    /* padding */
    w16(p, 0x4E71); p+=2;

    /* func_b: */
    w16(p, 0x4E55); w16(p+2, 0xFFFC); p+=4;  /* LINK A5,#-4 */
    w16(p, 0x2ABC); w32(p+2, 0x0000BEEF); p+=6; /* MOVE.L #$BEEF,(A5) → 0x2ABC */
    w16(p, 0x4E5D); p+=2;  /* UNLK A5 */
    w16(p, 0x4E75); p+=2;  /* RTS */
}

/* --- Test 8: Arithmetic chain with carry --- */
static void setup_carry_chain(void) {
    /*
     *   MOVE.L #$FFFFFFFF,D0
     *   MOVE.L #$FFFFFFFF,D1
     *   MOVEQ  #0,D2
     *   ADD.L  D0,D1         ; D1=FFFFFFFE, X=1
     *   ADDX.L D2,D2         ; D2=0+0+X=1
     *   MOVE.L #$80000000,D3
     *   MOVE.L #$80000000,D4
     *   ADD.L  D3,D4         ; D4=0, X=1, V=1
     *   ADDX.L D2,D5         ; D5=0+1+1=2
     *   ILLEGAL
     */
    uint32_t p = 0x1000;
    w16(p, 0x203C); w32(p+2, 0xFFFFFFFF); p+=6; /* MOVE.L #-1,D0 */
    w16(p, 0x223C); w32(p+2, 0xFFFFFFFF); p+=6; /* MOVE.L #-1,D1 */
    w16(p, 0x7400); p+=2;                        /* MOVEQ #0,D2 */
    w16(p, 0xD280); p+=2;                        /* ADD.L D0,D1 */
    w16(p, 0xD582); p+=2;                        /* ADDX.L D2,D2 */
    w16(p, 0x263C); w32(p+2, 0x80000000); p+=6;  /* MOVE.L #$80000000,D3 */
    w16(p, 0x283C); w32(p+2, 0x80000000); p+=6;  /* MOVE.L #$80000000,D4 */
    w16(p, 0xD883); p+=2;                        /* ADD.L D3,D4 */
    w16(p, 0xDB82); p+=2;                        /* ADDX.L D2,D5 */
    w16(p, 0x4AFC); p+=2;                        /* ILLEGAL */
}

/* ------------------------------------------------------------------ */
/* Test table                                                          */
/* ------------------------------------------------------------------ */

static SeqTest tests[] = {
    { "Fibonacci(10)",       setup_fibonacci,    100 },
    { "Function call chain", setup_call_chain,   100 },
    { "Memory copy 8 longs", setup_memcopy,      100 },
    { "Bubble sort 6 words", setup_bubblesort,   500 },
    { "MOVEM save/restore",  setup_movem_cycle,  100 },
    { "Conditional maze",    setup_cond_maze,    100 },
    { "Stack frame nesting", setup_stack_frames, 100 },
    { "Carry chain ADDX",   setup_carry_chain,   100 },
};
#define NUM_TESTS (sizeof tests / sizeof tests[0])

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void) {
    printf("MC68020 — Multi-Instruction Sequence A/B Comparison\n");
    printf("====================================================\n\n");

    m68k_init();

    for (int i = 0; i < (int)NUM_TESTS; i++) {
        run_sequence(&tests[i]);
    }

    printf("\n====================================================\n");
    printf("Results: %d/%d passed", g_pass, g_pass + g_fail);
    if (g_fail) printf(" (%d FAILED)", g_fail);
    printf("\n");

    return g_fail ? 1 : 0;
}
