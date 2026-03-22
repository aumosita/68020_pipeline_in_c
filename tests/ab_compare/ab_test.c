/*
 * ab_test.c — A/B comparison: our emulator vs Musashi (68020 mode).
 *
 * For each test:
 *   1. Generate random CPU state + random valid opcode
 *   2. Set both emulators to the same state
 *   3. Execute one instruction on each
 *   4. Compare resulting state (registers, SR, PC, memory)
 *
 * Compile with both our libm68020.a and Musashi's object files.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

/* ---- Our emulator ---- */
#include "m68020.h"
#include "m68020_internal.h"

/* ---- Musashi (prefix all symbols to avoid collision) ---- */
/* Musashi uses global state, so we include it directly */
#include "m68k.h"

/* ------------------------------------------------------------------ */
/* Shared memory for both emulators                                    */
/* ------------------------------------------------------------------ */

#define MEM_SIZE (16 * 1024 * 1024)
static uint8_t g_mem[MEM_SIZE];

/* ---- Musashi bus callbacks ---- */
unsigned int m68k_read_memory_8(unsigned int addr)  { return g_mem[addr & 0xFFFFFF]; }
unsigned int m68k_read_memory_16(unsigned int addr) {
    addr &= 0xFFFFFF;
    return ((unsigned)g_mem[addr]<<8) | g_mem[addr+1];
}
unsigned int m68k_read_memory_32(unsigned int addr) {
    addr &= 0xFFFFFF;
    return ((unsigned)g_mem[addr]<<24)|((unsigned)g_mem[addr+1]<<16)
          |((unsigned)g_mem[addr+2]<<8)|g_mem[addr+3];
}
void m68k_write_memory_8(unsigned int addr, unsigned int val)  { g_mem[addr & 0xFFFFFF] = val; }
void m68k_write_memory_16(unsigned int addr, unsigned int val) {
    addr &= 0xFFFFFF;
    g_mem[addr] = val>>8; g_mem[addr+1] = val;
}
void m68k_write_memory_32(unsigned int addr, unsigned int val) {
    addr &= 0xFFFFFF;
    g_mem[addr]=val>>24; g_mem[addr+1]=val>>16; g_mem[addr+2]=val>>8; g_mem[addr+3]=val;
}

/* ---- Our emulator bus callbacks ---- */
static BusResult our_read(void *ctx, u32 addr, BusSize sz,
                           FunctionCode fc, u32 *val, u32 *cyc) {
    (void)ctx; (void)fc; *cyc = 0;
    addr &= 0xFFFFFF;
    switch (sz) {
        case SIZE_BYTE: *val = g_mem[addr]; break;
        case SIZE_WORD: *val = ((u32)g_mem[addr]<<8)|g_mem[addr+1]; break;
        case SIZE_LONG: *val = ((u32)g_mem[addr]<<24)|((u32)g_mem[addr+1]<<16)
                              |((u32)g_mem[addr+2]<<8)|g_mem[addr+3]; break;
    }
    return BUS_OK;
}
static BusResult our_write(void *ctx, u32 addr, BusSize sz,
                            FunctionCode fc, u32 val, u32 *cyc) {
    (void)ctx; (void)fc; *cyc = 0;
    addr &= 0xFFFFFF;
    switch (sz) {
        case SIZE_BYTE: g_mem[addr]=val; break;
        case SIZE_WORD: g_mem[addr]=val>>8; g_mem[addr+1]=val; break;
        case SIZE_LONG: g_mem[addr]=val>>24; g_mem[addr+1]=val>>16;
                        g_mem[addr+2]=val>>8; g_mem[addr+3]=val; break;
    }
    return BUS_OK;
}
static u8 our_iack(void *ctx, u8 lvl) { (void)ctx; (void)lvl; return 0xFF; }

/* ------------------------------------------------------------------ */
/* Random state generation                                             */
/* ------------------------------------------------------------------ */

static uint32_t xrand(void) {
    return ((uint32_t)rand() << 16) ^ (uint32_t)rand();
}

/* Common register-only opcodes that are safe to test on 68020 */
static uint16_t safe_opcodes[] = {
    /* MOVEQ #imm, Dn: 0x70xx-0x7Exx (bit 8=0) */
    0x7000, 0x7001, 0x707F, 0x70FF, 0x7200, 0x7401, 0x7642, 0x7EAB,
    /* ADD.L Dn,Dn: 0xD080-0xD087 + variations */
    0xD081, 0xD082, 0xD283, 0xD480, 0xD681,
    /* SUB.L Dn,Dn */
    0x9081, 0x9082, 0x9283, 0x9480,
    /* AND.L Dn,Dn */
    0xC081, 0xC082, 0xC480,
    /* OR.L Dn,Dn */
    0x8081, 0x8082, 0x8480,
    /* EOR.L Dn,Dn: 0xB180-0xB187 */
    0xB181, 0xB182, 0xB381,
    /* CMP.L Dn,Dn */
    0xB081, 0xB082, 0xB280,
    /* NEG.L Dn */
    0x4480, 0x4481, 0x4482,
    /* NOT.L Dn */
    0x4680, 0x4681, 0x4682,
    /* CLR.L Dn */
    0x4280, 0x4281,
    /* TST.L Dn */
    0x4A80, 0x4A81,
    /* SWAP Dn */
    0x4840, 0x4841,
    /* EXT.W Dn, EXT.L Dn */
    0x4880, 0x48C0,
    /* ADDQ.L #n,Dn */
    0x5080, 0x5280, 0x5480,
    /* SUBQ.L #n,Dn */
    0x5180, 0x5380, 0x5580,
    /* LSL.L #n,Dn / LSR.L #n,Dn */
    0xE388, 0xE288, 0xE588, 0xE488,
    /* ASL.L #1,Dn / ASR.L #1,Dn */
    0xE380, 0xE280,
    /* ROL.L #1,Dn / ROR.L #1,Dn */
    0xE398, 0xE298,
    /* MULU.W Dn,Dn */
    0xC0C1, 0xC2C0,
    /* MULS.W Dn,Dn */
    0xC1C1, 0xC3C0,
};
#define NUM_SAFE_OPCODES (sizeof safe_opcodes / sizeof safe_opcodes[0])

/* ------------------------------------------------------------------ */
/* Main comparison loop                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv) {
    int num_tests = 1000;
    if (argc > 1) num_tests = atoi(argv[1]);

    srand((unsigned)time(NULL));

    /* Init Musashi */
    m68k_init();
    m68k_set_cpu_type(M68K_CPU_TYPE_68020);

    /* Init our emulator */
    M68020BusInterface bus = {
        .read=our_read, .write=our_write, .iack=our_iack, .ctx=NULL
    };
    M68020State *our = m68020_create(&bus);

    int pass = 0, fail = 0;
    uint32_t pc_base = 0x1000;

    for (int t = 0; t < num_tests; t++) {
        /* Pick a random safe opcode */
        uint16_t opcode = safe_opcodes[rand() % NUM_SAFE_OPCODES];

        /* Generate random register state */
        uint32_t d[8], a[7];
        for (int i = 0; i < 8; i++) d[i] = xrand();
        for (int i = 0; i < 7; i++) a[i] = xrand() & 0xFFFFFE; /* even addresses */

        uint16_t sr = 0x2700 | (xrand() & 0x1F); /* supervisor + random CCR */

        /* Clear memory and place opcode */
        memset(g_mem, 0, MEM_SIZE);
        /* Reset vectors */
        g_mem[0]=0;g_mem[1]=0;g_mem[2]=0x80;g_mem[3]=0;
        g_mem[4]=(pc_base>>24);g_mem[5]=(pc_base>>16);g_mem[6]=(pc_base>>8);g_mem[7]=pc_base;
        /* Place opcode at pc_base, then many NOPs so Musashi doesn't
         * execute garbage after our test instruction */
        g_mem[pc_base]   = opcode >> 8;
        g_mem[pc_base+1] = opcode & 0xFF;
        for (int j = 0; j < 100; j++) {
            g_mem[pc_base+2+j*2]   = 0x4E; /* NOP */
            g_mem[pc_base+2+j*2+1] = 0x71;
        }

        /* ---- Set Musashi state ---- */
        m68k_set_reg(M68K_REG_SR, sr);
        m68k_set_reg(M68K_REG_PC, pc_base);
        for (int i = 0; i < 8; i++) m68k_set_reg(M68K_REG_D0+i, d[i]);
        for (int i = 0; i < 7; i++) m68k_set_reg(M68K_REG_A0+i, a[i]);
        m68k_set_reg(M68K_REG_A7, 0x8000); /* SSP */

        /* Save memory state before Musashi execution */
        static uint8_t mem_before[MEM_SIZE];
        memcpy(mem_before, g_mem, MEM_SIZE);

        /* Execute one instruction on Musashi.
         * m68k_execute(n) executes AT LEAST one instruction, consuming
         * at least n cycles. With n=1, it runs exactly one instruction. */
        m68k_execute(1);

        /* Save Musashi results */
        uint32_t m_d[8], m_a[7], m_pc, m_sr;
        for (int i = 0; i < 8; i++) m_d[i] = m68k_get_reg(NULL, M68K_REG_D0+i);
        for (int i = 0; i < 7; i++) m_a[i] = m68k_get_reg(NULL, M68K_REG_A0+i);
        m_pc = m68k_get_reg(NULL, M68K_REG_PC);
        m_sr = m68k_get_reg(NULL, M68K_REG_SR);

        /* ---- Set our emulator state ---- */
        /* Restore memory to before-state */
        memcpy(g_mem, mem_before, MEM_SIZE);

        /* Reset our CPU and set state directly */
        our->SR = sr;
        our->PC = pc_base;
        for (int i = 0; i < 8; i++) our->D[i] = d[i];
        for (int i = 0; i < 7; i++) our->A[i] = a[i];
        our->A[7] = 0x8000; /* SSP */
        our->ISP = 0x8000;
        our->MSP = 0x8000;
        our->halted = false;
        our->stopped = false;
        our->in_exception = false;
        our->flush_pending = false;
        pipeline_flush(our, pc_base);

        /* Execute one instruction */
        m68020_step(our);

        /* ---- Compare ---- */
        int mismatch = 0;
        char errbuf[1024] = "";
        int ep = 0;

        for (int i = 0; i < 8; i++) {
            if (our->D[i] != m_d[i]) {
                ep += snprintf(errbuf+ep, sizeof(errbuf)-ep,
                    "D%d:0x%X!=0x%X ", i, our->D[i], m_d[i]);
                mismatch++;
            }
        }
        uint16_t our_sr = our->SR & 0xFFFF;
        uint16_t exp_sr = m_sr & 0xFFFF;
        /* Compare only CCR (lower 5 bits) since upper SR may differ */
        if ((our_sr & 0x1F) != (exp_sr & 0x1F)) {
            ep += snprintf(errbuf+ep, sizeof(errbuf)-ep,
                "SR:0x%04X!=0x%04X ", our_sr, exp_sr);
            mismatch++;
        }

        if (mismatch) {
            fail++;
            if (fail <= 20) {
                printf("FAIL [%d] op=0x%04X: %s\n", t, opcode, errbuf);
            }
        } else {
            pass++;
        }
    }

    printf("\n%d/%d passed", pass, pass+fail);
    if (fail) printf(" (%d FAILED)", fail);
    printf("\n");

    m68020_destroy(our);
    return fail ? 1 : 0;
}
