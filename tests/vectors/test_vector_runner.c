/*
 * test_vector_runner.c — Execute one SingleStepTests test vector.
 *
 * Reads initial state from stdin, executes one instruction, compares
 * with expected final state. Exits 0 on match, 1 on mismatch.
 *
 * Input format (one line, decimal values, | separates initial from expected):
 *   pc sr d0..d7 a0..a6 usp ssp ram_count [addr:val ...] | pc sr d0..d7 a0..a6 usp ssp ram_count [addr:val ...]
 */

#include "m68020.h"
#include "m68020_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE (16 * 1024 * 1024)  /* 16 MB flat memory */

static uint8_t *g_mem = NULL;

static BusResult vec_read(void *ctx, u32 addr, BusSize size,
                           FunctionCode fc, u32 *val, u32 *cycles_out) {
    (void)ctx; (void)fc;
    *cycles_out = 0;
    addr &= (MEM_SIZE - 1);
    switch (size) {
        case SIZE_BYTE: *val = g_mem[addr]; break;
        case SIZE_WORD: *val = ((u32)g_mem[addr]<<8)|g_mem[addr+1]; break;
        case SIZE_LONG: *val = ((u32)g_mem[addr]<<24)|((u32)g_mem[addr+1]<<16)
                              |((u32)g_mem[addr+2]<<8)|g_mem[addr+3]; break;
    }
    return BUS_OK;
}

static BusResult vec_write(void *ctx, u32 addr, BusSize size,
                            FunctionCode fc, u32 val, u32 *cycles_out) {
    (void)ctx; (void)fc;
    *cycles_out = 0;
    addr &= (MEM_SIZE - 1);
    switch (size) {
        case SIZE_BYTE: g_mem[addr]=(uint8_t)val; break;
        case SIZE_WORD: g_mem[addr]=(uint8_t)(val>>8); g_mem[addr+1]=(uint8_t)val; break;
        case SIZE_LONG: g_mem[addr]=(uint8_t)(val>>24); g_mem[addr+1]=(uint8_t)(val>>16);
                        g_mem[addr+2]=(uint8_t)(val>>8); g_mem[addr+3]=(uint8_t)val; break;
    }
    return BUS_OK;
}

static u8 vec_iack(void *ctx, u8 level) { (void)ctx; (void)level; return 0xFF; }

typedef struct {
    u32 pc, sr;
    u32 d[8], a[7];
    u32 usp, ssp;
    int ram_count;
    u32 ram_addr[1024];
    u8  ram_val[1024];
} CpuState;

static int parse_state(CpuState *s, char **pos) {
    char *p = *pos;
    if (sscanf(p, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %d%n",
               &s->pc, &s->sr,
               &s->d[0], &s->d[1], &s->d[2], &s->d[3],
               &s->d[4], &s->d[5], &s->d[6], &s->d[7],
               &s->a[0], &s->a[1], &s->a[2], &s->a[3],
               &s->a[4], &s->a[5], &s->a[6],
               &s->usp, &s->ssp,
               &s->ram_count, &(int){0}) < 20) return -1;

    /* Advance past the parsed portion */
    int consumed = 0;
    sscanf(p, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %d%n",
           &s->pc, &s->sr,
           &s->d[0], &s->d[1], &s->d[2], &s->d[3],
           &s->d[4], &s->d[5], &s->d[6], &s->d[7],
           &s->a[0], &s->a[1], &s->a[2], &s->a[3],
           &s->a[4], &s->a[5], &s->a[6],
           &s->usp, &s->ssp,
           &s->ram_count, &consumed);
    p += consumed;

    for (int i = 0; i < s->ram_count && i < 1024; i++) {
        u32 addr; u32 val;
        int n = 0;
        if (sscanf(p, " %u:%u%n", &addr, &val, &n) < 2) return -1;
        s->ram_addr[i] = addr;
        s->ram_val[i] = (u8)val;
        p += n;
    }
    *pos = p;
    return 0;
}

int main(void) {
    g_mem = (uint8_t *)calloc(1, MEM_SIZE);
    if (!g_mem) { fprintf(stderr, "OOM\n"); return 2; }

    char line[65536];
    if (!fgets(line, sizeof line, stdin)) return 2;

    /* Split at '|' */
    char *sep = strchr(line, '|');
    if (!sep) { fprintf(stderr, "No separator\n"); free(g_mem); return 2; }
    *sep = '\0';

    CpuState ini, fin;
    char *p_ini = line;
    char *p_fin = sep + 1;

    if (parse_state(&ini, &p_ini) < 0) { fprintf(stderr, "Parse initial\n"); free(g_mem); return 2; }
    if (parse_state(&fin, &p_fin) < 0) { fprintf(stderr, "Parse final\n"); free(g_mem); return 2; }

    /* Set up memory from initial RAM */
    memset(g_mem, 0, MEM_SIZE);

    /* Write reset vectors so the CPU can initialize:
     * Vector 0 (SSP) at addr 0, Vector 1 (PC) at addr 4 */
    u32 ssp = ini.ssp;
    u32 pc  = ini.pc;
    g_mem[0]=(ssp>>24); g_mem[1]=(ssp>>16); g_mem[2]=(ssp>>8); g_mem[3]=ssp;
    g_mem[4]=(pc>>24);  g_mem[5]=(pc>>16);  g_mem[6]=(pc>>8);  g_mem[7]=pc;

    /* Load initial RAM (instruction bytes + data) */
    for (int i = 0; i < ini.ram_count; i++) {
        u32 addr = ini.ram_addr[i] & (MEM_SIZE - 1);
        g_mem[addr] = ini.ram_val[i];
    }

    M68020BusInterface bus = {
        .read = vec_read, .write = vec_write, .iack = vec_iack,
        .reset_peripherals = NULL, .ctx = NULL
    };
    M68020State *cpu = m68020_create(&bus);

    /* Instead of m68020_reset (which reads vectors), set state directly */
    cpu->SR = (u16)ini.sr;
    cpu->PC = ini.pc;
    for (int i = 0; i < 8; i++) cpu->D[i] = ini.d[i];
    for (int i = 0; i < 7; i++) cpu->A[i] = ini.a[i];
    cpu->USP = ini.usp;

    /* A7 = SSP in supervisor mode, USP in user mode */
    if (ini.sr & 0x2000) {
        cpu->A[7] = ini.ssp;
        cpu->ISP = ini.ssp;
        cpu->MSP = ini.ssp;
    } else {
        cpu->A[7] = ini.usp;
        cpu->ISP = ini.ssp;
    }

    /* Flush prefetch and set PC */
    pipeline_flush(cpu, ini.pc);
    cpu->PC = ini.pc;
    cpu->flush_pending = false;
    cpu->halted = false;
    cpu->stopped = false;

    /* Execute one instruction */
    m68020_step(cpu);

    /* Compare results */
    int mismatches = 0;
    char errbuf[4096];
    int errpos = 0;

#define CHK(name, got, exp) do { \
    if ((u32)(got) != (u32)(exp)) { \
        errpos += snprintf(errbuf+errpos, sizeof(errbuf)-errpos, \
            "%s: got=%u(0x%X) exp=%u(0x%X) ", name, (u32)(got), (u32)(got), (u32)(exp), (u32)(exp)); \
        mismatches++; \
    } \
} while(0)

    /* Compare registers */
    for (int i = 0; i < 8; i++) {
        char nm[4]; snprintf(nm, sizeof nm, "D%d", i);
        CHK(nm, cpu->D[i], fin.d[i]);
    }
    for (int i = 0; i < 7; i++) {
        char nm[4]; snprintf(nm, sizeof nm, "A%d", i);
        CHK(nm, cpu->A[i], fin.a[i]);
    }

    /* A7/SP depends on supervisor mode */
    u32 exp_ssp = fin.ssp;
    u32 exp_usp = fin.usp;
    if (fin.sr & 0x2000) {
        CHK("SSP", cpu->A[7], exp_ssp);
    } else {
        CHK("USP", cpu->A[7], exp_usp);
    }

    /* Our emulator's cpu->PC = address of last executed instruction.
     * The test vectors expect PC = address of NEXT instruction.
     * Use pipeline_peek_pc() which gives the next fetch address. */
    CHK("PC", pipeline_peek_pc(cpu), fin.pc);
    CHK("SR", cpu->SR & 0xFFFF, fin.sr & 0xFFFF);

    /* Compare RAM */
    for (int i = 0; i < fin.ram_count; i++) {
        u32 addr = fin.ram_addr[i] & (MEM_SIZE - 1);
        u8 got = g_mem[addr];
        u8 exp = fin.ram_val[i];
        if (got != exp) {
            errpos += snprintf(errbuf+errpos, sizeof(errbuf)-errpos,
                "RAM[%u]: got=%u exp=%u ", addr, got, exp);
            mismatches++;
        }
    }

    if (mismatches) {
        errbuf[errpos] = '\0';
        printf("FAIL: %s\n", errbuf);
    }

    m68020_destroy(cpu);
    free(g_mem);
    return mismatches ? 1 : 0;
}
