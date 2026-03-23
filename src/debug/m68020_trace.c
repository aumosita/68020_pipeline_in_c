/*
 * m68020_trace.c — Execution trace / logging.
 *
 * Provides a ring buffer of recent instruction executions.
 * Can be attached to the CPU's trace hook to automatically record
 * every instruction, or called manually for selective tracing.
 *
 * API:
 *   m68020_trace_init()   — allocate trace buffer
 *   m68020_trace_free()   — free trace buffer
 *   m68020_trace_enable() — attach to CPU's trace hook
 *   m68020_trace_disable()— detach from CPU
 *   m68020_trace_dump()   — print recent N entries to FILE*
 *   m68020_trace_entry()  — get a specific entry from the ring buffer
 */

#include "m68020_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Forward declaration — implemented in m68020_disasm.c */
int m68020_disasm(const u8 *mem, u32 pc, char *buf, int buflen);

/* ------------------------------------------------------------------ */
/* Trace entry and buffer                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 pc;
    u16 sr;
    u16 opword;
    u32 d[8];
    u32 a[8];
    u64 cycle;
} M68020TraceEntry;

typedef struct {
    M68020TraceEntry *entries;
    u32 capacity;    /* total slots in ring buffer */
    u32 head;        /* next write position */
    u32 count;       /* entries written (saturates at capacity) */
} M68020TraceBuffer;

static M68020TraceBuffer g_trace = { NULL, 0, 0, 0 };

/* ------------------------------------------------------------------ */
/* Trace hook callback                                                 */
/* ------------------------------------------------------------------ */

/*
 * Called before every instruction via the CPU's trace_hook.
 * Records the current state into the ring buffer.
 */
static void trace_hook_cb(M68020State *cpu, void *user) {
    (void)user;
    if (!g_trace.entries || g_trace.capacity == 0)
        return;

    M68020TraceEntry *e = &g_trace.entries[g_trace.head];
    e->pc     = cpu->PC;
    e->sr     = cpu->SR;
    e->cycle  = cpu->cycle_count;

    /* Read opword from the prefetch queue peek address */
    /* We can't easily get the opword before it's consumed, so we
     * read it from the bus directly (for logging purposes only). */
    u32 pc = cpu->PC;
    if (pc < (1u << 20) - 1) {
        /* Read from the bus context if available */
        u32 raw = 0;
        u32 cyc = 0;
        cpu->bus.read(cpu->bus.ctx, pc, SIZE_WORD, cpu_prog_fc(cpu), &raw, &cyc);
        e->opword = (u16)raw;
    } else {
        e->opword = 0;
    }

    memcpy(e->d, cpu->D, sizeof cpu->D);
    memcpy(e->a, cpu->A, sizeof cpu->A);

    g_trace.head = (g_trace.head + 1) % g_trace.capacity;
    if (g_trace.count < g_trace.capacity)
        g_trace.count++;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void m68020_trace_init(u32 capacity) {
    m68020_trace_free();
    g_trace.entries = (M68020TraceEntry *)calloc(capacity, sizeof(M68020TraceEntry));
    g_trace.capacity = capacity;
    g_trace.head = 0;
    g_trace.count = 0;
}

void m68020_trace_free(void) {
    free(g_trace.entries);
    g_trace.entries = NULL;
    g_trace.capacity = 0;
    g_trace.head = 0;
    g_trace.count = 0;
}

void m68020_trace_enable(M68020State *cpu) {
    m68020_set_trace_hook(cpu, trace_hook_cb, NULL);
}

void m68020_trace_disable(M68020State *cpu) {
    m68020_set_trace_hook(cpu, NULL, NULL);
}

/*
 * Dump the last `n` trace entries to the given file.
 * If n > count, dumps all available entries.
 * For each entry, disassembles the instruction at that PC.
 */
void m68020_trace_dump(FILE *out, u32 n, const u8 *mem, u32 mem_size) {
    if (!g_trace.entries || g_trace.count == 0) {
        fprintf(out, "(trace empty)\n");
        return;
    }
    if (n > g_trace.count) n = g_trace.count;

    /* Start position: head - n (wrapping) */
    u32 start = (g_trace.head + g_trace.capacity - n) % g_trace.capacity;

    for (u32 i = 0; i < n; i++) {
        u32 idx = (start + i) % g_trace.capacity;
        M68020TraceEntry *e = &g_trace.entries[idx];

        char disasm[128] = "???";
        if (e->pc < mem_size - 8)
            m68020_disasm(mem + e->pc, e->pc, disasm, sizeof disasm);

        fprintf(out, "%6llu  %08X  %04X  %-30s  D7=%08X D0=%08X A7=%08X SR=%04X\n",
                (unsigned long long)e->cycle,
                e->pc, e->opword, disasm,
                e->d[7], e->d[0], e->a[7], e->sr);
    }
}

u32 m68020_trace_count(void) {
    return g_trace.count;
}
