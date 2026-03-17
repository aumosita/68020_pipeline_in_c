#pragma once

#include "m68020_types.h"

/* Forward declarations */
typedef struct M68020State      M68020State;
typedef struct M68020Coprocessor M68020Coprocessor;

/* ------------------------------------------------------------------ */
/* Bus Interface                                                        */
/* ------------------------------------------------------------------ */
/*
 * M68020BusInterface — filled in by the computer simulator, passed to
 * m68020_create().  Every memory access (instruction fetch, data read/write)
 * is routed through these callbacks so that the bus subsystem can attach
 * RAM, ROM, MMIO, DMA, and bus-error injection transparently.
 *
 * cycles_out: if non-NULL the callee writes the number of extra bus cycles
 *             consumed by this access (wait states).  Write 0 if not modelled.
 */
typedef struct {
    /* Data/instruction read: returns raw value in *val (right-justified) */
    BusResult (*read)(void *ctx, u32 addr, BusSize size,
                      FunctionCode fc, u32 *val, u32 *cycles_out);

    /* Data write: value is in the low `size` bytes */
    BusResult (*write)(void *ctx, u32 addr, BusSize size,
                       FunctionCode fc, u32 val, u32 *cycles_out);

    /* Interrupt acknowledge cycle (FC=7, A3:A1 = level).
     * Returns: 0x00 = spurious, 0xFF = autovector, else vector number. */
    u8 (*iack)(void *ctx, u8 level);

    /* Called when the CPU executes the RESET instruction */
    void (*reset_peripherals)(void *ctx);

    /* Opaque pointer passed back as ctx in every callback */
    void *ctx;
} M68020BusInterface;

/* ------------------------------------------------------------------ */
/* Register identifiers                                                 */
/* ------------------------------------------------------------------ */

typedef enum {
    REG_D0, REG_D1, REG_D2, REG_D3, REG_D4, REG_D5, REG_D6, REG_D7,
    REG_A0, REG_A1, REG_A2, REG_A3, REG_A4, REG_A5, REG_A6, REG_A7,
    REG_PC,
    REG_SR,
    REG_USP,
    REG_MSP,
    REG_ISP,
    REG_VBR,
    REG_CACR,
    REG_CAAR,
    REG_SFC,
    REG_DFC,
} M68020Reg;

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/* Allocate and initialise a new CPU instance */
M68020State *m68020_create(const M68020BusInterface *bus);

/* Free a CPU instance */
void m68020_destroy(M68020State *cpu);

/* Hardware reset: loads SSP from vector 0, PC from vector 1 */
void m68020_reset(M68020State *cpu);

/* Execute exactly one instruction. Returns bus cycles consumed. */
u32 m68020_step(M68020State *cpu);

/* Execute until cycle_budget cycles are consumed.
 * Returns actual cycles executed (may slightly exceed budget). */
u64 m68020_run(M68020State *cpu, u64 cycle_budget);

/* Assert an interrupt priority level (0 = deassert, 1-7 = request) */
void m68020_set_ipl(M68020State *cpu, u8 level);

/* Attach a coprocessor to slot cpid (0-7) */
void m68020_attach_coprocessor(M68020State *cpu, M68020Coprocessor *cp);

/* Register access (for debuggers / test harnesses) */
u32  m68020_get_reg(const M68020State *cpu, M68020Reg reg);
void m68020_set_reg(M68020State *cpu, M68020Reg reg, u32 value);
u16  m68020_get_sr(const M68020State *cpu);
void m68020_set_sr(M68020State *cpu, u16 sr);

/* Install a pre-decode trace hook (called before every instruction) */
void m68020_set_trace_hook(M68020State *cpu,
                            void (*hook)(M68020State *, void *),
                            void *user);
