#pragma once

/*
 * m68020_internal.h — internal CPU state and helper declarations.
 * NOT part of the public API; only included by emulator source files.
 */

#include "m68020.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"
#include <setjmp.h>

/* ------------------------------------------------------------------ */
/* Prefetch Queue                                                        */
/* ------------------------------------------------------------------ */

#define PREFETCH_DEPTH 4   /* 4 × 16-bit words = 8 bytes */

typedef struct {
    u16  buf[PREFETCH_DEPTH];  /* buffered instruction words        */
    u32  pc [PREFETCH_DEPTH];  /* bus address each word was fetched from */
    u8   head;                 /* index of next word to consume     */
    u8   count;                /* number of valid words in queue    */
    u32  fetch_addr;           /* next bus address to prefetch from */
} PrefetchQueue;

/* ------------------------------------------------------------------ */
/* Instruction Cache (256 B, 64-entry, direct-mapped, read-only)       */
/* ------------------------------------------------------------------ */

#define ICACHE_ENTRIES 64

typedef struct {
    u32  tag  [ICACHE_ENTRIES];  /* upper address bits (tag)       */
    u32  data [ICACHE_ENTRIES];  /* cached long word (4 bytes)     */
    bool valid[ICACHE_ENTRIES];  /* valid bit                      */
} InstructionCache;

/* ------------------------------------------------------------------ */
/* Coprocessor                                                          */
/* ------------------------------------------------------------------ */

struct M68020Coprocessor {
    u8   cpid;
    void (*execute      )(struct M68020Coprocessor *, M68020State *, u16);
    void (*save         )(struct M68020Coprocessor *, M68020State *, u32);
    void (*restore      )(struct M68020Coprocessor *, M68020State *, u32);
    bool (*test_condition)(struct M68020Coprocessor *, u8);
    void *state;
};

/* ------------------------------------------------------------------ */
/* CPU State                                                            */
/* ------------------------------------------------------------------ */

struct M68020State {
    /* -------- Architecturally visible registers -------- */
    u32  D[8];          /* D0–D7 data registers                   */
    u32  A[8];          /* A0–A7 address registers; A7 = active SP*/
    u32  PC;            /* PC of current instruction (opword addr)*/
    u16  SR;            /* Status Register (T1,T0,S,M,IPL,X,N,Z,V,C) */

    /* -------- 68020 control registers -------- */
    u32  VBR;           /* Vector Base Register                   */
    u32  CACR;          /* Cache Control Register                 */
    u32  CAAR;          /* Cache Address Register                 */
    u32  SFC;           /* Source Function Code (3-bit)           */
    u32  DFC;           /* Destination Function Code (3-bit)      */

    /* -------- Saved stack pointers -------- */
    u32  USP;           /* User Stack Pointer (saved when S=1)    */
    u32  MSP;           /* Master Stack Pointer (supervisor, M=1) */
    u32  ISP;           /* Interrupt Stack Pointer (supervisor, M=0) */

    /* -------- Pipeline -------- */
    PrefetchQueue    prefetch;
    InstructionCache icache;

    /* -------- Pipeline Overlap Accounting -------- */
    /*
     * The MC68020 3-stage pipeline (B/C/E) allows the bus controller
     * (B-stage) to prefetch while the execution unit (E-stage) runs.
     * We model this overlap without full cycle-by-cycle simulation:
     *
     * b_avail_cycles: How many cycles the B-stage had to prefetch
     *   during the previous instruction's E-stage.  Subtracted from
     *   the next instruction's bus fetch cost.
     *
     * last_e_cycles: The E-stage cycle count of the previous instruction.
     *   This becomes b_avail_cycles for the next instruction.
     *
     * e_bus_cycles: Bus cycles consumed by the E-stage for data access
     *   (not instruction fetch). During these cycles, B-stage cannot
     *   use the bus → reduces effective overlap.
     *
     * flush_pending: Set by pipeline_flush() when a branch is taken.
     *   The next instruction pays a refill penalty instead of getting
     *   overlap benefit.
     */
    u32  b_avail_cycles;
    u32  last_e_cycles;
    u32  e_bus_cycles;
    bool flush_pending;

    /* Phase 6: Cache miss / prefetch depth tracking */
    u32  b_fetch_cycles;       /* ifetch bus cycles this instruction (miss=4, hit=0) */
    u32  last_b_fetch_cycles;  /* saved from previous instruction */
    u8   words_consumed;       /* prefetch words consumed this instruction */
    u8   last_words_consumed;  /* saved from previous instruction */

    /* -------- Execution control -------- */
    u64  cycle_count;
    u32  instr_count;
    bool halted;         /* double bus error → HALT                */
    bool stopped;        /* STOP instruction waiting for interrupt  */
    bool trace_pending;  /* deferred trace exception               */

    /* -------- Interrupt/exception state -------- */
    u8   pending_ipl;    /* current IPL on the pins (0–7)          */
    bool in_exception;   /* set while processing an exception      */

    /* -------- Exception longjmp target -------- */
    /*
     * When fault_jmp_active is true, any exception (bus error, illegal
     * instruction, privilege violation, …) will longjmp here with:
     *   val=1 : exception taken, continue from new PC
     *   val=2 : double bus error, CPU halted
     */
    jmp_buf fault_jmp;
    bool    fault_jmp_active;

    /* -------- Bus interface -------- */
    M68020BusInterface bus;

    /* -------- Coprocessors -------- */
    M68020Coprocessor *coprocessors[8];  /* CpID 0–7, NULL if absent */

    /* -------- Debug hook -------- */
    void (*trace_hook)(struct M68020State *, void *);
    void *trace_hook_user;
};

/* ------------------------------------------------------------------ */
/* SR bit-field macros (read)                                           */
/* ------------------------------------------------------------------ */

#define SR_T1(sr)  (((sr) >> 15) & 1u)
#define SR_T0(sr)  (((sr) >> 14) & 1u)
#define SR_S(sr)   (((sr) >> 13) & 1u)
#define SR_M(sr)   (((sr) >> 12) & 1u)
#define SR_IPL(sr) (((sr) >>  8) & 7u)
#define SR_X(sr)   (((sr) >>  4) & 1u)
#define SR_N(sr)   (((sr) >>  3) & 1u)
#define SR_Z(sr)   (((sr) >>  2) & 1u)
#define SR_V(sr)   (((sr) >>  1) & 1u)
#define SR_C(sr)   (((sr) >>  0) & 1u)

/* CCR bit masks (within the low byte of SR) */
#define CCR_X  0x10u
#define CCR_N  0x08u
#define CCR_Z  0x04u
#define CCR_V  0x02u
#define CCR_C  0x01u

/* Mask that clears reserved SR bits (11, 7, 6, 5) */
#define SR_VALID_MASK  0xF71Fu

/* ------------------------------------------------------------------ */
/* Instruction Cache                                                    */
/* ------------------------------------------------------------------ */

bool icache_lookup(M68020State *cpu, u32 addr, u16 *val);
void icache_fill(M68020State *cpu, u32 addr, u32 longword);
void icache_invalidate_all(M68020State *cpu);
void icache_invalidate_entry(M68020State *cpu, u32 addr);
void icache_update_cacr(M68020State *cpu, u32 new_cacr);

/* ------------------------------------------------------------------ */
/* Pipeline                                                             */
/* ------------------------------------------------------------------ */

/* Consume and return the next instruction word from the prefetch queue.
 * If the queue is empty, fetches from bus (may trigger bus error). */
u16  pipeline_consume_word(M68020State *cpu);

/* Return the bus address of the next word in the prefetch queue
 * (the address that would be returned by the next consume_word). */
u32  pipeline_peek_pc(const M68020State *cpu);

/* Flush the prefetch queue and set the next-fetch address to new_pc. */
void pipeline_flush(M68020State *cpu, u32 new_pc);

/* Fill the prefetch queue up to PREFETCH_DEPTH words if possible. */
void pipeline_refill(M68020State *cpu);

/* Partial refill: fill up to n words (for minimal B-stage restart). */
void pipeline_refill_n(M68020State *cpu, u8 n);

/* ------------------------------------------------------------------ */
/* Pipeline Overlap Accounting                                         */
/* ------------------------------------------------------------------ */

/* Called before each instruction to compute how many prefetch cycles
 * were overlapped with the previous E-stage execution. */
u32  pipeline_overlap_begin(M68020State *cpu);

/* Called after each instruction to record its E-stage timing for
 * the next instruction's overlap calculation. */
void pipeline_overlap_end(M68020State *cpu, u32 e_cycles, u32 e_bus_cycles);

/* Track data bus cycles consumed by E-stage (called from bus helpers) */
void pipeline_note_data_bus(M68020State *cpu, u32 cycles);

/* ------------------------------------------------------------------ */
/* Cycle Timing                                                        */
/* ------------------------------------------------------------------ */

/* C-stage decode penalty for a given EA mode (mode, reg fields) */
u32  ea_decode_cost(u8 mode, u8 reg);

/* Extract EA from standard opword bits [5:0] and return decode cost */
u32  opword_decode_cost(u16 opword);

/* ------------------------------------------------------------------ */
/* Bus helpers (internal — carry FC, alignment checks, etc.)           */
/* ------------------------------------------------------------------ */

FunctionCode cpu_data_fc(const M68020State *cpu);
FunctionCode cpu_prog_fc(const M68020State *cpu);

BusResult cpu_read_byte(M68020State *cpu, u32 addr, u8  *val);
BusResult cpu_read_word(M68020State *cpu, u32 addr, u16 *val);
BusResult cpu_read_long(M68020State *cpu, u32 addr, u32 *val);
BusResult cpu_write_byte(M68020State *cpu, u32 addr, u8   val);
BusResult cpu_write_word(M68020State *cpu, u32 addr, u16  val);
BusResult cpu_write_long(M68020State *cpu, u32 addr, u32  val);

/* Instruction fetch (uses program FC) */
BusResult cpu_fetch_word(M68020State *cpu, u32 addr, u16 *val);

/* Stack push/pop (use active A[7]) */
void cpu_push_word(M68020State *cpu, u16 val);
void cpu_push_long(M68020State *cpu, u32 val);
u16  cpu_pop_word (M68020State *cpu);
u32  cpu_pop_long (M68020State *cpu);

/* ------------------------------------------------------------------ */
/* SR management                                                        */
/* ------------------------------------------------------------------ */

/* Set the SR, handling stack-pointer switching when S or M changes. */
void cpu_set_sr(M68020State *cpu, u16 new_sr);

/* ------------------------------------------------------------------ */
/* Exception processing                                                 */
/* ------------------------------------------------------------------ */

void exception_process(M68020State *cpu, u32 vector);
void exception_bus_error(M68020State *cpu, u32 fault_addr,
                         bool is_write, u16 ssw);
void exception_address_error(M68020State *cpu, u32 fault_addr, bool is_write);
void m68020_process_interrupt(M68020State *cpu);
bool m68020_test_cc(u16 sr, u8 cc);

/* ------------------------------------------------------------------ */
/* CCR helpers (used by all instruction modules)                       */
/* ------------------------------------------------------------------ */

static inline u32 sz_mask(BusSize sz) {
    return sz == SIZE_BYTE ? 0xFFu : sz == SIZE_WORD ? 0xFFFFu : 0xFFFFFFFFu;
}
static inline u32 sz_sign(BusSize sz) {
    return sz == SIZE_BYTE ? 0x80u : sz == SIZE_WORD ? 0x8000u : 0x80000000u;
}

/* CCR for logic operations (C=0, V=0, N/Z from result) */
static inline u16 ccr_logic(u32 result, BusSize sz) {
    result &= sz_mask(sz);
    u16 c = 0;
    if (!result)           c |= CCR_Z;
    if (result & sz_sign(sz)) c |= CCR_N;
    return c;
}

/* CCR for ADD (dst + src = result) — includes X flag */
static inline u16 ccr_add(u32 src, u32 dst, u32 result, BusSize sz) {
    u32 m = sz_mask(sz), s = sz_sign(sz);
    src &= m; dst &= m; result &= m;
    u16 c = 0;
    if (!result)                         c |= CCR_Z;
    if (result & s)                      c |= CCR_N;
    if ((u64)src + (u64)dst > (u64)m)   c |= CCR_C | CCR_X;   /* unsigned carry */
    /* Signed overflow: both operands same sign, result different */
    if (!((src ^ dst) & s) && ((src ^ result) & s)) c |= CCR_V;
    return c;
}

/* CCR for SUB (dst - src = result) — includes X flag */
static inline u16 ccr_sub(u32 src, u32 dst, u32 result, BusSize sz) {
    u32 m = sz_mask(sz), s = sz_sign(sz);
    src &= m; dst &= m; result &= m;
    u16 c = 0;
    if (!result)    c |= CCR_Z;
    if (result & s) c |= CCR_N;
    if (src > dst)  c |= CCR_C | CCR_X;   /* borrow */
    /* Signed overflow: operands have different signs, result sign ≠ dst sign */
    if ((src ^ dst) & s && (dst ^ result) & s) c |= CCR_V;
    return c;
}

/* ------------------------------------------------------------------ */
/* Instruction dispatch                                                 */
/* ------------------------------------------------------------------ */

/* Each handler receives the full CPU state and the 16-bit opword.
 * Returns: number of bus cycles consumed by this instruction. */
typedef u32 (*InsnHandler)(M68020State *cpu, u16 opword);

/* Global flat dispatch table: indexed by the 16-bit opword */
extern InsnHandler g_dispatch[65536];

/* Build the dispatch table at startup */
void m68020_build_dispatch_table(void);

/* Per-module handler installers — called by m68020_build_dispatch_table */
void m68020_move_install_handlers    (InsnHandler *t);
void m68020_branch_install_handlers  (InsnHandler *t);
void m68020_jump_install_handlers    (InsnHandler *t);
void m68020_alu_install_handlers     (InsnHandler *t);
void m68020_shift_install_handlers   (InsnHandler *t);
void m68020_mul_div_install_handlers (InsnHandler *t);
void m68020_load_store_install_handlers(InsnHandler *t);
void m68020_compare_install_handlers (InsnHandler *t);
void m68020_cas_install_handlers     (InsnHandler *t);
void m68020_misc_install_handlers    (InsnHandler *t);
void m68020_bitfield_install_handlers(InsnHandler *t);
void m68020_callm_rtm_install_handlers(InsnHandler *t);
void m68020_coproc_install_handlers  (InsnHandler *t);
void m68020_privileged_install_handlers(InsnHandler *t);
