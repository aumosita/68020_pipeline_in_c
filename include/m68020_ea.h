#pragma once

/*
 * m68020_ea.h — Effective Address resolver.
 *
 * The 68020 has 18 addressing modes.  Phase 1 implements the 11 classic
 * (pre-68020) modes; Phase 4 adds the 7 new memory-indirect modes.
 */

#include "m68020_types.h"

typedef struct M68020State M68020State;

/* ------------------------------------------------------------------ */
/* EA mode field encoding (bits [5:3] of opword, or [8:6] for MOVE dst)*/
/* ------------------------------------------------------------------ */

#define EA_MODE_DN    0u  /* 000  Dn                   */
#define EA_MODE_AN    1u  /* 001  An                   */
#define EA_MODE_IND   2u  /* 010  (An)                 */
#define EA_MODE_POST  3u  /* 011  (An)+                */
#define EA_MODE_PRE   4u  /* 100  -(An)                */
#define EA_MODE_D16   5u  /* 101  (d16,An)             */
#define EA_MODE_IDX   6u  /* 110  (d8,An,Xn) / full   */
#define EA_MODE_EXT   7u  /* 111  extended sub-modes   */

/* For mode=7, register field selects: */
#define EA_EXT_ABSW   0u  /* (xxx).W                  */
#define EA_EXT_ABSL   1u  /* (xxx).L                  */
#define EA_EXT_D16PC  2u  /* (d16,PC)                 */
#define EA_EXT_IDXPC  3u  /* (d8,PC,Xn)               */
#define EA_EXT_IMM    4u  /* #immediate               */

/* ------------------------------------------------------------------ */
/* EA descriptor                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    EAK_Dn,   /* data register direct — value is in D[reg]         */
    EAK_An,   /* address register direct — value is in A[reg]      */
    EAK_Mem,  /* memory — address is in ea.address                 */
    EAK_Imm,  /* immediate — value is in ea.imm_val                */
} EAKind;

typedef struct {
    EAKind  kind;
    u8      reg;         /* register number 0–7 (for Dn/An)         */
    u32     address;     /* resolved effective address (Mem modes)  */
    u32     imm_val;     /* immediate value (Imm mode)              */
    /* Post-increment bookkeeping: if is_postinc, apply after access */
    bool    is_postinc;
    u8      postinc_reg; /* address register index to increment     */
    u32     postinc_amt; /* amount to add after access              */
} EADesc;

/* ------------------------------------------------------------------ */
/* Functions                                                            */
/* ------------------------------------------------------------------ */

/*
 * Resolve the EA encoded by (mode, reg).
 *
 *   cpu      — CPU state (prefetch queue is consumed for extension words)
 *   mode     — 3-bit mode field from the opword
 *   reg      — 3-bit register field from the opword
 *   size     — operand size (needed for IMM and (An)+/-(An) increment)
 *   out      — descriptor filled in on success
 *
 * Returns false if an exception was triggered (bus error, address error).
 * On false return the CPU PC/SR have already been updated by the
 * exception handler and a longjmp has been issued; the caller should
 * simply return after checking the result (or it will never be reached).
 */
bool ea_resolve(M68020State *cpu, u8 mode, u8 reg,
                BusSize size, EADesc *out);

/*
 * Read from a resolved EA.
 * Applies post-increment side effects after a successful read.
 */
bool ea_read(M68020State *cpu, const EADesc *ea, BusSize size, u32 *val);

/*
 * Write to a resolved EA.
 * Applies post-increment side effects after a successful write.
 */
bool ea_write(M68020State *cpu, const EADesc *ea, BusSize size, u32 val);
