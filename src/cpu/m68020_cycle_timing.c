/*
 * m68020_cycle_timing.c — Per-instruction cycle timing helpers.
 *
 * The MC68020 C-stage (sequencer/decoder) partially overlaps with the
 * E-stage of the previous instruction.  The C-stage cost depends on
 * the complexity of the effective address mode:
 *
 *   - Register direct, immediate, (An), (An)+, -(An): 0 extra cycles
 *     (fully overlapped with previous E-stage, no extension words)
 *   - (d16,An) / (d16,PC): 2 cycles (one extension word to decode)
 *   - (d8,An,Xn) brief format: 4 cycles (index scaling + displacement)
 *   - (d8,PC,Xn) brief format: 4 cycles
 *   - Full format extension (bd, od, memory indirect): 4-10 cycles
 *   - (xxx).W: 2 cycles
 *   - (xxx).L: 4 cycles (two extension words)
 *
 * These costs are modeled as the "decode penalty" — the portion of
 * C-stage work that could NOT be overlapped with the previous E-stage
 * and must be paid sequentially.  For simple modes (register, immediate,
 * (An)+, etc.) the decode penalty is 0 because the C-stage completes
 * within the previous instruction's E-stage time.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"

/* ------------------------------------------------------------------ */
/* EA decode cost (C-stage penalty)                                    */
/* ------------------------------------------------------------------ */

/*
 * Compute the C-stage decode penalty for a given EA mode.
 * This represents cycles the C-stage needs that cannot be hidden
 * behind the previous instruction's execution.
 *
 * Parameters:
 *   mode — 3-bit EA mode (bits [5:3] of opword)
 *   reg  — 3-bit EA register (bits [2:0] of opword)
 *
 * The caller doesn't need to consume extension words; this function
 * only estimates the cost based on the mode encoding.
 */
u32 ea_decode_cost(u8 mode, u8 reg) {
    switch (mode) {
    case EA_MODE_DN:    /* Dn — register direct */
    case EA_MODE_AN:    /* An — register direct */
    case EA_MODE_IND:   /* (An) */
    case EA_MODE_POST:  /* (An)+ */
    case EA_MODE_PRE:   /* -(An) */
        return 0;

    case EA_MODE_D16:   /* (d16,An) — one extension word */
        return 2;

    case EA_MODE_IDX:   /* (d8,An,Xn) or full format — complex */
        /* Brief format: 4 cycles for index calculation.
         * Full format: up to 10 cycles (base disp + outer disp + indirection).
         * We estimate 4 as a reasonable average; the full format's extra
         * cost is primarily bus cycles which are tracked separately. */
        return 4;

    case EA_MODE_EXT:   /* Extended modes — depends on reg field */
        switch (reg) {
        case EA_EXT_ABSW:    /* (xxx).W — one extension word */
            return 2;
        case EA_EXT_ABSL:    /* (xxx).L — two extension words */
            return 4;
        case EA_EXT_D16PC:   /* (d16,PC) — one extension word */
            return 2;
        case EA_EXT_IDXPC:   /* (d8,PC,Xn) — index calculation */
            return 4;
        case EA_EXT_IMM:     /* #immediate — 0-2 extension words */
            return 0;         /* typically overlapped with prefetch */
        default:
            return 0;
        }

    default:
        return 0;
    }
}

/*
 * Extract EA mode and register from a standard opword (source EA in bits [5:0]).
 * Returns the C-stage decode penalty.
 */
u32 opword_decode_cost(u16 opword) {
    u8 mode = (opword >> 3) & 7u;
    u8 reg  = opword & 7u;
    return ea_decode_cost(mode, reg);
}
