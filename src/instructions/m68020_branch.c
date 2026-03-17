/*
 * m68020_branch.c — BRA, BSR, Bcc, DBcc, Scc.
 *
 * Branch encoding (0110 CCCC disp8):
 *   CCCC = 0000 → BRA
 *   CCCC = 0001 → BSR
 *   CCCC = 0010-1111 → Bcc
 *
 * Displacement encoding:
 *   disp8 = 0x00 → 16-bit displacement in next word
 *   disp8 = 0xFF → 32-bit displacement in next two words (68020 only)
 *   otherwise → sign-extended 8-bit displacement (relative to end of opword)
 *
 * The branch target address = PC_of_opword + 2 + displacement
 * (PC advances past the opword before the displacement is added).
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* Forward declaration from exceptions.c */
bool m68020_test_cc(u16 sr, u8 cc);

/* ------------------------------------------------------------------ */
/* Branch displacement decoder                                         */
/* ------------------------------------------------------------------ */

/*
 * Compute the branch target address.
 * opword_pc = address of the branch opword.
 * disp8     = low byte of the opword (the short displacement field).
 * Returns the computed target address and consumes extension words.
 */
static u32 branch_target(M68020State *cpu, u32 opword_pc, u8 disp8) {
    /* PC at displacement calculation = opword_pc + 2 */
    u32 base = opword_pc + 2;

    if (disp8 == 0x00) {
        /* 16-bit displacement */
        s16 disp = (s16)pipeline_consume_word(cpu);
        return base + (s32)disp;
    } else if (disp8 == 0xFF) {
        /* 32-bit displacement (68020) */
        u16 hi = pipeline_consume_word(cpu);
        u16 lo = pipeline_consume_word(cpu);
        s32 disp = (s32)(((u32)hi << 16) | lo);
        return base + disp;
    } else {
        /* 8-bit signed displacement */
        return base + (s32)(s8)disp8;
    }
}

/* ------------------------------------------------------------------ */
/* BRA — Branch Always                                                 */
/* ------------------------------------------------------------------ */

static u32 handler_bra(M68020State *cpu, u16 opword) {
    u32 opword_pc = cpu->PC;
    u8  disp8     = (u8)(opword & 0xFFu);
    u32 target    = branch_target(cpu, opword_pc, disp8);

    pipeline_flush(cpu, target);
    cpu->PC = target;
    if (SR_T0(cpu->SR)) cpu->trace_pending = true;
    return 10;
}

/* ------------------------------------------------------------------ */
/* BSR — Branch to Subroutine                                          */
/* ------------------------------------------------------------------ */

static u32 handler_bsr(M68020State *cpu, u16 opword) {
    u32 opword_pc = cpu->PC;
    u8  disp8     = (u8)(opword & 0xFFu);

    /* Compute return address = address of instruction after BSR */
    u32 return_pc;
    u32 target;

    if (disp8 == 0x00) {
        /* Need to peek the PC BEFORE consuming the word */
        u32 ext_pc = pipeline_peek_pc(cpu);
        s16 disp   = (s16)pipeline_consume_word(cpu);
        return_pc  = ext_pc + 2;
        target     = (opword_pc + 2) + (s32)disp;
    } else if (disp8 == 0xFF) {
        u32 ext_pc1 = pipeline_peek_pc(cpu);
        u16 hi      = pipeline_consume_word(cpu);
        u16 lo      = pipeline_consume_word(cpu);
        return_pc   = ext_pc1 + 4;
        s32 disp    = (s32)(((u32)hi << 16) | lo);
        target      = (opword_pc + 2) + disp;
    } else {
        return_pc = opword_pc + 2;
        target    = return_pc + (s32)(s8)disp8;
    }

    cpu_push_long(cpu, return_pc);
    pipeline_flush(cpu, target);
    cpu->PC = target;
    if (SR_T0(cpu->SR)) cpu->trace_pending = true;
    return 18;
}

/* ------------------------------------------------------------------ */
/* Bcc — Branch on Condition                                           */
/* ------------------------------------------------------------------ */

static u32 handler_bcc(M68020State *cpu, u16 opword) {
    u8  cc        = (opword >> 8) & 0xFu;
    u8  disp8     = (u8)(opword & 0xFFu);
    u32 opword_pc = cpu->PC;

    /* Pre-consume extension word(s) to advance the instruction stream,
     * regardless of whether the branch is taken (the CPU always prefetches) */
    u32 target = branch_target(cpu, opword_pc, disp8);

    if (m68020_test_cc(cpu->SR, cc)) {
        pipeline_flush(cpu, target);
        cpu->PC = target;
        if (SR_T0(cpu->SR)) cpu->trace_pending = true;
        return 10;  /* taken */
    }
    return 8;  /* not taken (fall through, prefetch already advanced) */
}

/* ------------------------------------------------------------------ */
/* DBcc — Decrement and Branch                                         */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0101 CCCC 1100 1rrr  (followed by 16-bit displacement)
 * If condition is false: decrement Dn.W; if Dn.W != -1, branch.
 */
static u32 handler_dbcc(M68020State *cpu, u16 opword) {
    u8  cc        = (opword >> 8) & 0xFu;
    u8  dn        = opword & 7u;
    u32 opword_pc = cpu->PC;

    s16 disp    = (s16)pipeline_consume_word(cpu);
    u32 target  = (opword_pc + 2) + (s32)disp;

    if (m68020_test_cc(cpu->SR, cc)) {
        /* Condition true: don't loop */
        return 12;
    }

    /* Decrement low 16 bits of Dn */
    u16 counter = (u16)(cpu->D[dn]) - 1;
    cpu->D[dn] = (cpu->D[dn] & 0xFFFF0000u) | counter;

    if (counter != 0xFFFFu) {
        /* Branch taken */
        pipeline_flush(cpu, target);
        cpu->PC = target;
        if (SR_T0(cpu->SR)) cpu->trace_pending = true;
        return 10;
    }
    /* Counter expired (== -1): fall through */
    return 14;
}

/* ------------------------------------------------------------------ */
/* Scc — Set on Condition                                              */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0101 CCCC 11 ea  (byte EA, sets 0xFF or 0x00)
 * Note: DBcc and Scc share the same primary opcode nibble (0x5).
 * DBcc uses mode=001 (An direct), Scc uses all other modes.
 * The dispatch table must distinguish them.
 */
static u32 handler_scc(M68020State *cpu, u16 opword) {
    u8 cc      = (opword >> 8) & 0xFu;
    u8 ea_mode = EA_SRC_MODE(opword);
    u8 ea_reg  = EA_SRC_REG(opword);

    EADesc dst;
    if (!ea_resolve(cpu, ea_mode, ea_reg, SIZE_BYTE, &dst)) return 8;

    u8 val = m68020_test_cc(cpu->SR, cc) ? 0xFFu : 0x00u;
    ea_write(cpu, &dst, SIZE_BYTE, val);

    /* If Dn and condition true: 6 cycles; else 4 */
    return (dst.kind == EAK_Dn && val == 0xFF) ? 6 : 4;
}

/* ------------------------------------------------------------------ */
/* TRAPcc — Trap on Condition (68020)                                  */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0101 CCCC 1111 10xx
 *   xx = 010 (TRAPcc.W — consume one word operand)
 *   xx = 011 (TRAPcc.L — consume one long operand)
 *   xx = 100 (TRAPcc   — no operand)
 *
 * The optional operand is consumed but has no effect on the operation.
 * If the condition is true, raises a TRAPV exception (vector 7).
 */
static u32 handler_trapcc(M68020State *cpu, u16 opword) {
    u8 cc   = (opword >> 8) & 0xFu;
    u8 form = opword & 7u;  /* 2=.W, 3=.L, 4=no operand */

    /* Consume optional operand (ignored functionally) */
    if (form == 2)
        pipeline_consume_word(cpu);
    else if (form == 3) {
        pipeline_consume_word(cpu);
        pipeline_consume_word(cpu);
    }

    if (m68020_test_cc(cpu->SR, cc))
        exception_process(cpu, VEC_TRAPV);

    return 4;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_branch_install_handlers(InsnHandler *t) {
    /* BRA: 0x60xx */
    for (u32 op = 0x6000u; op <= 0x60FFu; op++) t[op] = handler_bra;

    /* BSR: 0x61xx */
    for (u32 op = 0x6100u; op <= 0x61FFu; op++) t[op] = handler_bsr;

    /* Bcc: 0x62xx - 0x6Fxx */
    for (u32 op = 0x6200u; op <= 0x6FFFu; op++) t[op] = handler_bcc;

    /* DBcc: 0101 CCCC 1100 1rrr = 0x50C8..0x5FC8+7 */
    for (u32 cc = 0; cc <= 0xF; cc++) {
        u32 base = 0x50C8u | (cc << 8);
        for (u32 r = 0; r < 8; r++)
            t[base + r] = handler_dbcc;
    }

    /* Scc: 0101 CCCC 11 mmm rrr  — all modes except 001 (An direct = DBcc) */
    for (u32 cc = 0; cc <= 0xF; cc++) {
        u32 base = 0x50C0u | (cc << 8);
        for (u32 ea = 0; ea < 64; ea++) {
            u8 mode = (ea >> 3) & 7u;
            if (mode == EA_MODE_AN) continue;            /* skip DBcc encodings */
            if (mode == EA_MODE_EXT && (ea & 7u) >= 5u) continue; /* no PC-rel/imm dst */
            t[base | ea] = handler_scc;
        }
    }

    /* TRAPcc (68020): 0101 CCCC 1111 10xx (xx=010, 011, 100) */
    for (u32 cc = 0; cc <= 0xF; cc++) {
        t[0x50FAu | (cc << 8)] = handler_trapcc;  /* TRAPcc.W */
        t[0x50FBu | (cc << 8)] = handler_trapcc;  /* TRAPcc.L */
        t[0x50FCu | (cc << 8)] = handler_trapcc;  /* TRAPcc   */
    }
}
