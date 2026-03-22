/*
 * m68020_jump.c — JMP, JSR, RTS, RTR, LINK, UNLK.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* ------------------------------------------------------------------ */
/* JMP — Jump                                                          */
/* ------------------------------------------------------------------ */

/* JMP cycle cost per EA mode (MC68020 User's Manual Table 9-11) */
static u32 jmp_cycles(u8 mode, u8 reg) {
    switch (mode) {
    case EA_MODE_IND:  return 8;   /* (An) */
    case EA_MODE_D16:  return 10;  /* (d16,An) */
    case EA_MODE_IDX:  return 14;  /* (d8,An,Xn) */
    case EA_MODE_EXT:
        switch (reg) {
        case EA_EXT_ABSW:   return 10;  /* (xxx).W */
        case EA_EXT_ABSL:   return 12;  /* (xxx).L */
        case EA_EXT_D16PC:  return 10;  /* (d16,PC) */
        case EA_EXT_IDXPC:  return 14;  /* (d8,PC,Xn) */
        }
        break;
    }
    return 8;
}

static u32 handler_jmp(M68020State *cpu, u16 opword) {
    u8 mode = EA_SRC_MODE(opword);
    u8 reg  = EA_SRC_REG(opword);
    EADesc ea;
    if (!ea_resolve(cpu, mode, reg, SIZE_LONG, &ea))
        return 8;
    if (ea.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 8;
    }
    pipeline_flush(cpu, ea.address);
    cpu->PC = ea.address;
    if (SR_T0(cpu->SR)) cpu->trace_pending = true;
    return jmp_cycles(mode, reg);
}

/* ------------------------------------------------------------------ */
/* JSR — Jump to Subroutine                                            */
/* ------------------------------------------------------------------ */

/* JSR cycle cost = JMP cost + 8 (push return address overhead) */
static u32 handler_jsr(M68020State *cpu, u16 opword) {
    u8 mode = EA_SRC_MODE(opword);
    u8 reg  = EA_SRC_REG(opword);
    EADesc ea;
    if (!ea_resolve(cpu, mode, reg, SIZE_LONG, &ea))
        return 16;
    if (ea.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 16;
    }

    /* Return address = address of the word after JSR (already past EA ext words) */
    u32 return_pc = pipeline_peek_pc(cpu);
    cpu_push_long(cpu, return_pc);

    pipeline_flush(cpu, ea.address);
    cpu->PC = ea.address;
    if (SR_T0(cpu->SR)) cpu->trace_pending = true;
    return jmp_cycles(mode, reg) + 8;
}

/* ------------------------------------------------------------------ */
/* RTS — Return from Subroutine                                        */
/* ------------------------------------------------------------------ */

static u32 handler_rts(M68020State *cpu, u16 opword) {
    (void)opword;
    u32 ret_pc = cpu_pop_long(cpu);
    pipeline_flush(cpu, ret_pc);
    cpu->PC = ret_pc;
    if (SR_T0(cpu->SR)) cpu->trace_pending = true;
    return 16;
}

/* ------------------------------------------------------------------ */
/* RTR — Return and Restore CCR                                        */
/* ------------------------------------------------------------------ */

static u32 handler_rtr(M68020State *cpu, u16 opword) {
    (void)opword;
    u16 ccr    = cpu_pop_word(cpu) & 0x1Fu;
    u32 ret_pc = cpu_pop_long(cpu);
    cpu->SR    = (cpu->SR & 0xFF00u) | ccr;
    pipeline_flush(cpu, ret_pc);
    cpu->PC = ret_pc;
    if (SR_T0(cpu->SR)) cpu->trace_pending = true;
    return 20;
}

/* ------------------------------------------------------------------ */
/* LINK — Link and Allocate                                            */
/* ------------------------------------------------------------------ */
/*
 * LINK.W An,#disp16   (4E50 + n)
 * LINK.L An,#disp32   (4808 + n)  [68020]
 * Saves An to stack, copies SP to An, adds displacement.
 */
static u32 handler_link_w(M68020State *cpu, u16 opword) {
    u8  an   = opword & 7u;
    s16 disp = (s16)pipeline_consume_word(cpu);

    cpu_push_long(cpu, cpu->A[an]);
    cpu->A[an] = cpu->A[7];
    cpu->A[7] += (s32)disp;
    return 16;
}

static u32 handler_link_l(M68020State *cpu, u16 opword) {
    u8  an  = opword & 7u;
    u16 hi  = pipeline_consume_word(cpu);
    u16 lo  = pipeline_consume_word(cpu);
    s32 disp = (s32)(((u32)hi << 16) | lo);

    cpu_push_long(cpu, cpu->A[an]);
    cpu->A[an] = cpu->A[7];
    cpu->A[7] += disp;
    return 20;
}

/* ------------------------------------------------------------------ */
/* UNLK — Unlink                                                       */
/* ------------------------------------------------------------------ */

static u32 handler_unlk(M68020State *cpu, u16 opword) {
    u8 an = opword & 7u;
    cpu->A[7] = cpu->A[an];
    cpu->A[an] = cpu_pop_long(cpu);
    return 12;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_jump_install_handlers(InsnHandler *t) {
    /* JMP: 0x4EC0 | ea (control modes only, but we let illegal handler catch bad EAs) */
    for (u32 ea = 0; ea < 64; ea++) {
        u8 mode = (ea >> 3) & 7u;
        /* JMP valid modes: (An), (d16,An), (d8,An,Xn), (xxx).W, (xxx).L, (d16,PC), (d8,PC,Xn) */
        if (mode == EA_MODE_IND  || mode == EA_MODE_D16 || mode == EA_MODE_IDX ||
            (mode == EA_MODE_EXT && (ea & 7u) <= EA_EXT_IDXPC)) {
            t[OP_JMP_BASE | ea] = handler_jmp;
        }
    }

    /* JSR: 0x4E80 | ea */
    for (u32 ea = 0; ea < 64; ea++) {
        u8 mode = (ea >> 3) & 7u;
        if (mode == EA_MODE_IND  || mode == EA_MODE_D16 || mode == EA_MODE_IDX ||
            (mode == EA_MODE_EXT && (ea & 7u) <= EA_EXT_IDXPC)) {
            t[OP_JSR_BASE | ea] = handler_jsr;
        }
    }

    t[OP_RTS] = handler_rts;
    t[OP_RTR] = handler_rtr;

    /* LINK.W: 0x4E50-0x4E57 */
    for (u32 r = 0; r < 8; r++) t[OP_LINK_W + r] = handler_link_w;

    /* LINK.L (68020): 0x4808-0x480F */
    for (u32 r = 0; r < 8; r++) t[OP_LINK_L + r] = handler_link_l;

    /* UNLK: 0x4E58-0x4E5F */
    for (u32 r = 0; r < 8; r++) t[OP_UNLK + r] = handler_unlk;
}
