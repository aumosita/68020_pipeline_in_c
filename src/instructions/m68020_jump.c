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

static u32 handler_jmp(M68020State *cpu, u16 opword) {
    EADesc ea;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_LONG, &ea))
        return 8;
    if (ea.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 8;
    }
    pipeline_flush(cpu, ea.address);
    cpu->PC = ea.address;
    return 8;
}

/* ------------------------------------------------------------------ */
/* JSR — Jump to Subroutine                                            */
/* ------------------------------------------------------------------ */

static u32 handler_jsr(M68020State *cpu, u16 opword) {
    EADesc ea;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_LONG, &ea))
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
    return 16;
}

/* ------------------------------------------------------------------ */
/* RTS — Return from Subroutine                                        */
/* ------------------------------------------------------------------ */

static u32 handler_rts(M68020State *cpu, u16 opword) {
    (void)opword;
    u32 ret_pc = cpu_pop_long(cpu);
    pipeline_flush(cpu, ret_pc);
    cpu->PC = ret_pc;
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
