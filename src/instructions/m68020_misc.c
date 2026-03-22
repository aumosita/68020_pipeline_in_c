/*
 * m68020_misc.c — NOP, ILLEGAL, TRAP, TRAPV, TAS, TST, NBCD, EXG, LEA, PEA.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* ------------------------------------------------------------------ */
/* NOP                                                                 */
/* ------------------------------------------------------------------ */

static u32 handler_nop(M68020State *cpu, u16 opword) {
    (void)cpu; (void)opword;
    return 4;
}

/* ------------------------------------------------------------------ */
/* ILLEGAL                                                             */
/* ------------------------------------------------------------------ */

static u32 handler_illegal_insn(M68020State *cpu, u16 opword) {
    (void)opword;
    exception_process(cpu, VEC_ILLEGAL_INSN);
    return 4;
}

/* ------------------------------------------------------------------ */
/* TRAP #n                                                             */
/* ------------------------------------------------------------------ */

static u32 handler_trap(M68020State *cpu, u16 opword) {
    u8 n = opword & 0xFu;
    exception_process(cpu, VEC_TRAP_BASE + n);
    return 4;
}

/* ------------------------------------------------------------------ */
/* TRAPV                                                               */
/* ------------------------------------------------------------------ */

static u32 handler_trapv(M68020State *cpu, u16 opword) {
    (void)opword;
    if (SR_V(cpu->SR))
        exception_process(cpu, VEC_TRAPV);
    return 4;
}

/* ------------------------------------------------------------------ */
/* TST — Test                                                          */
/* ------------------------------------------------------------------ */

static BusSize tst_size(u8 ss) {
    static const BusSize sz[4] = { SIZE_BYTE, SIZE_WORD, SIZE_LONG, SIZE_LONG };
    return sz[ss & 3u];
}

static u32 handler_tst(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = tst_size(ss);
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &src)) return 4;
    u32 val = 0;
    if (!ea_read(cpu, &src, sz, &val)) return 4;
    cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(val, sz);
    return 4;
}

/* ------------------------------------------------------------------ */
/* TAS — Test and Set                                                  */
/* ------------------------------------------------------------------ */

static u32 handler_tas(M68020State *cpu, u16 opword) {
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_BYTE, &dst)) return 14;
    u32 val = 0;
    if (!ea_read(cpu, &dst, SIZE_BYTE, &val)) return 14;

    /* CCR update from original value */
    cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(val, SIZE_BYTE);

    /* Set bit 7 */
    val |= 0x80u;
    ea_write(cpu, &dst, SIZE_BYTE, val);
    return 14;
}

/* ------------------------------------------------------------------ */
/* EXG — Exchange Registers                                            */
/* ------------------------------------------------------------------ */

static u32 handler_exg(M68020State *cpu, u16 opword) {
    u8  mode = (opword >> 3) & 0x1Fu;
    u8  rx   = (opword >> 9) & 7u;
    u8  ry   = opword & 7u;
    u32 tmp;

    switch (mode) {
        case 0x08:  /* EXG Dx,Dy */
            tmp = cpu->D[rx]; cpu->D[rx] = cpu->D[ry]; cpu->D[ry] = tmp;
            break;
        case 0x09:  /* EXG Ax,Ay */
            tmp = cpu->A[rx]; cpu->A[rx] = cpu->A[ry]; cpu->A[ry] = tmp;
            break;
        case 0x11:  /* EXG Dx,Ay */
            tmp = cpu->D[rx]; cpu->D[rx] = cpu->A[ry]; cpu->A[ry] = tmp;
            break;
        default:
            exception_process(cpu, VEC_ILLEGAL_INSN);
            return 6;
    }
    return 6;
}

/* ------------------------------------------------------------------ */
/* LEA — Load Effective Address                                        */
/* ------------------------------------------------------------------ */

static u32 handler_lea(M68020State *cpu, u16 opword) {
    u8  an      = (opword >> 9) & 7u;
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    EADesc src;
    if (!ea_resolve(cpu, ea_mode, ea_reg, SIZE_LONG, &src)) return 4;
    if (src.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 4;
    }
    cpu->A[an] = src.address;
    return 4;
}

/* ------------------------------------------------------------------ */
/* PEA — Push Effective Address                                        */
/* ------------------------------------------------------------------ */

static u32 handler_pea(M68020State *cpu, u16 opword) {
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    EADesc src;
    if (!ea_resolve(cpu, ea_mode, ea_reg, SIZE_LONG, &src)) return 12;
    if (src.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 12;
    }
    cpu_push_long(cpu, src.address);
    return 12;
}

/* ------------------------------------------------------------------ */
/* BCD helpers                                                         */
/* ------------------------------------------------------------------ */

static u8 bcd_add_byte(u8 src, u8 dst, u32 x, bool *carry_out) {
    u32 lo = (src & 0xFu) + (dst & 0xFu) + x;
    u32 result = (u32)src + dst + x;
    if (lo > 9u) result += 6u;
    if (result > 0x99u) { result += 0x60u; *carry_out = true; }
    else                                   *carry_out = false;
    return (u8)result;
}

static u8 bcd_sub_byte(u8 src, u8 dst, u32 x, bool *borrow_out) {
    s32 lo = (s32)(dst & 0xFu) - (s32)(src & 0xFu) - (s32)x;
    s32 result = (s32)dst - (s32)src - (s32)x;
    if (lo < 0) result -= 6;
    if (result < 0) { result += 0x100; *borrow_out = true; }
    else                               *borrow_out = false;
    return (u8)(result & 0xFFu);
}

/* Common logic for ABCD / SBCD */
static u32 do_bcd_addsub(M68020State *cpu, u16 opword, bool is_sub) {
    u8 dx = (opword >> 9) & 7u;
    u8 rm = (opword >> 3) & 1u;  /* 0=register, 1=predecrement */
    u8 dy = opword & 7u;
    u32 x = SR_X(cpu->SR);

    u8 src_val, dst_val;
    EADesc dst_ea;
    dst_ea.kind = EAK_Dn;

    if (rm == 0) {
        src_val = (u8)cpu->D[dy];
        dst_val = (u8)cpu->D[dx];
    } else {
        EADesc src_ea;
        if (!ea_resolve(cpu, EA_MODE_PRE, dy, SIZE_BYTE, &src_ea)) return 8;
        u32 tmp = 0;
        if (!ea_read(cpu, &src_ea, SIZE_BYTE, &tmp)) return 8;
        src_val = (u8)tmp;
        if (!ea_resolve(cpu, EA_MODE_PRE, dx, SIZE_BYTE, &dst_ea)) return 8;
        tmp = 0;
        if (!ea_read(cpu, &dst_ea, SIZE_BYTE, &tmp)) return 8;
        dst_val = (u8)tmp;
    }

    bool carry;
    u8 result = is_sub ? bcd_sub_byte(src_val, dst_val, x, &carry)
                       : bcd_add_byte(src_val, dst_val, x, &carry);

    u16 ccr = cpu->SR & ~0x1Fu;
    if (carry)   ccr |= CCR_C | CCR_X;
    if (result & 0x80u) ccr |= CCR_N;
    if (result != 0) ccr &= ~CCR_Z;
    else             ccr |= (cpu->SR & CCR_Z);

    if (rm == 0) {
        cpu->D[dx] = (cpu->D[dx] & 0xFFFFFF00u) | result;
    } else {
        ea_write(cpu, &dst_ea, SIZE_BYTE, result);
    }
    cpu->SR = ccr;
    return 8;
}

static u32 handler_abcd(M68020State *cpu, u16 op) { return do_bcd_addsub(cpu, op, false); }
static u32 handler_sbcd(M68020State *cpu, u16 op) { return do_bcd_addsub(cpu, op, true);  }

/* ------------------------------------------------------------------ */
/* NBCD — Negate BCD                                                   */
/* ------------------------------------------------------------------ */

static u32 handler_nbcd(M68020State *cpu, u16 opword) {
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_BYTE, &dst)) return 8;
    u32 raw = 0;
    if (!ea_read(cpu, &dst, SIZE_BYTE, &raw)) return 8;

    u32 x = SR_X(cpu->SR);
    bool borrow;
    u8 result = bcd_sub_byte((u8)raw, 0, x, &borrow);

    u16 ccr = cpu->SR & ~0x1Fu;
    if (borrow)          ccr |= CCR_C | CCR_X;
    if (result & 0x80u)  ccr |= CCR_N;
    if (result != 0) ccr &= ~CCR_Z;
    else             ccr |= (cpu->SR & CCR_Z);

    ea_write(cpu, &dst, SIZE_BYTE, result);
    cpu->SR = ccr;
    return 8;
}

/* ------------------------------------------------------------------ */
/* PACK / UNPK (68020) — Pack/Unpack BCD                              */
/* ------------------------------------------------------------------ */

static u32 handler_pack(M68020State *cpu, u16 opword) {
    u8 dx = (opword >> 9) & 7u;
    u8 rm = (opword >> 3) & 1u;
    u8 dy = opword & 7u;
    u16 adj = pipeline_consume_word(cpu);

    if (rm == 0) {
        /* PACK Dy,Dx,#adj — register form */
        u16 tmp = (u16)(cpu->D[dy] & 0xFFFFu) + adj;
        u8  packed = (u8)(((tmp >> 4) & 0xF0u) | (tmp & 0x0Fu));
        cpu->D[dx] = (cpu->D[dx] & 0xFFFFFF00u) | packed;
    } else {
        /* PACK -(Ay),-(Ax),#adj — predecrement form */
        /* Read 2 bytes from -(Ay) into a 16-bit word */
        u32 byte_hi = 0, byte_lo = 0;
        cpu->A[dy] -= 1;
        cpu_read_byte(cpu, cpu->A[dy], (u8*)&byte_lo);
        cpu->A[dy] -= 1;
        cpu_read_byte(cpu, cpu->A[dy], (u8*)&byte_hi);
        u16 tmp = (u16)((byte_hi << 8) | byte_lo) + adj;
        u8  packed = (u8)(((tmp >> 4) & 0xF0u) | (tmp & 0x0Fu));
        cpu->A[dx] -= 1;
        cpu_write_byte(cpu, cpu->A[dx], packed);
    }
    return 8;
}

static u32 handler_unpk(M68020State *cpu, u16 opword) {
    u8 dx = (opword >> 9) & 7u;
    u8 rm = (opword >> 3) & 1u;
    u8 dy = opword & 7u;
    u16 adj = pipeline_consume_word(cpu);

    if (rm == 0) {
        /* UNPK Dy,Dx,#adj — register form */
        u8  src = (u8)cpu->D[dy];
        u16 tmp = (u16)((((u16)src & 0xF0u) << 4) | (src & 0x0Fu)) + adj;
        cpu->D[dx] = (cpu->D[dx] & 0xFFFF0000u) | tmp;
    } else {
        /* UNPK -(Ay),-(Ax),#adj — predecrement form */
        u32 byte_val = 0;
        cpu->A[dy] -= 1;
        cpu_read_byte(cpu, cpu->A[dy], (u8*)&byte_val);
        u8  src = (u8)byte_val;
        u16 tmp = (u16)((((u16)src & 0xF0u) << 4) | (src & 0x0Fu)) + adj;
        cpu->A[dx] -= 2;
        cpu_write_byte(cpu, cpu->A[dx],     (u8)(tmp >> 8));
        cpu_write_byte(cpu, cpu->A[dx] + 1, (u8)tmp);
    }
    return 8;
}

/* ------------------------------------------------------------------ */
/* BKPT — Breakpoint                                                   */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0100 1000 0100 1 vvv  (0x4848 | vector)
 * On the real 68020 this triggers a breakpoint bus cycle; external
 * hardware responds with an instruction word to execute.
 * In emulation: raise an illegal instruction exception (vector 4),
 * which the debugger can intercept via the trace hook.
 */
static u32 handler_bkpt(M68020State *cpu, u16 opword) {
    (void)opword;
    exception_process(cpu, VEC_ILLEGAL_INSN);
    return 4;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_misc_install_handlers(InsnHandler *t) {
    t[OP_NOP]     = handler_nop;
    t[OP_ILLEGAL] = handler_illegal_insn;
    t[OP_TRAPV]   = handler_trapv;

    /* TRAP #0-#15: 0100 1110 0100 nnnn */
    for (u32 n = 0; n < 16; n++)
        t[0x4E40u | n] = handler_trap;

    /* TST: 0100 1010 ss ea (ss=00/01/10) */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x4A00u | (ss << 6) | ea] = handler_tst;

    /* TAS: 0100 1010 11 ea */
    for (u32 ea = 0; ea < 64; ea++)
        t[0x4AC0u | ea] = handler_tas;

    /* EXG: 1100 DDD1 OPMODE rrr
     * Valid OPMODE: 01000=Dx,Dy  01001=Ax,Ay  10001=Dx,Ay
     * Only install for these specific opmode values to avoid
     * clobbering MULS.W (which shares the 1100 DDD1 prefix). */
    for (u32 dx = 0; dx < 8; dx++) {
        for (u32 ry = 0; ry < 8; ry++) {
            u32 base = 0xC100u | (dx << 9) | ry;
            t[base | (0x08u << 3)] = handler_exg;  /* 01000: Dx,Dy */
            t[base | (0x09u << 3)] = handler_exg;  /* 01001: Ax,Ay */
            t[base | (0x11u << 3)] = handler_exg;  /* 10001: Dx,Ay */
        }
    }

    /* LEA: 0100 DDD1 11 ea (control modes) */
    for (u32 an = 0; an < 8; an++) {
        u32 base = 0x41C0u | (an << 9);
        for (u32 ea = 0; ea < 64; ea++) {
            u8 mode = (ea >> 3) & 7u;
            if (mode == EA_MODE_DN || mode == EA_MODE_AN ||
                mode == EA_MODE_POST || mode == EA_MODE_PRE) continue;
            if (mode == EA_MODE_EXT && (ea & 7u) == EA_EXT_IMM) continue;
            t[base | ea] = handler_lea;
        }
    }

    /* PEA: 0100 1000 01 ea (control modes) */
    for (u32 ea = 0; ea < 64; ea++) {
        u8 mode = (ea >> 3) & 7u;
        if (mode == EA_MODE_DN || mode == EA_MODE_AN ||
            mode == EA_MODE_POST || mode == EA_MODE_PRE) continue;
        if (mode == EA_MODE_EXT && (ea & 7u) == EA_EXT_IMM) continue;
        t[0x4840u | ea] = handler_pea;
    }

    /* BKPT: 0100 1000 0100 1 vvv (0x4848-0x484F) */
    for (u32 v = 0; v < 8; v++)
        t[0x4848u | v] = handler_bkpt;

    /* NBCD: 0100 1000 00 ea */
    for (u32 ea = 0; ea < 64; ea++)
        t[0x4800u | ea] = handler_nbcd;

    /* ABCD: 1100 Dx 1 00 00 M Dy — overwrite slots installed by AND handler */
    for (u32 dx = 0; dx < 8; dx++)
        for (u32 dy = 0; dy < 8; dy++) {
            t[0xC100u | (dx << 9) | dy]        = handler_abcd;  /* M=0 Dn,Dn */
            t[0xC100u | (dx << 9) | 0x08u | dy] = handler_abcd;  /* M=1 -(An),-(An) */
        }

    /* SBCD: 1000 Dx 1 00 00 M Dy — overwrite slots installed by OR handler */
    for (u32 dx = 0; dx < 8; dx++)
        for (u32 dy = 0; dy < 8; dy++) {
            t[0x8100u | (dx << 9) | dy]        = handler_sbcd;
            t[0x8100u | (dx << 9) | 0x08u | dy] = handler_sbcd;
        }

    /* PACK: 1000 Dx 1 01 00 M Dy — overwrite OR/DIVU slots */
    for (u32 dx = 0; dx < 8; dx++)
        for (u32 dy = 0; dy < 8; dy++) {
            t[0x8140u | (dx << 9) | dy]        = handler_pack;
            t[0x8140u | (dx << 9) | 0x08u | dy] = handler_pack;
        }

    /* UNPK: 1000 Dx 1 10 00 M Dy — overwrite OR/DIVU slots */
    for (u32 dx = 0; dx < 8; dx++)
        for (u32 dy = 0; dy < 8; dy++) {
            t[0x8180u | (dx << 9) | dy]        = handler_unpk;
            t[0x8180u | (dx << 9) | 0x08u | dy] = handler_unpk;
        }
}
