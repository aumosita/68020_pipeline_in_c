/*
 * m68020_compare.c — CMP, CMPA, CMPI, CMPM, CHK, CHK2, CMP2.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

static BusSize cmp_size(u8 ss) {
    static const BusSize sz[4] = { SIZE_BYTE, SIZE_WORD, SIZE_LONG, SIZE_LONG };
    return sz[ss & 3u];
}

/* ------------------------------------------------------------------ */
/* CMP <ea>,Dn                                                         */
/* ------------------------------------------------------------------ */

static u32 handler_cmp(M68020State *cpu, u16 opword) {
    u8  dn      = (opword >> 9) & 7u;
    u8  ss      = (opword >> 6) & 3u;
    BusSize sz  = cmp_size(ss);
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &src)) return 6;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, sz, &src_val)) return 6;
    u32 dst_val = cpu->D[dn] & sz_mask(sz);
    u32 result  = dst_val - src_val;
    /* CMP: update N,Z,V,C but NOT X */
    u16 ccr = ccr_sub(src_val, dst_val, result, sz);
    cpu->SR = (cpu->SR & ~0x0Fu) | (ccr & 0x0Fu);
    return 6;
}

/* ------------------------------------------------------------------ */
/* CMPA — Compare Address                                              */
/* ------------------------------------------------------------------ */

static u32 handler_cmpa(M68020State *cpu, u16 opword) {
    u8  an     = (opword >> 9) & 7u;
    BusSize sz = (opword & 0x0100u) ? SIZE_LONG : SIZE_WORD;
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &src)) return 6;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, sz, &src_val)) return 6;
    if (sz == SIZE_WORD) src_val = (u32)(s32)(s16)(u16)src_val;
    u32 dst_val = cpu->A[an];
    u32 result  = dst_val - src_val;
    u16 ccr = ccr_sub(src_val, dst_val, result, SIZE_LONG);
    cpu->SR = (cpu->SR & ~0x0Fu) | (ccr & 0x0Fu);
    return 6;
}

/* ------------------------------------------------------------------ */
/* CMPI — Compare Immediate                                            */
/* ------------------------------------------------------------------ */

static u32 handler_cmpi(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = cmp_size(ss);

    u32 imm = 0;
    if (sz == SIZE_BYTE)      imm = (u8)pipeline_consume_word(cpu);
    else if (sz == SIZE_WORD) imm = pipeline_consume_word(cpu);
    else { u16 h = pipeline_consume_word(cpu); u16 l = pipeline_consume_word(cpu); imm = ((u32)h<<16)|l; }

    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst)) return 8;
    u32 dst_val = 0;
    if (!ea_read(cpu, &dst, sz, &dst_val)) return 8;

    u32 result = dst_val - imm;
    u16 ccr = ccr_sub(imm, dst_val, result, sz);
    cpu->SR = (cpu->SR & ~0x0Fu) | (ccr & 0x0Fu);
    return 8;
}

/* ------------------------------------------------------------------ */
/* CMPM — Compare Memory to Memory: (An)+,(An)+                       */
/* ------------------------------------------------------------------ */

static u32 handler_cmpm(M68020State *cpu, u16 opword) {
    u8  ax  = (opword >> 9) & 7u;
    u8  ay  = opword & 7u;
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = cmp_size(ss);

    /* CMPM (Ay)+,(Ax)+ — post-increment both address registers */
    u32 src_val = 0, dst_val = 0;
    EADesc src, dst_ea;
    src.kind = EAK_Mem; src.address = cpu->A[ay]; src.is_postinc = true;
    src.postinc_reg = ay;
    src.postinc_amt = (ay == 7 && sz == SIZE_BYTE) ? 2u : (u32)sz;
    dst_ea.kind = EAK_Mem; dst_ea.address = cpu->A[ax]; dst_ea.is_postinc = true;
    dst_ea.postinc_reg = ax;
    dst_ea.postinc_amt = (ax == 7 && sz == SIZE_BYTE) ? 2u : (u32)sz;

    if (!ea_read(cpu, &src, sz, &src_val)) return 12;
    if (!ea_read(cpu, &dst_ea, sz, &dst_val)) return 12;

    u32 result = dst_val - src_val;
    u16 ccr = ccr_sub(src_val, dst_val, result, sz);
    cpu->SR = (cpu->SR & ~0x0Fu) | (ccr & 0x0Fu);
    return 12;
}

/* ------------------------------------------------------------------ */
/* CHK — Check Register Against Bounds                                 */
/* ------------------------------------------------------------------ */

static u32 handler_chk(M68020State *cpu, u16 opword) {
    u8  dn  = (opword >> 9) & 7u;
    BusSize sz = (opword & 0x0080u) ? SIZE_WORD : SIZE_LONG;
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &src)) return 10;
    u32 bound = 0;
    if (!ea_read(cpu, &src, sz, &bound)) return 10;

    s32 val = (sz == SIZE_WORD) ? (s32)(s16)(u16)cpu->D[dn] : (s32)cpu->D[dn];
    s32 bnd = (sz == SIZE_WORD) ? (s32)(s16)(u16)bound : (s32)bound;

    if (val < 0) {
        cpu->SR |= CCR_N;
        exception_process(cpu, VEC_CHK);
    } else if (val > bnd) {
        cpu->SR &= ~CCR_N;
        exception_process(cpu, VEC_CHK);
    }
    return 10;
}

/* ------------------------------------------------------------------ */
/* CHK2/CMP2 — Compare Register Against Bounds (68020)                 */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0000 0ss0 11 ea  (ss: 00=byte, 01=word, 10=long)
 *   Extension word:
 *     bit 15:    0=Dn, 1=An
 *     bits 14:12: register number
 *     bit 11:    0=CMP2, 1=CHK2
 *
 * Lower bound at EA, upper bound at EA + operand_size.
 * Comparison for An: signed 32-bit (bounds sign-extended from .B/.W)
 * Comparison for Dn: unsigned, masked to operand size.
 *
 * Z=1 if Rn equals either bound.
 * C=1 if Rn is out of range (< lower or > upper).
 * CHK2: raises VEC_CHK if C=1.
 */
static u32 handler_chk2_cmp2(M68020State *cpu, u16 opword) {
    u8  ss = (opword >> 9) & 3u;
    BusSize sz = (ss == 0) ? SIZE_BYTE : (ss == 1) ? SIZE_WORD : SIZE_LONG;

    u16 ext    = pipeline_consume_word(cpu);
    u8  is_an  = (ext >> 15) & 1u;
    u8  rn     = (ext >> 12) & 7u;
    u8  is_chk = (ext >> 11) & 1u;  /* 0=CMP2, 1=CHK2 */

    EADesc ea;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &ea)) return 8;
    if (ea.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 8;
    }

    /* Read lower bound, then upper bound at EA + operand_size bytes */
    u32 lower = 0, upper = 0;
    if (!ea_read(cpu, &ea, sz, &lower)) return 8;
    EADesc upper_ea = ea;
    upper_ea.address += (u32)sz;
    upper_ea.is_postinc = false;
    if (!ea_read(cpu, &upper_ea, sz, &upper)) return 8;

    bool z_flag, c_flag;

    if (is_an) {
        /* Address register: sign-extend bounds to 32-bit, compare signed */
        s32 rval, lo, hi;
        u32 aval = cpu->A[rn];
        if (sz == SIZE_BYTE) {
            rval = (s32)(s8)(u8)aval;
            lo   = (s32)(s8)(u8)lower;
            hi   = (s32)(s8)(u8)upper;
        } else if (sz == SIZE_WORD) {
            rval = (s32)(s16)(u16)aval;
            lo   = (s32)(s16)(u16)lower;
            hi   = (s32)(s16)(u16)upper;
        } else {
            rval = (s32)aval;
            lo   = (s32)lower;
            hi   = (s32)upper;
        }
        z_flag = (rval == lo || rval == hi);
        c_flag = (rval < lo || rval > hi);
    } else {
        /* Data register: unsigned comparison, masked to operand size */
        u32 m    = sz_mask(sz);
        u32 rval = cpu->D[rn] & m;
        lower &= m;
        upper &= m;
        z_flag = (rval == lower || rval == upper);
        c_flag = (rval < lower || rval > upper);
    }

    cpu->SR &= ~(CCR_Z | CCR_C);
    if (z_flag) cpu->SR |= CCR_Z;
    if (c_flag) cpu->SR |= CCR_C;

    if (is_chk && c_flag)
        exception_process(cpu, VEC_CHK);

    return 8;
}

void m68020_compare_install_handlers(InsnHandler *t) {
    /* CMP: 1011 DDD0 ss ea */
    for (u32 op = 0xB000u; op <= 0xBFFFu; op++) {
        u8 dir = (op >> 8) & 1u;
        u8 ss  = (op >> 6) & 3u;
        if (dir == 0 && ss != 3) t[op] = handler_cmp;
        /* CMPA: dir=1, ss=010 (.W) or ss=110 (.L) */
        if (dir == 0 && ss == 3) { /* CMPA.W */ t[op] = handler_cmpa; }
    }
    /* CMPA.W: 1011 DDD0 11 ea */
    for (u32 an = 0; an < 8; an++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0xB0C0u | (an << 9) | ea] = handler_cmpa;
    /* CMPA.L: 1011 DDD1 11 ea */
    for (u32 an = 0; an < 8; an++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0xB1C0u | (an << 9) | ea] = handler_cmpa;

    /* CMPI: 0000 1100 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0C00u | (ss << 6) | ea] = handler_cmpi;

    /* CMPM: 1011 XXX1 ss 001 YYY */
    for (u32 ax = 0; ax < 8; ax++)
        for (u32 ay = 0; ay < 8; ay++)
            for (u32 ss = 0; ss <= 2; ss++)
                t[0xB108u | (ax << 9) | (ss << 6) | ay] = handler_cmpm;

    /* CHK: 0100 DDD1 10 ea (.W) and 0100 DDD1 00 ea (.L, 68020) */
    for (u32 dn = 0; dn < 8; dn++)
        for (u32 ea = 0; ea < 64; ea++) {
            t[0x4180u | (dn << 9) | ea] = handler_chk;  /* CHK.W */
            t[0x4100u | (dn << 9) | ea] = handler_chk;  /* CHK.L (68020) */
        }

    /* CHK2/CMP2 (68020): 0000 0ss0 11 ea
     *   ss=00 (byte): 0x00C0 | ea
     *   ss=01 (word): 0x02C0 | ea
     *   ss=10 (long): 0x04C0 | ea
     * Valid EA modes: control alterable (no Dn/An/post/pre/imm)
     */
    for (u32 ss = 0; ss <= 2; ss++) {
        u32 base = (ss << 9) | 0x00C0u;
        for (u32 ea = 0; ea < 64; ea++) {
            u8 mode = (ea >> 3) & 7u;
            /* Skip Dn, An, (An)+, -(An), immediate */
            if (mode == EA_MODE_DN || mode == EA_MODE_AN ||
                mode == EA_MODE_POST || mode == EA_MODE_PRE) continue;
            if (mode == EA_MODE_EXT && (ea & 7u) == EA_EXT_IMM) continue;
            t[base | ea] = handler_chk2_cmp2;
        }
    }
}
