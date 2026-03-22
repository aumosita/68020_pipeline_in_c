/*
 * m68020_alu.c — ADD, ADDA, ADDI, ADDQ, ADDX,
 *                SUB, SUBA, SUBI, SUBQ, SUBX,
 *                AND, ANDI, OR, ORI, EOR, EORI,
 *                NOT, NEG, NEGX, CLR, EXT, EXTB.
 *
 * Phase 1: ADDQ, SUBQ, ADD <ea>,Dn, SUB <ea>,Dn, AND, OR, EOR,
 *          NOT, NEG, CLR, CMP (basic forms) are fully implemented.
 *          Remaining forms (ADDX, SUBX, ADDA, SUBA, memory destinations)
 *          will be added in Phase 2.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* ------------------------------------------------------------------ */
/* ADDQ / SUBQ — Add/Subtract Quick                                    */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0101 DDD0 ss ea  (ADD) / 0101 DDD1 ss ea (SUB)
 *   DDD = immediate data (1-8, where 000 encodes 8)
 *   ss  = size: 00=byte, 01=word, 10=long
 *
 * ADDQ/SUBQ to An: size is always long, CCR not affected.
 */

static BusSize addq_size(u8 ss) {
    static const BusSize sz[4] = { SIZE_BYTE, SIZE_WORD, SIZE_LONG, SIZE_LONG };
    return sz[ss & 3u];
}

static u32 do_addq(M68020State *cpu, u16 opword, bool is_sub) {
    u8 imm8    = (opword >> 9) & 7u;
    u32 imm    = imm8 ? imm8 : 8u;   /* 000 = 8 */
    u8  ss     = (opword >> 6) & 3u;
    BusSize sz = addq_size(ss);
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    EADesc dst;
    if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &dst)) return 8;

    if (dst.kind == EAK_An) {
        /* Address register: always .L, no CCR update */
        u32 old_val = cpu->A[ea_reg];
        cpu->A[ea_reg] = is_sub ? old_val - imm : old_val + imm;
        return 8;
    }

    u32 old_val = 0;
    if (!ea_read(cpu, &dst, sz, &old_val)) return 8;

    u32 result;
    u16 ccr;
    if (is_sub) {
        result = old_val - imm;
        ccr    = ccr_sub(imm, old_val, result, sz);
    } else {
        result = old_val + imm;
        ccr    = ccr_add(imm, old_val, result, sz);
    }

    if (!ea_write(cpu, &dst, sz, result)) return 8;

    /* Preserve X flag bit pattern: X = C */
    u16 new_x = (ccr & CCR_C) ? CCR_X : 0;
    ccr = (ccr & ~CCR_X) | new_x;
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;

    return 8;
}

static u32 handler_addq(M68020State *cpu, u16 opword) { return do_addq(cpu, opword, false); }
static u32 handler_subq(M68020State *cpu, u16 opword) { return do_addq(cpu, opword, true);  }

/* ------------------------------------------------------------------ */
/* ADD / SUB — <ea>,Dn and Dn,<ea>                                    */
/* ------------------------------------------------------------------ */

static u32 do_add_sub(M68020State *cpu, u16 opword, bool is_sub) {
    u8  dn      = (opword >> 9) & 7u;
    u8  dir     = (opword >> 8) & 1u;  /* 0=ea→Dn, 1=Dn→ea */
    u8  ss      = (opword >> 6) & 3u;
    BusSize sz  = addq_size(ss);
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    if (dir == 0) {
        /* <ea> → Dn */
        EADesc src;
        if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &src)) return 8;
        u32 src_val = 0;
        if (!ea_read(cpu, &src, sz, &src_val)) return 8;

        u32 dst_val = cpu->D[dn] & sz_mask(sz);
        u32 result  = is_sub ? dst_val - src_val : dst_val + src_val;
        u16 ccr     = is_sub ? ccr_sub(src_val, dst_val, result, sz)
                             : ccr_add(src_val, dst_val, result, sz);

        /* Write only the appropriate bytes of Dn */
        switch (sz) {
            case SIZE_BYTE: cpu->D[dn] = (cpu->D[dn] & 0xFFFFFF00u) | (result & 0xFFu); break;
            case SIZE_WORD: cpu->D[dn] = (cpu->D[dn] & 0xFFFF0000u) | (result & 0xFFFFu); break;
            case SIZE_LONG: cpu->D[dn] = result; break;
        }
        cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    } else {
        /* Dn → <ea> */
        EADesc dst;
        if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &dst)) return 8;
        u32 dst_val = 0;
        if (!ea_read(cpu, &dst, sz, &dst_val)) return 8;

        u32 src_val = cpu->D[dn] & sz_mask(sz);
        u32 result  = is_sub ? dst_val - src_val : dst_val + src_val;
        u16 ccr     = is_sub ? ccr_sub(src_val, dst_val, result, sz)
                             : ccr_add(src_val, dst_val, result, sz);

        if (!ea_write(cpu, &dst, sz, result)) return 8;
        cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    }
    return 8;
}

static u32 handler_add(M68020State *cpu, u16 opword) { return do_add_sub(cpu, opword, false); }
static u32 handler_sub(M68020State *cpu, u16 opword) { return do_add_sub(cpu, opword, true);  }

/* ------------------------------------------------------------------ */
/* ADDA / SUBA — Add/Sub Address                                       */
/* ------------------------------------------------------------------ */

static u32 do_adda_suba(M68020State *cpu, u16 opword, bool is_sub) {
    u8  an      = (opword >> 9) & 7u;
    BusSize sz  = (opword & 0x0100u) ? SIZE_LONG : SIZE_WORD;
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    EADesc src;
    if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &src)) return 8;
    u32 val = 0;
    if (!ea_read(cpu, &src, sz, &val)) return 8;

    if (sz == SIZE_WORD) val = (u32)(s32)(s16)(u16)val; /* sign-extend */

    cpu->A[an] = is_sub ? cpu->A[an] - val : cpu->A[an] + val;
    /* ADDA/SUBA do not affect CCR */
    return 8;
}

static u32 handler_adda(M68020State *cpu, u16 opword) { return do_adda_suba(cpu, opword, false); }
static u32 handler_suba(M68020State *cpu, u16 opword) { return do_adda_suba(cpu, opword, true);  }

/* ------------------------------------------------------------------ */
/* ADDI / SUBI / ANDI / ORI / EORI — Immediate to EA                 */
/* ------------------------------------------------------------------ */

static u32 do_imm_op(M68020State *cpu, u16 opword, int op) {
    /* op: 0=OR, 1=AND, 5=EOR, 6=ADD, 7=SUB */
    u8  ss      = (opword >> 6) & 3u;
    BusSize sz  = addq_size(ss);
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    /* Read immediate value */
    u32 imm = 0;
    if (sz == SIZE_BYTE) {
        imm = (u8)pipeline_consume_word(cpu);
    } else if (sz == SIZE_WORD) {
        imm = pipeline_consume_word(cpu);
    } else {
        u16 hi = pipeline_consume_word(cpu);
        u16 lo = pipeline_consume_word(cpu);
        imm = ((u32)hi << 16) | lo;
    }

    EADesc dst;
    if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &dst)) return 8;
    u32 old_val = 0;
    if (!ea_read(cpu, &dst, sz, &old_val)) return 8;

    u32 result;
    u16 ccr;
    switch (op) {
        case 0: result = old_val | imm;  ccr = ccr_logic(result, sz); break;
        case 1: result = old_val & imm;  ccr = ccr_logic(result, sz); break;
        case 5: result = old_val ^ imm;  ccr = ccr_logic(result, sz); break;
        case 6: result = old_val + imm;  ccr = ccr_add(imm, old_val, result, sz); break;
        case 7: result = old_val - imm;  ccr = ccr_sub(imm, old_val, result, sz); break;
        default: result = 0; ccr = 0;
    }

    if (!ea_write(cpu, &dst, sz, result)) return 8;
    /* ADDI/SUBI (ops 6,7) change X; ORI/ANDI/EORI (ops 0,1,5) preserve X */
    u16 mask = (op == 6 || op == 7) ? ~0x1Fu : ~0x0Fu;
    cpu->SR = (cpu->SR & mask) | ccr;
    return 12;
}

static u32 handler_ori (M68020State *cpu, u16 op) { return do_imm_op(cpu, op, 0); }
static u32 handler_andi(M68020State *cpu, u16 op) { return do_imm_op(cpu, op, 1); }
static u32 handler_eori(M68020State *cpu, u16 op) { return do_imm_op(cpu, op, 5); }
static u32 handler_addi(M68020State *cpu, u16 op) { return do_imm_op(cpu, op, 6); }
static u32 handler_subi(M68020State *cpu, u16 op) { return do_imm_op(cpu, op, 7); }

/* Special: ANDI/ORI/EORI to SR/CCR */
static u32 handler_ori_to_ccr(M68020State *cpu, u16 op) {
    (void)op;
    u16 imm = pipeline_consume_word(cpu);
    cpu->SR |= imm & 0x1Fu;
    return 20;
}
static u32 handler_ori_to_sr(M68020State *cpu, u16 op) {
    (void)op;
    if (!SR_S(cpu->SR)) { exception_process(cpu, VEC_PRIVILEGE); return 20; }
    u16 imm = pipeline_consume_word(cpu);
    cpu_set_sr(cpu, cpu->SR | imm);
    return 20;
}
static u32 handler_andi_to_ccr(M68020State *cpu, u16 op) {
    (void)op;
    u16 imm = pipeline_consume_word(cpu);
    cpu->SR = (cpu->SR & ~0x1Fu) | (cpu->SR & imm & 0x1Fu);
    return 20;
}
static u32 handler_andi_to_sr(M68020State *cpu, u16 op) {
    (void)op;
    if (!SR_S(cpu->SR)) { exception_process(cpu, VEC_PRIVILEGE); return 20; }
    u16 imm = pipeline_consume_word(cpu);
    cpu_set_sr(cpu, cpu->SR & imm);
    return 20;
}
static u32 handler_eori_to_ccr(M68020State *cpu, u16 op) {
    (void)op;
    u16 imm = pipeline_consume_word(cpu);
    cpu->SR ^= imm & 0x1Fu;
    return 20;
}
static u32 handler_eori_to_sr(M68020State *cpu, u16 op) {
    (void)op;
    if (!SR_S(cpu->SR)) { exception_process(cpu, VEC_PRIVILEGE); return 20; }
    u16 imm = pipeline_consume_word(cpu);
    cpu_set_sr(cpu, cpu->SR ^ imm);
    return 20;
}

/* ------------------------------------------------------------------ */
/* AND / OR / EOR — register and memory forms                         */
/* ------------------------------------------------------------------ */

static u32 do_logic(M68020State *cpu, u16 opword, u8 op) {
    u8  dn      = (opword >> 9) & 7u;
    u8  dir     = (opword >> 8) & 1u;
    u8  ss      = (opword >> 6) & 3u;
    BusSize sz  = addq_size(ss);
    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    if (dir == 0) {
        EADesc src;
        if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &src)) return 8;
        u32 src_val = 0;
        if (!ea_read(cpu, &src, sz, &src_val)) return 8;
        u32 dst_val = cpu->D[dn] & sz_mask(sz);
        u32 result = (op == 0) ? (dst_val | src_val) :
                     (op == 1) ? (dst_val & src_val) :
                                 (dst_val ^ src_val);
        switch (sz) {
            case SIZE_BYTE: cpu->D[dn] = (cpu->D[dn] & 0xFFFFFF00u) | (result & 0xFFu); break;
            case SIZE_WORD: cpu->D[dn] = (cpu->D[dn] & 0xFFFF0000u) | (result & 0xFFFFu); break;
            case SIZE_LONG: cpu->D[dn] = result; break;
        }
        cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(result, sz);
    } else {
        EADesc dst;
        if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &dst)) return 8;
        u32 dst_val = 0;
        if (!ea_read(cpu, &dst, sz, &dst_val)) return 8;
        u32 src_val = cpu->D[dn] & sz_mask(sz);
        u32 result = (op == 0) ? (dst_val | src_val) :
                     (op == 1) ? (dst_val & src_val) :
                                 (dst_val ^ src_val);
        if (!ea_write(cpu, &dst, sz, result)) return 8;
        cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(result, sz);
    }
    return 8;
}

static u32 handler_or (M68020State *cpu, u16 op) { return do_logic(cpu, op, 0); }
static u32 handler_and(M68020State *cpu, u16 op) { return do_logic(cpu, op, 1); }
static u32 handler_eor(M68020State *cpu, u16 op) { return do_logic(cpu, op, 2); }

/* ------------------------------------------------------------------ */
/* NOT / NEG / NEGX / CLR                                             */
/* ------------------------------------------------------------------ */

static u32 handler_not(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = addq_size(ss);
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst)) return 8;
    u32 val = 0;
    if (!ea_read(cpu, &dst, sz, &val)) return 8;
    u32 result = ~val;
    if (!ea_write(cpu, &dst, sz, result)) return 8;
    cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(result, sz);
    return 8;
}

static u32 handler_neg(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = addq_size(ss);
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst)) return 8;
    u32 val = 0;
    if (!ea_read(cpu, &dst, sz, &val)) return 8;
    u32 result = 0u - val;
    if (!ea_write(cpu, &dst, sz, result)) return 8;
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr_sub(val, 0, result, sz);
    return 8;
}

static u32 handler_negx(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = addq_size(ss);
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst)) return 8;
    u32 val = 0;
    if (!ea_read(cpu, &dst, sz, &val)) return 8;
    u32 x = SR_X(cpu->SR);
    u32 result = 0u - val - x;
    /* NEGX: Z is only cleared, never set (set if both zero) */
    u16 ccr = ccr_sub(val + x, 0, result, sz);
    if (result & sz_mask(sz))
        ccr &= ~CCR_Z;
    else
        ccr |= (cpu->SR & CCR_Z);  /* preserve Z if result is 0 */
    if (!ea_write(cpu, &dst, sz, result)) return 8;
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    return 8;
}

static u32 handler_clr(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 6) & 3u;
    BusSize sz = addq_size(ss);
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst)) return 8;
    if (!ea_write(cpu, &dst, sz, 0)) return 8;
    /* CLR: N=0, Z=1, V=0, C=0 */
    cpu->SR = (cpu->SR & ~0x0Fu) | CCR_Z;
    return 8;
}

/* ------------------------------------------------------------------ */
/* EXT / EXTB                                                          */
/* ------------------------------------------------------------------ */

static u32 handler_ext(M68020State *cpu, u16 opword) {
    u8 dn = opword & 7u;
    u32 result;

    if (opword & 0x0040u) {
        /* EXT.L: sign-extend word to long */
        result = (u32)(s32)(s16)(u16)cpu->D[dn];
    } else {
        /* EXT.W: sign-extend byte to word */
        u16 w = (u16)(s16)(s8)(u8)cpu->D[dn];
        result = (cpu->D[dn] & 0xFFFF0000u) | w;
    }
    cpu->D[dn] = result;
    cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(result, SIZE_LONG);
    return 4;
}

static u32 handler_extb(M68020State *cpu, u16 opword) {
    /* EXTB.L (68020): sign-extend byte to long */
    u8 dn = opword & 7u;
    u32 result = (u32)(s32)(s8)(u8)cpu->D[dn];
    cpu->D[dn] = result;
    cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(result, SIZE_LONG);
    return 4;
}

/* ------------------------------------------------------------------ */
/* SWAP                                                                */
/* ------------------------------------------------------------------ */

static u32 handler_swap(M68020State *cpu, u16 opword) {
    u8 dn = opword & 7u;
    u32 val = cpu->D[dn];
    cpu->D[dn] = (val << 16) | (val >> 16);
    cpu->SR = (cpu->SR & ~0x0Fu) | ccr_logic(cpu->D[dn], SIZE_LONG);
    return 4;
}

/* ------------------------------------------------------------------ */
/* ADDX / SUBX — Add/Subtract Extended (with X flag)                  */
/* ------------------------------------------------------------------ */
/*
 * Encoding:  1101 Dx 1 ss 00 M Dy  (ADDX)
 *            1001 Dx 1 ss 00 M Dy  (SUBX)
 *   ss: 00=byte 01=word 10=long
 *   M=0: Dy,Dx register form
 *   M=1: -(Ay),-(Ax) predecrement form
 *
 * Z flag: only cleared (never set) — for multi-precision arithmetic.
 */

static u32 do_addx_subx(M68020State *cpu, u16 opword, bool is_sub) {
    u8 dx = (opword >> 9) & 7u;
    u8 ss = (opword >> 6) & 3u;
    u8 rm = (opword >> 3) & 1u;   /* 0=register, 1=predecrement */
    u8 dy = opword & 7u;
    BusSize sz = (ss == 0) ? SIZE_BYTE : (ss == 1) ? SIZE_WORD : SIZE_LONG;
    u32 m = sz_mask(sz);
    u32 s = sz_sign(sz);
    u32 x = SR_X(cpu->SR);

    u32 src_val = 0, dst_val = 0;
    EADesc dst_ea;
    dst_ea.kind = EAK_Dn;   /* silence may-be-uninitialized warnings */

    if (rm == 0) {
        src_val = cpu->D[dy] & m;
        dst_val = cpu->D[dx] & m;
    } else {
        EADesc src_ea;
        if (!ea_resolve(cpu, EA_MODE_PRE, dy, sz, &src_ea)) return 8;
        if (!ea_read(cpu, &src_ea, sz, &src_val)) return 8;
        if (!ea_resolve(cpu, EA_MODE_PRE, dx, sz, &dst_ea)) return 8;
        if (!ea_read(cpu, &dst_ea, sz, &dst_val)) return 8;
        src_val &= m;
        dst_val &= m;
    }

    u32 result;
    bool carry;
    u16 ccr = cpu->SR & ~0x1Fu;   /* preserve upper SR, clear CCR */

    if (is_sub) {
        u64 borrow_check = (u64)src_val + x;
        carry  = borrow_check > (u64)dst_val;
        result = (u32)(((u64)dst_val - (u64)src_val - x) & (u64)m);
        if (((src_val ^ dst_val) & s) && ((dst_val ^ result) & s)) ccr |= CCR_V;
    } else {
        u64 sum64 = (u64)dst_val + (u64)src_val + x;
        carry  = sum64 > (u64)m;
        result = (u32)(sum64 & (u64)m);
        if (!((src_val ^ dst_val) & s) && ((src_val ^ result) & s)) ccr |= CCR_V;
    }

    if (carry)     ccr |= CCR_C | CCR_X;
    if (result & s) ccr |= CCR_N;
    /* Z: cleared by non-zero result; preserved (not set) when zero */
    if (result != 0) ccr &= ~CCR_Z;
    else             ccr |= (cpu->SR & CCR_Z);

    /* Write result */
    if (rm == 0) {
        switch (sz) {
            case SIZE_BYTE: cpu->D[dx] = (cpu->D[dx] & 0xFFFFFF00u) | (result & 0xFFu); break;
            case SIZE_WORD: cpu->D[dx] = (cpu->D[dx] & 0xFFFF0000u) | (result & 0xFFFFu); break;
            case SIZE_LONG: cpu->D[dx] = result; break;
        }
    } else {
        if (!ea_write(cpu, &dst_ea, sz, result)) return 8;
    }

    cpu->SR = ccr;
    return 8;
}

static u32 handler_addx(M68020State *cpu, u16 op) { return do_addx_subx(cpu, op, false); }
static u32 handler_subx(M68020State *cpu, u16 op) { return do_addx_subx(cpu, op, true);  }

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_alu_install_handlers(InsnHandler *t) {
    /* ADDQ: 0101 DDD0 ss ea (ss = 00/01/10, ea varies) */
    for (u32 op = 0x5000u; op <= 0x5FFFu; op++) {
        u8 bit8 = (op >> 8) & 1u;
        u8 ss   = (op >> 6) & 3u;
        if (bit8 == 0 && ss != 3) t[op] = handler_addq;
        if (bit8 == 1 && ss != 3) t[op] = handler_subq;
    }

    /* ADD: 1101 DDD d ss ea */
    for (u32 op = 0xD000u; op <= 0xDFFFu; op++) {
        u8 ss  = (op >> 6) & 3u;
        u8 dir = (op >> 8) & 1u;
        u8 ea_mode = (op >> 3) & 7u;
        /* ADDA: dir=1 and size bits 8:6 = 011 (.W) or 111 (.L) */
        if ((op & 0x01C0u) == 0x00C0u) { t[op] = handler_adda; continue; }
        if ((op & 0x01C0u) == 0x01C0u) { t[op] = handler_adda; continue; }
        if (ss == 3) continue;  /* reserved / ADDA */
        if (dir == 1 && ea_mode == EA_MODE_AN) continue; /* ADDA */
        t[op] = handler_add;
    }

    /* SUB: 1001 DDD d ss ea */
    for (u32 op = 0x9000u; op <= 0x9FFFu; op++) {
        u8 ss = (op >> 6) & 3u;
        if ((op & 0x01C0u) == 0x00C0u) { t[op] = handler_suba; continue; }
        if ((op & 0x01C0u) == 0x01C0u) { t[op] = handler_suba; continue; }
        if (ss == 3) continue;
        t[op] = handler_sub;
    }

    /* ADDX: 1101 Dx 1 ss 00 M Dy — installed AFTER ADD to overwrite incorrect slots */
    for (u32 dx = 0; dx < 8; dx++)
        for (u32 ss = 0; ss < 3; ss++)
            for (u32 dy = 0; dy < 8; dy++) {
                u32 base = 0xD100u | (dx << 9) | (ss << 6) | dy;
                t[base]        = handler_addx;  /* M=0 register form */
                t[base | 0x08u] = handler_addx;  /* M=1 predecrement form */
            }

    /* SUBX: 1001 Dx 1 ss 00 M Dy — installed AFTER SUB to overwrite incorrect slots */
    for (u32 dx = 0; dx < 8; dx++)
        for (u32 ss = 0; ss < 3; ss++)
            for (u32 dy = 0; dy < 8; dy++) {
                u32 base = 0x9100u | (dx << 9) | (ss << 6) | dy;
                t[base]        = handler_subx;
                t[base | 0x08u] = handler_subx;
            }

    /* OR: 1000 DDD d ss ea */
    for (u32 op = 0x8000u; op <= 0x8FFFu; op++) {
        u8 ss = (op >> 6) & 3u;
        if (ss == 3) continue;
        t[op] = handler_or;
    }

    /* AND: 1100 DDD d ss ea */
    for (u32 op = 0xC000u; op <= 0xCFFFu; op++) {
        u8 ss = (op >> 6) & 3u;
        if (ss == 3) continue;
        t[op] = handler_and;
    }

    /* EOR: 1011 DDD1 ss ea */
    for (u32 op = 0xB000u; op <= 0xBFFFu; op++) {
        u8 dir = (op >> 8) & 1u;
        u8 ss  = (op >> 6) & 3u;
        if (dir == 1 && ss != 3) t[op] = handler_eor;
    }

    /* Immediate operations (group 0x0000) */
    /* ORI:  0000 0000 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0000u | (ss << 6) | ea] = handler_ori;
    t[0x003Cu] = handler_ori_to_ccr;  /* ORI #imm,CCR */
    t[0x007Cu] = handler_ori_to_sr;   /* ORI #imm,SR  */

    /* ANDI: 0000 0010 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0200u | (ss << 6) | ea] = handler_andi;
    t[0x023Cu] = handler_andi_to_ccr;
    t[0x027Cu] = handler_andi_to_sr;

    /* SUBI: 0000 0100 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0400u | (ss << 6) | ea] = handler_subi;

    /* ADDI: 0000 0110 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0600u | (ss << 6) | ea] = handler_addi;

    /* EORI: 0000 1010 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0A00u | (ss << 6) | ea] = handler_eori;
    t[0x0A3Cu] = handler_eori_to_ccr;
    t[0x0A7Cu] = handler_eori_to_sr;

    /* NOT: 0100 0110 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x4600u | (ss << 6) | ea] = handler_not;

    /* NEG: 0100 0100 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x4400u | (ss << 6) | ea] = handler_neg;

    /* NEGX: 0100 0000 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x4000u | (ss << 6) | ea] = handler_negx;

    /* CLR: 0100 0010 ss ea */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x4200u | (ss << 6) | ea] = handler_clr;

    /* EXT.W and EXT.L: 0100 1000 1s0 rrr */
    for (u32 r = 0; r < 8; r++) {
        t[OP_EXT_W + r] = handler_ext;
        t[OP_EXT_L + r] = handler_ext;
        t[OP_EXTB_L + r] = handler_extb;
    }

    /* SWAP: 0100 1000 0100 0rrr */
    for (u32 r = 0; r < 8; r++)
        t[OP_SWAP + r] = handler_swap;
}
