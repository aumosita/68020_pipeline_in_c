/*
 * m68020_mul_div.c — MULS, MULU, DIVS, DIVU.
 * Phase 1: 16×16→32 multiply, 32÷16 divide.
 * Phase 4: 32×32→64 and 64÷32 long forms.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

/* MULU.W <ea>,Dn — unsigned 16×16→32 */
static u32 handler_mulu_w(M68020State *cpu, u16 opword) {
    u8 dn = (opword >> 9) & 7u;
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &src)) return 70;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, SIZE_WORD, &src_val)) return 70;
    u32 result = (u16)cpu->D[dn] * (u16)src_val;
    cpu->D[dn] = result;
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr_logic(result, SIZE_LONG);
    return 70;
}

/* MULS.W <ea>,Dn — signed 16×16→32 */
static u32 handler_muls_w(M68020State *cpu, u16 opword) {
    u8 dn = (opword >> 9) & 7u;
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &src)) return 70;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, SIZE_WORD, &src_val)) return 70;
    s32 result = (s32)(s16)(u16)cpu->D[dn] * (s32)(s16)(u16)src_val;
    cpu->D[dn] = (u32)result;
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr_logic((u32)result, SIZE_LONG);
    return 70;
}

/* DIVU.W <ea>,Dn — unsigned 32÷16→16:16 (quotient:remainder) */
static u32 handler_divu_w(M68020State *cpu, u16 opword) {
    u8 dn = (opword >> 9) & 7u;
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &src)) return 140;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, SIZE_WORD, &src_val)) return 140;
    u16 divisor = (u16)src_val;
    if (divisor == 0) { exception_process(cpu, VEC_ZERO_DIVIDE); return 38; }
    u32 dividend = cpu->D[dn];
    u32 quotient = dividend / divisor;
    if (quotient > 0xFFFFu) {
        cpu->SR |= CCR_V;
        return 140;
    }
    u16 remainder = (u16)(dividend % divisor);
    cpu->D[dn] = ((u32)remainder << 16) | (quotient & 0xFFFFu);
    u16 ccr = ccr_logic(quotient & 0xFFFFu, SIZE_WORD);
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    return 140;
}

/* DIVS.W <ea>,Dn — signed 32÷16→16:16 */
static u32 handler_divs_w(M68020State *cpu, u16 opword) {
    u8 dn = (opword >> 9) & 7u;
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &src)) return 158;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, SIZE_WORD, &src_val)) return 158;
    s16 divisor = (s16)(u16)src_val;
    if (divisor == 0) { exception_process(cpu, VEC_ZERO_DIVIDE); return 38; }
    s32 dividend = (s32)cpu->D[dn];
    s32 quotient = dividend / divisor;
    if (quotient > 32767 || quotient < -32768) {
        cpu->SR |= CCR_V;
        return 158;
    }
    s16 remainder = (s16)(dividend % divisor);
    cpu->D[dn] = ((u32)(u16)remainder << 16) | (u16)(s16)quotient;
    u16 ccr = ccr_logic((u32)(u16)(s16)quotient, SIZE_WORD);
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    return 158;
}

/* ------------------------------------------------------------------ */
/* MULS.L / MULU.L — 32×32 → 32 or 64 (68020)                        */
/* ------------------------------------------------------------------ */
/*
 * Opword: 0100 1100 00 ea = 0x4C00 | ea
 * Extension word:
 *   bits 14:12 : Dh (high result register, 64-bit form only)
 *   bit  11    : S  (0=MULU, 1=MULS)
 *   bit  10    : L  (0=32-bit result in Dl, 1=64-bit result Dh:Dl)
 *   bits  2:0  : Dl (low result register / only register for 32-bit form)
 */
static u32 handler_mul_l(M68020State *cpu, u16 opword) {
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_LONG, &src)) return 43;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, SIZE_LONG, &src_val)) return 43;

    u16 ext = pipeline_consume_word(cpu);
    u8  dh  = (ext >> 12) & 7u;
    bool s  = (ext >> 11) & 1u;  /* signed */
    bool l  = (ext >> 10) & 1u;  /* 64-bit result */
    u8  dl  = ext & 7u;

    u16 ccr;
    if (s) {
        s64 result = (s64)(s32)cpu->D[dl] * (s64)(s32)src_val;
        if (l) {
            cpu->D[dl] = (u32)(u64)result;          /* low 32 bits  */
            cpu->D[dh] = (u32)((u64)result >> 32);  /* high 32 bits */
        } else {
            cpu->D[dl] = (u32)(s32)result;
        }
        u32 vis = l ? (u32)((u64)result >> 32) : (u32)(s32)result;
        ccr = ccr_logic(vis, SIZE_LONG);
        /* V=1 if result doesn't fit in 32 bits (for 32-bit form) */
        if (!l && ((s64)(s32)cpu->D[dl] != result)) ccr |= CCR_V;
    } else {
        u64 result = (u64)cpu->D[dl] * (u64)src_val;
        if (l) {
            cpu->D[dl] = (u32)result;          /* low 32 bits  */
            cpu->D[dh] = (u32)(result >> 32);  /* high 32 bits */
        } else {
            cpu->D[dl] = (u32)result;
        }
        u32 vis = l ? (u32)(result >> 32) : (u32)result;
        ccr = ccr_logic(vis, SIZE_LONG);
        if (!l && (result >> 32)) ccr |= CCR_V;
    }
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    return 43;
}

/* ------------------------------------------------------------------ */
/* DIVS.L / DIVU.L — 32÷32 → 32 or 64÷32 → 32 (68020)               */
/* ------------------------------------------------------------------ */
/*
 * Opword: 0100 1100 01 ea = 0x4C40 | ea
 * Extension word:
 *   bits 14:12 : Dr (remainder register; high half of dividend if L=1)
 *   bit  11    : S  (0=DIVU, 1=DIVS)
 *   bit  10    : L  (0=32÷32, 1=64÷32 with dividend in Dr:Dq)
 *   bits  2:0  : Dq (quotient register; low half of dividend if L=1)
 */
static u32 handler_div_l(M68020State *cpu, u16 opword) {
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_LONG, &src)) return 84;
    u32 src_val = 0;
    if (!ea_read(cpu, &src, SIZE_LONG, &src_val)) return 84;

    u16 ext = pipeline_consume_word(cpu);
    u8  dr  = (ext >> 12) & 7u;
    bool s  = (ext >> 11) & 1u;
    bool l  = (ext >> 10) & 1u;
    u8  dq  = ext & 7u;

    if (src_val == 0) {
        exception_process(cpu, VEC_ZERO_DIVIDE);
        return 38;
    }

    if (s) {
        /* Signed divide */
        s64 dividend = l
            ? (s64)(((u64)cpu->D[dr] << 32) | cpu->D[dq])
            : (s64)(s32)cpu->D[dq];
        s32 divisor  = (s32)src_val;

        s64 quot = dividend / divisor;
        s32 rem  = (s32)(dividend % divisor);

        /* Overflow check: quotient must fit in 32 bits */
        if (quot > (s64)0x7FFFFFFFll || quot < (s64)(-0x80000000ll - 1)) {
            cpu->SR |= CCR_V;
            return 84;
        }
        cpu->D[dq] = (u32)(s32)quot;
        if (l) cpu->D[dr] = (u32)rem;

        u16 ccr = ccr_logic((u32)(s32)quot, SIZE_LONG);
        cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    } else {
        /* Unsigned divide */
        u64 dividend = l
            ? (((u64)cpu->D[dr] << 32) | cpu->D[dq])
            : (u64)cpu->D[dq];
        u32 divisor  = src_val;

        u64 quot = dividend / divisor;
        u32 rem  = (u32)(dividend % divisor);

        if (quot > 0xFFFFFFFFuLL) {
            cpu->SR |= CCR_V;
            return 84;
        }
        cpu->D[dq] = (u32)quot;
        if (l) cpu->D[dr] = rem;

        u16 ccr = ccr_logic((u32)quot, SIZE_LONG);
        cpu->SR = (cpu->SR & ~0x1Fu) | ccr;
    }
    return 84;
}

void m68020_mul_div_install_handlers(InsnHandler *t) {
    /* MULU.W: 1100 DDD0 11 ea */
    for (u32 dn = 0; dn < 8; dn++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0xC0C0u | (dn << 9) | ea] = handler_mulu_w;

    /* MULS.W: 1100 DDD1 11 ea */
    for (u32 dn = 0; dn < 8; dn++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0xC1C0u | (dn << 9) | ea] = handler_muls_w;

    /* DIVU.W: 1000 DDD0 11 ea */
    for (u32 dn = 0; dn < 8; dn++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x80C0u | (dn << 9) | ea] = handler_divu_w;

    /* DIVS.W: 1000 DDD1 11 ea */
    for (u32 dn = 0; dn < 8; dn++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x81C0u | (dn << 9) | ea] = handler_divs_w;

    /* MULS.L / MULU.L (68020): 0100 1100 00 ea = 0x4C00 | ea */
    for (u32 ea = 0; ea < 64; ea++)
        t[0x4C00u | ea] = handler_mul_l;

    /* DIVS.L / DIVU.L (68020): 0100 1100 01 ea = 0x4C40 | ea */
    for (u32 ea = 0; ea < 64; ea++)
        t[0x4C40u | ea] = handler_div_l;
}
