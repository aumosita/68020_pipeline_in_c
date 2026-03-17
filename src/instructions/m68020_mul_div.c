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
}
