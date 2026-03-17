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

void m68020_compare_install_handlers(InsnHandler *t) {
    /* CMP: 1011 DDD0 ss ea */
    for (u32 op = 0xB000u; op <= 0xBFFFu; op++) {
        u8 dir = (op >> 8) & 1u;
        u8 ss  = (op >> 6) & 3u;
        u8 ea_mode = (op >> 3) & 7u;
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
}
