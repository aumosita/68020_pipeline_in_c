/*
 * m68020_shift.c — ASL/ASR, LSL/LSR, ROL/ROR, ROXL/ROXR.
 * Phase 1: full implementation of all shift/rotate instructions.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

static BusSize shift_size(u8 ss) {
    static const BusSize sz[4] = { SIZE_BYTE, SIZE_WORD, SIZE_LONG, SIZE_LONG };
    return sz[ss & 3u];
}

/* ------------------------------------------------------------------ */
/* Register shift/rotate: 1110 CCC d ss i0T rrr                       */
/* Memory shift/rotate:   1110 00T d 11 ea                            */
/* ------------------------------------------------------------------ */

static u32 do_shift(M68020State *cpu, u16 opword) {
    u8  type = (opword >> 3) & 3u;   /* 00=AS, 01=LS, 10=ROX, 11=RO */
    u8  dir  = (opword >> 8) & 1u;   /* 0=right, 1=left              */
    u8  ss   = (opword >> 6) & 3u;
    BusSize sz = shift_size(ss);

    u32 val, count;

    bool is_mem_shift = (ss == 3);  /* ss=11 means memory shift, 1-bit */

    if (is_mem_shift) {
        /* Memory shift: count is always 1 */
        EADesc dst;
        if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &dst))
            return 8;
        u32 tmp = 0;
        if (!ea_read(cpu, &dst, SIZE_WORD, &tmp)) return 8;
        val   = tmp;
        count = 1;
        sz    = SIZE_WORD;
    } else {
        u8 ir = (opword >> 5) & 1u;   /* 0=imm count, 1=Dn count */
        u8 dn = opword & 7u;
        u8 count_src = (opword >> 9) & 7u;

        val = cpu->D[dn];
        if (ir) {
            count = cpu->D[count_src] & 63u;  /* Dn mod 64 */
        } else {
            count = count_src ? count_src : 8u;  /* 000 = 8 */
        }
    }

    u32 m    = sz_mask(sz);
    u32 sign = sz_sign(sz);
    u32 bits = (u32)sz * 8u;
    val &= m;

    u16 ccr  = cpu->SR & ~0x1Fu;
    bool x   = SR_X(cpu->SR) != 0;
    u32  result = val;
    u32  last_out = 0;

    if (count == 0) {
        /* Shift by 0: C=0, X unchanged, N/Z from value, V=0 */
        if (!result) ccr |= CCR_Z;
        if (result & sign) ccr |= CCR_N;
        cpu->SR = ccr;
        return 8;
    }

    switch (type) {
    case 0: /* AS — Arithmetic Shift */
        if (dir) { /* left */
            /* V=1 if any bits shifted out differ from final sign */
            u32 old_sign = val & sign;
            bool v = false;
            for (u32 i = 0; i < count; i++) {
                last_out = (result >> (bits - 1)) & 1u;
                result = (result << 1) & m;
                if ((result & sign) != old_sign) v = true;
            }
            if (v) ccr |= CCR_V;
        } else { /* right */
            for (u32 i = 0; i < count; i++) {
                last_out = result & 1u;
                result = ((result >> 1) | (result & sign)) & m;
            }
        }
        break;

    case 1: /* LS — Logical Shift */
        if (dir) { /* left */
            for (u32 i = 0; i < count; i++) {
                last_out = (result >> (bits - 1)) & 1u;
                result = (result << 1) & m;
            }
        } else { /* right */
            for (u32 i = 0; i < count; i++) {
                last_out = result & 1u;
                result >>= 1;
            }
        }
        break;

    case 2: /* ROX — Rotate with Extend */
        if (dir) { /* left */
            for (u32 i = 0; i < count; i++) {
                last_out = (result >> (bits - 1)) & 1u;
                result = ((result << 1) | (x ? 1u : 0u)) & m;
                x = last_out != 0;
            }
        } else { /* right */
            for (u32 i = 0; i < count; i++) {
                last_out = result & 1u;
                result = (result >> 1) | ((x ? 1u : 0u) << (bits - 1));
                x = last_out != 0;
            }
        }
        if (x) ccr |= CCR_X | CCR_C;
        if (!result) ccr |= CCR_Z;
        if (result & sign) ccr |= CCR_N;
        cpu->SR = ccr;
        if (is_mem_shift) {
            EADesc dst;
            ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst);
            ea_write(cpu, &dst, sz, result);
        } else {
            switch(sz){
            case SIZE_BYTE: cpu->D[opword&7]=(cpu->D[opword&7]&0xFFFFFF00u)|(result&0xFFu); break;
            case SIZE_WORD: cpu->D[opword&7]=(cpu->D[opword&7]&0xFFFF0000u)|(result&0xFFFFu); break;
            case SIZE_LONG: cpu->D[opword&7]=result; break;
            }
        }
        return 8;

    case 3: /* RO — Rotate */
        if (dir) { /* left */
            count &= (bits - 1);
            if (count)
                result = ((result << count) | (result >> (bits - count))) & m;
            last_out = result & 1u;
        } else { /* right */
            count &= (bits - 1);
            if (count)
                result = ((result >> count) | (result << (bits - count))) & m;
            last_out = (result >> (bits - 1)) & 1u;
        }
        if (last_out) ccr |= CCR_C;
        if (!result) ccr |= CCR_Z;
        if (result & sign) ccr |= CCR_N;
        cpu->SR = ccr;
        if (is_mem_shift) {
            EADesc dst;
            ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst);
            ea_write(cpu, &dst, sz, result);
        } else {
            switch(sz){
            case SIZE_BYTE: cpu->D[opword&7]=(cpu->D[opword&7]&0xFFFFFF00u)|(result&0xFFu); break;
            case SIZE_WORD: cpu->D[opword&7]=(cpu->D[opword&7]&0xFFFF0000u)|(result&0xFFFFu); break;
            case SIZE_LONG: cpu->D[opword&7]=result; break;
            }
        }
        return 8;
    }

    /* Common exit for AS/LS */
    if (last_out) ccr |= CCR_C | CCR_X;
    if (!result) ccr |= CCR_Z;
    if (result & sign) ccr |= CCR_N;
    cpu->SR = ccr;

    if (is_mem_shift) {
        EADesc dst;
        ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &dst);
        ea_write(cpu, &dst, sz, result);
    } else {
        u8 dn = opword & 7u;
        switch(sz){
        case SIZE_BYTE: cpu->D[dn]=(cpu->D[dn]&0xFFFFFF00u)|(result&0xFFu); break;
        case SIZE_WORD: cpu->D[dn]=(cpu->D[dn]&0xFFFF0000u)|(result&0xFFFFu); break;
        case SIZE_LONG: cpu->D[dn]=result; break;
        }
    }
    return 8;
}

void m68020_shift_install_handlers(InsnHandler *t) {
    /* Shift group: 1110 xxxx xxxx xxxx */
    for (u32 op = 0xE000u; op <= 0xEFFFu; op++) {
        u8 ss = (op >> 6) & 3u;
        /* Memory shifts have ss=11, ea in [5:0] */
        /* Register shifts have ss=00/01/10 */
        if (ss <= 2 || ss == 3)
            t[op] = do_shift;
    }
}
