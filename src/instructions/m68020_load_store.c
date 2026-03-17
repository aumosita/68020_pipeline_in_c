/*
 * m68020_load_store.c — MOVEP, BCHG, BCLR, BSET, BTST.
 * LEA, PEA, LINK, UNLK, EXG, SWAP are in misc.c / jump.c.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

/* ------------------------------------------------------------------ */
/* Bit instructions: BTST, BCHG, BCLR, BSET                           */
/* ------------------------------------------------------------------ */

static u32 do_bit_op(M68020State *cpu, u16 opword, u8 op) {
    /* op: 0=BTST, 1=BCHG, 2=BCLR, 3=BSET */
    u8 ea_mode = EA_SRC_MODE(opword);
    u8 ea_reg  = EA_SRC_REG(opword);
    bool is_static = !(opword & 0x0100u);  /* 0=static(imm), 1=dynamic(Dn) */

    u32 bit_num;
    if (is_static) {
        bit_num = pipeline_consume_word(cpu) & 0xFFu;
    } else {
        u8 dn = (opword >> 9) & 7u;
        bit_num = cpu->D[dn];
    }

    bool is_dn = (ea_mode == EA_MODE_DN);
    BusSize sz = is_dn ? SIZE_LONG : SIZE_BYTE;

    if (is_dn) {
        bit_num &= 31u;
        u32 mask = 1u << bit_num;
        u32 val  = cpu->D[ea_reg];
        if (val & mask) cpu->SR &= ~CCR_Z; else cpu->SR |= CCR_Z;
        switch (op) {
            case 1: cpu->D[ea_reg] ^= mask; break;
            case 2: cpu->D[ea_reg] &= ~mask; break;
            case 3: cpu->D[ea_reg] |= mask; break;
            default: break;
        }
    } else {
        bit_num &= 7u;
        u8 mask = (u8)(1u << bit_num);
        EADesc dst;
        if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &dst)) return 8;
        u32 raw = 0;
        if (!ea_read(cpu, &dst, sz, &raw)) return 8;
        u8 val = (u8)raw;
        if (val & mask) cpu->SR &= ~CCR_Z; else cpu->SR |= CCR_Z;
        switch (op) {
            case 1: val ^= mask;  ea_write(cpu, &dst, sz, val); break;
            case 2: val &= ~mask; ea_write(cpu, &dst, sz, val); break;
            case 3: val |= mask;  ea_write(cpu, &dst, sz, val); break;
            default: break;
        }
    }
    return 8;
}

static u32 handler_btst(M68020State *cpu, u16 op) { return do_bit_op(cpu, op, 0); }
static u32 handler_bchg(M68020State *cpu, u16 op) { return do_bit_op(cpu, op, 1); }
static u32 handler_bclr(M68020State *cpu, u16 op) { return do_bit_op(cpu, op, 2); }
static u32 handler_bset(M68020State *cpu, u16 op) { return do_bit_op(cpu, op, 3); }

/* ------------------------------------------------------------------ */
/* MOVEP — Move Peripheral                                             */
/* ------------------------------------------------------------------ */

static u32 handler_movep(M68020State *cpu, u16 opword) {
    u8  dn   = (opword >> 9) & 7u;
    u8  an   = opword & 7u;
    u8  dir  = (opword >> 7) & 1u;  /* 0=mem→reg, 1=reg→mem */
    u8  size = (opword >> 6) & 1u;  /* 0=word, 1=long */
    s16 disp = (s16)pipeline_consume_word(cpu);
    u32 addr = cpu->A[an] + (s32)disp;

    if (dir) {
        /* Reg → Mem (alternating bytes at even/odd addresses) */
        if (size) {
            cpu_write_byte(cpu, addr,     (u8)(cpu->D[dn] >> 24));
            cpu_write_byte(cpu, addr + 2, (u8)(cpu->D[dn] >> 16));
            cpu_write_byte(cpu, addr + 4, (u8)(cpu->D[dn] >>  8));
            cpu_write_byte(cpu, addr + 6, (u8)(cpu->D[dn]      ));
        } else {
            cpu_write_byte(cpu, addr,     (u8)(cpu->D[dn] >>  8));
            cpu_write_byte(cpu, addr + 2, (u8)(cpu->D[dn]      ));
        }
    } else {
        /* Mem → Reg */
        if (size) {
            u8 b0=0,b1=0,b2=0,b3=0;
            cpu_read_byte(cpu, addr,     &b0);
            cpu_read_byte(cpu, addr + 2, &b1);
            cpu_read_byte(cpu, addr + 4, &b2);
            cpu_read_byte(cpu, addr + 6, &b3);
            cpu->D[dn] = ((u32)b0<<24)|((u32)b1<<16)|((u32)b2<<8)|b3;
        } else {
            u8 b0=0, b1=0;
            cpu_read_byte(cpu, addr,     &b0);
            cpu_read_byte(cpu, addr + 2, &b1);
            cpu->D[dn] = (cpu->D[dn] & 0xFFFF0000u) | ((u32)b0<<8) | b1;
        }
    }
    return 16;
}

void m68020_load_store_install_handlers(InsnHandler *t) {
    /* BTST static: 0000 100 0 11 ea  (0x0800) */
    for (u32 ea = 0; ea < 64; ea++) t[0x0800u | ea] = handler_btst;
    /* BCHG static: 0000 101 0 11 ea  (0x0840... wait no) */
    /* Actually: BCHG=0x0840, BCLR=0x0880, BSET=0x08C0 ... */
    /* Static bit ops: 0000 1000 xx ea */
    for (u32 ea = 0; ea < 64; ea++) {
        t[0x0800u | ea] = handler_btst;
        t[0x0840u | ea] = handler_bchg;
        t[0x0880u | ea] = handler_bclr;
        t[0x08C0u | ea] = handler_bset;
    }
    /* Dynamic bit ops: 0000 DDD1 00 ea (BTST=000), 01=BCHG, 10=BCLR, 11=BSET */
    for (u32 dn = 0; dn < 8; dn++)
        for (u32 ea = 0; ea < 64; ea++) {
            t[0x0100u | (dn << 9) | ea] = handler_btst;
            t[0x0140u | (dn << 9) | ea] = handler_bchg;
            t[0x0180u | (dn << 9) | ea] = handler_bclr;
            t[0x01C0u | (dn << 9) | ea] = handler_bset;
        }

    /* MOVEP: 0000 DDD1 mm 001 AAA (mm=00..11, various) */
    for (u32 op = 0x0108u; op <= 0x01FFu; op++) {
        u8 lo = op & 0xFFu;
        if ((lo & 0x38u) == 0x08u)  /* bits [5:3] = 001 (An) */
            t[op] = handler_movep;
    }
}
