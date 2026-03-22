/*
 * m68020_bitfield.c — BFTST, BFEXTU, BFCHG, BFEXTS, BFCLR, BFFFO, BFSET, BFINS.
 *
 * Opcode map (opword & 0xFFC0 after masking lower 6 EA bits):
 *   BFTST:  0xE8C0 | ea
 *   BFEXTU: 0xE9C0 | ea
 *   BFCHG:  0xEAC0 | ea
 *   BFEXTS: 0xEBC0 | ea
 *   BFCLR:  0xECC0 | ea
 *   BFFFO:  0xEDC0 | ea
 *   BFSET:  0xEEC0 | ea
 *   BFINS:  0xEFC0 | ea
 *
 * Extension word format (all instructions):
 *   bit 15:    reserved (0)
 *   bits 14:12: Dn register (destination for BFEXTU/BFEXTS/BFFFO; source for BFINS)
 *   bit 11:    D/O — 0=immediate offset, 1=register offset
 *   bits 10:6: immediate offset (5-bit unsigned, 0-31) when D/O=0
 *              bits 8:6 = offset register number when D/O=1
 *   bit 5:     D/W — 0=immediate width, 1=register width
 *   bits 4:0:  immediate width (0=32, 1-31) when D/W=0
 *              bits 2:0 = width register number when D/W=1
 *
 * Flags: N = MSB of extracted field; Z = field all zeros; V=0; C=0; X unchanged.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

/* ------------------------------------------------------------------ */
/* Bitfield spec decode                                                 */
/* ------------------------------------------------------------------ */

typedef struct {
    u32 offset;   /* bit offset (signed-capable via s32 cast for memory) */
    u32 width;    /* 1..32 */
    u8  dn;       /* result/source Dn register                           */
} BFSpec;

static void bf_decode(const M68020State *cpu, u16 ext, BFSpec *bf) {
    bool do_bit = (ext >> 11) & 1u;
    bool dw_bit = (ext >>  5) & 1u;

    bf->dn     = (ext >> 12) & 7u;
    bf->offset = do_bit ? cpu->D[(ext >> 6) & 7u]
                        : (u32)((ext >> 6) & 31u);
    u32 w      = dw_bit ? (cpu->D[ext & 7u] & 31u)
                        : (u32)(ext & 31u);
    bf->width  = (w == 0) ? 32u : w;
}

/* ------------------------------------------------------------------ */
/* Bitfield extract/insert for Dn (register) EA                        */
/* ------------------------------------------------------------------ */

/*
 * Extract `width` bits from a 32-bit register, starting at bit `start`
 * counted from the MSB (i.e., start=0 → MSB = bit 31 of the u32).
 * Wraps around within the 32-bit register.
 */
static u32 bf_reg_extract(u32 reg, u32 start, u32 width) {
    start &= 31u;
    /* Duplicate register in 64 bits to handle wrap-around */
    u64 v    = ((u64)reg << 32) | (u64)reg;
    u32 shift = 64u - start - width;
    u64 mask  = (width == 32) ? 0xFFFFFFFFuLL : ((1uLL << width) - 1u);
    return (u32)((v >> shift) & mask);
}

/*
 * Insert `width` bits from `field` (right-aligned) into `reg`
 * at bit position `start` from MSB, with wrap-around.
 */
static u32 bf_reg_insert(u32 reg, u32 start, u32 width, u32 field) {
    start &= 31u;
    u64 v     = ((u64)reg << 32) | (u64)reg;
    u32 shift = 64u - start - width;
    u64 fmask = (width == 32) ? 0xFFFFFFFFuLL : ((1uLL << width) - 1u);
    u64 ins   = ((u64)(field & (u32)fmask)) << shift;
    u64 mask  = fmask << shift;
    v = (v & ~mask) | ins;
    u32 hi = (u32)(v >> 32);
    u32 lo = (u32)(v & 0xFFFFFFFFuLL);
    if (start + width <= 32u)
        return hi;   /* no wrap — upper half is the correct result */
    /* Wrap: hi has the field's leading bits (at the low end of the register),
     * lo has the field's trailing bits (at the high end of the register).
     * Take trailing field bits from lo, everything else from hi. */
    u32 wrap_bits = start + width - 32u;
    u32 lo_mask = (wrap_bits >= 32u) ? ~0u : ((1u << wrap_bits) - 1u) << (32u - wrap_bits);
    return (hi & ~lo_mask) | (lo & lo_mask);
}

/* ------------------------------------------------------------------ */
/* Bitfield extract/insert for memory EA                               */
/* ------------------------------------------------------------------ */

/*
 * Read `width` bits from memory at `byte_addr`, starting at `bit_in_byte`
 * (0=MSB of that byte). Returns right-aligned extracted value.
 * Reads up to 5 bytes to cover any alignment.
 */
static bool bf_mem_read(M68020State *cpu, u32 byte_addr, u32 bit_in_byte,
                        u32 width, u32 *val) {
    u32 nbytes = (bit_in_byte + width + 7u) / 8u;
    u64 bits = 0;
    for (u32 i = 0; i < nbytes; i++) {
        u8 b = 0;
        if (cpu_read_byte(cpu, byte_addr + i, &b) != BUS_OK)
            return false;
        bits = (bits << 8) | b;
    }
    u32 shift = nbytes * 8u - bit_in_byte - width;
    u64 mask  = (width == 32) ? 0xFFFFFFFFuLL : ((1uLL << width) - 1u);
    *val = (u32)((bits >> shift) & mask);
    return true;
}

static bool bf_mem_write(M68020State *cpu, u32 byte_addr, u32 bit_in_byte,
                         u32 width, u32 field) {
    u32 nbytes = (bit_in_byte + width + 7u) / 8u;
    /* Read original bytes */
    u64 orig = 0;
    for (u32 i = 0; i < nbytes; i++) {
        u8 b = 0;
        if (cpu_read_byte(cpu, byte_addr + i, &b) != BUS_OK)
            return false;
        orig = (orig << 8) | b;
    }
    u32 shift  = nbytes * 8u - bit_in_byte - width;
    u64 fmask  = (width == 32) ? 0xFFFFFFFFuLL : ((1uLL << width) - 1u);
    u64 result = (orig & ~(fmask << shift)) | (((u64)field & fmask) << shift);
    for (u32 i = 0; i < nbytes; i++) {
        u8 b = (u8)(result >> ((nbytes - 1u - i) * 8u));
        if (cpu_write_byte(cpu, byte_addr + i, b) != BUS_OK)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* Common bitfield read: resolves EA, extracts field                   */
/* ------------------------------------------------------------------ */

typedef struct {
    bool  is_dn;       /* true = Dn register EA */
    u8    reg;         /* Dn number (if is_dn) */
    u32   byte_addr;   /* memory byte address (if !is_dn) */
    u32   bit_in_byte; /* bit offset within byte_addr (0..7) */
    u32   field;       /* extracted field value (right-aligned) */
    u32   start;       /* for Dn: start_bit (0..31) */
} BFAccess;

static bool bf_access_read(M68020State *cpu, u16 opword, const BFSpec *bf,
                           BFAccess *acc) {
    u8 mode = EA_SRC_MODE(opword);
    u8 reg  = EA_SRC_REG(opword);

    if (mode == EA_MODE_DN) {
        acc->is_dn  = true;
        acc->reg    = reg;
        acc->start  = bf->offset & 31u;
        acc->field  = bf_reg_extract(cpu->D[reg], acc->start, bf->width);
    } else {
        EADesc ea;
        if (!ea_resolve(cpu, mode, reg, SIZE_BYTE, &ea)) return false;
        if (ea.kind != EAK_Mem) {
            exception_process(cpu, VEC_ILLEGAL_INSN);
            return false;
        }
        s32 bit_offset = (s32)bf->offset;
        s32 byte_off   = bit_offset >> 3;    /* arithmetic (floor) divide by 8 */
        acc->is_dn     = false;
        acc->byte_addr = ea.address + (u32)(s32)byte_off;
        acc->bit_in_byte = (u32)(bit_offset - (byte_off << 3));
        if (!bf_mem_read(cpu, acc->byte_addr, acc->bit_in_byte,
                         bf->width, &acc->field))
            return false;
    }
    return true;
}

/* Set N and Z flags from extracted bitfield */
static void bf_set_nz(M68020State *cpu, u32 field, u32 width) {
    u32 sign = (width == 32) ? 0x80000000u : (1u << (width - 1));
    cpu->SR &= ~(CCR_N | CCR_Z | CCR_V | CCR_C);
    if (field & sign) cpu->SR |= CCR_N;
    if (field == 0)   cpu->SR |= CCR_Z;
}

/* ------------------------------------------------------------------ */
/* BFTST — test bitfield                                               */
/* ------------------------------------------------------------------ */
static u32 handler_bftst(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 6;
    bf_set_nz(cpu, acc.field, bf.width);
    return 6;
}

/* ------------------------------------------------------------------ */
/* BFEXTU / BFEXTS — extract unsigned/signed                          */
/* ------------------------------------------------------------------ */
static u32 handler_bfextu(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 8;
    bf_set_nz(cpu, acc.field, bf.width);
    cpu->D[bf.dn] = acc.field;   /* zero-extended */
    return 8;
}

static u32 handler_bfexts(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 8;
    bf_set_nz(cpu, acc.field, bf.width);
    if (bf.width < 32) {
        s32 sval = (s32)(acc.field << (32u - bf.width)) >> (32u - bf.width);
        cpu->D[bf.dn] = (u32)sval;
    } else {
        cpu->D[bf.dn] = acc.field;
    }
    return 8;
}

/* ------------------------------------------------------------------ */
/* BFCHG — change (complement) bitfield                               */
/* ------------------------------------------------------------------ */
static u32 handler_bfchg(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 8;
    bf_set_nz(cpu, acc.field, bf.width);
    u32 fmask  = (bf.width == 32) ? ~0u : ((1u << bf.width) - 1u);
    u32 result = (~acc.field) & fmask;
    if (acc.is_dn)
        cpu->D[acc.reg] = bf_reg_insert(cpu->D[acc.reg], acc.start, bf.width, result);
    else
        bf_mem_write(cpu, acc.byte_addr, acc.bit_in_byte, bf.width, result);
    return 8;
}

/* ------------------------------------------------------------------ */
/* BFCLR — clear bitfield to 0                                        */
/* ------------------------------------------------------------------ */
static u32 handler_bfclr(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 8;
    bf_set_nz(cpu, acc.field, bf.width);
    if (acc.is_dn)
        cpu->D[acc.reg] = bf_reg_insert(cpu->D[acc.reg], acc.start, bf.width, 0u);
    else
        bf_mem_write(cpu, acc.byte_addr, acc.bit_in_byte, bf.width, 0u);
    return 8;
}

/* ------------------------------------------------------------------ */
/* BFSET — set bitfield to all 1s                                     */
/* ------------------------------------------------------------------ */
static u32 handler_bfset(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 8;
    bf_set_nz(cpu, acc.field, bf.width);
    u32 fmask  = (bf.width == 32) ? ~0u : ((1u << bf.width) - 1u);
    if (acc.is_dn)
        cpu->D[acc.reg] = bf_reg_insert(cpu->D[acc.reg], acc.start, bf.width, fmask);
    else
        bf_mem_write(cpu, acc.byte_addr, acc.bit_in_byte, bf.width, fmask);
    return 8;
}

/* ------------------------------------------------------------------ */
/* BFFFO — find first one                                             */
/* ------------------------------------------------------------------ */
static u32 handler_bfffo(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);
    BFAccess acc;
    if (!bf_access_read(cpu, opword, &bf, &acc)) return 18;
    bf_set_nz(cpu, acc.field, bf.width);
    /* Scan from MSB of field for first 1 bit */
    u32 result = bf.offset + bf.width;   /* default if no 1 found */
    for (u32 i = 0; i < bf.width; i++) {
        if (acc.field & (1u << (bf.width - 1u - i))) {
            result = bf.offset + i;
            break;
        }
    }
    cpu->D[bf.dn] = result;
    return 18;
}

/* ------------------------------------------------------------------ */
/* BFINS — insert bitfield from Dn                                    */
/* ------------------------------------------------------------------ */
static u32 handler_bfins(M68020State *cpu, u16 opword) {
    u16 ext = pipeline_consume_word(cpu);
    BFSpec bf; bf_decode(cpu, ext, &bf);

    u8 mode = EA_SRC_MODE(opword);
    u8 reg  = EA_SRC_REG(opword);

    u32 fmask  = (bf.width == 32) ? ~0u : ((1u << bf.width) - 1u);
    u32 ins    = cpu->D[bf.dn] & fmask;

    bf_set_nz(cpu, ins, bf.width);

    if (mode == EA_MODE_DN) {
        u32 start = bf.offset & 31u;
        cpu->D[reg] = bf_reg_insert(cpu->D[reg], start, bf.width, ins);
    } else {
        EADesc ea;
        if (!ea_resolve(cpu, mode, reg, SIZE_BYTE, &ea)) return 8;
        if (ea.kind != EAK_Mem) { exception_process(cpu, VEC_ILLEGAL_INSN); return 8; }
        s32 bit_offset  = (s32)bf.offset;
        s32 byte_off    = bit_offset >> 3;
        u32 byte_addr   = ea.address + (u32)(s32)byte_off;
        u32 bit_in_byte = (u32)(bit_offset - (byte_off << 3));
        bf_mem_write(cpu, byte_addr, bit_in_byte, bf.width, ins);
    }
    return 8;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_bitfield_install_handlers(InsnHandler *t) {
    /*
     * All 8 bitfield instructions share the pattern 1110 1xxx 11 ea.
     * The 3 bits xxx identify the instruction (0-7).
     * Valid EA: Dn or memory control modes (not An, pre/post, imm).
     * For simplicity, install for all EA values and rely on ea_resolve
     * to reject invalid modes.
     */
    for (u32 ea = 0; ea < 64; ea++) {
        u8 mode = (ea >> 3) & 7u;
        if (mode == EA_MODE_AN) continue;  /* An not valid */

        t[0xE8C0u | ea] = handler_bftst;
        t[0xE9C0u | ea] = handler_bfextu;
        t[0xEAC0u | ea] = handler_bfchg;
        t[0xEBC0u | ea] = handler_bfexts;
        t[0xECC0u | ea] = handler_bfclr;
        t[0xEDC0u | ea] = handler_bfffo;
        t[0xEEC0u | ea] = handler_bfset;
        t[0xEFC0u | ea] = handler_bfins;
    }
}
