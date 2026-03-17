/*
 * m68020_ea.c — Effective Address resolver.
 *
 * Phase 1 implements the 11 classic (68000-compatible) addressing modes:
 *   Dn, An, (An), (An)+, -(An), (d16,An), (d8,An,Xn),
 *   (xxx).W, (xxx).L, (d16,PC), (d8,PC,Xn), #imm
 *
 * Phase 4 will add the 7 new 68020 memory-indirect modes (full ext word).
 *
 * Extension words are consumed from the prefetch queue via
 * pipeline_consume_word(), which automatically advances the PC tracking.
 * For (d16,PC) and (d8,PC,Xn), the relevant PC value is the address of
 * the first extension word — obtained via pipeline_peek_pc() BEFORE
 * consuming the word.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Byte-increment amount for (An)+ / -(An), accounting for A7 alignment */
static u32 ea_inc_amt(u8 reg, BusSize size) {
    if (size == SIZE_BYTE && reg == 7)
        return 2;   /* A7 always stays word-aligned */
    return (u32)size;
}

/*
 * Decode the index register value from a brief or full extension word.
 *   ext[15]    : 0=Dn, 1=An
 *   ext[14:12] : register number
 *   ext[11]    : 0=sign-extend word, 1=use full long
 *   ext[10:9]  : scale (00=×1, 01=×2, 10=×4, 11=×8)
 */
static s32 decode_index(M68020State *cpu, u16 ext) {
    u8  idx_type = (ext >> 15) & 1u;
    u8  idx_reg  = (ext >> 12) & 7u;
    u8  idx_size = (ext >> 11) & 1u;
    u8  scale    = (ext >>  9) & 3u;

    u32 raw = idx_type ? cpu->A[idx_reg] : cpu->D[idx_reg];
    s32 idx = idx_size ? (s32)raw : (s32)(s16)(u16)raw;
    return idx << scale;
}

/* ------------------------------------------------------------------ */
/* EA resolver                                                         */
/* ------------------------------------------------------------------ */

bool ea_resolve(M68020State *cpu, u8 mode, u8 reg,
                BusSize size, EADesc *out) {
    out->is_postinc  = false;
    out->postinc_reg = 0;
    out->postinc_amt = 0;

    switch (mode) {
    /* ---- Dn -------------------------------------------------------- */
    case EA_MODE_DN:
        out->kind = EAK_Dn;
        out->reg  = reg;
        return true;

    /* ---- An -------------------------------------------------------- */
    case EA_MODE_AN:
        out->kind = EAK_An;
        out->reg  = reg;
        return true;

    /* ---- (An) ------------------------------------------------------ */
    case EA_MODE_IND:
        out->kind    = EAK_Mem;
        out->address = cpu->A[reg];
        return true;

    /* ---- (An)+ ----------------------------------------------------- */
    case EA_MODE_POST:
        out->kind        = EAK_Mem;
        out->address     = cpu->A[reg];
        out->is_postinc  = true;
        out->postinc_reg = reg;
        out->postinc_amt = ea_inc_amt(reg, size);
        return true;

    /* ---- -(An) ----------------------------------------------------- */
    case EA_MODE_PRE:
        cpu->A[reg] -= ea_inc_amt(reg, size);
        out->kind    = EAK_Mem;
        out->address = cpu->A[reg];
        return true;

    /* ---- (d16,An) -------------------------------------------------- */
    case EA_MODE_D16: {
        s16 disp = (s16)pipeline_consume_word(cpu);
        out->kind    = EAK_Mem;
        out->address = cpu->A[reg] + (s32)disp;
        return true;
    }

    /* ---- (d8,An,Xn) / full extension word -------------------------- */
    case EA_MODE_IDX: {
        u32 ext_pc = pipeline_peek_pc(cpu);
        u16 ext    = pipeline_consume_word(cpu);

        if (ext & 0x0100u) {
            /*
             * Full extension word (bit 8 = 1) — 68020 only.
             * Phase 1: support base-displacement-only (no memory indirect).
             */
            bool bs = (ext >> 7) & 1u;  /* base suppress  */
            bool is = (ext >> 6) & 1u;  /* index suppress */
            u8   bds = (ext >> 4) & 3u; /* base displacement size */
            u8   iis = ext & 7u;        /* index/indirect select  */

            u32 base = bs ? 0 : cpu->A[reg];
            s32 idx  = is ? 0 : decode_index(cpu, ext);

            s32 bd = 0;
            if (bds == 2)
                bd = (s32)(s16)pipeline_consume_word(cpu);
            else if (bds == 3) {
                u16 hi = pipeline_consume_word(cpu);
                u16 lo = pipeline_consume_word(cpu);
                bd = (s32)((u32)hi << 16 | lo);
            }

            if (iis != 0) {
                /* Memory indirect modes (Phase 4).
                 *
                 * Pre-indexed  (iis=1,2,3): intermediate = base + bd + idx
                 *                            final = *intermediate + od
                 * Post-indexed (iis=5,6,7): intermediate = base + bd
                 *                            final = *intermediate + idx + od
                 * No-index     (iis=4):      same as post-indexed (IS=1, idx=0)
                 *
                 * Outer displacement size (iis & 3):
                 *   0,1 → null (od=0)   2 → word   3 → long
                 */
                u32 intermediate = (iis < 4)
                    ? (base + (u32)idx + (u32)bd)   /* pre-indexed */
                    : (base             + (u32)bd);  /* post-indexed / no-index */

                u32 ptr = 0;
                if (cpu_read_long(cpu, intermediate, &ptr) != BUS_OK) {
                    exception_bus_error(cpu, intermediate, false, 0);
                    return false;
                }

                /* Outer displacement — consumed AFTER memory read */
                s32 od = 0;
                switch (iis & 3u) {
                    case 2: od = (s32)(s16)pipeline_consume_word(cpu); break;
                    case 3: {
                        u16 hi = pipeline_consume_word(cpu);
                        u16 lo = pipeline_consume_word(cpu);
                        od = (s32)(((u32)hi << 16) | lo);
                        break;
                    }
                    default: break;  /* null */
                }

                out->kind    = EAK_Mem;
                out->address = (iis < 4)
                    ? (ptr + (u32)od)              /* pre-indexed */
                    : (ptr + (u32)idx + (u32)od);  /* post-indexed */
                return true;
            }

            out->kind    = EAK_Mem;
            out->address = base + (u32)idx + (u32)bd;
        } else {
            /* Brief extension word (bit 8 = 0): (d8,An,Xn) */
            s32 idx  = decode_index(cpu, ext);
            s8  disp = (s8)(ext & 0xFFu);
            out->kind    = EAK_Mem;
            out->address = cpu->A[reg] + idx + (s32)disp;
        }
        (void)ext_pc;
        return true;
    }

    /* ---- Extended modes (mode = 7) --------------------------------- */
    case EA_MODE_EXT:
        switch (reg) {

        /* (xxx).W */
        case EA_EXT_ABSW: {
            u16 w = pipeline_consume_word(cpu);
            out->kind    = EAK_Mem;
            out->address = (u32)(s32)(s16)w;
            return true;
        }

        /* (xxx).L */
        case EA_EXT_ABSL: {
            u16 hi = pipeline_consume_word(cpu);
            u16 lo = pipeline_consume_word(cpu);
            out->kind    = EAK_Mem;
            out->address = ((u32)hi << 16) | lo;
            return true;
        }

        /* (d16,PC) */
        case EA_EXT_D16PC: {
            u32 pc_of_ext = pipeline_peek_pc(cpu);
            s16 disp      = (s16)pipeline_consume_word(cpu);
            out->kind     = EAK_Mem;
            out->address  = pc_of_ext + (s32)disp;
            return true;
        }

        /* (d8,PC,Xn) or full extension word with PC base */
        case EA_EXT_IDXPC: {
            u32 pc_of_ext = pipeline_peek_pc(cpu);
            u16 ext       = pipeline_consume_word(cpu);

            if (ext & 0x0100u) {
                /* Full extension word — PC as base register */
                bool bs  = (ext >> 7) & 1u;
                bool is  = (ext >> 6) & 1u;
                u8   bds = (ext >> 4) & 3u;
                u8   iis = ext & 7u;

                u32 base = bs ? 0u : pc_of_ext;
                s32 idx2 = is ? 0 : decode_index(cpu, ext);

                s32 bd = 0;
                if (bds == 2)
                    bd = (s32)(s16)pipeline_consume_word(cpu);
                else if (bds == 3) {
                    u16 hi = pipeline_consume_word(cpu);
                    u16 lo = pipeline_consume_word(cpu);
                    bd = (s32)(((u32)hi << 16) | lo);
                }

                if (iis == 0) {
                    out->kind    = EAK_Mem;
                    out->address = base + (u32)idx2 + (u32)bd;
                    return true;
                }

                /* Memory indirect with PC base */
                u32 intermediate = (iis < 4)
                    ? (base + (u32)idx2 + (u32)bd)
                    : (base             + (u32)bd);

                u32 ptr = 0;
                if (cpu_read_long(cpu, intermediate, &ptr) != BUS_OK) {
                    exception_bus_error(cpu, intermediate, false, 0);
                    return false;
                }

                s32 od = 0;
                switch (iis & 3u) {
                    case 2: od = (s32)(s16)pipeline_consume_word(cpu); break;
                    case 3: {
                        u16 hi = pipeline_consume_word(cpu);
                        u16 lo = pipeline_consume_word(cpu);
                        od = (s32)(((u32)hi << 16) | lo);
                        break;
                    }
                    default: break;
                }

                out->kind    = EAK_Mem;
                out->address = (iis < 4)
                    ? (ptr + (u32)od)
                    : (ptr + (u32)idx2 + (u32)od);
                return true;
            } else {
                /* Brief extension word: (d8,PC,Xn) */
                s32 idx2 = decode_index(cpu, ext);
                s8  disp = (s8)(ext & 0xFFu);
                out->kind    = EAK_Mem;
                out->address = pc_of_ext + idx2 + (s32)disp;
                return true;
            }
        }

        /* #immediate */
        case EA_EXT_IMM: {
            out->kind = EAK_Imm;
            if (size == SIZE_BYTE) {
                /* Byte immediate: high byte of the word is 0, data is low byte */
                u16 w = pipeline_consume_word(cpu);
                out->imm_val = (u8)w;
            } else if (size == SIZE_WORD) {
                out->imm_val = pipeline_consume_word(cpu);
            } else {
                u16 hi = pipeline_consume_word(cpu);
                u16 lo = pipeline_consume_word(cpu);
                out->imm_val = ((u32)hi << 16) | lo;
            }
            return true;
        }

        default:
            /* Unrecognised extended mode */
            exception_process(cpu, VEC_ILLEGAL_INSN);
            return false;
        }

    default:
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return false;
    }
}

/* ------------------------------------------------------------------ */
/* EA read                                                             */
/* ------------------------------------------------------------------ */

bool ea_read(M68020State *cpu, const EADesc *ea, BusSize size, u32 *val) {
    bool ok = true;

    switch (ea->kind) {
    case EAK_Dn: {
        u32 mask = sz_mask(size);
        *val = cpu->D[ea->reg] & mask;
        break;
    }
    case EAK_An:
        /* Address register reads always return the full 32-bit value */
        *val = cpu->A[ea->reg];
        break;
    case EAK_Imm:
        *val = ea->imm_val;
        break;
    case EAK_Mem:
        switch (size) {
        case SIZE_BYTE: { u8  v = 0; ok = cpu_read_byte(cpu, ea->address, &v) == BUS_OK; *val = v; break; }
        case SIZE_WORD: { u16 v = 0; ok = cpu_read_word(cpu, ea->address, &v) == BUS_OK; *val = v; break; }
        case SIZE_LONG: ok = cpu_read_long(cpu, ea->address, val) == BUS_OK; break;
        }
        break;
    }

    if (ok && ea->is_postinc)
        cpu->A[ea->postinc_reg] += ea->postinc_amt;

    return ok;
}

/* ------------------------------------------------------------------ */
/* EA write                                                            */
/* ------------------------------------------------------------------ */

bool ea_write(M68020State *cpu, const EADesc *ea, BusSize size, u32 val) {
    bool ok = true;

    switch (ea->kind) {
    case EAK_Dn:
        /* Write only the low `size` bytes; upper bytes are preserved */
        switch (size) {
        case SIZE_BYTE: cpu->D[ea->reg] = (cpu->D[ea->reg] & 0xFFFFFF00u) | (val & 0xFFu);     break;
        case SIZE_WORD: cpu->D[ea->reg] = (cpu->D[ea->reg] & 0xFFFF0000u) | (val & 0xFFFFu);   break;
        case SIZE_LONG: cpu->D[ea->reg] = val;                                                   break;
        }
        break;
    case EAK_An:
        /* MOVEA always writes the full 32-bit value (sign-extended for .W) */
        cpu->A[ea->reg] = val;
        break;
    case EAK_Imm:
        /* Writing to an immediate is illegal */
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return false;
    case EAK_Mem:
        switch (size) {
        case SIZE_BYTE: ok = cpu_write_byte(cpu, ea->address, (u8) val) == BUS_OK; break;
        case SIZE_WORD: ok = cpu_write_word(cpu, ea->address, (u16)val) == BUS_OK; break;
        case SIZE_LONG: ok = cpu_write_long(cpu, ea->address,      val) == BUS_OK; break;
        }
        break;
    }

    if (ok && ea->is_postinc)
        cpu->A[ea->postinc_reg] += ea->postinc_amt;

    return ok;
}
