/*
 * m68020_cas.c — CAS and CAS2 (68020 compare-and-swap).
 *
 * CAS <Dc>,<Du>,<ea>
 *   Opword: 0000 1ss0 11 ea   (ss: 01=byte, 10=word, 11=long)
 *   Extword: 0000 000 Du 000 Dc  (bits 8:6 = Du, bits 2:0 = Dc)
 *
 *   Compare Dc with <ea>; if equal: <ea>←Du, Z=1
 *                          else:    Dc←<ea>, Z=0
 *   CCR reflects: N,Z,V,C as if CMP Dc,<ea> (destination minus source)
 *
 * CAS2 <Dc1>:<Dc2>,<Du1>:<Du2>,(<Rn1>):(<Rn2>)
 *   Opword: 0000 1ss0 1111 1100  (ss: 10=word=0x0CFC, 11=long=0x0EFC)
 *   Two extension words:
 *     word1: bit15=0/1 (Dn/An), bits14:12=Rn1, bits8:6=Du1, bits2:0=Dc1
 *     word2: bit15=0/1 (Dn/An), bits14:12=Rn2, bits8:6=Du2, bits2:0=Dc2
 *
 *   Compare *Rn1 with Dc1 AND *Rn2 with Dc2.
 *   If BOTH equal: *Rn1←Du1, *Rn2←Du2, Z=1
 *   Else:          Dc1←*Rn1, Dc2←*Rn2, Z=0
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

/* ------------------------------------------------------------------ */
/* CAS                                                                 */
/* ------------------------------------------------------------------ */

static u32 handler_cas(M68020State *cpu, u16 opword) {
    u8  ss  = (opword >> 9) & 3u;
    static const BusSize sz_tbl[4] = {
        SIZE_LONG, SIZE_BYTE, SIZE_WORD, SIZE_LONG
    };
    BusSize sz = sz_tbl[ss & 3u];

    u16 ext = pipeline_consume_word(cpu);
    u8  du  = (ext >> 6) & 7u;   /* update register  */
    u8  dc  = ext & 7u;          /* compare register */

    EADesc ea;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), sz, &ea)) return 16;
    if (ea.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 16;
    }

    u32 mem_val = 0;
    if (!ea_read(cpu, &ea, sz, &mem_val)) return 16;

    u32 m      = sz_mask(sz);
    u32 dc_val = cpu->D[dc] & m;
    u32 result = (mem_val - dc_val) & m;

    /* Set CCR as CMP Dc,<ea>: destination=mem_val, source=dc_val */
    u16 ccr = ccr_sub(dc_val, mem_val, result, sz);
    cpu->SR = (cpu->SR & ~0x0Fu) | (ccr & 0x0Fu);

    if (result == 0) {
        /* Equal: store Du → <ea> */
        ea_write(cpu, &ea, sz, cpu->D[du]);
    } else {
        /* Not equal: load <ea> → Dc (low part only) */
        switch (sz) {
            case SIZE_BYTE:
                cpu->D[dc] = (cpu->D[dc] & 0xFFFFFF00u) | (mem_val & 0xFFu);
                break;
            case SIZE_WORD:
                cpu->D[dc] = (cpu->D[dc] & 0xFFFF0000u) | (mem_val & 0xFFFFu);
                break;
            case SIZE_LONG:
                cpu->D[dc] = mem_val;
                break;
        }
    }
    return 16;
}

/* ------------------------------------------------------------------ */
/* CAS2                                                                */
/* ------------------------------------------------------------------ */

static u32 handler_cas2(M68020State *cpu, u16 opword) {
    BusSize sz = (opword == 0x0CFCu) ? SIZE_WORD : SIZE_LONG;

    u16 ext1 = pipeline_consume_word(cpu);
    u16 ext2 = pipeline_consume_word(cpu);

    u8  rn1_an = (ext1 >> 15) & 1u;
    u8  rn1    = (ext1 >> 12) & 7u;
    u8  du1    = (ext1 >>  6) & 7u;
    u8  dc1    = ext1 & 7u;

    u8  rn2_an = (ext2 >> 15) & 1u;
    u8  rn2    = (ext2 >> 12) & 7u;
    u8  du2    = (ext2 >>  6) & 7u;
    u8  dc2    = ext2 & 7u;

    u32 addr1 = rn1_an ? cpu->A[rn1] : cpu->D[rn1];
    u32 addr2 = rn2_an ? cpu->A[rn2] : cpu->D[rn2];

    u32 mem1 = 0, mem2 = 0;
    u32 m = sz_mask(sz);

    if (sz == SIZE_WORD) {
        u16 v1 = 0, v2 = 0;
        if (cpu_read_word(cpu, addr1, &v1) != BUS_OK) return 12;
        if (cpu_read_word(cpu, addr2, &v2) != BUS_OK) return 12;
        mem1 = v1; mem2 = v2;
    } else {
        if (cpu_read_long(cpu, addr1, &mem1) != BUS_OK) return 12;
        if (cpu_read_long(cpu, addr2, &mem2) != BUS_OK) return 12;
    }

    u32 dc1_val = cpu->D[dc1] & m;
    u32 dc2_val = cpu->D[dc2] & m;

    u32 cmp1 = (mem1 - dc1_val) & m;
    u32 cmp2 = (mem2 - dc2_val) & m;

    /* CCR: set based on first comparison (or combined) — use first pair */
    u16 ccr = ccr_sub(dc1_val, mem1, cmp1, sz);
    if (cmp1 == 0 && cmp2 != 0)
        ccr = ccr_sub(dc2_val, mem2, cmp2, sz);
    cpu->SR = (cpu->SR & ~0x0Fu) | (ccr & 0x0Fu);

    if (cmp1 == 0 && cmp2 == 0) {
        /* Both equal: store Du1→*Rn1, Du2→*Rn2 */
        if (sz == SIZE_WORD) {
            cpu_write_word(cpu, addr1, (u16)cpu->D[du1]);
            cpu_write_word(cpu, addr2, (u16)cpu->D[du2]);
        } else {
            cpu_write_long(cpu, addr1, cpu->D[du1]);
            cpu_write_long(cpu, addr2, cpu->D[du2]);
        }
    } else {
        /* Not equal: load *Rn1→Dc1, *Rn2→Dc2 */
        if (sz == SIZE_WORD) {
            cpu->D[dc1] = (cpu->D[dc1] & 0xFFFF0000u) | (mem1 & 0xFFFFu);
            cpu->D[dc2] = (cpu->D[dc2] & 0xFFFF0000u) | (mem2 & 0xFFFFu);
        } else {
            cpu->D[dc1] = mem1;
            cpu->D[dc2] = mem2;
        }
    }
    return 12;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_cas_install_handlers(InsnHandler *t) {
    /* CAS.B: 0000 1010 11 ea = 0x0AC0 | ea */
    for (u32 ea = 0; ea < 64; ea++) {
        u8 mode = (ea >> 3) & 7u;
        /* CAS valid EA: memory alterable (not Dn, An, PC-rel, imm) */
        if (mode == EA_MODE_DN || mode == EA_MODE_AN) continue;
        if (mode == EA_MODE_EXT && (ea & 7u) >= EA_EXT_D16PC) continue;
        t[0x0AC0u | ea] = handler_cas;   /* CAS.B */
        t[0x0CC0u | ea] = handler_cas;   /* CAS.W */
        t[0x0EC0u | ea] = handler_cas;   /* CAS.L */
    }

    /* CAS2: fixed opcodes */
    t[0x0CFCu] = handler_cas2;   /* CAS2.W */
    t[0x0EFCu] = handler_cas2;   /* CAS2.L */
}
