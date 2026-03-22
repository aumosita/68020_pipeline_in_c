/*
 * m68020_disasm.c — MC68020 instruction disassembler.
 *
 * Decodes a 16-bit opword (and extension words from memory) into a
 * human-readable mnemonic string.  Used by the trace module and
 * debuggers.
 *
 * API:
 *   int m68020_disasm(const u8 *mem, u32 pc, char *buf, int buflen);
 *   Returns the total instruction length in bytes.
 */

#include "m68020_internal.h"
#include "m68020_opcodes.h"
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static u16 rd16(const u8 *p) { return (u16)((p[0]<<8)|p[1]); }
static u32 rd32(const u8 *p) { return ((u32)p[0]<<24)|((u32)p[1]<<16)|((u32)p[2]<<8)|p[3]; }

static const char *cc_names[16] = {
    "T","F","HI","LS","CC","CS","NE","EQ",
    "VC","VS","PL","MI","GE","LT","GT","LE"
};

static const char *sz_suffix[4] = { ".B", ".W", ".L", ".L" };

/*
 * Format an effective address. Returns bytes consumed from extension words.
 * p points to the first extension word after the opword.
 */
static int fmt_ea(char *out, int outlen, u8 mode, u8 reg,
                  u8 sz, const u8 *ext) {
    int eaten = 0;
    switch (mode) {
    case 0: snprintf(out, outlen, "D%u", reg); break;
    case 1: snprintf(out, outlen, "A%u", reg); break;
    case 2: snprintf(out, outlen, "(A%u)", reg); break;
    case 3: snprintf(out, outlen, "(A%u)+", reg); break;
    case 4: snprintf(out, outlen, "-(A%u)", reg); break;
    case 5: {
        s16 d = (s16)rd16(ext); eaten = 2;
        snprintf(out, outlen, "(%d,A%u)", d, reg);
        break;
    }
    case 6: {
        /* Brief or full extension word — just show brief format */
        u16 ew = rd16(ext); eaten = 2;
        u8 xr = (ew >> 12) & 0xF;
        s8 d8 = (s8)(ew & 0xFF);
        char xtype = (ew & 0x8000) ? 'A' : 'D';
        const char *xsz = (ew & 0x0800) ? ".L" : ".W";
        snprintf(out, outlen, "(%d,A%u,%c%u%s)", d8, reg,
                 xtype, xr & 7, xsz);
        break;
    }
    case 7:
        switch (reg) {
        case 0: {
            s16 a = (s16)rd16(ext); eaten = 2;
            snprintf(out, outlen, "$%04X.W", (u16)a);
            break;
        }
        case 1: {
            u32 a = rd32(ext); eaten = 4;
            snprintf(out, outlen, "$%08X.L", a);
            break;
        }
        case 2: {
            s16 d = (s16)rd16(ext); eaten = 2;
            snprintf(out, outlen, "(%d,PC)", d);
            break;
        }
        case 3: {
            u16 ew = rd16(ext); eaten = 2;
            s8 d8 = (s8)(ew & 0xFF);
            char xtype = (ew & 0x8000) ? 'A' : 'D';
            u8 xr = (ew >> 12) & 7;
            snprintf(out, outlen, "(%d,PC,%c%u)", d8, xtype, xr);
            break;
        }
        case 4:
            if (sz == 0) { /* byte */
                snprintf(out, outlen, "#$%02X", ext[1]); eaten = 2;
            } else if (sz == 1) { /* word */
                snprintf(out, outlen, "#$%04X", rd16(ext)); eaten = 2;
            } else { /* long */
                snprintf(out, outlen, "#$%08X", rd32(ext)); eaten = 4;
            }
            break;
        default:
            snprintf(out, outlen, "???");
            break;
        }
        break;
    default:
        snprintf(out, outlen, "???");
        break;
    }
    return eaten;
}

/* ------------------------------------------------------------------ */
/* Main disassembler                                                   */
/* ------------------------------------------------------------------ */

int m68020_disasm(const u8 *mem, u32 pc, char *buf, int buflen) {
    u16 op = rd16(mem);
    const u8 *ext = mem + 2;
    int len = 2; /* minimum instruction length */
    char ea_str[64];
    int ea_len;

    u8 grp = (op >> 12) & 0xF;

    switch (grp) {

    /* ---- Group 0: Immediate / Bit ops ---- */
    case 0x0: {
        u8 upper = (op >> 8) & 0xF;
        if (upper == 0x0 && (op & 0xC0) != 0xC0) {
            /* ORI */
            u8 ss = (op >> 6) & 3;
            u16 imm = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "ORI%s   #$%X,%s", sz_suffix[ss], imm, ea_str);
        } else if (upper == 0x2 && (op & 0xC0) != 0xC0) {
            u8 ss = (op >> 6) & 3;
            u16 imm = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "ANDI%s  #$%X,%s", sz_suffix[ss], imm, ea_str);
        } else if (upper == 0x4 && (op & 0xC0) != 0xC0) {
            u8 ss = (op >> 6) & 3;
            u16 imm = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "SUBI%s  #$%X,%s", sz_suffix[ss], imm, ea_str);
        } else if (upper == 0x6 && (op & 0xC0) != 0xC0) {
            u8 ss = (op >> 6) & 3;
            u16 imm = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "ADDI%s  #$%X,%s", sz_suffix[ss], imm, ea_str);
        } else if (upper == 0xA && (op & 0xC0) != 0xC0) {
            u8 ss = (op >> 6) & 3;
            u16 imm = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "EORI%s  #$%X,%s", sz_suffix[ss], imm, ea_str);
        } else if (upper == 0xC && (op & 0xC0) != 0xC0) {
            u8 ss = (op >> 6) & 3;
            u16 imm = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "CMPI%s  #$%X,%s", sz_suffix[ss], imm, ea_str);
        } else {
            snprintf(buf, buflen, "DC.W    $%04X", op);
        }
        break;
    }

    /* ---- Group 1-3: MOVE ---- */
    case 0x1: case 0x2: case 0x3: {
        const char *msz = (grp == 1) ? ".B" : (grp == 3) ? ".W" : ".L";
        u8 ssz = (grp == 1) ? 0 : (grp == 3) ? 1 : 2;
        u8 dst_reg = (op >> 9) & 7;
        u8 dst_mode = (op >> 6) & 7;
        u8 src_mode = EA_SRC_MODE(op);
        u8 src_reg  = EA_SRC_REG(op);

        ea_len = fmt_ea(ea_str, sizeof ea_str, src_mode, src_reg, ssz, ext);
        len += ea_len;

        char dst_str[64];
        int dst_len = fmt_ea(dst_str, sizeof dst_str, dst_mode, dst_reg, ssz, ext + ea_len);
        len += dst_len;

        if (dst_mode == 1)
            snprintf(buf, buflen, "MOVEA%s %s,A%u", msz, ea_str, dst_reg);
        else
            snprintf(buf, buflen, "MOVE%s  %s,%s", msz, ea_str, dst_str);
        break;
    }

    /* ---- Group 4: Miscellaneous ---- */
    case 0x4: {
        if (op == OP_NOP)       { snprintf(buf, buflen, "NOP"); break; }
        if (op == OP_RTS)       { snprintf(buf, buflen, "RTS"); break; }
        if (op == OP_RTR)       { snprintf(buf, buflen, "RTR"); break; }
        if (op == OP_RTE)       { snprintf(buf, buflen, "RTE"); break; }
        if (op == OP_TRAPV)     { snprintf(buf, buflen, "TRAPV"); break; }
        if (op == OP_RESET)     { snprintf(buf, buflen, "RESET"); break; }
        if (op == OP_ILLEGAL)   { snprintf(buf, buflen, "ILLEGAL"); break; }
        if (op == OP_STOP)      {
            u16 imm = rd16(ext); len += 2;
            snprintf(buf, buflen, "STOP    #$%04X", imm);
            break;
        }
        if (op == OP_MOVEC_CR || op == OP_MOVEC_RC) {
            u16 ew = rd16(ext); len += 2;
            snprintf(buf, buflen, "MOVEC   $%04X", ew);
            break;
        }

        /* LEA */
        if ((op & 0xF1C0) == 0x41C0) {
            u8 an = (op >> 9) & 7;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 2, ext);
            len += ea_len;
            snprintf(buf, buflen, "LEA     %s,A%u", ea_str, an);
            break;
        }
        /* JMP */
        if ((op & 0xFFC0) == 0x4EC0) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 2, ext);
            len += ea_len;
            snprintf(buf, buflen, "JMP     %s", ea_str);
            break;
        }
        /* JSR */
        if ((op & 0xFFC0) == 0x4E80) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 2, ext);
            len += ea_len;
            snprintf(buf, buflen, "JSR     %s", ea_str);
            break;
        }
        /* CLR */
        if ((op & 0xFF00) == 0x4200) {
            u8 ss = (op >> 6) & 3;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "CLR%s   %s", sz_suffix[ss], ea_str);
            break;
        }
        /* NEG */
        if ((op & 0xFF00) == 0x4400) {
            u8 ss = (op >> 6) & 3;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "NEG%s   %s", sz_suffix[ss], ea_str);
            break;
        }
        /* NOT */
        if ((op & 0xFF00) == 0x4600) {
            u8 ss = (op >> 6) & 3;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "NOT%s   %s", sz_suffix[ss], ea_str);
            break;
        }
        /* TST */
        if ((op & 0xFF00) == 0x4A00) {
            u8 ss = (op >> 6) & 3;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "TST%s   %s", sz_suffix[ss], ea_str);
            break;
        }
        /* SWAP */
        if ((op & 0xFFF8) == OP_SWAP) {
            snprintf(buf, buflen, "SWAP    D%u", op & 7);
            break;
        }
        /* EXT */
        if ((op & 0xFFF8) == OP_EXT_W) {
            snprintf(buf, buflen, "EXT.W   D%u", op & 7); break;
        }
        if ((op & 0xFFF8) == OP_EXT_L) {
            snprintf(buf, buflen, "EXT.L   D%u", op & 7); break;
        }
        if ((op & 0xFFF8) == OP_EXTB_L) {
            snprintf(buf, buflen, "EXTB.L  D%u", op & 7); break;
        }
        /* LINK.W */
        if ((op & 0xFFF8) == OP_LINK_W) {
            s16 d = (s16)rd16(ext); len += 2;
            snprintf(buf, buflen, "LINK.W  A%u,#%d", op & 7, d);
            break;
        }
        /* UNLK */
        if ((op & 0xFFF8) == OP_UNLK) {
            snprintf(buf, buflen, "UNLK    A%u", op & 7); break;
        }
        /* PEA */
        if ((op & 0xFFC0) == 0x4840) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 2, ext);
            len += ea_len;
            snprintf(buf, buflen, "PEA     %s", ea_str);
            break;
        }
        /* MOVEM */
        if ((op & 0xFB80) == 0x4880) {
            bool to_mem = !(op & 0x0400);
            u8 ss = (op & 0x0040) ? 2 : 1;
            u16 mask = rd16(ext); len += 2;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext+2);
            len += ea_len;
            snprintf(buf, buflen, "MOVEM%s %s,$%04X", sz_suffix[ss],
                     to_mem ? "regs" : ea_str, mask);
            break;
        }
        /* MOVE from SR */
        if ((op & 0xFFC0) == 0x40C0) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 1, ext);
            len += ea_len;
            snprintf(buf, buflen, "MOVE    SR,%s", ea_str);
            break;
        }
        /* MOVE to SR */
        if ((op & 0xFFC0) == 0x46C0) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 1, ext);
            len += ea_len;
            snprintf(buf, buflen, "MOVE    %s,SR", ea_str);
            break;
        }
        /* TRAP */
        if ((op & 0xFFF0) == 0x4E40) {
            snprintf(buf, buflen, "TRAP    #%u", op & 0xF);
            break;
        }

        snprintf(buf, buflen, "DC.W    $%04X", op);
        break;
    }

    /* ---- Group 5: ADDQ/SUBQ/Scc/DBcc ---- */
    case 0x5: {
        u8 ss = (op >> 6) & 3;
        if (ss != 3) {
            /* ADDQ / SUBQ */
            u8 data = (op >> 9) & 7; if (data == 0) data = 8;
            bool is_sub = (op >> 8) & 1;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "%s%s  #%u,%s",
                     is_sub ? "SUBQ" : "ADDQ", sz_suffix[ss], data, ea_str);
        } else {
            u8 cc = (op >> 8) & 0xF;
            if (EA_SRC_MODE(op) == 1) {
                /* DBcc */
                s16 d = (s16)rd16(ext); len += 2;
                snprintf(buf, buflen, "DB%s    D%u,$%X",
                         cc_names[cc], EA_SRC_REG(op), pc + 2 + d);
            } else if ((op & 0x38) == 0x38 && (op & 7) >= 2) {
                /* TRAPcc */
                snprintf(buf, buflen, "TRAP%s", cc_names[cc]);
            } else {
                /* Scc */
                ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 0, ext);
                len += ea_len;
                snprintf(buf, buflen, "S%s     %s", cc_names[cc], ea_str);
            }
        }
        break;
    }

    /* ---- Group 6: Bcc/BRA/BSR ---- */
    case 0x6: {
        u8 cc = (op >> 8) & 0xF;
        s32 disp;
        u8 d8 = op & 0xFF;
        if (d8 == 0x00) {
            disp = (s16)rd16(ext); len += 2;
        } else if (d8 == 0xFF) {
            disp = (s32)rd32(ext); len += 4;
        } else {
            disp = (s32)(s8)d8;
        }
        u32 target = pc + 2 + disp;
        if (cc == 0)
            snprintf(buf, buflen, "BRA     $%08X", target);
        else if (cc == 1)
            snprintf(buf, buflen, "BSR     $%08X", target);
        else
            snprintf(buf, buflen, "B%s     $%08X", cc_names[cc], target);
        break;
    }

    /* ---- Group 7: MOVEQ ---- */
    case 0x7: {
        u8 dn = (op >> 9) & 7;
        s8 data = (s8)(op & 0xFF);
        snprintf(buf, buflen, "MOVEQ   #%d,D%u", data, dn);
        break;
    }

    /* ---- Group 8: OR/DIV/SBCD ---- */
    case 0x8: {
        u8 ss = (op >> 6) & 3;
        if (ss == 3) {
            /* DIVU/DIVS */
            bool is_signed = (op >> 8) & 1;
            u8 dn = (op >> 9) & 7;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 1, ext);
            len += ea_len;
            snprintf(buf, buflen, "%s.W  %s,D%u",
                     is_signed ? "DIVS" : "DIVU", ea_str, dn);
        } else {
            u8 dn = (op >> 9) & 7;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "OR%s    %s,D%u", sz_suffix[ss], ea_str, dn);
        }
        break;
    }

    /* ---- Group 9: SUB ---- */
    case 0x9: {
        u8 ss = (op >> 6) & 3;
        u8 dn = (op >> 9) & 7;
        u8 dir = (op >> 8) & 1;
        if (ss == 3) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), dir ? 2 : 1, ext);
            len += ea_len;
            snprintf(buf, buflen, "SUBA%s  %s,A%u", dir ? ".L" : ".W", ea_str, dn);
        } else {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "SUB%s   %s,D%u", sz_suffix[ss], ea_str, dn);
        }
        break;
    }

    /* ---- Group A: Line-A ---- */
    case 0xA:
        snprintf(buf, buflen, "DC.W    $%04X  ; Line-A", op);
        break;

    /* ---- Group B: CMP/EOR ---- */
    case 0xB: {
        u8 ss = (op >> 6) & 3;
        u8 dn = (op >> 9) & 7;
        if (ss == 3) {
            u8 sz2 = ((op >> 8) & 1) ? 2 : 1;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), sz2, ext);
            len += ea_len;
            snprintf(buf, buflen, "CMPA%s  %s,A%u", sz2==2?".L":".W", ea_str, dn);
        } else {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "CMP%s   %s,D%u", sz_suffix[ss], ea_str, dn);
        }
        break;
    }

    /* ---- Group C: AND/MUL/ABCD/EXG ---- */
    case 0xC: {
        u8 ss = (op >> 6) & 3;
        if (ss == 3) {
            bool is_signed = (op >> 8) & 1;
            u8 dn = (op >> 9) & 7;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), 1, ext);
            len += ea_len;
            snprintf(buf, buflen, "%s.W  %s,D%u",
                     is_signed ? "MULS" : "MULU", ea_str, dn);
        } else {
            u8 dn = (op >> 9) & 7;
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "AND%s   %s,D%u", sz_suffix[ss], ea_str, dn);
        }
        break;
    }

    /* ---- Group D: ADD ---- */
    case 0xD: {
        u8 ss = (op >> 6) & 3;
        u8 dn = (op >> 9) & 7;
        u8 dir = (op >> 8) & 1;
        if (ss == 3) {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), dir ? 2 : 1, ext);
            len += ea_len;
            snprintf(buf, buflen, "ADDA%s  %s,A%u", dir ? ".L" : ".W", ea_str, dn);
        } else {
            ea_len = fmt_ea(ea_str, sizeof ea_str, EA_SRC_MODE(op), EA_SRC_REG(op), ss, ext);
            len += ea_len;
            snprintf(buf, buflen, "ADD%s   %s,D%u", sz_suffix[ss], ea_str, dn);
        }
        break;
    }

    /* ---- Group E: Shifts / Bitfields ---- */
    case 0xE: {
        static const char *sh_names[4] = { "AS", "LS", "ROX", "RO" };
        u8 ss = (op >> 6) & 3;
        if (ss == 3) {
            /* Memory shift or bitfield — simplified */
            snprintf(buf, buflen, "DC.W    $%04X  ; shift/bf", op);
        } else {
            u8 type = (op >> 3) & 3;
            u8 dir  = (op >> 8) & 1;
            u8 ir   = (op >> 5) & 1;
            u8 reg  = op & 7;
            u8 cnt  = (op >> 9) & 7;
            if (!ir) { if (cnt == 0) cnt = 8; }
            snprintf(buf, buflen, "%s%c%s  %s%u,D%u",
                     sh_names[type], dir ? 'L' : 'R', sz_suffix[ss],
                     ir ? "D" : "#", ir ? cnt : cnt, reg);
        }
        break;
    }

    /* ---- Group F: Line-F / Coprocessor ---- */
    case 0xF:
        snprintf(buf, buflen, "DC.W    $%04X  ; Line-F", op);
        break;

    default:
        snprintf(buf, buflen, "DC.W    $%04X", op);
        break;
    }

    return len;
}
