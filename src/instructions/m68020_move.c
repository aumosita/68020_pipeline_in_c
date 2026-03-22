/*
 * m68020_move.c — MOVE, MOVEA, MOVEQ, MOVEM, MOVEP, MOVE SR/CCR/USP.
 *
 * MOVE opcode map (bits [15:12]):
 *   0001 = MOVE.B
 *   0010 = MOVE.L / MOVEA.L
 *   0011 = MOVE.W / MOVEA.W
 *
 * MOVE instruction word:  00ss DDD ddd SSS sss
 *   ss     = size (01=byte, 10=long, 11=word)
 *   DDD    = destination register  (bits [11:9])
 *   ddd    = destination mode      (bits [8:6])
 *   SSS sss = source EA            (bits [5:0])
 *
 * MOVEA: destination mode = 001 (An).
 *
 * MOVEQ: 0111 DDD 0 data8   — sign-extends 8-bit immediate to D register.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* ------------------------------------------------------------------ */
/* MOVE.B / MOVE.W / MOVE.L                                           */
/* ------------------------------------------------------------------ */

static u32 do_move(M68020State *cpu, u16 opword, BusSize size) {
    u8 src_mode = EA_SRC_MODE(opword);
    u8 src_reg  = EA_SRC_REG(opword);
    u8 dst_mode = EA_DST_MODE(opword);
    u8 dst_reg  = EA_DST_REG(opword);

    /* Resolve source EA */
    EADesc src;
    if (!ea_resolve(cpu, src_mode, src_reg, size, &src))
        return 4;

    /* Resolve destination EA (MOVE dst uses [8:6]=mode, [11:9]=reg) */
    EADesc dst;
    if (!ea_resolve(cpu, dst_mode, dst_reg, size, &dst))
        return 4;

    /* Read source */
    u32 val = 0;
    if (!ea_read(cpu, &src, size, &val))
        return 4;

    /* MOVEA: address register — no CCR update, sign-extend .W */
    if (dst_mode == EA_MODE_AN) {
        if (size == SIZE_WORD)
            val = (u32)(s32)(s16)(u16)val;  /* sign-extend */
        cpu->A[dst_reg] = val;
        return 4;
    }

    /* Write destination */
    if (!ea_write(cpu, &dst, size, val))
        return 4;

    /* Update CCR: N, Z cleared; C=0, V=0 */
    u16 ccr = ccr_logic(val, size);
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;

    return 4;
}

static u32 handler_move_b(M68020State *cpu, u16 opword) {
    return do_move(cpu, opword, SIZE_BYTE);
}
static u32 handler_move_w(M68020State *cpu, u16 opword) {
    return do_move(cpu, opword, SIZE_WORD);
}
static u32 handler_move_l(M68020State *cpu, u16 opword) {
    return do_move(cpu, opword, SIZE_LONG);
}

/* ------------------------------------------------------------------ */
/* MOVEQ — Move Quick                                                  */
/* ------------------------------------------------------------------ */

static u32 handler_moveq(M68020State *cpu, u16 opword) {
    u8  dn  = (opword >> 9) & 7u;
    s32 val = (s32)(s8)(u8)(opword & 0xFFu);   /* sign-extend 8-bit */

    cpu->D[dn] = (u32)val;

    /* CCR: N, Z set from result; C=0, V=0 */
    u16 ccr = ccr_logic((u32)val, SIZE_LONG);
    cpu->SR = (cpu->SR & ~0x1Fu) | ccr;

    return 4;
}

/* ------------------------------------------------------------------ */
/* MOVEM — Move Multiple Registers                                     */
/* ------------------------------------------------------------------ */
/*
 * Opcode: 0100 1d00 1sss mmmm rrr    (d=direction, s=size, ea follows)
 * 0x4880 / 0x48C0 = MOVEM reg→mem (.W/.L)
 * 0x4C80 / 0x4CC0 = MOVEM mem→reg (.W/.L)
 *
 * Followed by a 16-bit register mask word.
 * For mem→reg:  mask bit 0 = D0 ... bit 15 = A7
 * For reg→mem with -(An): mask is reversed (bit 15 = D0, bit 0 = A7)
 */
/* MOVEM base cycles per EA mode (MC68020 User's Manual Table 9-17/9-18) */
static u32 movem_base(u8 mode, u8 reg, bool to_mem) {
    if (to_mem) {
        switch (mode) {
        case EA_MODE_IND:   return 8;
        case EA_MODE_PRE:   return 8;
        case EA_MODE_D16:   return 12;
        case EA_MODE_IDX:   return 14;
        case EA_MODE_EXT:
            switch (reg) {
            case EA_EXT_ABSW: return 12;
            case EA_EXT_ABSL: return 16;
            }
            break;
        }
    } else {
        switch (mode) {
        case EA_MODE_IND:   return 12;
        case EA_MODE_POST:  return 12;
        case EA_MODE_D16:   return 16;
        case EA_MODE_IDX:   return 18;
        case EA_MODE_EXT:
            switch (reg) {
            case EA_EXT_ABSW:   return 16;
            case EA_EXT_ABSL:   return 20;
            case EA_EXT_D16PC:  return 16;
            case EA_EXT_IDXPC:  return 18;
            }
            break;
        }
    }
    return 8;
}

static u32 handler_movem(M68020State *cpu, u16 opword) {
    bool to_mem = !(opword & 0x0400u);         /* 0 = reg→mem, 1 = mem→reg */
    BusSize size = (opword & 0x0040u) ? SIZE_LONG : SIZE_WORD;
    u8 ea_mode = EA_SRC_MODE(opword);
    u8 ea_reg  = EA_SRC_REG(opword);

    u16 mask = pipeline_consume_word(cpu);
    u32 cycles = movem_base(ea_mode, ea_reg, to_mem);

    if (to_mem) {
        /* Registers → Memory */
        /* For pre-decrement mode (-(An)), the mask is reversed */
        bool predec = (ea_mode == EA_MODE_PRE);
        u32  addr;

        if (predec) {
            /* -(An): mask is reversed — bit 0 = A7, bit 15 = D0.
             * Iterate from bit 0 (A7) upward; each register is pre-decremented
             * onto the stack. This pushes A7 first (highest addr) ... D0 last.
             * Map: bit i → register (15 - i) → 0=A7, 1=A6, ..., 15=D0 */
            addr = cpu->A[ea_reg];
            for (int i = 0; i < 16; i++) {
                if (!(mask & (1u << i))) continue;
                addr -= (u32)size;
                int r = 15 - i;
                u32 val = (r < 8) ? cpu->D[r] : cpu->A[r - 8];
                if (size == SIZE_WORD)
                    cpu_write_word(cpu, addr, (u16)val);
                else
                    cpu_write_long(cpu, addr, val);
                cycles += (u32)size;
            }
            cpu->A[ea_reg] = addr;
        } else {
            EADesc ea;
            if (!ea_resolve(cpu, ea_mode, ea_reg, size, &ea)) return 8;
            addr = ea.address;
            for (int i = 0; i < 16; i++) {
                if (!(mask & (1u << i))) continue;
                u32 val = (i < 8) ? cpu->D[i] : cpu->A[i - 8];
                if (size == SIZE_WORD)
                    cpu_write_word(cpu, addr, (u16)val);
                else
                    cpu_write_long(cpu, addr, val);
                addr += (u32)size;
                cycles += (u32)size;
            }
        }
    } else {
        /* Memory → Registers (mask: bit 0 = D0, bit 15 = A7) */
        EADesc ea;
        bool postinc = (ea_mode == EA_MODE_POST);
        u32  addr;

        if (postinc) {
            addr = cpu->A[ea_reg];
        } else {
            if (!ea_resolve(cpu, ea_mode, ea_reg, size, &ea)) return 8;
            addr = ea.address;
        }

        for (int i = 0; i < 16; i++) {
            if (!(mask & (1u << i))) continue;
            u32 val = 0;
            if (size == SIZE_WORD) {
                u16 w = 0;
                cpu_read_word(cpu, addr, &w);
                val = (u32)(s32)(s16)w;  /* sign-extend */
            } else {
                cpu_read_long(cpu, addr, &val);
            }
            if (i < 8) cpu->D[i] = val;
            else        cpu->A[i - 8] = val;
            addr += (u32)size;
            cycles += (u32)size;
        }

        if (postinc)
            cpu->A[ea_reg] = addr;
    }

    return cycles;
}

/* ------------------------------------------------------------------ */
/* MOVE to/from SR and CCR                                             */
/* ------------------------------------------------------------------ */

/* MOVE <ea>,SR  (0x46C0 | ea) — supervisor only */
static u32 handler_move_to_sr(M68020State *cpu, u16 opword) {
    if (!SR_S(cpu->SR)) {
        exception_process(cpu, VEC_PRIVILEGE);
        return 12;
    }
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &src))
        return 12;
    u32 val;
    if (!ea_read(cpu, &src, SIZE_WORD, &val)) return 12;
    cpu_set_sr(cpu, (u16)val);
    return 12;
}

/* MOVE <ea>,CCR  (0x44C0 | ea) — user ok */
static u32 handler_move_to_ccr(M68020State *cpu, u16 opword) {
    EADesc src;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &src))
        return 12;
    u32 val;
    if (!ea_read(cpu, &src, SIZE_WORD, &val)) return 12;
    cpu->SR = (cpu->SR & 0xFF00u) | (val & 0x1Fu);
    return 12;
}

/* MOVE SR,<ea>  (0x40C0 | ea) — on 68020 this is supervisor-only */
static u32 handler_move_from_sr(M68020State *cpu, u16 opword) {
    /* On 68010+, MOVE SR,<ea> is privileged */
    if (!SR_S(cpu->SR)) {
        exception_process(cpu, VEC_PRIVILEGE);
        return 8;
    }
    EADesc dst;
    if (!ea_resolve(cpu, EA_SRC_MODE(opword), EA_SRC_REG(opword), SIZE_WORD, &dst))
        return 8;
    if (!ea_write(cpu, &dst, SIZE_WORD, cpu->SR)) return 8;
    return 8;
}

/* MOVE USP,An / MOVE An,USP  (0x4E60 / 0x4E68) */
static u32 handler_move_usp(M68020State *cpu, u16 opword) {
    if (!SR_S(cpu->SR)) {
        exception_process(cpu, VEC_PRIVILEGE);
        return 4;
    }
    u8 reg = opword & 7u;
    if (opword & 8u) {
        /* MOVE USP,An */
        cpu->A[reg] = cpu->USP;
    } else {
        /* MOVE An,USP */
        cpu->USP = cpu->A[reg];
    }
    return 4;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_move_install_handlers(InsnHandler *t) {
    /* MOVEQ: 0111 DDD 0 imm8  (bit 8 must be 0) */
    for (u32 op = 0x7000u; op <= 0x7FFFu; op++) {
        if (!(op & 0x0100u))   /* bit 8 = 0 */
            t[op] = handler_moveq;
    }

    /* MOVE.B (0x1000-0x1FFF), MOVE.L (0x2000-0x2FFF), MOVE.W (0x3000-0x3FFF) */
    for (u32 op = 0x1000u; op <= 0x1FFFu; op++) t[op] = handler_move_b;
    for (u32 op = 0x2000u; op <= 0x2FFFu; op++) t[op] = handler_move_l;
    for (u32 op = 0x3000u; op <= 0x3FFFu; op++) t[op] = handler_move_w;

    /* MOVEM — reg→mem: 0x48 0s ea  (s=0 → .W, s=1 → .L) */
    for (u32 op = 0x4880u; op <= 0x48BFu; op++) t[op] = handler_movem;  /* .W, various EA */
    for (u32 op = 0x48C0u; op <= 0x48FFu; op++) t[op] = handler_movem;  /* .L */
    /* MOVEM — mem→reg: 0x4C 8s ea */
    for (u32 op = 0x4C80u; op <= 0x4CBFu; op++) t[op] = handler_movem;
    for (u32 op = 0x4CC0u; op <= 0x4CFFu; op++) t[op] = handler_movem;

    /* MOVE SR / CCR */
    for (u32 op = 0x40C0u; op <= 0x40FFu; op++) t[op] = handler_move_from_sr;
    for (u32 op = 0x44C0u; op <= 0x44FFu; op++) t[op] = handler_move_to_ccr;
    for (u32 op = 0x46C0u; op <= 0x46FFu; op++) t[op] = handler_move_to_sr;

    /* MOVE USP */
    for (u32 op = 0x4E60u; op <= 0x4E6Fu; op++) t[op] = handler_move_usp;
}
