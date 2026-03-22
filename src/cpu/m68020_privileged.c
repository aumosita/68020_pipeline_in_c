/*
 * m68020_privileged.c — Privileged instructions.
 *
 * Phase 1: STOP, RESET, RTE, MOVEC (stubs for MOVEC/MOVES).
 * Phase 3 will add the full Format-B RTE and MOVES.
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* ------------------------------------------------------------------ */
/* Privilege check helper                                              */
/* ------------------------------------------------------------------ */

/* Returns true if we are in supervisor mode; otherwise raises PRIVILEGE. */
static bool require_supervisor(M68020State *cpu) {
    if (!SR_S(cpu->SR)) {
        exception_process(cpu, VEC_PRIVILEGE);
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* STOP #imm                                                           */
/* ------------------------------------------------------------------ */

static u32 handler_stop(M68020State *cpu, u16 opword) {
    (void)opword;
    if (!require_supervisor(cpu)) return 8;
    u16 imm = pipeline_consume_word(cpu);
    cpu_set_sr(cpu, imm);
    cpu->stopped = true;
    return 8;
}

/* ------------------------------------------------------------------ */
/* RESET                                                               */
/* ------------------------------------------------------------------ */

static u32 handler_reset(M68020State *cpu, u16 opword) {
    (void)opword;
    if (!require_supervisor(cpu)) return 4;
    if (cpu->bus.reset_peripherals)
        cpu->bus.reset_peripherals(cpu->bus.ctx);
    return 518;   /* 68020: 518 clock cycles */
}

/* ------------------------------------------------------------------ */
/* RTE — Return from Exception                                         */
/* ------------------------------------------------------------------ */
/*
 * Phase 1: handles Format 0 and Format 2 frames.
 * Phase 3 will add Format B (bus/address error).
 *
 * RTE reads the stack frame, validates the format word, restores SR
 * (and thus potentially switches stack pointers), then jumps to saved PC.
 */
static u32 handler_rte(M68020State *cpu, u16 opword) {
    (void)opword;
    if (!require_supervisor(cpu)) return 20;

    /* Read format/vector word from top of stack */
    u16 fmt_word = cpu_pop_word(cpu);
    u8  fmt      = fmt_word >> 12;

    /* Read return PC */
    u32 ret_pc = cpu_pop_long(cpu);

    /* Read saved SR */
    u16 saved_sr = cpu_pop_word(cpu);

    /* Consume any extra words for longer frame formats */
    switch (fmt) {
        case FRAME_FORMAT_0:
            /* nothing extra */
            break;
        case FRAME_FORMAT_2:
            /* 2 extra words (instruction PC) — discard */
            cpu->A[7] += 4;
            break;
        case FRAME_FORMAT_B:
            /* Format B: 46 words = 92 bytes total.
             * We already popped fmt_word(2) + ret_pc(4) + saved_sr(2) = 8 bytes.
             * Skip remaining 84 bytes of internal state. */
            cpu->A[7] += 84;
            break;
        default:
            /* Unknown format — format error exception */
            exception_process(cpu, VEC_FORMAT_ERR);
            return 20;
    }

    cpu_set_sr(cpu, saved_sr);
    pipeline_flush(cpu, ret_pc);
    cpu->PC = ret_pc;
    return 20;
}

/* ------------------------------------------------------------------ */
/* RTD — Return and Deallocate (68010+)                               */
/* ------------------------------------------------------------------ */

static u32 handler_rtd(M68020State *cpu, u16 opword) {
    (void)opword;
    s16 disp = (s16)pipeline_consume_word(cpu);
    u32 ret_pc = cpu_pop_long(cpu);
    cpu->A[7] += (s32)disp;
    pipeline_flush(cpu, ret_pc);
    cpu->PC = ret_pc;
    return 10;
}

/* ------------------------------------------------------------------ */
/* MOVEC — Move Control Register (68010+)                             */
/* ------------------------------------------------------------------ */

static u32 handler_movec(M68020State *cpu, u16 opword) {
    if (!require_supervisor(cpu)) return 12;

    bool to_ctrl = (opword & 1u) == 0;  /* 0x4E7A = to reg, 0x4E7B = from reg */
    u16  ext     = pipeline_consume_word(cpu);

    u8   gp_type = (ext >> 15) & 1u;   /* 0=Dn, 1=An */
    u8   gp_num  = (ext >> 12) & 7u;
    u16  ctrl_id = ext & 0x0FFFu;

    u32 *gp_reg = gp_type ? &cpu->A[gp_num] : &cpu->D[gp_num];

    if (to_ctrl) {
        /* MOVEC Rc,Rn — from control reg to general reg */
        switch (ctrl_id) {
            case 0x000: *gp_reg = cpu->SFC;  break;
            case 0x001: *gp_reg = cpu->DFC;  break;
            case 0x002: *gp_reg = cpu->CACR; break;
            case 0x800: *gp_reg = cpu->USP;  break;
            case 0x801: *gp_reg = cpu->VBR;  break;
            case 0x802: *gp_reg = cpu->CAAR; break;
            case 0x803: *gp_reg = cpu->MSP;  break;
            case 0x804: *gp_reg = cpu->ISP;  break;
            default:
                exception_process(cpu, VEC_ILLEGAL_INSN);
                return 12;
        }
    } else {
        /* MOVEC Rn,Rc — from general reg to control reg */
        switch (ctrl_id) {
            case 0x000: cpu->SFC  = *gp_reg & 7u; break;
            case 0x001: cpu->DFC  = *gp_reg & 7u; break;
            case 0x002: icache_update_cacr(cpu, *gp_reg); break;
            case 0x800: cpu->USP  = *gp_reg; break;
            case 0x801: cpu->VBR  = *gp_reg; break;
            case 0x802: cpu->CAAR = *gp_reg; break;
            case 0x803: cpu->MSP  = *gp_reg; break;
            case 0x804: cpu->ISP  = *gp_reg; break;
            default:
                exception_process(cpu, VEC_ILLEGAL_INSN);
                return 12;
        }
    }
    return 12;
}

/* ------------------------------------------------------------------ */
/* MOVES — Move to/from Alternate Address Space (68010+)              */
/* ------------------------------------------------------------------ */
/*
 * Encoding: 0000 1110 ss ea  (opword), followed by extension word:
 *   bit 15:    0=Dn, 1=An
 *   bits 14-12: register number
 *   bit 11:    dr (0 = EA → Rn  read from alt space,
 *                  1 = Rn → EA  write to alt space)
 *   bits 10-0: (reserved, must be 0)
 *
 * The bus access uses SFC (for reads) or DFC (for writes) as the
 * function code instead of the normal CPU data FC.
 */
static u32 handler_moves(M68020State *cpu, u16 opword) {
    if (!require_supervisor(cpu)) return 8;

    u8  ss  = (opword >> 6) & 3u;
    static const BusSize sz_tbl[4] = { SIZE_BYTE, SIZE_WORD, SIZE_LONG, SIZE_LONG };
    BusSize sz = sz_tbl[ss & 3u];

    u16 ext    = pipeline_consume_word(cpu);
    u8  is_an  = (ext >> 15) & 1u;
    u8  rn     = (ext >> 12) & 7u;
    u8  dr     = (ext >> 11) & 1u;   /* 0=read, 1=write */

    u8  ea_mode = EA_SRC_MODE(opword);
    u8  ea_reg  = EA_SRC_REG(opword);

    EADesc ea;
    if (!ea_resolve(cpu, ea_mode, ea_reg, sz, &ea)) return 8;
    if (ea.kind != EAK_Mem) {
        exception_process(cpu, VEC_ILLEGAL_INSN);
        return 8;
    }

    u32 addr = ea.address;

    if (dr == 0) {
        /* EA → Rn: read from alternate space (SFC) */
        u32 val = 0;
        u32 cyc = 0;
        BusResult res = cpu->bus.read(cpu->bus.ctx, addr, sz,
                                      (FunctionCode)(cpu->SFC & 7u), &val, &cyc);
        cpu->cycle_count += cyc;
        if (res != BUS_OK) {
            exception_bus_error(cpu, addr, false, 0);
            return 8;
        }
        if (sz == SIZE_WORD && is_an)
            val = (u32)(s32)(s16)(u16)val;  /* sign-extend .W for An */
        if (is_an) {
            cpu->A[rn] = val;
        } else {
            switch (sz) {
                case SIZE_BYTE: cpu->D[rn] = (cpu->D[rn] & 0xFFFFFF00u) | (val & 0xFFu);   break;
                case SIZE_WORD: cpu->D[rn] = (cpu->D[rn] & 0xFFFF0000u) | (val & 0xFFFFu); break;
                case SIZE_LONG: cpu->D[rn] = val; break;
            }
        }
    } else {
        /* Rn → EA: write to alternate space (DFC) */
        u32 val = is_an ? cpu->A[rn] : cpu->D[rn];
        u32 cyc = 0;
        BusResult res = cpu->bus.write(cpu->bus.ctx, addr, sz,
                                       (FunctionCode)(cpu->DFC & 7u), val, &cyc);
        cpu->cycle_count += cyc;
        if (res != BUS_OK) {
            exception_bus_error(cpu, addr, true, 0);
            return 8;
        }
    }
    return 8;
}

/* ------------------------------------------------------------------ */
/* Handler installer                                                   */
/* ------------------------------------------------------------------ */

void m68020_privileged_install_handlers(InsnHandler *t) {
    t[OP_STOP]  = handler_stop;
    t[OP_RESET] = handler_reset;
    t[OP_RTE]   = handler_rte;
    t[OP_RTD]   = handler_rtd;
    t[OP_MOVEC_CR] = handler_movec;
    t[OP_MOVEC_RC] = handler_movec;

    /* MOVES: 0000 1110 ss ea  (ss=00/01/10) */
    for (u32 ss = 0; ss <= 2; ss++)
        for (u32 ea = 0; ea < 64; ea++)
            t[0x0E00u | (ss << 6) | ea] = handler_moves;
}
