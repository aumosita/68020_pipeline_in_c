/*
 * m68020_coproc.c — Coprocessor interface (Line F instructions).
 *
 * All opcodes 0xF000..0xFFFF are handled by a single dispatcher.
 *
 * Opword layout:
 *   15:12 = 1111  (Line F marker)
 *   11:9  = CpID  (coprocessor slot 0–7)
 *    8:6  = type
 *            000 = cpGEN    — general coprocessor instruction
 *            001 = cpScc / cpDBcc / cpTRAPcc
 *            010 = cpBcc.W  — 16-bit branch displacement
 *            011 = cpBcc.L  — 32-bit branch displacement
 *            100 = cpSAVE   — save coprocessor state
 *            101 = cpRESTORE — restore coprocessor state
 *    5:0  = EA or condition code (type-specific)
 *
 * If no coprocessor is attached in the requested slot, VEC_LINE_F is raised.
 * This is the correct 68020 hardware behaviour and allows OS software-
 * emulation traps (e.g. NetBSD's FPU emulator uses Line F for 68881 opcodes).
 *
 * Branch displacements are measured from the address of the first extension
 * word (cpu->PC + 2), per the MC68020 User's Manual sections 9-14/9-15.
 */

#include "m68020_internal.h"
#include "m68020_ea.h"
#include "m68020_exceptions.h"

static u32 handler_coproc(M68020State *cpu, u16 opword) {
    u8 cpid = (opword >> 9) & 7u;
    u8 type = (opword >> 6) & 7u;
    u8 mode = (opword >> 3) & 7u;
    u8 reg  =  opword       & 7u;

    M68020Coprocessor *cp = cpu->coprocessors[cpid];

    if (!cp) {
        exception_process(cpu, VEC_LINE_F);
        return 4;
    }

    switch (type) {

    /* ---- cpGEN -------------------------------------------------------- */
    case 0: {
        u16 cmd = pipeline_consume_word(cpu);
        cp->execute(cp, cpu, cmd);
        return 4;
    }

    /* ---- cpScc / cpDBcc / cpTRAPcc ------------------------------------ */
    case 1: {
        u16  ext  = pipeline_consume_word(cpu);
        u8   cc   = ext & 0x3Fu;
        bool cond = cp->test_condition(cp, cc);

        /* cpDBcc: type=1, mode=001 (EA_MODE_AN bit pattern), reg=Dn */
        if (mode == EA_MODE_AN) {
            u16 disp_word = pipeline_consume_word(cpu);
            s16 disp      = (s16)disp_word;
            if (!cond) {
                u16 dn_low = (u16)(cpu->D[reg] & 0xFFFFu) - 1u;
                cpu->D[reg] = (cpu->D[reg] & 0xFFFF0000u) | dn_low;
                if (dn_low != 0xFFFFu) {
                    /* target measured from address of condition word */
                    u32 target = cpu->PC + 2u + (u32)(s32)disp;
                    pipeline_flush(cpu, target);
                }
            }
            return 10;
        }

        /* cpTRAPcc.W: mode=7, reg=2 — one 16-bit immediate word */
        if (mode == EA_MODE_EXT && reg == EA_EXT_D16PC) {
            pipeline_consume_word(cpu);
            if (cond) exception_process(cpu, VEC_TRAPV);
            return 4;
        }
        /* cpTRAPcc.L: mode=7, reg=3 — one 32-bit immediate */
        if (mode == EA_MODE_EXT && reg == EA_EXT_IDXPC) {
            pipeline_consume_word(cpu);
            pipeline_consume_word(cpu);
            if (cond) exception_process(cpu, VEC_TRAPV);
            return 4;
        }
        /* cpTRAPcc (no operand): mode=7, reg=4 */
        if (mode == EA_MODE_EXT && reg == EA_EXT_IMM) {
            if (cond) exception_process(cpu, VEC_TRAPV);
            return 4;
        }

        /* cpScc: write 0xFF (true) or 0x00 (false) to <ea> */
        EADesc ea;
        if (!ea_resolve(cpu, mode, reg, SIZE_BYTE, &ea)) return 4;
        ea_write(cpu, &ea, SIZE_BYTE, cond ? 0xFFu : 0x00u);
        return 6;
    }

    /* ---- cpBcc.W ------------------------------------------------------ */
    case 2: {
        u8   cc   = opword & 0x3Fu;
        bool cond = cp->test_condition(cp, cc);
        s16  disp = (s16)pipeline_consume_word(cpu);
        if (cond) {
            /* displacement from first extension word = cpu->PC + 2 */
            pipeline_flush(cpu, cpu->PC + 2u + (u32)(s32)disp);
            if (SR_T0(cpu->SR)) cpu->trace_pending = true;
        }
        return 10;
    }

    /* ---- cpBcc.L ------------------------------------------------------ */
    case 3: {
        u8   cc   = opword & 0x3Fu;
        bool cond = cp->test_condition(cp, cc);
        u16  hi   = pipeline_consume_word(cpu);
        u16  lo   = pipeline_consume_word(cpu);
        s32  disp = (s32)(((u32)hi << 16) | lo);
        if (cond) {
            pipeline_flush(cpu, cpu->PC + 2u + (u32)disp);
            if (SR_T0(cpu->SR)) cpu->trace_pending = true;
        }
        return 10;
    }

    /* ---- cpSAVE ------------------------------------------------------- */
    case 4: {
        EADesc ea;
        if (!ea_resolve(cpu, mode, reg, SIZE_LONG, &ea)) return 4;
        if (ea.kind != EAK_Mem) {
            exception_process(cpu, VEC_ILLEGAL_INSN);
            return 4;
        }
        cp->save(cp, cpu, ea.address);
        return 8;
    }

    /* ---- cpRESTORE ---------------------------------------------------- */
    case 5: {
        EADesc ea;
        if (!ea_resolve(cpu, mode, reg, SIZE_LONG, &ea)) return 4;
        if (ea.kind != EAK_Mem) {
            exception_process(cpu, VEC_ILLEGAL_INSN);
            return 4;
        }
        cp->restore(cp, cpu, ea.address);
        return 8;
    }

    default:
        exception_process(cpu, VEC_LINE_F);
        return 4;
    }
}

void m68020_coproc_install_handlers(InsnHandler *t) {
    for (u32 op = 0xF000u; op <= 0xFFFFu; op++)
        t[op] = handler_coproc;
}
