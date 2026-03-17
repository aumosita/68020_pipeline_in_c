/*
 * m68020_exceptions.c — Exception and interrupt processing.
 *
 * Phase 1 implements Format 0 (short) and Format 2 (instruction address)
 * stack frames, which cover:
 *   - Illegal instruction, privilege violation, trace, TRAP, TRAPV, CHK
 *   - Interrupts (autovector and vectored)
 *
 * Phase 3 will add Format B (bus/address error, 46-word frame).
 *
 * Stack frame layout (see m68020_exceptions.h for details):
 *
 *  Format 0 (4 words):  SP+0 = format/vec, SP+2..5 = PC, SP+6 = SR
 *  Format 2 (6 words):  same + SP+8..11 = instruction PC
 *
 * Push order (highest address first, so last push = lowest = SP):
 *   Format 0: push SR, push PC, push format_word
 *   Format 2: push instr_PC, push SR, push PC, push format_word
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"

/* ------------------------------------------------------------------ */
/* Condition test (used by Bcc, DBcc, Scc, TRAPcc)                    */
/* ------------------------------------------------------------------ */

bool m68020_test_cc(u16 sr, u8 cc) {
    bool n = SR_N(sr), z = SR_Z(sr), v = SR_V(sr), c = SR_C(sr);
    switch (cc & 0xFu) {
        case 0x0: return true;           /* T  */
        case 0x1: return false;          /* F  */
        case 0x2: return !c && !z;       /* HI */
        case 0x3: return  c ||  z;       /* LS */
        case 0x4: return !c;             /* CC */
        case 0x5: return  c;             /* CS */
        case 0x6: return !z;             /* NE */
        case 0x7: return  z;             /* EQ */
        case 0x8: return !v;             /* VC */
        case 0x9: return  v;             /* VS */
        case 0xA: return !n;             /* PL */
        case 0xB: return  n;             /* MI */
        case 0xC: return n == v;         /* GE */
        case 0xD: return n != v;         /* LT */
        case 0xE: return !z && (n == v); /* GT */
        case 0xF: return  z || (n != v); /* LE */
        default:  return false;
    }
}

/* ------------------------------------------------------------------ */
/* Core exception processing                                           */
/* ------------------------------------------------------------------ */

/*
 * exception_process — enter exception for the given vector number.
 *
 * This function:
 *   1. Saves current SR
 *   2. Enters supervisor mode (clears T1/T0, sets S)
 *   3. Switches stack pointer (ISP for interrupts, current SP otherwise)
 *   4. Pushes the appropriate stack frame
 *   5. Loads the new PC from the vector table
 *   6. Flushes the pipeline
 *   7. longjmp's back to m68020_step() so execution resumes at the handler
 */
void exception_process(M68020State *cpu, u32 vector) {
    u16 saved_sr = cpu->SR;
    u32 saved_pc = cpu->PC;

    bool was_in_exception = cpu->in_exception;
    cpu->in_exception = true;

    /* Determine new SR: enter supervisor, clear trace bits */
    u16 new_sr = (saved_sr | (1u << 13))       /* set S   */
               & ~((1u << 15) | (1u << 14));   /* clr T1/T0 */

    /* For interrupts (vectors 25-31): also clear M to use ISP */
    bool is_interrupt = (vector >= VEC_AUTOVEC_1 && vector <= VEC_AUTOVEC_7);
    if (is_interrupt)
        new_sr &= ~(1u << 12);   /* clear M */

    /* Switch stack pointers */
    if (!SR_S(saved_sr)) {
        /* User → Supervisor */
        cpu->USP  = cpu->A[7];
        cpu->A[7] = is_interrupt ? cpu->ISP : cpu->ISP; /* use ISP */
    } else if (is_interrupt && SR_M(saved_sr)) {
        /* Supervisor MSP → ISP (interrupt exception while in master mode) */
        cpu->MSP  = cpu->A[7];
        cpu->A[7] = cpu->ISP;
    }

    cpu->SR = new_sr & SR_VALID_MASK;

    /* Choose stack frame format.
     * Format 2 is used for instruction-related exceptions where the
     * faulting instruction address is useful to the handler.
     * Format 0 is used for interrupts and other exceptions.          */
    FrameFormat fmt;
    switch (vector) {
        case VEC_CHK:
        case VEC_TRAPV:
        case VEC_ILLEGAL_INSN:
        case VEC_PRIVILEGE:
        case VEC_LINE_A:
        case VEC_LINE_F:
            fmt = FRAME_FORMAT_2;
            break;
        default:
            fmt = FRAME_FORMAT_0;
            break;
    }

    u16 format_word = ((u16)fmt << 12) | (u16)((vector * 4u) & 0xFFFu);

    if (fmt == FRAME_FORMAT_2) {
        /*
         * Format 2 frame (MC68020 manual §8.3.1):
         *   SP+0:  format/vector word
         *   SP+2:  return PC (where RTE will resume execution)
         *   SP+6:  SR (saved status register)
         *   SP+8:  instruction PC (address of the faulting instruction)
         *
         * Return PC semantics:
         *   - CHK, TRAPV: return to the instruction AFTER the faulting one
         *     (the handler is expected to fix and resume; the fault is recoverable)
         *   - ILLEGAL, LINE_A, LINE_F, PRIVILEGE: return to the faulting instruction
         *     (so the handler can emulate or re-execute it after fixing privilege)
         */
        u32 return_pc;
        switch (vector) {
            case VEC_ILLEGAL_INSN:
            case VEC_LINE_A:
            case VEC_LINE_F:
            case VEC_PRIVILEGE:
                return_pc = saved_pc;              /* re-execute the faulting insn */
                break;
            default:
                return_pc = pipeline_peek_pc(cpu); /* continue after faulting insn */
                break;
        }
        cpu_push_long(cpu, saved_pc);    /* instruction PC (SP+8) */
        cpu_push_word(cpu, saved_sr);
        cpu_push_long(cpu, return_pc);   /* return PC   (SP+2)  */
        cpu_push_word(cpu, format_word);
    } else {
        /* Format 0 */
        cpu_push_word(cpu, saved_sr);
        cpu_push_long(cpu, saved_pc);
        cpu_push_word(cpu, format_word);
    }

    /* Read new PC from vector table: VBR + vector*4 */
    u32 vec_addr = cpu->VBR + (vector * 4u);
    u32 new_pc   = 0;
    if (cpu_read_long(cpu, vec_addr, &new_pc) != BUS_OK) {
        /* Bus error while reading vector: double fault → halt */
        cpu->halted = true;
        cpu->in_exception = was_in_exception;
        if (cpu->fault_jmp_active)
            longjmp(cpu->fault_jmp, 2);
        return;
    }

    pipeline_flush(cpu, new_pc);
    cpu->PC = new_pc;
    cpu->in_exception = false;

    /* Return to the step() dispatch loop */
    if (cpu->fault_jmp_active)
        longjmp(cpu->fault_jmp, 1);
}

/* ------------------------------------------------------------------ */
/* Interrupt handling                                                  */
/* ------------------------------------------------------------------ */

/*
 * Process a pending interrupt at the given priority level.
 * Called from the main execution loop when pending_ipl > SR_IPL(SR).
 */
void m68020_process_interrupt(M68020State *cpu) {
    u8 level = cpu->pending_ipl;
    if (level == 0 || level <= SR_IPL(cpu->SR))
        return;

    /* Update SR interrupt mask BEFORE the IACK cycle */
    cpu->SR = (cpu->SR & ~0x0700u) | ((u16)level << 8);

    /* Interrupt acknowledge cycle */
    u8 vec = 0;
    if (cpu->bus.iack)
        vec = cpu->bus.iack(cpu->bus.ctx, level);
    else
        vec = 0xFF;  /* autovector if no iack handler */

    u32 vector;
    if (vec == 0x00)
        vector = VEC_SPURIOUS;
    else if (vec == 0xFF)
        vector = VEC_AUTOVEC_1 + level - 1;
    else
        vector = vec;

    exception_process(cpu, vector);
}
