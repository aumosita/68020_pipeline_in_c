/*
 * m68020_core.c — Main execution loop and dispatch table.
 *
 * Architecture:
 *   • g_dispatch[] is a flat 65536-entry function-pointer table indexed
 *     by the full 16-bit opword.  It is built once at startup.
 *   • m68020_step() uses setjmp/longjmp to catch any exception that
 *     occurs mid-instruction.  On exception, the CPU state (PC, SR)
 *     has already been updated by exception_process(); we just return.
 *   • Interrupt checking happens at the start of each instruction decode.
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"
#include "m68020_opcodes.h"

/* ------------------------------------------------------------------ */
/* Global dispatch table                                               */
/* ------------------------------------------------------------------ */

InsnHandler g_dispatch[65536];

/* ------------------------------------------------------------------ */
/* Illegal-instruction handler (fills all un-assigned slots)           */
/* ------------------------------------------------------------------ */

static u32 handler_illegal(M68020State *cpu, u16 opword) {
    (void)opword;
    /* Line A and Line F have dedicated vectors */
    u8 top = opword >> 12;
    if (top == 0xA)
        exception_process(cpu, VEC_LINE_A);
    else if (top == 0xF)
        exception_process(cpu, VEC_LINE_F);
    else
        exception_process(cpu, VEC_ILLEGAL_INSN);
    return 4;  /* not reached after longjmp, but silences compiler */
}

/* ------------------------------------------------------------------ */
/* Build the dispatch table                                            */
/* ------------------------------------------------------------------ */

void m68020_build_dispatch_table(void) {
    /* Fill everything with the illegal handler first */
    for (int i = 0; i < 65536; i++)
        g_dispatch[i] = handler_illegal;

    /* Install handlers from each instruction module */
    m68020_move_install_handlers    (g_dispatch);
    m68020_branch_install_handlers  (g_dispatch);
    m68020_jump_install_handlers    (g_dispatch);
    m68020_alu_install_handlers     (g_dispatch);
    m68020_shift_install_handlers   (g_dispatch);
    m68020_mul_div_install_handlers (g_dispatch);
    m68020_load_store_install_handlers(g_dispatch);
    m68020_compare_install_handlers (g_dispatch);
    m68020_cas_install_handlers     (g_dispatch);
    m68020_misc_install_handlers    (g_dispatch);
    m68020_bitfield_install_handlers(g_dispatch);
    m68020_callm_rtm_install_handlers(g_dispatch);
    m68020_coproc_install_handlers  (g_dispatch);
    m68020_privileged_install_handlers(g_dispatch);
}

/* ------------------------------------------------------------------ */
/* Single-step execution                                               */
/* ------------------------------------------------------------------ */

u32 m68020_step(M68020State *cpu) {
    if (cpu->halted)
        return 1;

    if (cpu->stopped) {
        /* STOP: wait for interrupt */
        if (cpu->pending_ipl > SR_IPL(cpu->SR)) {
            cpu->stopped = false;
            /* fall through to interrupt check below */
        } else {
            cpu->cycle_count += 2;
            return 2;
        }
    }

    /* Install exception-recovery setjmp.
     * longjmp val=1 → exception taken, continue from new PC
     * longjmp val=2 → CPU halted (double bus error)            */
    cpu->fault_jmp_active = true;
    int jval = setjmp(cpu->fault_jmp);
    if (jval == 2) {
        cpu->fault_jmp_active = false;
        cpu->halted = true;
        return 1;
    }
    if (jval == 1) {
        /* Exception was taken; CPU PC/SR already updated.
         * Deactivate guard and return — caller will call step() again. */
        cpu->fault_jmp_active = false;
        cpu->instr_count++;
        return 0;
    }

    /* Check for pending interrupts */
    if (cpu->pending_ipl > SR_IPL(cpu->SR)) {
        m68020_process_interrupt(cpu);
        cpu->fault_jmp_active = false;
        cpu->instr_count++;
        return 0;
    }

    /* Opportunistically fill the prefetch queue */
    pipeline_refill(cpu);

    /* Fire trace hook if installed */
    if (cpu->trace_hook)
        cpu->trace_hook(cpu, cpu->trace_hook_user);

    /* Record start cycle count to compute instruction cycles */
    u64 start_cycles = cpu->cycle_count;

    /* Fetch opword (consumes one word from the prefetch queue) */
    cpu->PC = pipeline_peek_pc(cpu);
    u16 opword = pipeline_consume_word(cpu);

    /* Handle deferred trace exception (fires at instruction boundary) */
    if (cpu->trace_pending) {
        cpu->trace_pending = false;
        if (SR_T1(cpu->SR) || SR_T0(cpu->SR))
            exception_process(cpu, VEC_TRACE);
    }

    /* Set new trace_pending based on current T bits */
    if (SR_T1(cpu->SR))
        cpu->trace_pending = true;  /* trace every instruction (T1) */
    /* T0 (branch trace) is handled in branch/jump handlers */

    /* Dispatch */
    u32 extra_cycles = g_dispatch[opword](cpu, opword);
    (void)extra_cycles;

    cpu->fault_jmp_active = false;
    cpu->instr_count++;

    return (u32)(cpu->cycle_count - start_cycles);
}

/* ------------------------------------------------------------------ */
/* Run loop                                                            */
/* ------------------------------------------------------------------ */

u64 m68020_run(M68020State *cpu, u64 cycle_budget) {
    u64 start = cpu->cycle_count;
    u64 end   = start + cycle_budget;

    while (cpu->cycle_count < end && !cpu->halted)
        m68020_step(cpu);

    return cpu->cycle_count - start;
}
