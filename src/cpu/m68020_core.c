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

/* IPL 7 is non-maskable (NMI): always recognized regardless of mask */
static inline bool interrupt_pending(const M68020State *cpu) {
    u8 ipl  = cpu->pending_ipl;
    u8 mask = SR_IPL(cpu->SR);
    return ipl > mask || ipl == 7;
}

u32 m68020_step(M68020State *cpu) {
    if (cpu->halted)
        return 1;

    if (cpu->stopped) {
        /* STOP: wait for interrupt */
        if (interrupt_pending(cpu)) {
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
        cpu->in_exception = false;
        cpu->instr_count++;
        return 0;
    }

    /* Check for pending interrupts */
    if (interrupt_pending(cpu)) {
        m68020_process_interrupt(cpu);
        cpu->fault_jmp_active = false;
        cpu->instr_count++;
        return 0;
    }

    /* Fire trace hook if installed */
    if (cpu->trace_hook)
        cpu->trace_hook(cpu, cpu->trace_hook_user);

    /* Compute pipeline overlap benefit from previous instruction.
     * Must be called BEFORE pipeline_refill so that the refill cost
     * is included in the cycle accounting that overlap can recover. */
    bool was_flush = cpu->flush_pending;
    u32 overlap = pipeline_overlap_begin(cpu);

    /* Record start cycle count to compute instruction cycles */
    u64 start_cycles = cpu->cycle_count;

    /* Reset per-instruction tracking */
    cpu->e_bus_cycles = 0;
    cpu->b_fetch_cycles = 0;
    cpu->words_consumed = 0;

    /* Fill the prefetch queue.  The cost of these fetches is part of
     * the cycle accounting that the overlap model can recover.
     * After a flush, the B-stage had to restart, so we only fill
     * enough for opword + one extension word (minimal restart). */
    if (was_flush)
        pipeline_refill_n(cpu, 2);  /* minimal: opword + 1 extension */
    else
        pipeline_refill(cpu);       /* full: up to PREFETCH_DEPTH */

    /* Fetch opword (consumes one word from the prefetch queue) */
    cpu->PC = pipeline_peek_pc(cpu);
    u16 opword = pipeline_consume_word(cpu);

    /* C-stage: decode cost depends on EA complexity.
     * This penalty represents decode work that couldn't be hidden
     * behind the previous instruction's E-stage. If the previous
     * E-stage was long enough, the C-stage cost is fully overlapped. */
    u32 c_cost = opword_decode_cost(opword);
    if (c_cost > overlap) {
        cpu->cycle_count += (c_cost - overlap);
        overlap = 0;  /* overlap fully consumed by C-stage */
    } else {
        overlap -= c_cost;  /* remaining overlap benefits E-stage */
    }

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

    /* E-stage: dispatch to instruction handler */
    (void)g_dispatch[opword](cpu, opword);

    /* Apply remaining pipeline overlap: subtract prefetch time that was
     * hidden behind the previous instruction's execution. */
    u32 raw_cycles = (u32)(cpu->cycle_count - start_cycles);
    if (overlap > 0 && overlap <= raw_cycles) {
        cpu->cycle_count -= overlap;
    }

    /* Record timing for next instruction's overlap calculation */
    u32 actual = (u32)(cpu->cycle_count - start_cycles);
    pipeline_overlap_end(cpu, actual, cpu->e_bus_cycles);

    cpu->fault_jmp_active = false;
    cpu->instr_count++;

    return actual;
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
