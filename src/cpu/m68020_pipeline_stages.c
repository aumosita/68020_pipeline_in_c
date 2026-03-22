/*
 * m68020_pipeline_stages.c — Pipeline overlap accounting.
 *
 * The MC68020 has a 3-stage pipeline:
 *   B-stage (Bus controller): prefetches instruction words, manages icache
 *   C-stage (Sequencer):      decodes opword, begins EA resolution
 *   E-stage (Execution):      ALU, data transfers, address calculation
 *
 * Rather than simulate each stage cycle-by-cycle (which would require
 * rewriting every instruction handler), we use an "overlap accounting"
 * model:
 *
 *   1. While the E-stage executes instruction N, the B-stage prefetches
 *      words for instruction N+1 (and possibly N+2, N+3...).
 *
 *   2. When instruction N+1 begins, its fetch cost is reduced by
 *      however many cycles the B-stage had available during N's execution.
 *
 *   3. If the E-stage used the data bus (memory read/write), the B-stage
 *      couldn't use the bus during those cycles → reduced overlap.
 *
 *   4. After a pipeline flush (taken branch, exception), there is no
 *      overlap — the B-stage must restart from the new PC.
 *
 * This model preserves the existing handler architecture while producing
 * more accurate cycle counts than the naive sequential model.
 */

#include "m68020_internal.h"

/*
 * Cost to fetch one instruction word from the bus (no cache hit).
 * The 68020 fetches aligned long words (4 bytes = 2 words) per bus cycle.
 * One long-word bus read = ~4 cycles; each word costs ~2 cycles.
 * With cache hit: 0 cycles.
 */
#define WORD_FETCH_CYCLES 2u

/*
 * pipeline_overlap_begin — Compute the prefetch overlap benefit.
 *
 * Returns the number of cycles to SUBTRACT from this instruction's
 * effective cost due to B-stage prefetch overlap.
 *
 * After a flush, overlap is 0 (B-stage had to restart).
 * Otherwise, overlap = min(b_avail_cycles, prefetch_cost), where
 * b_avail_cycles is how long the B-stage ran during the previous E-stage,
 * minus any data bus contention.
 */
u32 pipeline_overlap_begin(M68020State *cpu) {
    if (cpu->flush_pending) {
        cpu->flush_pending = false;
        cpu->b_avail_cycles = 0;
        return 0;  /* no overlap after flush */
    }

    /* B-stage available time = previous E-stage cycles minus data bus usage.
     * During data bus cycles, B-stage yields the bus. */
    u32 avail = cpu->b_avail_cycles;
    u32 contention = cpu->e_bus_cycles;
    u32 effective = (avail > contention) ? (avail - contention) : 0;

    /* The overlap benefit is limited by:
     * 1. Prefetch queue slots available × fetch cost per word
     * 2. Actual B-stage fetch cycles (cache hits cost 0, misses cost 4)
     * 3. Words the B-stage could fill (PREFETCH_DEPTH minus what the
     *    previous instruction consumed) */
    u32 slots_available = cpu->prefetch.count;
    u32 prefetch_benefit = slots_available * WORD_FETCH_CYCLES;

    /* Cap by actual fetch cost: can't overlap more than B-stage actually spent.
     * When cache is warm (all hits), fetch cost ≈ 0 → minimal overlap.
     * When cache is cold (all misses), fetch cost reflects bus latency. */
    u32 fetch_cap = cpu->last_b_fetch_cycles;
    if (fetch_cap > 0 && fetch_cap < prefetch_benefit)
        prefetch_benefit = fetch_cap;

    /* Cap by how many new words the B-stage could fetch during the
     * previous instruction: PREFETCH_DEPTH minus words consumed by prev. */
    u8 consumed = cpu->last_words_consumed;
    u32 fetchable = (consumed < PREFETCH_DEPTH)
                  ? (PREFETCH_DEPTH - consumed) * WORD_FETCH_CYCLES
                  : 0;
    if (fetchable < prefetch_benefit)
        prefetch_benefit = fetchable;

    u32 overlap = (effective < prefetch_benefit) ? effective : prefetch_benefit;
    return overlap;
}

/*
 * pipeline_overlap_end — Record timing for the next instruction's overlap.
 *
 * e_cycles:     Total cycles consumed by the E-stage (handler return value
 *               plus any bus cycles from data access).
 * e_bus_cycles: Data bus cycles consumed by E-stage (read/write, not ifetch).
 *               These represent bus contention with the B-stage.
 */
void pipeline_overlap_end(M68020State *cpu, u32 e_cycles, u32 e_bus_cycles) {
    cpu->last_e_cycles = e_cycles;
    cpu->b_avail_cycles = e_cycles;
    cpu->e_bus_cycles = e_bus_cycles;
    cpu->last_b_fetch_cycles = cpu->b_fetch_cycles;
    cpu->last_words_consumed = cpu->words_consumed;
}

/*
 * pipeline_note_data_bus — Accumulate data bus cycles during E-stage.
 *
 * Called from bus read/write helpers when the E-stage performs a data
 * access.  These cycles represent bus contention: while E uses the bus
 * for data, B cannot prefetch.
 */
void pipeline_note_data_bus(M68020State *cpu, u32 cycles) {
    cpu->e_bus_cycles += cycles;
}
