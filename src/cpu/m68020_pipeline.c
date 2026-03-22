/*
 * m68020_pipeline.c — Prefetch queue management.
 *
 * The 68020 has a 3-stage pipeline (B/C/E) and maintains an internal
 * prefetch buffer of two long words (4 × 16-bit words).  We model it
 * as a circular queue of 4 slots, each tagged with its bus address.
 *
 * Instruction words are consumed from the queue via pipeline_consume_word().
 * After branches or exceptions, pipeline_flush() discards all buffered
 * words and resets the fetch pointer.
 */

#include "m68020_internal.h"

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/* Fetch one word from the bus and append it to the prefetch queue. */
static void prefetch_one(M68020State *cpu) {
    PrefetchQueue *q = &cpu->prefetch;
    if (q->count >= PREFETCH_DEPTH)
        return;  /* queue full */

    u16 word = 0;
    if (cpu_fetch_word(cpu, q->fetch_addr, &word) != BUS_OK)
        return;  /* bus error — exception already triggered */

    u8 tail = (q->head + q->count) % PREFETCH_DEPTH;
    q->buf[tail] = word;
    q->pc [tail] = q->fetch_addr;
    q->count++;
    q->fetch_addr += 2;
}

/* ------------------------------------------------------------------ */
/* Public pipeline API                                                 */
/* ------------------------------------------------------------------ */

/*
 * Consume and return the next word from the prefetch queue.
 * If the queue is empty, a blocking fetch is issued first.
 * Each consumed word costs 2 cycles (one bus word fetch if not
 * already buffered; the pipeline hides most of this cost).
 */
u16 pipeline_consume_word(M68020State *cpu) {
    PrefetchQueue *q = &cpu->prefetch;

    if (q->count == 0)
        prefetch_one(cpu);   /* stall: must wait for the bus */

    if (q->count == 0)
        return 0;            /* fetch failed (bus error already taken) */

    u16 word = q->buf[q->head];
    q->head   = (q->head + 1) % PREFETCH_DEPTH;
    q->count--;
    cpu->words_consumed++;  /* track for overlap ceiling calculation */
    return word;
}

/*
 * Return the bus address of the next word that would be consumed.
 * This is used by EA resolvers to compute (d16,PC) addresses.
 */
u32 pipeline_peek_pc(const M68020State *cpu) {
    const PrefetchQueue *q = &cpu->prefetch;
    if (q->count == 0)
        return q->fetch_addr;   /* nothing buffered yet */
    return q->pc[q->head];
}

/*
 * Flush the prefetch queue and restart fetching from new_pc.
 * Called after taken branches, exceptions, and RTE.
 * Penalty: 4 cycles (2 dead pipeline stages + 2 for first new fetch).
 */
void pipeline_flush(M68020State *cpu, u32 new_pc) {
    cpu->prefetch.head       = 0;
    cpu->prefetch.count      = 0;
    cpu->prefetch.fetch_addr = new_pc;
    cpu->cycle_count        += 2;   /* B-stage restart cost (2 cycles) */
    cpu->flush_pending       = true; /* kill overlap for next instruction */
}

/*
 * Opportunistically fill the prefetch queue up to PREFETCH_DEPTH words.
 * Called at the start of each instruction to hide bus latency.
 */
void pipeline_refill(M68020State *cpu) {
    while (cpu->prefetch.count < PREFETCH_DEPTH)
        prefetch_one(cpu);
}

/*
 * Partial refill: fill up to `n` words (not exceeding PREFETCH_DEPTH).
 * Used after pipeline flush for minimal B-stage restart.
 */
void pipeline_refill_n(M68020State *cpu, u8 n) {
    u8 target = (n < PREFETCH_DEPTH) ? n : PREFETCH_DEPTH;
    while (cpu->prefetch.count < target)
        prefetch_one(cpu);
}
