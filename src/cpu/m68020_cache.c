/*
 * m68020_cache.c — 256-byte instruction cache.
 *
 * The MC68020 instruction cache is:
 *   - 256 bytes total, 64 entries, direct-mapped
 *   - Each entry: 1 long word (4 bytes), tag, valid bit
 *   - Index = (address >> 2) & 0x3F  (bits 7:2 of address)
 *   - Tag  = address >> 8            (bits 31:8)
 *   - Read-only: only instruction fetches use the cache;
 *     data reads and writes always bypass it.
 *
 * CACR (Cache Control Register) bits:
 *   Bit 0: Enable (E)  — 1 = cache enabled
 *   Bit 1: Freeze (F)  — 1 = no new fills, hits still served
 *   Bit 2: Clear Entry (CE) — write-only, uses CAAR to select entry
 *   Bit 3: Clear (C)   — write-only, invalidates all entries
 *
 * CAAR (Cache Address Register):
 *   Used with CE bit to select which entry to invalidate.
 */

#include "m68020_internal.h"

/* CACR bit definitions */
#define CACR_ENABLE       0x01u
#define CACR_FREEZE       0x02u
#define CACR_CLEAR_ENTRY  0x04u
#define CACR_CLEAR_ALL    0x08u

/* ------------------------------------------------------------------ */
/* Cache index and tag extraction                                      */
/* ------------------------------------------------------------------ */

static inline u32 cache_index(u32 addr) {
    return (addr >> 2) & 0x3Fu;   /* 6-bit index from address bits 7:2 */
}

static inline u32 cache_tag(u32 addr) {
    return addr >> 8;             /* upper 24 bits */
}

/* ------------------------------------------------------------------ */
/* Public cache API                                                    */
/* ------------------------------------------------------------------ */

/*
 * Try to read a word from the instruction cache.
 * Returns true on hit, writing the word to *val.
 * Returns false on miss (caller must fetch from bus).
 *
 * The 68020 caches long words (4 bytes) per entry.
 * A word read at `addr` hits if the long word at `addr & ~3` is cached.
 */
bool icache_lookup(M68020State *cpu, u32 addr, u16 *val) {
    if (!(cpu->CACR & CACR_ENABLE))
        return false;

    u32 idx = cache_index(addr);
    u32 tag = cache_tag(addr);

    InstructionCache *ic = &cpu->icache;

    if (ic->valid[idx] && ic->tag[idx] == tag) {
        /* Hit: extract the requested word from the cached long word.
         * Long word is stored in big-endian register order.
         * addr bit 1 selects upper (0) or lower (1) word. */
        u32 longword = ic->data[idx];
        if (addr & 2u)
            *val = (u16)(longword & 0xFFFFu);        /* lower word */
        else
            *val = (u16)(longword >> 16);             /* upper word */
        return true;
    }

    return false;
}

/*
 * Fill a cache line from a long word fetched from the bus.
 * addr must be long-aligned (addr & 3 == 0).
 * Respects the freeze bit: no fill when frozen.
 */
void icache_fill(M68020State *cpu, u32 addr, u32 longword) {
    if (!(cpu->CACR & CACR_ENABLE))
        return;
    if (cpu->CACR & CACR_FREEZE)
        return;   /* frozen: serve hits but don't fill new entries */

    u32 idx = cache_index(addr);

    cpu->icache.tag[idx]   = cache_tag(addr);
    cpu->icache.data[idx]  = longword;
    cpu->icache.valid[idx] = true;
}

/*
 * Invalidate all cache entries.
 */
void icache_invalidate_all(M68020State *cpu) {
    for (u32 i = 0; i < ICACHE_ENTRIES; i++)
        cpu->icache.valid[i] = false;
}

/*
 * Invalidate a single cache entry selected by address (from CAAR).
 */
void icache_invalidate_entry(M68020State *cpu, u32 addr) {
    u32 idx = cache_index(addr);
    u32 tag = cache_tag(addr);

    if (cpu->icache.valid[idx] && cpu->icache.tag[idx] == tag)
        cpu->icache.valid[idx] = false;
}

/*
 * Process a CACR write: handle the write-only control bits (clear, clear entry)
 * and mask out the write-only bits from the stored value.
 */
void icache_update_cacr(M68020State *cpu, u32 new_cacr) {
    if (new_cacr & CACR_CLEAR_ALL)
        icache_invalidate_all(cpu);

    if (new_cacr & CACR_CLEAR_ENTRY)
        icache_invalidate_entry(cpu, cpu->CAAR);

    /* Store only the persistent bits (E and F); clear write-only bits */
    cpu->CACR = new_cacr & (CACR_ENABLE | CACR_FREEZE);
}
