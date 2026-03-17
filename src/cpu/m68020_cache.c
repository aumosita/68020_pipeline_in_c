/*
 * m68020_cache.c — 256-byte instruction cache (Phase 1: stub).
 *
 * The real 68020 cache is a 256-byte, 64-entry, direct-mapped cache
 * with one long word (4 bytes) per line.  It is controlled via CACR
 * (Cache Control Register) and CAAR (Cache Address Register).
 *
 * Phase 1: the cache is always disabled (every fetch goes to the bus).
 * Phase 6 will implement the full cache with CACR/CAAR support.
 */

#include "m68020_internal.h"

/* Nothing to implement in Phase 1 — cache is bypassed. */
/* The InstructionCache struct in M68020State is zeroed at reset,
 * which means all entries are invalid (valid[] == false).         */
