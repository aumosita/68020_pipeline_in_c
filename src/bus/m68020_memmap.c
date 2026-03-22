/*
 * m68020_memmap.c — Memory map layer.
 *
 * Provides address-range-based routing for the MC68020 bus interface.
 * System integrators register memory regions (ROM, RAM, MMIO, etc.)
 * with base address, size, and callbacks.  The memory map generates
 * an M68020BusInterface that routes all CPU accesses to the correct
 * region handler.
 *
 * Features:
 *   - Up to 32 regions (configurable)
 *   - Each region: base address, size, read/write callbacks, context
 *   - Read-only regions (ROM): write returns BUS_OK but is ignored
 *   - Unmapped addresses: returns BUS_ERROR (or configurable open-bus)
 *   - Wait states per region (e.g., slow ROM vs fast RAM)
 *
 * Usage:
 *   M68020MemMap *map = memmap_create();
 *   memmap_add_ram(map, 0x000000, 0x100000, ram_buffer);
 *   memmap_add_rom(map, 0xFC0000, 0x040000, rom_buffer);
 *   memmap_add_io(map, 0xDFF000, 0x001000, io_read, io_write, io_ctx);
 *   M68020BusInterface bus = memmap_bus_interface(map);
 *   M68020State *cpu = m68020_create(&bus);
 */

#include "m68020.h"
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Region types                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    REGION_RAM,   /* Read/write from buffer */
    REGION_ROM,   /* Read-only from buffer */
    REGION_IO,    /* Custom read/write callbacks */
} RegionType;

typedef struct {
    u32  base;
    u32  size;
    RegionType type;
    u32  wait_states;  /* extra bus cycles per access */

    /* For RAM/ROM: pointer to backing memory */
    u8  *data;

    /* For IO: custom callbacks */
    BusResult (*io_read)(void *ctx, u32 offset, BusSize size, u32 *val);
    BusResult (*io_write)(void *ctx, u32 offset, BusSize size, u32 val);
    void *io_ctx;
} MemRegion;

#define MEMMAP_MAX_REGIONS 32

struct M68020MemMap {
    MemRegion regions[MEMMAP_MAX_REGIONS];
    int       count;

    /* Interrupt controller callback */
    u8 (*iack)(void *ctx, u8 level);
    void *iack_ctx;

    /* Reset callback */
    void (*reset)(void *ctx);
    void *reset_ctx;
};

typedef struct M68020MemMap M68020MemMap;

/* ------------------------------------------------------------------ */
/* Region lookup                                                       */
/* ------------------------------------------------------------------ */

static MemRegion *find_region(M68020MemMap *map, u32 addr) {
    for (int i = 0; i < map->count; i++) {
        MemRegion *r = &map->regions[i];
        if (addr >= r->base && addr < r->base + r->size)
            return r;
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Byte-level read/write for RAM/ROM regions                           */
/* ------------------------------------------------------------------ */

static u8 region_read_byte(MemRegion *r, u32 offset) {
    if (offset < r->size) return r->data[offset];
    return 0xFF;
}

static void region_write_byte(MemRegion *r, u32 offset, u8 val) {
    if (r->type == REGION_ROM) return;  /* silently ignore ROM writes */
    if (offset < r->size) r->data[offset] = val;
}

/* ------------------------------------------------------------------ */
/* Bus interface callbacks                                             */
/* ------------------------------------------------------------------ */

static BusResult memmap_read(void *ctx, u32 addr, BusSize size,
                              FunctionCode fc, u32 *val, u32 *cycles_out) {
    M68020MemMap *map = (M68020MemMap *)ctx;
    (void)fc;
    *cycles_out = 0;

    MemRegion *r = find_region(map, addr);
    if (!r) {
        *val = 0xFFFFFFFFu;  /* open bus */
        return BUS_ERROR;
    }

    *cycles_out = r->wait_states;
    u32 offset = addr - r->base;

    if (r->type == REGION_IO) {
        return r->io_read(r->io_ctx, offset, size, val);
    }

    /* RAM or ROM: big-endian byte read */
    switch (size) {
    case SIZE_BYTE:
        *val = region_read_byte(r, offset);
        break;
    case SIZE_WORD:
        *val = ((u32)region_read_byte(r, offset) << 8)
             | region_read_byte(r, offset + 1);
        break;
    case SIZE_LONG:
        *val = ((u32)region_read_byte(r, offset)     << 24)
             | ((u32)region_read_byte(r, offset + 1) << 16)
             | ((u32)region_read_byte(r, offset + 2) <<  8)
             |  (u32)region_read_byte(r, offset + 3);
        break;
    }
    return BUS_OK;
}

static BusResult memmap_write(void *ctx, u32 addr, BusSize size,
                               FunctionCode fc, u32 val, u32 *cycles_out) {
    M68020MemMap *map = (M68020MemMap *)ctx;
    (void)fc;
    *cycles_out = 0;

    MemRegion *r = find_region(map, addr);
    if (!r) return BUS_ERROR;

    *cycles_out = r->wait_states;
    u32 offset = addr - r->base;

    if (r->type == REGION_IO) {
        return r->io_write(r->io_ctx, offset, size, val);
    }

    /* RAM or ROM: big-endian byte write */
    switch (size) {
    case SIZE_BYTE:
        region_write_byte(r, offset, (u8)val);
        break;
    case SIZE_WORD:
        region_write_byte(r, offset,     (u8)(val >> 8));
        region_write_byte(r, offset + 1, (u8)(val));
        break;
    case SIZE_LONG:
        region_write_byte(r, offset,     (u8)(val >> 24));
        region_write_byte(r, offset + 1, (u8)(val >> 16));
        region_write_byte(r, offset + 2, (u8)(val >>  8));
        region_write_byte(r, offset + 3, (u8)(val));
        break;
    }
    return BUS_OK;
}

static u8 memmap_iack(void *ctx, u8 level) {
    M68020MemMap *map = (M68020MemMap *)ctx;
    if (map->iack) return map->iack(map->iack_ctx, level);
    return 0xFF;  /* autovector */
}

static void memmap_reset(void *ctx) {
    M68020MemMap *map = (M68020MemMap *)ctx;
    if (map->reset) map->reset(map->reset_ctx);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

M68020MemMap *memmap_create(void) {
    M68020MemMap *map = (M68020MemMap *)calloc(1, sizeof(M68020MemMap));
    return map;
}

void memmap_destroy(M68020MemMap *map) {
    free(map);
}

bool memmap_add_ram(M68020MemMap *map, u32 base, u32 size,
                    u8 *buffer, u32 wait_states) {
    if (map->count >= MEMMAP_MAX_REGIONS) return false;
    MemRegion *r = &map->regions[map->count++];
    r->base = base;
    r->size = size;
    r->type = REGION_RAM;
    r->data = buffer;
    r->wait_states = wait_states;
    return true;
}

bool memmap_add_rom(M68020MemMap *map, u32 base, u32 size,
                    const u8 *buffer, u32 wait_states) {
    if (map->count >= MEMMAP_MAX_REGIONS) return false;
    MemRegion *r = &map->regions[map->count++];
    r->base = base;
    r->size = size;
    r->type = REGION_ROM;
    r->data = (u8 *)buffer;  /* const-cast: we never write to ROM */
    r->wait_states = wait_states;
    return true;
}

bool memmap_add_io(M68020MemMap *map, u32 base, u32 size,
                   BusResult (*rd)(void *, u32, BusSize, u32 *),
                   BusResult (*wr)(void *, u32, BusSize, u32),
                   void *ctx, u32 wait_states) {
    if (map->count >= MEMMAP_MAX_REGIONS) return false;
    MemRegion *r = &map->regions[map->count++];
    r->base = base;
    r->size = size;
    r->type = REGION_IO;
    r->io_read = rd;
    r->io_write = wr;
    r->io_ctx = ctx;
    r->wait_states = wait_states;
    return true;
}

void memmap_set_iack(M68020MemMap *map,
                     u8 (*iack)(void *, u8), void *ctx) {
    map->iack = iack;
    map->iack_ctx = ctx;
}

void memmap_set_reset(M68020MemMap *map,
                      void (*reset)(void *), void *ctx) {
    map->reset = reset;
    map->reset_ctx = ctx;
}

M68020BusInterface memmap_bus_interface(M68020MemMap *map) {
    M68020BusInterface bus = {
        .read             = memmap_read,
        .write            = memmap_write,
        .iack             = memmap_iack,
        .reset_peripherals = memmap_reset,
        .ctx              = map,
    };
    return bus;
}
