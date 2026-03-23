/*
 * mac_lc.c — Macintosh LC system emulator.
 *
 * Phase A: ROM loading, memory map, I/O stubs.
 * The ROM is overlaid at address 0 during reset so the CPU can fetch
 * the initial SSP and PC from the exception vector table.
 * After the first access to a VIA register, the overlay is disabled
 * and RAM appears at address 0.
 */

#include "mac_lc.h"
#include "m68020_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* I/O stub callbacks                                                  */
/* ------------------------------------------------------------------ */

/*
 * VIA I/O handler.
 * Phase A: returns default values. Writing disables ROM overlay.
 */
static BusResult via_read(void *ctx, u32 offset, BusSize size, u32 *val) {
    MacLC *sys = (MacLC *)ctx;
    (void)size;

    /* VIA registers are at every 0x200 bytes (active on even addresses) */
    u8 reg = (offset >> 9) & 0xF;

    switch (reg) {
    case 0x0:  /* Port B data — overlay control in bit 3 */
        *val = sys->rom_overlay ? 0x00 : 0x08;
        break;
    case 0x1:  /* Port A data */
        *val = 0x7F;  /* no key pressed, no ADB event */
        break;
    case 0xD:  /* IFR (Interrupt Flag Register) */
        *val = 0x00;  /* no pending interrupts */
        break;
    case 0xE:  /* IER (Interrupt Enable Register) */
        *val = 0x00;
        break;
    default:
        *val = sys->via_regs[reg & 0xF];
        break;
    }
    return BUS_OK;
}

static BusResult via_write(void *ctx, u32 offset, BusSize size, u32 val) {
    MacLC *sys = (MacLC *)ctx;
    (void)size;

    u8 reg = (offset >> 9) & 0xF;
    sys->via_regs[reg & 0xF] = (u8)val;

    /* Writing to VIA Port B (reg 0) with bit 3 controls ROM overlay */
    if (reg == 0x0 && (val & 0x08)) {
        if (sys->rom_overlay) {
            sys->rom_overlay = false;
            /* TODO: remap memory — for now the overlay read handler
             * checks the flag dynamically */
        }
    }

    return BUS_OK;
}

/* Generic I/O stub: reads return 0, writes are ignored */
static BusResult io_stub_read(void *ctx, u32 offset, BusSize size, u32 *val) {
    (void)ctx; (void)size;
    static u32 last_io = 0xFFFFFFFF;
    if (offset != last_io) {
        fprintf(stderr, "IO_READ: offset=0x%X size=%d\n", offset, size);
        last_io = offset;
    }
    *val = 0;
    return BUS_OK;
}

static BusResult io_stub_write(void *ctx, u32 offset, BusSize size, u32 val) {
    (void)ctx; (void)offset; (void)size; (void)val;
    return BUS_OK;
}

/* ------------------------------------------------------------------ */
/* Overlay-aware RAM read/write                                        */
/* ------------------------------------------------------------------ */
/*
 * During reset, ROM is overlaid at address 0 so the CPU can read
 * the exception vectors. We handle this with a custom bus callback
 * rather than remapping regions.
 */
static BusResult overlay_read(void *ctx, u32 addr, BusSize size,
                               FunctionCode fc, u32 *val, u32 *cycles_out) {
    MacLC *sys = (MacLC *)ctx;
    *cycles_out = 0;
    (void)fc;

    /* ROM overlay: ROM appears at address 0 during reset.
     * MAME v8.cpp: overlay is disabled after first access at 0x000000-0x0FFFFF
     * (the rom_switch_r handler). We disable after reading the reset vectors. */
    if (sys->rom_overlay && addr < sys->rom_size) {
        switch (size) {
        case SIZE_BYTE: *val = sys->rom[addr]; break;
        case SIZE_WORD: *val = ((u32)sys->rom[addr]<<8)|sys->rom[addr+1]; break;
        case SIZE_LONG: *val = ((u32)sys->rom[addr]<<24)|((u32)sys->rom[addr+1]<<16)
                              |((u32)sys->rom[addr+2]<<8)|sys->rom[addr+3]; break;
        }
        /* Keep overlay active until ROM code explicitly disables it
         * via VIA Port B write. ROM code starts at low addresses (e.g. 0x2A)
         * and needs overlay to read its own code from ROM at address 0. */
        return BUS_OK;
    }

    /* Normal RAM access */
    if (addr < sys->ram_size) {
        switch (size) {
        case SIZE_BYTE: *val = sys->ram[addr]; break;
        case SIZE_WORD: *val = ((u32)sys->ram[addr]<<8)|sys->ram[addr+1]; break;
        case SIZE_LONG: *val = ((u32)sys->ram[addr]<<24)|((u32)sys->ram[addr+1]<<16)
                              |((u32)sys->ram[addr+2]<<8)|sys->ram[addr+3]; break;
        }
        return BUS_OK;
    }

    /* ROM at normal address */
    if (addr >= MAC_ROM_BASE && addr < MAC_ROM_BASE + sys->rom_size) {
        u32 off = addr - MAC_ROM_BASE;
        switch (size) {
        case SIZE_BYTE: *val = sys->rom[off]; break;
        case SIZE_WORD: *val = ((u32)sys->rom[off]<<8)|sys->rom[off+1]; break;
        case SIZE_LONG: *val = ((u32)sys->rom[off]<<24)|((u32)sys->rom[off+1]<<16)
                              |((u32)sys->rom[off+2]<<8)|sys->rom[off+3]; break;
        }
        return BUS_OK;
    }

    /* I/O range: 0x50000000-0x50FFFFFF */
    if (addr >= 0x50000000u && addr < 0x51000000u) {
        /* Try registered I/O regions first */
        M68020BusInterface mbus = memmap_bus_interface(sys->memmap);
        BusResult r = mbus.read(mbus.ctx, addr, size, fc, val, cycles_out);
        if (r == BUS_OK) return BUS_OK;

        /* Slot $F (0x50F00000-0x50FFFFFF): built-in video (V8).
         * ROM probes for declaration ROM at offset 0x1C00.
         * Return bus error — V8 video doesn't have a declaration ROM,
         * the ROM knows about built-in video through other means. */

        /* All other unregistered I/O → bus error */
        *val = 0;
        return BUS_ERROR;
    }

    /* All other unmapped addresses → bus error */
    static int unmapped_count = 0;
    if (unmapped_count < 20) {
        fprintf(stderr, "UNMAPPED READ: 0x%08X (PC=0x%08X)\n", addr, sys->cpu->PC);
        unmapped_count++;
    }
    *val = 0;
    return BUS_ERROR;
}

static BusResult overlay_write(void *ctx, u32 addr, BusSize size,
                                FunctionCode fc, u32 val, u32 *cycles_out) {
    MacLC *sys = (MacLC *)ctx;
    *cycles_out = 0;
    (void)fc;

    /* RAM write */
    if (addr < sys->ram_size) {
        switch (size) {
        case SIZE_BYTE: sys->ram[addr]=(u8)val; break;
        case SIZE_WORD: sys->ram[addr]=(u8)(val>>8); sys->ram[addr+1]=(u8)val; break;
        case SIZE_LONG: sys->ram[addr]=(u8)(val>>24); sys->ram[addr+1]=(u8)(val>>16);
                        sys->ram[addr+2]=(u8)(val>>8); sys->ram[addr+3]=(u8)val; break;
        }
        return BUS_OK;
    }

    /* ROM write — ignored */
    if (addr >= MAC_ROM_BASE && addr < MAC_ROM_BASE + sys->rom_size)
        return BUS_OK;

    /* I/O range */
    if (addr >= 0x50000000u && addr < 0x51000000u) {
        M68020BusInterface mbus = memmap_bus_interface(sys->memmap);
        return mbus.write(mbus.ctx, addr, size, fc, val, cycles_out);
    }

    return BUS_ERROR;
}

static u8 maclc_iack(void *ctx, u8 level) {
    (void)ctx; (void)level;
    return 0xFF;  /* autovector */
}

/* ------------------------------------------------------------------ */
/* ROM loader                                                          */
/* ------------------------------------------------------------------ */

static u8 *load_rom(const char *path, u32 *size_out) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "ERROR: Cannot open ROM file: %s\n", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0 || fsize > 4 * 1024 * 1024) {
        fprintf(stderr, "ERROR: Invalid ROM size: %ld bytes\n", fsize);
        fclose(f);
        return NULL;
    }

    u8 *rom = (u8 *)malloc((size_t)fsize);
    if (!rom) {
        fclose(f);
        return NULL;
    }

    if (fread(rom, 1, (size_t)fsize, f) != (size_t)fsize) {
        fprintf(stderr, "ERROR: Failed to read ROM\n");
        free(rom);
        fclose(f);
        return NULL;
    }

    fclose(f);
    *size_out = (u32)fsize;
    printf("ROM loaded: %s (%u bytes)\n", path, *size_out);
    return rom;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

MacLC *maclc_create(const char *rom_path, u32 ram_mb) {
    MacLC *sys = (MacLC *)calloc(1, sizeof(MacLC));
    if (!sys) return NULL;

    /* Load ROM */
    sys->rom = load_rom(rom_path, &sys->rom_size);
    if (!sys->rom) { free(sys); return NULL; }

    /* Allocate RAM */
    sys->ram_size = ram_mb * 1024 * 1024;
    if (sys->ram_size > MAC_RAM_MAX) sys->ram_size = MAC_RAM_MAX;
    sys->ram = (u8 *)calloc(1, sys->ram_size);
    if (!sys->ram) { free(sys->rom); free(sys); return NULL; }

    /* Allocate VRAM */
    sys->vram = (u8 *)calloc(1, MAC_VRAM_SIZE);

    /* Set up memory map with I/O regions */
    sys->memmap = memmap_create();
    memmap_add_io(sys->memmap, MAC_VIA_BASE, MAC_VIA_SIZE,
                  via_read, via_write, sys, 0);
    memmap_add_io(sys->memmap, MAC_SCC_BASE, MAC_SCC_SIZE,
                  io_stub_read, io_stub_write, sys, 0);
    memmap_add_io(sys->memmap, MAC_SCSI_BASE, MAC_SCSI_SIZE,
                  io_stub_read, io_stub_write, sys, 0);
    memmap_add_io(sys->memmap, MAC_ASC_BASE, MAC_ASC_SIZE,
                  io_stub_read, io_stub_write, sys, 0);
    memmap_add_io(sys->memmap, MAC_SWIM_BASE, MAC_SWIM_SIZE,
                  io_stub_read, io_stub_write, sys, 0);
    memmap_add_io(sys->memmap, MAC_VDAC_BASE, MAC_VDAC_SIZE,
                  io_stub_read, io_stub_write, sys, 0);
    memmap_add_io(sys->memmap, MAC_PSEUDOVIA_BASE, MAC_PSEUDOVIA_SIZE,
                  io_stub_read, io_stub_write, sys, 0);

    /* VRAM */
    memmap_add_ram(sys->memmap, MAC_VRAM_BASE, MAC_VRAM_SIZE, sys->vram, 0);

    /* NOTE: Unmapped I/O addresses (0x50028000+) intentionally NOT registered.
     * Mac LC ROM probes NuBus/PDS slot addresses during hardware detection.
     * These must return BUS_ERROR so the ROM's bus error handler detects
     * "no device present" and moves on.  If we return 0 instead, the ROM
     * thinks a device exists and gets stuck trying to initialize it. */

    /*
     * For Phase A: use a single bus interface that handles ROM overlay
     * directly, plus dispatches I/O through the memmap for VIA etc.
     * This is simpler than trying to remap regions on the fly.
     */
    M68020BusInterface bus_if;
    bus_if.read = overlay_read;
    bus_if.write = overlay_write;
    bus_if.iack = maclc_iack;
    bus_if.reset_peripherals = NULL;
    bus_if.ctx = sys;

    sys->cpu = m68020_create(&bus_if);
    sys->target_hz = 16000000;  /* 16 MHz */
    sys->rom_overlay = true;

    return sys;
}

void maclc_destroy(MacLC *sys) {
    if (!sys) return;
    m68020_destroy(sys->cpu);
    memmap_destroy(sys->memmap);
    free(sys->ram);
    free(sys->rom);
    free(sys->vram);
    free(sys);
}

void maclc_reset(MacLC *sys) {
    sys->rom_overlay = true;
    memset(sys->ram, 0, sys->ram_size);
    memset(sys->via_regs, 0, sizeof sys->via_regs);
    m68020_reset(sys->cpu);
    /* After reset, ROM overlay is active — CPU reads vectors from ROM */
}

void maclc_run(MacLC *sys, u64 cycles) {
    m68020_run(sys->cpu, cycles);
}

void maclc_step(MacLC *sys) {
    m68020_step(sys->cpu);

    /* Deassert interrupt after processing */
    if (sys->cpu->pending_ipl > 0 && !sys->cpu->in_exception)
        m68020_set_ipl(sys->cpu, 0);
}
