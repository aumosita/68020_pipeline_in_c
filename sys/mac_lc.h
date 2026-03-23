/*
 * mac_lc.h — Macintosh LC system emulator.
 *
 * Combines the MC68020 CPU core with Mac LC hardware:
 *   ROM, RAM, VIA, SCC, SCSI, ASC, SWIM, V8 video.
 */

#pragma once

#include "m68020.h"
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Mac LC memory map constants                                         */
/* ------------------------------------------------------------------ */

#define MAC_RAM_BASE     0x00000000u
#define MAC_RAM_MAX      (10u * 1024u * 1024u)  /* 10 MB max */
#define MAC_ROM_BASE     0x40800000u
#define MAC_ROM_SIZE     (512u * 1024u)          /* 512 KB */
#define MAC_ROM_OVERLAY  0x00000000u             /* ROM overlaid at 0 during reset */

/* I/O addresses */
#define MAC_VIA_BASE     0x50000000u
#define MAC_VIA_SIZE     0x00002000u
#define MAC_SCC_BASE     0x50004000u
#define MAC_SCC_SIZE     0x00001000u
#define MAC_SCSI_BASE    0x50006000u
#define MAC_SCSI_SIZE    0x00002000u
#define MAC_ASC_BASE     0x50014000u
#define MAC_ASC_SIZE     0x00001000u
#define MAC_SWIM_BASE    0x50016000u
#define MAC_SWIM_SIZE    0x00002000u
#define MAC_VDAC_BASE    0x50024000u
#define MAC_VDAC_SIZE    0x00001000u
#define MAC_V8_BASE      0x50026000u
#define MAC_V8_SIZE      0x00001000u

/* VRAM */
#define MAC_VRAM_BASE    0x00100000u  /* after first 1MB of RAM */
#define MAC_VRAM_SIZE    (256u * 1024u)

/* ------------------------------------------------------------------ */
/* System state                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    M68020State   *cpu;
    M68020MemMap  *memmap;

    /* Memory */
    u8  *ram;
    u32  ram_size;
    u8  *rom;
    u32  rom_size;
    u8  *vram;

    /* ROM overlay: at reset, ROM appears at address 0 for vector fetch */
    bool rom_overlay;

    /* I/O register state (stubs for Phase A) */
    u8   via_regs[32];
    u8   scc_regs[16];
    u8   v8_regs[256];

    /* Cycle tracking */
    u64  target_hz;     /* 16 MHz */
} MacLC;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/* Create Mac LC system. rom_path = path to 512KB ROM file. */
MacLC *maclc_create(const char *rom_path, u32 ram_mb);

/* Destroy and free all resources. */
void maclc_destroy(MacLC *sys);

/* Reset the system (cold boot). */
void maclc_reset(MacLC *sys);

/* Run for the given number of CPU cycles. */
void maclc_run(MacLC *sys, u64 cycles);

/* Run one CPU step (for debugging). */
void maclc_step(MacLC *sys);
