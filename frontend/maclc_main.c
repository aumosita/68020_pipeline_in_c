/*
 * maclc_main.c — Macintosh LC emulator entry point.
 *
 * Phase A: Load ROM, run CPU, print execution trace.
 * No GUI yet — console output only.
 *
 * Usage: ./maclc <rom_file> [ram_mb] [max_steps]
 */

#include "mac_lc.h"
#include "m68020.h"
#include "m68020_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <rom_file> [ram_mb] [max_steps]\n", argv[0]);
        fprintf(stderr, "  rom_file  : Path to Mac LC ROM image (512 KB)\n");
        fprintf(stderr, "  ram_mb    : RAM size in MB (default 4)\n");
        fprintf(stderr, "  max_steps : Max instructions to execute (default 1000)\n");
        return 1;
    }

    const char *rom_path = argv[1];
    u32 ram_mb = (argc > 2) ? (u32)atoi(argv[2]) : 4;
    int max_steps = (argc > 3) ? atoi(argv[3]) : 1000;

    printf("Macintosh LC Emulator — Phase A\n");
    printf("================================\n");
    printf("ROM:  %s\n", rom_path);
    printf("RAM:  %u MB\n", ram_mb);
    printf("Max:  %d instructions\n\n", max_steps);

    MacLC *sys = maclc_create(rom_path, ram_mb);
    if (!sys) {
        fprintf(stderr, "Failed to create Mac LC system\n");
        return 1;
    }

    maclc_reset(sys);

    /* Print ROM header info */
    printf("ROM vectors (from overlay at 0x00000000):\n");
    u32 ssp = ((u32)sys->rom[0]<<24)|((u32)sys->rom[1]<<16)
             |((u32)sys->rom[2]<<8)|sys->rom[3];
    u32 pc  = ((u32)sys->rom[4]<<24)|((u32)sys->rom[5]<<16)
             |((u32)sys->rom[6]<<8)|sys->rom[7];
    printf("  Initial SSP: 0x%08X\n", ssp);
    printf("  Initial PC:  0x%08X\n\n", pc);

    /* Enable trace */
    m68020_trace_init(4096);
    m68020_trace_enable(sys->cpu);

    /* Execute */
    printf("Executing %d instructions...\n\n", max_steps);

    int executed = 0;
    for (int i = 0; i < max_steps; i++) {
        if (sys->cpu->halted) {
            printf("CPU HALTED at step %d\n", i);
            break;
        }
        if (sys->cpu->stopped) {
            printf("CPU STOPPED at step %d\n", i);
            break;
        }
        maclc_step(sys);
        executed++;
    }

    m68020_trace_disable(sys->cpu);

    /* Print final state */
    printf("\n--- Final CPU State ---\n");
    printf("PC:  0x%08X\n", m68020_get_reg(sys->cpu, REG_PC));
    printf("SR:  0x%04X\n", m68020_get_sr(sys->cpu));
    printf("SSP: 0x%08X\n", m68020_get_reg(sys->cpu, REG_A7));
    for (int i = 0; i < 8; i++)
        printf("D%d:  0x%08X\n", i, m68020_get_reg(sys->cpu, REG_D0 + i));
    for (int i = 0; i < 7; i++)
        printf("A%d:  0x%08X\n", i, m68020_get_reg(sys->cpu, REG_A0 + i));

    printf("\nExecuted %d instructions, %llu cycles\n",
           executed, (unsigned long long)sys->cpu->cycle_count);
    printf("ROM overlay: %s\n", sys->rom_overlay ? "ACTIVE" : "disabled");

    /* Print last 20 trace entries */
    printf("\n--- Execution Trace (last 20) ---\n");
    /* Need to pass ROM as memory for disassembly */
    if (sys->rom_overlay) {
        /* Dump trace — use ROM for disassembly at ROM addresses */
    m68020_trace_dump(stdout, 100, sys->rom, sys->rom_size);
    } else {
        m68020_trace_dump(stdout, 20, sys->ram, sys->ram_size);
    }

    maclc_destroy(sys);
    m68020_trace_free();

    return 0;
}
