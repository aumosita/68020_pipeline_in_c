/*
 * m68020_state.c — CPU instance lifecycle and register access.
 */

#include "m68020_internal.h"
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

M68020State *m68020_create(const M68020BusInterface *bus) {
    M68020State *cpu = (M68020State *)calloc(1, sizeof(M68020State));
    if (!cpu) return NULL;

    cpu->bus = *bus;
    m68020_build_dispatch_table();
    return cpu;
}

void m68020_destroy(M68020State *cpu) {
    free(cpu);
}

/* ------------------------------------------------------------------ */
/* Hardware reset                                                      */
/* ------------------------------------------------------------------ */
/*
 * Sequence (per MC68020 User's Manual §8.4):
 *  1. Enter supervisor state, interrupt level 7, clear T/M bits
 *  2. Load SSP from vector 0 (address 0x00000000)
 *  3. Load PC  from vector 1 (address 0x00000004)
 *  4. Flush prefetch queue
 */
void m68020_reset(M68020State *cpu) {
    cpu->SR   = 0x2700u;   /* S=1, IPL=7, all flags clear          */
    cpu->VBR  = 0;
    cpu->CACR = 0;
    cpu->CAAR = 0;
    cpu->SFC  = FC_USER_DATA;
    cpu->DFC  = FC_USER_DATA;

    memset(&cpu->prefetch, 0, sizeof cpu->prefetch);
    memset(&cpu->icache,   0, sizeof cpu->icache);

    cpu->halted        = false;
    cpu->stopped       = false;
    cpu->trace_pending = false;
    cpu->in_exception  = false;
    cpu->pending_ipl   = 0;
    cpu->cycle_count   = 0;
    cpu->instr_count   = 0;

    /* Read initial SSP from vector 0 */
    u32 ssp = 0;
    {
        u32 raw, cycles = 0;
        cpu->bus.read(cpu->bus.ctx, 0x00000000u, SIZE_LONG,
                      FC_SUPERVISOR_DATA, &raw, &cycles);
        cpu->cycle_count += cycles;
        ssp = raw;
    }

    /* Read initial PC from vector 1 */
    u32 initial_pc = 0;
    {
        u32 raw, cycles = 0;
        cpu->bus.read(cpu->bus.ctx, 0x00000004u, SIZE_LONG,
                      FC_SUPERVISOR_DATA, &raw, &cycles);
        cpu->cycle_count += cycles;
        initial_pc = raw;
    }

    /* Set up stack pointers: all supervisor SPs start at SSP, USP at 0 */
    cpu->ISP  = ssp;
    cpu->MSP  = ssp;
    cpu->USP  = 0;
    cpu->A[7] = ssp;   /* active SP = ISP (S=1, M=0) */

    pipeline_flush(cpu, initial_pc);
    cpu->PC = initial_pc;
}

/* ------------------------------------------------------------------ */
/* Register access                                                     */
/* ------------------------------------------------------------------ */

u32 m68020_get_reg(const M68020State *cpu, M68020Reg reg) {
    switch (reg) {
        case REG_D0: case REG_D1: case REG_D2: case REG_D3:
        case REG_D4: case REG_D5: case REG_D6: case REG_D7:
            return cpu->D[reg - REG_D0];
        case REG_A0: case REG_A1: case REG_A2: case REG_A3:
        case REG_A4: case REG_A5: case REG_A6: case REG_A7:
            return cpu->A[reg - REG_A0];
        case REG_PC:   return cpu->PC;
        case REG_SR:   return cpu->SR;
        case REG_USP:  return cpu->USP;
        case REG_MSP:  return cpu->MSP;
        case REG_ISP:  return cpu->ISP;
        case REG_VBR:  return cpu->VBR;
        case REG_CACR: return cpu->CACR;
        case REG_CAAR: return cpu->CAAR;
        case REG_SFC:  return cpu->SFC;
        case REG_DFC:  return cpu->DFC;
        default:       return 0;
    }
}

void m68020_set_reg(M68020State *cpu, M68020Reg reg, u32 value) {
    switch (reg) {
        case REG_D0: case REG_D1: case REG_D2: case REG_D3:
        case REG_D4: case REG_D5: case REG_D6: case REG_D7:
            cpu->D[reg - REG_D0] = value; break;
        case REG_A0: case REG_A1: case REG_A2: case REG_A3:
        case REG_A4: case REG_A5: case REG_A6: case REG_A7:
            cpu->A[reg - REG_A0] = value; break;
        case REG_PC:
            cpu->PC = value;
            pipeline_flush(cpu, value);
            break;
        case REG_SR:   cpu_set_sr(cpu, (u16)value); break;
        case REG_USP:  cpu->USP  = value; break;
        case REG_MSP:  cpu->MSP  = value; break;
        case REG_ISP:  cpu->ISP  = value; break;
        case REG_VBR:  cpu->VBR  = value; break;
        case REG_CACR: cpu->CACR = value; break;
        case REG_CAAR: cpu->CAAR = value; break;
        case REG_SFC:  cpu->SFC  = value & 7u; break;
        case REG_DFC:  cpu->DFC  = value & 7u; break;
        default: break;
    }
}

u16 m68020_get_sr(const M68020State *cpu) { return cpu->SR; }

void m68020_set_sr(M68020State *cpu, u16 sr) { cpu_set_sr(cpu, sr); }

void m68020_set_ipl(M68020State *cpu, u8 level) {
    cpu->pending_ipl = level & 7u;
}

void m68020_attach_coprocessor(M68020State *cpu, M68020Coprocessor *cp) {
    if (cp && cp->cpid < 8)
        cpu->coprocessors[cp->cpid] = cp;
}

void m68020_set_trace_hook(M68020State *cpu,
                            void (*hook)(M68020State *, void *),
                            void *user) {
    cpu->trace_hook      = hook;
    cpu->trace_hook_user = user;
}
