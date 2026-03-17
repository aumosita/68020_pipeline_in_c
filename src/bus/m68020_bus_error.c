/*
 * m68020_bus_error.c — Bus error and address error exceptions.
 *
 * Phase 3: full MC68020 Format B (long bus cycle fault) stack frame.
 *
 * Frame layout (46 words = 92 bytes, lowest address = SP):
 *
 *   SP+ 0: Format/vector word  (0xB000 | vec_offset)
 *   SP+ 2: Return PC [31:16]
 *   SP+ 4: Return PC [15:0]
 *   SP+ 6: Saved SR
 *   SP+ 8: Internal register  (0)
 *   SP+10: Special Status Word (SSW)
 *   SP+12: Pipeline stage C   (0)
 *   SP+14: Pipeline stage B   (0)
 *   SP+16: Data fault address [31:16]
 *   SP+18: Data fault address [15:0]
 *   SP+20..SP+90: Internal registers (36 words, all 0)
 *
 * SSW bit layout (from MC68020 User Manual §8.3.4):
 *   Bit 8 (RW): 1 = faulted cycle was a READ, 0 = WRITE
 *   Bits 12-10 (FC): function code active during the faulting cycle
 *
 * RTE from a Format B handler: pop fmt + PC + SR (8 bytes),
 * then skip the remaining 84 bytes of internal state.
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"

/* ------------------------------------------------------------------ */
/* Build and push a Format B frame, then vector to the handler.        */
/* ------------------------------------------------------------------ */

static void push_format_b(M68020State *cpu, u32 vector,
                          u32 fault_addr, bool is_write, u16 ssw) {
    u16 saved_sr = cpu->SR;
    u32 saved_pc = cpu->PC;   /* instruction being executed at fault */

    /* Enter supervisor mode, clear trace bits */
    u16 new_sr = (saved_sr | (1u << 13)) & ~((1u << 15) | (1u << 14));
    if (!SR_S(saved_sr)) {
        /* User → Supervisor: switch to ISP */
        cpu->USP  = cpu->A[7];
        cpu->A[7] = cpu->ISP;
    }
    cpu->SR = new_sr & SR_VALID_MASK;

    /* Build SSW: bit 8 = RW (1=read, 0=write); caller may supply extra bits */
    u16 full_ssw = (u16)(ssw | (is_write ? 0x0000u : 0x0100u));

    /* Push the frame from word 45 down to word 0.
     * Push in reverse so word 0 ends up at the final SP. */

    /* Words 45..10: 36 zero words = 18 zero longs */
    for (int i = 0; i < 18; i++)
        cpu_push_long(cpu, 0u);

    /* Word 9: fault address [15:0]  (SP+18 after all pushes) */
    cpu_push_word(cpu, (u16)(fault_addr & 0xFFFFu));
    /* Word 8: fault address [31:16] (SP+16) */
    cpu_push_word(cpu, (u16)(fault_addr >> 16));
    /* Word 7: pipeline stage B = 0  (SP+14) */
    cpu_push_word(cpu, 0u);
    /* Word 6: pipeline stage C = 0  (SP+12) */
    cpu_push_word(cpu, 0u);
    /* Word 5: SSW               (SP+10) */
    cpu_push_word(cpu, full_ssw);
    /* Word 4: internal reg = 0  (SP+8) */
    cpu_push_word(cpu, 0u);
    /* Word 3: saved SR          (SP+6) */
    cpu_push_word(cpu, saved_sr);
    /* Words 1-2: return PC (long, big-endian): SP+2 = high, SP+4 = low */
    cpu_push_long(cpu, saved_pc);
    /* Word 0: format/vector     (SP+0) */
    u16 fmt_word = ((u16)FRAME_FORMAT_B << 12) | (u16)((vector * 4u) & 0xFFFu);
    cpu_push_word(cpu, fmt_word);

    /* Read new PC from vector table */
    u32 vec_addr = cpu->VBR + (vector * 4u);
    u32 new_pc   = 0;
    if (cpu_read_long(cpu, vec_addr, &new_pc) != BUS_OK) {
        /* Bus error while reading the vector: double fault → halt */
        cpu->halted = true;
        if (cpu->fault_jmp_active)
            longjmp(cpu->fault_jmp, 2);
        return;
    }

    pipeline_flush(cpu, new_pc);
    cpu->PC = new_pc;
}

/* ------------------------------------------------------------------ */
/* Public exception entry points                                        */
/* ------------------------------------------------------------------ */

void exception_bus_error(M68020State *cpu, u32 fault_addr,
                         bool is_write, u16 ssw) {
    if (cpu->in_exception) {
        /* Double bus error: halt */
        cpu->halted = true;
        if (cpu->fault_jmp_active)
            longjmp(cpu->fault_jmp, 2);
        return;
    }

    cpu->in_exception = true;
    push_format_b(cpu, VEC_BUS_ERROR, fault_addr, is_write, ssw);
    cpu->in_exception = false;

    if (cpu->fault_jmp_active)
        longjmp(cpu->fault_jmp, 1);
}

void exception_address_error(M68020State *cpu, u32 fault_addr, bool is_write) {
    if (cpu->in_exception) {
        cpu->halted = true;
        if (cpu->fault_jmp_active)
            longjmp(cpu->fault_jmp, 2);
        return;
    }

    cpu->in_exception = true;
    push_format_b(cpu, VEC_ADDRESS_ERR, fault_addr, is_write, 0u);
    cpu->in_exception = false;

    if (cpu->fault_jmp_active)
        longjmp(cpu->fault_jmp, 1);
}
