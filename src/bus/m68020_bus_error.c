/*
 * m68020_bus_error.c — Bus error and address error exception stubs.
 *
 * Phase 1: simplified implementation that builds a Format 0 frame
 * (not the full Format B bus-error frame required by real 68020 OS code).
 * The full Format B frame (46 words) will be implemented in Phase 3.
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"

void exception_bus_error(M68020State *cpu, u32 fault_addr,
                         bool is_write, u16 ssw) {
    (void)fault_addr; (void)is_write; (void)ssw;

    if (cpu->in_exception) {
        /* Double bus error: halt the CPU */
        cpu->halted = true;
        if (cpu->fault_jmp_active)
            longjmp(cpu->fault_jmp, 2);
        return;
    }

    exception_process(cpu, VEC_BUS_ERROR);
}

void exception_address_error(M68020State *cpu, u32 fault_addr, bool is_write) {
    (void)fault_addr; (void)is_write;

    if (cpu->in_exception) {
        cpu->halted = true;
        if (cpu->fault_jmp_active)
            longjmp(cpu->fault_jmp, 2);
        return;
    }

    exception_process(cpu, VEC_ADDRESS_ERR);
}
