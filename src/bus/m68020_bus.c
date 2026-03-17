/*
 * m68020_bus.c — Bus interface: internal read/write helpers.
 *
 * All memory accesses go through these functions.  They:
 *   • Attach the correct function code (FC) to every cycle
 *   • Enforce 68020 alignment rules (word access must be even;
 *     long access may be odd but costs extra cycles)
 *   • Forward any bus error / address error to the exception system
 *   • Accumulate wait-state cycles in cpu->cycle_count
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* Function-code helpers                                               */
/* ------------------------------------------------------------------ */

FunctionCode cpu_data_fc(const M68020State *cpu) {
    return SR_S(cpu->SR) ? FC_SUPERVISOR_DATA : FC_USER_DATA;
}

FunctionCode cpu_prog_fc(const M68020State *cpu) {
    return SR_S(cpu->SR) ? FC_SUPERVISOR_PROG : FC_USER_PROG;
}

/* ------------------------------------------------------------------ */
/* Raw bus dispatch (no alignment / exception logic)                   */
/* ------------------------------------------------------------------ */

static BusResult raw_read(M68020State *cpu, u32 addr, BusSize size,
                          FunctionCode fc, u32 *val) {
    u32 cycles = 0;
    BusResult r = cpu->bus.read(cpu->bus.ctx, addr, size, fc, val, &cycles);
    cpu->cycle_count += cycles;
    return r;
}

static BusResult raw_write(M68020State *cpu, u32 addr, BusSize size,
                           FunctionCode fc, u32 val) {
    u32 cycles = 0;
    BusResult r = cpu->bus.write(cpu->bus.ctx, addr, size, fc, val, &cycles);
    cpu->cycle_count += cycles;
    return r;
}

/* ------------------------------------------------------------------ */
/* Data reads                                                          */
/* ------------------------------------------------------------------ */

BusResult cpu_read_byte(M68020State *cpu, u32 addr, u8 *val) {
    u32 raw;
    if (raw_read(cpu, addr, SIZE_BYTE, cpu_data_fc(cpu), &raw) != BUS_OK) {
        exception_bus_error(cpu, addr, false, 0);
        return BUS_ERROR;
    }
    *val = (u8)raw;
    return BUS_OK;
}

BusResult cpu_read_word(M68020State *cpu, u32 addr, u16 *val) {
    if (addr & 1u) {
        exception_address_error(cpu, addr, false);
        return BUS_ERROR;
    }
    u32 raw;
    if (raw_read(cpu, addr, SIZE_WORD, cpu_data_fc(cpu), &raw) != BUS_OK) {
        exception_bus_error(cpu, addr, false, 0);
        return BUS_ERROR;
    }
    *val = (u16)raw;
    return BUS_OK;
}

BusResult cpu_read_long(M68020State *cpu, u32 addr, u32 *val) {
    FunctionCode fc = cpu_data_fc(cpu);

    if (addr & 1u) {
        /* Misaligned long: split into two word reads (+2 cycle penalty) */
        u32 hi, lo;
        cpu->cycle_count += 2;
        if (raw_read(cpu, addr,     SIZE_WORD, fc, &hi) != BUS_OK) {
            exception_bus_error(cpu, addr, false, 0);
            return BUS_ERROR;
        }
        if (raw_read(cpu, addr + 2, SIZE_WORD, fc, &lo) != BUS_OK) {
            exception_bus_error(cpu, addr + 2, false, 0);
            return BUS_ERROR;
        }
        *val = ((u32)(u16)hi << 16) | (u16)lo;
        return BUS_OK;
    }

    u32 raw;
    if (raw_read(cpu, addr, SIZE_LONG, fc, &raw) != BUS_OK) {
        exception_bus_error(cpu, addr, false, 0);
        return BUS_ERROR;
    }
    *val = raw;
    return BUS_OK;
}

/* ------------------------------------------------------------------ */
/* Data writes                                                         */
/* ------------------------------------------------------------------ */

BusResult cpu_write_byte(M68020State *cpu, u32 addr, u8 val) {
    if (raw_write(cpu, addr, SIZE_BYTE, cpu_data_fc(cpu), val) != BUS_OK) {
        exception_bus_error(cpu, addr, true, 0);
        return BUS_ERROR;
    }
    return BUS_OK;
}

BusResult cpu_write_word(M68020State *cpu, u32 addr, u16 val) {
    if (addr & 1u) {
        exception_address_error(cpu, addr, true);
        return BUS_ERROR;
    }
    if (raw_write(cpu, addr, SIZE_WORD, cpu_data_fc(cpu), val) != BUS_OK) {
        exception_bus_error(cpu, addr, true, 0);
        return BUS_ERROR;
    }
    return BUS_OK;
}

BusResult cpu_write_long(M68020State *cpu, u32 addr, u32 val) {
    FunctionCode fc = cpu_data_fc(cpu);

    if (addr & 1u) {
        /* Misaligned long write: split into two word writes */
        cpu->cycle_count += 2;
        if (raw_write(cpu, addr,     SIZE_WORD, fc, val >> 16) != BUS_OK) {
            exception_bus_error(cpu, addr, true, 0);
            return BUS_ERROR;
        }
        if (raw_write(cpu, addr + 2, SIZE_WORD, fc, val & 0xFFFFu) != BUS_OK) {
            exception_bus_error(cpu, addr + 2, true, 0);
            return BUS_ERROR;
        }
        return BUS_OK;
    }

    if (raw_write(cpu, addr, SIZE_LONG, fc, val) != BUS_OK) {
        exception_bus_error(cpu, addr, true, 0);
        return BUS_ERROR;
    }
    return BUS_OK;
}

/* ------------------------------------------------------------------ */
/* Instruction fetch (uses program FC)                                 */
/* ------------------------------------------------------------------ */

BusResult cpu_fetch_word(M68020State *cpu, u32 addr, u16 *val) {
    if (addr & 1u) {
        exception_address_error(cpu, addr, false);
        return BUS_ERROR;
    }
    u32 raw;
    if (raw_read(cpu, addr, SIZE_WORD, cpu_prog_fc(cpu), &raw) != BUS_OK) {
        exception_bus_error(cpu, addr, false, 0);
        return BUS_ERROR;
    }
    *val = (u16)raw;
    return BUS_OK;
}

/* ------------------------------------------------------------------ */
/* Stack push / pop                                                    */
/* ------------------------------------------------------------------ */

void cpu_push_word(M68020State *cpu, u16 val) {
    cpu->A[7] -= 2;
    cpu_write_word(cpu, cpu->A[7], val);
}

void cpu_push_long(M68020State *cpu, u32 val) {
    cpu->A[7] -= 4;
    cpu_write_long(cpu, cpu->A[7], val);
}

u16 cpu_pop_word(M68020State *cpu) {
    u16 val = 0;
    cpu_read_word(cpu, cpu->A[7], &val);
    cpu->A[7] += 2;
    return val;
}

u32 cpu_pop_long(M68020State *cpu) {
    u32 val = 0;
    cpu_read_long(cpu, cpu->A[7], &val);
    cpu->A[7] += 4;
    return val;
}

/* ------------------------------------------------------------------ */
/* SR management (handles SP switching when S or M changes)            */
/* ------------------------------------------------------------------ */

void cpu_set_sr(M68020State *cpu, u16 new_sr) {
    u16 old_sr = cpu->SR;
    new_sr &= SR_VALID_MASK;

    bool old_s = SR_S(old_sr) != 0;
    bool new_s = SR_S(new_sr) != 0;
    bool old_m = SR_M(old_sr) != 0;
    bool new_m = SR_M(new_sr) != 0;

    /* Save current active SP to its shadow register */
    if (!old_s) {
        cpu->USP = cpu->A[7];
    } else if (old_m) {
        cpu->MSP = cpu->A[7];
    } else {
        cpu->ISP = cpu->A[7];
    }

    /* Load new active SP */
    if (!new_s) {
        cpu->A[7] = cpu->USP;
    } else if (new_m) {
        cpu->A[7] = cpu->MSP;
    } else {
        cpu->A[7] = cpu->ISP;
    }

    cpu->SR = new_sr;

    (void)old_s; (void)new_s; (void)old_m; (void)new_m;
}
