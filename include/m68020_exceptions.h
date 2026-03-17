#pragma once

/*
 * m68020_exceptions.h — Exception vector numbers and stack frame formats.
 */

#include "m68020_types.h"

typedef struct M68020State M68020State;

/* ------------------------------------------------------------------ */
/* Exception vector numbers (index into the vector table)              */
/* ------------------------------------------------------------------ */

typedef enum {
    VEC_RESET_SSP     =  0,
    VEC_RESET_PC      =  1,
    VEC_BUS_ERROR     =  2,
    VEC_ADDRESS_ERR   =  3,
    VEC_ILLEGAL_INSN  =  4,
    VEC_ZERO_DIVIDE   =  5,
    VEC_CHK           =  6,
    VEC_TRAPV         =  7,
    VEC_PRIVILEGE     =  8,
    VEC_TRACE         =  9,
    VEC_LINE_A        = 10,
    VEC_LINE_F        = 11,
    VEC_COPROC_VIOL   = 13,
    VEC_FORMAT_ERR    = 14,
    VEC_UNINIT_INT    = 15,
    VEC_SPURIOUS      = 24,
    VEC_AUTOVEC_1     = 25,
    VEC_AUTOVEC_2     = 26,
    VEC_AUTOVEC_3     = 27,
    VEC_AUTOVEC_4     = 28,
    VEC_AUTOVEC_5     = 29,
    VEC_AUTOVEC_6     = 30,
    VEC_AUTOVEC_7     = 31,
    VEC_TRAP_BASE     = 32,   /* 32–47: TRAP #0–#15            */
    VEC_USER_FIRST    = 64,
    VEC_USER_LAST     = 255,
} ExceptionVector;

/* ------------------------------------------------------------------ */
/* Stack frame formats (upper nibble of the format/vector word)        */
/* ------------------------------------------------------------------ */
/*
 * Frame layout (from SP growing upward):
 *
 *  Format 0 (4 words, 8 bytes):
 *    SP+0: Format/Vector word  (0x0000 | vec*4)
 *    SP+2: PC[31:16]
 *    SP+4: PC[15:0]
 *    SP+6: SR (saved)
 *
 *  Format 2 (6 words, 12 bytes):
 *    SP+0:  Format/Vector word  (0x2000 | vec*4)
 *    SP+2:  PC[31:16]   (return address = instruction after fault)
 *    SP+4:  PC[15:0]
 *    SP+6:  SR (saved)
 *    SP+8:  Instruction PC[31:16]  (address of faulting instruction)
 *    SP+10: Instruction PC[15:0]
 *
 *  Format B (46 words) is used for bus/address errors — Phase 3.
 */

typedef enum {
    FRAME_FORMAT_0 = 0x0,
    FRAME_FORMAT_2 = 0x2,
    FRAME_FORMAT_B = 0xB,
} FrameFormat;

/* ------------------------------------------------------------------ */
/* Functions (declared in m68020_internal.h; repeated here for callers)*/
/* ------------------------------------------------------------------ */

void exception_process(M68020State *cpu, u32 vector);
void exception_bus_error(M68020State *cpu, u32 fault_addr,
                         bool is_write, u16 ssw);
void exception_address_error(M68020State *cpu, u32 fault_addr, bool is_write);
