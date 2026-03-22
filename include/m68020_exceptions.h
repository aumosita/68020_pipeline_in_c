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
/* SSW (Special Status Word) construction macros                       */
/* ------------------------------------------------------------------ */
/*
 * MC68020 SSW bit layout:
 *   Bits 12-10: FC2-FC0 (function code of faulted bus cycle)
 *   Bit  8:     RW (1=read, 0=write) — set by push_format_b
 *   Bit  6:     RM (1=read-modify-write cycle, e.g. TAS/CAS)
 *   Bit  5:     DF (1=data fault, 0=instruction fetch fault)
 *   Bits  1-0:  SIZE (00=reserved, 01=byte, 10=word, 11=long)
 */
#define SSW_FC(fc)     (((u16)(fc) & 7u) << 10)
#define SSW_RM         (1u << 6)
#define SSW_DF         (1u << 5)
#define SSW_SIZE_BYTE  (1u << 0)
#define SSW_SIZE_WORD  (2u << 0)
#define SSW_SIZE_LONG  (3u << 0)

/* Convert BusSize to SSW size bits */
static inline u16 ssw_size(BusSize sz) {
    switch (sz) {
        case SIZE_BYTE: return SSW_SIZE_BYTE;
        case SIZE_WORD: return SSW_SIZE_WORD;
        case SIZE_LONG: return SSW_SIZE_LONG;
    }
    return 0;
}

/* Build SSW for a data access fault */
#define SSW_DATA(fc, sz)     (SSW_FC(fc) | SSW_DF | ssw_size(sz))
/* Build SSW for an instruction fetch fault */
#define SSW_IFETCH(fc, sz)   (SSW_FC(fc) | ssw_size(sz))

/* ------------------------------------------------------------------ */
/* Functions (declared in m68020_internal.h; repeated here for callers)*/
/* ------------------------------------------------------------------ */

void exception_process(M68020State *cpu, u32 vector);
void exception_bus_error(M68020State *cpu, u32 fault_addr,
                         bool is_write, u16 ssw);
void exception_address_error(M68020State *cpu, u32 fault_addr, bool is_write);
