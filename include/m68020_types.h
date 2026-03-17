#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Primitive types                                                      */
/* ------------------------------------------------------------------ */

typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;
typedef int8_t    s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;

/* ------------------------------------------------------------------ */
/* Bus types                                                            */
/* ------------------------------------------------------------------ */

/* Transfer sizes as the 68020 sees them */
typedef enum {
    SIZE_BYTE = 1,
    SIZE_WORD = 2,
    SIZE_LONG = 4,
} BusSize;

/* Function code values on FC2:FC0 pins */
typedef enum {
    FC_USER_DATA       = 1,
    FC_USER_PROG       = 2,
    FC_SUPERVISOR_DATA = 5,
    FC_SUPERVISOR_PROG = 6,
    FC_CPU_SPACE       = 7,
} FunctionCode;

/* Result of a bus callback */
typedef enum {
    BUS_OK    = 0,
    BUS_ERROR = 1,
} BusResult;
