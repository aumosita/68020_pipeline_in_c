#pragma once

/*
 * m68020_opcodes.h — Opcode encoding constants and masks.
 *
 * References: MC68020 User's Manual, Appendix A (Operation Code Map).
 */

/* ------------------------------------------------------------------ */
/* Primary opcode groups (bits [15:12])                                */
/* ------------------------------------------------------------------ */

#define OP_GRP_BIT_IMM   0x0000u  /* Bit manip / MOVEP / Immediate    */
#define OP_GRP_MOVE_B    0x1000u  /* MOVE.B                           */
#define OP_GRP_MOVE_L    0x2000u  /* MOVE.L / MOVEA.L                 */
#define OP_GRP_MOVE_W    0x3000u  /* MOVE.W / MOVEA.W                 */
#define OP_GRP_MISC      0x4000u  /* Miscellaneous                    */
#define OP_GRP_ADDQ_SCC  0x5000u  /* ADDQ / SUBQ / Scc / DBcc / TRAPcc*/
#define OP_GRP_BCC       0x6000u  /* Bcc / BRA / BSR                  */
#define OP_GRP_MOVEQ     0x7000u  /* MOVEQ                            */
#define OP_GRP_OR_DIV    0x8000u  /* OR / DIVU / DIVS / SBCD          */
#define OP_GRP_SUB       0x9000u  /* SUB / SUBX / SUBA                */
#define OP_GRP_LINEA     0xA000u  /* (unassigned / Line-A emulator)   */
#define OP_GRP_CMP_EOR   0xB000u  /* CMP / EOR                        */
#define OP_GRP_AND_MUL   0xC000u  /* AND / MULU / MULS / ABCD / EXG   */
#define OP_GRP_ADD       0xD000u  /* ADD / ADDX / ADDA                */
#define OP_GRP_SHIFT     0xE000u  /* Shift / Rotate / Bit Field        */
#define OP_GRP_COPROC    0xF000u  /* Line-F / Coprocessor             */

/* ------------------------------------------------------------------ */
/* Specific opcode words                                               */
/* ------------------------------------------------------------------ */

#define OP_NOP      0x4E71u
#define OP_RTS      0x4E75u
#define OP_RTR      0x4E77u
#define OP_RTE      0x4E73u
#define OP_RTD      0x4E74u   /* 68010+                             */
#define OP_ILLEGAL  0x4AFCu
#define OP_STOP     0x4E72u   /* followed by immediate SR value     */
#define OP_RESET    0x4E70u
#define OP_TRAPV    0x4E76u
#define OP_MOVEC_CR 0x4E7Au   /* MOVEC Rc,Rn (to register)          */
#define OP_MOVEC_RC 0x4E7Bu   /* MOVEC Rn,Rc (from register)        */

/* JMP / JSR — bits [11:6] = 111011 / 111010, bits [5:0] = ea */
#define OP_JMP_BASE  0x4EC0u  /* 0100 1110 11 <ea>                  */
#define OP_JSR_BASE  0x4E80u  /* 0100 1110 10 <ea>                  */

/* LINK — 0100 1110 0101 0rrr (16-bit displacement follows) */
#define OP_LINK_W    0x4E50u
/* LINK.L (68020): 0100 1000 0000 1rrr (32-bit displacement follows) */
#define OP_LINK_L    0x4808u
/* UNLK — 0100 1110 0101 1rrr */
#define OP_UNLK      0x4E58u

/* SWAP — 0100 1000 0100 0rrr */
#define OP_SWAP      0x4840u

/* EXT — 0100 1000 100s 0rrr (s=0: .W extend byte, s=1: .L extend word) */
#define OP_EXT_W     0x4880u
#define OP_EXT_L     0x48C0u
/* EXTB.L (68020) — 0100 1001 1100 0rrr */
#define OP_EXTB_L    0x49C0u

/* ------------------------------------------------------------------ */
/* EA field masks                                                       */
/* ------------------------------------------------------------------ */

/* Source EA in standard position (bits [5:0]) */
#define EA_SRC_MODE(op)  (((op) >> 3) & 7u)
#define EA_SRC_REG(op)   (((op)     ) & 7u)

/* Destination EA in MOVE position (bits [11:6]) */
#define EA_DST_MODE(op)  (((op) >> 6) & 7u)
#define EA_DST_REG(op)   (((op) >> 9) & 7u)

/* ------------------------------------------------------------------ */
/* Condition code field (bits [11:8] for Bcc, bits [3:0] for DBcc/Scc)*/
/* ------------------------------------------------------------------ */

#define CC_T   0x0u   /* True          (always)        */
#define CC_F   0x1u   /* False         (never)         */
#define CC_HI  0x2u   /* High          !C & !Z         */
#define CC_LS  0x3u   /* Low or Same   C | Z           */
#define CC_CC  0x4u   /* Carry Clear   !C   (HS)       */
#define CC_CS  0x5u   /* Carry Set     C    (LO)       */
#define CC_NE  0x6u   /* Not Equal     !Z              */
#define CC_EQ  0x7u   /* Equal         Z               */
#define CC_VC  0x8u   /* Overflow Clr  !V              */
#define CC_VS  0x9u   /* Overflow Set  V               */
#define CC_PL  0xAu   /* Plus          !N              */
#define CC_MI  0xBu   /* Minus         N               */
#define CC_GE  0xCu   /* Greater/Equal N==V            */
#define CC_LT  0xDu   /* Less Than     N!=V            */
#define CC_GT  0xEu   /* Greater Than  !Z & N==V       */
#define CC_LE  0xFu   /* Less/Equal    Z | N!=V        */
