/*
 * m68020_callm_rtm.c — CALLM and RTM (68020 module call/return).
 *
 * CALLM <arg_count>, <ea>
 *   Opcode: 0000 0110 11 ea  (control EAs only, not Dn/An)
 *   Extension word: argument count in bits 7:0
 *
 * RTM Rn
 *   Opcode: 0000 0110 1100 xrrr  (x=0 → Dn, x=1 → An; rrr = register)
 *   No extension words.
 *
 * Both instructions implement 68020 module-based call semantics that were
 * specified in the architecture but essentially never used in practice.
 * They are decoded and their extension words consumed, then VEC_ILLEGAL_INSN
 * is raised to allow OS-level emulation traps.
 *
 * Dispatch table allocation:
 *   0x06C0..0x06C7  RTM Dn     (mode=0, reg=0..7)
 *   0x06C8..0x06CF  RTM An     (mode=1, reg=0..7)
 *   0x06D0..0x06D7  CALLM (An)         indirect
 *   0x06E8..0x06EF  CALLM (d16,An)     displaced
 *   0x06F0..0x06F7  CALLM (d8,An,Xn)  indexed
 *   0x06F8          CALLM (xxx).W
 *   0x06F9          CALLM (xxx).L
 *   0x06FA          CALLM (d16,PC)
 *   0x06FB          CALLM (d8,PC,Xn)
 */

#include "m68020_internal.h"
#include "m68020_exceptions.h"

/* RTM: no extension words to consume; raise VEC_ILLEGAL_INSN */
static u32 handler_rtm(M68020State *cpu, u16 opword) {
    (void)opword;
    exception_process(cpu, VEC_ILLEGAL_INSN);
    return 4;
}

/* CALLM: consume the arg-count extension word, then raise VEC_ILLEGAL_INSN */
static u32 handler_callm(M68020State *cpu, u16 opword) {
    (void)opword;
    pipeline_consume_word(cpu);   /* arg_count word */
    exception_process(cpu, VEC_ILLEGAL_INSN);
    return 4;
}

void m68020_callm_rtm_install_handlers(InsnHandler *t) {
    /* RTM Dn: 0x06C0..0x06C7 */
    for (u32 r = 0; r < 8; r++)
        t[0x06C0u | r] = handler_rtm;

    /* RTM An: 0x06C8..0x06CF */
    for (u32 r = 0; r < 8; r++)
        t[0x06C8u | r] = handler_rtm;

    /* CALLM (An): 0x06D0..0x06D7 */
    for (u32 r = 0; r < 8; r++)
        t[0x06D0u | r] = handler_callm;

    /* CALLM (d16,An): 0x06E8..0x06EF */
    for (u32 r = 0; r < 8; r++)
        t[0x06E8u | r] = handler_callm;

    /* CALLM (d8,An,Xn): 0x06F0..0x06F7 */
    for (u32 r = 0; r < 8; r++)
        t[0x06F0u | r] = handler_callm;

    /* CALLM absolute / PC-relative */
    t[0x06F8u] = handler_callm;   /* (xxx).W   */
    t[0x06F9u] = handler_callm;   /* (xxx).L   */
    t[0x06FAu] = handler_callm;   /* (d16,PC)  */
    t[0x06FBu] = handler_callm;   /* (d8,PC,Xn)*/
}
