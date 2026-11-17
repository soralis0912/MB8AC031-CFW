/* arm_dione.c -- waking the ARM bands on dione (GV-M2TV).  arm.c is the
 * rest; docs/arm.md.
 *
 * Measured on a GV-M2TV: with the high half of 0x00F10118 at
 * 0xFFFF, both control bands answer (CTL_EN reads 0x0001) and stay awake.
 * ts_dione.c uses the same value for the TS path, so leaving it there is
 * not a new state for the chip.
 */
#include "fw.h"
#include "mb86m01.h"
#include "regs_dione.h"

/* 0x00F10118 is the high half of its word.  It is read and written as a
 * word, keeping the low half (ts_dione.c does the same). */
unsigned dione_arm_gate(void)
{
    return MMIO(DN_PIN_GATE) >> 16;
}

/* Both bands wake together, so the window is not needed. */
void dione_arm_wake(unsigned win)
{
    (void)win;
    if (dione_arm_gate() != DN_PIN_GATE_ALL)
        MMIO(DN_PIN_GATE) = (DN_PIN_GATE_ALL << 16)
                            | (MMIO(DN_PIN_GATE) & 0xffffu);
}
