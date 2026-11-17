/* arm_selene.c -- waking the ARM bands on selene.  arm.c is the rest;
 * docs/arm.md, "selene".
 *
 * MEASURED ON A PIX-DT295.  With 0x00F10118 (half-word) at
 * 0xFFFF the band answers: CTL_EN reads 1, and after +0x404 bit2 and the
 * usual hold our image loads into ITCM and runs ('ARM1').  Put back to the
 * value it had (0x9000 after our I2C bring-up), the band reads 0 and the
 * hold fails.  So the gate is left open.  Opening more bits than were set
 * does not take any away from the TS path (it uses bit4).
 *
 * Done here: 0x00F10118 = 0xFFFF, and +0x404 bit2 for the one core asked
 * for.  Not done: any handshake on 0x00F10080..0x00F100FF, because reading
 * that range from this side is measured to break the resident image
 * (mb86m01.h); 0x00F1011C, whose meaning is unknown; and closing
 * 0x00F10118 again afterwards, which shuts the band (above).
 */
#include "fw.h"
#include "mb86m01.h"
#include "regs_selene.h"
#include "armcore.h"

/* 0x00F10118 is a half-word here, read and written as one (as measured). */
unsigned selene_arm_gate(void)
{
    return MMIO16(SL_UNIT_GATE) & 0xffffu;
}

void selene_arm_wake(unsigned win)
{
    unsigned r = win + ARM_CTL_404_OFF;

    MMIO16(SL_UNIT_GATE) = (unsigned short)SL_UNIT_GATE_ALL;
    MMIO16(r) = (unsigned short)((MMIO16(r) & 0xffffu) | SL_UNIT_404_ON);
}
