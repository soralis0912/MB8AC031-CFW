/* shared/fjcore.h -- the Fujitsu I2C controller, wherever it is in the chip.
 *
 * Shared by src/main/i2c/ and src/auxcore/.  Two instances of this controller exist on this
 * family (mb86m01.h), they behave the same, and the only difference the
 * driver sees is the base address -- so the state machine takes one.
 *
 * Only one transport is ever selected for a board, so the control-register
 * shadow inside fjcore.c is a single copy.  fj_setup() is what establishes
 * it; calling fj_xfer() on a base that was not set up is a caller bug, and
 * i2c.c is what makes sure it cannot happen (its init is idempotent and runs
 * before every transfer).
 */
#ifndef FJCORE_H
#define FJCORE_H

/* Write the divider and the control register's initial value, then read the
 * control register back.  Does NOT clock or un-reset the block: whether that
 * is needed is a property of the instance, so it belongs to the caller.
 */
int fj_setup(unsigned base);

/* Write wn bytes then, with a restart, read rn bytes.  Either count may be
 * zero.  Always leaves a STOP on the bus, success or failure.
 */
int fj_xfer(unsigned base, unsigned addr, const unsigned char *wb, unsigned wn,
            unsigned char *rb, unsigned rn);

#define FJ_MAX_XFER     40u     /* what one FRQ1 data field and one aux slot carry */

#endif
