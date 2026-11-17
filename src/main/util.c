/* util.c -- the few helpers a freestanding build needs.
 *
 * `-ffreestanding -fno-builtin` means the compiler will not reach for a libc
 * that is not there, so anything like memset has to exist here.  Kept
 * deliberately dumb: byte at a time, volatile-safe, no alignment tricks.
 * The frame and the MMIO windows are both volatile, and a cleverer version
 * would be free to coalesce accesses that the hardware counts.
 */
#include "fw.h"
#include "mb86m01.h"

void fw_memset(volatile unsigned char *p, int v, unsigned n)
{
    while (n--)
        *p++ = (unsigned char)v;
}

void fw_memcpy(volatile unsigned char *d, const volatile unsigned char *s,
               unsigned n)
{
    while (n--)
        *d++ = *s++;
}

/* A spin, not a measured delay.  The CPU clock is 243 or 216 MHz, assumed
 * to depend on bits 1..0 of the half-word at 0x00F10250 (not confirmed on
 * hardware),
 * and a real delay should scale by it.  We do not read that value yet, so
 * callers must treat the unit as "some loops" and not as time.
 */
void fw_delay(unsigned loops)
{
    volatile unsigned i;

    for (i = 0; i < loops; i++)
        ;
}

/* Unsigned divide.
 *
 * There is no libgcc in this build, so the helper GCC emits a call to
 * (__udivsi3) does not exist -- a `/` on a value the compiler cannot reduce
 * to shifts fails at link time, loudly, which is the behaviour we want.
 * Where a real division is needed, call this: restoring division, 32 steps,
 * no table.  Dividing by zero answers 0 rather than trapping, because the
 * callers here are turning a byte count into a record count and a zero
 * record size is a configuration fault they already report.
 */
unsigned fw_udiv(unsigned n, unsigned d, unsigned *rem)
{
    unsigned q = 0, r = 0;
    int i;

    if (!d) {
        if (rem)
            *rem = n;
        return 0;
    }
    for (i = 31; i >= 0; i--) {
        r = (r << 1) | ((n >> i) & 1u);
        q <<= 1;
        if (r >= d) {
            r -= d;
            q |= 1u;
        }
    }
    if (rem)
        *rem = r;
    return q;
}
