/* sys.c -- bring-up and the things that have no other home.
 *
 * sys_init is what entry.S would call if this firmware ever needed to set
 * itself up before answering.  It does not today: the resident is entered
 * per command, answers, and returns, so there is no main loop and no state
 * to initialise.  That changes the moment streaming exists, because then
 * something has to own the TS interrupt and keep running between frames.
 *
 * What is assumed about interrupts here (not confirmed on hardware): the
 * servicer on the device does not touch the interrupt setup, but it does
 * keep hold of EP0.  So a resident that wants interrupts has to
 * install its own handlers, and must not assume EP0 is free.
 *
 * NOT WRITTEN YET, on purpose: there is nothing to initialise until a module
 * needs it, and an init that touches hardware we are not using would only
 * risk the connection we are answering on.
 */
#include "fw.h"
#include "mb86m01.h"

int sys_init(void)
{
    return FW_ENOTIMPL;
}
