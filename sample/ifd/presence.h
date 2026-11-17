/* presence.h -- when the IFD handler says a card is in the slot.
 *
 * Kept apart from ifd_m2tv.c so test_presence.c can run it with no device.
 *
 * TWO KINDS OF SLOT.  The GV-M2TV's card cannot be pulled (docs/card.md),
 * so once CFW has answered CARD_STATUS with the present bit, the answer is
 * kept and the device is not asked again: the driver opens the USB device
 * only inside a PC/SC call, and pcscd asks for presence several times a
 * second.  A slot whose card can leave sets `removable` (the bundle's
 * CFWCardRemovable): then the device is asked again every
 * PRESENCE_REMOVABLE_MS whatever the last answer was, and an answer without
 * the present bit drops the card and its power.
 *
 * An ask that gets no answer (the device is busy, unplugged, or CFW is not
 * installed) changes nothing: the last answer stands.
 */
#ifndef PRESENCE_H
#define PRESENCE_H

#include <stdint.h>

#define PRESENCE_EVERY_MS       3000u   /* fixed card: until it is seen   */
#define PRESENCE_REMOVABLE_MS   500u    /* removable card: always         */

struct presence {
    int removable;              /* CFWCardRemovable                       */
    int powered;                /* IFDHPowerICC brought it up             */
    int known_present;          /* CFW answered with the present bit      */
    uint64_t next_probe;        /* card_now_ms() of the next ask          */
};

enum presence_step {
    PRESENCE_NO,                /* answer "no card"                       */
    PRESENCE_YES,               /* answer "card"                          */
    PRESENCE_ASK                /* ask the device, then presence_learn()  */
};

static inline int presence_held(const struct presence *p)
{
    return p->powered || p->known_present;
}

/* What to do for one IFDHICCPresence at time `now` (ms). */
static inline enum presence_step presence_plan(struct presence *p,
                                               uint64_t now)
{
    if (presence_held(p) && !p->removable)
        return PRESENCE_YES;
    if (now < p->next_probe)
        return presence_held(p) ? PRESENCE_YES : PRESENCE_NO;
    p->next_probe = now + (p->removable ? PRESENCE_REMOVABLE_MS
                                        : PRESENCE_EVERY_MS);
    return PRESENCE_ASK;
}

/* The device's answer to the ask: `answered` when CARD_STATUS came back
 * OK, `present` its present bit.  Returns PRESENCE_YES or PRESENCE_NO. */
static inline enum presence_step presence_learn(struct presence *p,
                                                int answered, int present)
{
    if (!answered)
        return presence_held(p) ? PRESENCE_YES : PRESENCE_NO;
    if (present) {
        p->known_present = 1;
        return PRESENCE_YES;
    }
    /* Gone (or never there).  Power goes with it: the next transmit fails
     * until pcscd powers the next card up. */
    p->known_present = 0;
    p->powered = 0;
    return PRESENCE_NO;
}

#endif
