/* test_presence.c -- presence.h against a simulated slot.  No hardware.
 *
 *     make test
 *
 * Plays pcscd's side: IFDHICCPresence every 100 ms of simulated time, with
 * the card going in and out, the device not answering for a while, and the
 * card powered up between.  The slot answers CARD_STATUS from `card_in`,
 * the way CFW's present bit would.  Checks both kinds of slot: the fixed
 * one (the GV-M2TV) and one whose card can leave (CFWCardRemovable).
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "presence.h"

struct sim {
    struct presence pr;
    uint64_t now;
    int card_in;                /* what the present bit would say          */
    int device_up;              /* CARD_STATUS gets an answer              */
    unsigned asks;              /* times the device was asked              */
};

/* One IFDHICCPresence, the way ifd_m2tv.c does it.  1 card, 0 none. */
static int presence(struct sim *s)
{
    enum presence_step st = presence_plan(&s->pr, s->now);

    if (st == PRESENCE_ASK) {
        s->asks++;
        st = presence_learn(&s->pr, s->device_up,
                            s->device_up && s->card_in);
    }
    return st == PRESENCE_YES;
}

/* Poll every 100 ms for `ms`; returns the ms until the answer first
 * matched `want` (and stayed so to the end), or -1 if it never settled. */
static long settle(struct sim *s, unsigned ms, int want)
{
    long at = -1;
    unsigned t;

    for (t = 0; t < ms; t += 100u) {
        int got = presence(s);

        if (got == want && at < 0)
            at = (long)t;
        else if (got != want)
            at = -1;
        s->now += 100u;
    }
    return at;
}

static void fresh(struct sim *s, int removable)
{
    memset(s, 0, sizeof(*s));
    s->pr.removable = removable;
    s->now = 1000u;
    s->device_up = 1;
}

static void fixed_slot(void)
{
    struct sim s;
    unsigned asks;

    fresh(&s, 0);
    s.card_in = 1;
    assert(settle(&s, 1000, 1) == 0);
    asks = s.asks;
    assert(asks == 1);
    /* Seen once: never asked again, even if the bit would now say gone.
     * (The GV-M2TV's card cannot leave; asking would only hold EP0.) */
    s.card_in = 0;
    assert(settle(&s, 10000, 1) == 0);
    assert(s.asks == asks);
    printf("fixed slot: seen once, kept, asked %u time(s): ok\n", s.asks);

    /* No card at first: asked every PRESENCE_EVERY_MS until one shows. */
    fresh(&s, 0);
    assert(settle(&s, 9000, 0) == 0);
    assert(s.asks == 3);
    s.card_in = 1;
    assert(settle(&s, 4000, 1) >= 0);
    printf("fixed slot: empty, then a card within %u ms: ok\n",
           PRESENCE_EVERY_MS);
}

static void removable_slot(void)
{
    struct sim s;
    long at;

    fresh(&s, 1);
    s.card_in = 1;
    assert(settle(&s, 1000, 1) == 0);

    /* Powered up, then pulled: "no card" within one interval, and the
     * power is dropped with it. */
    s.pr.powered = 1;
    s.card_in = 0;
    at = settle(&s, 2000, 0);
    assert(at >= 0 && at <= (long)PRESENCE_REMOVABLE_MS);
    assert(!s.pr.powered && !s.pr.known_present);
    printf("removable: pulled while powered, seen gone in %ld ms, "
           "power dropped: ok\n", at);

    /* Back in. */
    s.card_in = 1;
    at = settle(&s, 2000, 1);
    assert(at >= 0 && at <= (long)PRESENCE_REMOVABLE_MS);
    printf("removable: put back, seen in %ld ms: ok\n", at);

    /* The device stops answering (busy, CFW reinstalled): the last
     * answer stands, both ways. */
    s.device_up = 0;
    assert(settle(&s, 3000, 1) == 0);
    s.device_up = 1;
    s.card_in = 0;
    assert(settle(&s, 2000, 0) >= 0);
    s.device_up = 0;
    s.card_in = 1;
    assert(settle(&s, 3000, 0) == 0);
    s.device_up = 1;
    assert(settle(&s, 2000, 1) >= 0);
    printf("removable: no answer keeps the last one: ok\n");

    /* Asked at most once per interval however often pcscd polls. */
    fresh(&s, 1);
    s.card_in = 1;
    (void)settle(&s, 10000, 1);
    assert(s.asks == 10000u / PRESENCE_REMOVABLE_MS);
    printf("removable: %u asks in 10 s of polling every 100 ms: ok\n",
           s.asks);
}

int main(void)
{
    fixed_slot();
    removable_slot();
    printf("all ok\n");
    return 0;
}
