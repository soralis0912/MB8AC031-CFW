/* cfwdev.h -- where each machine's CFW takes its frames, for the tools that
 * share a link with others (arm_probe, arm_ipc, the card driver).
 *
 * The loader generations differ (../docs/frq1.md):
 *
 *   dione  (GV-M2TV)    four mailboxes at 0x000C5000 + 0x40 * k, so a card
 *                       driver and an ARM tool can each have their own while
 *                       ts_grab records on the first
 *   selene (PIX-DT295)  four mailboxes at 0x003B0000 + 0x40 * k, the start
 *                       of our .bss seen uncached (entry_frc1_selene.S).
 *                       Same split as dione
 *
 * The hook is where frc1_load put our stub; reading it tells whether CFW is
 * installed before any frame is sent.
 */
#ifndef CFWDEV_H
#define CFWDEV_H

#include <stdint.h>
#include <string.h>

struct cfwdev {
    const char *name;
    unsigned vid, pid;
    int dione;
    uint32_t hook;
    uint32_t card_box;          /* the card driver's mailbox          */
    uint32_t arm_box;           /* arm_probe / arm_ipc's mailbox      */
};

static const struct cfwdev cfwdevs[] = {
    {"m2tv",  0x04BBu, 0x053Au, 1, 0x000E9F80u, 0x000C5040u, 0x000C5080u},
    {"dt295", 0x06B8u, 0x103Bu, 0, 0x000EA6FCu, 0x003B0040u, 0x003B0080u},
};

#define CFWDEV_COUNT (sizeof(cfwdevs) / sizeof(cfwdevs[0]))

/* -> the entry named `name`, or 0. */
static inline const struct cfwdev *cfwdev_find(const char *name)
{
    unsigned i;

    for (i = 0; i < CFWDEV_COUNT; i++)
        if (strcmp(cfwdevs[i].name, name) == 0)
            return &cfwdevs[i];
    return 0;
}

#endif
