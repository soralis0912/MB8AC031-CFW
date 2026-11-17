/* card_tool.c -- the IFD handler's card path, from a terminal.
 *
 *     ./card_tool status      CARD_STATUS only (reads, writes no card line)
 *     ./card_tool atr         power up, ATR, CARD_PARAM, S(IFS), power off
 *
 * Same code as the driver (card_link.h, t1.h), so a run here says whether
 * the driver's power-up will work before anything is installed.  It sends
 * no APDU: the only block is the protocol-level S(IFS request).
 */
#include <stdio.h>
#include <string.h>

#include "card_link.h"
#include "t1.h"

#define IFSD 251u

static void print_hex(const char *label, const uint8_t *b, size_t n)
{
    size_t i;

    printf("%s", label);
    for (i = 0; i < n; i++)
        printf(" %02X", b[i]);
    printf("\n");
}

static int do_status(struct card_link *l)
{
    unsigned stage = 0;
    uint32_t event = 0;
    int rc = card_status(l, &stage, &event);

    if (rc != FRQ1_ST_OK) {
        fprintf(stderr, "CARD_STATUS: %s\n%s\n", card_rc_name(rc), l->err);
        return 1;
    }
    printf("stage %u  CARD_EVENT %08X  card %s\n", stage, event,
           event & CARD_EVENT_PRESENT ? "present (bit 10)" : "not seen");
    return 0;
}

static int do_atr(struct card_link *l)
{
    uint8_t atr[ATR_MAX];
    size_t natr = 0;
    struct atr_info ai;
    struct t1 t;
    const char *why;
    int rc, ret = 1;
    uint64_t t0;

    rc = card_off(l);
    if (rc != FRQ1_ST_OK) {
        fprintf(stderr, "CARD_OFF: %s\n%s\n", card_rc_name(rc), l->err);
        return 1;
    }
    t0 = card_now_ms();
    rc = card_on(l, atr, &natr, 3000u);
    if (rc != FRQ1_ST_OK) {
        fprintf(stderr, "power up: %s\n", l->err);
        goto off;
    }
    printf("ATR after %llu ms\n", (unsigned long long)(card_now_ms() - t0));
    print_hex("ATR", atr, natr);
    why = atr_parse(atr, natr, &ai);
    if (why != NULL) {
        fprintf(stderr, "ATR: %s\n", why);
        goto off;
    }
    printf("protocols %x  F %u  D %u  IFSC %u  CWI %u  BWI %u  %s mode\n",
           ai.protocols, ai.fi, ai.di, ai.ifsc, ai.cwi, ai.bwi,
           ai.specific ? "specific" : "negotiable");
    rc = card_param(l, ai.fi, ai.di, ai.cwi, ai.bwi);
    if (rc != FRQ1_ST_OK) {
        fprintf(stderr, "CARD_PARAM: %s\n", card_rc_name(rc));
        goto off;
    }
    t1_init(&t, card_xfer, l, &ai, IFSD);
    t0 = card_now_ms();
    if (t1_set_ifsd(&t) != T1_XFER_OK) {
        fprintf(stderr, "S(IFS %u): %s / %s\n", IFSD, t.err, l->err);
        goto off;
    }
    printf("S(IFS %u) answered in %llu ms\n", IFSD,
           (unsigned long long)(card_now_ms() - t0));
    ret = 0;
off:
    rc = card_off(l);
    printf("CARD_OFF: %s\n", card_rc_name(rc));
    return ret;
}

int main(int argc, char **argv)
{
    struct card_link l;
    int ret;

    memset(&l, 0, sizeof(l));
    if (argc != 2 || (strcmp(argv[1], "status") && strcmp(argv[1], "atr"))) {
        fprintf(stderr, "usage: %s status | atr\n", argv[0]);
        return 2;
    }
    if (card_open(&l) != 0) {
        fprintf(stderr, "%s\n", l.err);
        return 1;
    }
    if (card_hooked(&l) != 1) {
        fprintf(stderr, "%s\n", l.err);
        card_close(&l);
        return 1;
    }
    ret = strcmp(argv[1], "status") == 0 ? do_status(&l) : do_atr(&l);
    card_close(&l);
    return ret;
}
