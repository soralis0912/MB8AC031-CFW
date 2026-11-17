/* pcsc_atr.c -- the smallest PC/SC client that proves a reader: list the
 * readers, wait for a card, connect, print the ATR, disconnect.
 *
 *     ./pcsc_atr                  every reader
 *     ./pcsc_atr GV-M2TV          only readers whose name contains this
 *
 * It sends no APDU.  SCardConnect powers the card (IFDHPowerICC) and
 * SCardStatus returns the ATR the driver collected.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __APPLE__
#include <PCSC/winscard.h>
#include <PCSC/wintypes.h>
#else
#include <winscard.h>           /* pcsc-lite: pkg-config libpcsclite */
#endif

static const char *err(LONG rc)
{
    return pcsc_stringify_error(rc);
}

static int one_reader(SCARDCONTEXT ctx, const char *name)
{
    SCARD_READERSTATE rs;
    SCARDHANDLE h;
    DWORD proto = 0, state = 0, nlen = 0, alen = 0;
    BYTE atr[MAX_ATR_SIZE];
    LONG rc;
    DWORD i;

    memset(&rs, 0, sizeof(rs));
    rs.szReader = name;
    rs.dwCurrentState = SCARD_STATE_UNAWARE;
    rc = SCardGetStatusChange(ctx, 0, &rs, 1);
    if (rc != SCARD_S_SUCCESS) {
        printf("  status change: %s\n", err(rc));
        return 1;
    }
    if (!(rs.dwEventState & SCARD_STATE_PRESENT)) {
        printf("  waiting up to 10 s for a card...\n");
        rs.dwCurrentState = rs.dwEventState;
        rc = SCardGetStatusChange(ctx, 10000, &rs, 1);
        if (rc != SCARD_S_SUCCESS ||
            !(rs.dwEventState & SCARD_STATE_PRESENT)) {
            printf("  no card: %s (state %08x)\n", err(rc),
                   (unsigned)rs.dwEventState);
            return 1;
        }
    }
    printf("  card present (state %08x)\n", (unsigned)rs.dwEventState);

    rc = SCardConnect(ctx, name, SCARD_SHARE_SHARED, SCARD_PROTOCOL_T1, &h,
                      &proto);
    if (rc != SCARD_S_SUCCESS) {
        printf("  connect: %s\n", err(rc));
        return 1;
    }
    alen = sizeof(atr);
    rc = SCardStatus(h, NULL, &nlen, &state, &proto, atr, &alen);
    if (rc != SCARD_S_SUCCESS) {
        printf("  status: %s\n", err(rc));
        (void)SCardDisconnect(h, SCARD_UNPOWER_CARD);
        return 1;
    }
    printf("  protocol T=%d  ATR", proto == SCARD_PROTOCOL_T1 ? 1 : 0);
    for (i = 0; i < alen; i++)
        printf(" %02X", atr[i]);
    printf("\n");
    rc = SCardDisconnect(h, SCARD_UNPOWER_CARD);
    printf("  disconnect: %s\n", err(rc));
    return 0;
}

int main(int argc, char **argv)
{
    SCARDCONTEXT ctx;
    DWORD len = 0;
    char *names, *p;
    LONG rc;
    int seen = 0, bad = 0;

    rc = SCardEstablishContext(SCARD_SCOPE_SYSTEM, NULL, NULL, &ctx);
    if (rc != SCARD_S_SUCCESS) {
        fprintf(stderr, "establish context: %s\n", err(rc));
        return 1;
    }
    rc = SCardListReaders(ctx, NULL, NULL, &len);
    if (rc != SCARD_S_SUCCESS || len == 0) {
        fprintf(stderr, "no readers: %s\n", err(rc));
        SCardReleaseContext(ctx);
        return 1;
    }
    names = malloc(len);
    rc = SCardListReaders(ctx, NULL, names, &len);
    if (rc != SCARD_S_SUCCESS) {
        fprintf(stderr, "list readers: %s\n", err(rc));
        return 1;
    }
    for (p = names; *p; p += strlen(p) + 1) {
        printf("reader: %s\n", p);
        if (argc > 1 && strstr(p, argv[1]) == NULL)
            continue;
        seen++;
        bad += one_reader(ctx, p);
    }
    free(names);
    SCardReleaseContext(ctx);
    if (seen == 0) {
        fprintf(stderr, "no reader matched\n");
        return 1;
    }
    return bad ? 1 : 0;
}
