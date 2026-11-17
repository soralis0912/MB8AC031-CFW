/* ifd_m2tv.c -- a PC/SC IFD handler for the GV-M2TV's card slot, driven
 * through CFW's card frames.
 *
 * macOS (CryptoTokenKit's com.apple.ifdreader) loads third-party IFD
 * bundles from /usr/local/libexec/SmartCardServices/drivers and hands a
 * USB device to the one whose Info.plist lists its vendor and product id.
 * This bundle claims 04bb:053a (GV-M2TV) and 06b8:103b (PIX-DT295).
 * README.md says how to build and install.
 *
 * Layers:
 *     PC/SC (APDU)  -> this file
 *     T=1 blocks    -> t1.h, entirely on the host
 *     card frames   -> card_link.h: CARD_ON / PARAM / PUT / SEND / RECV
 *     the 0xBC window, libusb -> ../chipio.h
 *
 * THE USB DEVICE IS OPENED PER CALL, NOT HELD.  The same window is how
 * frc1_load installs CFW and how ts_grab streams, so this driver only
 * touches the device inside a PC/SC call and closes it again.  And
 * presence is not polled once known: the GV-M2TV's card cannot be pulled
 * (docs/card.md), so after CFW has once answered with CARD_EVENT bit 10 set
 * the slot is reported occupied without asking again (presence.h; a slot
 * whose card can leave sets CFWCardRemovable and is asked every 500 ms).  Until then the
 * driver asks at most every few seconds -- and never sends a frame before
 * a read-only look at the hook says CFW is there (card_hooked()), so a
 * probe cannot land while frc1_load is writing that hook.
 *
 * THIS ONLY MAKES THE SLOT VISIBLE AS A PC/SC READER.  It relays APDUs and
 * does nothing with what is on the card.
 *
 * APDUS ARE REFUSED UNLESS ENABLED.  Power-up, the ATR and the protocol-level S(IFS) always
 * work; IFDHTransmitToICC refuses every APDU (and logs its header) unless
 * the bundle's Info.plist has CFWAllowAPDU = true.  macOS itself sends
 * APDUs to every card it sees (token driver matching), which is the reason
 * the default is off.
 *
 * Logs go to the unified log: log stream --predicate 'subsystem == "cfw.ifd"'
 * On Linux (pcsc-lite's pcscd) they go to syslog as "ifd-cfw-m2tv".
 */
#define _DARWIN_C_SOURCE 1
#define _GNU_SOURCE 1           /* dladdr on glibc */

#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __APPLE__
#include <CoreFoundation/CoreFoundation.h>
#include <os/log.h>
#else
#include <stdarg.h>
#include <stdlib.h>
#include <syslog.h>
#endif

#include "ifd_api.h"
#include "card_link.h"
#include "t1.h"
#include "presence.h"

#define IFSD            251u    /* docs/card.md: keep blocks under 256 B */
#define ATR_LIMIT_MS    3000u
#define MAX_APDU        261u
#define MAX_RESP        (65536u + 2u)

#ifdef __APPLE__
#define LOG(...) os_log(cfw_log(), __VA_ARGS__)
#define LOGE(...) os_log_error(cfw_log(), __VA_ARGS__)
#else
static void cfw_syslog(int prio, const char *fmt, ...);
#define LOG(...) cfw_syslog(LOG_INFO, __VA_ARGS__)
#define LOGE(...) cfw_syslog(LOG_ERR, __VA_ARGS__)
#endif

struct slot {
    int created;
    struct card_link link;
    struct presence pr;         /* powered, seen, when to ask: presence.h */
    uint8_t atr[ATR_MAX];
    size_t natr;
    struct atr_info ai;
    struct t1 t1;
    int allow_apdu;
    int arm_core;               /* CFWCardARM: -1 main FR, 0 ARM-E, 1 ARM-C */
};

static struct slot g_slot;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

#ifdef __APPLE__
static os_log_t cfw_log(void)
{
    static os_log_t l;

    if (l == NULL)
        l = os_log_create("cfw.ifd", "m2tv");
    return l;
}
#else
/* The formats are written for os_log, whose "%{public}s" marks a string as
 * not private.  syslog has no such marker: drop it, then format. */
static void cfw_syslog(int prio, const char *fmt, ...)
{
    char plain[256];
    const char *mark = "{public}";
    size_t i = 0, n = strlen(mark);
    va_list ap;

    while (*fmt != '\0' && i + 1 < sizeof(plain)) {
        if (strncmp(fmt, mark, n) == 0) {
            fmt += n;
            continue;
        }
        plain[i++] = *fmt++;
    }
    plain[i] = '\0';
    openlog("ifd-cfw-m2tv", LOG_PID, LOG_DAEMON);
    va_start(ap, fmt);
    vsyslog(prio, plain, ap);
    va_end(ap);
}
#endif

/* --- configuration ---------------------------------------------------- */

/* Read this bundle's own keys from its Info.plist.  The library is at
 * <bundle>/Contents/MacOS/<name> (macOS) or <bundle>/Contents/Linux/<name>
 * (pcsc-lite), so the bundle is three levels up.
 *
 *   CFWAllowAPDU  boolean   send APDUs at all (default false)
 *   CFWCardARM    integer   0 ARM-E or 1 ARM-C does the card's waiting
 *                           (card_link.h's ARM path); absent = main FR
 *   CFWCardRemovable boolean  keep asking whether the card is there
 *                           (presence.h); default false, the GV-M2TV's
 *                           card cannot be pulled
 *
 * The system starts pcscd, so a driver installed there cannot be given an
 * environment variable: this is the way to pick the ARM path for it.  The
 * variable CFW_CARD_ARM (or M2TV_CARD_ARM) still wins when it is set
 * (tests run by hand). */
#ifndef __APPLE__
/* The value element that follows <key>name</key> in the plist text, from
 * its '<' on, or NULL.  Enough for the two flat keys read here. */
static const char *plist_value(const char *text, const char *name)
{
    char key[64];
    const char *p;

    snprintf(key, sizeof(key), "<key>%s</key>", name);
    p = strstr(text, key);
    if (p == NULL)
        return NULL;
    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')
        p++;
    return *p == '<' ? p : NULL;
}

static void read_plist(struct slot *s, const char *bundle)
{
    char path[1100], text[8192];
    const char *v;
    size_t got;
    FILE *f;

    snprintf(path, sizeof(path), "%s/Contents/Info.plist", bundle);
    f = fopen(path, "r");
    if (f == NULL)
        return;
    got = fread(text, 1, sizeof(text) - 1, f);
    fclose(f);
    text[got] = '\0';
    v = plist_value(text, "CFWAllowAPDU");
    if (v != NULL && strncmp(v, "<true/>", 7) == 0)
        s->allow_apdu = 1;
    v = plist_value(text, "CFWCardARM");
    if (v != NULL && strncmp(v, "<integer>", 9) == 0 &&
        (v[9] == '0' || v[9] == '1') && v[10] == '<')
        s->arm_core = v[9] - '0';
    v = plist_value(text, "CFWCardRemovable");
    if (v != NULL && strncmp(v, "<true/>", 7) == 0)
        s->pr.removable = 1;
}
#endif

static void read_config(struct slot *s)
{
    Dl_info info;
    char path[1024];
#ifdef __APPLE__
    CFURLRef url;
    CFBundleRef b;
#endif
    int i;

    s->allow_apdu = 0;
    s->arm_core = -1;
    s->pr.removable = 0;
    /* Any object of this library will do; an object, not a function,
     * because ISO C has no function-to-object pointer conversion. */
    if (dladdr((const void *)&g_slot, &info) == 0 ||
        info.dli_fname == NULL)
        return;
    snprintf(path, sizeof(path), "%s", info.dli_fname);
    for (i = 0; i < 3; i++) {
        char *p = strrchr(path, '/');

        if (p == NULL)
            return;
        *p = '\0';
    }
#ifndef __APPLE__
    read_plist(s, path);
#else
    url = CFURLCreateFromFileSystemRepresentation(
        NULL, (const UInt8 *)path, (CFIndex)strlen(path), true);
    if (url == NULL)
        return;
    b = CFBundleCreate(NULL, url);
    CFRelease(url);
    if (b != NULL) {
        CFTypeRef v = CFBundleGetValueForInfoDictionaryKey(
            b, CFSTR("CFWAllowAPDU"));
        int n;

        if (v != NULL && CFGetTypeID(v) == CFBooleanGetTypeID())
            s->allow_apdu = CFBooleanGetValue((CFBooleanRef)v) ? 1 : 0;
        v = CFBundleGetValueForInfoDictionaryKey(b, CFSTR("CFWCardARM"));
        if (v != NULL && CFGetTypeID(v) == CFNumberGetTypeID() &&
            CFNumberGetValue((CFNumberRef)v, kCFNumberIntType, &n) &&
            (n == 0 || n == 1))
            s->arm_core = n;
        v = CFBundleGetValueForInfoDictionaryKey(b,
                                                 CFSTR("CFWCardRemovable"));
        if (v != NULL && CFGetTypeID(v) == CFBooleanGetTypeID())
            s->pr.removable = CFBooleanGetValue((CFBooleanRef)v) ? 1 : 0;
        CFRelease(b);
    }
#endif
}

/* card_open() takes the path from CFW_CARD_ARM (or M2TV_CARD_ARM); without it, the bundle's
 * CFWCardARM decides. */
static int slot_open(struct slot *s)
{
    if (card_open(&s->link) != 0)
        return -1;
    if (card_arm_env() == NULL)
        s->link.arm = s->arm_core;
    return 0;
}

/* --- device access ---------------------------------------------------- */

/* Open the device and make sure CFW is the one that will answer.  -1 the
 * device did not open, -2 it opened but CFW is not hooked (closed again). */
static int dev_begin(struct slot *s)
{
    if (slot_open(s) != 0) {
        LOGE("open: %{public}s", s->link.err);
        return -1;
    }
    if (card_hooked(&s->link) != 1) {
        LOG("%{public}s", s->link.err);
        card_close(&s->link);
        return -2;
    }
    return 0;
}

static void dev_end(struct slot *s)
{
    card_close(&s->link);
}

static void forget(struct slot *s)
{
    s->pr.powered = 0;
    s->natr = 0;
    s->pr.known_present = 0;
    s->pr.next_probe = 0;
}

/* After a power-up that failed, say "no card" until the next probe.  The
 * system marks a card that did not answer as mute and does not try it
 * again until it sees the card leave and come back; without this, one
 * slow power-up (a recording holding EP0, say) left the slot mute until
 * the device was replugged. */
static void drop_card(struct slot *s)
{
    s->pr.powered = 0;
    s->natr = 0;
    s->pr.known_present = 0;
    s->pr.next_probe = card_now_ms() + PRESENCE_EVERY_MS;
}

static RESPONSECODE rc_from_card(struct slot *s, int rc)
{
    if (rc == CARD_SILENT || rc == CARD_XPORT)
        forget(s);
    return rc == CARD_SILENT ? IFD_ICC_NOT_PRESENT : IFD_COMMUNICATION_ERROR;
}

/* --- channel ---------------------------------------------------------- */

static RESPONSECODE create(DWORD Lun, const char *name)
{
    struct slot *s = &g_slot;
    RESPONSECODE rc = IFD_SUCCESS;

    pthread_mutex_lock(&g_lock);
    LOG("create channel lun %08x name '%{public}s'", (unsigned)Lun,
        name ? name : "(none)");
    memset(s, 0, sizeof(*s));
    read_config(s);
    LOG("APDUs %{public}s", s->allow_apdu ? "allowed (CFWAllowAPDU)"
                                           : "refused (default)");
    LOG("presence %{public}s", s->pr.removable
        ? "asked every 500 ms (CFWCardRemovable)"
        : "kept once seen (the card cannot be pulled)");
    LOG("card path %{public}s", card_arm_env() ? "from CFW_CARD_ARM / M2TV_CARD_ARM"
        : s->arm_core == 0 ? "ARM-E (CFWCardARM)"
        : s->arm_core == 1 ? "ARM-C (CFWCardARM)" : "main FR (default)");
    /* The device must open; CFW may be installed later (presence asks
     * again every few seconds). */
    if (slot_open(s) != 0) {
        LOGE("open: %{public}s", s->link.err);
        rc = IFD_COMMUNICATION_ERROR;
    } else {
        if (card_hooked(&s->link) != 1)
            LOG("%{public}s", s->link.err);
        card_close(&s->link);
    }
    s->created = rc == IFD_SUCCESS;
    pthread_mutex_unlock(&g_lock);
    return rc;
}

RESPONSECODE IFDHCreateChannelByName(DWORD Lun, LPSTR DeviceName)
{
    return create(Lun, DeviceName);
}

RESPONSECODE IFDHCreateChannel(DWORD Lun, DWORD Channel)
{
    char name[32];

    snprintf(name, sizeof(name), "channel %u", (unsigned)Channel);
    return create(Lun, name);
}

RESPONSECODE IFDHCloseChannel(DWORD Lun)
{
    struct slot *s = &g_slot;

    pthread_mutex_lock(&g_lock);
    LOG("close channel lun %08x", (unsigned)Lun);
    if (s->pr.powered && dev_begin(s) == 0) {
        (void)card_off(&s->link);
        dev_end(s);
    }
    memset(s, 0, sizeof(*s));
    pthread_mutex_unlock(&g_lock);
    return IFD_SUCCESS;
}

/* --- capabilities ----------------------------------------------------- */

static RESPONSECODE put_value(PDWORD Length, PUCHAR Value, const void *v,
                              size_t n)
{
    if (*Length < n)
        return IFD_ERROR_INSUFFICIENT_BUFFER;
    memcpy(Value, v, n);
    *Length = (DWORD)n;
    return IFD_SUCCESS;
}

RESPONSECODE IFDHGetCapabilities(DWORD Lun, DWORD Tag, PDWORD Length,
                                 PUCHAR Value)
{
    struct slot *s = &g_slot;
    RESPONSECODE rc;
    uint8_t one = 1, zero = 0;
    uint32_t maxin = MAX_APDU;

    (void)Lun;
    pthread_mutex_lock(&g_lock);
    switch (Tag) {
    case TAG_IFD_ATR:
    case SCARD_ATTR_ATR_STRING:
        rc = put_value(Length, Value, s->atr, s->natr);
        break;
    case TAG_IFD_SLOTNUM:
        rc = put_value(Length, Value, &zero, 1);
        break;
    case TAG_IFD_SLOTS_NUMBER:
    case TAG_IFD_SIMULTANEOUS_ACCESS:
        rc = put_value(Length, Value, &one, 1);
        break;
    case TAG_IFD_THREAD_SAFE:
    case TAG_IFD_SLOT_THREAD_SAFE:
        rc = put_value(Length, Value, &zero, 1);
        break;
    case SCARD_ATTR_MAXINPUT:
        rc = put_value(Length, Value, &maxin, sizeof(maxin));
        break;
    default:
        rc = IFD_ERROR_TAG;
        break;
    }
    pthread_mutex_unlock(&g_lock);
    return rc;
}

RESPONSECODE IFDHSetCapabilities(DWORD Lun, DWORD Tag, DWORD Length,
                                 PUCHAR Value)
{
    (void)Lun; (void)Tag; (void)Length; (void)Value;
    return IFD_NOT_SUPPORTED;
}

/* No PPS: the card runs in the mode its ATR names (TA2 present: specific
 * mode with TA1's F and D; absent: the defaults), which needs none.  Only
 * T=1 is spoken. */
RESPONSECODE IFDHSetProtocolParameters(DWORD Lun, DWORD Protocol, UCHAR Flags,
                                       UCHAR PTS1, UCHAR PTS2, UCHAR PTS3)
{
    struct slot *s = &g_slot;
    RESPONSECODE rc = IFD_SUCCESS;

    (void)Lun; (void)PTS2; (void)PTS3;
    pthread_mutex_lock(&g_lock);
    LOG("set protocol %08x flags %02x pts1 %02x", (unsigned)Protocol, Flags,
        PTS1);
    if (Protocol != SCARD_PROTOCOL_T1)
        rc = IFD_PROTOCOL_NOT_SUPPORTED;
    else if (!s->pr.powered)
        rc = IFD_COMMUNICATION_ERROR;
    pthread_mutex_unlock(&g_lock);
    return rc;
}

/* --- power ------------------------------------------------------------ */

static void hex(char *out, size_t cap, const uint8_t *b, size_t n)
{
    size_t i, o = 0;

    out[0] = '\0';
    for (i = 0; i < n && o + 4 < cap; i++)
        o += (size_t)snprintf(out + o, cap - o, i ? " %02X" : "%02X", b[i]);
}

static RESPONSECODE power_up(struct slot *s)
{
    const char *why;
    char text[3 * ATR_MAX + 1];
    int rc;

    s->pr.powered = 0;
    s->natr = 0;
    rc = card_off(&s->link);
    if (rc != FRQ1_ST_OK) {
        LOGE("CARD_OFF: %{public}s", card_rc_name(rc));
        return rc_from_card(s, rc);
    }
    rc = card_on(&s->link, s->atr, &s->natr, ATR_LIMIT_MS);
    if (rc != FRQ1_ST_OK) {
        LOGE("power up: %{public}s", s->link.err);
        (void)rc_from_card(s, rc);
        drop_card(s);
        return rc == CARD_XPORT ? IFD_COMMUNICATION_ERROR
                                : IFD_ICC_NOT_PRESENT;
    }
    hex(text, sizeof(text), s->atr, s->natr);
    LOG("ATR %{public}s", text);

    why = atr_parse(s->atr, s->natr, &s->ai);
    if (why != NULL) {
        LOGE("ATR: %{public}s", why);
        (void)card_off(&s->link);
        s->natr = 0;
        return IFD_COMMUNICATION_ERROR;
    }
    if (!(s->ai.protocols & 2u) || s->ai.crc) {
        LOGE("card offers no T=1 with LRC (protocols %x, crc %d)",
             s->ai.protocols, s->ai.crc);
        (void)card_off(&s->link);
        s->natr = 0;
        return IFD_PROTOCOL_NOT_SUPPORTED;
    }
    rc = card_param(&s->link, s->ai.fi, s->ai.di, s->ai.cwi, s->ai.bwi);
    if (rc != FRQ1_ST_OK) {
        LOGE("CARD_PARAM F %u D %u CWI %u BWI %u: %{public}s", s->ai.fi,
             s->ai.di, s->ai.cwi, s->ai.bwi, card_rc_name(rc));
        (void)card_off(&s->link);
        s->natr = 0;
        return rc_from_card(s, rc);
    }
    t1_init(&s->t1, card_xfer, &s->link, &s->ai, IFSD);
    if (t1_set_ifsd(&s->t1) != T1_XFER_OK) {
        LOGE("S(IFS %u): %{public}s / %{public}s", IFSD, s->t1.err,
             s->link.err);
        (void)card_off(&s->link);
        s->natr = 0;
        return IFD_COMMUNICATION_ERROR;
    }
    LOG("T=1 up: F %u D %u IFSC %u IFSD %u CWI %u BWI %u", s->ai.fi,
        s->ai.di, s->t1.ifsc, IFSD, s->ai.cwi, s->ai.bwi);
    s->pr.powered = 1;
    s->pr.known_present = 1;
    return IFD_SUCCESS;
}

RESPONSECODE IFDHPowerICC(DWORD Lun, DWORD Action, PUCHAR Atr,
                          PDWORD AtrLength)
{
    struct slot *s = &g_slot;
    RESPONSECODE rc;
    DWORD cap = AtrLength ? *AtrLength : 0;

    (void)Lun;
    if (AtrLength)
        *AtrLength = 0;
    pthread_mutex_lock(&g_lock);
    LOG("power action %u", (unsigned)Action);
    if (dev_begin(s) != 0) {
        forget(s);
        pthread_mutex_unlock(&g_lock);
        return IFD_COMMUNICATION_ERROR;
    }
    switch (Action) {
    case IFD_POWER_DOWN: {
        int r = card_off(&s->link);

        s->pr.powered = 0;
        s->natr = 0;
        rc = r == FRQ1_ST_OK ? IFD_SUCCESS : rc_from_card(s, r);
        break;
    }
    case IFD_POWER_UP:
    case IFD_RESET:
        rc = power_up(s);
        if (rc == IFD_SUCCESS && Atr != NULL && AtrLength != NULL) {
            size_t n = s->natr <= cap || cap == 0 ? s->natr : cap;

            memcpy(Atr, s->atr, n);
            *AtrLength = (DWORD)n;
        }
        break;
    default:
        rc = IFD_ERROR_NOT_SUPPORTED;
        break;
    }
    dev_end(s);
    pthread_mutex_unlock(&g_lock);
    return rc;
}

/* --- data ------------------------------------------------------------- */

RESPONSECODE IFDHTransmitToICC(DWORD Lun, SCARD_IO_HEADER SendPci,
                               PUCHAR TxBuffer, DWORD TxLength,
                               PUCHAR RxBuffer, PDWORD RxLength,
                               PSCARD_IO_HEADER RecvPci)
{
    static uint8_t resp[MAX_RESP];
    struct slot *s = &g_slot;
    RESPONSECODE rc = IFD_SUCCESS;
    size_t nresp = 0;
    DWORD cap = *RxLength;

    (void)Lun;
    *RxLength = 0;
    pthread_mutex_lock(&g_lock);
    if (!s->pr.powered) {
        rc = IFD_COMMUNICATION_ERROR;
        goto out;
    }
    if (TxLength < 4u) {
        rc = IFD_COMMUNICATION_ERROR;
        goto out;
    }
    if (!s->allow_apdu) {
        LOG("refused APDU %02X %02X %02X %02X (%u bytes, proto %u): "
            "CFWAllowAPDU is off", TxBuffer[0], TxBuffer[1], TxBuffer[2],
            TxBuffer[3], (unsigned)TxLength, (unsigned)SendPci.Protocol);
        rc = IFD_COMMUNICATION_ERROR;
        goto out;
    }
    if (dev_begin(s) != 0) {
        forget(s);
        rc = IFD_COMMUNICATION_ERROR;
        goto out;
    }
    if (t1_transceive(&s->t1, TxBuffer, TxLength, resp, sizeof(resp),
                      &nresp) != 0) {
        LOGE("T=1: %{public}s / %{public}s", s->t1.err, s->link.err);
        rc = IFD_COMMUNICATION_ERROR;
    } else if (nresp > cap) {
        rc = IFD_ERROR_INSUFFICIENT_BUFFER;
    } else {
        memcpy(RxBuffer, resp, nresp);
        *RxLength = (DWORD)nresp;
        if (RecvPci != NULL) {
            RecvPci->Protocol = SendPci.Protocol;
            RecvPci->Length = sizeof(*RecvPci);
        }
    }
    dev_end(s);
out:
    pthread_mutex_unlock(&g_lock);
    return rc;
}

RESPONSECODE IFDHControl(DWORD Lun, DWORD dwControlCode, PUCHAR TxBuffer,
                         DWORD TxLength, PUCHAR RxBuffer, DWORD RxLength,
                         LPDWORD pdwBytesReturned)
{
    (void)Lun; (void)dwControlCode; (void)TxBuffer; (void)TxLength;
    (void)RxBuffer; (void)RxLength;
    if (pdwBytesReturned)
        *pdwBytesReturned = 0;
    return IFD_ERROR_NOT_SUPPORTED;
}

/* --- presence --------------------------------------------------------- */

RESPONSECODE IFDHICCPresence(DWORD Lun)
{
    struct slot *s = &g_slot;
    enum presence_step step;

    (void)Lun;
    pthread_mutex_lock(&g_lock);
    step = presence_plan(&s->pr, card_now_ms());
    if (step == PRESENCE_ASK) {
        int answered = 0, present = 0, held = presence_held(&s->pr);

        if (dev_begin(s) == 0) {
            unsigned stage = 0;
            uint32_t event = 0;
            int r = card_status(&s->link, &stage, &event);

            dev_end(s);
            answered = r == FRQ1_ST_OK;
            present = answered && (event & CARD_EVENT_PRESENT);
            if (present && !held)
                LOG("card present: CARD_EVENT %08x stage %u", event, stage);
            else if (answered && !present && held)
                LOG("card gone: CARD_EVENT %08x stage %u", event, stage);
            else if (!answered)
                LOG("presence: %{public}s", card_rc_name(r));
        }
        step = presence_learn(&s->pr, answered, present);
        if (step == PRESENCE_NO)
            s->natr = 0;
    }
    pthread_mutex_unlock(&g_lock);
    return step == PRESENCE_YES ? IFD_ICC_PRESENT : IFD_ICC_NOT_PRESENT;
}
