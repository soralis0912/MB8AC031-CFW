/* drivers.h -- the per-device interfaces.
 *
 * This LSI does not demodulate.  The demodulator and the tuner live in a
 * module outside the chip, and which module that is depends on the product.
 * The way the firmware reaches them differs too:
 * PIX-DT295 can be driven through the Fujitsu I2C controller at 0x00FD8FE0,
 * while on the GV-MVP/FZ2 that controller gets through setup but never
 * completes a transfer, and the way in is assumed to be the auxiliary unit
 * instead.  So the transport is per-device as well, not just the parts on the
 * far end of it.
 *
 * Nothing here names a product.  Each driver implements one of these two
 * interfaces and says in its own header what it knows; config/devices/
 * says which combination a product gets, and genboards.py turns that row into
 * `fw_board` (build/board_<id>.c), linked into that product's image.  Adding
 * a board is a JSON edit.
 *
 * `source` is which front end: 0 and 1 are the two terrestrial
 * demodulators and 2 and 3 the two satellite ones (docs/demod.md).  Unit 0
 * of each is measured on a PIX-DT295 (0x10 and 0x11, demod/tc9052x.c);
 * unit 1 (0x12 and 0x13) is not confirmed on hardware.
 * A driver that has no business with one of them returns FW_ENOTIMPL for it
 * rather than pretending.
 */
#ifndef DRIVERS_H
#define DRIVERS_H

#define FW_SRC_T        0u      /* terrestrial, unit 0 (ISDB-T)           */
#define FW_SRC_T2       1u      /* terrestrial, unit 1                    */
#define FW_SRC_S        2u      /* satellite, unit 0 (ISDB-S)             */
#define FW_SRC_S2       3u      /* satellite, unit 1                      */
#define FW_SRC_COUNT    4u
#define FW_SRC_IS_SAT(s) ((s) >= FW_SRC_S)

/* How far the facts behind a table entry go.  The generated row carries
 * this so the code can refuse to drive hardware it only half knows, instead
 * of that judgement living in a comment someone will not read.
 */
#define EV_UNKNOWN      0u
#define EV_INFERRED     1u
#define EV_MEASURED     2u

struct board;

/* --- transport -------------------------------------------------------- */
struct i2c_ops {
    const char *name;
    int (*init)(const struct board *b);
    /* One transfer: write `wn` bytes, then (without releasing the bus) read
     * `rn` bytes.  Either may be zero.  `addr` is the 7-bit address.
     */
    int (*xfer)(const struct board *b, unsigned addr,
                const unsigned char *wb, unsigned wn,
                unsigned char *rb, unsigned rn);
};

/* --- demodulator ------------------------------------------------------ */
struct demod_ops {
    const char *name;
    int (*init)(const struct board *b, unsigned source);
    int (*locked)(const struct board *b, unsigned source, int *locked);
    int (*cnr_raw)(const struct board *b, unsigned source, unsigned *raw);
};

/* One row of config/devices/: the product this image was built for.
 * Every image is linked for one product (one FRC1 container each), so the
 * row is fixed at link time and nothing at run time looks it up.
 *
 * A null ops pointer means this product's part is not known; the
 * dispatchers in i2c.c / demod.c turn that into FW_ENOTIMPL.
 * Addresses are 7-bit, 0 meaning "not established".
 */
/* One receive band of the board (IDENTIFY page 1, docs/frq1.md).  The
 * edges come from the row's channel plan; which parts of it were seen to
 * lock is in `flags`. */
#define FW_BANDS        2u
#define BAND_HOST_TUNED 0x01u   /* the host tunes it (I2CXFER), not TUNE  */
#define BAND_LOCK_SEEN  0x02u   /* a lock was measured inside the band    */
struct band {
    unsigned char       source;         /* FW_SRC_*                       */
    unsigned char       flags;          /* BAND_*                         */
    unsigned short      record;         /* bytes per record, 188 for TS   */
    unsigned            lo_khz, hi_khz; /* centre frequencies, inclusive  */
    unsigned            step_hz;        /* the tuner's frequency step     */
};

struct board {
    unsigned            usb;            /* vid<<16 | pid, 0 = no product  */
    const struct i2c_ops    *i2c;
    const struct demod_ops  *demod;
    unsigned char       demod_addr[FW_SRC_COUNT];
    /* The tuner: facts only (address, how many, how sure).  The firmware
     * never drives it; the host does, by I2CXFER (docs/frq1.md). */
    unsigned char       tuner_addr;
    unsigned char       tuners;
    unsigned char       ev_demod;
    unsigned char       ev_tuner;
    /* The card interface (docs/card.md).  card_slot and card_polarity are
     * 0, 1, or CARD_UNKNOWN; card.c refuses (FW_ENOTIMPL) unless ev_card
     * says measured or inferred AND the polarity is known. */
    unsigned char       card_slot;
    unsigned char       card_polarity;
    unsigned char       ev_card;
    unsigned char       nbands;
    struct band         bands[FW_BANDS];
    /* The TS input (tsif_selene.c).  ev_ts = EV_MEASURED means this board's
     * capture sequence was measured on hardware and ts_in0 / ts_in1 are the
     * values its 0x01901000 / 0x01901004 take.  Any other ev_ts and
     * tsif_init() / tsif_start() refuse with FW_ENOTIMPL. */
    unsigned            ts_in0;
    unsigned            ts_in1;
    unsigned char       ev_ts;
};

#define CARD_UNKNOWN    0xffu

/* build/board_<id>.c, generated by host/genboards.py */
extern const struct board fw_board;

/* demod/tc9052x.c -- the register pair, for the tuner path that goes
 * through the demodulator's I2C gateway.
 */
int demod_tc9052x_reg_write(const struct board *b, unsigned source,
                            unsigned char reg, unsigned char val);
int demod_tc9052x_reg_read(const struct board *b, unsigned source,
                           unsigned char reg, unsigned char *val);
int demod_tc9052x_gateway(const struct board *b, unsigned source,
                          unsigned char addr, const unsigned char *wb,
                          unsigned wn, unsigned char *rb, unsigned rn);

#endif
