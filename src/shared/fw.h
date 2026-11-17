/* fw.h -- what each module offers.
 *
 * One header for the module APIs, because the firmware is small enough that
 * sixteen headers would cost more than they explain.  The frame itself is in
 * frq1.h, the chip registers in mb86m01.h, the image layout in resident.h.
 *
 * Every function here either works or says it does not.  A stub returns
 * FW_ENOTIMPL (or does nothing where there is nothing to fail at), so the
 * build is always honest about which parts exist.  Each module source says at
 * the top what is known about it and what is still missing.
 */
#ifndef FW_H
#define FW_H

#include "frq1.h"

#define FW_OK           0
#define FW_ENOTIMPL     (-1)    /* not written yet                        */
#define FW_EIO          (-2)    /* the hardware did not answer            */
#define FW_ETIMEOUT     (-3)
#define FW_EINVAL       (-4)
#define FW_EAGAIN       (-5)    /* nothing wrong, just not enough yet     */
#define FW_EBADCMD      (-6)    /* unknown command; only the router returns it */
#define FW_EBUSY        (-7)    /* another core has that block            */

/* These are what the host reads: the frame's status word carries them as
 * they are, with no second error space in between (docs/frq1.md). */

/* --- util.c ----------------------------------------------------------- */
void fw_memset(volatile unsigned char *p, int v, unsigned n);
void fw_memcpy(volatile unsigned char *d, const volatile unsigned char *s,
               unsigned n);
void fw_delay(unsigned loops);
unsigned fw_udiv(unsigned n, unsigned d, unsigned *rem);

/* --- clk.c ------------------------------------------------------------ */
int clk_init(void);

/* --- gpio.c ----------------------------------------------------------- */
unsigned gpio_straps(void);

/* --- hif.c ------------------------------------------------------------ */
int hif_init(void);

/* --- i2c.c ------------------------------------------------------------ */
#define FW_I2C_MAX      40u     /* one transfer's total, both transports  */
int i2c_init(void);
int i2c_xfer_rw(unsigned addr, const unsigned char *wb, unsigned wn,
                unsigned char *rb, unsigned rn);
int i2c_write(unsigned addr, const unsigned char *buf, unsigned n);
int i2c_read(unsigned addr, unsigned char *buf, unsigned n);
int i2c_reg_write(unsigned addr, unsigned char reg, unsigned char val);
int i2c_reg_read(unsigned addr, unsigned char reg, unsigned char *val);

/* --- demod.c ---------------------------------------------------------- */
int demod_init(unsigned source);
int demod_locked(unsigned source, int *locked);
int demod_cnr_raw(unsigned source, unsigned *raw);

/* --- fe.c ------------------------------------------------------------- */
int fe_init(void);
int fe_stop(void);
int fe_channel_info(frame_t f);
int fe_cnr(frame_t f);

/* --- tsif_selene.c / ring_selene.c / tsout_selene.c / stream.c ----------------------------- */
int tsif_init(void);
int tsif_start(void);
int tsif_stop(void);
int ring_init(void);
unsigned ring_used(void);
int ring_sync(void);
unsigned ring_avail(void);
unsigned ring_offset(unsigned i);
int ring_read(unsigned off, unsigned char *dst, unsigned n);
int ring_consume(unsigned n);
int ring_to_fifo(unsigned off, unsigned n, unsigned fifo);
int tsout_init(void);
int tsout_ready(int *ready);
int tsout_push(unsigned *bytes);
int stream_init(void);
int stream_start(frame_t f);
int stream_stop(void);
int stream_running(void);  /* 1 while a capture is running */

/* --- ts_dione.c ------------------------------------------------------- */
/* The dione generation's capture and copy (docs/ts.md).  stream.c hands
 * STREAM and STOP here when this image was linked for dione. */
int fw_is_dione(void);
int dione_ts_start(unsigned length);
int dione_ts_stop(void);
int dione_ts_running(void);
int dione_stream(frame_t f);

/* --- fe_dione.c ------------------------------------------------------- */
/* Front-end power and the I2C path on dione; I2CXFER for the host. */
int dione_fe_up(void);
int dione_i2cxfer(frame_t f);
int i2c_frame_xfer(frame_t f);   /* i2c.c: I2CXFER through the board's transport */

/* --- health.c -------------------------------------------------------- */
/* CMD_HEALTH: phase, flags, and the first failure per part (docs/frq1.md).
 * frq1.c tells it about every frame it answers. */
void health_started(int on);
void health_note(unsigned cmd, int rc);
int health_report(frame_t f);

/* --- card.c ----------------------------------------------------------- */
/* One per FRQ1 command in the 0x50 band (docs/card.md).  The host keeps the
 * time: a handler with a wait in it does one step and returns FW_EAGAIN. */
int card_status(frame_t f);
int card_on(frame_t f);
int card_off(frame_t f);
int card_param(frame_t f);
int card_put(frame_t f);
int card_send(frame_t f);
int card_recv(frame_t f);
int card_get(frame_t f);
int card_available(void);  /* 1 if this board's row lets card.c run */
int card_idle(void);       /* 1 if card.c has the card powered down  */
int card_polarity(void);   /* the row's polarity, -1 if no card      */

/* --- arm.c ------------------------------------------------------------ */
/* The two ARM cores, one per FRQ1 command in the 0x60 band (docs/arm.md).
 * Both cores, both generations (measured on dione only). */
int arm_status(frame_t f);
int arm_hold(frame_t f);
int arm_load(frame_t f);
int arm_run(frame_t f);
unsigned arm_pick(frame_t f, int *rc);  /* the window of arg0's core */

/* --- arm_ipc.c -------------------------------------------------------- */
/* The work queue to our ARM image (src/shared/armipc.h): ARM_CALL queues a
 * request, ARM_REAP takes a reply, ARM_CANCEL stops a task even when the
 * rings are full.  The ARM does the waiting between. */
int arm_call(frame_t f);
int arm_reap(frame_t f);
int arm_cancel(frame_t f);
int arm_put(frame_t f);
int arm_get(frame_t f);
void arm_ipc_reset(unsigned win);
int arm_has_card(void);    /* 1 while an ARM core owns the card block */

/* --- arm_dione.c / arm_selene.c --------------------------------------- */
/* Waking a core's control band, and the register that does it.  arm.c
 * picks one with fw_is_dione(). */
unsigned dione_arm_gate(void);
void dione_arm_wake(unsigned win);
unsigned selene_arm_gate(void);
void selene_arm_wake(unsigned win);

/* --- pm.c ------------------------------------------------------------- */
int pm_suspend(frame_t f);

/* --- usbep.c ---------------------------------------------------------- */
int usb_ep_ready(unsigned status_reg);
void usb_ep_commit(unsigned status_reg);
int usb_ep_burst(unsigned status_reg, unsigned fifo_reg,
                 const volatile unsigned char *src);

/* --- probe.c ---------------------------------------------------------- */
int probe_peek(frame_t f);
int probe_poke(frame_t f);
int probe_ring(frame_t f);
int probe_i2cscan(frame_t f);
int probe_epdma(frame_t f);

/* --- sys.c ------------------------------------------------------------ */
int sys_init(void);

#endif
