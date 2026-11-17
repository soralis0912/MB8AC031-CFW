/* armipc.h -- the work queue between the main FR and our ARM image.
 *
 * Included by BOTH sides (fr30-elf and arm-none-eabi) and by the host
 * sample, like armcore.h: plain #defines only.  src/armcore/README.md has
 * the design; this file is the wire.
 *
 * WHERE.  In DTCM, right below the mailbox (armcore.h).  The ARM image's
 * stack starts below this (src/armcore/link.ld).
 *
 *     ARM side   ARM_IPC      = ARM_MBOX - 0x500    (0x0010FA00)
 *     main side  ARM_IPC_OFF  = ARM_MBOX_OFF - 0x500 (window + 0x8FA00)
 *
 * TWO RINGS, ONE WRITER PER WORD.  Requests go main FR -> ARM, replies go
 * ARM -> main FR.  Each ring has a head (the count of slots ever filled)
 * written only by the producer, and a tail (the count ever taken) written
 * only by the consumer.  So no word is written by both sides and nothing
 * needs a lock.  head - tail is how many are waiting (unsigned arithmetic,
 * so the counts may wrap).  The slot for count n is n % IPC_SLOTS.
 *
 * The producer writes the whole slot first and the head last; the consumer
 * reads the head first and the slot after.  The ARM runs with caches off,
 * so a store is in DTCM when the instruction is done.
 *
 * Every field is a whole word.  dione's window takes word accesses only,
 * and stores narrower than a word into memory another master reads are not
 * trusted here (armcore.h).
 *
 * STOPPING WITHOUT THE RINGS.  A STOP request needs room in the request
 * ring, and the ARM takes requests only while the reply ring has room.  So
 * when the host stops reading replies, or 16 requests wait, a STOP cannot
 * get through.  The cancel words go around that: the main FR writes
 * CANCEL_WHAT (a request's SEQ, or IPC_CANCEL_ALL) and then bumps
 * CANCEL_REQ; the ARM checks CANCEL_REQ against CANCEL_ACK on every pass,
 * before anything else, whatever state the rings are in.
 *
 *   - a SEQ: the task that request started (a WAIT or a START) ends at once,
 *     with no reply.  CANCEL_DONE says 1 if there was one, 0 if not (it had
 *     finished, or it is still waiting in the request ring)
 *   - IPC_CANCEL_ALL: every task ends with no reply, and every request
 *     still waiting in the request ring is dropped (CANCEL_DROPPED)
 *
 * The ARM writes CANCEL_DONE and CANCEL_DROPPED, then CANCEL_ACK =
 * CANCEL_REQ last.  Replies already in the reply ring stay there.  The main
 * FR takes a new cancel only once the last one is acknowledged.
 *
 * THE BUFFER.  A slot carries 4 data words; a card block is up to 259
 * bytes.  So bulk data goes through ARM_BUF, the 0x400 bytes below the
 * queue, and the slots only say how much.  It has two halves with one
 * writer each, like the rings: BUF_TX is written by the main FR (ARM_PUT)
 * and read by the ARM; BUF_RX is written by the ARM and read by the main
 * FR (ARM_GET).  The ARM fills BUF_RX before it queues the reply that says
 * so, and reads BUF_TX only after it has taken the request that says it is
 * ready.  Bytes are packed into words big-endian by value: byte i is bits
 * 31-8*(i%4) .. 24-8*(i%4) of word i/4.  Both sides handle whole words only,
 * so how the bus lays out bytes does not matter.
 *
 * WHO CLEARS IT.  The main FR zeroes the header before it starts the core
 * (ARM_RUN), so both sides begin at count 0.  The ARM writes IPC_MAGIC last
 * in its start-up; until then the main FR refuses to queue.
 */
#ifndef CFW_ARMIPC_H
#define CFW_ARMIPC_H

#include "armcore.h"

#define ARM_IPC             (ARM_MBOX - 0x500u)         /* ARM side   */
#define ARM_IPC_OFF         (ARM_MBOX_OFF - 0x500u)     /* main side  */
#define ARM_IPC_BYTES       0x500u

/* --- header ----------------------------------------------------------- */
#define IPC_MAGIC           0x00u   /* ARM: IPC_MAGIC_VAL once it listens   */
#define IPC_REQ_HEAD        0x04u   /* main FR: requests ever queued        */
#define IPC_REQ_TAIL        0x08u   /* ARM: requests ever taken             */
#define IPC_REP_HEAD        0x0cu   /* ARM: replies ever queued             */
#define IPC_REP_TAIL        0x10u   /* main FR: replies ever taken          */
#define IPC_EVENTS          0x14u   /* ARM: events ever queued (a count)    */
#define IPC_CANCEL_REQ      0x18u   /* main FR: cancels ever asked for      */
#define IPC_CANCEL_WHAT     0x1cu   /* main FR: a SEQ, or IPC_CANCEL_ALL    */
#define IPC_CANCEL_ACK      0x20u   /* ARM: CANCEL_REQ once handled         */
#define IPC_CANCEL_DONE     0x24u   /* ARM: tasks the last cancel ended     */
#define IPC_CANCEL_DROPPED  0x28u   /* ARM: requests it dropped unread      */
#define IPC_HEADER_WORDS    16u

#define IPC_CANCEL_ALL      0xffffffffu

#define IPC_MAGIC_VAL       0x49504331u     /* 'IPC1' */

/* --- the buffer (THE BUFFER above) ------------------------------------ */
#define ARM_BUF             (ARM_IPC - 0x400u)          /* ARM side   */
#define ARM_BUF_OFF         (ARM_IPC_OFF - 0x400u)      /* main side  */
#define ARM_BUF_BYTES       0x400u
#define BUF_TX              0x000u  /* main FR writes, ARM reads            */
#define BUF_RX              0x200u  /* ARM writes, main FR reads            */
#define BUF_HALF            0x200u

/* --- time -------------------------------------------------------------
 * No timer on the ARM is known, so it counts passes of its loop.  An idle
 * pass took 0.70 us and a pass with a watch running about 0.59 us (GV-M2TV,
 * measured); 1800 passes a millisecond assumes 0.556 us, so a wait given
 * in ms lasts at least that long.  Work in a pass only makes it slower. */
#define IPC_PASSES_PER_MS   1800u

/* --- rings ------------------------------------------------------------ */
#define IPC_SLOTS           16u
#define IPC_SLOT_BYTES      0x20u
#define IPC_REQ             0x040u  /* request ring, IPC_SLOTS slots        */
#define IPC_REP             0x240u  /* reply ring, IPC_SLOTS slots          */
#define IPC_END             0x440u  /* must stay <= ARM_IPC_BYTES           */

/* A request slot.  SEQ is the request's count after queueing (1, 2, ...):
 * the main FR assigns it and the reply carries it back. */
#define IPC_Q_SEQ           0x00u
#define IPC_Q_CALL          0x04u   /* IPC_CALL(service, op)                */
#define IPC_Q_ARG0          0x08u
#define IPC_Q_ARG1          0x0cu
#define IPC_Q_DATA          0x10u   /* 4 words                              */

/* A reply slot.  An EVENT is a reply nobody asked for this time: it carries
 * the SEQ of the request that set it up (a watch) and IPC_EVENT in CALL. */
#define IPC_R_SEQ           0x00u
#define IPC_R_CALL          0x04u   /* the request's CALL, | IPC_EVENT      */
#define IPC_R_STATUS        0x08u   /* IPC_ST_*                             */
#define IPC_R_VAL           0x0cu
#define IPC_R_DATA          0x10u   /* 4 words                              */

#define IPC_DATA_WORDS      4u

#define IPC_CALL(svc, op)   (((svc) << 8) | (op))
#define IPC_CALL_SVC(c)     (((c) >> 8) & 0xffu)
#define IPC_CALL_OP(c)      ((c) & 0xffu)
#define IPC_EVENT           0x80000000u

/* --- statuses --------------------------------------------------------- */
#define IPC_ST_OK           0u
#define IPC_ST_BADSVC       1u      /* no such service                      */
#define IPC_ST_BADOP        2u      /* the service has no such op           */
#define IPC_ST_ABORT        3u      /* the access took a data abort         */
#define IPC_ST_BUSY         4u      /* no free slot in the service          */
#define IPC_ST_TIMEOUT      5u      /* a wait ran out                       */
#define IPC_ST_ARG          6u      /* a bad argument                       */
#define IPC_ST_STOPPED      7u      /* a watch ended (STOP)                 */
#define IPC_ST_IO           8u      /* the hardware said something wrong    */
#define IPC_ST_STATE        9u      /* not now: the service is not ready    */

/* --- services ----------------------------------------------------------
 * SVC_SYS: answered at once.
 *   ECHO   arg0 = x, data = 4 words  -> val = ~x, data echoed
 *   INFO                             -> val = IPC_VERSION, data = LOOPS,
 *                                       CP15 main ID, requests served,
 *                                       events queued
 *   PEEK   arg0 = addr, arg1 = width (32/16/8)          -> val = [addr]
 *   POKE   arg0 = addr, arg1 = width, data[0] = value   -> val = read back
 *   SUM    arg0 = addr (word aligned), arg1 = words (1..16M)
 *          -> answered LATER: val = the words folded (rotate left 1, xor),
 *             data[0] = passes it took (256 words a pass).  For timing
 *             how fast the ARM reads; the host keeps the clock
 *   EPTEST arg0 = seed (high half) | pair (bit0: 0 = EP 0x81, 1 = EP 0x84),
 *          arg1 = words (1..128) | spins << 16, bit31 = the
 *          spins read DTCM instead of the status register (an
 *          experiment's delay before the first write)
 *          -> one burst into that pair's FIFO, the way the main FR's ts_dione.c
 *             does it: word i = (seed & 0xFFFF0000) | i into the FIFO, then
 *             guard 8, commit, guard 0xF.  The FIFO is never read.  At
 *             once: val = the status register before, data[0] after,
 *             data[1] the guard after.  IPC_ST_BUSY if the status says
 *             the host has not taken the last burst (bit2 clear)
 *
 * SVC_WATCH: the work the ARM is here for -- waiting.
 *   WAIT   arg0 = addr, arg1 = timeout in loops (0 = none),
 *          data[0] = mask, data[1] = value
 *          -> answered LATER, when ([addr] & mask) == value
 *             (val = [addr]) or when the timeout runs out (IPC_ST_TIMEOUT)
 *   START  arg0 = addr, data[0] = mask
 *          -> val = watch number, data[0] = [addr] & mask, at once; then
 *             an EVENT each time ([addr] & mask) changes: val = new,
 *             data[0] = old
 *   STOP   arg0 = watch number -> IPC_ST_OK at once; the watch's last
 *          reply is an EVENT with IPC_ST_STOPPED
 *
 * SVC_CARD: the card interface (ISO 7816, docs/card.md), owned by the ARM
 * while it has it.  Stages are card.c's: 0 off, 1..17 power-up steps, 18
 * collecting the ATR, 19 ready, 20 a block sent, 21 a block back.
 *   STATUS                           -> val = stage, data = CARD_EVENT,
 *                                       CARD_PINS, CARD_MODE, ATR length
 *   ON     arg0 = polarity (0/1), arg1 = ATR timeout in ms (0 = 1000)
 *          -> answered LATER: val = ATR length, the ATR in BUF_RX.
 *             IPC_ST_TIMEOUT (val = bytes so far), IPC_ST_IO (not an ATR).
 *             If already up: at once, the same answer
 *   OFF                              -> powers down at once.  A task in
 *                                       progress answers IPC_ST_STOPPED
 *   PARAM  arg0 = F << 16 | D, arg1 = CWI << 8 | BWI  (card.c's CARD_PARAM)
 *   XFER   arg0 = bytes of BUF_TX to send (a whole T=1 block, LRC
 *          included, 1..255), arg1 = timeout in ms (0 = 2000)
 *          -> answered LATER: val = bytes back, the block in BUF_RX.
 *             IPC_ST_TIMEOUT (val = bytes so far, data[0] = bytes needed),
 *             IPC_ST_IO (more than one block came)
 *   One card task at a time: a second ON or XFER while one runs is
 *   IPC_ST_BUSY; PARAM or XFER before the ATR is IPC_ST_STATE.  A cancel
 *   (the cancel words) ends the task with no reply; cancelling an ON also
 *   powers the card down.
 *
 * No SVC_TS.  EP 0x81 stays on the main FR (ts_dione.c): an ARM core that
 * filled its FIFO lost bytes whenever an EP0 transfer ran at the same time
 * (docs/arm.md).
 *
 * Addresses are the ARM's own (armcore.h / docs/arm.md: peripherals at the
 * main FR's addresses, main SRAM at 0x00F40000, DRAM physical).  A data
 * abort comes back as IPC_ST_ABORT; the image keeps running. */
#define IPC_VERSION         2u      /* 2: SVC_CARD and the buffer          */

#define SVC_SYS             0u
#define SYS_ECHO            0u
#define SYS_INFO            1u
#define SYS_PEEK            2u
#define SYS_POKE            3u
#define SYS_SUM             4u
#define SYS_EPTEST          5u

#define SVC_WATCH           1u
#define WATCH_WAIT          0u
#define WATCH_START         1u
#define WATCH_STOP          2u
#define WATCH_MAX           4u      /* WAITs and STARTs together            */

#define SVC_CARD            2u
#define CARDSVC_STATUS      0u
#define CARDSVC_ON          1u
#define CARDSVC_OFF         2u
#define CARDSVC_PARAM       3u
#define CARDSVC_XFER        4u

#define CARDST_OFF          0u
#define CARDST_STEPS        3u
#define CARDST_ATR          4u
#define CARDST_READY        5u
#define CARDST_SENT         6u
#define CARDST_GOT          7u

#endif
