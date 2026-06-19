/* =============================================================================
 * datalink.h  --  L2 Data-Link interface: framing, CRC, CSMA/CA, ACK, dup-drop
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY -- interface signatures, no implementation.
 *
 * Role: sits between L1 PHY (phy_sx1278.h) and L3 Network. Responsibilities:
 *   (1) FRAMING       -- serialize/deserialize mesh_header_t + payload to/from
 *                        raw bytes for the PHY (big-endian wire codec).
 *   (2) INTEGRITY     -- append CRC-16/CCITT on TX; verify on RX, DROP corrupt.
 *                        Also validates magic + version + length sanity.
 *   (3) MEDIUM ACCESS -- CSMA/CA: CAD before every TX, random backoff on busy,
 *                        bounded attempts; cooperates with PHY duty governor.
 *   (4) LINK ACK      -- optional per-HOP (NextHop) L2 ACK + bounded
 *                        retransmit with exponential backoff. UNICAST ONLY.
 *                        Broadcasts (NextHop==0xFFFF: BEACON/RREQ) are NEVER
 *                        ACKed (no single recipient) -> send once, no retx.
 *   (5) DUP SUPPRESS  -- short (src,seq) cache to drop link-level duplicates
 *                        (e.g. a retransmit whose ACK was lost). Distinct from
 *                        the L3 (origin,seq) broadcast-storm seen-set.
 *   (6) HANDOFF       -- deliver validated, deserialized frames up to L3 via a
 *                        fixed-size RX FIFO + callback. Accept frames from L3
 *                        for transmission via a fixed-size TX queue.
 *
 * NON-BLOCKING CONTRACT: everything is a millis()-based state machine driven by
 * link_poll(), which internally calls phy_poll(). No delay(), no busy-wait.
 *
 * WHO SETS WHICH HEADER FIELDS
 *   L3 fills: src, dst, pkt_type, ttl, hop_count, seq, and (for unicast) the
 *             resolved next_hop; plus payload + payload_len. L3 also sets
 *             FLAG_ACK_REQ if it wants link reliability for this hop.
 *   L2 fills: magic, version, reserved=0, FLAG_IS_ACK/FLAG_RETX as needed, and
 *             the trailing CRC. L2 owns retransmit bookkeeping and ACK framing.
 * ===========================================================================*/
#ifndef DATALINK_H
#define DATALINK_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"
#include "mesh_frame.h"
#include "phy_sx1278.h"

/* ----------------------------------------------------------------------------
 * TX request handed down from L3. L2 copies this (no retained pointers) into a
 * slot of its internal TX queue. `require_ack` is honoured only for unicast
 * (next_hop != ADDR_BROADCAST); it is forced false for broadcasts.
 * --------------------------------------------------------------------------*/
typedef struct {
    node_id_t  src;          /* origin endpoint (L3)                          */
    node_id_t  dst;          /* final destination (L3)                        */
    node_id_t  next_hop;     /* link recipient this hop; ADDR_BROADCAST=flood */
    pkt_type_t pkt_type;
    uint8_t    ttl;
    uint8_t    hop_count;
    seq_t      seq;          /* per-origin sequence (set by L3)               */
    bool       require_ack;  /* request per-hop L2 ACK (unicast only)         */
    uint8_t    payload_len;  /* <= MAX_PAYLOAD                                 */
    uint8_t    payload[MAX_PAYLOAD];
} link_tx_req_t;

/* ----------------------------------------------------------------------------
 * Per-slot delivery state, reported back to L3 when a TX finishes (success or
 * permanent failure) via the optional tx-complete callback.
 * --------------------------------------------------------------------------*/
typedef enum {
    LINK_TX_PENDING = 0, /* queued / in CSMA / awaiting ACK                   */
    LINK_TX_SENT,        /* sent OK (broadcast: airdone; unicast: ACKed)      */
    LINK_TX_NO_ACK,      /* unicast: MAX_RETX exhausted, no ACK -> route err  */
    LINK_TX_BUSY_FAIL,   /* CSMA: channel busy after CAD_MAX_ATTEMPTS         */
    LINK_TX_DUTY_FAIL,   /* duty budget exhausted; not sent this window       */
    LINK_TX_DROPPED      /* invalid request / queue evicted                   */
} link_tx_state_t;

/* Opaque handle identifying a queued TX so L3 can correlate completions.     */
typedef uint8_t link_tx_handle_t;
#define LINK_TX_HANDLE_INVALID 0xFF

/* ----------------------------------------------------------------------------
 * Callbacks into L3. Registered at link_begin(). Invoked from link_poll()
 * context (core0, main loop) -- NEVER from an ISR -> safe to do table work.
 * --------------------------------------------------------------------------*/

/* RX hand-off: a frame passed magic/version/CRC checks and was NOT a duplicate
 * and NOT a pure L2 ACK. `frame` is owned by L2 for the duration of the call
 * only; L3 must copy anything it wants to retain. Return value ignored. */
typedef void (*link_rx_cb_t)(const mesh_rx_frame_t *frame);

/* TX completion: final state of a previously queued send. */
typedef void (*link_tx_cb_t)(link_tx_handle_t handle, link_tx_state_t state);

typedef struct {
    link_rx_cb_t on_rx;        /* nullable: poll RX FIFO manually instead     */
    link_tx_cb_t on_tx_done;   /* nullable                                    */
} link_callbacks_t;

/* ============================================================================
 * LIFECYCLE
 * ==========================================================================*/

/* Initialize L2 (and the PHY underneath via phy_begin). `self` is this node's
 * id, used for ACK addressing and dup-cache scoping. `phy_cfg` is forwarded to
 * phy_begin (nullable -> defaults). `cb` registers L3 callbacks (nullable).
 * Leaves the radio receiving. Returns MESH_OK or a propagated MESH_ERR_*. */
mesh_err_t link_begin(node_id_t self,
                      const phy_config_t *phy_cfg /*nullable*/,
                      const link_callbacks_t *cb /*nullable*/);

/* This node's link address. */
node_id_t link_self_id(void);

/* ============================================================================
 * MAIN-LOOP PUMP  (call once per loop(); drives PHY + L2 state machines)
 * ----------------------------------------------------------------------------
 * Order of work each call:
 *   1. phy_poll()  -- finalize radio TX/RX/CAD.
 *   2. RX: if a raw frame arrived -> deframe, check magic/ver/len, verify CRC
 *          (drop on fail), dup-check. If it is an ACK addressed to us, match it
 *          to the in-flight unicast and mark delivered. Otherwise enqueue to
 *          the RX FIFO / fire on_rx. If it is a unicast addressed to us (this
 *          hop) with FLAG_ACK_REQ, schedule an ACK frame back to its sender.
 *   3. TX state machine: service the head of the TX queue through the phases
 *          CSMA(CAD/backoff) -> TRANSMIT -> (unicast) WAIT_ACK -> retx/backoff
 *          -> DONE, all on millis() deadlines.
 * Returns MESH_OK (work may or may not have happened). Never blocks. */
mesh_err_t link_poll(void);

/* ============================================================================
 * TRANSMIT  (L3 -> L2)
 * ==========================================================================*/

/* Enqueue a frame for transmission. L2 copies `req`. For unicast with
 * require_ack, L2 will CSMA-send then await an ACK and retransmit up to
 * MAX_RETX with exponential backoff. For broadcast, require_ack is ignored and
 * the frame is CSMA-sent exactly once.
 * Returns a handle (>=0 cast) on success, or LINK_TX_HANDLE_INVALID with the
 * error available via link_last_err():
 *   MESH_ERR_QUEUE_FULL, MESH_ERR_FRAME_TOO_LONG, MESH_ERR_INVALID_ARG. */
link_tx_handle_t link_send(const link_tx_req_t *req);

/* Number of free slots in the TX queue (for L3 back-pressure / store-&-fwd). */
uint8_t link_tx_free_slots(void);

/* Cancel a still-PENDING queued TX (best effort). MESH_OK or MESH_ERR_INVALID_ARG. */
mesh_err_t link_tx_cancel(link_tx_handle_t handle);

/* ============================================================================
 * RECEIVE  (L2 -> L3)  -- callback OR polled FIFO; pick one in link_begin.
 * ==========================================================================*/

/* Pull the next validated frame from the RX FIFO into `out` (when not using the
 * on_rx callback). Returns MESH_OK, or MESH_ERR_NO_RX if the FIFO is empty. */
mesh_err_t link_recv(mesh_rx_frame_t *out);

/* Number of frames waiting in the RX FIFO. */
uint8_t link_rx_pending(void);

/* ============================================================================
 * POWER / DUTY HOOKS  (thin pass-through to PHY for L3 duty-cycling policy)
 * ==========================================================================*/
mesh_err_t link_sleep(void);                 /* radio sleep (TX queue retained) */
mesh_err_t link_wake(void);                  /* standby + resume RX             */
uint32_t   link_duty_remaining_ms(void);     /* remaining TX air-time budget    */

/* ============================================================================
 * INTROSPECTION / DIAGNOSTICS
 * ==========================================================================*/
typedef struct {
    uint32_t rx_ok;          /* frames delivered up to L3                     */
    uint32_t rx_crc_drop;    /* dropped on CRC mismatch                       */
    uint32_t rx_bad_hdr;     /* dropped on magic/version/length               */
    uint32_t rx_dup_drop;    /* dropped by L2 dup cache                       */
    uint32_t tx_ok;          /* unicast ACKed or broadcast air-done           */
    uint32_t tx_retx;        /* total retransmissions issued                  */
    uint32_t tx_no_ack;      /* unicast give-ups (MAX_RETX exhausted)         */
    uint32_t tx_busy;        /* CSMA give-ups (channel busy)                  */
    uint32_t acks_sent;      /* per-hop ACKs we transmitted                   */
} link_stats_t;

link_stats_t link_get_stats(void);
mesh_err_t   link_last_err(void);   /* last error from link_send / internals  */

/* ============================================================================
 * TIMING DERIVATION NOTE (justification of the mesh_config.h constants)
 * ----------------------------------------------------------------------------
 * All L2 deadlines are expressed in terms of frame AIR-TIME (phy_airtime_ms),
 * so changing SF/BW automatically rescales them if the dynamic variants are
 * used. Default numeric constants are pre-computed for the SF9/BW125/CR4-5
 * profile:
 *
 *   ACK_TIMEOUT  = airtime(data) + peer_turnaround(~few ms) + airtime(ack)
 *                  + slack.  SF9 32B data (247ms) + 16B ACK (165ms) ~= 412ms
 *                  -> ACK_TIMEOUT_MS = 600 (margin). SF7 ~150ms; SF12 ~3.5-4s.
 *
 *   RETX backoff = RETX_BACKOFF_BASE_MS * 2^min(attempt, RETX_BACKOFF_EXP_CAP)
 *                  + random(0..RETX_JITTER_MS).  Base ~= 1 data-airtime; the
 *                  exponential term spreads repeated collisions; the random
 *                  jitter de-synchronizes nodes that backed off together.
 *
 *   CSMA backoff = random(CAD_BACKOFF_MIN_MS..CAD_BACKOFF_MAX_MS) between CAD
 *                  attempts, up to CAD_MAX_ATTEMPTS, then LINK_TX_BUSY_FAIL.
 *
 *   DUP cache    = L2_DUP_CACHE_SIZE recent (src,seq) entries aged out after
 *                  L2_DUP_TTL_MS (~ a few ACK_TIMEOUTs) so a duplicate caused
 *                  by a lost ACK is recognized while the retx window is open.
 * ==========================================================================*/

#endif /* DATALINK_H */
