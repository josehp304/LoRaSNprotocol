/* =============================================================================
 * datalink.cpp  --  L2 Data-Link implementation (P3)
 * -----------------------------------------------------------------------------
 * Implements the contract in datalink.h. Sits between L1 PHY (phy_sx1278.h) and
 * L3 Network. Responsibilities (see datalink.h header for the full charter):
 *   (1) FRAMING       -- via the mesh_frame.h codec (LE, offset-based).
 *   (2) INTEGRITY     -- CRC + magic/version/length validation on RX.
 *   (3) MEDIUM ACCESS -- CSMA/CA: CAD before each data TX, bounded random backoff.
 *   (4) LINK ACK      -- per-hop ACK + bounded exponential-backoff retransmit,
 *                        UNICAST ONLY. Broadcasts are sent once, never ACKed.
 *   (5) DUP SUPPRESS  -- short (src,seq) cache to drop link-level duplicates.
 *   (6) HANDOFF       -- validated frames up to L3 (callback or RX FIFO); accepts
 *                        L3 sends into a fixed TX queue.
 *
 * RADIO IS HALF-DUPLEX + SINGLE-OWNER  -> we run ONE transmit "engine" at a time.
 * The engine drives the head of the TX queue through:
 *     IDLE -> CAD(scan/backoff) -> TRANSMIT -> [unicast] WAIT_ACK -> retx/backoff
 * ACK frames bypass CAD (tiny, time-critical, §6.5) and may PREEMPT the engine
 * only while it is in a *waiting* sub-state (the radio is otherwise idle/RX);
 * they never interrupt an in-flight CAD or data transmit.
 *
 * ACK ADDRESSING NOTE (important):
 *   The 17-byte header has NO link-source field -- the immediate sender of a hop
 *   is not recoverable from the frame. A recipient therefore cannot unicast an
 *   ACK back by id. Instead ACKs are BROADCAST and carry the acknowledged
 *   frame's (src,seq); the original sender, while awaiting an ACK for exactly
 *   that (src,seq), matches it. All other hearers classify it as an ACK, find no
 *   matching in-flight send, and drop it. (src,seq) names the in-flight packet
 *   uniquely across the mesh, so this is correct and cheap.
 *
 * NON-BLOCKING: everything is a millis()-based state machine pumped by
 * link_poll(); no delay(), no busy-wait. All storage is static (no malloc).
 * ===========================================================================*/

#include <Arduino.h>        /* millis()                                         */
#include <string.h>         /* memcpy                                           */

#include "datalink.h"
#include "phy_sx1278.h"
#include "mesh_frame.h"
#include "node_identity.h"  /* node_next_seq() for the ACK frame's own seq      */
#include "mesh_config.h"
#include "mesh_types.h"

/* ============================================================================
 * LOCAL SIZING (L2-owned; distinct from the L3 store-and-forward queue sizes).
 * ==========================================================================*/
#define LINK_TX_QUEUE_SIZE   8u    /* outstanding L3 sends buffered at L2        */
#define LINK_ACK_QUEUE_SIZE  4u    /* pending outbound ACKs                      */
#define LINK_RX_FIFO_SIZE    4u    /* validated frames buffered for polled recv  */

/* Safety bound: a CAD scan should finalize within a few symbols. If CadDone is
 * somehow never observed, treat the channel as busy and back off so the engine
 * cannot wedge. SF9 CAD is well under this; generous to cover mode-switch slack.*/
#define CAD_WAIT_TIMEOUT_MS  150u

/* ----------------------------------------------------------------------------
 * ACK payload wire layout (5 bytes). Defined HERE field-by-field (not as a
 * packed struct) per the project's binding serialization rule -- the canonical
 * production headers do not define an ACK payload struct (mesh_wire.h, which had
 * one, is superseded and must not be included).
 *   off 0  u16 LE  ack_src   (hdr.src of the frame being acknowledged)
 *   off 2  u16 LE  ack_seq   (hdr.seq of the frame being acknowledged)
 *   off 4  u8      status    (MESH_ACK_OK / NAK reasons)
 * --------------------------------------------------------------------------*/
#define ACK_PL_OFF_SRC     0u
#define ACK_PL_OFF_SEQ     2u
#define ACK_PL_OFF_STATUS  4u
#define ACK_PL_LEN         5u

#define MESH_ACK_OK          0x00u
#define MESH_ACK_NAK_QUEUE   0x01u
#define MESH_ACK_NAK_NOROUTE 0x02u

/* ============================================================================
 * TRANSMIT-ENGINE STATE
 * ==========================================================================*/
typedef enum {
    ENG_IDLE = 0,     /* no active send; may pick a queued slot or send an ACK   */
    ENG_CAD_WAIT,     /* CAD scan armed; waiting for phy_cad_result()            */
    ENG_CAD_BACKOFF,  /* channel was busy; waiting out a random backoff          */
    ENG_TX_WAIT,      /* data frame in flight; waiting for TxDone                */
    ENG_WAIT_ACK,     /* unicast sent; waiting for a matching ACK / timeout      */
    ENG_RETX_BACKOFF, /* ACK timed out; waiting out exponential backoff          */
    ENG_ACK_TX_WAIT   /* an ACK frame is in flight; resume s_eng_resume on TxDone*/
} eng_state_t;

/* ----------------------------------------------------------------------------
 * A queued transmit (a copy of the L3 request plus per-attempt bookkeeping).
 * --------------------------------------------------------------------------*/
typedef struct {
    bool          used;          /* slot occupied                               */
    bool          acked;         /* set by RX path when a matching ACK arrives  */
    uint8_t       retx;          /* retransmissions performed so far            */
    uint8_t       cad_attempts;  /* consecutive busy CADs for the current try   */
    link_tx_req_t req;           /* the caller's request (copied; no pointers)  */
} tx_slot_t;

/* ----------------------------------------------------------------------------
 * A pending outbound ACK (built lazily when the engine can send it).
 * --------------------------------------------------------------------------*/
typedef struct {
    bool      used;
    node_id_t ack_src;
    seq_t     ack_seq;
    uint8_t   status;
} ack_slot_t;

/* ----------------------------------------------------------------------------
 * L2 duplicate-suppression cache entry (src,seq) with an age stamp.
 * --------------------------------------------------------------------------*/
typedef struct {
    bool      used;
    node_id_t src;
    seq_t     seq;
    uint32_t  t_ms;
} dup_entry_t;

/* ============================================================================
 * MODULE STATE  (all static; nothing allocated after init)
 * ==========================================================================*/
static node_id_t      s_self        = ADDR_INVALID;
static link_rx_cb_t   s_on_rx       = NULL;
static link_tx_cb_t   s_on_tx_done  = NULL;
static bool           s_inited      = false;
static mesh_err_t     s_last_err    = MESH_OK;
static link_stats_t   s_stats;

/* Transmit queue + engine */
static tx_slot_t      s_tx[LINK_TX_QUEUE_SIZE];
static int            s_active      = -1;          /* index of the active slot   */
static eng_state_t    s_eng         = ENG_IDLE;
static eng_state_t    s_eng_resume  = ENG_IDLE;    /* state to resume after ACK  */

/* Engine deadlines (absolute millis()). */
static uint32_t       s_cad_start_ms      = 0;
static uint32_t       s_cad_backoff_at    = 0;
static uint32_t       s_ack_deadline      = 0;
static uint32_t       s_retx_at           = 0;

/* Outbound ACK queue */
static ack_slot_t     s_ackq[LINK_ACK_QUEUE_SIZE];
static int            s_ack_active        = -1;    /* ACK slot being transmitted */

/* Duplicate cache */
static dup_entry_t    s_dup[L2_DUP_CACHE_SIZE];

/* Receive FIFO (used only when no on_rx callback is registered) */
static mesh_rx_frame_t s_rxfifo[LINK_RX_FIFO_SIZE];
static uint8_t        s_rx_head = 0;   /* next slot to read                      */
static uint8_t        s_rx_tail = 0;   /* next slot to write                     */
static uint8_t        s_rx_count = 0;

/* Scratch wire buffers (reused; never grown). */
static uint8_t        s_tx_wire[MAX_FRAME_LEN];
static uint8_t        s_ack_wire[MAX_FRAME_LEN];
static uint8_t        s_rx_raw[MAX_FRAME_LEN];

/* Small xorshift RNG for CSMA / retx jitter (seeded in link_begin). */
static uint32_t       s_rng = 0x1234abcdUL;

/* ============================================================================
 * SMALL HELPERS
 * ==========================================================================*/

/* Wrap-safe "deadline reached" test (matches the main.cpp millis() idiom). */
static inline bool reached(uint32_t deadline_ms) {
    return (int32_t)(millis() - deadline_ms) >= 0;
}

static inline uint32_t rng_next(void) {
    /* xorshift32: cheap, adequate for backoff de-synchronization. */
    uint32_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng = x ? x : 0xA5A5A5A5UL;   /* never let the state collapse to 0 */
    return s_rng;
}

/* Uniform integer in [lo, hi] inclusive (hi >= lo). */
static uint32_t rng_range(uint32_t lo, uint32_t hi) {
    if (hi <= lo) return lo;
    return lo + (rng_next() % (hi - lo + 1u));
}

/* True while the radio is NOT actively transmitting or scanning (i.e. it is in
 * RX or STANDBY) -- the only times it is safe to inject an outbound ACK TX. */
static inline bool radio_free_for_ack(void) {
    phy_state_t st = phy_state();
    return (st == PHY_STATE_RX) || (st == PHY_STATE_STANDBY);
}

static inline bool is_broadcast_req(const link_tx_req_t *r) {
    return r->next_hop == ADDR_BROADCAST;
}

/* On-air length of a request's frame (for the duty pre-check). */
static inline uint16_t req_frame_len(const link_tx_req_t *r) {
    return (uint16_t)(HDR_LEN + (uint16_t)r->payload_len + CRC_LEN);
}

/* ============================================================================
 * FRAME BUILDERS
 * ==========================================================================*/

/* Serialize a data/control frame for slot `r` into `out`; returns wire length
 * (0 on serialize failure). `retx` sets FLAG_RETX for diagnostics. L2 owns
 * magic/version/reserved/flags; L3 owns the routing fields. */
static uint16_t build_data_frame(const link_tx_req_t *r, bool retx, uint8_t *out) {
    mesh_header_t h;
    h.magic       = FRAME_MAGIC;
    h.version     = PROTO_VERSION;
    h.src         = r->src;
    h.dst         = r->dst;
    h.next_hop    = r->next_hop;
    h.link_src    = s_self;          /* we are the one-hop sender of this frame  */
    h.hop_count   = r->hop_count;
    h.pkt_type    = (uint8_t)r->pkt_type;
    h.ttl         = r->ttl;
    h.seq         = r->seq;

    uint8_t flags = 0;
    /* ACK is requested only for unicast; broadcasts have no single recipient. */
    if (!is_broadcast_req(r) && r->require_ack) flags |= FLAG_ACK_REQ;
    if (retx)                                   flags |= FLAG_RETX;
    h.flags       = flags;
    h.reserved    = 0;
    h.payload_len = r->payload_len;

    int16_t n = mesh_frame_serialize(&h, r->payload, r->payload_len,
                                     out, MAX_FRAME_LEN);
    return (n < 0) ? 0u : (uint16_t)n;
}

/* Serialize a broadcast ACK frame acknowledging (ack_src,ack_seq). */
static uint16_t build_ack_frame(node_id_t ack_src, seq_t ack_seq,
                                uint8_t status, uint8_t *out) {
    uint8_t pl[ACK_PL_LEN];
    pl[ACK_PL_OFF_SRC]     = (uint8_t)(ack_src & 0xFFu);
    pl[ACK_PL_OFF_SRC + 1] = (uint8_t)(ack_src >> 8);
    pl[ACK_PL_OFF_SEQ]     = (uint8_t)(ack_seq & 0xFFu);
    pl[ACK_PL_OFF_SEQ + 1] = (uint8_t)(ack_seq >> 8);
    pl[ACK_PL_OFF_STATUS]  = status;

    mesh_header_t h;
    h.magic       = FRAME_MAGIC;
    h.version     = PROTO_VERSION;
    h.src         = s_self;          /* this node emits the ACK                  */
    h.dst         = ack_src;         /* informational; ACK is link-local (TTL=1) */
    h.next_hop    = ADDR_BROADCAST;  /* see ACK ADDRESSING NOTE at top of file   */
    h.link_src    = s_self;          /* we are the one-hop sender of this ACK     */
    h.hop_count   = 0;
    h.pkt_type    = (uint8_t)PKT_ACK;
    h.ttl         = 1;               /* never forwarded                          */
    h.seq         = node_next_seq(); /* the ACK frame's own seq (not the matched)*/
    h.flags       = FLAG_IS_ACK;     /* fast classification on the hearer side   */
    h.reserved    = 0;
    h.payload_len = ACK_PL_LEN;

    int16_t n = mesh_frame_serialize(&h, pl, ACK_PL_LEN, out, MAX_FRAME_LEN);
    return (n < 0) ? 0u : (uint16_t)n;
}

/* ============================================================================
 * DUPLICATE CACHE
 * ==========================================================================*/

/* Return true if (src,seq) is in the cache and not yet aged out. */
static bool dup_seen(node_id_t src, seq_t seq) {
    const uint32_t now = millis();
    for (uint8_t i = 0; i < L2_DUP_CACHE_SIZE; ++i) {
        if (s_dup[i].used && s_dup[i].src == src && s_dup[i].seq == seq) {
            if ((uint32_t)(now - s_dup[i].t_ms) < (uint32_t)L2_DUP_TTL_MS) {
                return true;          /* present and not yet aged out            */
            }
        }
    }
    return false;
}

/* Insert/refresh (src,seq). Reuse a free/expired slot, else evict the oldest. */
static void dup_insert(node_id_t src, seq_t seq) {
    const uint32_t now = millis();
    int free_idx = -1;
    int oldest_idx = 0;
    uint32_t oldest_age = 0;

    for (uint8_t i = 0; i < L2_DUP_CACHE_SIZE; ++i) {
        if (!s_dup[i].used) {
            free_idx = i;
            break;
        }
        if (s_dup[i].src == src && s_dup[i].seq == seq) {
            s_dup[i].t_ms = now;          /* refresh existing                    */
            return;
        }
        uint32_t age = (uint32_t)(now - s_dup[i].t_ms);
        if (age >= (uint32_t)L2_DUP_TTL_MS) {
            free_idx = i;                 /* expired -> reuse                    */
            break;
        }
        if (age > oldest_age) {
            oldest_age = age;
            oldest_idx = i;
        }
    }

    int idx = (free_idx >= 0) ? free_idx : oldest_idx;
    s_dup[idx].used = true;
    s_dup[idx].src  = src;
    s_dup[idx].seq  = seq;
    s_dup[idx].t_ms = now;
}

/* ============================================================================
 * RX FIFO
 * ==========================================================================*/
static void rxfifo_push(const mesh_rx_frame_t *f) {
    if (s_rx_count >= LINK_RX_FIFO_SIZE) {
        /* Full: drop the NEWEST (keep the older, in-order frames). */
        return;
    }
    s_rxfifo[s_rx_tail] = *f;
    s_rx_tail = (uint8_t)((s_rx_tail + 1u) % LINK_RX_FIFO_SIZE);
    s_rx_count++;
}

/* ============================================================================
 * OUTBOUND ACK QUEUE
 * ==========================================================================*/

/* Queue an ACK for (ack_src,ack_seq). De-duplicates against already-queued
 * ACKs; silently drops if the small queue is full (the peer will retransmit). */
static void ack_enqueue(node_id_t ack_src, seq_t ack_seq, uint8_t status) {
    int free_idx = -1;
    for (uint8_t i = 0; i < LINK_ACK_QUEUE_SIZE; ++i) {
        if (s_ackq[i].used) {
            if (s_ackq[i].ack_src == ack_src && s_ackq[i].ack_seq == ack_seq) {
                return;                    /* already queued                     */
            }
        } else if (free_idx < 0) {
            free_idx = i;
        }
    }
    if (free_idx < 0) return;              /* full: best-effort drop             */
    s_ackq[free_idx].used    = true;
    s_ackq[free_idx].ack_src = ack_src;
    s_ackq[free_idx].ack_seq = ack_seq;
    s_ackq[free_idx].status  = status;
}

static int ack_first_pending(void) {
    for (uint8_t i = 0; i < LINK_ACK_QUEUE_SIZE; ++i) {
        if (s_ackq[i].used) return (int)i;
    }
    return -1;
}

/* ============================================================================
 * TX-ENGINE TRANSITIONS
 * ==========================================================================*/

/* Pick the next ready (used, not-active) queued slot. Lowest index first. */
static int pick_ready_slot(void) {
    for (uint8_t i = 0; i < LINK_TX_QUEUE_SIZE; ++i) {
        if (s_tx[i].used && (int)i != s_active) {
            return (int)i;
        }
    }
    return -1;
}

/* Report a finished active send up to L3, update stats, free the slot. */
static void complete_active(link_tx_state_t state) {
    if (s_active < 0) {
        s_eng = ENG_IDLE;
        return;
    }
    link_tx_handle_t handle = (link_tx_handle_t)s_active;

    switch (state) {
        case LINK_TX_SENT:     s_stats.tx_ok++;     break;
        case LINK_TX_NO_ACK:   s_stats.tx_no_ack++; break;
        case LINK_TX_BUSY_FAIL:s_stats.tx_busy++;   break;
        default: break;
    }

    s_tx[s_active].used = false;
    int finished = s_active;
    s_active = -1;
    s_eng = ENG_IDLE;
    (void)finished;

    if (s_on_tx_done) {
        s_on_tx_done(handle, state);
    }
}

/* Arm a CAD scan for the active slot; advance to CAD_WAIT (or back off on a
 * radio error so the engine never wedges). */
static void engine_start_cad(void) {
    mesh_err_t rc = phy_start_cad();
    if (rc != MESH_OK) {
        /* Could not even start a scan: treat as a busy attempt and back off. */
        s_tx[s_active].cad_attempts++;
        if (s_tx[s_active].cad_attempts >= CAD_MAX_ATTEMPTS) {
            complete_active(LINK_TX_BUSY_FAIL);
        } else {
            s_cad_backoff_at = millis() + rng_range(CAD_BACKOFF_MIN_MS, CAD_BACKOFF_MAX_MS);
            s_eng = ENG_CAD_BACKOFF;
        }
        return;
    }
    s_cad_start_ms = millis();
    s_eng = ENG_CAD_WAIT;
}

/* Channel was busy (or CAD errored): count the attempt and back off or give up. */
static void engine_cad_busy(void) {
    s_tx[s_active].cad_attempts++;
    if (s_tx[s_active].cad_attempts >= CAD_MAX_ATTEMPTS) {
        s_last_err = MESH_ERR_CHANNEL_BUSY;
        complete_active(LINK_TX_BUSY_FAIL);
        return;
    }
    s_cad_backoff_at = millis() + rng_range(CAD_BACKOFF_MIN_MS, CAD_BACKOFF_MAX_MS);
    s_eng = ENG_CAD_BACKOFF;
}

/* Channel was free: hand the frame to the PHY. */
static void engine_transmit(void) {
    tx_slot_t *slot = &s_tx[s_active];
    uint16_t n = build_data_frame(&slot->req, slot->retx > 0, s_tx_wire);
    if (n == 0) {
        s_last_err = MESH_ERR_FRAME_TOO_LONG;
        complete_active(LINK_TX_DROPPED);
        return;
    }

    mesh_err_t rc = phy_start_transmit(s_tx_wire, n);
    if (rc == MESH_OK) {
        s_eng = ENG_TX_WAIT;
        return;
    }
    if (rc == MESH_ERR_DUTY_EXCEEDED || rc == MESH_ERR_TX_BUSY) {
        /* Defer (never silently drop, per DESIGN §6.6): release the engine and
         * retry this same slot on a later poll once budget/radio frees up. The
         * CAD result is now stale, so re-CAD on the next attempt. */
        s_eng = ENG_IDLE;
        return;
    }
    /* Hard radio error: report and drop this send. */
    s_last_err = MESH_ERR_RADIO;
    complete_active(LINK_TX_DROPPED);
}

/* Unicast send timed out without an ACK: retransmit or give up. */
static void engine_ack_timeout(void) {
    tx_slot_t *slot = &s_tx[s_active];
    if (slot->retx >= MAX_RETX) {
        s_last_err = MESH_ERR_RETX_EXHAUSTED;
        complete_active(LINK_TX_NO_ACK);
        return;
    }
    slot->retx++;
    slot->cad_attempts = 0;
    s_stats.tx_retx++;

    /* Exponential backoff with cap, plus random jitter to de-sync collisions. */
    uint8_t shift = slot->retx;
    if (shift > RETX_BACKOFF_EXP_CAP) shift = RETX_BACKOFF_EXP_CAP;
    uint32_t backoff = (uint32_t)RETX_BACKOFF_BASE_MS << shift;
    backoff += rng_range(0, RETX_JITTER_MS);

    s_retx_at = millis() + backoff;
    s_eng = ENG_RETX_BACKOFF;
}

/* Try to inject a queued ACK by preempting an idle/waiting engine. Returns true
 * if an ACK transmit was started (engine is now ENG_ACK_TX_WAIT). */
static bool engine_try_send_ack(void) {
    int idx = ack_first_pending();
    if (idx < 0) return false;
    if (!radio_free_for_ack()) return false;

    uint16_t n = build_ack_frame(s_ackq[idx].ack_src, s_ackq[idx].ack_seq,
                                 s_ackq[idx].status, s_ack_wire);
    if (n == 0) {
        s_ackq[idx].used = false;   /* unbuildable ACK: drop it                  */
        return false;
    }

    mesh_err_t rc = phy_start_transmit(s_ack_wire, n);
    if (rc != MESH_OK) {
        /* Duty/radio busy: leave the ACK queued and try again next poll. */
        return false;
    }
    s_ack_active = idx;
    s_eng_resume = s_eng;           /* return to whatever we were doing          */
    s_eng = ENG_ACK_TX_WAIT;
    return true;
}

/* ============================================================================
 * TX SERVICE  (drive the engine one step per poll)
 * ==========================================================================*/
static void service_tx(void) {
    /* --- Finish an in-flight ACK transmit first ---------------------------- */
    if (s_eng == ENG_ACK_TX_WAIT) {
        if (phy_tx_done()) {
            if (s_ack_active >= 0) {
                s_ackq[s_ack_active].used = false;
                s_ack_active = -1;
                s_stats.acks_sent++;
            }
            s_eng = s_eng_resume;   /* resume the preempted state                */
        }
        return;                     /* radio busy until TxDone; do nothing else  */
    }

    /* --- Opportunistically inject a pending ACK --------------------------- *
     * Only while the engine is idle or in a non-radio-using waiting state, so
     * we never clobber an active CAD or data transmit. ACKs bypass CAD. */
    if (s_eng == ENG_IDLE     || s_eng == ENG_WAIT_ACK ||
        s_eng == ENG_RETX_BACKOFF || s_eng == ENG_CAD_BACKOFF) {
        if (engine_try_send_ack()) {
            return;
        }
    }

    /* --- Main engine step -------------------------------------------------- */
    switch (s_eng) {
        case ENG_IDLE: {
            int slot = pick_ready_slot();
            if (slot < 0) return;                 /* nothing to send            */
            /* Duty pre-check: if this frame won't fit the budget, defer it
             * (leave it queued) and try again later -- never drop for duty. */
            if (!phy_duty_can_tx(req_frame_len(&s_tx[slot].req))) {
                return;
            }
            s_active = slot;
            s_tx[slot].cad_attempts = 0;
            engine_start_cad();
            break;
        }

        case ENG_CAD_WAIT: {
            phy_cad_result_t r = phy_cad_result();
            if (r == PHY_CAD_PENDING) {
                if (reached(s_cad_start_ms + CAD_WAIT_TIMEOUT_MS)) {
                    engine_cad_busy();            /* scan stuck -> back off      */
                }
                return;
            }
            if (r == PHY_CAD_FREE) {
                engine_transmit();
            } else {
                engine_cad_busy();                /* BUSY or ERROR               */
            }
            break;
        }

        case ENG_CAD_BACKOFF:
            if (reached(s_cad_backoff_at)) {
                engine_start_cad();
            }
            break;

        case ENG_TX_WAIT:
            if (phy_tx_done()) {
                tx_slot_t *slot = &s_tx[s_active];
                bool unicast_ack = !is_broadcast_req(&slot->req) && slot->req.require_ack;
                if (!unicast_ack) {
                    complete_active(LINK_TX_SENT);   /* broadcast / no-ACK done   */
                } else {
                    slot->acked = false;
                    s_ack_deadline = millis() + ACK_TIMEOUT_MS;
                    s_eng = ENG_WAIT_ACK;
                }
            }
            break;

        case ENG_WAIT_ACK:
            if (s_tx[s_active].acked) {
                complete_active(LINK_TX_SENT);
            } else if (reached(s_ack_deadline)) {
                engine_ack_timeout();
            }
            break;

        case ENG_RETX_BACKOFF:
            if (reached(s_retx_at)) {
                engine_start_cad();               /* re-arbitrate before retx    */
            }
            break;

        case ENG_ACK_TX_WAIT:
            /* handled above */
            break;
    }
}

/* ============================================================================
 * RX SERVICE  (deframe, validate, classify, dispatch)
 * ==========================================================================*/

/* Match an inbound ACK against the in-flight unicast and mark it delivered. */
static void handle_incoming_ack(const mesh_rx_frame_t *f) {
    if (f->hdr.payload_len < ACK_PL_LEN) return;          /* malformed ACK      */
    node_id_t ack_src = (node_id_t)((uint16_t)f->payload[ACK_PL_OFF_SRC] |
                                    ((uint16_t)f->payload[ACK_PL_OFF_SRC + 1] << 8));
    seq_t     ack_seq = (seq_t)((uint16_t)f->payload[ACK_PL_OFF_SEQ] |
                                ((uint16_t)f->payload[ACK_PL_OFF_SEQ + 1] << 8));

    if (s_eng == ENG_WAIT_ACK && s_active >= 0) {
        const link_tx_req_t *r = &s_tx[s_active].req;
        if (r->src == ack_src && r->seq == ack_seq) {
            s_tx[s_active].acked = true;
        }
    }
    /* ACKs are never deduped, delivered up, or themselves acknowledged. */
}

static void service_rx(void) {
    if (!phy_frame_available()) return;

    uint16_t          len = 0;
    phy_link_metric_t m   = { 0, 0, 0 };
    if (phy_read_frame(s_rx_raw, sizeof(s_rx_raw), &len, &m) != MESH_OK) {
        return;
    }

    mesh_rx_frame_t f;
    mesh_err_t rc = mesh_frame_deserialize(s_rx_raw, len,
                                           m.rssi_dbm, m.snr_db, millis(),
                                           ADDR_BROADCAST, &f);
    if (rc != MESH_OK) {
        if (rc == MESH_ERR_CRC) s_stats.rx_crc_drop++;
        else                    s_stats.rx_bad_hdr++;   /* magic/version/runt/len */
        return;
    }

    /* Link addressing filter: accept only frames for us this hop or broadcasts.
     * Overheard unicasts destined for another next-hop are dropped silently. */
    if (f.hdr.next_hop != ADDR_BROADCAST && f.hdr.next_hop != s_self) {
        return;
    }

    /* ACK frames: classify (fast path), match, and stop -- no dup/deliver. */
    if (f.hdr.pkt_type == (uint8_t)PKT_ACK || (f.hdr.flags & FLAG_IS_ACK)) {
        handle_incoming_ack(&f);
        return;
    }

    /* If this is a unicast to us requesting an ACK, schedule one regardless of
     * duplication -- a duplicate means our earlier ACK was lost and the sender
     * is still retransmitting; it must be re-ACKed to stop. */
    if (f.hdr.next_hop == s_self && (f.hdr.flags & FLAG_ACK_REQ)) {
        ack_enqueue(f.hdr.src, f.hdr.seq, MESH_ACK_OK);
    }

    /* Link-level duplicate suppression (distinct from L3's seen-set). */
    if (dup_seen(f.hdr.src, f.hdr.seq)) {
        s_stats.rx_dup_drop++;
        return;
    }
    dup_insert(f.hdr.src, f.hdr.seq);

    /* Deliver to L3: callback if registered, else into the polled RX FIFO. */
    s_stats.rx_ok++;
    if (s_on_rx) {
        s_on_rx(&f);
    } else {
        rxfifo_push(&f);
    }
}

/* ============================================================================
 * PUBLIC API
 * ==========================================================================*/
mesh_err_t link_begin(node_id_t self,
                      const phy_config_t *phy_cfg,
                      const link_callbacks_t *cb) {
    s_self       = self;
    s_on_rx      = cb ? cb->on_rx      : NULL;
    s_on_tx_done = cb ? cb->on_tx_done : NULL;

    /* Reset all tables/engine state. */
    memset(s_tx,    0, sizeof(s_tx));
    memset(s_ackq,  0, sizeof(s_ackq));
    memset(s_dup,   0, sizeof(s_dup));
    memset(&s_stats, 0, sizeof(s_stats));
    s_active     = -1;
    s_ack_active = -1;
    s_eng        = ENG_IDLE;
    s_eng_resume = ENG_IDLE;
    s_rx_head = s_rx_tail = s_rx_count = 0;
    s_last_err = MESH_OK;

    /* Seed the backoff RNG from time + identity so co-powered nodes differ. */
    s_rng = (uint32_t)millis() ^ ((uint32_t)self << 16) ^ 0x9E3779B9UL;
    if (s_rng == 0) s_rng = 0xA5A5A5A5UL;

    /* Bring up the PHY and start listening. */
    mesh_err_t rc = phy_begin(phy_cfg);
    if (rc != MESH_OK) {
        return rc;
    }
    phy_set_auto_rx(true);          /* always return to RX after TX/RX/CAD       */
    rc = phy_start_receive();
    if (rc != MESH_OK) {
        return rc;
    }

    s_inited = true;
    return MESH_OK;
}

node_id_t link_self_id(void) {
    return s_self;
}

mesh_err_t link_poll(void) {
    if (!s_inited) return MESH_ERR_NOT_INIT;
    phy_poll();        /* 1. finalize radio TX/RX/CAD                            */
    service_rx();      /* 2. deframe / validate / dispatch any received frame    */
    service_tx();      /* 3. advance the transmit engine                        */
    return MESH_OK;
}

link_tx_handle_t link_send(const link_tx_req_t *req) {
    if (!s_inited) { s_last_err = MESH_ERR_NOT_INIT;   return LINK_TX_HANDLE_INVALID; }
    if (req == NULL) { s_last_err = MESH_ERR_INVALID_ARG; return LINK_TX_HANDLE_INVALID; }
    if (req->payload_len > MAX_PAYLOAD) {
        s_last_err = MESH_ERR_FRAME_TOO_LONG;
        return LINK_TX_HANDLE_INVALID;
    }

    /* Find a free queue slot. */
    int slot = -1;
    for (uint8_t i = 0; i < LINK_TX_QUEUE_SIZE; ++i) {
        if (!s_tx[i].used) { slot = (int)i; break; }
    }
    if (slot < 0) {
        s_last_err = MESH_ERR_QUEUE_FULL;
        return LINK_TX_HANDLE_INVALID;
    }

    s_tx[slot].used         = true;
    s_tx[slot].acked        = false;
    s_tx[slot].retx         = 0;
    s_tx[slot].cad_attempts = 0;
    s_tx[slot].req          = *req;                /* copy; no retained pointers */
    /* Broadcasts are never ACKed regardless of what L3 asked for. */
    if (is_broadcast_req(&s_tx[slot].req)) {
        s_tx[slot].req.require_ack = false;
    }

    s_last_err = MESH_OK;
    return (link_tx_handle_t)slot;
}

uint8_t link_tx_free_slots(void) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < LINK_TX_QUEUE_SIZE; ++i) {
        if (!s_tx[i].used) n++;
    }
    return n;
}

mesh_err_t link_tx_cancel(link_tx_handle_t handle) {
    if (handle >= LINK_TX_QUEUE_SIZE) return MESH_ERR_INVALID_ARG;
    if (!s_tx[handle].used)           return MESH_ERR_INVALID_ARG;
    if ((int)handle == s_active)      return MESH_ERR_INVALID_ARG; /* in flight  */
    s_tx[handle].used = false;
    return MESH_OK;
}

mesh_err_t link_recv(mesh_rx_frame_t *out) {
    if (out == NULL) return MESH_ERR_INVALID_ARG;
    if (s_rx_count == 0) return MESH_ERR_NO_RX;
    *out = s_rxfifo[s_rx_head];
    s_rx_head = (uint8_t)((s_rx_head + 1u) % LINK_RX_FIFO_SIZE);
    s_rx_count--;
    return MESH_OK;
}

uint8_t link_rx_pending(void) {
    return s_rx_count;
}

mesh_err_t link_sleep(void) {
    phy_set_auto_rx(false);
    return phy_sleep();
}

mesh_err_t link_wake(void) {
    phy_set_auto_rx(true);
    mesh_err_t rc = phy_standby();
    if (rc != MESH_OK) return rc;
    return phy_start_receive();
}

uint32_t link_duty_remaining_ms(void) {
    return phy_duty_remaining_ms();
}

link_stats_t link_get_stats(void) {
    return s_stats;
}

mesh_err_t link_last_err(void) {
    return s_last_err;
}
