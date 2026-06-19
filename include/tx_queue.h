/* =============================================================================
 * tx_queue.h  --  L2 store-and-forward ring buffer for serialized frames.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * This queue holds fully-serialized, ready-to-transmit frames waiting for:
 *   - CSMA/CAD clearance (channel busy)
 *   - ACK window (in-flight unicast)
 *   - Duty-cycle budget (rate limiting)
 *
 * Distinct from L3's queue_entry_t[] (which holds pre-serialization DATA
 * frames waiting for a route). L3 buffers by route; L2 buffers by channel.
 *
 * Drop policy: if the queue is full and a new frame arrives,
 *   1. Evict the lowest-priority Q_PENDING slot (never Q_AWAIT_ACK).
 *   2. Priority order: ACK/RREP > DATA > BEACON.
 *   3. Among equal-priority slots, evict the oldest.
 * ===========================================================================*/
#ifndef TX_QUEUE_H
#define TX_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"
#include "mesh_types.h"

/* State of one TX queue slot. */
typedef MESH_ENUM8(q_slot_state_e) {
    Q_FREE      = 0,   /* slot available                                     */
    Q_PENDING,         /* waiting for CSMA clearance                         */
    Q_AWAIT_CAD,       /* CAD in progress; radio busy                        */
    Q_AWAIT_ACK,       /* frame sent; waiting per-hop ACK                   */
    Q_DONE             /* tx done (success or fail); ready to reclaim        */
} q_slot_state_t;

/* Priority levels for drop policy. Higher number = higher priority.        */
#define Q_PRIO_BEACON   1u
#define Q_PRIO_DATA     2u
#define Q_PRIO_CONTROL  3u   /* RREQ, RERR                                  */
#define Q_PRIO_RREP     4u   /* RREP is time-critical (route setup window)  */
#define Q_PRIO_ACK      5u   /* ACK is most time-critical; never evict      */

/* One slot in the L2 TX queue. */
typedef struct {
    uint8_t          frame[MAX_FRAME_LEN]; /* serialized on-air frame        */
    uint8_t          frame_len;            /* valid bytes in frame[]          */
    node_id_t        next_hop;             /* unicast target or ADDR_BROADCAST*/
    q_slot_state_t   state;
    uint8_t          retries;              /* retransmit attempts so far      */
    uint8_t          priority;             /* Q_PRIO_* for eviction policy    */
    uint32_t         enqueue_ms;           /* millis() when enqueued          */
    uint32_t         next_attempt_ms;      /* earliest next TX / backoff      */
} q_slot_t;

/* The queue itself. head/tail/count are indices into slots[].              */
typedef struct {
    q_slot_t slots[MAX_QUEUE];
    uint8_t  count;                        /* occupied slots                  */
} tx_queue_t;

/* ── Operations ──────────────────────────────────────────────────────────── */

/* Zero-initialize the queue. Call once at startup. */
bool txq_init(tx_queue_t *q);

/* Enqueue a serialized frame. Returns true on success.
 * If full, applies drop policy (evicts lowest-priority Q_PENDING slot).
 * Returns false (without enqueuing) only if priority <= all occupied slots
 * AND all occupied slots are Q_AWAIT_ACK (i.e. nothing can be evicted).  */
bool txq_enqueue(tx_queue_t *q,
                 const uint8_t *frame, uint8_t frame_len,
                 node_id_t next_hop, uint8_t priority, uint32_t now_ms);

/* Return a pointer to the next slot in Q_PENDING state whose next_attempt_ms
 * <= now (i.e. ready to transmit). NULL if none ready. The returned pointer
 * is live inside the queue — do not enqueue while using it. */
q_slot_t *txq_peek_ready(tx_queue_t *q, uint32_t now_ms);

/* Mark a slot as Q_AWAIT_ACK (just transmitted, awaiting per-hop ACK).    */
void txq_mark_inflight(tx_queue_t *q, q_slot_t *s);

/* Mark a slot as done and free it. Call on ACK receipt, broadcast TX done,
 * or permanent failure (retries exhausted, duty exceeded). */
void txq_mark_done(tx_queue_t *q, q_slot_t *s);

/* Schedule a retransmit: update s->next_attempt_ms and bump s->retries.
 * s->state stays Q_PENDING (backed out from AWAIT_ACK on timeout).       */
void txq_schedule_retx(tx_queue_t *q, q_slot_t *s, uint32_t backoff_ms);

/* Age out entries that have been in the queue > QUEUE_ENTRY_TTL_MS.
 * Frees them and returns the count evicted. */
uint8_t txq_age(tx_queue_t *q, uint32_t now_ms);

/* Slot count / full check. */
uint8_t txq_count(const tx_queue_t *q);
bool    txq_full(const tx_queue_t *q);

#endif /* TX_QUEUE_H */
