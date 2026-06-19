/* =====================================================================
 * mesh_l3.h  --  L3 NETWORK LAYER public interface (the "mesh brain")
 * ---------------------------------------------------------------------
 * AODV-style on-demand routing + proactive BEACON neighbor discovery for
 * a lossy, low-bandwidth, multi-hop LoRa mesh on RP2040 / SX1278 /
 * RadioLib. Non-blocking, millis()-driven, fixed memory, no FPU math.
 *
 * This header is the CONTRACT between:
 *   - L4 (application)  -> calls mesh_l3_send() / receives via callback
 *   - L2 (data link)    -> calls mesh_l3_on_frame() on each good RX frame,
 *                          and mesh_l3_on_link_event() for ACK/loss events;
 *                          L3 calls back into L2 via the tx hook to send.
 *   - main loop / core  -> calls mesh_l3_tick() every iteration.
 *
 * DESIGN ONLY -- declarations + extensive semantics. No .cpp this phase.
 *
 * ============================ CONTENTS ===============================
 *   1. Glue/HAL function-pointer types (how L3 talks DOWN to L2 and UP
 *      to L4 without a hard build dependency).
 *   2. Result/status enums.
 *   3. Public lifecycle API: init / send / on_frame / on_link_event /
 *      tick / introspection.
 *   4. PROTOCOL MECHANICS (the design spec, in comments): BEACON, RREQ,
 *      RREP, RERR, DATA forwarding, sequence-number loop prevention,
 *      timer table, and the three guarantees (no storms / no loops /
 *      no stale routes).
 * ===================================================================== */
#ifndef MESH_L3_H
#define MESH_L3_H

#include "mesh_l3_types.h"
#include "mesh_l3_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 1. GLUE / HAL -- how L3 plugs into the neighbouring layers
 * ---------------------------------------------------------------------
 * L3 never touches RadioLib or the frame codec directly. It is handed:
 *   - a TX hook to enqueue a finished L3 frame at L2 (which does CSMA/CAD,
 *     per-hop ACK & retransmit, then PHY TX),
 *   - a DELIVER hook to pass a fully-arrived DATA payload up to L4,
 *   - a millis() source and an rng source (so the design is testable on
 *     host without Arduino).
 * These are provided once at init via mesh_l3_hooks_t.
 * ===================================================================== */

/* A decoded inbound frame as L3 sees it (L2 has already validated magic,
 * version, length and CRC, and stripped the on-air framing). L3 reads the
 * routing header fields and, for control packets, the typed payload. */
typedef struct {
    node_id_t      src;        /* header SrcID  (origin for DATA/RREQ)     */
    node_id_t      dst;        /* header DstID                             */
    node_id_t      nextHop;    /* header NextHop (== self if we were target)*/
    node_id_t      linkFrom;   /* L2 one-hop sender (who keyed this frame
                                *  on the radio; == previous hop). Distinct
                                *  from src for forwarded packets.          */
    uint8_t        hopCount;
    uint8_t        ttl;
    seq_t          seqNum;     /* header SeqNum                            */
    mesh_pkttype_t type;
    int16_t        rssi;       /* RadioLib getRSSI() of THIS reception      */
    int8_t         snr;        /* RadioLib getSNR()  of THIS reception      */
    const uint8_t *payload;    /* points into L2's RX buffer (do not keep)  */
    uint8_t        payloadLen;
} l3_rx_frame_t;

/* An outbound L3 frame L3 asks L2 to transmit. L2 stamps on-air framing,
 * runs CSMA/CAD + per-hop ACK/retransmit toward `nextHop`, then TX.
 * `wantsAck` lets L3 request a per-hop link ACK (used for unicast DATA /
 * RREP; broadcasts like BEACON/RREQ are never link-ACKed).               */
typedef struct {
    node_id_t      src;
    node_id_t      dst;
    node_id_t      nextHop;
    uint8_t        hopCount;
    uint8_t        ttl;
    seq_t          seqNum;
    mesh_pkttype_t type;
    bool           wantsAck;   /* request L2 per-hop ACK + retransmit       */
    const uint8_t *payload;
    uint8_t        payloadLen;
} l3_tx_frame_t;

/* L2 link events fed back into L3 -- the trigger for RERR / link-break. */
typedef MESH_ENUM8(l3_link_event_e) {
    L3_LINK_TX_DONE = 0,   /* frame to nextHop acked / sent OK             */
    L3_LINK_TX_FAILED,     /* per-hop ACK exhausted (MAX_RETX) => break    */
    L3_LINK_NEIGHBOR_LOST  /* L2/keepalive declared a neighbor dead        */
} l3_link_event_t;

/* Hooks supplied by the integrator at init. */
typedef struct {
    /* DOWN to L2: returns true if the frame was accepted for transmit. */
    bool      (*l2_send)(const l3_tx_frame_t *f);
    /* UP to L4: a DATA payload addressed to us has fully arrived. */
    void      (*l4_deliver)(node_id_t fromOrigin, seq_t seq,
                            const uint8_t *payload, uint8_t len);
    /* UP to L4: a locally-originated send finished or failed. */
    void      (*l4_send_result)(node_id_t dest, seq_t seq, bool ok);
    /* time + randomness (injectable for host tests). */
    uint32_t  (*now_ms)(void);
    uint32_t  (*rand_u32)(void);
} mesh_l3_hooks_t;

/* =====================================================================
 * 2. RESULT CODES
 * ===================================================================== */
typedef MESH_ENUM8S(l3_result_e) {
    L3_OK = 0,
    L3_ERR_QUEUE_FULL    = -1,  /* store-and-forward queue saturated        */
    L3_ERR_NO_ROUTE      = -2,  /* (transient) buffered, RREQ started       */
    L3_ERR_PAYLOAD_TOO_BIG = -3,
    L3_ERR_BAD_DEST      = -4,  /* dest == ADDR_INVALID                     */
    L3_ERR_NOT_INIT      = -5
} l3_result_t;

/* =====================================================================
 * 3. PUBLIC LIFECYCLE API
 * ===================================================================== */

/* Initialise L3 with this node's id and the layer hooks. Clears all
 * tables (static storage), seeds seq/bcastId, schedules first beacon.
 * Must be called once before any other L3 call. Returns L3_OK or error. */
l3_result_t mesh_l3_init(node_id_t selfId, const mesh_l3_hooks_t *hooks);

/* ---- L4 -> L3 : originate an application DATA packet ----------------
 * Sends `len` payload bytes to `dest`.
 *  - dest == self            : delivered locally (loopback) -> L3_OK.
 *  - route VALID             : frame stamped + handed to L2 -> L3_OK.
 *  - route unknown/invalid   : payload buffered in the S&F queue and an
 *                              RREQ flood is started -> L3_ERR_NO_ROUTE
 *                              (NOT a hard failure; l4_send_result fires
 *                              later with ok=true once delivered to next
 *                              hop, or ok=false if discovery ultimately
 *                              fails / TTL/queue limits hit).
 *  - queue full              : L3_ERR_QUEUE_FULL (caller may retry later).
 * The returned seq is the packet's originator sequence number (also
 * reported back through l4_send_result). */
l3_result_t mesh_l3_send(node_id_t dest, const uint8_t *payload, uint8_t len,
                         seq_t *out_seq);

/* ---- L2 -> L3 : a validated inbound frame -------------------------- */
/* Called by L2 for every frame that passes magic/version/length/CRC.
 * L3 updates neighbor state from (linkFrom,rssi,snr), then dispatches by
 * type: DATA->deliver-or-forward; BEACON->neighbor table; RREQ/RREP/RERR
 * ->routing machinery. ACKs are consumed by L2 and never reach here. */
void mesh_l3_on_frame(const l3_rx_frame_t *f);

/* ---- L2 -> L3 : per-hop link outcome / neighbor-loss notification --- */
/* `nextHop` is the neighbor the event concerns; `ctx` echoes the seqNum
 * (or 0) of the frame that triggered it so L3 can correlate to a queue
 * entry. TX_FAILED / NEIGHBOR_LOST trigger route invalidation + RERR. */
void mesh_l3_on_link_event(l3_link_event_t ev, node_id_t nextHop, seq_t ctx);

/* ---- main loop -> L3 : periodic maintenance ------------------------ */
/* Drives every L3 timer in one non-blocking pass:
 *   - emit a BEACON when due (interval +/- jitter);
 *   - age out neighbors (NEIGHBOR_TIMEOUT) -> may invalidate routes/RERR;
 *   - expire stale routes (ROUTE_EXPIRY);
 *   - age out seen-set entries (SEEN_ENTRY_TTL);
 *   - retry/expire pending RREQs (ring search widen, then give up);
 *   - flush ready store-and-forward entries to L2 when a route appears;
 *     drop queue entries past QUEUE_ENTRY_TTL (-> l4_send_result ok=false).
 * Returns the suggested ms until the next useful tick (so a caller may
 * sleep/duty-cycle until then); 0 means "work pending, call again soon". */
uint32_t mesh_l3_tick(void);

/* ---- introspection (read-only; for diagnostics / second core) ------ */
const mesh_l3_state_t *mesh_l3_state(void);
bool  mesh_l3_lookup_route(node_id_t dest, node_id_t *outNextHop,
                           uint8_t *outHopCount, uint16_t *outMetric);
uint8_t mesh_l3_neighbor_count(void);

/* ---- integer link-cost mapping (NO FLOAT; pure-fn, exposed for test) -
 * Maps a received (rssi dBm, snr dB) to a 1..LINK_COST_MAX link cost.
 * Lower = better. A reference mapping (justified below) is implemented in
 * mesh_l3.cpp; declared here so unit tests can pin its behaviour. */
link_cost_t mesh_link_cost(int16_t rssi, int8_t snr);

#ifdef __cplusplus
} /* extern "C" */
#endif

/* =====================================================================
 * 4. PROTOCOL MECHANICS  (DESIGN SPECIFICATION -- normative comments)
 * =====================================================================
 *
 * Conventions:
 *   - "origin"  = node that first created a packet (header Src, immutable).
 *   - "self"    = mesh_l3_state.selfId.
 *   - seq compare uses RFC-1982 serial arithmetic:
 *         newer(a,b) := (int16_t)(a - b) > 0
 *     valid because the in-flight window << 2^15.
 *
 * --------------------------------------------------------------------
 * 4.1 BEACON  (proactive one-hop neighbor discovery)
 * --------------------------------------------------------------------
 * EMIT: every BEACON_INTERVAL_MS +/- random(BEACON_JITTER_MS). Jitter
 *   de-synchronises neighbors so beacons don't collide each period.
 * CARRIES: beacon_hdr_t{ nbrCount, flags } + up to BEACON_MAX_NBRS
 *   beacon_nbr_t{ nodeId, linkCost } drawn from our freshest neighbors.
 *   Header SeqNum = our mySeq snapshot. TTL=1, Dst=NextHop=BROADCAST.
 * NEVER FLOODED: TTL=1, recipients do not re-broadcast (HopCount stays 0).
 * ON RECEIPT (mesh_l3_on_frame, type==BEACON):
 *   1. Upsert neighbor[src]: lastHeard=now, rssi/snr from THIS reception,
 *      linkCost = mesh_link_cost(rssi,snr), missed=0, flags|=VALID.
 *      Copy BEACON_FLAG_ROUTER -> NEIGHBOR_FLAG_ROUTER.
 *   2. Scan the advertised nbr list: if WE (selfId) appear in it, set
 *      NEIGHBOR_FLAG_SYMMETRIC (the link is confirmed bidirectional --
 *      important on lossy LoRa where one direction may be much weaker).
 *      Only SYMMETRIC neighbors are eligible as a route nextHop.
 * LIVENESS / TIMEOUT (in tick):
 *   - each elapsed BEACON_INTERVAL with no frame from a neighbor ++missed.
 *   - lastHeard older than NEIGHBOR_TIMEOUT_MS (~3 intervals) => evict the
 *     neighbor; any route whose nextHop == that neighbor is invalidated
 *     and an RERR is generated for its precursors (see 4.4).
 *
 * --------------------------------------------------------------------
 * 4.2 RREQ  (on-demand route discovery -- flood w/ TTL + (origin,bcastId)
 *            dedup + reverse-route setup + additive metric)
 * --------------------------------------------------------------------
 * ORIGINATE (we need a route to `dest`, none VALID):
 *   - ++mySeq is NOT done here; we BUMP our OWN seq is unrelated. We take
 *     a fresh bcastId = myBcastId++ and record a pending_rreq_t{dest,
 *     bcastId, ttl=RREQ_START_TTL, retries=0, sentAt=now}.
 *   - Build rreq_payload_t{ origin=self, dest, bcastId, originSeq=mySeq,
 *     destSeq=last-known (destSeqValid as known), metric=0 }.
 *   - Header: Src=self, Dst=BROADCAST, NextHop=BROADCAST, HopCount=0,
 *     TTL=ttl, SeqNum=bcastId.  Broadcast via L2 (no link-ACK).
 *   - TTL RING SEARCH: first attempt uses a small TTL (RREQ_START_TTL) to
 *     find nearby destinations cheaply; on timeout the TTL is widened by
 *     RREQ_TTL_INCREMENT up to RREQ_MAX_TTL (see timer table 4.7).
 *
 * FORWARD (mesh_l3_on_frame, type==RREQ, not addressed only-to-us):
 *   1. DEDUP: if (origin,bcastId) is in the seen-set -> DROP (this is the
 *      core broadcast-storm suppressor). Otherwise INSERT it (firstSeen=
 *      now) and continue.
 *   2. REVERSE ROUTE: install/refresh a route toward `origin` with
 *      nextHop = linkFrom (the one-hop sender), hopCount = rxHopCount+1,
 *      metric = rxMetric + mesh_link_cost(thisRx), destSeq = originSeq,
 *      expiry = now + REVERSE_ROUTE_LIFE_MS. Apply the SAME freshness rule
 *      as any route update (4.5) so we keep the best reverse path. This is
 *      what lets the future RREP travel back.
 *   3. ANSWER-OR-RELAY:
 *        a. if self == dest  -> generate an RREP (4.3) back to origin.
 *        b. else if we hold a VALID route to dest whose destSeqValid and
 *           destSeq is >= the RREQ's destSeq (i.e. our route is at least as
 *           fresh as requested) -> generate an INTERMEDIATE RREP on dest's
 *           behalf (and, if RREQ_FLAG_GRATUITOUS, also a gratuitous RREP to
 *           dest so the forward path is primed).
 *        c. else RELAY: --TTL; if TTL==0 DROP (flood radius bound). Else
 *           ++HopCount, metric += our incoming link cost, re-broadcast the
 *           RREQ unchanged except HopCount/TTL/metric. NextHop stays
 *           BROADCAST, Src/origin/bcastId/SeqNum unchanged.
 *
 * --------------------------------------------------------------------
 * 4.3 RREP  (unicast back along reverse route -- forward-route setup)
 * --------------------------------------------------------------------
 * GENERATE (by dest, or by an intermediate with a fresh-enough route):
 *   - If self==dest: BUMP our own mySeq if needed so destSeq is monotone
 *     and >= any destSeq the RREQ already knew (AODV rule: dest sets its
 *     seq to max(ownSeq, rreq.destSeq)). Build rrep_payload_t{ dest=self,
 *     origin=rreq.origin, destSeq=mySeq, hopCount=0, metric=0,
 *     lifetimeMs=ACTIVE_ROUTE_LIFE_MS }.
 *   - If intermediate: dest=rreq.dest, destSeq=our route's destSeq,
 *     hopCount=our route's hopCount, metric=our route's metric.
 *   - Unicast toward origin: look up the reverse route (installed in 4.2);
 *     Header Src=self? NO -- Src/Dst carry the END POINTS: Src=dest-of-
 *     route is not needed; we set Dst = origin (final recipient), NextHop =
 *     reverseRoute.nextHop, TTL = reverseRoute.hopCount + margin, SeqNum =
 *     destSeq. wantsAck=true (unicast => L2 per-hop ACK + retransmit).
 *
 * FORWARD (mesh_l3_on_frame, type==RREP, NextHop==self, Dst!=self):
 *   1. FORWARD ROUTE: install/refresh a route toward `dest` (rrep.dest)
 *      with nextHop = linkFrom, hopCount = rrep.hopCount + 1,
 *      metric = rrep.metric + thisLinkCost, destSeq = rrep.destSeq,
 *      state=VALID, expiry = now + rrep.lifetimeMs. Apply freshness rule
 *      (4.5): only overwrite an existing route if STRICTLY newer destSeq,
 *      or equal destSeq AND strictly-better metric. Record the precursor
 *      (the neighbor we will forward toward origin) on this route.
 *   2. RELAY toward origin: look up reverse route to origin; --TTL (drop at
 *      0), ++hopCount, metric += thisLinkCost; rewrite NextHop = reverse
 *      nextHop; unicast on (wantsAck=true).
 *
 * ARRIVAL AT ORIGIN (Dst==self):
 *   - install forward route to dest exactly as in step 1, mark the matching
 *     pending_rreq_t resolved, then FLUSH every queued DATA whose finalDest
 *     == dest (set nextHop, --TTL, hand to L2). l4_send_result fires per
 *     queued packet as L2 confirms the next hop.
 *
 * --------------------------------------------------------------------
 * 4.4 RERR  (link-break detection -> route invalidation -> notify)
 * --------------------------------------------------------------------
 * TRIGGERS (any of):
 *   - mesh_l3_on_link_event(L3_LINK_TX_FAILED, nextHop, ...) -- L2 used up
 *     MAX_RETX per-hop ACKs to `nextHop`.
 *   - mesh_l3_on_link_event(L3_LINK_NEIGHBOR_LOST, nextHop, ...).
 *   - neighbor aged out in tick (4.1).
 *   - received a DATA we must forward but have no VALID route for (we tell
 *     the upstream sender via RERR so it stops using us).
 * ACTION:
 *   1. Find every VALID route whose nextHop == the broken neighbor.
 *   2. For each, set state=INVALID, metric=LINK_COST_INFINITE, and
 *      INCREMENT its destSeq by 1 (advertise that this route is now stale
 *      -- the monotonic bump is the anti-count-to-infinity mechanism).
 *   3. Collect the affected (dest,destSeq) into rerr_hdr_t +
 *      unreachable_dest_t[] (cap RERR_MAX_DESTS; emit multiple RERRs if
 *      more).
 *   4. Send to PRECURSORS: for each distinct precursor of the broken
 *      routes, unicast the RERR (TTL=1, wantsAck=true). If a route has no
 *      recorded precursor, optionally broadcast (TTL=1) as a fallback.
 *   5. (Optional) if ROUTE_FLAG_REPAIRABLE and the break is close to dest,
 *      attempt LOCAL REPAIR first (originate an RREQ for dest with
 *      RREQ_FLAG_REPAIR before emitting RERR); only emit RERR if repair
 *      fails within RREQ_TIMEOUT.
 * ON RECEIPT (type==RERR): for each listed (dest,destSeq): if we hold a
 *   route to dest whose nextHop == the RERR sender (linkFrom) AND our
 *   route's destSeq is OLDER than the advertised destSeq, invalidate it,
 *   adopt the advertised (incremented) destSeq, and re-propagate the RERR
 *   to OUR precursors for that dest. The destSeq freshness gate stops RERR
 *   storms and stale RERRs from killing newer routes.
 *
 * --------------------------------------------------------------------
 * 4.5 DATA FORWARDING (store-and-forward)
 * --------------------------------------------------------------------
 * On mesh_l3_on_frame(type==DATA):
 *   - end-to-end DEDUP: if (src,seqNum) seen recently as a DELIVERED/seen
 *     DATA, drop (covers duplicate retransmissions that slipped through).
 *   - if Dst==self: l4_deliver(src,seqNum,payload,len). Done.
 *   - else FORWARD: look up route to Dst.
 *       * VALID route: --TTL; if TTL==0 DROP + ++statDroppedTtl (loop
 *         bound). Else ++HopCount, NextHop=route.nextHop, refresh route
 *         expiry (active use), wantsAck=true, hand to L2. Record the
 *         upstream (linkFrom) as a precursor on the route (so a future
 *         break RERRs back correctly).
 *       * NO/!VALID route: buffer the frame in the S&F queue
 *         (QSTATE_WAIT_ROUTE) and originate an RREQ for Dst (4.2). If the
 *         queue is full, DROP oldest WAIT_ROUTE entry (or reject) and
 *         ++statDroppedNoRoute. Optionally RERR upstream so it re-routes.
 * On mesh_l3_send (locally originated): same as forward but origin=self,
 *   seq = mySeq-derived per-packet counter, HopCount=0, TTL=DEFAULT_TTL.
 *
 * --------------------------------------------------------------------
 * 4.6 SEQUENCE NUMBERS -> LOOP & COUNT-TO-INFINITY PREVENTION
 * --------------------------------------------------------------------
 * Two independent monotone counters, both 16-bit, both compared with
 * RFC-1982 serial arithmetic (seq_newer):
 *   (i) DESTINATION SEQUENCE NUMBER (destSeq): owned by each destination,
 *       only ever increased by that destination (and bumped by +1 on a
 *       broken route during RERR). The route-update rule is AODV's:
 *           accept new route iff
 *             newer(newDestSeq, curDestSeq)                       // fresher
 *           OR (newDestSeq == curDestSeq AND newMetric < curMetric) // shorter
 *       Because destSeq only moves FORWARD and a higher destSeq always
 *       wins, a forwarding loop would require a node to accept a route
 *       with an older-or-equal seq and worse metric -- which the rule
 *       forbids. This is the core loop-freedom guarantee (AODV theorem).
 *  (ii) BROADCAST ID (bcastId): per-originator flood id, used only in the
 *       (origin,bcastId) seen-set to suppress duplicate RREQ copies. Does
 *       not affect path selection.
 * COUNT-TO-INFINITY: classic distance-vector counts to infinity because
 * nodes believe each other's stale advertisements. Here, a route is only
 * believed if its destSeq is fresher; on a break the detecting node bumps
 * destSeq and floods RERR, so every node's stale copy is superseded by the
 * higher seq rather than slowly re-learned. TTL (DEFAULT_TTL) is the
 * absolute backstop: any packet caught in a transient loop is killed after
 * at most DEFAULT_TTL hops.
 *
 * --------------------------------------------------------------------
 * 4.7 TIMERS / INTERVALS / RETRIES  (defaults; all in mesh_config.h)
 * --------------------------------------------------------------------
 *   BEACON_INTERVAL_MS   60000  +/- BEACON_JITTER_MS 10000  (proactive HELLO)
 *   NEIGHBOR_TIMEOUT_MS  180000  (~3 missed beacons => neighbor dead)
 *   ROUTE_EXPIRY_MS      300000  (unused route expires; refreshed on use)
 *   ACTIVE_ROUTE_LIFE_MS == ROUTE_EXPIRY (lifetime stamped into RREP)
 *   REVERSE_ROUTE_LIFE_MS 6000   (reverse route held awaiting RREP)
 *   SEEN_ENTRY_TTL_MS    30000   (RREQ dedup entry aging; > max traversal)
 *   RREQ_START_TTL       2  -> +RREQ_TTL_INCREMENT 2 -> RREQ_MAX_TTL(=8)
 *   RREQ_TIMEOUT_MS      3000   (wait for RREP, then widen TTL & retry)
 *   RREQ_RETRY           2      (total discovery attempts before giving up)
 *   QUEUE_ENTRY_TTL_MS   8000   (buffered DATA dropped if no route in time)
 *   DEFAULT_TTL          8      (hop limit / hard loop backstop)
 * L2 owns ACK_TIMEOUT_MS(600)/MAX_RETX(3)/RETX_BACKOFF/CAD backoff; L3
 * consumes their *outcome* via mesh_l3_on_link_event.
 *
 * --------------------------------------------------------------------
 * 4.8 THE THREE GUARANTEES (explicit)
 * --------------------------------------------------------------------
 * BROADCAST STORMS PREVENTED BY:
 *   - (origin,bcastId) seen-set: every flooded RREQ is re-broadcast at
 *     most ONCE per node per flood; duplicates dropped (statDupSuppressed).
 *   - TTL on every flooded packet bounds the flood radius/hops.
 *   - TTL RING SEARCH keeps most discoveries local (small initial TTL).
 *   - Beacons are TTL=1 (never flooded) and jittered (no synchronized
 *     period collisions).
 *   - L2 CAD/CSMA + random backoff before each TX spaces re-broadcasts.
 * LOOPS PREVENTED BY:
 *   - Destination-sequence-number monotonicity + the AODV route-accept
 *     rule (newer-seq-or-equal-seq-shorter-metric) => loop-free paths.
 *   - Per-packet TTL hard backstop kills any transient loop in <=TTL hops.
 *   - HopCount carried for tie-break/diagnostics; metric is additive &
 *     isotonic so path comparison is globally consistent.
 * STALE ROUTES PREVENTED BY:
 *   - Soft state: every route has an expiry; unused routes age out
 *     (ROUTE_EXPIRY) and are refreshed on active forwarding.
 *   - RERR with incremented destSeq invalidates broken routes everywhere
 *     and prevents re-learning the dead path (anti count-to-infinity).
 *   - Neighbor liveness (beacon timeout) proactively detects dead next
 *     hops before traffic even tries them; symmetric-link flag stops us
 *     choosing a one-way (asymmetric) neighbor as a forwarding next hop.
 *   - Seen-set entries themselves age out (SEEN_ENTRY_TTL) so the dedup
 *     cache cannot fill permanently or wrongly suppress a legitimately new
 *     flood after wraparound.
 * ===================================================================== */

#endif /* MESH_L3_H */
