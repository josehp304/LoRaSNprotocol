/* =====================================================================
 * mesh_l3_types.h  --  L3 NETWORK LAYER: shared types & fixed-size tables
 * ---------------------------------------------------------------------
 * Routed multi-hop LoRa mesh (AODV-style on-demand routing + proactive
 * neighbor beacons), application-layer over a raw SX1278 LoRa link.
 *
 * Target: RP2040 (Pico W), Arduino-pico, RadioLib. No HW FPU, no dynamic
 * allocation after init. All tables are static fixed-size arrays.
 *
 * THIS HEADER = DATA STRUCTURES ONLY (deliverable item (a)).
 *   - Neighbor table entry
 *   - Routing table entry
 *   - Seen-set / duplicate-cache entry (flood suppression)
 *   - Reverse-route / pending-RREQ bookkeeping
 *   - Store-and-forward queue entry
 *
 * Protocol mechanics live in mesh_l3.h (the public L3 interface) and the
 * on-wire control-payload layouts live in mesh_l3_control.h.
 *
 * DESIGN ONLY: no .cpp implementation in this phase.
 * ===================================================================== */
#ifndef MESH_L3_TYPES_H
#define MESH_L3_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 * Portable "enum with 1-byte underlying type".
 *   - C++11 and C23 support `enum Name : uint8_t {...}` natively.
 *   - Older C (the Arduino build is C++, but a plain-C L2 codec may also
 *     include these headers) gets a normal enum; struct fields that must
 *     be exactly one byte on the wire are serialized field-by-field by the
 *     L2 codec regardless of the in-RAM enum width, so portability is safe.
 * Usage:  typedef MESH_ENUM8(route_state_t) { A, B } route_state_t;       */
#if defined(__cplusplus) && (__cplusplus >= 201103L)
#  define MESH_ENUM8(name) enum name : uint8_t
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ > 201710L)
#  define MESH_ENUM8(name) enum name : uint8_t
#else
#  define MESH_ENUM8(name) enum name
#endif

/* signed 1-byte-underlying enum (for negative result/error codes) */
#if defined(__cplusplus) && (__cplusplus >= 201103L)
#  define MESH_ENUM8S(name) enum name : int8_t
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ > 201710L)
#  define MESH_ENUM8S(name) enum name : int8_t
#else
#  define MESH_ENUM8S(name) enum name
#endif

/* ---------------------------------------------------------------------
 * Dependency on the shared project config (mesh_config.h).
 *
 * mesh_config.h is authored by the config/PHY owner and carries the
 * RECOMMENDED DEFAULT CONSTANTS block (LORA_*, table sizes, addressing,
 * frame layout, L3 timers). To let this L3 header compile and be reviewed
 * STANDALONE before that file lands, every symbol we depend on is given a
 * fallback below, each guarded so the real mesh_config.h always wins.
 *
 * When mesh_config.h exists, simply #include it before this header (or let
 * the include below pull it in) and the fallbacks become inert.
 * ------------------------------------------------------------------- */
#if defined(__has_include)
#  if __has_include("mesh_config.h")
#    include "mesh_config.h"
#  endif
#endif

/* ---- Addressing ---------------------------------------------------- */
#ifndef ADDR_INVALID
#define ADDR_INVALID   0x0000u   /* unassigned / "no route" sentinel    */
#endif
#ifndef ADDR_BROADCAST
#define ADDR_BROADCAST 0xFFFFu   /* link-local broadcast (DstID/NextHop)*/
#endif

#ifndef MESH_NODE_ID_T_DEFINED
#define MESH_NODE_ID_T_DEFINED
typedef uint16_t node_id_t;      /* 65534 usable nodes; 0 & 0xFFFF rsvd */
#endif
#ifndef MESH_SEQ_T_DEFINED
#define MESH_SEQ_T_DEFINED
typedef uint16_t seq_t;          /* per-originator monotone seq, wraps   */
#endif

/* broadcast-id: per-node flood identifier for RREQ dedup. Distinct from
 * the AODV "destination sequence number" (which lives in routing state).
 * 16-bit, wraps mod 2^16, compared with RFC-1982 serial arithmetic.     */
typedef uint16_t bcast_id_t;

/* ---- Fixed table sizes (fallbacks mirror the constants report) ----- */
#ifndef MAX_NEIGHBORS
#define MAX_NEIGHBORS 16
#endif
#ifndef MAX_ROUTES
#define MAX_ROUTES    32
#endif
#ifndef MAX_SEEN
#define MAX_SEEN      64
#endif
#ifndef MAX_QUEUE
#define MAX_QUEUE     8
#endif
#ifndef MAX_PAYLOAD
#define MAX_PAYLOAD   64
#endif
/* number of concurrently-outstanding route discoveries we track */
#ifndef MAX_PENDING_RREQ
#define MAX_PENDING_RREQ 8
#endif

/* ---- L3 timers / limits (fallbacks mirror the constants report) ---- */
#ifndef DEFAULT_TTL
#define DEFAULT_TTL          8
#endif
#ifndef BEACON_INTERVAL_MS
#define BEACON_INTERVAL_MS   60000u
#endif
#ifndef BEACON_JITTER_MS
#define BEACON_JITTER_MS     10000u
#endif
#ifndef NEIGHBOR_TIMEOUT_MS
#define NEIGHBOR_TIMEOUT_MS  180000u   /* ~3 missed beacons => stale     */
#endif
#ifndef ROUTE_EXPIRY_MS
#define ROUTE_EXPIRY_MS      300000u   /* 5 min unused => expire          */
#endif
#ifndef RREQ_RETRY
#define RREQ_RETRY           2
#endif
#ifndef RREQ_TIMEOUT_MS
#define RREQ_TIMEOUT_MS      3000u     /* wait for RREP before retry      */
#endif
#ifndef SEEN_ENTRY_TTL_MS
#define SEEN_ENTRY_TTL_MS    30000u    /* dup-suppression aging           */
#endif

/* ---------------------------------------------------------------------
 * L3-specific tunables NOT in the base config (justified in mesh_l3.h):
 * ------------------------------------------------------------------- */
#ifndef RREQ_BACKOFF_BASE_MS
#define RREQ_BACKOFF_BASE_MS 0u        /* exp backoff base mult (see .h)  */
#endif
/* TTL ring-search: first RREQ goes out with a small TTL and widens on
 * retry, to bound the flood radius for nearby destinations.            */
#ifndef RREQ_START_TTL
#define RREQ_START_TTL       2
#endif
#ifndef RREQ_TTL_INCREMENT
#define RREQ_TTL_INCREMENT   2
#endif
#ifndef RREQ_MAX_TTL
#define RREQ_MAX_TTL         DEFAULT_TTL
#endif
/* lifetime a reverse route (set up by a forwarded RREQ) is held waiting
 * for the matching RREP to come back. Must exceed worst-case round trip.*/
#ifndef REVERSE_ROUTE_LIFE_MS
#define REVERSE_ROUTE_LIFE_MS 6000u
#endif
/* a freshly forward-routed entry from an RREP is at least this fresh    */
#ifndef ACTIVE_ROUTE_LIFE_MS
#define ACTIVE_ROUTE_LIFE_MS  ROUTE_EXPIRY_MS
#endif
/* how long a buffered DATA packet may wait for a route before we drop   */
#ifndef QUEUE_ENTRY_TTL_MS
#define QUEUE_ENTRY_TTL_MS    8000u
#endif

/* =====================================================================
 * (a.1) NEIGHBOR TABLE
 * ---------------------------------------------------------------------
 * One-hop adjacency learned PROACTIVELY from BEACON frames (and
 * opportunistically from any correctly-received L2 frame). Drives the
 * link-cost metric fed into route selection.
 * ===================================================================== */

/* Link cost is an 8-bit integer (NO FLOAT -- RP2040 has no FPU). It is
 * derived from RSSI/SNR at frame-receive time by an integer mapping
 * (see mesh_l3.h::mesh_link_cost). Smaller = better link.
 *   1   = excellent (strong RSSI, high SNR)
 *   ... = monotonically worse
 *   LINK_COST_MAX = barely-decodable / unusable
 * Route "metric" = sum of per-hop link costs along the path (additive,
 * isotonic => Bellman-Ford/AODV path comparison stays loop-consistent). */
typedef uint8_t link_cost_t;
/* Guarded so the canonical mesh_types.h definitions (1u/255u) win when both
 * headers land in one translation unit; identical values, avoids a redefinition
 * warning from the differing 'u' suffix. */
#ifndef LINK_COST_BEST
#define LINK_COST_BEST     1
#endif
#ifndef LINK_COST_MAX
#define LINK_COST_MAX      255
#endif
#ifndef LINK_COST_INFINITE
#define LINK_COST_INFINITE 0xFFFFu  /* route metric sentinel (16-bit)    */
#endif

/* Bounded precursor set per route: which neighbor(s) actively use this
 * route, so a RERR can be targeted to them on link break. Defined before
 * routing_entry_t because the struct embeds a precursors[] array of it. */
#ifndef ROUTE_PRECURSORS
#define ROUTE_PRECURSORS 4
#endif

typedef struct {
    node_id_t   nodeId;     /* neighbor's L3 id (0 => slot empty)        */
    int16_t     lastRssi;   /* dBm * 1 (RadioLib getRSSI rounded to int) */
    int8_t      lastSnr;    /* dB   * 1 (RadioLib getSNR  rounded to int)*/
    link_cost_t linkCost;   /* integer cost from (rssi,snr) mapping      */
    uint32_t    lastHeard;  /* millis() of most recent frame from node   */
    uint8_t     flags;      /* NEIGHBOR_FLAG_* bitfield                   */
    uint8_t     missed;     /* consecutive beacon intervals missed       */
} neighbor_entry_t;          /* ~14 B; 16 B padded                        */

/* neighbor flags */
#define NEIGHBOR_FLAG_NONE      0x00
#define NEIGHBOR_FLAG_VALID     0x01  /* slot in use                      */
#define NEIGHBOR_FLAG_SYMMETRIC 0x02  /* we appear in their beacon's nbr  */
                                      /* list => link confirmed two-way   */
#define NEIGHBOR_FLAG_ROUTER    0x04  /* neighbor advertises forwarding   */

/* =====================================================================
 * (a.2) ROUTING TABLE  (AODV-style)
 * ---------------------------------------------------------------------
 * On-demand destination -> next-hop entries. The destination sequence
 * number (destSeq) is the loop-prevention / freshness key: a route is
 * only replaced by one carrying a STRICTLY-NEWER destSeq, or an EQUAL
 * destSeq with a STRICTLY-BETTER metric. (See loop-prevention notes in
 * mesh_l3.h.)
 * ===================================================================== */

typedef MESH_ENUM8(route_state_e) {
    ROUTE_INVALID = 0,   /* empty slot OR invalidated by RERR             */
    ROUTE_VALID,         /* usable; forward DATA along nextHop            */
    ROUTE_DISCOVERING,   /* RREQ outstanding; no usable nextHop yet       */
    ROUTE_REPAIRING      /* local repair / re-discovery of a broken route */
} route_state_t;

typedef struct {
    node_id_t    dest;       /* destination node id (0 => slot empty)     */
    node_id_t    nextHop;    /* neighbor to forward to (0 => none)        */
    seq_t        destSeq;    /* last-known dest sequence number (freshness)*/
    uint8_t      destSeqValid;/* 0 => destSeq unknown (treat as oldest)   */
    uint8_t      hopCount;   /* hops to dest via nextHop                   */
    uint16_t     metric;     /* additive link-cost sum (LINK_COST_INFINITE
                              *  => invalid); primary path comparator      */
    uint32_t     expiry;     /* millis() at which route goes stale         */
    route_state_t state;     /* ROUTE_* lifecycle                          */
    uint8_t      flags;      /* ROUTE_FLAG_* (e.g. repairable)             */
    /* precursor tracking: which neighbor(s) actively use THIS route, so a
     * RERR can be unicast/targeted to them on link break. To stay bounded
     * we keep a small fixed precursor set rather than a list.            */
    node_id_t    precursors[ROUTE_PRECURSORS];
} routing_entry_t;            /* ~ (12 + 2*ROUTE_PRECURSORS) B             */

#define ROUTE_FLAG_NONE       0x00
#define ROUTE_FLAG_REPAIRABLE 0x01  /* attempt local repair before RERR   */
#define ROUTE_FLAG_GRATUITOUS 0x02  /* set up from a gratuitous RREP      */

/* =====================================================================
 * (a.3) SEEN-SET / DUPLICATE CACHE  (broadcast-storm suppression)
 * ---------------------------------------------------------------------
 * Keyed by (origin, broadcastId). Every flooded packet (RREQ, and any
 * other broadcast control) is recorded here on first receipt; later
 * copies that match an unexpired entry are DROPPED without re-flooding.
 * This is THE mechanism that bounds broadcast storms.
 * ===================================================================== */
typedef struct {
    node_id_t  origin;     /* originator of the flood (0 => slot empty)   */
    bcast_id_t bcastId;    /* originator's broadcast id for this flood    */
    uint32_t   firstSeen;  /* millis() of first receipt; entry ages out   */
} seen_entry_t;             /* 8 B                                         */

/* =====================================================================
 * (a.4) PENDING-RREQ TABLE  (our own outstanding route discoveries)
 * ---------------------------------------------------------------------
 * Tracks discoveries WE originated so we can: time out, retry with a
 * widened TTL (ring search), give up (=> drop buffered DATA + report
 * failure to L4), and match incoming RREPs.
 * ===================================================================== */
typedef struct {
    node_id_t  dest;        /* destination we're searching for (0 => free)*/
    bcast_id_t bcastId;     /* broadcast id used for the current attempt  */
    seq_t      lastRreqSeq; /* our seq carried in the RREQ                */
    uint8_t    ttl;         /* TTL used for the current attempt (ring)    */
    uint8_t    retries;     /* attempts made so far (0..RREQ_RETRY)       */
    uint32_t   sentAt;      /* millis() the current RREQ was broadcast    */
    uint8_t    active;      /* 1 => discovery in progress                 */
} pending_rreq_t;

/* =====================================================================
 * (a.5) STORE-AND-FORWARD QUEUE
 * ---------------------------------------------------------------------
 * Holds DATA frames awaiting a route (route unknown => buffer + trigger
 * RREQ) or awaiting their transmit/ACK turn from L2. Fixed capacity;
 * full queue => oldest-droppable / reject new (policy in mesh_l3.h).
 *
 * The frame bytes stored are the FULLY-FORMED L3 frame minus the fields
 * that change per hop (NextHop/HopCount/TTL/CRC), which are stamped at
 * dequeue time just before handing to L2. We store the canonical header
 * + payload and let the forwarder finalize per-hop fields.
 * ===================================================================== */
typedef MESH_ENUM8(queue_state_e) {
    QSTATE_FREE = 0,
    QSTATE_WAIT_ROUTE,   /* buffered; RREQ in flight for dest             */
    QSTATE_READY,        /* route known; awaiting L2 send slot            */
    QSTATE_INFLIGHT      /* handed to L2; awaiting link-ACK / TxDone       */
} queue_state_t;

typedef struct {
    node_id_t   finalDest;  /* ultimate L3 destination                    */
    node_id_t   origin;     /* original source (== our id if locally gen) */
    seq_t       seq;        /* packet seq (origin's), for end-to-end dedup*/
    uint8_t     payloadLen; /* bytes valid in payload[]                   */
    uint8_t     pktType;    /* DATA (control pkts are normally not queued)*/
    uint8_t     ttl;        /* remaining TTL to stamp at send             */
    uint8_t     retries;    /* L3-level send attempts (route re-resolves) */
    uint32_t    enqueueTime;/* millis() enqueued; ages out via QUEUE_TTL  */
    queue_state_t qstate;   /* QSTATE_*                                    */
    uint8_t     payload[MAX_PAYLOAD];
} queue_entry_t;            /* ~ 16 + MAX_PAYLOAD B                        */

/* =====================================================================
 * AGGREGATE L3 STATE  (single static instance lives in mesh_l3.cpp)
 * ===================================================================== */
typedef struct {
    node_id_t        selfId;          /* this node's L3 address           */
    seq_t            mySeq;           /* our own dest-seq (even-increment) */
    bcast_id_t       myBcastId;       /* next broadcast id for our RREQs   */

    neighbor_entry_t neighbors[MAX_NEIGHBORS];
    routing_entry_t  routes[MAX_ROUTES];
    seen_entry_t     seen[MAX_SEEN];
    pending_rreq_t   pending[MAX_PENDING_RREQ];
    queue_entry_t    queue[MAX_QUEUE];

    uint32_t         nextBeaconAt;    /* millis() of next scheduled beacon */

    /* lightweight counters for diagnostics / health (optional to use) */
    uint16_t         statForwarded;
    uint16_t         statDroppedTtl;
    uint16_t         statDroppedNoRoute;
    uint16_t         statDupSuppressed;
} mesh_l3_state_t;

#endif /* MESH_L3_TYPES_H */
