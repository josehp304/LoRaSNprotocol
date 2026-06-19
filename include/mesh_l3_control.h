/* =====================================================================
 * mesh_l3_control.h  --  On-wire layouts for L3 control PAYLOADS
 * ---------------------------------------------------------------------
 * The L2/L3 FRAME HEADER is defined by the frame owner (mesh_frame.h):
 *
 *   Magic(2)|Ver(1)|Src(2)|Dst(2)|NextHop(2)|HopCount(1)|PktType(1)|
 *   TTL(1)|SeqNum(2)|PayloadLen(1)|Payload(var)|CRC(2)
 *
 * NOTE (design justification): the constants report recommends ADDING a
 * 2-byte SeqNum to the original 15-byte header sketch so that EVERY
 * packet type can be deduplicated via the (origin,seq) seen-set and so
 * route freshness is uniform. We adopt that: header = 17 B. This file
 * assumes that field exists. The MEANING of the header fields per packet
 * type is tabulated at the bottom of this header.
 *
 * The structs below describe the PAYLOAD bytes (what follows the header)
 * for each L3 control packet. They are declared __attribute__((packed))
 * and MUST be (de)serialized field-by-field in little-endian by the L2
 * codec -- DO NOT memcpy the struct over the wire (alignment/endianness
 * portability). The structs document the exact wire order & widths.
 *
 * DESIGN ONLY: no .cpp in this phase.
 * ===================================================================== */
#ifndef MESH_L3_CONTROL_H
#define MESH_L3_CONTROL_H

#include "mesh_l3_types.h"

/* ---- Packet types (mirror mesh_config.h PktType enum) -------------- */
#ifndef MESH_PKTTYPE_DEFINED
#define MESH_PKTTYPE_DEFINED
typedef MESH_ENUM8(mesh_pkttype_e) {
    PKT_DATA   = 0,
    PKT_BEACON = 1,
    PKT_RREQ   = 2,
    PKT_RREP   = 3,
    PKT_RERR   = 4,
    PKT_ACK    = 5   /* per-hop link ACK -- owned by L2, listed for parity */
} mesh_pkttype_t;
#endif

#if defined(__GNUC__)
#  define MESH_PACKED __attribute__((packed))
#else
#  define MESH_PACKED
#endif

/* =====================================================================
 * BEACON payload  (PktType = PKT_BEACON, broadcast, TTL = 1, NOT flooded)
 * ---------------------------------------------------------------------
 * Proactive neighbor discovery. A beacon is single-hop only (TTL=1, never
 * re-broadcast). It advertises liveness and (optionally) a short list of
 * the sender's current neighbors so links can be confirmed SYMMETRIC.
 *
 * Header fields for a BEACON:
 *   Src      = sender id
 *   Dst      = ADDR_BROADCAST
 *   NextHop  = ADDR_BROADCAST
 *   HopCount = 0
 *   TTL      = 1
 *   SeqNum   = sender's mySeq snapshot (lets neighbors learn freshness)
 * ===================================================================== */
typedef struct MESH_PACKED {
    uint8_t   nbrCount;       /* number of entries in nbrs[] that follow   */
    uint8_t   flags;          /* BEACON_FLAG_* (e.g. is-router)            */
    /* followed by nbrCount * sizeof(beacon_nbr_t). Bounded so the whole
     * beacon stays small (see BEACON_MAX_NBRS). */
} beacon_hdr_t;

typedef struct MESH_PACKED {
    node_id_t nodeId;         /* a neighbor the sender currently hears      */
    uint8_t   linkCost;       /* sender's measured cost to that neighbor    */
} beacon_nbr_t;               /* 3 B each                                   */

#define BEACON_FLAG_NONE   0x00
#define BEACON_FLAG_ROUTER 0x01   /* sender will forward (mesh router)      */
/* Bound the advertised neighbor list so a beacon fits comfortably and
 * stays cheap on airtime/duty. With SF9 a ~16 B beacon ~ 165 ms.         */
#ifndef BEACON_MAX_NBRS
#define BEACON_MAX_NBRS 6
#endif

/* =====================================================================
 * RREQ payload  (PktType = PKT_RREQ, broadcast, FLOODED with TTL + dedup)
 * ---------------------------------------------------------------------
 * Originator floods a route request. Each forwarder: dedups on
 * (origin,bcastId) via the seen-set, sets up a REVERSE route toward the
 * origin, increments the accumulated metric/hopCount, decrements TTL, and
 * re-broadcasts (if TTL>0 and not seen).
 *
 * Header fields for an RREQ:
 *   Src      = ORIGINATOR id            (does NOT change while flooding)
 *   Dst      = ADDR_BROADCAST           (it's a flood)
 *   NextHop  = ADDR_BROADCAST
 *   HopCount = hops travelled so far    (incremented per hop)
 *   TTL      = remaining hops           (decremented per hop; drop at 0)
 *   SeqNum   = == bcastId of this RREQ  (so the seen-set key (Src,SeqNum)
 *              is exactly (origin,bcastId) -- uniform with other floods)
 * ===================================================================== */
typedef struct MESH_PACKED {
    node_id_t  origin;       /* who wants the route (== header Src)         */
    node_id_t  dest;         /* destination being searched for             */
    bcast_id_t bcastId;      /* originator's flood id (== header SeqNum)    */
    seq_t      originSeq;    /* originator's current seq (for reverse-route
                              *  freshness toward the origin)               */
    seq_t      destSeq;      /* last destSeq the origin knows for dest;
                              *  destSeqValid=0 => unknown (request newest) */
    uint8_t    destSeqValid; /* 1 => destSeq is meaningful                  */
    uint16_t   metric;       /* accumulated additive link-cost so far       */
    uint8_t    flags;        /* RREQ_FLAG_*                                 */
} rreq_payload_t;            /* 14 B                                        */

#define RREQ_FLAG_NONE       0x00
#define RREQ_FLAG_GRATUITOUS 0x01  /* ask dest to also send a gratuitous   */
                                   /* RREP so the route is bidirectional   */
#define RREQ_FLAG_REPAIR     0x02  /* this RREQ is a local route repair    */

/* =====================================================================
 * RREP payload  (PktType = PKT_RREP, UNICAST back along the reverse route)
 * ---------------------------------------------------------------------
 * Generated by the destination (or an intermediate node that has a fresh
 * enough route to dest). Travels hop-by-hop back to the origin using the
 * reverse routes the RREQ established. Each forwarder installs the
 * FORWARD route toward dest, increments metric/hopCount, decrements TTL.
 *
 * Header fields for an RREP:
 *   Src      = ORIGIN of the original RREQ (final unicast destination)
 *   Dst      = ORIGIN of the original RREQ
 *   NextHop  = the reverse-route next hop toward origin (changes per hop)
 *   HopCount = hops from dest so far (incremented per hop)
 *   TTL      = remaining hops
 *   SeqNum   = dest's current destSeq (the freshness this RREP carries)
 * ===================================================================== */
typedef struct MESH_PACKED {
    node_id_t dest;         /* destination this route leads to             */
    node_id_t origin;       /* who originally asked (final recipient)      */
    seq_t     destSeq;      /* dest's sequence number (== header SeqNum)   */
    uint8_t   hopCount;     /* hops from dest accumulated so far           */
    uint16_t  metric;       /* accumulated link-cost from dest so far      */
    uint32_t  lifetimeMs;   /* how long origin/forwarders should keep route*/
    uint8_t   flags;        /* RREP_FLAG_*                                 */
} rrep_payload_t;           /* 14 B                                        */

#define RREP_FLAG_NONE       0x00
#define RREP_FLAG_GRATUITOUS 0x01  /* this RREP was solicited gratuitously */

/* =====================================================================
 * RERR payload  (PktType = PKT_RERR, sent to precursors on link break)
 * ---------------------------------------------------------------------
 * When a next-hop link breaks (ACK exhaustion at L2, neighbor timeout,
 * or receipt of a DATA we can't forward), the node lists the now-
 * UNREACHABLE destinations and notifies the precursors that were using
 * those routes. Each listed dest carries an INCREMENTED destSeq so the
 * invalidation propagates monotonically (anti count-to-infinity).
 *
 * Sent as a unicast to each known precursor, or broadcast (TTL=1) if the
 * precursor set is empty/unknown. Recipients invalidate matching routes
 * and propagate the RERR to THEIR precursors.
 *
 * Header fields for an RERR:
 *   Src      = node detecting the break
 *   Dst      = a precursor id (unicast) or ADDR_BROADCAST
 *   NextHop  = precursor id  or ADDR_BROADCAST
 *   TTL      = 1 (RERR propagates one hop per precursor step, re-emitted
 *              by each recipient toward its own precursors)
 *   SeqNum   = sender's mySeq snapshot (dedup if broadcast)
 * ===================================================================== */
typedef struct MESH_PACKED {
    uint8_t  count;          /* number of unreachable_dest_t that follow    */
    uint8_t  flags;          /* reserved                                    */
    /* followed by count * sizeof(unreachable_dest_t) */
} rerr_hdr_t;

typedef struct MESH_PACKED {
    node_id_t dest;          /* destination now unreachable                 */
    seq_t     destSeq;       /* INCREMENTED dest seq (advertises staleness) */
} unreachable_dest_t;        /* 4 B each                                    */

/* Bound how many dead destinations one RERR can carry (keeps frame small).
 * Most breaks affect 1 next-hop and thus a handful of routes.            */
#ifndef RERR_MAX_DESTS
#define RERR_MAX_DESTS 8
#endif

/* =====================================================================
 * DATA payload  (PktType = PKT_DATA)
 * ---------------------------------------------------------------------
 * L3 does not interpret DATA payload bytes -- they belong to L4. L3 only
 * routes using the header. For completeness the per-type header semantics:
 *
 *   Src      = ORIGINATING node (end-to-end source; never rewritten)
 *   Dst      = FINAL destination (end-to-end; never rewritten)
 *   NextHop  = the neighbor this hop forwards to (REWRITTEN each hop)
 *   HopCount = hops travelled (incremented each hop)
 *   TTL      = remaining hops (decremented each hop; DROP at 0)
 *   SeqNum   = originator's per-packet seq (end-to-end dup detection)
 *
 * Forwarding rule summary (full logic in mesh_l3.h):
 *   1. If Dst == self -> deliver to L4.
 *   2. Else look up route to Dst. If found & VALID: set NextHop, ++HopCount,
 *      --TTL; if TTL==0 drop (loop bound). Hand to L2 for the NextHop link.
 *   3. If no route: buffer in store-and-forward queue, originate RREQ.
 * ===================================================================== */

/* ---------------------------------------------------------------------
 * Compile-time sanity: every control payload must fit MAX_PAYLOAD so it
 * never overflows the frame buffer. (Beacon/RERR also bounded by their
 * MAX_NBRS / MAX_DESTS list caps.)
 * ------------------------------------------------------------------- */
#define L3_BEACON_MAX_BYTES (sizeof(beacon_hdr_t) + BEACON_MAX_NBRS*sizeof(beacon_nbr_t))
#define L3_RERR_MAX_BYTES   (sizeof(rerr_hdr_t)   + RERR_MAX_DESTS*sizeof(unreachable_dest_t))

#if defined(__cplusplus) && (__cplusplus >= 201103L)
static_assert(sizeof(rreq_payload_t) <= MAX_PAYLOAD, "RREQ payload exceeds MAX_PAYLOAD");
static_assert(sizeof(rrep_payload_t) <= MAX_PAYLOAD, "RREP payload exceeds MAX_PAYLOAD");
static_assert(L3_BEACON_MAX_BYTES   <= MAX_PAYLOAD, "BEACON payload exceeds MAX_PAYLOAD");
static_assert(L3_RERR_MAX_BYTES     <= MAX_PAYLOAD, "RERR payload exceeds MAX_PAYLOAD");
#endif

#endif /* MESH_L3_CONTROL_H */
