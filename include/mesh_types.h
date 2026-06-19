/* =============================================================================
 * mesh_types.h  --  Canonical shared scalar types for the LoRa mesh stack.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * This is the SINGLE DEFINITION of all cross-layer scalar types, enum helpers,
 * the unified error code set, and the seq_newer() utility. Every other header
 * includes this (directly or via mesh_config.h). Nothing here should ever be
 * redefined elsewhere -- the MESH_*_DEFINED guards exist so older standalone
 * draft headers (mesh_l3_types.h, mesh_wire.h) do not double-define on
 * partial migration; they MUST be removed as the stack matures.
 *
 * Include order: mesh_config.h → mesh_types.h → everything else.
 * ===========================================================================*/
#ifndef MESH_TYPES_H
#define MESH_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_config.h"

/* ── Portable 1-byte-underlying enum macro ───────────────────────────────── */
/* C++11 and C23 support enum Name : uint8_t natively. Older C gets a plain
 * enum; wire (de)serialization reads/writes a uint8_t field-by-field so the
 * in-RAM storage width of the enum is irrelevant to correctness.            */
#if defined(__cplusplus) && (__cplusplus >= 201103L)
#  define MESH_ENUM8(name)  enum name : uint8_t
#  define MESH_ENUM8S(name) enum name : int8_t
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ > 201710L)
#  define MESH_ENUM8(name)  enum name : uint8_t
#  define MESH_ENUM8S(name) enum name : int8_t
#else
#  define MESH_ENUM8(name)  enum name
#  define MESH_ENUM8S(name) enum name
#endif

/* ── Addressing ──────────────────────────────────────────────────────────── */
#ifndef MESH_NODE_ID_T_DEFINED
#define MESH_NODE_ID_T_DEFINED
typedef uint16_t node_id_t;    /* 0x0000 = invalid, 0xFFFF = broadcast, else unicast */
#endif

/* ── Sequence numbers ────────────────────────────────────────────────────── */
#ifndef MESH_SEQ_T_DEFINED
#define MESH_SEQ_T_DEFINED
typedef uint16_t seq_t;        /* per-origin monotone counter, wraps mod 2^16       */
#endif

typedef uint16_t bcast_id_t;   /* per-node flood identifier (RREQ dedup)            */

/* RFC-1982 serial-number comparison: "a is strictly newer than b".
 * Valid while the in-flight seq window stays < 2^15 (trivially true here). */
static inline bool seq_newer(seq_t a, seq_t b) {
    return (int16_t)(a - b) > 0;
}

/* ── Link / routing cost ─────────────────────────────────────────────────── */
typedef uint8_t link_cost_t;   /* 1 = best, 255 = worst; no FPU on M0+            */
#define LINK_COST_BEST      1u
#define LINK_COST_MAX       255u
#define LINK_COST_INFINITE  0xFFFFu   /* 16-bit route-metric sentinel              */

/* ── Packet types (canonical set — one definition, used everywhere) ──────── */
/* Canonical name: mesh_pkt_type_t with MESH_PKT_* enumerators.
 * Short aliases PKT_* provided so L3 code stays readable.                   */
#ifndef MESH_PKTTYPE_DEFINED
#define MESH_PKTTYPE_DEFINED
typedef MESH_ENUM8(mesh_pkt_type_e) {
    MESH_PKT_DATA   = 0,   /* L4 application payload, routed end-to-end          */
    MESH_PKT_BEACON = 1,   /* 1-hop neighbor advertisement (never forwarded)     */
    MESH_PKT_RREQ   = 2,   /* route request, flooded                             */
    MESH_PKT_RREP   = 3,   /* route reply, unicast back along reverse path       */
    MESH_PKT_RERR   = 4,   /* route error / broken-link notification             */
    MESH_PKT_ACK    = 5    /* per-hop link acknowledgement (L2)                  */
} mesh_pkt_type_t;

/* Short aliases for L3 readability */
#define PKT_DATA    MESH_PKT_DATA
#define PKT_BEACON  MESH_PKT_BEACON
#define PKT_RREQ    MESH_PKT_RREQ
#define PKT_RREP    MESH_PKT_RREP
#define PKT_RERR    MESH_PKT_RERR
#define PKT_ACK     MESH_PKT_ACK

/* pkt_type_t alias for compatibility with existing draft headers           */
typedef mesh_pkt_type_t pkt_type_t;
typedef mesh_pkt_type_t mesh_pkttype_t;   /* mesh_l3_control.h alias       */
#endif

/* ── Unified return / error code ─────────────────────────────────────────── */
/* Convention: 0 = success, negatives = error. Codes -100..-199 are mesh
 * stack codes; they never collide with RadioLib's RADIOLIB_ERR_* values
 * (which are also negative but in different ranges).                         */
typedef int16_t mesh_err_t;

#define MESH_OK                    0
#define MESH_ERR_RADIO           -100   /* RadioLib call failed; see phy_last_radio_err() */
#define MESH_ERR_INVALID_ARG     -101
#define MESH_ERR_NOT_INIT        -102
#define MESH_ERR_FRAME_TOO_LONG  -103
#define MESH_ERR_BAD_MAGIC       -110
#define MESH_ERR_BAD_VERSION     -111
#define MESH_ERR_CRC             -112
#define MESH_ERR_RUNT            -113
#define MESH_ERR_DUPLICATE       -114
#define MESH_ERR_NO_RX           -115
#define MESH_ERR_CHANNEL_BUSY    -120
#define MESH_ERR_DUTY_EXCEEDED   -121
#define MESH_ERR_TX_BUSY         -122
#define MESH_ERR_ACK_TIMEOUT     -123
#define MESH_ERR_RETX_EXHAUSTED  -124
#define MESH_ERR_QUEUE_FULL      -125

#endif /* MESH_TYPES_H */
