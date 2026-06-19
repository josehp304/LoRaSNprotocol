/**
 * @file    mesh_wire.h
 * @brief   [SUPERSEDED — see DESIGN.md §3 resolution]
 *          This was an independent layer-design draft (15-byte header, packed-
 *          struct-is-wire, LE). The final authoritative wire format is in
 *          mesh_frame.h (17-byte header, offset-based LE codec). The two
 *          headers MUST NOT both be included; prefer mesh_frame.h.
 *          Kept for reference only. Do not include in production builds.
 *
 *          KEY DIFFERENCES vs. the canonical design:
 *            - Header: 15 B (no `flags` or `reserved` fields) vs 17 B canonical
 *            - MAX_PAYLOAD_PHY: 238 (from 15 B header) vs 236 canonical
 *            - Serialization: struct memcpy (LE-only) vs field-by-field offsets
 *
 * @brief   L2/L3 WIRE PROTOCOL — byte-level on-air frame format for the routed
 *          multi-hop LoRa mesh.  DESIGN-ONLY header: no implementation.
 *
 * Target : RP2040 (Cortex-M0+, LITTLE-ENDIAN, no FPU) / Arduino-pico / RadioLib SX1278.
 *
 * SCOPE OF THIS FILE
 *   - The common frame header that prefixes EVERY on-air frame.
 *   - The six per-type payload layouts (DATA, BEACON, RREQ, RREP, RERR, ACK).
 *   - CRC-16 definition (placement + coverage + polynomial).
 *   - Endianness / serialization contract.
 *   - static_assert guards so a layout drift breaks the build, not the link.
 *
 * WHAT IS *NOT* HERE
 *   - Table layouts (neighbor/route/seen/queue)  -> mesh_l3.h
 *   - Tunable timing/sizing constants             -> mesh_config.h
 *   - RadioLib glue / state machines              -> .cpp (later phase)
 *
 * SERIALIZATION CONTRACT (read this before touching any struct)
 *   The RP2040 is little-endian. ALL multi-byte integer fields in this protocol
 *   are defined LITTLE-ENDIAN on the wire. Because the MCU is also little-endian,
 *   a node MAY memcpy a packed struct straight into the TX buffer and back out of
 *   the RX buffer with ZERO byte-swapping. This is a deliberate cost optimisation
 *   for an FPU-less M0+: no htons()/ntohs() per field, no per-field shifting.
 *
 *   The price: any big-endian peer (none exist on this fixed hardware) would have
 *   to swap. We accept that — the network is homogeneous RP2040. The choice is
 *   documented so a future heterogeneous port knows exactly what to byte-swap.
 *
 *   The packed structs below are therefore BOTH the in-RAM representation AND the
 *   on-wire representation. They are #pragma pack(1) so the compiler inserts no
 *   padding that would desync the two. Every struct has a static_assert on sizeof.
 */
#ifndef MESH_WIRE_H
#define MESH_WIRE_H

#include <stdint.h>
#include <stddef.h>

/* ======================================================================== *
 *  0. COMPILE-TIME ENVIRONMENT GUARDS
 * ======================================================================== */

/* The zero-swap serialization contract is only valid on a little-endian build.
 * Trip the build loudly if someone ever cross-compiles for a BE target. */
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#  error "mesh_wire.h assumes a little-endian target (RP2040). Port the serializer."
#endif

/* Portable 1-byte packing.
 *
 * IMPORTANT PORTABILITY NOTE: bare __attribute__((packed)) is honoured by
 * arm-none-eabi-gcc (the RP2040 target toolchain) and by Linux/macOS GCC/Clang,
 * but it is SILENTLY IGNORED for sub-word alignment by MinGW GCC on Windows,
 * which defaults to the MS-struct ABI. That ABI keeps a uint16_t 2-byte aligned
 * even inside a "packed" struct, inserting a pad byte (so a 15-byte header
 * becomes 17 on a Windows host build). To make the wire layout identical on the
 * target AND on any host used for verification/sniffer tooling, force the GCC
 * struct ABI alongside packing. The static_asserts below are what make this
 * non-negotiable: a build that pads will FAIL TO COMPILE rather than emit a
 * desynced frame. */
#if defined(__GNUC__) || defined(__clang__)
#  if defined(__MINGW32__) || defined(__MINGW64__) || defined(_WIN32)
#    define MESH_PACKED __attribute__((gcc_struct, packed))
#  else
#    define MESH_PACKED __attribute__((packed))
#  endif
#else
#  define MESH_PACKED
#endif

/* ======================================================================== *
 *  1. PROTOCOL IDENTITY — magic, version
 * ======================================================================== */

/*
 * FRAME_MAGIC: 2 bytes, the first thing on the wire. Cheap, fast reject of
 * foreign LoRa traffic / noise / other protocols sharing the 433 channel BEFORE
 * we spend cycles on CRC. 2 bytes (not 1) keeps the false-accept probability of
 * random noise at ~1/65536 per frame instead of ~1/256.
 *
 * Value 0x4C6D = ASCII "Lm" (LoRa-mesh). Stored little-endian => byte[0]=0x6D('m'),
 * byte[1]=0x4C('L'). The matcher compares the 16-bit field, not byte order, so
 * the literal 0x4C6D is what you test against after deserialization.
 */
#define MESH_FRAME_MAGIC      ((uint16_t)0x4C6D)

/*
 * PROTO_VERSION: 1 byte. Bumped on ANY incompatible wire change (field move,
 * size change, new mandatory field). 1 byte = 255 future revisions, ample.
 *
 * VERSION-MISMATCH POLICY (justified):
 *   - rx.version == MESH_PROTO_VERSION   -> process normally.
 *   - rx.version  > MESH_PROTO_VERSION   -> peer is newer. DROP silently. We
 *       cannot know if header geometry changed, so parsing is unsafe. Do NOT
 *       forward (could corrupt the mesh). Optionally count for diagnostics.
 *   - rx.version  < MESH_PROTO_VERSION   -> peer is older. DROP by default.
 *       (A future minor-compatible path could parse known-older layouts, but the
 *        safe default for a lossy mesh is: same major == proceed, else drop.)
 *   In all mismatch cases the frame is neither acted on nor relayed. This keeps
 *   a half-upgraded network from forwarding garbage that loops or wedges routes.
 */
#define MESH_PROTO_VERSION    ((uint8_t)1)

/* ======================================================================== *
 *  2. ADDRESSING
 * ======================================================================== */

/*
 * node_id_t: 16-bit. 65534 usable addresses — vastly more than any realistic
 * mesh, while staying 2 bytes (matches Src/Dst/NextHop in the header sketch).
 * Going to 1 byte (256 nodes) was rejected: too few for ID collisions to be
 * comfortably rare with random/derived IDs, and the 1-byte saving is irrelevant.
 */
typedef uint16_t node_id_t;

/* seq_t: 16-bit monotonically-increasing-per-origin counter, wraps mod 2^16.
 * Drives (a) RREQ/route freshness (AODV "higher seq = newer") and
 *        (b) (origin,seq) duplicate / broadcast-storm suppression.
 * Compared with RFC-1982 serial arithmetic (see mesh_seq_newer below), never by
 * magnitude, so the 65535->0 rollover is handled correctly. */
typedef uint16_t seq_t;

#define MESH_ADDR_INVALID     ((node_id_t)0x0000)  /* unassigned / "no route"   */
#define MESH_ADDR_BROADCAST   ((node_id_t)0xFFFF)  /* all neighbours (1 hop)     */
/* 0x0001..0xFFFE are assignable unicast node IDs. */

/* RFC 1982 serial-number comparison: true iff a is strictly newer than b.
 * Valid while the in-flight seq window stays < 2^15 (trivially true here). */
static inline bool mesh_seq_newer(seq_t a, seq_t b) {
    return (int16_t)(a - b) > 0;
}

/* ======================================================================== *
 *  3. PACKET TYPES
 * ======================================================================== */

/*
 * 1-byte type tag. 6 defined values; 1 byte leaves 250 spare for future control
 * frames (e.g. RREP-ACK, time-sync, OTA). Kept as a plain enum cast to uint8_t
 * on the wire — see mesh_hdr_t.pkt_type. Do NOT rely on enum storage size for
 * the wire; the wire field is an explicit uint8_t.
 */
enum mesh_pkt_type_t {
    MESH_PKT_DATA   = 0,  /* L4 application payload, end-to-end (routed)        */
    MESH_PKT_BEACON = 1,  /* 1-hop neighbour advertisement (never forwarded)    */
    MESH_PKT_RREQ   = 2,  /* route request, flooded                            */
    MESH_PKT_RREP   = 3,  /* route reply, unicast back along reverse path      */
    MESH_PKT_RERR   = 4,  /* route error / broken-link notification            */
    MESH_PKT_ACK    = 5   /* per-hop link acknowledgement (L2)                 */
};

/* ======================================================================== *
 *  4. COMMON FRAME HEADER  (prefixes every frame)
 * ======================================================================== */

/*
 * HEADER FIELD TABLE (all multi-byte = little-endian on wire)
 *
 *  off sz  field        type      meaning / justification
 *  --- --  -----------  --------  -----------------------------------------------
 *   0  2   magic        u16       MESH_FRAME_MAGIC. Fast foreign-traffic reject.
 *   2  1   version      u8        MESH_PROTO_VERSION. Mismatch -> drop (see policy).
 *   3  2   src          u16       ORIGIN node (immutable end-to-end). Routing &
 *                                 (origin,seq) dedup key. NOT rewritten per hop.
 *   5  2   dst          u16       FINAL destination (immutable). 0xFFFF=broadcast.
 *   7  2   next_hop     u16       L2 next-hop address. REWRITTEN at every forward.
 *                                 Lets a node cheaply ignore frames not for it
 *                                 (unicast) without L3 work; 0xFFFF on floods.
 *   9  1   hop_count    u8        Hops travelled from origin. STARTS 0, +1/hop.
 *                                 Feeds route metric / diagnostics. Distinct from
 *                                 TTL (counts up vs counts down) — see note.
 *  10  1   pkt_type     u8        mesh_pkt_type_t.
 *  11  1   ttl          u8        Hops remaining. STARTS DEFAULT_TTL, -1/hop,
 *                                 drop at 0. Hard loop bound independent of seq.
 *  12  2   seq          u16       Per-origin sequence number. Dedup + freshness.
 *                                 *** ADDED vs the original sketch (justified). ***
 *  14  1   payload_len  u8        Bytes of payload following the header (0..MAX).
 *                                 1 byte caps at 255, which is also the LoRa FIFO
 *                                 limit — perfectly sufficient, no field waste.
 *  --- --
 *  15      = MESH_HDR_LEN
 *
 *  CRC-16 (2 bytes) trails the payload, NOT part of this struct — see §6.
 *
 *  WHY seq IS IN THE HEADER (sketch flagged it as missing):
 *    Every forwarder must dedup EVERY packet type uniformly via (src,seq) without
 *    parsing type-specific payloads. Putting seq in the common header makes
 *    broadcast-storm suppression and route-freshness a header-only operation —
 *    the L2/early-L3 fast path never needs to look at the payload. Carrying it in
 *    payload instead would force per-type special-casing and break dedup for DATA.
 *
 *  WHY BOTH hop_count AND ttl (not redundant):
 *    They move in OPPOSITE directions and serve different consumers.
 *      ttl  : monotonically DOWN, sole job = guarantee termination (loop kill),
 *             independent of any table state. Drop-at-zero is unconditional.
 *      hop_count : monotonically UP, feeds the ROUTING METRIC (path length in
 *             RREQ/RREP) and link diagnostics. RREP cost selection needs the
 *             accumulated hop count; TTL alone cannot supply "distance from
 *             origin" because its start value is configurable per packet.
 *    Keeping both costs 1 byte and removes an entire class of routing bugs.
 *
 *  WHY next_hop IS IN THE HEADER (and is the one mutable address field):
 *    Unicast forwarding: a receiver checks next_hop ∈ {me, BROADCAST}; if not,
 *    it can discard at L2 with no routing-table lookup. src/dst stay immutable so
 *    end-to-end identity and the dedup key survive every hop.
 */
typedef struct MESH_PACKED {
    uint16_t magic;        /* off 0  */
    uint8_t  version;      /* off 2  */
    node_id_t src;         /* off 3  */
    node_id_t dst;         /* off 5  */
    node_id_t next_hop;    /* off 7  */
    uint8_t  hop_count;    /* off 9  */
    uint8_t  pkt_type;     /* off 10 */
    uint8_t  ttl;          /* off 11 */
    seq_t    seq;          /* off 12 */
    uint8_t  payload_len;  /* off 14 */
} mesh_hdr_t;              /* total 15 */

#define MESH_HDR_LEN   15u
#define MESH_CRC_LEN    2u

static_assert(sizeof(mesh_hdr_t) == MESH_HDR_LEN,
              "mesh_hdr_t must be exactly 15 packed bytes (wire layout drift!)");

/* ======================================================================== *
 *  5. LENGTH BUDGET vs THE 255-BYTE LoRa LIMIT
 * ======================================================================== */

/*
 *  On-air frame = [ header 15 ][ payload payload_len ][ CRC 2 ]
 *
 *  Hard PHY ceiling (SX1278 256-byte FIFO, single packet):       255 bytes
 *  Fixed overhead (header + CRC):                                 17 bytes
 *  => ABSOLUTE max payload_len that still fits on air:           238 bytes
 *
 *  We expose two limits:
 *    MESH_MAX_PAYLOAD_PHY   238  - the protocol/PHY hard ceiling (validate RX).
 *    MESH_MAX_PAYLOAD_APP    64  - the *application* cap (mesh_config.h MAX_PAYLOAD).
 *                                  Smaller payloads => shorter airtime, fewer
 *                                  collisions, less duty-cycle burn, smaller
 *                                  store-and-forward queue slots (96 B/slot).
 *
 *  ENFORCEMENT RULES (must hold in the serializer/validator):
 *    TX: reject if payload_len > MESH_MAX_PAYLOAD_APP.
 *    RX: reject (drop, no forward) if
 *           (MESH_HDR_LEN + payload_len + MESH_CRC_LEN) > on_air_len   OR
 *           payload_len > MESH_MAX_PAYLOAD_PHY.
 *        i.e. payload_len must be self-consistent with the actually-received
 *        RadioLib getPacketLength(); a lying length field is a malformed frame.
 */
#define MESH_MAX_PAYLOAD_PHY   238u
#define MESH_MAX_PAYLOAD_APP    64u   /* keep in sync with mesh_config MAX_PAYLOAD */
#define MESH_MAX_FRAME_PHY     255u   /* header+payload+crc must never exceed this */

static_assert(MESH_HDR_LEN + MESH_MAX_PAYLOAD_PHY + MESH_CRC_LEN == MESH_MAX_FRAME_PHY,
              "PHY payload ceiling inconsistent with 255-byte frame limit");
static_assert(MESH_MAX_PAYLOAD_APP <= MESH_MAX_PAYLOAD_PHY,
              "App payload cap exceeds PHY ceiling");

/* Convenience: a full scratch frame buffer sized to the PHY max. */
typedef struct MESH_PACKED {
    uint8_t bytes[MESH_MAX_FRAME_PHY];
} mesh_frame_buf_t;

/* ======================================================================== *
 *  6. CRC-16 — choice, coverage, placement
 * ======================================================================== */

/*
 *  ALGORITHM: CRC-16/CCITT-FALSE
 *      polynomial 0x1021 (x^16 + x^12 + x^5 + 1), init 0xFFFF,
 *      no input/output reflection, no final XOR.
 *
 *  WHY THIS ONE:
 *    - 0x1021 has excellent Hamming-distance properties for the short (<256 B)
 *      frames used here (detects all 1-/2-bit errors, all odd-bit errors, all
 *      bursts <=16 bits) — the dominant LoRa error modes.
 *    - Ubiquitous, well-tested, byte-at-a-time table or bitwise; both are tiny
 *      and FAST on the M0+ (no FPU involved, integer only).
 *    - init=0xFFFF (CCITT-FALSE) catches leading-zero / all-zero corruption that
 *      init=0x0000 would miss.
 *
 *  COVERAGE: computed over [ header (15 B) .. end of payload ], i.e. exactly
 *            (MESH_HDR_LEN + payload_len) bytes starting at frame byte 0.
 *            The 2 CRC bytes themselves are NOT included.
 *
 *  PLACEMENT: the 2 CRC bytes are appended AFTER the payload, stored
 *             LITTLE-ENDIAN (crc & 0xFF first, then crc >> 8) to match the
 *             whole-protocol LE convention. They are a trailer, deliberately
 *             OUTSIDE mesh_hdr_t (header geometry stays fixed regardless of
 *             payload length; the CRC floats at offset 15+payload_len).
 *
 *  RELATION TO RadioLib's PHY CRC: leave RadioLib's built-in LoRa CRC ON. That
 *  is an independent PHY-layer check (hardware-assisted). Our L2 CRC-16 is an
 *  end-of-frame integrity check we control and can verify identically on every
 *  hop and on a sniffer; defence in depth, the two are not redundant.
 */
#define MESH_CRC16_POLY   ((uint16_t)0x1021)
#define MESH_CRC16_INIT   ((uint16_t)0xFFFF)

/* Offset of the CRC trailer within a frame, given a payload length. */
static inline size_t mesh_crc_offset(uint8_t payload_len) {
    return (size_t)MESH_HDR_LEN + (size_t)payload_len;
}
/* Total on-air length of a frame with the given payload length. */
static inline size_t mesh_frame_len(uint8_t payload_len) {
    return (size_t)MESH_HDR_LEN + (size_t)payload_len + MESH_CRC_LEN;
}

/* ======================================================================== *
 *  7. PER-TYPE PAYLOAD LAYOUTS
 *     Each struct is the bytes that follow mesh_hdr_t, i.e. it occupies the
 *     payload region and payload_len == sizeof(struct) for fixed-size types.
 *     All multi-byte fields little-endian. All static_asserted.
 * ======================================================================== */

/* ---- 7.1 DATA (type 0) ----------------------------------------------------
 * Routed end-to-end application data. The mesh treats the app bytes as opaque;
 * L4 imposes its own structure (see mesh_app.h). We define only the envelope:
 * a 1-byte app port/type so multiple L4 services can share the mesh, then raw
 * bytes. hdr.src/hdr.dst already carry the endpoints, so DATA adds no addresses.
 */
typedef struct MESH_PACKED {
    uint8_t  app_port;                         /* L4 demux: which sensor/service */
    uint8_t  app_data[];                       /* flexible: payload_len-1 bytes  */
} mesh_data_t;
#define MESH_DATA_HDR_LEN   1u
/* (flexible array member => no fixed sizeof assert; app_data length is
 *  hdr.payload_len - MESH_DATA_HDR_LEN, validated against MESH_MAX_PAYLOAD_APP.) */

/* ---- 7.2 BEACON (type 1) --------------------------------------------------
 * 1-hop neighbour advertisement. NEVER forwarded (TTL effectively 1; a receiver
 * must not relay BEACONs). Lets neighbours learn each other + a coarse self-view.
 * hdr.src = beaconing node. Link metric (RSSI/SNR) is measured by the RECEIVER
 * from the PHY, so it is NOT carried here — putting your own RSSI in the frame
 * is meaningless to the peer. We DO carry the sender's current routing-seq so
 * neighbours can spot freshness, and a small capability/flags byte.
 */
typedef struct MESH_PACKED {
    seq_t    origin_seq;     /* sender's latest sequence counter (freshness hint) */
    uint8_t  flags;          /* bit0=is_router, bit1=has_gateway, ... (see below) */
    uint8_t  hello_interval; /* sender's beacon period in *seconds* (0=unknown);   */
                             /* lets a neighbour size its NEIGHBOR_TIMEOUT.        */
    uint8_t  nbr_count;      /* how many neighbours sender currently has (density) */
} mesh_beacon_t;
#define MESH_BEACON_LEN   5u
static_assert(sizeof(mesh_beacon_t) == MESH_BEACON_LEN, "beacon payload size drift");

/* BEACON.flags bit definitions */
#define MESH_BCN_FLAG_ROUTER     0x01  /* node forwards traffic (vs leaf/endpoint) */
#define MESH_BCN_FLAG_GATEWAY     0x02  /* node has uplink / is a sink              */
#define MESH_BCN_FLAG_LOWBATT     0x04  /* deprioritise as a relay                  */
/* bits 0x08..0x80 reserved, send 0. */

/* ---- 7.3 RREQ (type 2) ----------------------------------------------------
 * AODV-style flooded route request. hdr.src=originator, hdr.dst=BROADCAST while
 * flooding (the *target* we want a route to is in the payload, because dst on the
 * wire is the flood scope, not the search target). hdr.seq = the RREQ id used in
 * the (src,seq) seen-set for storm suppression. We carry the target id and the
 * best-known sequence numbers so intermediate nodes can answer if fresh enough.
 */
typedef struct MESH_PACKED {
    node_id_t target;        /* the destination a route is being sought FOR        */
    seq_t    target_seq;     /* last-known seq of target (0 => "don't care/unknown")*/
    seq_t    orig_seq;       /* originator's own current seq (reverse-route fresh.) */
    uint8_t  metric;         /* accumulated path cost so far (sum of link costs)    */
                             /* distinct from hdr.hop_count: weighted, not just #hops */
    uint8_t  flags;          /* bit0=dest-only (only target may reply), ...         */
} mesh_rreq_t;
#define MESH_RREQ_LEN   8u
static_assert(sizeof(mesh_rreq_t) == MESH_RREQ_LEN, "rreq payload size drift");

#define MESH_RREQ_FLAG_DEST_ONLY   0x01  /* suppress intermediate replies */

/* ---- 7.4 RREP (type 3) ----------------------------------------------------
 * Route reply, UNICAST back along the reverse path toward the original RREQ
 * originator. hdr.src = the node that owns/answers for the route (the target, or
 * an intermediate with a fresh route). hdr.dst = the RREQ originator. next_hop is
 * rewritten each hop along the reverse route. Payload restates which destination
 * this reply is FOR and its authoritative seq + cost so every hop on the way back
 * installs a forward route entry.
 */
typedef struct MESH_PACKED {
    node_id_t target;        /* destination this route leads to                    */
    seq_t    target_seq;     /* authoritative seq for that destination (freshness) */
    uint8_t  metric;         /* total path cost from this point to target          */
    uint8_t  hop_count;      /* hops from replier to target (snapshot at reply)     */
    uint8_t  lifetime_s;     /* suggested route lifetime in seconds (cap to config) */
    uint8_t  flags;          /* reserved, send 0                                    */
} mesh_rrep_t;
#define MESH_RREP_LEN   8u
static_assert(sizeof(mesh_rrep_t) == MESH_RREP_LEN, "rrep payload size drift");

/* ---- 7.5 RERR (type 4) ----------------------------------------------------
 * Route error: a node detected a broken next-hop (e.g. ACK retries exhausted,
 * or neighbour timed out) and notifies upstream users of that route. Can list
 * MULTIPLE now-unreachable destinations to amortise the broadcast. count then
 * a packed array of (dest, dest_seq) pairs. payload_len = 1 + count*4.
 */
typedef struct MESH_PACKED {
    uint8_t  count;          /* number of unreachable entries that follow (>=1)     */
    /* followed by 'count' × mesh_rerr_entry_t */
} mesh_rerr_t;
#define MESH_RERR_HDR_LEN   1u

typedef struct MESH_PACKED {
    node_id_t dest;          /* destination now unreachable                         */
    seq_t    dest_seq;       /* incremented seq for that dest (invalidates routes)  */
} mesh_rerr_entry_t;
#define MESH_RERR_ENTRY_LEN   4u
static_assert(sizeof(mesh_rerr_entry_t) == MESH_RERR_ENTRY_LEN, "rerr entry size drift");

/* Max RERR entries that fit under the APP payload cap:
 *   1 (count) + N*4 <= MESH_MAX_PAYLOAD_APP  =>  N <= (64-1)/4 = 15 */
#define MESH_RERR_MAX_ENTRIES   ((MESH_MAX_PAYLOAD_APP - MESH_RERR_HDR_LEN) / MESH_RERR_ENTRY_LEN)

/* ---- 7.6 ACK (type 5) -----------------------------------------------------
 * PER-HOP link acknowledgement (L2). Sent by the immediate receiver back to the
 * immediate sender to confirm one hop succeeded; drives retransmit logic. It is
 * NOT end-to-end. The frame it acknowledges is identified by the ORIGINAL
 * frame's (src,seq) — those uniquely name the in-flight packet across the mesh.
 * ACK is unicast (hdr.dst = the prior sender), TTL=1, never forwarded.
 *
 * NOTE: hdr.src/hdr.dst on the ACK frame itself are the two link endpoints; the
 * ack_src/ack_seq fields below name the DATA/control frame BEING acknowledged.
 */
typedef struct MESH_PACKED {
    node_id_t ack_src;       /* hdr.src of the frame being acknowledged             */
    seq_t    ack_seq;        /* hdr.seq of the frame being acknowledged             */
    uint8_t  status;         /* 0 = OK/accepted; nonzero = nak reason (queue full…) */
} mesh_ack_t;
#define MESH_ACK_LEN   5u
static_assert(sizeof(mesh_ack_t) == MESH_ACK_LEN, "ack payload size drift");

/* ACK.status codes */
#define MESH_ACK_OK            0x00
#define MESH_ACK_NAK_QUEUE     0x01  /* received OK but forward queue full, will drop */
#define MESH_ACK_NAK_NOROUTE   0x02  /* received OK but I have no route onward         */

/* ======================================================================== *
 *  8. WHOLE-FRAME SIZE SANITY (largest fixed control payload still fits APP cap)
 * ======================================================================== */
static_assert(MESH_RREQ_LEN  <= MESH_MAX_PAYLOAD_APP, "RREQ exceeds app payload cap");
static_assert(MESH_RREP_LEN  <= MESH_MAX_PAYLOAD_APP, "RREP exceeds app payload cap");
static_assert(MESH_BEACON_LEN<= MESH_MAX_PAYLOAD_APP, "BEACON exceeds app payload cap");
static_assert(MESH_ACK_LEN   <= MESH_MAX_PAYLOAD_APP, "ACK exceeds app payload cap");

/* ======================================================================== *
 *  9. DESERIALIZATION HELPERS (zero-copy view; see contract at top of file)
 *     On little-endian RP2040 these are just pointer casts. Provided as the
 *     SANCTIONED access pattern so callers never hand-roll byte math.
 *     (Alignment: M0+ tolerates unaligned 16-bit loads via the compiler for
 *      __packed access; always go through the packed struct, never a raw
 *      uint16_t* into the buffer, to stay alignment-safe.)
 * ======================================================================== */
static inline const mesh_hdr_t* mesh_hdr_view(const uint8_t* frame) {
    return (const mesh_hdr_t*)frame;                 /* header at offset 0 */
}
static inline const uint8_t* mesh_payload_view(const uint8_t* frame) {
    return frame + MESH_HDR_LEN;                      /* payload at offset 15 */
}

#endif /* MESH_WIRE_H */
