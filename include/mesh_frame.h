/* =============================================================================
 * mesh_frame.h  --  On-wire frame format + stack-wide return codes
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * Defines the byte layout shared by L2 (framing/CRC) and L3 (routing fields),
 * plus the unified error/return code enum used by every public function in the
 * PHY and Data-Link interfaces.
 *
 * WIRE FORMAT (LITTLE-ENDIAN for all multi-byte fields, serialized via HF_OFF_*):
 *
 *   Offset  Size  Field        Owner  Meaning
 *   ------  ----  -----------  -----  ----------------------------------------
 *      0     2    Magic         L2    0x4C6D LE (byte[0]=0x6D,byte[1]=0x4C)
 *      2     1    Version       L2    PROTO_VERSION, reject if mismatch
 *      3     2    SrcID         L3    original source node (endpoint)
 *      5     2    DstID         L3    final destination node (endpoint/0xFFFF)
 *      7     2    NextHop       L2/L3 link-layer recipient this hop (0xFFFF=bcast)
 *      9     2    LinkSrc       L2    one-hop sender (prev hop); L2 stamps self.
 *                                     == SrcID on origin hop, differs on forwards.
 *                                     Required for AODV reverse-route setup (L3).
 *     11     1    HopCount      L3    increments each forward (from origin)
 *     12     1    PktType       L3    DATA/BEACON/RREQ/RREP/RERR/ACK
 *     13     1    TTL           L3    decremented each hop; drop at 0
 *     14     2    SeqNum        L3    per-origin seq; (Src,Seq) dup-suppression
 *     16     1    Flags         L2    bitfield: ACK-request, etc.
 *     17     1    reserved      --    set 0; word alignment / future use
 *     18     1    PayloadLen    L2    bytes of payload following this header
 *     19     N    Payload       L4    PayloadLen bytes (PktType-specific)
 *   19+N     2    CRC16         L2    CCITT over bytes [0 .. 19+N-1], LE
 *
 *   HDR_LEN = 19.
 *
 * NOTE on SeqNum: the original sketch lacked a per-packet sequence number.
 * It is REQUIRED for the (origin,seq) seen-set (broadcast-storm suppression)
 * and AODV-style route freshness, so it is promoted into the header. Together
 * with the 2-byte LinkSrc (one-hop sender, needed for AODV reverse routes) the
 * header is 19 bytes; still trivially within the 255-byte PHY frame and the
 * RP2040 memory budget.
 * ===========================================================================*/
#ifndef MESH_FRAME_H
#define MESH_FRAME_H

#include <stdint.h>
#include "mesh_config.h"
#include "mesh_types.h"   /* canonical pkt_type_t / mesh_err_t / MESH_* codes  */

/* Packet types (header PktType field, 1 byte) are defined canonically in
 * mesh_types.h as mesh_pkt_type_t with PKT_DATA/PKT_BEACON/... aliases. They
 * used to be re-declared here; that produced a typedef collision once both
 * headers landed in one translation unit, so the definition now lives solely
 * in mesh_types.h. The wire values are unchanged (DATA=0 ... ACK=5).          */

/* ----------------------------------------------------------------------------
 * Header Flags bitfield (header offset 14, 1 byte).
 * --------------------------------------------------------------------------*/
#define FLAG_ACK_REQ    0x01  /* sender requests a per-hop L2 ACK              */
#define FLAG_IS_ACK     0x02  /* this frame *is* an ACK (redundant w/ PktType, */
                              /* kept for fast ISR-side classification)        */
#define FLAG_RETX       0x04  /* this is a retransmission (diagnostics)        */
#define FLAG_RESERVED   0xF8  /* must be 0                                     */

/* ----------------------------------------------------------------------------
 * AUTHORITATIVE WIRE LAYOUT -- byte offsets into the raw frame.
 *
 * IMPORTANT DESIGN DECISION: the on-wire format is defined by these explicit
 * offset constants, NOT by a struct's memory layout. We deliberately do NOT
 * rely on `__attribute__((packed))` for the wire size, because struct packing
 * behaviour varies across toolchains/ABIs (e.g. some MinGW/host builds still
 * align uint16_t fields, yielding sizeof 19 not 17). The L2 codec serializes
 * and deserializes field-by-field through these offsets and writes all multi-
 * byte fields LITTLE-ENDIAN (RP2040 is LE; no per-field swaps needed on target;
 * host builds are little-endian on x86/x64 too).
 * --------------------------------------------------------------------------*/
#define HF_OFF_MAGIC        0   /* u16 LE */
#define HF_OFF_VERSION      2   /* u8     */
#define HF_OFF_SRC          3   /* u16 LE */
#define HF_OFF_DST          5   /* u16 LE */
#define HF_OFF_NEXTHOP      7   /* u16 LE */
#define HF_OFF_LINKSRC      9   /* u16 LE -- one-hop sender (prev hop), L2 stamps self */
#define HF_OFF_HOPCOUNT    11   /* u8     */
#define HF_OFF_PKTTYPE     12   /* u8     */
#define HF_OFF_TTL         13   /* u8     */
#define HF_OFF_SEQ         14   /* u16 LE */
#define HF_OFF_FLAGS       16   /* u8     */
#define HF_OFF_RESERVED    17   /* u8 = 0 */
#define HF_OFF_PAYLOADLEN  18   /* u8     */
#define HF_OFF_PAYLOAD     19   /* == HDR_LEN; CRC16 LE trailer follows payload */

/* Self-check: the documented offsets must end exactly at HDR_LEN.            */
typedef char _assert_hdr_layout[(HF_OFF_PAYLOAD == HDR_LEN) ? 1 : -1];

/* ----------------------------------------------------------------------------
 * In-memory DECODED header (host byte order). This is a convenience struct used
 * INSIDE the firmware only -- it is produced by the deserializer and consumed by
 * the serializer. Its sizeof / padding is irrelevant to the wire format, so it
 * is intentionally NOT packed (let the compiler align it naturally for fast
 * field access on the M0+). Never memcpy this straight to/from the radio FIFO;
 * always go through link_serialize()/link_deserialize() (L2).
 * --------------------------------------------------------------------------*/
typedef struct {
    uint16_t magic;      /* FRAME_MAGIC                                        */
    uint8_t  version;    /* PROTO_VERSION                                      */
    uint16_t src;        /* SrcID                                             */
    uint16_t dst;        /* DstID                                             */
    uint16_t next_hop;   /* NextHop                                            */
    uint16_t link_src;   /* one-hop sender (previous hop); L2 stamps self      */
    uint8_t  hop_count;  /* HopCount                                           */
    uint8_t  pkt_type;   /* pkt_type_t                                         */
    uint8_t  ttl;        /* TTL                                                */
    uint16_t seq;        /* SeqNum                                             */
    uint8_t  flags;      /* FLAG_*                                             */
    uint8_t  reserved;   /* set 0                                              */
    uint8_t  payload_len;/* Payload byte count (<= MAX_PAYLOAD)                */
} mesh_header_t;

/* ----------------------------------------------------------------------------
 * Decoded RX frame handed up from L2 to L3. The payload is copied into the
 * fixed inline buffer (no pointers into radio scratch -> safe to queue).
 * --------------------------------------------------------------------------*/
typedef struct {
    mesh_header_t hdr;                 /* deserialized, host byte order        */
    uint8_t       payload[MAX_PAYLOAD];/* hdr.payload_len valid bytes          */
    /* Link metrics of the hop this frame arrived on (from PHY last-RX):       */
    int16_t       rssi_dbm;            /* whole-dBm RSSI of received frame      */
    int8_t        snr_db;              /* whole-dB SNR                          */
    uint32_t      rx_millis;           /* millis() at RxDone                    */
} mesh_rx_frame_t;

/* ============================================================================
 * UNIFIED RETURN / STATUS CODES
 * ----------------------------------------------------------------------------
 * mesh_err_t (int16_t) and the full MESH_OK / MESH_ERR_* set are defined
 * canonically in mesh_types.h (included above). Convention (mirrors RadioLib):
 * 0 == success, negatives in -100.. are stack codes that never collide with
 * RadioLib's RADIOLIB_ERR_*. All public PHY/L2 functions return mesh_err_t.
 * ==========================================================================*/

/* ----------------------------------------------------------------------------
 * CRC-16/CCITT (poly 0x1021, init 0xFFFF) over [data .. data+len). Computed by
 * L2 on TX (appended LITTLE-ENDIAN, low byte first — matches the wire-layout
 * table above and mesh_config.h) and on RX (verified, frame dropped on
 * mismatch). Declared here so both the PHY test harness and L2 share one
 * definition.
 * --------------------------------------------------------------------------*/
uint16_t mesh_crc16(const uint8_t *data, uint16_t len);

/* ----------------------------------------------------------------------------
 * FRAME CODEC (L2-owned). Endian-safe field-by-field (de)serialization through
 * the HF_OFF_* offsets above. These decouple the wire format from struct
 * padding/endianness.
 *
 * mesh_frame_serialize(): writes hdr + payload(payload_len) + CRC16 (LE) into
 *   `out`; returns total frame length or a negative MESH_ERR_*.
 *   Caller's `out` capacity must be >= MAX_FRAME_LEN.
 *
 * mesh_frame_deserialize(): parses raw `in`/`in_len`; validates magic, version,
 *   length sanity and CRC-16; on success fills *out (host byte order, payload
 *   copied) and returns MESH_OK. The `link_from` argument is the one-hop sender
 *   (not necessarily the origin). PHY metrics (rssi,snr) are passed in so the
 *   decoded frame carries them upward to L3.
 *   Failure returns: MESH_ERR_RUNT / MESH_ERR_BAD_MAGIC / MESH_ERR_BAD_VERSION /
 *   MESH_ERR_CRC / MESH_ERR_FRAME_TOO_LONG.
 * --------------------------------------------------------------------------*/
int16_t  mesh_frame_serialize(const mesh_header_t *hdr,
                              const uint8_t *payload, uint8_t payload_len,
                              uint8_t *out, uint16_t out_cap);

mesh_err_t mesh_frame_deserialize(const uint8_t *in, uint16_t in_len,
                                  int16_t rssi_dbm, int8_t snr_db, uint32_t rx_millis,
                                  uint16_t link_from,
                                  mesh_rx_frame_t *out);

#endif /* MESH_FRAME_H */
