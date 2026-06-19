/* =============================================================================
 * mesh_frame.cpp  --  L2 frame codec: CRC-16/CCITT + (de)serialization
 * -----------------------------------------------------------------------------
 * Implements the on-wire frame format declared in mesh_frame.h. This file is
 * PURE LOGIC: no Arduino / RadioLib dependency, so it is host-testable.
 *
 * All multi-byte header fields and the CRC trailer are serialized field-by-field
 * through the HF_OFF_* offsets, LITTLE-ENDIAN (low byte first). We never memcpy
 * the mesh_header_t struct to the wire -- its padding is host-defined and
 * irrelevant to the format (see mesh_frame.h design notes).
 *
 * Frame on the wire:  [ HDR_LEN header ][ payload_len payload ][ CRC_LEN crc ]
 *   total = HDR_LEN + payload_len + CRC_LEN
 * CRC-16/CCITT-FALSE is computed over [0 .. HDR_LEN + payload_len) (header +
 * payload, excluding the CRC bytes themselves).
 * ===========================================================================*/

#include <string.h>          /* memcpy */
#include "mesh_frame.h"      /* contract: signatures, offsets, struct layouts  */
#include "mesh_config.h"     /* HDR_LEN, CRC_LEN, MAX_PAYLOAD, FRAME_MAGIC, ... */
#include "mesh_types.h"      /* mesh_err_t / MESH_* codes                       */

/* ----------------------------------------------------------------------------
 * Little-endian field accessors. Kept local (static) and tiny; the compiler
 * inlines these. They make the offset-driven (de)serialization read cleanly
 * and guarantee LE byte order regardless of host endianness.
 * --------------------------------------------------------------------------*/
static inline void put_u8(uint8_t *buf, uint16_t off, uint8_t v) {
    buf[off] = v;
}
static inline void put_u16le(uint8_t *buf, uint16_t off, uint16_t v) {
    buf[off]     = (uint8_t)(v & 0xFFu);          /* low byte first  */
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFu);   /* high byte next  */
}
static inline uint8_t get_u8(const uint8_t *buf, uint16_t off) {
    return buf[off];
}
static inline uint16_t get_u16le(const uint8_t *buf, uint16_t off) {
    return (uint16_t)((uint16_t)buf[off] |
                      ((uint16_t)buf[off + 1] << 8));
}

/* ----------------------------------------------------------------------------
 * mesh_crc16 -- CRC-16/CCITT-FALSE
 *   poly 0x1021, init 0xFFFF, no input/output reflection, no final XOR,
 *   MSB-first, processed byte by byte. Matches the L2 TX/RX contract.
 * --------------------------------------------------------------------------*/
uint16_t mesh_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFu;                 /* init */

    if (data == 0) {
        return crc;                         /* defensive: nothing to fold in   */
    }

    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;      /* bring in next byte at MSB        */
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 0x8000u) {            /* test top bit before shift        */
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;                             /* no final XOR */
}

/* ----------------------------------------------------------------------------
 * mesh_frame_serialize -- encode header + payload + CRC into `out`.
 *   Returns total frame length (HDR_LEN + payload_len + CRC_LEN) on success,
 *   or a negative MESH_ERR_* on failure. Caller's `out` should be >= MAX_FRAME_LEN.
 * --------------------------------------------------------------------------*/
int16_t mesh_frame_serialize(const mesh_header_t *hdr,
                             const uint8_t *payload, uint8_t payload_len,
                             uint8_t *out, uint16_t out_cap) {
    /* Null-argument validation. A NULL payload is only acceptable when there is
     * no payload to copy (payload_len == 0).                                   */
    if (hdr == 0 || out == 0) {
        return (int16_t)MESH_ERR_INVALID_ARG;
    }
    if (payload == 0 && payload_len > 0) {
        return (int16_t)MESH_ERR_INVALID_ARG;
    }

    /* Payload must fit the application cap. */
    if (payload_len > MAX_PAYLOAD) {
        return (int16_t)MESH_ERR_FRAME_TOO_LONG;
    }

    /* Total frame length and output-capacity check. */
    const uint16_t total = (uint16_t)(HDR_LEN + (uint16_t)payload_len + CRC_LEN);
    if (out_cap < total) {
        return (int16_t)MESH_ERR_FRAME_TOO_LONG;
    }

    /* --- Serialize header fields at their authoritative offsets (LE) -------- */
    put_u16le(out, HF_OFF_MAGIC,      FRAME_MAGIC);   /* always canonical magic */
    put_u8   (out, HF_OFF_VERSION,    hdr->version);
    put_u16le(out, HF_OFF_SRC,        hdr->src);
    put_u16le(out, HF_OFF_DST,        hdr->dst);
    put_u16le(out, HF_OFF_NEXTHOP,    hdr->next_hop);
    put_u16le(out, HF_OFF_LINKSRC,    hdr->link_src);
    put_u8   (out, HF_OFF_HOPCOUNT,   hdr->hop_count);
    put_u8   (out, HF_OFF_PKTTYPE,    hdr->pkt_type);
    put_u8   (out, HF_OFF_TTL,        hdr->ttl);
    put_u16le(out, HF_OFF_SEQ,        hdr->seq);
    put_u8   (out, HF_OFF_FLAGS,      hdr->flags);
    put_u8   (out, HF_OFF_RESERVED,   0u);            /* reserved MUST be 0      */
    put_u8   (out, HF_OFF_PAYLOADLEN, payload_len);

    /* --- Copy payload immediately after the header ------------------------- */
    if (payload_len > 0) {
        memcpy(out + HF_OFF_PAYLOAD, payload, payload_len);
    }

    /* --- Compute CRC over [0 .. HDR_LEN + payload_len) and append LE -------- */
    const uint16_t crc_span = (uint16_t)(HDR_LEN + (uint16_t)payload_len);
    const uint16_t crc = mesh_crc16(out, crc_span);
    put_u16le(out, crc_span, crc);                    /* trailer, low byte first */

    return (int16_t)total;
}

/* ----------------------------------------------------------------------------
 * mesh_frame_deserialize -- parse raw `in` into a decoded mesh_rx_frame_t.
 *   Validates length sanity, magic, version, and CRC-16. On success fills *out
 *   (host byte order, payload copied inline) plus the PHY link metrics passed
 *   in, and returns MESH_OK.
 *
 *   `link_from` is the one-hop sender (for L3's use). The mesh_rx_frame_t struct
 *   exposes no dedicated field for it (only hdr/payload/rssi_dbm/snr_db/
 *   rx_millis), so per the contract we do NOT invent a field -- link_from is
 *   accepted for interface stability and intentionally ignored here.
 * --------------------------------------------------------------------------*/
mesh_err_t mesh_frame_deserialize(const uint8_t *in, uint16_t in_len,
                                  int16_t rssi_dbm, int8_t snr_db,
                                  uint32_t rx_millis, uint16_t link_from,
                                  mesh_rx_frame_t *out) {
    (void)link_from;   /* no struct field for the one-hop sender; see note above */

    /* Null-argument validation. */
    if (in == 0 || out == 0) {
        return MESH_ERR_INVALID_ARG;
    }

    /* Must at least hold a full header plus the CRC trailer. */
    if (in_len < (uint16_t)(HDR_LEN + CRC_LEN)) {
        return MESH_ERR_RUNT;
    }

    /* Magic gate (LE). Cheapest rejection of foreign / corrupt traffic. */
    const uint16_t magic = get_u16le(in, HF_OFF_MAGIC);
    if (magic != (uint16_t)FRAME_MAGIC) {
        return MESH_ERR_BAD_MAGIC;
    }

    /* Version gate. */
    const uint8_t version = get_u8(in, HF_OFF_VERSION);
    if (version != (uint8_t)PROTO_VERSION) {
        return MESH_ERR_BAD_VERSION;
    }

    /* Declared payload length and the implied total frame length. */
    const uint8_t payload_len = get_u8(in, HF_OFF_PAYLOADLEN);
    if (payload_len > MAX_PAYLOAD) {
        return MESH_ERR_FRAME_TOO_LONG;
    }

    /* The buffer length must match HDR_LEN + payload_len + CRC_LEN exactly. A
     * mismatch means a truncated / over-long frame -> treat as runt.          */
    const uint16_t expected = (uint16_t)(HDR_LEN + (uint16_t)payload_len + CRC_LEN);
    if (in_len != expected) {
        return MESH_ERR_RUNT;
    }

    /* CRC-16 over [0 .. HDR_LEN + payload_len); compare to LE trailer. */
    const uint16_t crc_span = (uint16_t)(HDR_LEN + (uint16_t)payload_len);
    const uint16_t crc_calc = mesh_crc16(in, crc_span);
    const uint16_t crc_rx   = get_u16le(in, crc_span);
    if (crc_calc != crc_rx) {
        return MESH_ERR_CRC;
    }

    /* --- All checks passed: decode header into host-order struct ----------- */
    out->hdr.magic       = magic;
    out->hdr.version     = version;
    out->hdr.src         = get_u16le(in, HF_OFF_SRC);
    out->hdr.dst         = get_u16le(in, HF_OFF_DST);
    out->hdr.next_hop    = get_u16le(in, HF_OFF_NEXTHOP);
    out->hdr.link_src    = get_u16le(in, HF_OFF_LINKSRC);
    out->hdr.hop_count   = get_u8   (in, HF_OFF_HOPCOUNT);
    out->hdr.pkt_type    = get_u8   (in, HF_OFF_PKTTYPE);
    out->hdr.ttl         = get_u8   (in, HF_OFF_TTL);
    out->hdr.seq         = get_u16le(in, HF_OFF_SEQ);
    out->hdr.flags       = get_u8   (in, HF_OFF_FLAGS);
    out->hdr.reserved    = get_u8   (in, HF_OFF_RESERVED);
    out->hdr.payload_len = payload_len;

    /* --- Copy payload into the inline buffer (safe to queue upward) -------- */
    if (payload_len > 0) {
        memcpy(out->payload, in + HF_OFF_PAYLOAD, payload_len);
    }

    /* --- Stamp the PHY link metrics captured at RxDone --------------------- */
    out->rssi_dbm  = rssi_dbm;
    out->snr_db    = snr_db;
    out->rx_millis = rx_millis;

    return MESH_OK;
}
