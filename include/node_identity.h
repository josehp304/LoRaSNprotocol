/* =============================================================================
 * node_identity.h  --  Per-node identity accessors.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * Wraps the compile-time NODE_ID, NODE_ROLE, NODE_REGION, NODE_SINK_ADDR from
 * mesh_config.h as functions so they can be stubbed/mocked in host tests and
 * potentially overridden at runtime (e.g., loaded from flash/NVM in a future
 * phase). In the design phase all implementations are thin wrappers over the
 * #defines.
 *
 * Also provides node_next_seq() — the per-node per-packet sequence number
 * generator. This is the only place mySeq is incremented, ensuring all layers
 * produce unique (origin, seq) pairs through a single call site.
 * ===========================================================================*/
#ifndef NODE_IDENTITY_H
#define NODE_IDENTITY_H

#include "mesh_config.h"
#include "mesh_types.h"

/* Returns this node's unique 16-bit L3 address. Must not be ADDR_INVALID or
 * ADDR_BROADCAST. Set at compile time via -D NODE_ID=0xNNNN in platformio.ini. */
node_id_t node_id(void);

/* Returns this node's role (ROLE_SENDER, ROLE_RELAY, or ROLE_SINK).
 * Role governs whether the radio may sleep (SENDER only) and whether the
 * node participates in forwarding (RELAY/SINK always forward). */
uint8_t node_role(void);

/* Returns this node's regulatory region (REGION_EU433 or REGION_US433).
 * Determines TX power ceiling and duty-cycle budget. */
uint8_t node_region(void);

/* Returns the sink/gateway node ID (the node DATA is addressed to by default).
 * SENDER nodes send all telemetry here unless overridden by L4. */
node_id_t node_sink_addr(void);

/* Generate and return the next per-packet sequence number.
 * Pre-increments a static counter and returns the NEW value (first call
 * returns 1; 0 is reserved as a "no seq yet" sentinel). Wraps mod 2^16.
 * The returned value is used directly as the packet's seq. Thread-unsafe
 * (core0 only). All freshness comparisons use seq_newer() (RFC-1982), so a
 * wrapped seq of 0 is treated like any other value — never special-cased. */
seq_t node_next_seq(void);

/* Convenience: returns true if this node should never sleep its radio
 * (i.e., RELAY or SINK role). */
static inline bool node_is_always_rx(void) {
    return node_role() != ROLE_SENDER;
}

#endif /* NODE_IDENTITY_H */
