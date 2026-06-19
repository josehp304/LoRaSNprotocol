/* =============================================================================
 * node_identity.cpp  --  Per-node identity accessors (implementation).
 * -----------------------------------------------------------------------------
 * Implements the contract in node_identity.h. In this design phase the four
 * identity getters are thin wrappers over the compile-time #defines from
 * mesh_config.h (NODE_ID / NODE_ROLE / NODE_REGION / NODE_SINK_ADDR). Routing
 * everything through functions keeps the door open for a future phase that loads
 * identity from flash/NVM without touching call sites or layer code.
 *
 * node_next_seq() owns the one and only mutable piece of identity state: the
 * per-node packet sequence counter. Keeping it here guarantees every layer that
 * needs a fresh seq goes through a single call site, so (origin, seq) pairs stay
 * unique across L2/L3/L4.
 *
 * node_is_always_rx() is intentionally NOT defined here — it is a static inline
 * in the header.
 * ===========================================================================*/
#include "node_identity.h"
#include "mesh_config.h"
#include "mesh_types.h"

/* -----------------------------------------------------------------------------
 * Identity getters — compile-time wrappers.
 * Each returns its mesh_config.h #define cast to the declared return type so the
 * value is well-typed regardless of how the macro literal is spelled.
 * ---------------------------------------------------------------------------*/

/* This node's unique 16-bit L3 address (see header: never INVALID/BROADCAST). */
node_id_t node_id(void) {
    return (node_id_t)NODE_ID;
}

/* This node's role: ROLE_SENDER | ROLE_RELAY | ROLE_SINK. */
uint8_t node_role(void) {
    return (uint8_t)NODE_ROLE;
}

/* This node's regulatory region: REGION_EU433 | REGION_US433. */
uint8_t node_region(void) {
    return (uint8_t)NODE_REGION;
}

/* The default sink/gateway address DATA is sent to. */
node_id_t node_sink_addr(void) {
    return (node_id_t)NODE_SINK_ADDR;
}

/* -----------------------------------------------------------------------------
 * Sequence-number generator.
 *
 * File-scope counter holding the LAST sequence number that was handed out. We
 * pre-increment and return, so the very first call returns 1 (the counter
 * starts at 0 and 0 is never used as a live seq). Choosing to skip 0 keeps it
 * available as a sentinel for "no seq yet" in tables that initialise to zero.
 *
 * seq_t is 16-bit, so the pre-increment wraps mod 2^16 naturally (0xFFFF -> 0).
 * Note that wrap means 0 can reappear after the first full cycle; that is fine
 * because seq comparisons use the wrap-safe seq_newer() helper, not equality to
 * a fixed sentinel.
 *
 * Core0-only / not thread-safe, per the header contract: the read-modify-write
 * below is deliberately unguarded.
 * ---------------------------------------------------------------------------*/
static seq_t s_my_seq = 0;

seq_t node_next_seq(void) {
    /* Pre-increment: first returned value is 1, then 2, 3, ... wrapping at 2^16. */
    return ++s_my_seq;
}
