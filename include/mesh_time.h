/* =============================================================================
 * mesh_time.h  --  millis()-based time helpers (wrap-safe).
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * millis() rolls over at ~49.7 days (2^32 ms). All elapsed-time comparisons
 * in the mesh use the helpers here to stay correct across rollover. Never
 * subtract two millis() values directly — use time_elapsed() instead.
 * ===========================================================================*/
#ifndef MESH_TIME_H
#define MESH_TIME_H

#include <stdint.h>
#include <stdbool.h>

/* Elapsed ms from `start` to `now` (wrap-safe, assumes delta < 2^31 ms).   */
static inline uint32_t time_elapsed(uint32_t start, uint32_t now) {
    return now - start;   /* unsigned subtraction is correct mod 2^32 */
}

/* True if `interval` ms has passed since `start` (as of `now`).            */
static inline bool time_after(uint32_t start, uint32_t now, uint32_t interval) {
    return time_elapsed(start, now) >= interval;
}

/* Returns the absolute deadline (millis()) for `interval` ms from `start`. */
static inline uint32_t deadline(uint32_t start, uint32_t interval) {
    return start + interval;
}

/* Remaining ms until `deadline_at` (as of `now`). Returns 0 if past.      */
static inline uint32_t time_remaining(uint32_t deadline_at, uint32_t now) {
    if (time_elapsed(now, deadline_at) > 0x7FFFFFFFu) return 0u; /* past */
    return deadline_at - now;
}

/* ms since the last time a task ran. Used by the scheduler.               */
static inline uint32_t task_age(uint32_t last_run, uint32_t now) {
    return time_elapsed(last_run, now);
}

#endif /* MESH_TIME_H */
