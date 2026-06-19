/* =============================================================================
 * scheduler.h  --  Cooperative millis()-based task table.
 * -----------------------------------------------------------------------------
 * Phase: DESIGN ONLY.
 *
 * A dead-simple, non-preemptive scheduler: a fixed array of tasks, each with
 * a period, last-run timestamp, and optional jitter. `sched_run()` is called
 * once per loop() and fires any task whose deadline has passed.
 *
 * No dynamic allocation, no RTOS, no priorities — tasks should be short. If
 * a task takes longer than its period the next fire is simply late; there is
 * no catch-up logic (appropriate for a LoRa mesh where task periods >> task
 * durations).
 *
 * Intended task list:
 *   idx 0  TASK_BEACON     — emit proactive BEACON (mesh_l3_tick does this)
 *   idx 1  TASK_L3_MAINT   — mesh_l3_tick() for RREQ timers, route expiry, etc.
 *   idx 2  TASK_APP_SAMPLE — app_tick() for sensor sampling (SENDER role only)
 *   idx 3  TASK_POWER      — power_mgr_step() (also called from node_loop directly)
 *   Additional tasks may be added in later phases.
 * ===========================================================================*/
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_types.h"
#include "mesh_time.h"

/* ── Task descriptor ─────────────────────────────────────────────────────── */
typedef void (*task_fn_t)(uint32_t now_ms);

typedef struct {
    const char *name;        /* debug label (stored in flash if const)        */
    task_fn_t   fn;          /* function to call when due                     */
    uint32_t    period_ms;   /* how often to run (0 = disabled)               */
    uint32_t    last_run_ms; /* millis() of the most recent execution         */
    uint16_t    jitter_ms;   /* random 0..jitter_ms added to each deadline    */
    bool        enabled;     /* false = skip without resetting last_run       */
} sched_task_t;

/* ── Scheduler table ─────────────────────────────────────────────────────── */
typedef struct {
    sched_task_t *tasks;   /* pointer to a static array of sched_task_t      */
    uint8_t       count;   /* number of entries in tasks[]                   */
} sched_table_t;

/* ── Operations ──────────────────────────────────────────────────────────── */

/* Initialize the scheduler. Sets last_run_ms to `now_ms` for each enabled
 * task so they all fire approximately one period from init, not immediately. */
void sched_init(sched_table_t *t, uint32_t now_ms);

/* Run one scheduler pass (call once per loop()). Fires every task whose
 * period + jitter has elapsed since its last run. Tasks that are disabled
 * or have period==0 are skipped. */
void sched_run(sched_table_t *t, uint32_t now_ms);

/* Enable or disable a task by index (0-based).                             */
void sched_set_enabled(sched_table_t *t, uint8_t idx, bool enabled);

/* Return the ms until the next task is due (across all enabled tasks).
 * Used by power_mgr_step() to decide how long it is safe to sleep.
 * Returns 0 if any task is already overdue, UINT32_MAX if all disabled.   */
uint32_t sched_next_deadline(const sched_table_t *t, uint32_t now_ms);

/* Force an immediate re-fire of task[idx] on the next sched_run() call.
 * Used to e.g. trigger an emergency BEACON after a link-break event.      */
void sched_trigger_now(sched_table_t *t, uint8_t idx, uint32_t now_ms);

#endif /* SCHEDULER_H */
