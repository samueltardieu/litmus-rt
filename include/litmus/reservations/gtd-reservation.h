#ifndef LITMUS_GTD_RESERVATION_H
#define LITMUS_GTD_RESERVATION_H

#include "linux/sched.h"
#include "linux/spinlock_types.h"
#include <linux/list.h>
#include <linux/types.h>
#include <litmus/rt_param.h>
#include <litmus/reservations/reservation.h>
#include <litmus/reservations/table-driven.h>

struct gtd_reservation {
	unsigned int id;
	lt_t major_cycle;
	lt_t period;
	unsigned int criticality_level;

	// All intervals, starting at criticality mode 0
	struct list_head interval_sets;

	// For inclusion in the list of all reservations
	struct list_head all_reservations_list;

	// The lock protects entries below
	raw_spinlock_t lock;

	// The task bound to the reservation if any. This can be modified while holding the
	// environment writer_lock, and should be read in a RCU protected section only.
	struct task_struct *task;
};

struct gtd_interval_set {
	unsigned int criticality_mode;
	struct list_head intervals;

	// For inclusion in the reservation interval_sets
	struct list_head list;
};

// An interval relative to the major cycle frame
struct gtd_interval {
	struct list_head list;
	lt_t start;
	lt_t end;
	int cpu;
	bool terminates_period;
	bool terminates_major_cycle;
};

// Time not available (for example, when no interval is defined)
#define GTDRES_TIME_NA ((lt_t)~0)

// Add an interval to an existing global reservation. Locking must be used to ensure that no
// two threads can add intervals at the same time. The interval will be placed in the right
// criticality mode depending on its start and end times, and will start a new criticality mode
// if needed.
long gtd_reservation_add_interval(struct gtd_reservation *gtdres, lt_t start,
				  lt_t end, int cpu);

// Clear the dynamically allocated structures inside the global reservation. This must only happen
// when no task can reference this reservation ever.
void gtd_reservation_clear(struct gtd_reservation *gtdres);

// Mark intervals to identify the ones terminating a given period. If period is 0,
// all intervals are cleared. Return an error if an interval happens to span over
// a period boundary or if a period contains no intervals.
long gtd_reservation_mark_end_of_periods(struct gtd_reservation *gtdres,
					 lt_t period);

// Dump the current state of a reservation into Litmus trace device for debugging purpose
void gtd_reservation_dump(struct gtd_reservation *gtdres);

// Find the interval englobing time or following time (with wraparound) wich
// matches the filter. A NULL filter matches everything. Set gtdinterval
// and major_cycle_start to the right values.
// Return true if an interval containing time is found, false if an interval
// following is time is found or if no interval matches. In the later case,
// gtdinterval is set to NULL and major_cycle_start to GTDRES_TIME_NA.
bool gtd_reservation_find_interval(struct gtd_reservation *gtdres,
				   unsigned int criticality_mode, lt_t time,
				   struct gtd_interval **gtdinterval,
				   lt_t *major_cycle_start);

// Return the next interval in the reservation. major_cycle_start is incremented
// by the major cycle if the intervals wrap around.
struct gtd_interval *
gtd_reservation_next_interval(const struct gtd_reservation *gtdres,
			      const struct gtd_interval *gtdinterval,
			      lt_t *major_cycle_start);

// Environment for holding known reservations
struct gtd_env {
	// Global writer lock that must be held when creating a reservation or adding intervals to
	// an existing reservation
	raw_spinlock_t writer_lock;

	// All reservations
	struct list_head all_reservations;
};

void gtd_env_init(struct gtd_env *gtdenv);

// Find a reservation with the given id, or return NULL if none exist.
struct gtd_reservation *gtd_env_find(struct gtd_env *gtdenv, unsigned int id);

// Find or create a reservation with the given id. Write locking must be used to ensure that no two
// threads are creating the reservation simultaneously. This function may fail if the memory
// allocation to create the reservation fails or if the configuration major cycle doesn't correspond
// to the existing major cycle.
long gtd_env_find_or_create(struct gtd_env *gtdenv,
			    struct reservation_config *config,
			    struct gtd_reservation **gtdres);

// Compute the maximum criticality level of reservations bound to tasks. This function will in
// turn lock all reservations, so no reservation lock must be held while calling it.
unsigned int gtd_env_maximum_task_criticality_level(struct gtd_env *gtdenv);

#endif // LITMUS_GTD_RESERVATION_H
