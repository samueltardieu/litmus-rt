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
	// Extend basic reservation which serves as reference
	struct reservation res;

	lt_t major_cycle;
	lt_t period;
	struct list_head intervals;

	// For inclusion in the list of all reservations
	struct list_head all_reservations_list;

	// The lock protects entries below
	raw_spinlock_t lock;

	// The task bound to the reservation if any. This can be modified while holding the
	// environment writer_lock, and should be read in a RCU protected section only.
	struct task_struct *task;
};

struct gtd_interval {
	struct list_head list;
	lt_t start;
	lt_t end;
	int cpu;
	bool terminates_period;
};

// Time not available (for example, when no interval is defined)
#define GTDRES_TIME_NA ((lt_t)~0)

// Initialize the global reservation. The res field is initialized with its default
// value.
long gtd_reservation_init(struct gtd_reservation *gtdres, lt_t major_cycle);

// Add an interval to an existing global reservation. Locking must be used to ensure that no
// two threads can add intervals at the same time.
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

typedef bool (*gtd_interval_filter_t)(struct gtd_interval *gtdinterval,
				      void *opaque);

// Find the interval englobing time or following time (with wraparound) wich
// matches the filter. A NULL filter matches everything. Set gtdinterval
// and major_cycle_start to the right values.
// Return true if an interval containing time is found, false if an interval
// following is time is found or if no interval matches. In the later case,
// gtdinterval is set to NULL and major_cycle_start to GTDRES_TIME_NA.
bool gtd_reservation_find_interval(struct gtd_reservation *gtdres, lt_t time,
				   struct gtd_interval **gtdinterval,
				   lt_t *major_cycle_start,
				   gtd_interval_filter_t filter, void *opaque);

// Environment for holding known reservations
struct gtd_env {
	// All reservations
	struct list_head all_reservations;

	// Global writer lock that must be held when creating a reservation or adding intervals to
	// an existing reservation
	raw_spinlock_t writer_lock;
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

// Find the current or next interval for time on cpu and fill the gtdres pointer. It also
// also set gtdinterval to the corresponding interval, and major_cycle_start to the date
// of the major cycle to add to the interval.
// Return true if the interval includes time, or false if it is in the future. In the later case,
// gtdres and gtdinterval will be NULL and major_cycle_start equal to GTDRES_TIME_NA if we do not
// have any interval planned in the future.
bool gtd_env_find_interval(struct gtd_env *gtdenv, lt_t time, int cpu,
			   struct gtd_reservation **gtdres,
			   struct gtd_interval **gtdinterval,
			   lt_t *major_cycle_start);

#endif // LITMUS_GTD_RESERVATION_H
