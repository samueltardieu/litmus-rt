#include "linux/sched.h"
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/slab.h>

#include <litmus/rt_param.h>
#include <litmus/reservations/gtd-reservation.h>
#include <litmus/reservations/reservation.h>

long gtd_reservation_init(struct gtd_reservation *gtdres, lt_t major_cycle)
{
	memset(gtdres, 0, sizeof(*gtdres));
	INIT_LIST_HEAD(&gtdres->intervals);
	gtdres->major_cycle = major_cycle;
	raw_spin_lock_init(&gtdres->lock);
	reservation_init(&gtdres->res);
	return 0;
}

long gtd_reservation_add_interval(struct gtd_reservation *gtdres, lt_t start,
				  lt_t end, int cpu)
{
	struct gtd_interval *interval, *new_interval;
	// Sanity checks
	if (start >= end) {
		TRACE("Interval start (%llu) must be smaller than interval end (%llu) (cpu %d)\n",
		      start, end, cpu);
		return -EINVAL;
	}
	if (end > gtdres->major_cycle) {
		TRACE("Interval [%llu;%llu] (cpu %d) ends after major cycle end (%llu)\n",
		      start, end, cpu, gtdres->major_cycle);
		return -EINVAL;
	}
	// If interval is added after we have a period for the reservation, check that
	// the new interval doesn't cross a period boundary
	if (gtdres->period && end / gtdres->period != start / gtdres->period &&
	    (end / gtdres->period != start / gtdres->period + 1 ||
	     end / gtdres->period != 0)) {
		TRACE("Interval [%llu;%llu] (cpu %d) crosses a period (%llu) boundary\n",
		      start, end, cpu, gtdres->period);
		return -EINVAL;
	}
	// Find the interval before which the reservation must be inserted
	new_interval = kzalloc(sizeof(struct gtd_interval), GFP_KERNEL);
	if (new_interval == 0)
		return -ENOMEM;
	new_interval->start = start;
	new_interval->end = end;
	new_interval->cpu = cpu;
	list_for_each_entry (interval, &gtdres->intervals, list) {
		// If the interval starts after the new one, this is the place to insert the later
		if (interval->start > start) {
			// Ensure that the new interval terminates no later than the next one
			if (end > interval->start) {
				TRACE("Intervals [%llu;%llu] (cpu %d) "
				      "and [%llu;%llu] (cpu %d) overlap\n",
				      interval->start, interval->end,
				      interval->cpu, start, end, cpu);
				goto error_invalid;
			}
			// Insert before the found interval
			list_add(&new_interval->list, interval->list.prev);
			return 0;
		}
	}
	// The interval must go to the end of the list. Before that, check that the last element if
	// it exists terminates no later than the start of the new interval.
	if (!list_empty(&gtdres->intervals)) {
		interval = list_last_entry(&gtdres->intervals,
					   struct gtd_interval, list);
		if (interval->end > start) {
			TRACE("Intervals [%llu;%llu] (cpu %d) and [%llu;%llu] (cpu %d) overlap\n",
			      interval->start, interval->end, interval->cpu,
			      start, end, cpu);
			goto error_invalid;
		}
	}
	list_add_tail(&new_interval->list, &gtdres->intervals);
	return 0;
error_invalid:
	kfree(new_interval);
	return -EINVAL;
}

void gtd_reservation_clear(struct gtd_reservation *gtdres)
{
	struct gtd_interval *interval, *p;
	list_for_each_entry_safe (interval, p, &gtdres->intervals, list) {
		list_del(&interval->list);
		kfree(interval);
	}
}

long gtd_reservation_mark_end_of_periods(struct gtd_reservation *gtdres,
					 lt_t period)
{
	struct gtd_interval *interval;
	lt_t prev_period = gtdres->major_cycle;
	list_for_each_entry_reverse (interval, &gtdres->intervals, list) {
		if (period) {
			lt_t start_period = interval->start / period;
			lt_t end_period = interval->end / period;
			if (start_period != end_period) {
				TRACE("Reservation %u: interval [%llu-%llu] (on cpu %u) crosses"
				      "a period (%llu) boundary\n",
				      gtdres->res.id, interval->start,
				      interval->end, interval->cpu, period);
				return -EINVAL;
			}
			if (start_period == prev_period)
				interval->terminates_period = true;
			else if (start_period + 1 == prev_period) {
				interval->terminates_period = false;
				prev_period = start_period;
			} else
				goto error_no_interval;
			interval->terminates_period =
				start_period != prev_period;
			prev_period = start_period;
		} else {
			interval->terminates_period = false;
		}
	}
	if (prev_period != 0)
		goto error_no_interval;
	return 0;
error_no_interval:
	TRACE("Reservation %u: period [%llu-%llu) contains no interval for task execution\n",
	      gtdres->res.id, period * (prev_period - 1), period * prev_period);
	return -EINVAL;
}

void gtd_env_init(struct gtd_env *gtdenv)
{
	INIT_LIST_HEAD(&gtdenv->all_reservations);
	raw_spin_lock_init(&gtdenv->writer_lock);
}

struct gtd_reservation *gtd_env_find(struct gtd_env *gtdenv, unsigned int id)
{
	struct gtd_reservation *res;
	list_for_each_entry (res, &gtdenv->all_reservations,
			     all_reservations_list)
		if (res->res.id == id)
			return res;
	return NULL;
}

long gtd_env_find_or_create(struct gtd_env *gtdenv,
			    struct reservation_config *config,
			    struct gtd_reservation **gtdres)
{
	struct gtd_reservation *res;
	lt_t major_cycle = config->table_driven_params.major_cycle_length;
	*gtdres = NULL;
	list_for_each_entry (res, &gtdenv->all_reservations,
			     all_reservations_list)
		if (res->res.id == config->id) {
			if (res->major_cycle !=
			    config->table_driven_params.major_cycle_length) {
				TRACE("Reservation %u major cycle period mismatch (%llu != %llu)\n",
				      config->id, res->major_cycle,
				      major_cycle);
				return -EINVAL;
			}
			*gtdres = res;
			return 0;
		}

	// Create a new reservation with the right parameters
	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (res == NULL)
		return -ENOMEM;
	gtd_reservation_init(res, major_cycle);
	reservation_init(&res->res);
	res->res.id = config->id;

	// Add it at the end of the reservations list
	list_add_tail(&res->all_reservations_list, &gtdenv->all_reservations);
	*gtdres = res;
	return 0;
}

bool gtd_reservation_find_interval(struct gtd_reservation *gtdres, lt_t time,
				   struct gtd_interval **gtdinterval,
				   lt_t *major_cycle_start,
				   gtd_interval_filter_t filter, void *opaque)
{
	lt_t next_interval_delay = ~0;
	struct gtd_interval *interval;
	lt_t res_major_cycle_start = time - (time % gtdres->major_cycle);
	*gtdinterval = NULL;
	*major_cycle_start = GTDRES_TIME_NA;
	list_for_each_entry (interval, &gtdres->intervals, list) {
		if (!filter || filter(interval, opaque)) {
			lt_t start = res_major_cycle_start + interval->start;
			lt_t end = res_major_cycle_start + interval->end;
			// If the interval has already terminated in the current major
			// cycle, adjust the times to those of the next execution.
			if (end < time) {
				start += gtdres->major_cycle;
				end += gtdres->major_cycle;
			}
			// If time is within the interval or it is the next one (that we
			// know so far) that we are going to see happening with this filter,
			// record it.
			if (start <= time ||
			    start - time < next_interval_delay) {
				*gtdinterval = interval;
				*major_cycle_start = start - interval->start;
				// If time is within the interval, return an immediate
				// match.
				if (start <= time)
					return true;
				next_interval_delay = start - time;
			}
			// If we are already past the current time in the list, we cannot
			// find a better match since we know we have the next interval already.
			if (res_major_cycle_start + interval->start > time)
				break;
		}
	}
	return false;
}

static bool is_right_cpu(struct gtd_interval *interval, void *cpu)
{
	return interval->cpu == *(int *)cpu;
}

bool gtd_env_find_interval(struct gtd_env *gtdenv, lt_t time, int cpu,
			   struct gtd_reservation **gtdres,
			   struct gtd_interval **gtdinterval,
			   lt_t *major_cycle_start)
{
	struct gtd_reservation *res;
	lt_t next_interval_delay = ~0;
	*gtdres = NULL;
	*gtdinterval = NULL;
	*major_cycle_start = GTDRES_TIME_NA;
	list_for_each_entry (res, &gtdenv->all_reservations,
			     all_reservations_list) {
		struct gtd_interval *interval;
		lt_t mcs;
		bool is_inside_interval = gtd_reservation_find_interval(
			res, time, &interval, &mcs, is_right_cpu, &cpu);
		if (interval &&
		    (is_inside_interval ||
		     interval->start - time < next_interval_delay)) {
			*gtdres = res;
			*gtdinterval = interval;
			*major_cycle_start = mcs;
			if (is_inside_interval)
				return true;
			next_interval_delay = interval->start - time;
		}
	}
	return false;
}
