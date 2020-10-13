#include "linux/sched.h"
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/slab.h>

#include <litmus/rt_param.h>
#include <litmus/reservations/gtd-reservation.h>
#include <litmus/reservations/reservation.h>

static long gtd_reservation_init(unsigned int id,
				 struct gtd_reservation *gtdres,
				 lt_t major_cycle)
{
	memset(gtdres, 0, sizeof(*gtdres));
	INIT_LIST_HEAD(&gtdres->interval_sets);
	gtdres->id = id, gtdres->major_cycle = major_cycle;
	raw_spin_lock_init(&gtdres->lock);
	return 0;
}

// Return or create the interval set for the given criticality mode.
// Returning NULL means that there was a memory allocation error.
static struct gtd_interval_set *
find_or_create_interval_set(struct gtd_reservation *gtdres,
			    unsigned int criticality_mode)
{
	struct list_head *head = &gtdres->interval_sets;
	unsigned int i;
	for (i = 0; i <= criticality_mode; i++) {
		if (head->next == &gtdres->interval_sets) {
			struct gtd_interval_set *set;
			set = kzalloc(sizeof(struct gtd_interval_set),
				      GFP_KERNEL);
			if (set == NULL)
				return NULL;
			set->criticality_mode = i;
			INIT_LIST_HEAD(&set->intervals);
			list_add(&set->list, head);
			gtdres->criticality_level = i;
			head = &set->list;
		} else
			head = head->next;
	}
	return container_of(head, struct gtd_interval_set, list);
}

long gtd_reservation_add_interval(struct gtd_reservation *gtdres, lt_t start,
				  lt_t end, int cpu)
{
	struct gtd_interval_set *set;
	struct gtd_interval *interval, *new_interval;
	unsigned int criticality_mode;
	// Sanity checks
	if (start >= end) {
		TRACE("Interval start (%llu) must be smaller than interval end (%llu) (cpu %d)\n",
		      start, end, cpu);
		return -EINVAL;
	}
	if (end / gtdres->major_cycle != start / gtdres->major_cycle) {
		TRACE("Interval [%llu;%llu] (cpu %d) crosses a major cycle boundary (%llu)\n",
		      start, end, cpu, gtdres->major_cycle);
		return -EINVAL;
	}
	// If interval is added after we have a period for the reservation, check that
	// the new interval doesn't cross a period boundary
	if (gtdres->period && end / gtdres->period != start / gtdres->period) {
		TRACE("Interval [%llu;%llu] (cpu %d) crosses a period (%llu) boundary\n",
		      start, end, cpu, gtdres->period);
		return -EINVAL;
	}
	// Work in the right criticality mode and adjust start and end times.
	criticality_mode = start / gtdres->major_cycle;
	set = find_or_create_interval_set(gtdres, criticality_mode);
	if (set == NULL)
		return -ENOMEM;
	BUG_ON(set->criticality_mode != criticality_mode);
	start -= criticality_mode * gtdres->major_cycle;
	end -= criticality_mode * gtdres->major_cycle;
	// Find the interval before which the reservation must be inserted
	new_interval = kzalloc(sizeof(struct gtd_interval), GFP_KERNEL);
	if (new_interval == 0)
		return -ENOMEM;
	new_interval->start = start;
	new_interval->end = end;
	new_interval->cpu = cpu;
	TRACE("Reservation %d: adding interval [%d-%d) (cpu %d) (criticality mode %u)\n",
	      gtdres->id, start, end, cpu, criticality_mode);
	list_for_each_entry (interval, &set->intervals, list) {
		// If the interval starts after the new one, this is the place to insert the later
		if (interval->start > start) {
			// Ensure that the new interval terminates no later than the next one
			if (end > interval->start) {
				TRACE("Intervals [%llu;%llu] (cpu %d) "
				      "and [%llu;%llu] (cpu %d) overlap (criticality mode %u)\n",
				      interval->start, interval->end,
				      interval->cpu, start, end, cpu,
				      criticality_mode);
				goto error_invalid;
			}
			// Insert before the found interval
			list_add(&new_interval->list, interval->list.prev);
			return 0;
		}
	}
	// The interval must go to the end of the list. Before that, check that the last element if
	// it exists terminates no later than the start of the new interval. Also, the new one
	// becomes the current end of the major cycle.
	if (!list_empty(&set->intervals)) {
		interval = list_last_entry(&set->intervals, struct gtd_interval,
					   list);
		if (interval->end > start) {
			TRACE("Intervals [%llu;%llu] (cpu %d) and [%llu;%llu] (cpu %d) overlap "
			      "(criticality mode %u)\n",
			      interval->start, interval->end, interval->cpu,
			      start, end, cpu, criticality_mode);
			goto error_invalid;
		}
		BUG_ON(!interval->terminates_major_cycle);
		interval->terminates_major_cycle = false;
	}
	new_interval->terminates_major_cycle = true;
	list_add_tail(&new_interval->list, &set->intervals);
	return 0;
error_invalid:
	// Note that there is no point in clearing a newly created intervals set because its space
	// will be reclaimed when the while reservation is.
	kfree(new_interval);
	return -EINVAL;
}

void gtd_reservation_clear(struct gtd_reservation *gtdres)
{
	struct gtd_interval_set *set, *ps;
	list_for_each_entry_safe (set, ps, &gtdres->interval_sets, list) {
		struct gtd_interval *interval, *pi;
		list_for_each_entry_safe (interval, pi, &set->intervals, list) {
			list_del(&interval->list);
			kfree(interval);
		}
		list_del(&set->list);
		kfree(set);
	}
}

long gtd_reservation_mark_end_of_periods(struct gtd_reservation *gtdres,
					 lt_t period)
{
	struct gtd_interval_set *set;
	struct gtd_interval *interval;
	lt_t prev_period;
	unsigned int criticality_mode = 0;

	list_for_each_entry (set, &gtdres->interval_sets, list) {
		BUG_ON(set->criticality_mode != criticality_mode);
		prev_period = gtdres->major_cycle;
		list_for_each_entry_reverse (interval, &set->intervals, list) {
			if (period) {
				lt_t start_period = interval->start / period;
				lt_t end_period = interval->end / period;
				if (start_period != end_period) {
					TRACE("Reservation %u: interval [%llu-%llu] (on cpu %u) "
					      "crosses a period (%llu) boundary "
					      "at criticality mode %u\n",
					      gtdres->id, interval->start,
					      interval->end, interval->cpu,
					      period, criticality_mode);
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
		criticality_mode++;
	}
	BUG_ON(criticality_mode != gtdres->criticality_level + 1);
	return 0;
error_no_interval:
	TRACE("Reservation %u: period [%llu-%llu) at criticality mode %u "
	      "contains no interval for task execution\n",
	      gtdres->id, period * (prev_period - 1), period * prev_period,
	      criticality_mode);
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
		if (res->id == id)
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
		if (res->id == config->id) {
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
	gtd_reservation_init(config->id, res, major_cycle);

	// Add it at the end of the reservations list
	list_add_tail(&res->all_reservations_list, &gtdenv->all_reservations);
	*gtdres = res;
	return 0;
}

bool gtd_reservation_find_interval(struct gtd_reservation *gtdres,
				   unsigned int criticality_mode, lt_t time,
				   struct gtd_interval **gtdinterval,
				   lt_t *major_cycle_start)
{
	struct gtd_interval *interval;
	struct gtd_interval_set *set;
	lt_t next_interval_delay = ~0,
	     res_major_cycle_start = time - (time % gtdres->major_cycle);

	*gtdinterval = NULL;
	*major_cycle_start = GTDRES_TIME_NA;

	// If the criticality mode is above the criticality level, we will
	// never find an interval.
	if (criticality_mode > gtdres->criticality_level)
		return false;

	// This will necessarily find the set, which must exist (unless the
	// reservation has been created empty, which should not be possible)
	set = find_or_create_interval_set(gtdres, criticality_mode);
	BUG_ON(!set || set->criticality_mode != criticality_mode);

	list_for_each_entry (interval, &set->intervals, list) {
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
		if (start <= time || start - time < next_interval_delay) {
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
	return false;
}

struct gtd_interval *
gtd_reservation_next_interval(const struct gtd_reservation *gtdres,
			      const struct gtd_interval *gtdinterval,
			      lt_t *major_cycle_start)
{
	const struct list_head *next_interval = gtdinterval->list.next;
	if (gtdinterval->terminates_major_cycle) {
		next_interval = next_interval->next;
		*major_cycle_start += gtdres->major_cycle;
	}
	return container_of(next_interval, struct gtd_interval, list);
}
