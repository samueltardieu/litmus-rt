#include "asm/atomic.h"
#include "asm/smp.h"
#include "linux/completion.h"
#include "linux/cpumask.h"
#include "linux/key.h"
#include "linux/list.h"
#include "linux/time.h"
#include "linux/types.h"
#include "litmus/rt_param.h"
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include <litmus/debug_trace.h>
#include <litmus/jobs.h>
#include <litmus/litmus.h>
#include <litmus/litmus_proc.h>
#include <litmus/preempt.h>
#include <litmus/reservations/gtd-reservation.h>
#include <litmus/sched_plugin.h>

// The smallest acceptable major cycle in lt_t units (nanoseconds)
// 1000ns = 1us
#define MIN_MAJOR_CYCLE 1000

// The number of bits on which the criticality mode is stored
// 2^10 = 1024 criticality levels
#define CRIT_MODE_BITS 10

// Define TIMING_TRACES to generate additional timing traces in order
// to analyze performances. Some accesses to lt_t fields may be done outside
// proper locks, but this only affects tracing.
#define TIMING_TRACES

static struct gtd_env gtdenv;

struct gmcres_cpu_state {
	raw_spinlock_t lock;

	// The current CPU number, for easy reference
	int cpu;

	// True after scheduling if the task we have descheduled last was a real-time
	// task (i.e. if the task may be implicated in a deadlock between Litmus tasks).
	bool prev_was_realtime;

	// Task being current scheduled
	struct task_struct *scheduled;

	// Task interested in being scheduled
	struct task_struct *linked;

#ifdef TIMING_TRACES
	// Local reschedule request on the same CPU, or 0 if none
	lt_t reschedule_date;

	// With migration reschedule request on this CPU, or 0 if none
	lt_t migration_reschedule_date;
#endif // TIMING_TRACES
};

static DEFINE_PER_CPU(struct gmcres_cpu_state, gmcres_cpu_state);

#define cpu_state_for(cpu_id) (&per_cpu(gmcres_cpu_state, cpu_id))
#define local_cpu_state() (this_cpu_ptr(&gmcres_cpu_state))

struct gmcres_task_state {
	raw_spinlock_t lock;

	// The reservation associated to this task, cannot be NULL once the task
	// has been admitted, and contains the reverse link to the struct task_struct * entry.
	struct gtd_reservation *gtdres;

	// The current or next interval if outside an interval. NULL means that the task
	// is being removed from the gmcres scheduler alltogether, or that the task is waiting
	// for a criticality mode change to happen. In this case, the event_timer is not armed
	// either.
	struct gtd_interval *gtdinterval;

	// The task major cycle start for the interval defined above.
	lt_t major_cycle_start;

	// The criticality mode corresponding to the latest non-NULL interval defined above.
	unsigned criticality_mode;

	// The timer associated with task events, such as the beginning or end
	// of an interval, or a task release time. It can only be armed if an
	// interval is defined.
	struct hrtimer event_timer;
};

// The current criticality mode and major cycle count:
//   - The lowest CRIT_MODE_BITS bits contain the last set criticality mode
//   - The highest bits contain the major cycle count at which this mode was
//     set (generation)
static atomic64_t system_criticality_state;

// The number of mode changes in progress
static atomic_t access_counter;

// The maximum criticality level of any existing task currently admitted
// by the plugin.
static atomic_t maximum_criticality_level;

// The current major cycle of executing tasks. Never zero, but only meaningful
// when at least one task has entered the system.
static lt_t system_major_cycle;

#ifdef TIMING_TRACES
static lt_t mode_change_event_date;
static atomic_t mode_change_rescheduling_cpus;
#endif // TIMING_TRACES

// Return the criticality mode contained in a criticality state without concern
// for the generation.
static unsigned int criticality_mode(uint64_t criticality_state)
{
	return criticality_state & ((1 << CRIT_MODE_BITS) - 1);
}

// Return the generation of a criticality state
static uint64_t generation(uint64_t criticality_state)
{
	return criticality_state >> CRIT_MODE_BITS;
}

// Return the generation at a given date
static uint64_t generation_at(lt_t date)
{
	return date / system_major_cycle;
}

// Return the criticality mode contained in a criticality state at a given date.
static unsigned int criticality_mode_at(uint64_t criticality_state, lt_t date)
{
	return generation(criticality_state) == generation_at(date) ?
			     criticality_mode(criticality_state) :
			     0;
}

static unsigned int system_criticality_mode(void)
{
	return criticality_mode_at(
		(uint64_t)atomic64_read(&system_criticality_state),
		litmus_clock());
}

static uint64_t criticality_state(lt_t date, unsigned int criticality_mode)
{
	TRACE_WARN_ON(criticality_mode >= 1 << CRIT_MODE_BITS);
	return ((date / system_major_cycle) << CRIT_MODE_BITS) |
	       criticality_mode;
}

static lt_t major_cycle_start(lt_t at_time)
{
	return at_time - (at_time % system_major_cycle);
}

static struct gmcres_task_state *get_gmcres_task_state(struct task_struct *tsk)
{
	return tsk_rt(tsk)->plugin_state;
}

// For debugging: add a trace to check if we are in an IRQ
#define CHECK_IRQ_CONTEXT()                                                                \
	do {                                                                               \
		TRACE("%s: in_irq:%u, in_softirq:%u, in_interrupt:%u, irqs_disabled:%u\n", \
		      __FUNCTION__, in_irq(), in_softirq(), in_interrupt(),                \
		      irqs_disabled());                                                    \
	} while (0)

// Enhance TRACE and TRACE_TASK with timing and reservation information
#undef TRACE
#define TRACE(fmt, args...)                                                    \
	LITMUS_TRACE("(now:%llu - %s) " fmt, litmus_clock(), __FUNCTION__,     \
		     ##args)
#undef TRACE_TASK
#define TRACE_TASK(t, fmt, args...)                                            \
	LITMUS_TRACE_TASK(t, "(now:%llu - res:%u - %s) " fmt, litmus_clock(),  \
			  get_gmcres_task_state(t)->gtdres->id, __FUNCTION__,  \
			  ##args)

static bool
ensure_minimum_criticality_state(uint64_t target_state,
				 struct task_struct *originating_task);

// Take the next task scheduling decision and return the time at which the
// decision should be reconsidered. This function must be called with the
// gmcres_task_state lock held and IRQ disabled.
// If the interval crosses a major cycle boundary, or if the task has an insufficient
// criticality level, reset the task criticality mode to 0 and use this criticality mode.
// min_criticality_mode, if not NULL, will be updated with the requested minimal criticality
// mode in casd this task has missed its deadline, and let untouched otherwise.
static lt_t task_scheduling_decision(struct task_struct *tsk,
				     unsigned int *min_criticality_mode)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gmcres_cpu_state *state;
	const lt_t now = litmus_clock();
	const lt_t current_major_cycle_start = major_cycle_start(now);
	lt_t start, end, next_timer;
	int previous_cpu, next_cpu;
	unsigned int scm = system_criticality_mode();
	bool is_inside_interval, is_running, is_retained, is_last_of_period,
		past_current_interval;

	// Is the task currently running?
	is_running = tsk->state == TASK_RUNNING;

	// Remember the previous CPU this task could have been scheduled on
	previous_cpu = tinfo->gtdinterval->cpu;

	// Is this interval the last of its period?
	is_last_of_period = tinfo->gtdinterval->terminates_period;

	// Are we past the current interval (dealing with the end)?
	past_current_interval =
		now > tinfo->major_cycle_start + tinfo->gtdinterval->end;

	// Is the task being retained waiting for a synchronized release?
	is_retained = tsk_rt(tsk)->sporadic_release;

	TRACE_TASK(
		tsk,
		"(is_running:%u - completed:%u - is_retained:%u - is_present:%u) "
		"for interval [%llu-%llu] (cpu %u) (is-last-of-period:%u)\n",
		is_running, is_completed(tsk), is_retained, is_present(tsk),
		tinfo->major_cycle_start + tinfo->gtdinterval->start,
		tinfo->major_cycle_start + tinfo->gtdinterval->end,
		previous_cpu, is_last_of_period);

	// Check if the task is overdue - if the task criticality mode is different
	// from the system one, do nothing here as we are called from the mode change
	// function.
	if (is_last_of_period && past_current_interval && !is_completed(tsk) &&
	    !is_retained && is_running && tinfo->criticality_mode == scm) {
		TRACE_TASK(tsk, "has missed its deadline (%llu)\n",
			   tinfo->major_cycle_start + tinfo->gtdinterval->end);

		// Indicate that a criticality mode change is requested
		if (min_criticality_mode) {
			*min_criticality_mode = tinfo->criticality_mode + 1;
			TRACE_TASK(
				tsk,
				"will request a criticality mode of at least %u\n",
				*min_criticality_mode);
		}
	}

	// If the task has been released, release it (mark it as not completed)
	if (is_completed(tsk) && is_released(tsk, now)) {
		TRACE_TASK(tsk, "releasing\n");
		tsk_rt(tsk)->completed = 0;
	}

	// Advance intervals as much as needed if there is an interval and set end.
	if (tinfo->criticality_mode == scm)
		while (tinfo->major_cycle_start + tinfo->gtdinterval->end <=
		       now)
			tinfo->gtdinterval = gtd_reservation_next_interval(
				tinfo->gtdres, tinfo->gtdinterval,
				&tinfo->major_cycle_start);
	else {
		// The system criticality mode is greater than the task one
		TRACE_TASK(
			tsk,
			"looking for interval at different criticality mode %u (was %u)\n",
			scm, tinfo->criticality_mode);
		gtd_reservation_find_interval(tinfo->gtdres, scm, now,
					      &tinfo->gtdinterval,
					      &tinfo->major_cycle_start);
		tinfo->criticality_mode = scm;
	}

	if (tinfo->criticality_mode &&
	    (!tinfo->gtdinterval ||
	     tinfo->major_cycle_start != current_major_cycle_start)) {
		// We have no interval at the requested mode or it crosses a major cycle boundary.
		// We must return to criticality mode 0.
		TRACE_TASK(tsk, "falling back to criticality mode 0\n");
		gtd_reservation_find_interval(
			tinfo->gtdres, 0,
			current_major_cycle_start + system_major_cycle,
			&tinfo->gtdinterval, &tinfo->major_cycle_start);
		WARN_ON(!tinfo->gtdinterval);
		WARN_ON(tinfo->major_cycle_start !=
			current_major_cycle_start + system_major_cycle);
		tinfo->criticality_mode = 0;
	}

	start = tinfo->major_cycle_start + tinfo->gtdinterval->start;
	end = tinfo->major_cycle_start + tinfo->gtdinterval->end;
	is_inside_interval = now >= start;

	TRACE_TASK(
		tsk,
		"has computed new interval [%llu-%llu] (cpu %u) (mode %u) (is_inside_interval:%u)\n",
		start, end, tinfo->gtdinterval->cpu, tinfo->criticality_mode,
		is_inside_interval);
	// Check the next CPU to use, or NO_CPU
	next_cpu = is_inside_interval && is_running && !is_completed(tsk) ?
				 tinfo->gtdinterval->cpu :
				 NO_CPU;

	// Mark and reschedule concerned CPUs
	state = cpu_state_for(previous_cpu);
	raw_spin_lock(&state->lock);
	if (next_cpu == previous_cpu) {
		TRACE_TASK(tsk, "requesting scheduling on same CPU %u\n",
			   previous_cpu);
		state->linked = tsk;
		if (state->scheduled != tsk) {
			TRACE_TASK(
				tsk,
				"reschedule operation requested for CPU %u\n",
				previous_cpu);
#ifdef TIMING_TRACES
			atomic_or(1 << previous_cpu,
				  &mode_change_rescheduling_cpus);
			WRITE_ONCE(state->reschedule_date, now);
#endif // TIMING_TRACES
			litmus_reschedule(previous_cpu);
		}
	} else if (state->linked == tsk) {
		TRACE_TASK(tsk, "requesting descheduling from CPU %u\n",
			   previous_cpu);
		state->linked = NULL;
		TRACE_TASK(tsk, "reschedule operation requested for CPU %u\n",
			   previous_cpu);
#ifdef TIMING_TRACES
		atomic_or(1 << previous_cpu, &mode_change_rescheduling_cpus);
#endif // TIMING_TRACES
		litmus_reschedule(previous_cpu);
	}
	// If we have state->linked != tsk and state->scheduled == tsk,
	// then another task has requested a rescheduling in order to satisfy
	// its recent "linked" requirement, no need to schedule an extra one.
	raw_spin_unlock(&state->lock);

	if (next_cpu != previous_cpu && next_cpu != NO_CPU) {
		int needs_reschedule;
		state = cpu_state_for(next_cpu);
		raw_spin_lock(&state->lock);
		// We needs to reschedule the target CPU unless some other task is
		// scheduled there and has asked to be descheduled by setting
		// state->linked to NULL, in which case the reschedule will happen
		// anyway and the current task will be scheduled spontaneously once
		// state->linked is set and the lock is released.
		needs_reschedule = !(state->scheduled && !state->linked);
		TRACE_TASK(tsk, "requesting scheduling on new CPU %u\n",
			   next_cpu);
		state->linked = tsk;
		if (needs_reschedule) {
			TRACE_TASK(
				tsk,
				"reschedule operation requested for CPU %u\n",
				next_cpu);
#ifdef TIMING_TRACES
			WRITE_ONCE(state->migration_reschedule_date, now);
			atomic_or(1 << next_cpu,
				  &mode_change_rescheduling_cpus);
#endif // TIMING_TRACES
			litmus_reschedule(next_cpu);
		}
		raw_spin_unlock(&state->lock);
	}

	// Set the timer to the next event (interval start or end, or task release)
	next_timer = is_inside_interval ? end : start;
	if (is_completed(tsk) && lt_before(get_release(tsk), next_timer)) {
		next_timer = get_release(tsk);
		WARN_ON(lt_before(next_timer, now));
	}
	TRACE_TASK(tsk,
		   "computing next timer at %llu for interval [%llu-%llu]\n",
		   next_timer, start, end);

	return next_timer;
}

// Act on interval timer
static enum hrtimer_restart on_event_timer(struct hrtimer *timer)
{
	struct gmcres_task_state *tinfo =
		container_of(timer, struct gmcres_task_state, event_timer);
	struct task_struct *tsk = tinfo->gtdres->task;
	unsigned int min_criticality_mode = 0;
	lt_t next_timer;
#ifdef TIMING_TRACES
	lt_t theoretical_event_date = 0;
	lt_t tfe_lag = 0;
#endif // TIMING_TRACES

	// hrtimer handlers are called from a IRQ handler, so IRQ are already disabled
	raw_spin_lock(&tinfo->lock);
	if (tinfo->gtdinterval) {
#ifdef TIMING_TRACES
		theoretical_event_date =
			tinfo->major_cycle_start +
			(litmus_clock() >= tinfo->major_cycle_start +
						   tinfo->gtdinterval->end ?
				       tinfo->gtdinterval->end :
				       tinfo->gtdinterval->start);
		tfe_lag = litmus_clock() - theoretical_event_date;
#endif // TIMING_TRACES
		TRACE_TASK(tsk, "on timer with interval [%llu-%llu]\n",
			   tinfo->major_cycle_start + tinfo->gtdinterval->start,
			   tinfo->major_cycle_start + tinfo->gtdinterval->end);
	} else
		TRACE_TASK(tsk, "on timer with no interval");
	next_timer = task_scheduling_decision(tsk, &min_criticality_mode);
	raw_spin_unlock(&tinfo->lock);

	if (min_criticality_mode > system_criticality_mode()) {
		TRACE_TASK(tsk, "will increase the criticality mode to %u\n",
			   min_criticality_mode);
#ifdef TIMING_TRACES
		mode_change_event_date = theoretical_event_date;
		if (tfe_lag)
			TRACE_TASK(tsk, "TFE lag is %llu ns\n", tfe_lag);
#endif // TIMING_TRACES
		ensure_minimum_criticality_state(
			criticality_state(litmus_clock(), min_criticality_mode),
			tsk);
	} else {
		TRACE_TASK(tsk, "next timer expiration set to %llu\n",
			   next_timer);
		hrtimer_set_expires(timer, ns_to_ktime(next_timer));
	}

	return HRTIMER_RESTART;
}

// Ensure that the criticality state is at least target_state when exiting this function.
// If the global criticality state is already at least at target_state, return false. It
// does not mean that the mode change is completed, but that it has at least started
// already. If the mode change is necessary, force every task to reconsider its scheduling
// decisions and return true.
// When this function is called, no gmcres_task_state or gmcres_cpu_state locks must be
// held, as both will be taken during the rescheduling decisions if the mode actually
// changes.
// Concerning the task originating_task, it is assumed that its timer entry is locked
// since this function is called from the timer handler. Therefore, only its expiration
// date is set, the timer will be restarted by the caller.
static bool
ensure_minimum_criticality_state(uint64_t target_state,
				 struct task_struct *originating_task)
{
	struct gtd_reservation *gtdres;
	unsigned int target_mode = criticality_mode(target_state);
	uint64_t current_state;
	unsigned long flags;
#ifdef TIMING_TRACES
	lt_t emcs_entry = litmus_clock();
#endif // TIMING_TRACES

	if (target_mode > atomic_read(&maximum_criticality_level)) {
		TRACE("ignoring request to switch to too high criticality mode %u\n",
		      target_mode);
	}

	atomic_inc_return_acquire(&access_counter);

	current_state = atomic64_read(&system_criticality_state);
	if (current_state >= target_state) {
		atomic_dec(&access_counter);
		return false;
	}
	if (atomic64_cmpxchg(&system_criticality_state, current_state,
			     target_state) != current_state) {
		atomic_dec(&access_counter);
		return false;
	}

	TRACE("switching to criticality mode %u\n", target_mode);

	// For every task in the reservations, reconsider scheduling decisions.
	// To ensure new reservations are not added, use the reservation lock.
	raw_spin_lock_irqsave(&gtdenv.writer_lock, flags);
	list_for_each_entry (gtdres, &gtdenv.all_reservations,
			     all_reservations_list) {
		struct task_struct *tsk = gtdres->task;
		if (tsk) {
			struct gmcres_task_state *tinfo =
				get_gmcres_task_state(tsk);
			lt_t next_timer;
			raw_spin_lock(&tinfo->lock);

			// Make the task compute its next iteration time at either the
			// current criticality mode if appropriate, or at criticality
			// mode 0 at the next major cycle.
			next_timer = task_scheduling_decision(tsk, NULL);
			if (tsk == originating_task)
				hrtimer_set_expires(&tinfo->event_timer,
						    ns_to_ktime(next_timer));
			else
				hrtimer_start(&tinfo->event_timer,
					      ns_to_ktime(next_timer),
					      HRTIMER_MODE_ABS);
			TRACE_TASK(tsk, "timer reset to %llu\n", next_timer);
			raw_spin_unlock(&tinfo->lock);
		}
	}
	raw_spin_unlock_irqrestore(&gtdenv.writer_lock, flags);

	TRACE("criticality mode switch done\n");

	atomic_dec_return_release(&access_counter);
#ifdef TIMING_TRACES
	TRACE("tables recomputation done in %llu ns\n",
	      litmus_clock() - emcs_entry);
#endif // TIMING_TRACES
	return true;
}

static struct task_struct *gmcres_schedule(struct task_struct *prev)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	uint64_t scs;
	lt_t now;
#ifdef TIMING_TRACES
	lt_t schedule_entry = litmus_clock();
#endif // TIMING_TRACES

	raw_spin_lock(&state->lock);

	// Sanity checks and previous task information. This must be
	// done before the "restart" label below as to reflect the
	// real previous conditions.
	WARN_ON(state->scheduled && state->scheduled != prev);
	state->prev_was_realtime = prev && is_realtime(prev);
	WARN_ON(state->scheduled && !state->prev_was_realtime);

	// Wait until no change is in progress and release the lock in the
	// meantime as the CPU state might be modified during the mode change.
	if (atomic_read(&access_counter)) {
#ifdef TIMING_TRACES
		lt_t spin_entry;
#endif // TIMING_TRACES
		TRACE("access counter is set, release the lock temporarily\n");
		raw_spin_unlock(&state->lock);
		// Restart here if a mode change happens during the scheduling,
		// while the lock has been released.
	restart:
#ifdef TIMING_TRACES
		spin_entry = litmus_clock();
#endif // TIMING_TRACES
		TRACE("waiting for access counter to reset\n");
		while (atomic_read(&access_counter))
			cpu_relax();
#ifdef TIMING_TRACES
		if (state->linked || state->scheduled)
			TRACE("blocked on access_counter for %llu ns\n",
			      litmus_clock() - spin_entry);
#endif // TIMING_TRACES
		raw_spin_lock(&state->lock);
		TRACE("access counter has been reset\n");
	}
#ifdef TIMING_TRACES
	else if (state->linked || state->scheduled)
		TRACE("blocked on access_counter for 0 ns\n");
#endif // TIMING_TRACES
	scs = atomic64_read(&system_criticality_state);

	now = litmus_clock();

	// Link the task we were asked to link
	if (state->scheduled != state->linked) {
		if (state->scheduled)
			TRACE_TASK(state->scheduled, "descheduled\n");
		if (state->linked)
			TRACE_TASK(state->linked, "scheduled\n");
	} else if (state->scheduled)
		TRACE_TASK(state->scheduled, "already scheduled\n");

	state->scheduled = state->linked;
	sched_state_task_picked();

	raw_spin_unlock(&state->lock);

	// Check if a system criticality mode change, or change in progress, has
	// not made this scheduling decision obsolete already, and restart the
	// process if it does. It is important that we do this once the CPU state lock
	// has been released, because the mode change process might also spin on this
	// CPU lock.
	if (atomic64_read(&system_criticality_state) > scs ||
	    atomic_read(&access_counter)) {
		TRACE("criticality mode change detected while scheduling new task, "
		      "restarting decision\n");
		goto restart;
	}

	if (state->scheduled) {
#ifdef TIMING_TRACES
		lt_t reschedule_date = READ_ONCE(state->reschedule_date);
		lt_t migration_reschedule_date =
			READ_ONCE(state->migration_reschedule_date);
		lt_t now = litmus_clock();
		if (reschedule_date && reschedule_date < now &&
		    !migration_reschedule_date) {
			TRACE_TASK(state->scheduled,
				   "has been scheduled in %llu ns\n",
				   now - reschedule_date);
			WRITE_ONCE(state->reschedule_date, 0);
		} else if (migration_reschedule_date &&
			   migration_reschedule_date < now &&
			   !reschedule_date) {
			WRITE_ONCE(state->reschedule_date, now);
			TRACE_TASK(
				state->scheduled,
				"has been scheduled but not migrated in %llu ns\n",
				now - migration_reschedule_date);
		}
		TRACE("gmcres_schedule took %llu ns\n",
		      litmus_clock() - schedule_entry);
#endif // TIMING_TRACES
		TRACE_TASK(state->scheduled,
			   "returning decision for criticality mode %u\n",
			   criticality_mode_at(scs, now));
	} else
		TRACE("returning decision (nothing) for criticality mode %u\n",
		      criticality_mode_at(scs, now));
#ifdef TIMING_TRACES
	if (atomic_read(&mode_change_rescheduling_cpus) & (1 << state->cpu)) {
		lt_t event_date = mode_change_event_date;
		atomic_andnot(1 << state->cpu, &mode_change_rescheduling_cpus);
		if (!atomic_read(&mode_change_rescheduling_cpus) &&
		    event_date) {
			mode_change_event_date = 0;
			TRACE("total mode change in %llu ns\n",
			      litmus_clock() - event_date);
		}
	}
#endif // TIMING_TRACES
	return state->scheduled;
}

static long gmcres_admit_task(struct task_struct *tsk)
{
	// Note: in this function, use LITMUS_TRACE_TASK instead of TRACE_TASK
	// while the gmcres_task_state is not complete and attached to the task.
	struct gmcres_task_state *tinfo;
	unsigned int res_id = tsk_rt(tsk)->task_params.cpu;
	struct gtd_reservation *gtdres = gtd_env_find(&gtdenv, res_id);
	lt_t period = get_rt_period(tsk);
	long ret = 0;
	unsigned long flags;

	if (!gtdres) {
		LITMUS_TRACE_TASK(tsk, "requires an unknown reservation %u\n",
				  res_id);
		return -EINVAL;
	}

	// The task period must divide the reservation major cycle
	if (gtdres->major_cycle % period) {
		LITMUS_TRACE_TASK(
			tsk,
			"period (%llu) is not an integral factor of the reservation %u "
			"major cycle (%llu)\n",
			period, res_id, gtdres->major_cycle);
		return -EINVAL;
	}

	// If the task major cycle is different from the system major cycle,
	// check that we have no task currently in the system. If we have none,
	// update the system major cycle and ensure that the system criticality mode
	// is at its minimum, otherwise refuse the task.
	raw_spin_lock_irqsave(&gtdenv.writer_lock, flags);
	if (gtdres->major_cycle != system_major_cycle) {
		struct gtd_reservation *res;
		list_for_each_entry (res, &gtdenv.all_reservations,
				     all_reservations_list)
			if (res->task) {
				LITMUS_TRACE_TASK(
					tsk,
					"new task reservation uses a a major cycle (%llu) "
					"different from the current system major cycle (%llu)\n",
					gtdres->major_cycle,
					system_major_cycle);
				raw_spin_unlock_irqrestore(&gtdenv.writer_lock,
							   flags);
				return -EINVAL;
			}
		// Update the system major cycle and reset criticality mode
		LITMUS_TRACE_TASK(tsk,
				  "setting the system major cycle to %llu\n",
				  gtdres->major_cycle);
		system_major_cycle = gtdres->major_cycle;
		atomic64_set(&system_criticality_state,
			     criticality_state(litmus_clock(), 0));
	}
	raw_spin_unlock_irqrestore(&gtdenv.writer_lock, flags);

	tinfo = kzalloc(sizeof(*tinfo), GFP_ATOMIC);
	if (!tinfo)
		return -ENOMEM;

	preempt_disable();
	raw_spin_lock_init(&tinfo->lock);
	tinfo->gtdres = gtdres;
	tinfo->gtdinterval = NULL;
	tinfo->major_cycle_start = GTDRES_TIME_NA;
	hrtimer_init(&tinfo->event_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	tinfo->event_timer.function = on_event_timer;
	tsk_rt(tsk)->plugin_state = tinfo;

	// Note: TRACE_TASK can be used from now on

	// Attempt attachment to the reservation
	raw_spin_lock(&gtdres->lock);
	if (gtdres->task) {
		TRACE_TASK(tsk,
			   "attempts to bind to already-used reservation %u\n",
			   res_id);
		TRACE_TASK(gtdres->task,
			   "has been the victim of a hijack attempt "
			   "on reservation %u\n",
			   res_id);
		kfree(tinfo);
		ret = -EPERM;
		goto unlock_and_return;
	}

	// Set end-of-period markers
	if ((ret = gtd_reservation_mark_end_of_periods(gtdres, period)))
		goto unlock_and_return;

	// The budget enforcement will be done on interval boundaries only
	tsk_rt(tsk)->task_params.budget_policy = NO_ENFORCEMENT;

	// Update the maximum criticality level of admitted tasks
	while (true) {
		unsigned int mcl = atomic_read(&maximum_criticality_level);
		if (mcl >= gtdres->criticality_level)
			break;
		TRACE("Increasing the maximum criticality level to %u\n",
		      gtdres->criticality_level);
		if (atomic_cmpxchg(&maximum_criticality_level, mcl,
				   gtdres->criticality_level) == mcl)
			break;
	}

	gtdres->task = tsk;

	gtd_reservation_dump(gtdres);

unlock_and_return:
	raw_spin_unlock(&gtdres->lock);
	preempt_enable();

	return ret;
}

// Compute the date of the next timer related to the task tsk and the time
// now. This must be called with interrupts disabled and the task structure locked.
static lt_t next_task_timer(struct gmcres_task_state *tinfo, lt_t now)
{
	lt_t start = tinfo->major_cycle_start + tinfo->gtdinterval->start;
	if (start > now)
		return start;
	return tinfo->major_cycle_start + tinfo->gtdinterval->end;
}

// If required, advance event_timer to the given deadline. This must be called
// with interrupts disabled and the task structure locked. The current interval
// must not be NULL.
static void advance_event_timer_at(struct task_struct *tsk, lt_t deadline)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	if (lt_before(deadline,
		      ktime_to_ns(hrtimer_get_expires(&tinfo->event_timer)))) {
		TRACE_TASK(
			current,
			"requires advancing timer since release is before next timer\n");
		hrtimer_start(&tinfo->event_timer, ns_to_ktime(deadline),
			      HRTIMER_MODE_ABS);
	}
}

static void gmcres_task_new(struct task_struct *tsk, int on_runqueue,
			    int is_running)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	lt_t now, next_timer, release_time;
	unsigned long flags;
	bool is_inside_interval;
	int wanted_cpu, current_cpu;

	raw_spin_lock_irqsave(&tinfo->lock, flags);
	now = litmus_clock();

	tinfo->criticality_mode = 0;
	is_inside_interval =
		gtd_reservation_find_interval(tinfo->gtdres, 0, now,
					      &tinfo->gtdinterval,
					      &tinfo->major_cycle_start);
	BUG_ON(!tinfo->gtdinterval);
	TRACE_TASK(
		tsk,
		"registers interest for interval [%llu-%llu] (is_inside_interval:%u)\n",
		tinfo->major_cycle_start + tinfo->gtdinterval->start,
		tinfo->major_cycle_start + tinfo->gtdinterval->end,
		is_inside_interval);
	next_timer = next_task_timer(tinfo, now);
	TRACE_TASK(tsk, "will set first timer for %llu (now is %llu)\n",
		   next_timer, now);
	hrtimer_start(&tinfo->event_timer, ns_to_ktime(next_timer),
		      HRTIMER_MODE_ABS);

	TRACE_TASK(tsk,
		   "is a new RT task at %llu (on runqueue:%u, running:%u)\n",
		   now, on_runqueue, is_running);

	// Setup job parameters by aligning the release time to the next occurrence of the
	// task period.
	release_time = now + get_rt_period(tsk) - (now % get_rt_period(tsk));
	release_at(tsk, release_time);
	tsk_rt(tsk)->completed = 1;
	TRACE_TASK(tsk, "will be first released at %llu (now is %llu)\n",
		   release_time, now);

	// Compute the wanted CPU or NO_CPU
	wanted_cpu = is_inside_interval ? tinfo->gtdinterval->cpu : NO_CPU;

	if (is_running) {
		// Register the task as running on the current CPU. Insist on
		// staying there if this is the right CPU, else reschedule.
		struct gmcres_cpu_state *state = local_cpu_state();
		raw_spin_lock(&state->lock);
		current_cpu = state->cpu;
		WARN_ON(state->scheduled);
		state->scheduled = tsk;
		TRACE_TASK(tsk, "registered as currently running here\n");
		if (current_cpu == wanted_cpu) {
			TRACE_TASK(
				tsk,
				"is currently running on the CPU it wants\n");
			state->linked = tsk;
		} else {
			TRACE_TASK(tsk,
				   "will ask to get descheduled from CPU %u\n",
				   current_cpu);
			litmus_reschedule_local();
		}
		raw_spin_unlock(&state->lock);

		// If we want the task to run right now on another CPU, tell it
		// to do so and reschedule both the current CPU to release the
		// task and the wanted one to schedule the task.
		if (wanted_cpu != NO_CPU && wanted_cpu != current_cpu) {
			struct gmcres_cpu_state *state =
				cpu_state_for(wanted_cpu);
			raw_spin_lock(&state->lock);
			state->linked = tsk;
#ifdef TIMING_TRACES
			WRITE_ONCE(state->reschedule_date, litmus_clock());
#endif // TIMING_TRACES
			TRACE_TASK(tsk, "is requesting to run on CPU %u\n",
				   wanted_cpu);
			raw_spin_unlock(&state->lock);
			litmus_reschedule(wanted_cpu);
		}
	}
	raw_spin_unlock_irqrestore(&tinfo->lock, flags);
}

static void gmcres_task_exit(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	int cpu;
	unsigned long flags;
	unsigned int tcl;

	TRACE_TASK(tsk, "exiting at %llu\n", litmus_clock());

	// Synchronously ensure that the timer can no longer fire
	hrtimer_cancel(&tinfo->event_timer);

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	// Remove the task from the reservation so that it will no
	// longer be scheduled.
	raw_spin_lock_irqsave(&tinfo->gtdres->lock, flags);
	BUG_ON(tinfo->gtdres->task != tsk);
	tinfo->gtdres->task = NULL;
	tcl = tinfo->gtdres->criticality_level;
	raw_spin_unlock(&tinfo->gtdres->lock);

	// Note: use LITMUS_TRACE_TASK instead of TRACE_TASK from now on since
	// the reservation is detached from the task.

	// If the exiting task had the maximum criticality level and it was
	// not 0, update the maximum criticality level since it might have
	// gone down.
	if (tcl == atomic_read(&maximum_criticality_level) && tcl > 0) {
		atomic_set(&maximum_criticality_level,
			   gtd_env_maximum_task_criticality_level(&gtdenv));
	}

	// If the task is scheduled on any CPU, deschedule it and deschedule requests.
	for_each_online_cpu (cpu) {
		struct gmcres_cpu_state *state = cpu_state_for(cpu);
		raw_spin_lock_irqsave(&state->lock, flags);
		if (state->scheduled == tsk) {
			TRACE_TASK(
				tsk,
				"has been forcefully descheduled from CPU %u\n",
				cpu);
			state->scheduled = NULL;
		}
		if (state->linked == tsk) {
			TRACE_TASK(
				tsk,
				"has been forcefully derequested from CPU %u\n",
				cpu);
			state->linked = NULL;
		}
		raw_spin_unlock_irqrestore(&state->lock, flags);
	}

	tsk_rt(tsk)->plugin_state = NULL;
	kfree(tinfo);
}

static long gmcres_reservation_create(int res_type, void *__user _config)
{
	struct reservation_config config;
	struct gtd_reservation *gtdres;
	struct lt_interval interval;
	long ret;
	unsigned int i;

	if (res_type != TABLE_DRIVEN) {
		TRACE("Only table-driven reservations are allowed\n");
		return -EINVAL;
	}

	if (copy_from_user(&config, _config, sizeof(config)))
		return -EFAULT;

	if (config.cpu < 0 || !cpu_online(config.cpu)) {
		TRACE("Invalid reservation (%u): CPU %u offline\n", config.id,
		      config.cpu);
		return -EINVAL;
	}

	if (config.table_driven_params.major_cycle_length < MIN_MAJOR_CYCLE) {
		TRACE("Invalid reservation (%u): major cycle (%llu) is smaller "
		      "than the minimum allowed (%llu)\n",
		      config.id, config.table_driven_params.major_cycle_length,
		      MIN_MAJOR_CYCLE);
		return -EINVAL;
	}

	raw_spin_lock(&gtdenv.writer_lock);
	ret = gtd_env_find_or_create(&gtdenv, &config, &gtdres);
	if (ret < 0) {
		TRACE("Cannot create reservation %u\n", config.id);
		goto error_with_unlock;
	}

	// Add intervals to the reservation
	for (i = 0; i < config.table_driven_params.num_intervals; i++) {
		if ((ret = copy_from_user(
			     &interval,
			     &config.table_driven_params.intervals[i],
			     sizeof(interval)))) {
			goto error_with_unlock;
		}
		if (interval.end / gtdres->major_cycle >= 1 << CRIT_MODE_BITS) {
			TRACE("Cannot add interval [%llu-%llu] on CPU %u because it exceeds "
			      "the greatest allowed number of criticality levels (%u)\n",
			      interval.start, interval.end, config.cpu,
			      (1 << CRIT_MODE_BITS) - 1);
			ret = -EINVAL;
			goto error_with_unlock;
		}
		ret = gtd_reservation_add_interval(gtdres, interval.start,
						   interval.end, config.cpu);
		if (ret < 0) {
			TRACE("Cannot add interval [%llu-%llu] on CPU %u\n",
			      interval.start, interval.end, config.cpu);
			goto error_with_unlock;
		}
	}

	raw_spin_unlock(&gtdenv.writer_lock);

	return 0;
error_with_unlock:
	raw_spin_unlock(&gtdenv.writer_lock);
	return ret;
}

static bool gmcres_post_migration_validate(struct task_struct *next)
{
	struct gmcres_cpu_state *state = local_cpu_state();
#ifdef TIMING_TRACES
	lt_t migration_reschedule_date =
		READ_ONCE(state->migration_reschedule_date);
	lt_t reschedule_date = READ_ONCE(state->reschedule_date);
	lt_t now;
	if (migration_reschedule_date &&
	    migration_reschedule_date < (now = litmus_clock())) {
		TRACE_TASK(state->scheduled,
			   "has been scheduled in %llu ns (%llu ns wait)\n",
			   now - migration_reschedule_date,
			   now - reschedule_date);
		WRITE_ONCE(state->migration_reschedule_date, 0);
		WRITE_ONCE(state->reschedule_date, 0);
	}
#endif // TIMING_TRACES
	TRACE_TASK(next, "has arrived on CPU %u\n", state->cpu);
	return 1;
}

static void gmcres_next_became_invalid(struct task_struct *next)
{
	TRACE_TASK(next, "is being asked to be reconsidered\n");
}

static bool gmcres_should_wait_for_stack(struct task_struct *next)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	unsigned long flags;

	raw_spin_lock_irqsave(&state->lock, flags);
	TRACE_TASK(next, "cannot be acquired now\n");
	if (state->linked != next) {
		raw_spin_unlock_irqrestore(&state->lock, flags);
		TRACE_TASK(
			next,
			"should no longer scheduled on CPU %u anyway, giving up\n",
			state->cpu);
		return false;
	} else if (state->prev_was_realtime) {
		state->scheduled = NULL;
		raw_spin_unlock_irqrestore(&state->lock, flags);
		TRACE_TASK(next, "relinguished for now to avoid deadlock\n");
		return false;
	} else {
		raw_spin_unlock_irqrestore(&state->lock, flags);
		TRACE_TASK(
			next,
			"cannot be implicated in a deadlock, not giving up\n");
		return true;
	}
}

static bool gmcres_fork_task(struct task_struct *tsk)
{
	TRACE_CUR("is forking, refusing\n");
	return false;
}

static void gmcres_task_block(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gmcres_cpu_state *state;
	unsigned long flags;
	int current_cpu;

	TRACE_TASK(tsk, "blocked\n");

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	// We need to unschedule the task from the current CPU
	state = local_cpu_state();
	raw_spin_lock(&state->lock);
	current_cpu = state->cpu;
	WARN_ON(state->scheduled != tsk);
	if (state->scheduled == tsk)
		state->scheduled = NULL;
	if (state->linked == tsk)
		state->linked = NULL;
	raw_spin_unlock(&state->lock);

	// If the task had already made a request for another CPU, we need to
	// cancel it.
	if (tinfo->gtdinterval &&
	    litmus_clock() >=
		    tinfo->major_cycle_start + tinfo->gtdinterval->start) {
		int expected_cpu = tinfo->gtdinterval->cpu;
		if (current_cpu != expected_cpu) {
			TRACE_TASK(
				tsk,
				"must be unscheduled/unrequested from CPU %u\n",
				expected_cpu);
			state = cpu_state_for(expected_cpu);
			raw_spin_lock(&state->lock);
			if (state->linked == tsk) {
				state->linked = NULL;
				litmus_reschedule(expected_cpu);
			}
			raw_spin_unlock(&state->lock);
		}
	}

	raw_spin_unlock_irqrestore(&tinfo->lock, flags);
}

static void gmcres_task_wake_up(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gmcres_cpu_state *state;
	unsigned long flags;
	int expected_cpu;

	TRACE_TASK(tsk, "is waking up\n");

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	// If the task is in an interval, check the CPU it would like to be scheduled on
	expected_cpu =
		tinfo->gtdinterval &&
				litmus_clock() >=
					tinfo->major_cycle_start +
						tinfo->gtdinterval->start ?
			      tinfo->gtdinterval->cpu :
			      NO_CPU;

	// If another CPU than the current one is expected, mark the task as requested
	// and provoke a reschedule at both places.
	if (expected_cpu != NO_CPU) {
		TRACE_TASK(tsk, "is requesting to be scheduled on CPU %u\n",
			   expected_cpu);
		state = cpu_state_for(expected_cpu);
		raw_spin_lock(&state->lock);
		state->linked = tsk;
#ifdef TIMING_TRACES
		state->reschedule_date = litmus_clock();
#endif // TIMING_TRACES
		raw_spin_unlock(&state->lock);
		litmus_reschedule_local();
		litmus_reschedule(expected_cpu);
	}

	raw_spin_unlock_irqrestore(&tinfo->lock, flags);
}

static struct domain_proc_info gmcres_domain_proc_info;

static long gmcres_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &gmcres_domain_proc_info;
	return 0;
}

static void gmcres_setup_domain_proc(void)
{
	int i, cpu;
	int num_rt_cpus = num_online_cpus();

	struct cd_mapping *cpu_map, *domain_map;

	memset(&gmcres_domain_proc_info, 0, sizeof(gmcres_domain_proc_info));
	init_domain_proc_info(&gmcres_domain_proc_info, num_rt_cpus,
			      num_rt_cpus);
	gmcres_domain_proc_info.num_cpus = num_rt_cpus;
	gmcres_domain_proc_info.num_domains = num_rt_cpus;

	i = 0;
	for_each_online_cpu (cpu) {
		cpu_map = &gmcres_domain_proc_info.cpu_to_domains[i];
		domain_map = &gmcres_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i;
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long gmcres_activate_plugin(void)
{
	int cpu;
	struct gmcres_cpu_state *state;

	atomic64_set(&system_criticality_state, 0);
	atomic_set(&access_counter, 0);
	atomic_set(&maximum_criticality_level, 0);
	system_major_cycle = MIN_MAJOR_CYCLE; // Meaningless but non-zero value

	gtd_env_init(&gtdenv);

	for_each_online_cpu (cpu) {
		TRACE("Initializing CPU%u...\n", cpu);

		state = cpu_state_for(cpu);
		memset(state, 0, sizeof(*state));
		state->cpu = cpu;
	}

	gmcres_setup_domain_proc();

	return 0;
}

static long gmcres_deactivate_plugin(void)
{
	int cpu;
	struct gmcres_cpu_state *state;
	struct gtd_reservation *gtdres, *p;

	raw_spin_lock(&gtdenv.writer_lock);

	list_for_each_entry_safe (gtdres, p, &gtdenv.all_reservations,
				  all_reservations_list) {
		list_del(&gtdres->all_reservations_list);
		gtd_reservation_clear(gtdres);
		WARN_ON(gtdres->task);
		kfree(gtdres);
	}

	for_each_online_cpu (cpu) {
		state = cpu_state_for(cpu);
		raw_spin_lock(&state->lock);
		WARN_ON(state->scheduled);
	}

	for_each_online_cpu (cpu) {
		state = cpu_state_for(cpu);
		raw_spin_unlock(&state->lock);
	}

	raw_spin_unlock(&gtdenv.writer_lock);

	destroy_domain_proc_info(&gmcres_domain_proc_info);

	return 0;
}

long gmcres_complete_job(void)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(current);
	lt_t now;
	unsigned long flags;

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	prepare_for_next_period(current);
	now = litmus_clock();
	TRACE_TASK(current,
		   "has completed its job - "
		   "next release will be %llu, deadline will be %llu\n",
		   get_release(current), get_deadline(current));
	if (!is_released(current, now)) {
		struct gmcres_cpu_state *state = local_cpu_state();
		advance_event_timer_at(current, get_release(current));
		raw_spin_lock(&state->lock);
		WARN_ON(state->scheduled != current);
		if (state->linked == current)
			state->linked = NULL;
		raw_spin_unlock(&state->lock);
		TRACE_TASK(current, "will ask CPU %u to deschedule it\n",
			   state->cpu);
		litmus_reschedule_local();
		tsk_rt(current)->completed = 1;
	} else {
		TRACE_TASK(current,
			   "is already past its next release date %llu\n",
			   get_release(current));
	}

	raw_spin_unlock_irqrestore(&tinfo->lock, flags);

	return 0;
}

static struct sched_plugin gmcres_plugin = {
	.plugin_name = "G-MCRES",
	.schedule = gmcres_schedule,
	.admit_task = gmcres_admit_task,
	.task_new = gmcres_task_new,
	.task_exit = gmcres_task_exit,
	.fork_task = gmcres_fork_task,
	.task_block = gmcres_task_block,
	.task_wake_up = gmcres_task_wake_up,
	.complete_job = gmcres_complete_job,
	.get_domain_proc_info = gmcres_get_domain_proc_info,
	.activate_plugin = gmcres_activate_plugin,
	.deactivate_plugin = gmcres_deactivate_plugin,
	.reservation_create = gmcres_reservation_create,
	.should_wait_for_stack = gmcres_should_wait_for_stack,
	.next_became_invalid = gmcres_next_became_invalid,
	.post_migration_validate = gmcres_post_migration_validate,
};

static int __init init_gmcres(void)
{
	return register_sched_plugin(&gmcres_plugin);
}

module_init(init_gmcres);
