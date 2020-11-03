#include "asm/atomic.h"
#include "asm/smp.h"
#include "linux/completion.h"
#include "linux/cpumask.h"
#include "linux/key.h"
#include "linux/list.h"
#include "linux/time.h"
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

	// The CPU criticality mode. This is used to trigger the mode changes
	// since that cannot be done from a timer event so we defer it until
	// schedule time. This can never go down.
	unsigned int criticality_mode;
};

static DEFINE_PER_CPU(struct gmcres_cpu_state, gmcres_cpu_state);

#define cpu_state_for(cpu_id) (&per_cpu(gmcres_cpu_state, cpu_id))
#define local_cpu_state() (this_cpu_ptr(&gmcres_cpu_state))

struct gmcres_task_state {
	raw_spinlock_t lock;

	// The reservation associated to this task, cannot be NULL once the task
	// has been admitted, and contains the reserve link to the struct task_struct * entry.
	struct gtd_reservation *gtdres;

	// The current interval, cannot be NULL once the task has been started, and
	// the associated major cycle start time and criticality mode.
	struct gtd_interval *gtdinterval;
	lt_t major_cycle_start;
	unsigned criticality_mode;

	// The timer associated with beginning or end of intervals, is always
	// armed when the task has been admitted.
	struct hrtimer interval_timer;
};

// The current criticality mode
static atomic_t system_criticality_mode;

// The number of mode changes in progress
static atomic_t access_counter;

// The maximum criticality level of any existing task currently admitted
// by the plugin.
static atomic_t maximum_criticality_level;

static struct gmcres_task_state *get_gmcres_task_state(struct task_struct *tsk)
{
	return tsk_rt(tsk)->plugin_state;
}

// For debugging: add a trace to check if we are in an IRQ
#define CHECK_IRQ_CONTEXT()                                                                \
	do {                                                                               \
		TRACE("%s: in_irq:%d, in_softirq:%d, in_interrupt:%d, irqs_disabled:%d\n", \
		      __FUNCTION__, in_irq(), in_softirq(), in_interrupt(),                \
		      irqs_disabled());                                                    \
	} while (0)

// Take the next task scheduling decision and return the time at which the
// decision should be reconsidered. This function must be called with the
// gmcres_task_state lock held and IRQ disabled. Returning GTDRES_TIME_NA
// means that we do not want a timer to be set.
static lt_t task_scheduling_decision(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gmcres_cpu_state *state;
	lt_t now, start, end, next_timer;
	int previous_cpu, next_cpu;
	unsigned int scm;
	bool is_inside_interval, is_running, is_last_of_period,
		past_current_interval;

	now = litmus_clock();
	scm = atomic_read(&system_criticality_mode);

	// Is the task currently running?
	is_running = tsk->state == TASK_RUNNING;

	// Remember the previous CPU this task could have been scheduled on
	previous_cpu = tinfo->gtdinterval->cpu;

	// Is this interval the last of its period?
	is_last_of_period = tinfo->gtdinterval->terminates_period;

	// Are we past the current interval (dealing with the end)?
	past_current_interval = now > tinfo->gtdinterval->end;

	TRACE_TASK(tsk,
		   "(is_running:%d - completed:%d - is_between_jobs:%d) "
		   "executes on_interval_timer at %llu "
		   "for interval [%llu-%llu) (cpu %d) (is-last-of-period:%d)\n",
		   is_running, is_completed(tsk), now,
		   tinfo->major_cycle_start + tinfo->gtdinterval->start,
		   tinfo->major_cycle_start + tinfo->gtdinterval->end,
		   previous_cpu, is_last_of_period);

	// Check if the task is overdue.
	if (is_last_of_period && past_current_interval && !is_completed(tsk)) {
		TRACE_TASK(tsk, "has missed its deadline at %llu\n", now);
		TRACE_TASK(
			tsk,
			"should change criticality mode (task current %u - max %u) "
			"(system current %u - max %u)\n",
			tinfo->criticality_mode,
			tinfo->gtdres->criticality_level,
			atomic_read(&system_criticality_mode),
			atomic_read(&maximum_criticality_level));

		// Only increase criticality mode if we have at least one task ready
		// to execute at this mode.
		if (tinfo->criticality_mode <
		    atomic_read(&maximum_criticality_level)) {
			// Use the last CPU we were scheduled on to trigger the mode change
			state = cpu_state_for(previous_cpu);
			raw_spin_lock(&state->lock);
			if (state->criticality_mode <
			    tinfo->criticality_mode + 1) {
				state->criticality_mode =
					tinfo->criticality_mode + 1;
				litmus_reschedule(previous_cpu);
				TRACE_TASK(
					tsk,
					"used CPU %d to trigger criticality mode increase to %u\n",
					previous_cpu,
					tinfo->criticality_mode + 1);
			}
			raw_spin_unlock(&state->lock);

			// Do not set a new timer right now, the mode change will take effect
			// as soon as the scheduling takes place and a new interval will be
			// computed.
			return GTDRES_TIME_NA;
		} else
			TRACE_TASK(
				tsk,
				"cannot increase criticality mode as there would be "
				"no tasks to execute in criticality mode %u\n",
				tinfo->criticality_mode + 1);
	}

	// If the task has been released, release it (mark it as not completed)
	if (is_completed(tsk) && is_released(tsk, now)) {
		TRACE_TASK(tsk, "releasing at %llu\n", now);
		tsk_rt(tsk)->completed = 0;
	}

	// If the system criticality mode is above the task criticality mode,
	// the task will have to either update its current interval or to
	// terminate.
	if (tinfo->criticality_mode < scm) {
		// If the system criticality mode is above the task criticality level,
		// do not schedule anything and set the interval to NULL to indicate
		// that this function must never be called again.
		if (tinfo->gtdres->criticality_level < scm) {
			TRACE_TASK(
				tsk,
				"will no longer be scheduled because criticality level (%u) "
				"is less than system criticality mode (%u)\n",
				tinfo->gtdres->criticality_level, scm);
			// TODO: terminate the task at the Linux level as well to avoid
			// zombies
			tinfo->gtdinterval = NULL;
		} else {
			// Increase the task criticality mode and find an appropriate interval
			bool interval_found = gtd_reservation_find_interval(
				tinfo->gtdres, scm, now, &tinfo->gtdinterval,
				&tinfo->major_cycle_start);
			WARN_ON(!interval_found);
			tinfo->criticality_mode = scm;
		}
	}

	// Advance intervals as much as needed and compute start, end, is_inside_interval
	if (tinfo->gtdinterval) {
		while ((end = tinfo->major_cycle_start +
			      tinfo->gtdinterval->end) <= now)
			tinfo->gtdinterval = gtd_reservation_next_interval(
				tinfo->gtdres, tinfo->gtdinterval,
				&tinfo->major_cycle_start);
		start = tinfo->major_cycle_start + tinfo->gtdinterval->start;
		is_inside_interval = now >= start;

		TRACE_TASK(
			tsk,
			"has computed new interval [%llu-%llu) (cpu %d) (is_inside_interval:%d)\n",
			start, end, tinfo->gtdinterval->cpu,
			is_inside_interval);
		// Check the next CPU to use, or NO_CPU
		next_cpu =
			is_inside_interval && is_running && !is_completed(tsk) ?
				      tinfo->gtdinterval->cpu :
				      NO_CPU;

	} else {
		start = end = GTDRES_TIME_NA;
		is_inside_interval = false;
		next_cpu = NO_CPU;
	}

	// Mark and reschedule concerned CPUs
	state = cpu_state_for(previous_cpu);
	raw_spin_lock(&state->lock);
	if (next_cpu == previous_cpu) {
		TRACE_TASK(tsk, "requesting scheduling on CPU %d\n", next_cpu);
		state->linked = tsk;
		if (state->scheduled != tsk)
			litmus_reschedule(previous_cpu);
	} else if (state->linked == tsk) {
		TRACE_TASK(tsk, "requesting descheduling from CPU %d\n",
			   previous_cpu);
		state->linked = NULL;
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
		TRACE_TASK(tsk, "requesting scheduling on CPU %d\n", next_cpu);
		state->linked = tsk;
		if (needs_reschedule)
			litmus_reschedule(next_cpu);
		raw_spin_unlock(&state->lock);
	}

	// Set the timer to the next event (interval start or end, or task release)
	// if the interval is not NULL.
	if (tinfo->gtdinterval) {
		next_timer = is_inside_interval ? end : start;
		if (is_completed(tsk) &&
		    lt_before(get_release(tsk), next_timer)) {
			next_timer = get_release(tsk);
			WARN_ON(lt_before(next_timer, now));
		}
		TRACE_TASK(
			tsk,
			"computing next timer at %llu (now is %llu) for interval [%llu-%llu)\n",
			next_timer, now, start, end);
	} else {
		TRACE_TASK(tsk, "at %llu will no longer get timer interrupts\n",
			   now);
		next_timer = GTDRES_TIME_NA;
	}

	return next_timer;
}

// Act on interval timer
static enum hrtimer_restart on_interval_timer(struct hrtimer *timer)
{
	struct gmcres_task_state *tinfo =
		container_of(timer, struct gmcres_task_state, interval_timer);
	struct task_struct *tsk = tinfo->gtdres->task;
	lt_t next_timer;

	// hrtimer handlers are called from a IRQ handler, so IRQ are already disabled
	raw_spin_lock(&tinfo->lock);
	next_timer = task_scheduling_decision(tsk);
	if (next_timer != GTDRES_TIME_NA)
		hrtimer_set_expires(timer, ns_to_ktime(next_timer));
	raw_spin_unlock(&tinfo->lock);

	return next_timer == GTDRES_TIME_NA ? HRTIMER_NORESTART :
						    HRTIMER_RESTART;
}

// Ensure that the criticality mode is at least target_mode when exiting this function.
// If the global criticality mode is already at least at target_mode, return false. It
// does not mean that the mode change is completed, but that it has at least started
// already. If the mode change is necessary, force every task to reconsider its scheduling
// decisions and return true.
// When this function is called, no gmcres_task_state or gmcres_cpu_state locks must be
// held, as both will be taken during the rescheduling decisions if the mode actually
// changes.
static bool ensure_minimum_criticality_mode(unsigned int target_mode)
{
	struct gtd_reservation *gtdres;
	unsigned int current_mode;
	unsigned long flags;

	atomic_inc_return_acquire(&access_counter);

	current_mode = atomic_read(&system_criticality_mode);
	if (current_mode >= target_mode) {
		atomic_dec(&access_counter);
		return false;
	}
	// Warn about attempts to go up several levels at once; that
	// would indicate that some CPU or task is rescheduled very
	// late.
	TRACE_WARN_ON(current_mode + 1 < target_mode);
	if (atomic_cmpxchg(&system_criticality_mode, current_mode,
			   target_mode) != current_mode) {
		atomic_dec(&access_counter);
		return false;
	}
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
			next_timer = task_scheduling_decision(tsk);
			if (next_timer != GTDRES_TIME_NA)
				hrtimer_start(&tinfo->interval_timer,
					      ns_to_ktime(next_timer),
					      HRTIMER_MODE_ABS);
			else
				hrtimer_cancel(&tinfo->interval_timer);
			raw_spin_unlock(&tinfo->lock);
		}
	}
	raw_spin_unlock_irqrestore(&gtdenv.writer_lock, flags);

	atomic_dec_return_release(&access_counter);
	return true;
}

static struct task_struct *gmcres_schedule(struct task_struct *prev)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	lt_t now;

	raw_spin_lock(&state->lock);

	// Restart at this point if the system criticality mode changed while
	// this CPU has been busy selecting a task.
restart:

	// Check if we need to execute a requested criticality mode change.
	// We do this in a loop because as we unlock the state in order to
	// execute a mode change request another mode change request at an
	// upper level could come in as well.
	while (true) {
		unsigned int lcm;
		lcm = state->criticality_mode;
		if (lcm <= atomic_read(&system_criticality_mode))
			break;
		TRACE("will request a criticality mode change up to %u\n", lcm);
		// Unlock the CPU state around the criticality mode change as requested
		// by the function contract.
		raw_spin_unlock(&state->lock);
		ensure_minimum_criticality_mode(lcm);
		// Wait for a concurrent mode change to be over
		while (atomic_read_acquire(&access_counter))
			;
		// Relock the CPU state
		raw_spin_lock(&state->lock);
		// If this CPU did not receive new priority mode change requests in
		// the meantime, remember the current priority it has seen.
		if (state->criticality_mode == lcm) {
			state->criticality_mode =
				atomic_read_acquire(&system_criticality_mode);
			break;
		}
	}

	now = litmus_clock();

	// Sanity checks and previous task information
	WARN_ON(state->scheduled && state->scheduled != prev);
	state->prev_was_realtime = prev && is_realtime(prev);
	WARN_ON(state->scheduled && !state->prev_was_realtime);

	// Link the task we were asked to link
	if (state->scheduled != state->linked) {
		if (state->scheduled)
			TRACE_TASK(state->scheduled, "descheduled at %llu\n",
				   now);
		if (state->linked)
			TRACE_TASK(state->linked, "scheduled at %llu\n", now);
	}

	state->scheduled = state->linked;
	sched_state_task_picked();

	// Check if a system criticality mode change, or change in progress, has
	// not made this scheduling decision obsolete already, and restart the
	// process if it does.
	if (atomic_read(&system_criticality_mode) > state->criticality_mode ||
	    atomic_read(&access_counter)) {
		goto restart;
	}

	raw_spin_unlock(&state->lock);

	return state->scheduled;
}

static long gmcres_admit_task(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo;
	unsigned int res_id = tsk_rt(tsk)->task_params.cpu;
	struct gtd_reservation *gtdres = gtd_env_find(&gtdenv, res_id);
	lt_t period = get_rt_period(tsk);
	long ret = 0;

	if (!gtdres) {
		TRACE_TASK(tsk, "requires an unknown reservation %u\n", res_id);
		return -EINVAL;
	}

	// The task period must divide the reservation major cycle
	if (gtdres->major_cycle % period) {
		TRACE_TASK(
			tsk,
			"period (%llu) is not an integral factor of the reservation %d "
			"major cycle (%llu)\n",
			period, res_id, gtdres->major_cycle);
		return -EINVAL;
	}

	tinfo = kzalloc(sizeof(*tinfo), GFP_ATOMIC);
	if (!tinfo)
		return -ENOMEM;

	preempt_disable();
	raw_spin_lock_init(&tinfo->lock);
	tinfo->gtdres = gtdres;
	tinfo->gtdinterval = NULL;
	tinfo->major_cycle_start = GTDRES_TIME_NA;
	hrtimer_init(&tinfo->interval_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	tinfo->interval_timer.function = on_interval_timer;
	tsk_rt(tsk)->plugin_state = tinfo;

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
		if (atomic_cmpxchg(&maximum_criticality_level, mcl,
				   gtdres->criticality_level) == mcl)
			break;
	}

	gtdres->task = tsk;

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

// If required, advance interval_timer to the given deadline. This must be called
// with interrupts disabled and the task structure locked.
static void advance_interval_timer_at(struct task_struct *tsk, lt_t deadline)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	if (lt_before(deadline, ktime_to_ns(hrtimer_get_expires(
					&tinfo->interval_timer)))) {
		TRACE_TASK(
			current,
			"requires advancing timer since release is before next timer\n");
		hrtimer_start(&tinfo->interval_timer, ns_to_ktime(deadline),
			      HRTIMER_MODE_ABS);
	}
}

static void gmcres_task_new(struct task_struct *tsk, int on_runqueue,
			    int is_running)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	lt_t now, next_timer;
	unsigned long flags;
	bool is_inside_interval;
	int wanted_cpu, current_cpu;
	int mode;

	raw_spin_lock_irqsave(&tinfo->lock, flags);
	now = litmus_clock();

	mode = atomic_read(&system_criticality_mode);
	tinfo->criticality_mode = mode;
	is_inside_interval =
		gtd_reservation_find_interval(tinfo->gtdres, mode, now,
					      &tinfo->gtdinterval,
					      &tinfo->major_cycle_start);
	BUG_ON(!tinfo->gtdinterval);
	TRACE_TASK(
		tsk,
		"registers interest for interval [%llu-%llu) (is_inside_interval:%d)\n",
		tinfo->major_cycle_start + tinfo->gtdinterval->start,
		tinfo->major_cycle_start + tinfo->gtdinterval->end,
		is_inside_interval);
	next_timer = next_task_timer(tinfo, now);
	TRACE_TASK(tsk, "will set first timer for %llu (now is %llu)\n",
		   next_timer, now);
	hrtimer_start(&tinfo->interval_timer, ns_to_ktime(next_timer),
		      HRTIMER_MODE_ABS);

	TRACE_TASK(tsk,
		   "is a new RT task at %llu (on runqueue:%d, running:%d)\n",
		   now, on_runqueue, is_running);

	// Setup job parameters by aligning the release time to the next occurrence of the
	// task period.
	release_at(tsk, now - (now % get_rt_period(tsk)));

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
				   "will ask to get descheduled from CPU %d\n",
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
			TRACE_TASK(tsk, "is requesting to run on CPU %d\n",
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

	// Ensure that the timer can no longer fire
	hrtimer_cancel(&tinfo->interval_timer);

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	// Remove the task from the reservation so that it will no
	// longer be scheduled.
	raw_spin_lock_irqsave(&tinfo->gtdres->lock, flags);
	BUG_ON(tinfo->gtdres->task != tsk);
	tinfo->gtdres->task = NULL;
	tcl = tinfo->gtdres->criticality_level;
	raw_spin_unlock(&tinfo->gtdres->lock);

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
				"has been forcefully descheduled from CPU %d\n",
				cpu);
			state->scheduled = NULL;
		}
		if (state->linked == tsk) {
			TRACE_TASK(
				tsk,
				"has been forcefully derequested from CPU %d\n",
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
		TRACE("Invalid reservation (%u): CPU %d offline\n", config.id,
		      config.cpu);
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
		ret = gtd_reservation_add_interval(gtdres, interval.start,
						   interval.end, config.cpu);
		if (ret < 0) {
			TRACE("Cannot add interval [%llu-%llu] on CPU %d\n",
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
	TRACE_TASK(next, "has arrived on CPU %d at %llu\n", state->cpu,
		   litmus_clock());
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
	TRACE_TASK(next, "cannot be acquired now at %llu\n", litmus_clock());
	if (state->linked != next) {
		raw_spin_unlock_irqrestore(&state->lock, flags);
		TRACE_TASK(
			next,
			"should no longer scheduled on CPU %d anyway, giving up\n",
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

	TRACE_TASK(tsk, "blocked at %llu\n", litmus_clock());

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
	if (litmus_clock() >=
	    tinfo->major_cycle_start + tinfo->gtdinterval->start) {
		int expected_cpu = tinfo->gtdinterval->cpu;
		if (current_cpu != expected_cpu) {
			TRACE_TASK(
				tsk,
				"must be unscheduled/unrequested from CPU %d\n",
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

	TRACE_TASK(tsk, "is waking up at %llu\n", litmus_clock());

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	// If the task is in an interval, check the CPU it would like to be scheduled on
	expected_cpu = litmus_clock() >= tinfo->major_cycle_start +
						 tinfo->gtdinterval->start ?
				     tinfo->gtdinterval->cpu :
				     NO_CPU;

	// If another CPU than the current one is expected, mark the task as requested
	// and provoke a reschedule at both places.
	if (expected_cpu != NO_CPU) {
		TRACE_TASK(tsk, "is requesting to be scheduled on CPU %d\n",
			   expected_cpu);
		state = cpu_state_for(expected_cpu);
		raw_spin_lock(&state->lock);
		state->linked = tsk;
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

	atomic_set(&system_criticality_mode, 0);
	atomic_set(&access_counter, 0);
	atomic_set(&maximum_criticality_level, 0);

	gtd_env_init(&gtdenv);

	for_each_online_cpu (cpu) {
		TRACE("Initializing CPU%d...\n", cpu);

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
		   "has completed its job at %llu - "
		   "next release will be %llu, deadline will be %llu\n",
		   now, get_release(current), get_deadline(current));
	if (!is_released(current, now)) {
		struct gmcres_cpu_state *state = local_cpu_state();
		advance_interval_timer_at(current, get_release(current));
		raw_spin_lock(&state->lock);
		WARN_ON(state->scheduled != current);
		if (state->linked == current)
			state->linked = NULL;
		raw_spin_unlock(&state->lock);
		TRACE_TASK(current, "will ask CPU %d to deschedule it\n",
			   state->cpu);
		litmus_reschedule_local();
		tsk_rt(current)->completed = 1;
	} else {
		TRACE_TASK(
			current,
			"at %llu is already past its next release date %llu\n",
			now, get_release(current));
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
