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

	// The current (if in_interval is set) or next reservation and interval, as well
	// as the major_cycle_start offset to apply to the interval.
	bool in_interval;
	struct gtd_reservation *current_res;
	struct gtd_interval *interval;
	lt_t major_cycle_start;

	// True after scheduling if the task we have descheduled last was a real-time
	// task (i.e. if the task may be implicated in a deadlock between Litmus tasks).
	bool prev_was_realtime;

	// Timer set to the end of the current interval or beginning of the next interval.
	struct hrtimer scheduling_timer;

	// Task being current scheduled
	struct task_struct *scheduled;
};

static DEFINE_PER_CPU(struct gmcres_cpu_state, gmcres_cpu_state);

#define cpu_state_for(cpu_id) (&per_cpu(gmcres_cpu_state, cpu_id))
#define local_cpu_state() (this_cpu_ptr(&gmcres_cpu_state))

struct gmcres_task_state {
	// The reservation associated to this task, cannot be NULL once the task
	// has been admitted.
	struct gtd_reservation *gtdres;
	// The CPU this task is scheduled (or would be scheduled if not suspended) on.
	int cpu;
};

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

static enum hrtimer_restart on_scheduling_timer(struct hrtimer *timer)
{
	struct gmcres_cpu_state *state =
		container_of(timer, struct gmcres_cpu_state, scheduling_timer);
	WARN_ON(state->cpu != raw_smp_processor_id());
	TRACE("Scheduling timer event at %llu on CPU %d\n", litmus_clock(),
	      state->cpu);
	litmus_reschedule_local();
	return HRTIMER_NORESTART;
}

static struct task_struct *gmcres_schedule(struct task_struct *prev)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	lt_t now = litmus_clock();
	struct task_struct *tsk = NULL;

	raw_spin_lock(&state->lock);

	// Sanity checks and previous task information
	WARN_ON(state->scheduled && state->scheduled != prev);
	state->prev_was_realtime = prev && is_realtime(prev);

	// If the current interval we were in has expired or if we were not
	// in an interval, find a new interval.
	// TODO: can be optimized when we add chaining of per-CPU intervals
	if (!state->in_interval ||
	    now > state->major_cycle_start + state->interval->end) {
		lt_t old_start = GTDRES_TIME_NA, start = GTDRES_TIME_NA,
		     end = GTDRES_TIME_NA;
		bool in_interval, has_interval, interval_changed;

		// If the current CPU was already of a previous interval (being in it
		// or waiting for it), store its beginning (which is unique per CPU)
		// in order to be able to detect a change later.
		if (state->interval)
			old_start = state->major_cycle_start +
				    state->interval->start;

		in_interval = gtd_env_find_interval(&gtdenv, now, state->cpu,
						    &state->current_res,
						    &state->interval,
						    &state->major_cycle_start);
		has_interval = !!state->current_res;

		// Sanity checks:
		//  1- If we are in an interval, we must have a reservation
		//  2- We must either have a reservation, interval and valid major cycle start,
		//     or none at all.
		WARN_ON(in_interval && !has_interval);
		WARN_ON(has_interval ^ !!state->interval);
		WARN_ON(has_interval ^
			(state->major_cycle_start != GTDRES_TIME_NA));

		// Compute the offset start and end time of the current or next interval
		// if one is known.
		if (has_interval) {
			start = state->major_cycle_start +
				state->interval->start;
			end = state->major_cycle_start + state->interval->end;
		}

		// We consider an interval change as being either entering in an interval
		// or changing intervals. We should never be in a situation where we are
		// leaving an interval but the interval and major cycle start stay the same.
		WARN_ON(state->in_interval && !in_interval && has_interval &&
			old_start == start);
		interval_changed = (!state->in_interval && in_interval) ||
				   !has_interval || old_start != start;
		state->in_interval = in_interval;

		// If interval has changed, set up the timer to trigger at the next change event,
		// and indicate to the task (if any) that was or could have been executing its
		// next CPU target.
		if (interval_changed) {
			if (in_interval) {
				TRACE("CPU %d: at %llu, in interval [%llu-%llu] of res %u\n",
				      state->cpu, now, start, end,
				      state->current_res->res.id);
				// Ask to be scheduled at the end of the interval
				TRACE("scheduling end-of-interval timer at %llu\n",
				      end);
				hrtimer_start(&state->scheduling_timer,
					      ns_to_ktime(end),
					      HRTIMER_MODE_ABS_PINNED);
			} else if (state->current_res) {
				// We have found the next interval
				TRACE("CPU %d: at %llu, next interval is [%llu-%llu] of res %u\n",
				      state->cpu, now, start, end,
				      state->current_res->res.id);
				// Ask to be scheduled at the beginning of the interval
				TRACE("scheduling start-of-interval timer at %llu\n",
				      start);
				hrtimer_start(&state->scheduling_timer,
					      ns_to_ktime(start),
					      HRTIMER_MODE_ABS_PINNED);
			}
		}
	}

	// Select the task to run from the reservation
	if (state->in_interval) {
		raw_spin_lock(&state->current_res->lock);
		tsk = state->current_res->task;
		if (tsk) {
			// Don't schedule a task with NO_CPU, it didn't go through task_new yet
			// or it is beging destroyed.
			if (tsk->cpu == NO_CPU) {
				TRACE_TASK(
					tsk,
					"is not scheduled because it reads NO_CPU\n");
				tsk = NULL;
			} else {
				if (tsk->cpu != state->cpu)
					TRACE_TASK(
						tsk,
						"marked as requiring CPU %u instead of CPU %u\n",
						state->cpu, tsk->cpu);
				tsk->cpu = state->cpu;
				// If the task is not currently running, don't schedule it.
				if (tsk->state != TASK_RUNNING)
					tsk = NULL;
			}
		}
		raw_spin_unlock(&state->current_res->lock);
	}

	sched_state_task_picked();

	if (tsk && !is_released(tsk, litmus_clock()))
		tsk = NULL;

	if (state->scheduled != tsk) {
		if (state->scheduled)
			TRACE_TASK(state->scheduled, "is descheduled at %llu\n",
				   now);
		if (tsk)
			TRACE_TASK(tsk, "is scheduled at %llu\n", now);
		if (state->scheduled && !tsk)
			TRACE("entering idling mode at %llu\n", now);
	}

	state->scheduled = tsk;
	raw_spin_unlock(&state->lock);

	// XXXXX Last check
	if (state->scheduled && state->current_res->task != state->scheduled) {
		TRACE("Task has been obviously removed from reservation\n");
		WARN_ON(true);
		TRACE_TASK(state->scheduled, "This one");
		state->scheduled = NULL;
	}
	return state->scheduled;
}

static long gmcres_admit_task(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo;
	unsigned int res_id = tsk_rt(tsk)->task_params.cpu;
	struct gtd_reservation *gtdres = gtd_env_find(&gtdenv, res_id);

	if (!gtdres) {
		TRACE_TASK(tsk, "requires an unknown reservation %u\n", res_id);
		return -EINVAL;
	}

	// The task period must divide the reservation major cycle
	if (gtdres->major_cycle % tsk_rt(tsk)->task_params.period) {
		TRACE_TASK(
			tsk,
			"period (%llu) is not an integral factor of the reservation %d "
			"major cycle (%llu)\n",
			tsk_rt(tsk)->task_params.period, res_id,
			gtdres->major_cycle);
		return -EINVAL;
	}

	tinfo = kzalloc(sizeof(*tinfo), GFP_ATOMIC);
	if (!tinfo)
		return -ENOMEM;

	preempt_disable();
	tinfo->cpu = NO_CPU;
	tinfo->gtdres = gtdres;
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
		raw_spin_unlock(&gtdres->lock);
		kfree(tinfo);
		return -EPERM;
	}

	// The budget enforcement will be done on interval boundaries only
	tsk_rt(tsk)->task_params.budget_policy = NO_ENFORCEMENT;

	gtdres->task = tsk;
	raw_spin_unlock(&gtdres->lock);
	preempt_enable();

	TRACE_TASK(tsk, "has been accepted by the G-MCRES plugin\n");
	return 0;
}

static void gmcres_task_new(struct task_struct *tsk, int on_runqueue,
			    int is_running)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gtd_interval *interval;
	lt_t major_cycle_start, now = litmus_clock();
	bool is_inside_interval;
	unsigned long flags;

	TRACE_TASK(tsk,
		   "is a new RT task at %llu (on runqueue:%d, running:%d)\n",
		   now, on_runqueue, is_running);

	// Setup job parameters
	release_at(tsk, now);

	// Look for the next CPU that should be interested in this task
	is_inside_interval = gtd_reservation_find_interval(
		tinfo->gtdres, now, &interval, &major_cycle_start, NULL, NULL);
	BUG_ON(!interval);

	raw_spin_lock_irqsave(&state->lock, flags);
	if (is_running) {
		tinfo->cpu = state->cpu;
		state->scheduled = tsk;
		if (state->cpu != interval->cpu)
			litmus_reschedule(interval->cpu);
		litmus_reschedule_local();
	} else
		tinfo->cpu = interval->cpu;
	raw_spin_unlock_irqrestore(&state->lock, flags);
}

static void gmcres_task_exit(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gtd_reservation *gtdres;
	int cpu;
	unsigned long flags;

	TRACE_TASK(tsk, "exiting at %llu\n", litmus_clock());

	// Remove the task from the reservation so that it will no
	// longer be scheduled.
	gtdres = tinfo->gtdres;
	WARN_ON(gtdres->task != tsk);
	raw_spin_lock_irqsave(&gtdres->lock, flags);
	gtdres->task = NULL;
	tinfo->cpu = NO_CPU;
	raw_spin_unlock(&gtdres->lock);

	// If the task is scheduled on any CPU, deschedule it.
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
		raw_spin_unlock_irqrestore(&state->lock, flags);
	}
	WARN_ON(tinfo->cpu != NO_CPU);

	WRITE_ONCE(tsk_rt(tsk)->plugin_state, NULL);
	kfree(tinfo);
	local_irq_restore(flags);
}

static long gmcres_reservation_create(int res_type, void *__user _config)
{
	struct reservation_config config;
	struct gtd_reservation *res;
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
	ret = gtd_env_find_or_create(&gtdenv, &config, &res);
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
		ret = gtd_reservation_add_interval(res, interval.start,
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
	if (state->prev_was_realtime) {
		state->scheduled = NULL;
		raw_spin_unlock_irqrestore(&state->lock, flags);
		TRACE_TASK(next, "relinguished for now to avoid deadlock\n");
		return false;
	}
	raw_spin_unlock_irqrestore(&state->lock, flags);
	TRACE_TASK(next, "cannot be implicated in a deadlock, not giving up\n");
	return true;
}

static bool gmcres_fork_task(struct task_struct *tsk)
{
	TRACE_CUR("is forking, refusing\n");
	return false;
}

static void gmcres_task_block(struct task_struct *tsk)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	unsigned long flags;

	raw_spin_lock_irqsave(&state->lock, flags);
	TRACE_TASK(tsk, "suspends at %llu\n", litmus_clock());
	WARN_ON(state->scheduled != tsk);
	state->scheduled = NULL;
	raw_spin_unlock_irqrestore(&state->lock, flags);
}

static void gmcres_task_wake_up(struct task_struct *tsk)
{
	struct gmcres_task_state *tinfo = get_gmcres_task_state(tsk);
	struct gtd_reservation *gtdres = tinfo->gtdres;
	unsigned long flags;
	int cpu;

	BUG_ON(!gtdres);
	raw_spin_lock_irqsave(&gtdres->lock, flags);
	cpu = tinfo->cpu;
	if (cpu != NO_CPU) {
		BUG_ON(gtdres->task != tsk);
		TRACE_TASK(tsk,
			   "resumes at %llu, trying to reschedule CPU %d\n",
			   litmus_clock(), cpu);
		litmus_reschedule(cpu);
	} else {
		TRACE_TASK(
			tsk,
			"resuming but assigned to no CPU, no rescheduling\n");
	}
	raw_spin_unlock_irqrestore(&gtdres->lock, flags);
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

	gtd_env_init(&gtdenv);

	for_each_online_cpu (cpu) {
		TRACE("Initializing CPU%d...\n", cpu);

		state = cpu_state_for(cpu);
		memset(state, 0, sizeof(*state));
		state->cpu = cpu;
		hrtimer_init(&state->scheduling_timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_ABS);
		state->scheduling_timer.function = on_scheduling_timer;
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

	for_each_online_cpu (cpu) {
		state = cpu_state_for(cpu);
		raw_spin_lock(&state->lock);
		WARN_ON(state->scheduled);
		hrtimer_cancel(&state->scheduling_timer);
	}

	list_for_each_entry_safe (gtdres, p, &gtdenv.all_reservations,
				  all_reservations_list) {
		list_del(&gtdres->all_reservations_list);
		gtd_reservation_clear(gtdres);
		WARN_ON(gtdres->task);
		kfree(gtdres);
	}

	for_each_online_cpu (cpu) {
		state = cpu_state_for(cpu);
		raw_spin_unlock(&state->lock);
	}

	raw_spin_unlock(&gtdenv.writer_lock);

	destroy_domain_proc_info(&gmcres_domain_proc_info);

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
	.complete_job = complete_job_oneshot,
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
