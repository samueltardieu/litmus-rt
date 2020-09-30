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
	// the associated major cycle start time.
	struct gtd_interval *gtdinterval;
	lt_t major_cycle_start;

	// The timer associated with beginning or end of intervals, is always
	// armed when the task has been admitted.
	struct hrtimer interval_timer;
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

static enum hrtimer_restart on_interval_timer(struct hrtimer *timer)
{
	struct gmcres_task_state *tinfo =
		container_of(timer, struct gmcres_task_state, interval_timer);
	struct task_struct *tsk = tinfo->gtdres->task;
	struct gmcres_cpu_state *state;
	lt_t now, start, end;
	unsigned long flags;
	int previous_cpu, next_cpu;
	bool is_inside_interval, is_running, is_between_jobs;

	raw_spin_lock_irqsave(&tinfo->lock, flags);
	now = litmus_clock();

	// Is the task currently running?
	is_running = tsk->state == TASK_RUNNING;

	// Remember the previous CPU this task could have been scheduled on
	previous_cpu = tinfo->gtdinterval->cpu;

	TRACE_TASK(tsk,
		   "(is_running:%d) executes on_interval_timer at %llu "
		   "for interval [%llu-%llu) (cpu %d)\n",
		   is_running, now,
		   tinfo->major_cycle_start + tinfo->gtdinterval->start,
		   tinfo->major_cycle_start + tinfo->gtdinterval->end,
		   previous_cpu);

	// Advance intervals as much as needed and compute start, end, is_inside_interval
	while ((end = tinfo->major_cycle_start + tinfo->gtdinterval->end) <=
	       now)
		tinfo->gtdinterval = gtd_reservation_next_interval(
			tinfo->gtdres, tinfo->gtdinterval,
			&tinfo->major_cycle_start);
	start = tinfo->major_cycle_start + tinfo->gtdinterval->start;
	is_inside_interval = now >= start;

	TRACE_TASK(
		tsk,
		"has computed new interval [%llu-%llu) (cpu %d) (is_inside_interval:%d)\n",
		start, end, tinfo->gtdinterval->cpu, is_inside_interval);

	// Check if the task is between jobs
	is_between_jobs =
		tsk_rt(tsk)->completed && tsk_rt(tsk)->job_params.release > now;

	// Check the next CPU to use, or NO_CPU
	next_cpu = is_inside_interval && is_running && !is_between_jobs ?
				 tinfo->gtdinterval->cpu :
				 NO_CPU;

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
	// Note: if we have state->linked != tsk and state->scheduled == tsk,
	// then another task has requested a rescheduling in order to satisfy
	// its recent "linked" requirement, no need to schedule an extra one.
	raw_spin_unlock(&state->lock);

	if (next_cpu != previous_cpu && next_cpu != NO_CPU) {
		state = cpu_state_for(next_cpu);
		raw_spin_lock(&state->lock);
		TRACE_TASK(tsk, "requesting scheduling on CPU %d\n", next_cpu);
		state->linked = tsk;
		litmus_reschedule(next_cpu);
		raw_spin_unlock(&state->lock);
	}
	raw_spin_unlock_irqrestore(&tinfo->lock, flags);

	// Set next timer
	TRACE_TASK(
		tsk,
		"setting next timer at %llu to interval [%llu-%llu) %s: %llu\n",
		now, start, end, is_inside_interval ? "end" : "start",
		is_inside_interval ? end : start);
	hrtimer_set_expires(timer,
			    ns_to_ktime(is_inside_interval ? end : start));

	return HRTIMER_RESTART;
}

static struct task_struct *gmcres_schedule(struct task_struct *prev)
{
	struct gmcres_cpu_state *state = local_cpu_state();
	lt_t now;

	raw_spin_lock(&state->lock);

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

	raw_spin_unlock(&state->lock);

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
		raw_spin_unlock(&gtdres->lock);
		kfree(tinfo);
		return -EPERM;
	}

	// The budget enforcement will be done on interval boundaries only
	tsk_rt(tsk)->task_params.budget_policy = NO_ENFORCEMENT;

	gtdres->task = tsk;
	raw_spin_unlock(&gtdres->lock);
	preempt_enable();

	return 0;
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

	raw_spin_lock_irqsave(&tinfo->lock, flags);
	now = litmus_clock();
	is_inside_interval = gtd_reservation_find_interval(
		tinfo->gtdres, now, &tinfo->gtdinterval,
		&tinfo->major_cycle_start, NULL, NULL);
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

	TRACE_TASK(tsk, "exiting at %llu\n", litmus_clock());

	// Ensure that the timer can no longer fire
	hrtimer_cancel(&tinfo->interval_timer);

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	// Remove the task from the reservation so that it will no
	// longer be scheduled.
	raw_spin_lock_irqsave(&tinfo->gtdres->lock, flags);
	BUG_ON(tinfo->gtdres->task != tsk);
	tinfo->gtdres->task = NULL;
	raw_spin_unlock(&tinfo->gtdres->lock);

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
	struct rt_param *rt_param = tsk_rt(current);
	lt_t now;
	unsigned long flags;

	raw_spin_lock_irqsave(&tinfo->lock, flags);

	prepare_for_next_period(current);
	now = litmus_clock();
	TRACE_TASK(current,
		   "has completed its job at %llu - "
		   "next release will be %llu, deadline will be %llu\n",
		   now, rt_param->job_params.release,
		   rt_param->job_params.deadline);
	if (rt_param->job_params.release >= now) {
		struct gmcres_cpu_state *state = local_cpu_state();
		advance_interval_timer_at(current,
					  rt_param->job_params.release);
		raw_spin_lock(&state->lock);
		WARN_ON(state->scheduled != current);
		if (state->linked == current)
			state->linked = NULL;
		raw_spin_unlock(&state->lock);
		TRACE_TASK(current, "will ask CPU %d to deschedule it\n",
			   state->cpu);
		litmus_reschedule_local();
		rt_param->completed = 1;
	} else {
		TRACE_TASK(
			current,
			"at %llu is already past its next release date %llu\n",
			now, rt_param->job_params.release);
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
