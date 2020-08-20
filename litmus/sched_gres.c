#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <asm/uaccess.h>

#include <litmus/sched_plugin.h>
#include <litmus/preempt.h>
#include <litmus/debug_trace.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>
#include <litmus/budget.h>
#include <litmus/litmus_proc.h>
#include <litmus/sched_trace.h>

#include <litmus/reservations/reservation.h>
#include <litmus/reservations/alloc.h>
#include <litmus/reservations/table-driven.h>

struct reservation_list {
	struct list_head list;
	struct reservation *res;
	struct reservation_client *client;
	struct task_client res_info;
	int cpu;
};

struct gres_task_state {
	struct task_struct *tsk;
	struct list_head task_list;
	int hi_mode;
	struct list_head res_list;
	lt_t hi_exec_cost;
	struct list_head hi_res_list;
	int curr_cpu;
	struct hrtimer budget_timer;
};

struct gres_cpu_state {
	raw_spinlock_t lock;

	struct sup_reservation_environment sup_env;
	struct hrtimer timer;

	int cpu;
	struct task_struct* scheduled;
};

int hi_mode;

struct list_head task_list;

static DEFINE_PER_CPU(struct gres_cpu_state, gres_cpu_state);

#define cpu_state_for(cpu_id)	(&per_cpu(gres_cpu_state, cpu_id))
#define local_cpu_state()	(this_cpu_ptr(&gres_cpu_state))

static struct gres_task_state* get_gres_state(struct task_struct *tsk)
{
	return (struct gres_task_state*) tsk_rt(tsk)->plugin_state;
}

static void task_departs(struct task_struct *tsk, int job_complete)
{
	struct gres_task_state* state = get_gres_state(tsk);
	struct reservation* res;
	struct reservation_client *client;
	struct list_head *list_to_use = &state->res_list;
	struct reservation_list *rlist;
	
	if (state->hi_mode && !state->hi_exec_cost)
		return;

	if (state->hi_mode && state->hi_exec_cost)
		list_to_use = &state->hi_res_list;

	list_for_each_entry(rlist, list_to_use, list) {
		client = rlist->client;
		res    = client->reservation;

		res->ops->client_departs(res, client, job_complete);
		TRACE_TASK(tsk, "client_departs: removed from reservation R%d\n", res->id);
	}
}

static void task_arrives(struct task_struct *tsk)
{
	struct gres_task_state* state = get_gres_state(tsk);
	struct reservation* res;
	struct reservation_client *client;
	struct list_head *list_to_use = &state->res_list;
	struct reservation_list *rlist;

	if (state->hi_mode && !state->hi_exec_cost)
		return;

	if (state->hi_mode && state->hi_exec_cost)
		list_to_use = &state->hi_res_list;

	list_for_each_entry(rlist, list_to_use, list) {
		client = rlist->client;
		res    = client->reservation;

		res->ops->client_arrives(res, client);
		TRACE_TASK(tsk, "client_arrives: added to reservation R%d\n", res->id);
	}
}

/* NOTE: drops state->lock */
static void gres_update_timer_and_unlock(struct gres_cpu_state *state)
{
	int local;
	lt_t update, now;

	update = state->sup_env.next_scheduler_update;
	now = state->sup_env.env.current_time;

	/* Be sure we're actually running on the right core,
	 * as gres_update_timer() is also called from gres_task_resume(),
	 * which might be called on any CPU when a thread resumes.
	 */
	local = local_cpu_state() == state;

	/* Must drop state lock before calling into hrtimer_start(), which
	 * may raise a softirq, which in turn may wake ksoftirqd. */
	raw_spin_unlock(&state->lock);

	if (update <= now) {
		litmus_reschedule(state->cpu);
	} else if (likely(local && update != SUP_NO_SCHEDULER_UPDATE)) {
		/* Reprogram only if not already set correctly. */
		if (!hrtimer_active(&state->timer) ||
		    ktime_to_ns(hrtimer_get_expires(&state->timer)) != update) {
			TRACE("canceling timer...\n");
			hrtimer_cancel(&state->timer);
			TRACE("setting scheduler timer for %llu\n", update);
			hrtimer_start(&state->timer,
					ns_to_ktime(update),
					HRTIMER_MODE_ABS_PINNED);
			if (update < litmus_clock()) {
				/* uh oh, timer expired while trying to set it */
				TRACE("timer expired during setting "
				      "update:%llu now:%llu actual:%llu\n",
				      update, now, litmus_clock());
	 			/* The timer HW may not have been reprogrammed
	 			 * correctly; force rescheduling now. */
				litmus_reschedule(state->cpu);
			}
		}
	} else if (unlikely(!local && update != SUP_NO_SCHEDULER_UPDATE)) {
		/* Poke remote core only if timer needs to be set earlier than
		 * it is currently set.
		 */
		TRACE("gres_update_timer for remote CPU %d (update=%llu, "
		      "active:%d, set:%llu)\n",
			state->cpu,
			update,
			hrtimer_active(&state->timer),
			ktime_to_ns(hrtimer_get_expires(&state->timer)));
		if (!hrtimer_active(&state->timer) ||
		    ktime_to_ns(hrtimer_get_expires(&state->timer)) > update) {
			TRACE("poking CPU %d so that it can update its "
			       "scheduling timer (active:%d, set:%llu)\n",
			       state->cpu,
			       hrtimer_active(&state->timer),
			       ktime_to_ns(hrtimer_get_expires(&state->timer)));
			litmus_reschedule(state->cpu);
		}
	}
}

static enum hrtimer_restart on_scheduling_timer(struct hrtimer *timer)
{
	unsigned long flags;
	enum hrtimer_restart restart = HRTIMER_NORESTART;
	struct gres_cpu_state *state;
	lt_t update, now;

	state = container_of(timer, struct gres_cpu_state, timer);

	/* The scheduling timer should only fire on the local CPU, because
	 * otherwise deadlocks via timer_cancel() are possible.
	 * Note: this does not interfere with dedicated interrupt handling, as
	 * even under dedicated interrupt handling scheduling timers for
	 * budget enforcement must occur locally on each CPU.
	 */
	BUG_ON(state->cpu != raw_smp_processor_id());

	raw_spin_lock_irqsave(&state->lock, flags);
	sup_update_time(&state->sup_env, litmus_clock());

	update = state->sup_env.next_scheduler_update;
	now = state->sup_env.env.current_time;

	TRACE_CUR("on_scheduling_timer at %llu, upd:%llu (for cpu=%d)\n",
		now, update, state->cpu);

	if (update <= now) {
		litmus_reschedule_local();
	} else if (update != SUP_NO_SCHEDULER_UPDATE) {
		hrtimer_set_expires(timer, ns_to_ktime(update));
		restart = HRTIMER_RESTART;
	}

	raw_spin_unlock_irqrestore(&state->lock, flags);

	return restart;
}

static void mode_change(void) {
	int cpu;
	struct gres_cpu_state *state = local_cpu_state();
	struct gres_task_state *tinfo;
	struct reservation_list *rlist;
	struct reservation *res;

	raw_spin_lock(&state->lock);
	hi_mode = 1;
	raw_spin_unlock(&state->lock);

	list_for_each_entry(tinfo, &task_list, task_list) {
		state = cpu_state_for(tinfo->curr_cpu);
		raw_spin_lock(&state->lock);
		hrtimer_try_to_cancel(&tinfo->budget_timer);
		tsk_rt(tinfo->tsk)->task_params.exec_cost = tinfo->hi_exec_cost * 1000000;
		if (is_present(tinfo->tsk))
			task_departs(tinfo->tsk, 0);
		list_for_each_entry(rlist, &tinfo->res_list, list) {
			res = rlist->res;
			res->env->change_state(res->env, res, RESERVATION_INACTIVE);
		}
		tinfo->hi_mode = 1;
		if (is_present(tinfo->tsk) && tinfo->hi_exec_cost)
			task_arrives(tinfo->tsk);
		raw_spin_unlock(&state->lock);
	}

	for_each_online_cpu(cpu) {
		state = cpu_state_for(cpu);
		raw_spin_lock(&state->lock);
		litmus_reschedule(cpu);
		raw_spin_unlock(&state->lock);
	}
}

static enum hrtimer_restart on_budget_overrun(struct hrtimer *timer)
{
	if (hi_mode == 1) {
		TRACE("budget overrun in HI mode... huho\n");
		return HRTIMER_NORESTART;
	}

	TRACE("budget overrun!\n");
	mode_change();
	return HRTIMER_NORESTART;
}

static struct task_struct* gres_schedule(struct task_struct * prev)
{
	/* next == NULL means "schedule background work". */
	struct gres_cpu_state *state = local_cpu_state();
	struct gres_task_state *tinfo;
	lt_t when_to_fire;

	raw_spin_lock(&state->lock);

	BUG_ON(state->scheduled && state->scheduled != prev);
	BUG_ON(state->scheduled && !is_realtime(prev));

	/* update time */
	state->sup_env.will_schedule = true;
	sup_update_time(&state->sup_env, litmus_clock());

	/* figure out what to schedule next */
	state->scheduled = sup_dispatch(&state->sup_env);

	/* Notify LITMUS^RT core that we've arrived at a scheduling decision. */
	sched_state_task_picked();

	/* program scheduler timer */
	state->sup_env.will_schedule = false;
	/* NOTE: drops state->lock */
	gres_update_timer_and_unlock(state);

	if (prev != state->scheduled && is_realtime(prev)) {
		TRACE_TASK(prev, "descheduled.\n");
		tinfo = get_gres_state(prev);
		hrtimer_try_to_cancel(&tinfo->budget_timer);
	}
	if (state->scheduled) {
		TRACE_TASK(state->scheduled, "scheduled.\n");
		tinfo = get_gres_state(state->scheduled);
		tinfo->curr_cpu = state->cpu;
		when_to_fire = litmus_clock() + budget_remaining(state->scheduled);
		hrtimer_start(&tinfo->budget_timer, ns_to_ktime(when_to_fire),
				HRTIMER_MODE_ABS);
	}

	return state->scheduled;
}

static void resume_legacy_task_model_updates(struct task_struct *tsk)
{
	lt_t now;
	if (is_sporadic(tsk)) {
		/* If this sporadic task was gone for a "long" time and woke up past
		 * its deadline, then give it a new budget by triggering a job
		 * release. This is purely cosmetic and has no effect on the
		 * P-RES scheduler. */

		now = litmus_clock();
		if (is_tardy(tsk, now)) {
			inferred_sporadic_job_release_at(tsk, now);
		}
	}
}


/* Called when a task should be removed from the ready queue.
 */
static void gres_task_block(struct task_struct *tsk)
{
	unsigned long flags;
	struct gres_task_state* tinfo = get_gres_state(tsk);
	struct gres_cpu_state *state = cpu_state_for(tinfo->curr_cpu);

	TRACE_TASK(tsk, "thread suspends at %llu (state:%d, running:%d)\n",
		litmus_clock(), tsk->state, is_current_running());

	raw_spin_lock_irqsave(&state->lock, flags);
	sup_update_time(&state->sup_env, litmus_clock());
	task_departs(tsk, is_completed(tsk));
	raw_spin_unlock_irqrestore(&state->lock, flags);
}


/* Called when the state of tsk changes back to TASK_RUNNING.
 * We need to requeue the task.
 */
static void gres_task_resume(struct task_struct  *tsk)
{
	unsigned long flags;
	struct gres_task_state* tinfo = get_gres_state(tsk);
	struct gres_cpu_state *state = cpu_state_for(tinfo->curr_cpu);

	TRACE_TASK(tsk, "thread wakes up at %llu\n", litmus_clock());

	raw_spin_lock_irqsave(&state->lock, flags);
	/* Assumption: litmus_clock() is synchronized across cores,
	 * since we might not actually be executing on tinfo->curr_cpu
	 * at the moment. */
	sup_update_time(&state->sup_env, litmus_clock());
	task_arrives(tsk);
	/* NOTE: drops state->lock */
	gres_update_timer_and_unlock(state);
	local_irq_restore(flags);

	resume_legacy_task_model_updates(tsk);
}

static long gres_admit_task(struct task_struct *tsk)
{
	long err = -EINVAL;
	unsigned long flags;
	struct reservation *res;
	struct reservation_list *new_res = NULL;
	struct gres_cpu_state *state;
	struct gres_task_state *tinfo = kzalloc(sizeof(*tinfo), GFP_ATOMIC);
	int cpu;

	if (!tinfo)
		return -ENOMEM;

	preempt_disable();

	/* NOTE: this is obviously racy w.r.t. affinity changes since
	 *       we are not holding any runqueue locks. */
	if (tsk->nr_cpus_allowed != 1) {
		printk(KERN_WARNING "%s/%d: task does not have "
		       "singleton affinity mask\n",
			tsk->comm, tsk->pid);
		state = cpu_state_for(task_cpu(tsk));
	} else {
		state = cpu_state_for(cpumask_first(&tsk->cpus_allowed));
	}

	TRACE_TASK(tsk, "on CPU %d, valid?:%d\n",
		task_cpu(tsk), cpumask_test_cpu(task_cpu(tsk), &tsk->cpus_allowed));

	tinfo->tsk = tsk;
	tinfo->hi_mode = hi_mode;
	tinfo->curr_cpu = task_cpu(tsk);
	hrtimer_init(&tinfo->budget_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	tinfo->budget_timer.function = on_budget_overrun;
	INIT_LIST_HEAD(&tinfo->res_list);
	tinfo->hi_exec_cost = tsk_rt(tsk)->task_params.hi_exec_cost;
	if (tinfo->hi_exec_cost)
		INIT_LIST_HEAD(&tinfo->hi_res_list);

	for_each_online_cpu(cpu) {
		state = cpu_state_for(cpu);
		raw_spin_lock_irqsave(&state->lock, flags);

		res = sup_find_by_id(&state->sup_env, tsk_rt(tsk)->task_params.cpu);

		/* found the appropriate reservation (or vCPU) */
		if (res) {
			new_res = kzalloc(sizeof(*new_res), GFP_ATOMIC);
			new_res->res = res;
			new_res->cpu = cpu;
			task_client_init(&new_res->res_info, tsk, res);
			new_res->client = &new_res->res_info.client;
			list_add(&new_res->list, &tinfo->res_list);
			err = 0;
		}

		if (tinfo->hi_exec_cost) {
			res = sup_find_by_id(&state->sup_env,
					tsk_rt(tsk)->task_params.hi_res_id);
			if (res) {
				new_res = kzalloc(sizeof(*new_res), GFP_ATOMIC);
				new_res->res = res;
				new_res->cpu = cpu;
				task_client_init(&new_res->res_info, tsk, res);
				new_res->client = &new_res->res_info.client;
				list_add(&new_res->list, &tinfo->hi_res_list);
			}
		}

		raw_spin_unlock_irqrestore(&state->lock, flags);
	}

	if (!err) {
		state = cpu_state_for(tinfo->curr_cpu);
		raw_spin_lock_irqsave(&state->lock, flags);

		tsk_rt(tsk)->plugin_state = tinfo;
		list_add(&tinfo->task_list, &task_list);

		/* disable LITMUS^RT's per-thread budget enforcement */
		tsk_rt(tsk)->task_params.budget_policy = NO_ENFORCEMENT;
		raw_spin_unlock_irqrestore(&state->lock, flags);
	}

	preempt_enable();

	if (err)
		kfree(tinfo);

	return err;
}

static void task_new_legacy_task_model_updates(struct task_struct *tsk)
{
	lt_t now = litmus_clock();

	/* the first job exists starting as of right now */
	release_at(tsk, now);
	sched_trace_task_release(tsk);
}

static void gres_task_new(struct task_struct *tsk, int on_runqueue,
			  int is_running)
{
	unsigned long flags;
	struct gres_task_state* tinfo = get_gres_state(tsk);
	struct gres_cpu_state *state = cpu_state_for(tinfo->curr_cpu);

	TRACE_TASK(tsk, "new RT task %llu (on_rq:%d, running:%d)\n",
		   litmus_clock(), on_runqueue, is_running);

	/* acquire the lock protecting the state and disable interrupts */
	raw_spin_lock_irqsave(&state->lock, flags);

	if (is_running) {
		state->scheduled = tsk;
		/* make sure this task should actually be running */
		litmus_reschedule_local();
	}

	if (on_runqueue || is_running) {
		/* Assumption: litmus_clock() is synchronized across cores
		 * [see comment in gres_task_resume()] */
		sup_update_time(&state->sup_env, litmus_clock());

		if (!hi_mode && tinfo->hi_exec_cost) {
			tinfo->hi_mode = 1;
			task_arrives(tsk);
			task_departs(tsk,0);
			tinfo->hi_mode = 0;
		}

		task_arrives(tsk);
		/* NOTE: drops state->lock */
		gres_update_timer_and_unlock(state);
		local_irq_restore(flags);
	} else
		raw_spin_unlock_irqrestore(&state->lock, flags);

	task_new_legacy_task_model_updates(tsk);
}

static bool gres_fork_task(struct task_struct *tsk)
{
	TRACE_CUR("is forking\n");
	TRACE_TASK(tsk, "forked child rt:%d cpu:%d task_cpu:%d "
		        "wcet:%llu per:%llu\n",
		is_realtime(tsk),
		tsk_rt(tsk)->task_params.cpu,
		task_cpu(tsk),
		tsk_rt(tsk)->task_params.exec_cost,
		tsk_rt(tsk)->task_params.period);

	/* We always allow forking. */
	/* The newly forked task will be in the same reservation. */
	return true;
}

static void gres_task_exit(struct task_struct *tsk)
{
	unsigned long flags;
	struct gres_task_state* tinfo = get_gres_state(tsk);
	struct gres_cpu_state *state = cpu_state_for(tinfo->curr_cpu);
	struct reservation_list *rlist;

	raw_spin_lock_irqsave(&state->lock, flags);

	TRACE_TASK(tsk, "task exits at %llu (present:%d sched:%d)\n",
		litmus_clock(), is_present(tsk), state->scheduled == tsk);

	hrtimer_cancel(&tinfo->budget_timer);

	if (state->scheduled == tsk)
		state->scheduled = NULL;

	/* remove from queues */
	if (is_present(tsk)) {
		/* Assumption: litmus_clock() is synchronized across cores
		 * [see comment in gres_task_resume()] */
		sup_update_time(&state->sup_env, litmus_clock());
		task_departs(tsk, 0);
		/* NOTE: drops state->lock */
		gres_update_timer_and_unlock(state);
		local_irq_restore(flags);
	} else
		raw_spin_unlock_irqrestore(&state->lock, flags);

	while (!list_empty(&tinfo->res_list)) {
		rlist = list_first_entry(&tinfo->res_list, struct reservation_list, list);
		list_del(&rlist->list);
		kfree(rlist);
	}

	if (tinfo->hi_exec_cost) {
		while (!list_empty(&tinfo->hi_res_list)) {
			rlist = list_first_entry(&tinfo->hi_res_list,
					struct reservation_list, list);
			list_del(&rlist->list);
			kfree(rlist);
		}
	}

	list_del(&tinfo->task_list);
	kfree(tsk_rt(tsk)->plugin_state);
	tsk_rt(tsk)->plugin_state = NULL;
}

static void gres_current_budget(lt_t *used_so_far, lt_t *remaining)
{
	struct gres_task_state *tstate = get_gres_state(current);
	struct gres_cpu_state *state;
	struct reservation_list *rlist;
	struct reservation *res = NULL;

	/* FIXME: protect against concurrent task_exit() */

	local_irq_disable();

	state = cpu_state_for(tstate->curr_cpu);

	raw_spin_lock(&state->lock);

	sup_update_time(&state->sup_env, litmus_clock());
	list_for_each_entry(rlist, &tstate->res_list, list) {
		if (rlist->cpu == state->cpu)
			res = rlist->res;
	}
	if (remaining)
		*remaining = res->cur_budget;
	if (used_so_far)
		*used_so_far = res->budget_consumed;
	gres_update_timer_and_unlock(state);

	local_irq_enable();
}

static long do_gres_reservation_create(
	int res_type,
	struct reservation_config *config)
{
	struct gres_cpu_state *state;
	struct reservation* res;
	struct reservation* new_res = NULL;
	unsigned long flags;
	long err;

	/* Allocate before we grab a spin lock. */
	switch (res_type) {
		case PERIODIC_POLLING:
		case SPORADIC_POLLING:
			err = alloc_polling_reservation(res_type, config, &new_res);
			break;

		case TABLE_DRIVEN:
			err = alloc_table_driven_reservation(config, &new_res);
			break;

		default:
			err = -EINVAL;
			break;
	}

	if (err)
		return err;

	state = cpu_state_for(config->cpu);
	raw_spin_lock_irqsave(&state->lock, flags);

	res = sup_find_by_id(&state->sup_env, config->id);
	if (!res) {
		sup_add_new_reservation(&state->sup_env, new_res);
		err = config->id;
	} else {
		err = -EEXIST;
	}

	raw_spin_unlock_irqrestore(&state->lock, flags);

	if (err < 0)
		kfree(new_res);

	return err;
}

static long gres_reservation_create(int res_type, void* __user _config)
{
	struct reservation_config config;

	TRACE("Attempt to create reservation (%d)\n", res_type);

	if (copy_from_user(&config, _config, sizeof(config)))
		return -EFAULT;

	if (config.cpu < 0 || !cpu_online(config.cpu)) {
		printk(KERN_ERR "invalid polling reservation (%u): "
		       "CPU %d offline\n", config.id, config.cpu);
		return -EINVAL;
	}

	return do_gres_reservation_create(res_type, &config);
}

static struct domain_proc_info gres_domain_proc_info;

static long gres_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &gres_domain_proc_info;
	return 0;
}

static void gres_setup_domain_proc(void)
{
	int i, cpu;
	int num_rt_cpus = num_online_cpus();

	struct cd_mapping *map;

	memset(&gres_domain_proc_info, 0, sizeof(gres_domain_proc_info));
	init_domain_proc_info(&gres_domain_proc_info, num_rt_cpus, 1);
	gres_domain_proc_info.num_cpus = num_rt_cpus;
	gres_domain_proc_info.num_domains = 1;

	gres_domain_proc_info.domain_to_cpus[0].id = 0;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		map = &gres_domain_proc_info.cpu_to_domains[i];
		map->id = cpu;
		cpumask_set_cpu(0, map->mask);
		++i;

		cpumask_set_cpu(cpu, gres_domain_proc_info.domain_to_cpus[0].mask);
	}
}

static long gres_activate_plugin(void)
{
	int cpu;
	struct gres_cpu_state *state;

	hi_mode = 0;
	INIT_LIST_HEAD(&task_list);

	for_each_online_cpu(cpu) {
		TRACE("Initializing CPU%d...\n", cpu);

		state = cpu_state_for(cpu);

		raw_spin_lock_init(&state->lock);
		state->cpu = cpu;
		state->scheduled = NULL;

		sup_init(&state->sup_env);

		hrtimer_init(&state->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_PINNED);
		state->timer.function = on_scheduling_timer;
	}

	gres_setup_domain_proc();

	return 0;
}

static long gres_deactivate_plugin(void)
{
	int cpu;
	struct gres_cpu_state *state;
	struct reservation *res;

	for_each_online_cpu(cpu) {
		state = cpu_state_for(cpu);
		raw_spin_lock(&state->lock);

		hrtimer_cancel(&state->timer);

		/* Delete all reservations --- assumes struct reservation
		 * is prefix of containing struct. */

		while (!list_empty(&state->sup_env.all_reservations)) {
			res = list_first_entry(
				&state->sup_env.all_reservations,
			        struct reservation, all_list);
			list_del(&res->all_list);
			if (res->ops->shutdown)
				res->ops->shutdown(res);
			kfree(res);
		}

		raw_spin_unlock(&state->lock);
	}

	destroy_domain_proc_info(&gres_domain_proc_info);
	return 0;
}

static struct sched_plugin gres_plugin = {
	.plugin_name		= "G-RES",
	.schedule		= gres_schedule,
	.task_block		= gres_task_block,
	.task_wake_up		= gres_task_resume,
	.admit_task		= gres_admit_task,
	.task_new		= gres_task_new,
	.fork_task		= gres_fork_task,
	.task_exit		= gres_task_exit,
	.complete_job           = complete_job_oneshot,
	.get_domain_proc_info   = gres_get_domain_proc_info,
	.activate_plugin	= gres_activate_plugin,
	.deactivate_plugin      = gres_deactivate_plugin,
	.reservation_create     = gres_reservation_create,
	.current_budget         = gres_current_budget,
};

static int __init init_gres(void)
{
	return register_sched_plugin(&gres_plugin);
}

module_init(init_gres);

