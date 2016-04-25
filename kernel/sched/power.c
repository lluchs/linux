#include <linux/percpu.h>
#include <linux/topology.h>
#include <linux/types.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <stdbool.h>

#include "power.h"
#include "armpmu_lib.h"

#define POWER_UPDATE_INTERVAL 1000 // msec

struct current_energy_usage {
	s64 joule;
	s64 watt; // updated once per POWER_UPDATE_INTERVAL
	unsigned long time; // in jiffies
	bool disabled;
};

// Last energy usage of each CPU.
DEFINE_PER_CPU(struct current_energy_usage, current_energy_usage);
// Maximum energy usage per CPU in nW.
DEFINE_PER_CPU(s64, power_limit);

// A7 cores always come first. Disabling CPUs doesn't change the indices.
#define IS_A7(cpu) (cpu < 4)

struct energy_model {
	// Event number or -1 for PMCCNTR
	int event;
	s64 weight;
};

#define EM_PMCCNTR -1
#define EM_LAST -2
#define EM_MAX_SIZE 8 // A15: 6 counters + PMCCNTR + EM_LAST

#define EM_BASE 1e-12
#define EM_TO_INT(x) (s64) (x / EM_BASE)

// Remember to fix PMCCNTR

static const struct energy_model energy_model_a15[] = {
	{ .event = ASE_SPEC         , .weight = EM_TO_INT(6.448446679859954e-06   )} ,
	{ .event = BR_MIS_PRED      , .weight = EM_TO_INT(-1.3116397823286028e-07 )} ,
	{ .event = DP_SPEC          , .weight = EM_TO_INT(2.4606358411235e-10     )}  ,
	{ .event = L2D_CACHE_REFILL , .weight = EM_TO_INT(1.5813244507839535e-06  )}  ,
	{ .event = L2D_CACHE_WB     , .weight = EM_TO_INT(-8.824135849354271e-06  )} ,
	{ .event = EM_PMCCNTR       , .weight = EM_TO_INT(7.601199539578169e-10   )} ,
	{ .event = VFP_SPEC         , .weight = EM_TO_INT(1.5849463107519799e-09  )}  ,
	{ .event = EM_LAST }
};

static const struct energy_model energy_model_a7[] = {
	{ .event = BR_MIS_PRED      , .weight = EM_TO_INT(6.166023259107466e-10  )} ,
	{ .event = L1D_TLB_REFILL   , .weight = EM_TO_INT(3.252129874527141e-08  )}  ,
	{ .event = L2D_CACHE_REFILL , .weight = EM_TO_INT(-5.591860964520609e-08 )}  ,
	{ .event = L2D_CACHE_WB     , .weight = EM_TO_INT(1.8150459114876734e-07 )} ,
	{ .event = EM_PMCCNTR       , .weight = EM_TO_INT(1.0141460676251428e-10 )} ,
	{ .event = EM_LAST }
};

static inline const struct energy_model * energy_model(int cpu) { return IS_A7(cpu) ? energy_model_a7 : energy_model_a15; }

// Initialize counters used by the energy model.
static void initialize_pmu(int cpu)
{
	const struct energy_model *model = energy_model(cpu);

	int i, j;
	for (i = 0, j = 0; model[i].event != EM_LAST; i++) {
		if (model[i].event == EM_PMCCNTR) {
			// j fixes indexing of the performance counters.
			j = -1;
		} else {
			set_pmn(i + j, model[i].event);
		}
	}

}

// Actual implementation of the energy model evaluation.
static s64 evaluate_model(int cpu)
{
	s64 result = 0;
	const struct energy_model *model = energy_model(cpu);

	u32 counter;
	int i, j;
	for (i = 0, j = 0; model[i].event != EM_LAST; i++) {
		if (model[i].event == EM_PMCCNTR) {
			MRC_PMU(counter, PMCCNTR);
			// j fixes indexing of the performance counters.
			j = -1;
		} else {
			counter = read_pmn(i + j);
		}
		result += model[i].weight * counter;
	}
	return result;
}

// Calculate the current energy usage across all processors.
s64 total_current_energy_usage()
{
	s64 result = 0;
	int cpu;
	for_each_online_cpu(cpu) {
		result += per_cpu(current_energy_usage, cpu).joule;
	}
	return result;
}

// from drivers/base/cpu.c
static void change_cpu_under_node(struct cpu *cpu,
            unsigned int from_nid, unsigned int to_nid)
{
	int cpuid = cpu->dev.id;
	unregister_cpu_under_node(cpuid, from_nid);
	register_cpu_under_node(cpuid, to_nid);
	cpu->node_id = to_nid;
}

// Enables or disables a CPU. Returns 0 on success.
static int power_set_cpu_online(int cpuid, bool online)
{
	// from drivers/base/cpu.c
	struct device *dev = get_cpu_device(cpuid);
	struct cpu *cpu = container_of(dev, struct cpu, dev);
	int from_nid, to_nid;
	int ret;
	cpu_hotplug_driver_lock();
	if (!online) {
		ret = cpu_down(cpuid);
		if (!ret)
			kobject_uevent(&dev->kobj, KOBJ_OFFLINE);
	} else {
		from_nid = cpu_to_node(cpuid);
		ret = cpu_up(cpuid);

		/*
		 * When hot adding memory to memoryless node and enabling a cpu
		 * on the node, node number of the cpu may internally change.
		 */
		to_nid = cpu_to_node(cpuid);
		if (from_nid != to_nid)
			change_cpu_under_node(cpu, from_nid, to_nid);

		if (!ret)
			kobject_uevent(&dev->kobj, KOBJ_ONLINE);
	}
	cpu_hotplug_driver_unlock();
	return ret;
}

static struct task_struct *thread;

// Kernel thread for limiting.
static int powerlimitd(void *p)
{
	struct cpufreq_policy *policy = p;
	int cpu;
	struct current_energy_usage *usage;
	s64 limit, total_usage;
	unsigned int time_diff;
	while (!kthread_should_stop()) {
		limit = 0; total_usage = 0;
		// Frequency governors work on groups of CPUs. In our case, we
		// have all A7 cores and all A15 cores.
		for_each_cpu(cpu, policy->related_cpus) {
			limit += per_cpu(power_limit, cpu);
			usage = &per_cpu(current_energy_usage, cpu);
			total_usage += usage->joule;
		}

		if (limit > 0) {
			if (limit < div64_s64(total_usage, POWER_UPDATE_INTERVAL)) {
				cpufreq_driver_target(policy, policy->min, CPUFREQ_RELATION_L);
			} else {
				cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_L);
			}
		} else {
			cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_L);
		}

		msleep(100);
	}
	return 0;
}

static int cpufreq_governor_pmu(struct cpufreq_policy *policy, unsigned int event)
{
        switch (event) {
        case CPUFREQ_GOV_START:
		thread = kthread_run(powerlimitd, policy, "powerlimitd");
		if (IS_ERR(thread)) {
			printk(KERN_ERR "power: unable to create limiting thread\n");
		}
		// Run at maximum frequency per default.
                __cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_L);
                break;
	case CPUFREQ_GOV_STOP:
		kthread_stop(thread);
		break;
        default:
                break;
        }
        return 0;
}

struct cpufreq_governor cpufreq_gov_pmu = {
        .name = "pmugov",
        .governor = cpufreq_governor_pmu,
        .owner = THIS_MODULE,
};

static int __init cpufreq_gov_pmu_init(void)
{
        return cpufreq_register_governor(&cpufreq_gov_pmu);
}

static void __exit cpufreq_gov_pmu_exit(void)
{
        cpufreq_unregister_governor(&cpufreq_gov_pmu);
}

module_init(cpufreq_gov_pmu_init);
module_exit(cpufreq_gov_pmu_exit);

// Entry point from the scheduler: Evaluates performance counters.
void power_evaluate_pmu(int cpu)
{
	u32 cr, userenr;
	struct current_energy_usage *usage = &get_cpu_var(current_energy_usage);
	unsigned int time_diff;

	// Disable power evaluation if user-mode access is enabled.
	MRC_PMU(userenr, PMUSERENR);
	if (userenr) {
		usage->watt = usage->joule = 0;
		usage->disabled = true;
		goto out;
	} else {
		if (usage->disabled)
			disable_pmn();
		usage->disabled = false;
	}

	// Check whether the performance counters are enabled.
	MRC_PMU(cr, PMCR);
	if (cr & 1) {
		// Counters are running - disable them for the evaluation.
		cr &= ~1;
		MCR_PMU(cr, PMCR);

		usage->joule += evaluate_model(cpu);

		time_diff = jiffies_to_msecs(jiffies - usage->time);
		if (time_diff > POWER_UPDATE_INTERVAL) {
			usage->watt = div64_s64(usage->joule, time_diff);
			usage->joule = 0;
			usage->time = jiffies;
		}

	} else {
		// CPU was disabled before or this is the first call.
		initialize_pmu(cpu);
		usage->joule = 0;
		usage->watt = 0;
		usage->time = jiffies;
	}
	// Reset and restart the counters.
	reset_pmn();
	reset_ccnt();
	enable_pmn();
out:
	put_cpu_var(current_energy_usage);
}

// File nodes in sysfs

static ssize_t show_power_status(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, dev);
	struct current_energy_usage *usage = &per_cpu(current_energy_usage, cpu->dev.id);

	if (usage->disabled)
		return sprintf(buf, "monitoring disabled (USERENR = 1)\n");
	else
		return sprintf(buf, "%lld nW\n", usage->watt);
}

static DEVICE_ATTR(power_status, 0444, show_power_status, NULL);

static ssize_t show_power_limit(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct cpu *cpu = container_of(dev, struct cpu, dev);

	return sprintf(buf, "%lld\n", per_cpu(power_limit, cpu->dev.id));
}

static ssize_t __ref store_power_limit(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	ssize_t ret;
	s64 limit;

	ret = kstrtos64(buf, 0, &limit);
	if (ret >= 0) {
		per_cpu(power_limit, dev->id) = limit;
		ret = count;
	}
	return ret;
}

static DEVICE_ATTR(power_limit, 0644, show_power_limit, store_power_limit);

static int __init power_init(void)
{
	int i;

	for_each_possible_cpu(i) {
		device_create_file(get_cpu_device(i), &dev_attr_power_status);
		device_create_file(get_cpu_device(i), &dev_attr_power_limit);
		per_cpu(power_limit, i) = 0;
	}

	printk(KERN_NOTICE "power: finished initialization\n");

	return 0;
}
module_init(power_init);
