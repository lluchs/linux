#include <linux/percpu.h>
#include <linux/types.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
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

// Throttling function: returns true if the calling CPU may use more energy.
bool power_cpu_has_energy_left(void)
{
	int cpu = smp_processor_id();
	struct current_energy_usage *usage;
	s64 maximum_energy_usage = per_cpu(power_limit, cpu);
	bool result;

	// We're never throttling A7 cores.
	// TODO: Maybe only always execute the first CPU core?
	if (IS_A7(cpu) || maximum_energy_usage <= 0) return true;

	usage = &get_cpu_var(current_energy_usage);
	result = maximum_energy_usage < div64_s64(usage->joule, jiffies_to_msecs(jiffies - usage->time));
	put_cpu_var(current_energy_usage);
	return result;
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

	return 0;
}
module_init(power_init);
