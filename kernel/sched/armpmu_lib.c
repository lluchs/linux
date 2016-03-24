#include "armpmu_lib.h"

void enable_pmn()
{
	__u32 cr, set;
	// Make sure that all counters are enabled.
	set = 0xffffffff;
	MCR_PMU(set, PMCNTENSET);
	// Read the control register.
	MRC_PMU(cr, PMCR);
	// Set the "Enable" bit 0.
	cr |= 1;
	// Write the control register back.
	MCR_PMU(cr, PMCR);
}

void disable_pmn()
{
	__u32 cr;
	// Read the control register.
	MRC_PMU(cr, PMCR);
	// Unset the "Enable" bit 0.
	cr &= ~1;
	// Write the control register back.
	MCR_PMU(cr, PMCR);
}

void set_pmn(__u32 counter, __u32 event)
{
	__u32 cr;
	MRC_PMU(cr, PMCR);
	// Only five bits are valid, rest is reserved.
	counter &= 0x1f;
	// Select the given counter.
	MCR_PMU(counter, PMSELR);
	// Set the event.
	MCR_PMU(event, PMXEVTYPER);
}

__u32 read_pmn(__u32 counter)
{
	__u32 result;
	// Only four bits are valid, rest is reserved.
	counter &= 0x1f;
	// Select the given counter.
	MCR_PMU(counter, PMSELR);
	// Read the register.
	MRC_PMU(result, PMXEVCNTR);
	return result;
}

void reset_pmn()
{
	__u32 cr, ovsr;
	// Read the control register.
	MRC_PMU(cr, PMCR);
	// Set the "Event counter reset" bit 1.
	cr |= (1 << 1);
	// Write the control register back.
	MCR_PMU(cr, PMCR);
	// Reset all overflow bits except for PMCCNTR.
	ovsr = 0x7fffffff;
	MCR_PMU(ovsr, PMOVSR);
}

void reset_ccnt()
{
	__u32 cr, ovsr;
	// Read the control register.
	MRC_PMU(cr, PMCR);
	// Set the "Cycle counter reset" bit 2.
	cr |= (1 << 2);
	// Write the control register back.
	MCR_PMU(cr, PMCR);
	// Reset the overflow bit for PMCCNTR.
	ovsr = 0x80000000;
	MCR_PMU(ovsr, PMOVSR);
}

const char * pmn_event_name(int event)
{
	switch (event) {
#define X(NAME, VALUE) case VALUE: return #NAME;
	ARMPMU_EVENT_LIST
#undef X
	default:
		return 0;
	}
}
