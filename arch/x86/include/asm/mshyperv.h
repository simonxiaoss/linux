#ifndef _ASM_X86_MSHYPER_H
#define _ASM_X86_MSHYPER_H

#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/hyperv.h>
#include <linux/slab.h>

struct ms_hyperv_info {
	u32 features;
	u32 misc_features;
	u32 hints;
	u32 max_vp_index;
	u32 max_lp_index;
};

extern struct ms_hyperv_info ms_hyperv;

#define hv_get_vp_index(index) rdmsrl(HV_X64_MSR_VP_INDEX, index)

void hyperv_callback_vector(void);
#ifdef CONFIG_TRACING
#define trace_hyperv_callback_vector hyperv_callback_vector
#endif
void hyperv_vector_handler(struct pt_regs *regs);
void hv_setup_vmbus_irq(void (*handler)(void));
void hv_remove_vmbus_irq(void);

void hv_setup_kexec_handler(void (*handler)(void));
void hv_remove_kexec_handler(void);
void hv_setup_crash_handler(void (*handler)(struct pt_regs *regs));
void hv_remove_crash_handler(void);
#endif

/**
 * hv_cpu_number_to_vp_number() - Map CPU to VP.
 * @cpu_number: CPU number in Linux terms
 *
 * This function returns the mapping between the Linux processor
 * number and the hypervisor's virtual processor number, useful
 * in making hypercalls and such that talk about specific
 * processors.
 *
 * Return: Virtual processor number in Hyper-V terms
 */

static inline int hv_cpu_number_to_vp_number(int cpu_number)
{
	u64 msr_vp_index;
	u32 *hv_vp_index;
	int cpu;

	hv_vp_index = kmalloc_array(num_possible_cpus(), sizeof(*hv_vp_index),
				    GFP_KERNEL);

	if (!hv_vp_index) {
		printk(KERN_ERR "unable to allocate memory for hv_vp_index \n");
		return 0;
	}

	hv_get_vp_index(msr_vp_index);
	hv_vp_index[smp_processor_id()] = msr_vp_index;
	cpu = hv_vp_index[cpu_number];

	kfree(hv_vp_index);
	hv_vp_index = NULL;

	return cpu;
}
