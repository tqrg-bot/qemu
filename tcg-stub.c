/*
 * QEMU TCG accelerator stub
 *
 * Copyright Red Hat, Inc. 2013
 *
 * Author: Paolo Bonzini     <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu-common.h"
#include "exec/exec-all.h"
#include "exec/cputlb.h"
#include "translate-all.h"

volatile sig_atomic_t exit_request;

void tb_flush(CPUArchState *env)
{
}

void tlb_set_page(CPUState *cpu, target_ulong vaddr,
                  hwaddr paddr, int prot,
                  int mmu_idx, target_ulong size)
{
    abort();
}

void tb_invalidate_phys_addr(AddressSpace *as, hwaddr addr)
{
}

#ifndef __OPTIMIZE__
/* This function is only called inside conditionals which we
 * rely on the compiler to optimize out when CONFIG_KVM is not
 * defined.
 */
void cpu_tlb_reset_dirty_all(ram_addr_t start1, ram_addr_t length)
{
    abort();
}

int cpu_exec(CPUArchState *s)
{
    abort();
}
#endif
