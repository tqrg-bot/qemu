/*
 * Internal definitions for a target's KVM support
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef QEMU_KVM_INT_H
#define QEMU_KVM_INT_H

#include "sysemu/sysemu.h"
#include "sysemu/accel.h"
#include <sysemu/kvm.h>

#define KVM_MSI_HASHTAB_SIZE    256

typedef struct KVMSlot
{
    hwaddr start_addr;
    ram_addr_t memory_size;
    void *ram;
    int slot;
    int flags;
} KVMSlot;

struct KVMState
{
    AccelState parent_obj;

    KVMSlot *slots;
    int nr_slots;
    int fd;
    int vmfd;
    int coalesced_mmio;
    struct kvm_coalesced_mmio_ring *coalesced_mmio_ring;
    bool coalesced_flush_in_progress;
    int broken_set_mem_region;
    int migration_log;
    int vcpu_events;
    int robust_singlestep;
    int debugregs;
#ifdef KVM_CAP_SET_GUEST_DEBUG
    struct kvm_sw_breakpoint_head kvm_sw_breakpoints;
#endif
    int pit_state2;
    int xsave, xcrs;
    int many_ioeventfds;
    int intx_set_mask;
    /* The man page (and posix) say ioctl numbers are signed int, but
     * they're not.  Linux, glibc and *BSD all treat ioctl numbers as
     * unsigned, and treating them as signed here can break things */
    unsigned irq_set_ioctl;
    unsigned int sigmask_len;
#ifdef KVM_CAP_IRQ_ROUTING
    struct kvm_irq_routing *irq_routes;
    int nr_allocated_irq_routes;
    uint32_t *used_gsi_bitmap;
    unsigned int gsi_count;
    QTAILQ_HEAD(msi_hashtab, KVMMSIRoute) msi_hashtab[KVM_MSI_HASHTAB_SIZE];
    bool direct_msi;
#endif

    AddressSpace kvm_as;
    MemoryRegion kvm_as_root, kvm_as_mem;
};

#define TYPE_KVM_ACCEL ACCEL_CLASS_NAME("kvm")

#define KVM_STATE(obj) \
    OBJECT_CHECK(KVMState, (obj), TYPE_KVM_ACCEL)

#endif
