#ifndef SHPC_H
#define SHPC_H

#include "qemu-common.h"
#include "memory.h"
#include "vmstate.h"

struct SHPCDevice {
    /* PCI device hosting the SHPC */
    PCIDevice *parent;

    /* Capability offset in device's config space */
    int cap;

    /* # of hot-pluggable slots */
    int nslots;

    /* SHPC WRS: working register set */
    uint8_t *config;

    /* Used to enable checks on load. Note that writable bits are
     * never checked even if set in cmask. */
    uint8_t *cmask;

    /* Used to implement R/W bytes */
    uint8_t *wmask;

    /* Used to implement RW1C(Write 1 to Clear) bytes */
    uint8_t *w1cmask;

    /* MMIO for the SHPC BAR */
    MemoryRegion mmio;

    /* Bus controlled by this SHPC */
    PCIBus *sec_bus;

    /* MSI already requested for this event */
    int msi_requested;
};

int shpc_bar_size(PCIDevice *dev);

void shpc_reset(SHPCDevice *shpc);
int shpc_init(SHPCDevice *shpc, PCIDevice *dev, PCIBus *sec_bus,
              MemoryRegion *bar, unsigned off);
void shpc_cleanup(SHPCDevice *shpc, MemoryRegion *bar);
void shpc_cap_write_config(SHPCDevice *shpc, uint32_t addr,
                           uint32_t val, int len);

extern VMStateInfo shpc_vmstate_info;
#define SHPC_VMSTATE(_field, _type) \
    VMSTATE_BUFFER_UNSAFE_INFO(_field, _type, 0, shpc_vmstate_info, 0)

#endif
