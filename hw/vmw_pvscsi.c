/*
 * VMware Paravirtualized SCSI Host Bus Adapter emulation
 *
 * Copyright (c) 2011 Red Hat, Inc.
 * Written by Paolo Bonzini
 *
 * This code is licensed under GPLv2.
 */

#include <assert.h>

#include "hw.h"
#include "pci.h"
#include "scsi.h"
#include "scsi-defs.h"
#include "vmw_pvscsi.h"
#include "block_int.h"
#include "host-utils.h"
#include "trace.h"

#define PVSCSI_MAX_DEVS 127
#define PAGE_SIZE       4096
#define PAGE_SHIFT      12

typedef struct PVSCSISGState {
    target_phys_addr_t elemAddr;
    target_phys_addr_t dataAddr;
    uint32_t resid;
} PVSCSISGState;

typedef struct PVSCSIRequest {
    SCSIDevice *sdev;
    SCSIRequest *sreq;
    uint8_t sense_key;
    uint8_t completed;
    int lun;
    uint64_t resid;
    PVSCSISGState sg;
    struct PVSCSIRingReqDesc req;
    struct PVSCSIRingCmpDesc cmp;
    QTAILQ_ENTRY(PVSCSIRequest) next;
} PVSCSIRequest;

typedef QTAILQ_HEAD(, PVSCSIRequest) PVSCSIRequestList;

typedef struct {
    PCIDevice dev;
    SCSIBus bus;
    QEMUBH *complete_reqs_bh;

    MemoryRegion mmio;
    int mmio_io_addr;

    /* zeroed on reset */
    uint32_t cmd_latch;
    uint32_t cmd_buffer[sizeof(struct PVSCSICmdDescSetupRings)
                        / sizeof(uint32_t)];
    uint32_t cmd_ptr;
    uint32_t cmd_status;
    uint32_t intr_status;
    uint32_t intr_mask;
    uint32_t intr_cmpl;
    uint32_t intr_msg;
    struct PVSCSICmdDescSetupRings rings;
    struct PVSCSICmdDescSetupMsgRing msgRing;
    uint32_t reqNumEntriesLog2;
    uint32_t cmpNumEntriesLog2;
    uint32_t msgNumEntriesLog2;

    PVSCSIRequestList pending_queue;
    PVSCSIRequestList complete_queue;
} PVSCSIState;


static inline SCSIDevice *pvscsi_device_find(PVSCSIState *s, uint8_t *lun,
                                             uint32_t target, int *lunval)
{
    if (lun[0] || lun[2] || lun[3] || lun[4] || lun[5] || lun[6] || lun[7]) {
        *lunval = -1;
    } else {
        *lunval = lun[1];
    }
    return scsi_device_find(&s->bus, 0, target, *lunval);
}


/* Add a command to the pending queue.  */
static PVSCSIRequest *pvscsi_queue_request(PVSCSIState *s, SCSIDevice **d,
                                           struct PVSCSIRingReqDesc *req)
{
    PVSCSIRequest *p;

    trace_pvscsi_queue_request(req->context, req->cdb[0], req->dataLen);

    p = g_malloc0(sizeof(*p));
    p->req = *req;
    p->cmp.context = p->req.context;
    QTAILQ_INSERT_TAIL(&s->pending_queue, p, next);

    *d = pvscsi_device_find(s, req->lun, req->target, &p->lun);
    return p;
}

static void pvscsi_free_queue(PVSCSIRequestList *q)
{
    PVSCSIRequest *p;

    while (!QTAILQ_EMPTY(q)) {
        p = QTAILQ_FIRST(q);
        QTAILQ_REMOVE(q, p, next);
        g_free(p);
    }
}

static void pvscsi_soft_reset(PVSCSIState *s)
{
    qbus_reset_all_fn(&s->bus);
    pvscsi_free_queue(&s->complete_queue);
    assert(QTAILQ_EMPTY(&s->pending_queue));
    memset(&s->cmd_latch, 0, sizeof(*s) - offsetof(PVSCSIState, cmd_latch));
    s->intr_cmpl = PVSCSI_INTR_CMPL_0;
    s->intr_msg = PVSCSI_INTR_MSG_0;
    QTAILQ_INIT(&s->pending_queue);
    QTAILQ_INIT(&s->complete_queue);
}


static void pvscsi_raise_intr(PVSCSIState *s, int mask)
{
    int intr_raised = mask & ~s->intr_status;
    s->intr_status |= mask;
    trace_pvscsi_raise_intr(intr_raised,
                            (intr_raised & s->intr_mask) == 0 ? "masked" : "");
    if (intr_raised & s->intr_mask) {
        qemu_set_irq(s->dev.irq[0], 1);
    }
}

static void pvscsi_acknowledge_intr(PVSCSIState *s, int mask)
{
    trace_pvscsi_acknowledge_intr(mask);
    s->intr_status &= ~mask;
    if (mask == s->intr_cmpl) {
        s->intr_cmpl ^= PVSCSI_INTR_CMPL_MASK;

        /* Try putting more complete requests on the ring.  */
        if (!QTAILQ_EMPTY(&s->complete_queue)) {
            qemu_bh_schedule(s->complete_reqs_bh);
        }
    }
    if (mask == s->intr_msg) {
        s->intr_msg ^= PVSCSI_INTR_MSG_MASK;
    }
    if ((s->intr_status & s->intr_mask) == 0) {
        qemu_set_irq(s->dev.irq[0], 0);
    }
}

static void pvscsi_set_intr_mask(PVSCSIState *s, int mask)
{
    int intr_enabled = mask & ~s->intr_mask;
    s->intr_mask = mask;
    if (s->intr_status & intr_enabled) {
        qemu_set_irq(s->dev.irq[0], 1);
    }
    if ((s->intr_status & mask) == 0) {
        qemu_set_irq(s->dev.irq[0], 0);
    }
}


#define pvscsi_ld_ring_state(s, field) \
    ldl_le_phys(s->rings.ringsStatePPN * PAGE_SIZE + offsetof(struct PVSCSIRingsState, field))

#define pvscsi_st_ring_state(s, field, val) \
    stl_le_phys(s->rings.ringsStatePPN * PAGE_SIZE + offsetof(struct PVSCSIRingsState, field), \
             val)

/* Return number of free elements in the completion ring.  */
static inline int pvscsi_cmp_free(PVSCSIState *s)
{
    return ((1 << s->cmpNumEntriesLog2) - 1 -
            (pvscsi_ld_ring_state(s, cmpProdIdx) - pvscsi_ld_ring_state(s, cmpConsIdx)));
}

/* Return number of pending elements in the request ring.  */
static inline int pvscsi_req_pending(PVSCSIState *s)
{
    return pvscsi_ld_ring_state(s, reqProdIdx) - pvscsi_ld_ring_state(s, reqConsIdx);
}

/* Return the physical address of the idx-th element in the ring
 * whose physical page numbers are given by ppn.  Each element in
 * the ring has size bytes.  */
static target_phys_addr_t pvscsi_get_ring_addr(PVSCSIState *s, int idx,
                                               int size, uint64_t *ppn)
{
    uint32_t ofs = idx * size;
    return (ppn[ofs >> PAGE_SHIFT] * PAGE_SIZE) | (ofs & (PAGE_SIZE - 1));
}


/* Copy cmp_desc on the completion ring, assuming there is a free entry.  */
static void pvscsi_cmp_ring_put(PVSCSIState *s,
                                struct PVSCSIRingCmpDesc *cmp_desc)
{
    uint32_t cmp_entries = s->cmpNumEntriesLog2;
    uint32_t val = pvscsi_ld_ring_state(s, cmpProdIdx);
    uint32_t idx = val & MASK(cmp_entries);
    target_phys_addr_t addr;

    trace_pvscsi_cmp_ring_put(cmp_desc->context);
    addr = pvscsi_get_ring_addr(s, idx, sizeof(struct PVSCSIRingCmpDesc),
                                s->rings.cmpRingPPNs);

    barrier();
    cpu_physical_memory_write(addr, (void *)cmp_desc, sizeof(*cmp_desc));
    barrier();
    pvscsi_st_ring_state(s, cmpProdIdx, val + 1);
}

/* Put all completed requests on the completion ring.  */
static void pvscsi_complete_reqs(void *opaque)
{
    PVSCSIState *s = opaque;
    PVSCSIRequest *p;
    int n = pvscsi_cmp_free(s);
    int done = 0;
    while (n > 0 && !QTAILQ_EMPTY(&s->complete_queue)) {
        p = QTAILQ_FIRST(&s->complete_queue);
        QTAILQ_REMOVE(&s->complete_queue, p, next);
        pvscsi_cmp_ring_put(s, &p->cmp);
        g_free(p);
        n--;
        done++;
    }
    if (done) {
        pvscsi_raise_intr(s, s->intr_cmpl);
    }
}

/* Prepare to put r on the completion ring.  */
static void pvscsi_complete_req(PVSCSIState *s, PVSCSIRequest *p)
{
    assert(!p->completed);
    trace_pvscsi_complete_req(p->cmp.context, p->cmp.dataLen, p->sense_key);
    if (p->sreq != NULL) {
        scsi_req_unref(p->sreq);
        p->sreq = NULL;
    }
    p->completed = 1;
    QTAILQ_REMOVE(&s->pending_queue, p, next);
    QTAILQ_INSERT_TAIL(&s->complete_queue, p, next);
    qemu_bh_schedule(s->complete_reqs_bh);
}

/* Write sense data for a completed request.  */
static void pvscsi_write_sense(PVSCSIRequest *p, uint8_t *buf, int len)
{
    p->cmp.senseLen = MIN(p->req.senseLen, len);
    p->sense_key = buf[2];
    cpu_physical_memory_write(p->req.senseAddr, buf, p->cmp.senseLen);
}

static void pvscsi_transfer_data_with_buffer(PVSCSIRequest *p, bool to_host,
                                             uint8_t *buf, int len)
{
    if (len) {
        cpu_physical_memory_rw(p->req.dataAddr, buf, len, to_host);
        p->cmp.dataLen += len;
        p->req.dataAddr += len;
        p->resid -= len;
    }
}

static void pvscsi_get_next_sg_elem(PVSCSISGState *sg)
{
    struct PVSCSISGElement elem;

    for (;; sg->elemAddr = elem.addr) {
        cpu_physical_memory_read(sg->elemAddr, (void *)&elem,
                                 sizeof(elem));
#if 0
        /* PVSCSI_SGE_FLAG_CHAIN_ELEMENT not in the header file! */
        if ((elem.flags & PVSCSI_SGE_FLAG_CHAIN_ELEMENT) == 0) {
            break;
        }
#else
        break;
#endif
    }

    sg->elemAddr += sizeof(elem);
    sg->dataAddr = elem.addr;
    sg->resid = elem.length;
}

static void pvscsi_transfer_data_with_sg_list(PVSCSIRequest *p, bool to_host,
                                              uint8_t *buf, int len)
{
    int n;
    while (len) {
        while (!p->sg.resid) {
            pvscsi_get_next_sg_elem(&p->sg);
            trace_pvscsi_sg_elem(p->req.context, p->sg.dataAddr, p->sg.resid);
        }
        assert(len > 0);
        n = MIN((unsigned) len, p->sg.resid);
        if (n) {
            cpu_physical_memory_rw(p->sg.dataAddr, buf, n, to_host);
        }

        buf += n;
        p->cmp.dataLen += n;
        p->sg.dataAddr += n;

        len -= n;
        p->resid -= n;
        p->sg.resid -= n;
    }
}

/* Callback to indicate that the SCSI layer has completed a transfer.  */
static void pvscsi_transfer_data(SCSIRequest *req, uint32_t len)
{
    PVSCSIRequest *p = req->hba_private;
    uint8_t *buf = scsi_req_get_buf(req);
    int to_host = (p->req.flags & PVSCSI_FLAG_CMD_DIR_TOHOST) != 0;

    if (!p) {
        fprintf(stderr, "PVSCSI: Can't find request for tag 0x%x\n", req->tag);
        return;
    }

    assert(p->resid);
    trace_pvscsi_transfer_data(p->req.context, len);
    if (!len) {
        /* Short transfer.  */
        p->cmp.hostStatus = BTSTAT_DATARUN;
        scsi_req_cancel(req);
        return;
    }

    if (len > p->resid) {
        /* Small buffer.  */
        p->cmp.hostStatus = BTSTAT_DATARUN;
        scsi_req_cancel(req);
        return;
    }

    if (p->req.flags & PVSCSI_FLAG_CMD_WITH_SG_LIST) {
        pvscsi_transfer_data_with_sg_list(p, to_host, buf, len);
    } else {
        pvscsi_transfer_data_with_buffer(p, to_host, buf, len);
    }

    scsi_req_continue(req);
}

/* Callback to indicate that the SCSI layer has completed a transfer.  */
static void pvscsi_command_complete(SCSIRequest *req, uint32_t status)
{
    PVSCSIState *s = DO_UPCAST(PVSCSIState, dev.qdev, req->bus->qbus.parent);
    PVSCSIRequest *p = req->hba_private;

    if (!p) {
        fprintf(stderr, "PVSCSI: Can't find request for tag 0x%x\n", req->tag);
        return;
    }

    p->cmp.scsiStatus = status;
    if (p->cmp.scsiStatus == CHECK_CONDITION) {
	uint8_t sense[96];
        int n = scsi_req_get_sense(p->sreq, sense, sizeof(sense));
        pvscsi_write_sense(p, sense, n);
    }
    pvscsi_complete_req(s, p);
}

static void pvscsi_request_cancelled(SCSIRequest *req)
{
    PVSCSIState *s = DO_UPCAST(PVSCSIState, dev.qdev, req->bus->qbus.parent);
    PVSCSIRequest *p = req->hba_private;

    if (p->cmp.hostStatus == BTSTAT_SUCCESS) {
	p->cmp.hostStatus = BTSTAT_ABORTQUEUE;
    }
    pvscsi_complete_req(s, p);
}


/* Process a request from the request ring.  */
static void pvscsi_process_req(PVSCSIState *s, struct PVSCSIRingReqDesc *r)
{
    SCSIDevice *d;
    PVSCSIRequest *p = pvscsi_queue_request(s, &d, r);
    int64_t datalen, n;

    if (!d) {
        p->cmp.hostStatus = BTSTAT_SELTIMEO;
        pvscsi_complete_req(s, p);
        return;
    }

    if (r->flags & PVSCSI_FLAG_CMD_WITH_SG_LIST) {
        p->sg.elemAddr = r->dataAddr;
    }

    p->sreq = scsi_req_new(d, r->context, p->lun, r->cdb, p);
    if (p->sreq->cmd.mode == SCSI_XFER_FROM_DEV
        && (r->flags & PVSCSI_FLAG_CMD_DIR_TODEVICE)) {
        p->cmp.hostStatus = BTSTAT_BADMSG;
        scsi_req_cancel(p->sreq);
        return;
    }
    if (p->sreq->cmd.mode == SCSI_XFER_TO_DEV
        && (r->flags & PVSCSI_FLAG_CMD_DIR_TOHOST)) {
        p->cmp.hostStatus = BTSTAT_BADMSG;
        scsi_req_cancel(p->sreq);
        return;
    }
    n = scsi_req_enqueue(p->sreq);

    if (n) {
        datalen = (n < 0 ? -n : n);
        p->resid = MIN(datalen, r->dataLen);
        scsi_req_continue(p->sreq);
    }
}

/* Process pending requests on the request ring.  */
static void pvscsi_process_req_ring(PVSCSIState *s)
{
    uint32_t req_entries = s->reqNumEntriesLog2;

    trace_pvscsi_kick_io();
    while (pvscsi_req_pending(s)) {
        uint32_t val = pvscsi_ld_ring_state(s, reqConsIdx);
        uint32_t idx = val & MASK(req_entries);
        target_phys_addr_t addr;
        struct PVSCSIRingReqDesc req_desc;

        addr = pvscsi_get_ring_addr(s, idx, sizeof(struct PVSCSIRingReqDesc),
                                    s->rings.reqRingPPNs);

        barrier();
        cpu_physical_memory_read(addr, (void *)&req_desc, sizeof(req_desc));
        pvscsi_process_req(s, &req_desc);
        barrier();
        pvscsi_st_ring_state(s, reqConsIdx, val + 1);
    }
}


static int32_t pvscsi_cmd_bad(PVSCSIState *s)
{
    fprintf(stderr, "vmw_pvscsi: bad command %d\n", s->cmd_latch);
    return -1;
}

static int32_t pvscsi_cmd_unimpl(PVSCSIState *s)
{
    fprintf(stderr, "vmw_pvscsi: unimplemented command %d\n", s->cmd_latch);
    return -1;
}

static int32_t pvscsi_cmd_adapter_reset(PVSCSIState *s)
{
    pvscsi_soft_reset(s);
    return 0;
}

static int floor_log2(int x)
{
    assert(x);
    return 31 - clz32(x);
}

/* Setup ring buffers and initialize the ring state page.  */
static int32_t pvscsi_cmd_setup_rings(PVSCSIState *s)
{
    memcpy(&s->rings, s->cmd_buffer, sizeof(s->rings));
    if (s->rings.reqRingNumPages == 0 ||
        s->rings.cmpRingNumPages == 0) {
        return -1;
    }

    s->reqNumEntriesLog2 = floor_log2(s->rings.reqRingNumPages * PAGE_SIZE
                                      / sizeof(struct PVSCSIRingReqDesc));
    s->cmpNumEntriesLog2 = floor_log2(s->rings.cmpRingNumPages * PAGE_SIZE
                                      / sizeof(struct PVSCSIRingCmpDesc));

    trace_pvscsi_setup_req_ring(s->rings.reqRingNumPages,
                                1 << s->reqNumEntriesLog2);
    trace_pvscsi_setup_cmp_ring(s->rings.cmpRingNumPages,
                                1 << s->cmpNumEntriesLog2);

    pvscsi_st_ring_state(s, reqNumEntriesLog2, s->reqNumEntriesLog2);
    pvscsi_st_ring_state(s, cmpNumEntriesLog2, s->cmpNumEntriesLog2);
    pvscsi_st_ring_state(s, cmpProdIdx, 0);
    pvscsi_st_ring_state(s, cmpConsIdx, 0);
    pvscsi_st_ring_state(s, reqProdIdx, 0);
    pvscsi_st_ring_state(s, reqConsIdx, 0);
    return 0;
}

static int32_t pvscsi_cmd_reset_bus(PVSCSIState *s)
{
    qbus_reset_all_fn(&s->bus);
    return 0;
}

static int32_t pvscsi_cmd_reset_device(PVSCSIState *s)
{
    struct PVSCSICmdDescResetDevice *cmd =
        (struct PVSCSICmdDescResetDevice *) &s->cmd_buffer;
    SCSIDevice *sdev;
    int lun;

    sdev = pvscsi_device_find(s, cmd->lun, cmd->target, &lun);
    if (sdev != NULL && sdev->info->qdev.reset) {
        sdev->info->qdev.reset(&sdev->qdev);
    }

    return 0;
}

static int32_t pvscsi_cmd_abort_cmd(PVSCSIState *s)
{
    return 0;
}

static int32_t pvscsi_cmd_setup_msg_ring(PVSCSIState *s)
{
    memcpy(&s->msgRing, s->cmd_buffer, sizeof(s->msgRing));
    if (s->msgRing.numPages == 0) {
        return -1;
    }

    s->msgNumEntriesLog2 = floor_log2(s->msgRing.numPages * PAGE_SIZE
                                      / sizeof(struct PVSCSIRingMsgDesc));

    trace_pvscsi_setup_msg_ring(s->msgRing.numPages,
                                1 << s->msgNumEntriesLog2);

    pvscsi_st_ring_state(s, msgNumEntriesLog2, s->msgNumEntriesLog2);
    pvscsi_st_ring_state(s, msgProdIdx, 0);
    pvscsi_st_ring_state(s, msgConsIdx, 0);
    return 0;
}

typedef struct {
    int nargs;
    int32_t (*fn)(PVSCSIState *);
} PVSCSICmd;

static const PVSCSICmd pvscsi_commands[PVSCSI_CMD_LAST] = {
    [PVSCSI_CMD_FIRST] = {
        .nargs = 0,
        .fn = pvscsi_cmd_bad,
    },
    [PVSCSI_CMD_ADAPTER_RESET] = {
        .nargs = 0,
        .fn = pvscsi_cmd_adapter_reset
    },
    [PVSCSI_CMD_ISSUE_SCSI] = {
        .nargs = 0, /* unknown */
        .fn = pvscsi_cmd_unimpl
    },
    [PVSCSI_CMD_SETUP_RINGS] = {
        .nargs = sizeof(struct PVSCSICmdDescSetupRings) / sizeof(uint32_t),
        .fn = pvscsi_cmd_setup_rings
    },
    [PVSCSI_CMD_RESET_BUS] = {
        .nargs = 0,
        .fn = pvscsi_cmd_reset_bus
    },
    [PVSCSI_CMD_RESET_DEVICE] = {
        .nargs = sizeof(struct PVSCSICmdDescResetDevice) / sizeof(uint32_t),
        .fn = pvscsi_cmd_reset_device
    },
    [PVSCSI_CMD_ABORT_CMD] = {
        .nargs = sizeof(struct PVSCSICmdDescAbortCmd) / sizeof(uint32_t),
        .fn = pvscsi_cmd_abort_cmd
    },
    [PVSCSI_CMD_CONFIG] = {
        .nargs = 0, /* unknown */
        .fn = pvscsi_cmd_unimpl
    },
    [PVSCSI_CMD_SETUP_MSG_RING] = {
        .nargs = sizeof(struct PVSCSICmdDescSetupMsgRing) / sizeof(uint32_t),
        .fn = pvscsi_cmd_setup_msg_ring
    },
    [PVSCSI_CMD_DEVICE_UNPLUG] = {
        .nargs = 0, /* unknown */
        .fn = pvscsi_cmd_unimpl
    }
};


static void pvscsi_maybe_do_cmd(PVSCSIState *s)
{
    int cmd = s->cmd_latch >= PVSCSI_CMD_LAST ? PVSCSI_CMD_FIRST : s->cmd_latch;
    const PVSCSICmd *cmd_info = &pvscsi_commands[cmd];

    if (s->cmd_ptr >= cmd_info->nargs) {
        s->cmd_status = cmd_info->fn(s);
        s->cmd_latch = 0;
        s->cmd_ptr = 0;
    }
}

static uint64_t pvscsi_mmio_read(void *opaque, target_phys_addr_t addr,
			        unsigned size)
{
    PVSCSIState *s = opaque;

    switch (addr) {
    case PVSCSI_REG_OFFSET_COMMAND:
    case PVSCSI_REG_OFFSET_COMMAND_DATA:
    case PVSCSI_REG_OFFSET_KICK_NON_RW_IO:
    case PVSCSI_REG_OFFSET_KICK_RW_IO:
        fprintf(stderr, "vmw_pvscsi: read to write-only register %x\n",
		(unsigned) addr);
        break;
    case PVSCSI_REG_OFFSET_COMMAND_STATUS:
        return s->cmd_status;
        break;
    case PVSCSI_REG_OFFSET_INTR_STATUS:
        return s->intr_status;
        break;
    case PVSCSI_REG_OFFSET_INTR_MASK:
        return s->intr_mask;
        break;
    case PVSCSI_REG_OFFSET_LAST_STS_0:
    case PVSCSI_REG_OFFSET_LAST_STS_1:
    case PVSCSI_REG_OFFSET_LAST_STS_2:
    case PVSCSI_REG_OFFSET_LAST_STS_3:
    case PVSCSI_REG_OFFSET_DEBUG:
        fprintf(stderr, "vmw_pvscsi: read from unsupported register %x\n",
		(unsigned) addr);
        break;
    default:
        break;
    }
    return 0;
}

static void pvscsi_mmio_write(void *opaque, target_phys_addr_t addr,
			     uint64_t val, unsigned size)
{
    PVSCSIState *s = opaque;

    switch (addr) {
    case PVSCSI_REG_OFFSET_COMMAND:
        trace_pvscsi_cmd(val);
        s->cmd_latch = val;
        s->cmd_ptr = 0;
        pvscsi_maybe_do_cmd(s);
        break;
    case PVSCSI_REG_OFFSET_COMMAND_DATA:
        s->cmd_buffer[s->cmd_ptr++] = val;
        pvscsi_maybe_do_cmd(s);
        break;
    case PVSCSI_REG_OFFSET_COMMAND_STATUS:
        fprintf(stderr, "vmw_pvscsi: write to read-only register %x\n",
		(unsigned) addr);
        break;
    case PVSCSI_REG_OFFSET_INTR_STATUS:
        pvscsi_acknowledge_intr(s, val);
        break;
    case PVSCSI_REG_OFFSET_INTR_MASK:
        pvscsi_set_intr_mask(s, val);
        break;
    case PVSCSI_REG_OFFSET_KICK_NON_RW_IO:
    case PVSCSI_REG_OFFSET_KICK_RW_IO:
        pvscsi_process_req_ring(s);
        break;

    case PVSCSI_REG_OFFSET_LAST_STS_0:
    case PVSCSI_REG_OFFSET_LAST_STS_1:
    case PVSCSI_REG_OFFSET_LAST_STS_2:
    case PVSCSI_REG_OFFSET_LAST_STS_3:
    case PVSCSI_REG_OFFSET_DEBUG:
        fprintf(stderr, "vmw_pvscsi: write to unsupported register %x\n",
		(unsigned) addr);
        break;
    default:
        break;
    }
}


static const MemoryRegionOps pvscsi_mmio_ops = {
    .read = pvscsi_mmio_read,
    .write = pvscsi_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void pvscsi_reset(DeviceState *dev)
{
    PVSCSIState *s = DO_UPCAST(PVSCSIState, dev.qdev, dev);

    pvscsi_soft_reset(s);
}

static int pvscsi_uninit(PCIDevice *d)
{
    PVSCSIState *s = DO_UPCAST(PVSCSIState, dev, d);

    cpu_unregister_io_memory(s->mmio_io_addr);

    return 0;
}

static struct SCSIBusInfo pvscsi_scsi_info = {
    .tcq = true,
    .max_target = PVSCSI_MAX_DEVS,
    .max_lun = 255,

    .transfer_data = pvscsi_transfer_data,
    .complete = pvscsi_command_complete,
    .cancel = pvscsi_request_cancelled
};

static int pvscsi_init(PCIDevice *dev)
{
    PVSCSIState *s = DO_UPCAST(PVSCSIState, dev, dev);
    uint8_t *pci_conf;

    pci_conf = s->dev.config;

    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_VMWARE);
    pci_config_set_device_id(pci_conf, PCI_DEVICE_ID_VMWARE_PVSCSI);
    pci_config_set_class(pci_conf, PCI_CLASS_STORAGE_SCSI);

    /* PCI subsystem ID */
    pci_conf[PCI_SUBSYSTEM_ID] = 0x00;
    pci_conf[PCI_SUBSYSTEM_ID + 1] = 0x10;

    /* PCI latency timer = 255 */
    pci_conf[PCI_LATENCY_TIMER] = 0xff;

    /* Interrupt pin 1 */
    pci_conf[PCI_INTERRUPT_PIN] = 0x01;

    memory_region_init_io(&s->mmio, &pvscsi_mmio_ops, s,
                          "pvscsi", PVSCSI_MEM_SPACE_SIZE);
    pci_register_bar(&s->dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);

#if 0
    s->pio = memory_region_init_io(&s->mmio, &pvscsi_mmio_ops, s,
                                   "pvscsi", 256);
    pci_register_bar(&s->dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &s->pio);
#endif

    s->complete_reqs_bh = qemu_bh_new(pvscsi_complete_reqs, s);

    scsi_bus_new(&s->bus, &dev->qdev, &pvscsi_scsi_info);
    if (!dev->qdev.hotplugged) {
        return scsi_bus_legacy_handle_cmdline(&s->bus);
    }
    return 0;
}

static PCIDeviceInfo pvscsi_info = {
    .qdev.name  = "vmw_pvscsi",
    .qdev.size  = sizeof(PVSCSIState),
    .qdev.reset = pvscsi_reset,
    .init       = pvscsi_init,
    .exit       = pvscsi_uninit,
};

static void vmw_pvscsi_register_devices(void)
{
    pci_qdev_register(&pvscsi_info);
}

device_init(vmw_pvscsi_register_devices);
