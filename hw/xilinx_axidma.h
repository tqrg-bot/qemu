#ifndef XILINX_AXIDMA_H
#define XILINX_AXIDMA_H 1

/* AXI DMA connection. Used until qdev provides a generic way.  */
#define TYPE_XILINX_AXIDMA_PEER "xilinx-axidma-peer"

#define XILINX_AXIDMA_PEER_IFACE(klass) \
     OBJECT_CLASS_CHECK(XilinxAXIDMAPeerIface, klass, TYPE_XILINX_AXIDMA_PEER)

/* This is usually done implicitly by object_set_link_property.  */
#define XILINX_AXIDMA_PEER(obj) \
     OBJECT_CHECK(XilinxAXIDMAPeer, obj, TYPE_XILINX_AXIDMA_PEER)

typedef Interface XilinxAXIDMAPeer;
typedef struct XilinxAXIDMAPeerIface XilinxAXIDMAPeerIface;

struct XilinxAXIDMAPeerIface {
    InterfaceClass parent;

    void (*push)(Object *obj, unsigned char *buf, size_t len, uint32_t *app);
};

static inline
void xlx_dma_push(XilinxAXIDMAPeer *peer,
                  uint8_t *buf, size_t len, uint32_t *app)
{
    XilinxAXIDMAPeerIface *iface = container_of(INTERFACE_GET_CLASS(peer),
                                                XilinxAXIDMAPeerIface,
                                                parent);
    iface->push(INTERFACE_OBJECT(peer), buf, len, app);
}

#endif
