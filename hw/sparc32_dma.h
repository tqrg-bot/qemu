#ifndef SPARC32_DMA_H
#define SPARC32_DMA_H

/* sparc32_dma.c */
void ledma_memory_read(DeviceState *dma, target_phys_addr_t addr,
                       uint8_t *buf, int len, int do_bswap);
void ledma_memory_write(DeviceState *dma, target_phys_addr_t addr,
                        uint8_t *buf, int len, int do_bswap);
void espdma_memory_read(DeviceState *dma, uint8_t *buf, int len);
void espdma_memory_write(DeviceState *dma, uint8_t *buf, int len);

#endif
