#ifndef __DMA_H__
#define __DMA_H__

#define MMR_ADDR 0x2ff00000

volatile char *FLAGS = (char *)(MMR_ADDR);
volatile size_t *SRC = (size_t *)(MMR_ADDR + 1);
volatile size_t *DST = (size_t *)(MMR_ADDR + 9);
volatile int *LEN = (int *)(MMR_ADDR + 17);
volatile unsigned flag;

void
dmacpy(void *dst, void *src, int len)
{
    *SRC = (size_t)src;
    *DST = (size_t)dst;
    *LEN = len;
    *FLAGS |= 0x01;
}

int
pollDma()
{
    return ((*FLAGS & 0x04) == 0x04);
}
void
resetDma()
{
    *FLAGS = 0;
}

#endif //__DMA_H__
