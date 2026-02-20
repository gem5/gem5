#ifndef __DMA_H__
#define __DMA_H__

#include "inttypes.h"
#include "stdlib.h"

#define MMR_ADDR 0x2ff00000
#define QUEUE_SIZE 8

volatile char *FLAGS = (char *)(MMR_ADDR);
volatile size_t *SRC = (size_t *)(MMR_ADDR + 1);
volatile size_t *DST = (size_t *)(MMR_ADDR + 9);
volatile int *LEN = (int *)(MMR_ADDR + 17);
volatile unsigned flag;

typedef struct
{
    size_t dst;
    size_t src;
    int len;
} DMA_XFER;

static int rdQueuePtr = 0;
static int wrQueuePtr = 0;
static int inQueue = 0;
static DMA_XFER jobs[QUEUE_SIZE];
static int valid[QUEUE_SIZE];

int launchCpy();
int dmacpy(void *dst, void *src, int len);
int pollDma();
void resetDma();

#endif //__DMA_H__
