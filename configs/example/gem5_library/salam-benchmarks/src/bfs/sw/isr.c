#include <stdio.h>

#include "bench.h"

extern volatile uint8_t *top;

void
isr(void)
{
    printf("Interrupt\n");
    stage += 1;
    *top = 0x00;
    // printf("%d\n", *top);
    printf("Interrupt finished\n");
}
