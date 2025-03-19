#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define ACCELERATOR_BASE 0x80000000

int main()
{
    volatile uint32_t *gpu = (uint32_t *)(ACCELERATOR_BASE + 0x1);
    *gpu = 1;

    return 0;
}