#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../../../ext/nomali/lib/mali_midg_regmap.h"
#define ACCELERATOR_BASE 0x20000000

/*
void start_accelerator(uint32_t addr)
{
    volatile uint32_t *start_reg = (uint32_t *)ACCELERATOR_START;
    *start_reg = 1;
}

int is_accelerator_done()
{
    volatile uint32_t *done_reg = (uint32_t *)ACCELERATOR_DONE;
    return *done_reg; // Read from done register
}
*/

int main()
{
    //volatile uint32_t *gpu = (uint32_t *)(ACCELERATOR_BASE + GPU_CONTROL_BASE);
    //*gpu = 1;

    volatile uint32_t *job = (uint32_t *)(ACCELERATOR_BASE + JOB_CONTROL_BASE);
    //printf("address = %x, %x\n", ACCELERATOR_BASE, JOB_CONTROL_BASE);
    *job = 1;

    volatile uint32_t *job_slot = (uint32_t *)(ACCELERATOR_BASE + JOB_CONTROL_BASE + JOB_SLOT0);
    *job_slot = 1;

    // Choose which job;
    volatile uint32_t *job_command = (uint32_t *)(ACCELERATOR_BASE + JOB_CONTROL_BASE + JOB_SLOT0 + 0x20);
    *job_command = 0;

    volatile uint32_t *mmu = (uint32_t *)(ACCELERATOR_BASE + MEMORY_MANAGEMENT_BASE);
    *mmu = 1;


    return 0;
}