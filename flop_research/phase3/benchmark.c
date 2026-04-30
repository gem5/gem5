/**
 * benchmark.c — LVP Performance Microbenchmark (Pointer Chasing)
 *
 * Goal: Quantitatively prove that the LVP speeds up execution by breaking
 * a memory-bound critical path.
 */

#include <stdio.h>
#include <stdint.h>

/* A trivial linked list or index array where the element points to itself.
 * memory[0] = 0. */
volatile uint32_t memory[1] = {0};

int main(void)
{
    uint32_t current_idx = 0;
    int i;

    printf("Starting pointer-chasing RAW benchmark...\n");

    /*
     * The loop runs 10,000 times.
     * 
     * In each iteration, the address of the next load depends on the VALUE
     * of the current load (current_idx = memory[current_idx]).
     * This creates a strict Read-After-Write (RAW) dependency chain on the
     * memory loads themselves. The Out-of-Order CPU cannot fetch the next
     * iteration's load in parallel because it doesn't know the address yet!
     * 
     * With LVP OFF:
     * The CPU must wait for the L1 cache (2-3 cycles) to return '0' before
     * it can calculate the address for the next load.
     * 
     * With LVP ON:
     * The predictor learns this PC always returns '0'. It forwards '0' at
     * the Rename stage. The CPU immediately calculates the next address and
     * keeps executing without stalling for the L1 cache!
     */
    for (i = 0; i < 10000; i++) {
        current_idx = memory[current_idx];
    }

    printf("Benchmark complete. Final index = %d\n", current_idx);
    return 0;
}
