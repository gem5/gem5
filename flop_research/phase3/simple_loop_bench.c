/**
 * simple_loop_bench.c — Out-of-Order MLP Demonstration
 *
 * Goal: Demonstrate how Memory-Level Parallelism (MLP) in an Out-of-Order
 * CPU completely hides memory latency for independent loads, rendering
 * Load Value Prediction (LVP) unnecessary for performance in this scenario.
 */

#include <stdio.h>
#include <stdint.h>

volatile int32_t target_value = 1;

int main(void)
{
    int32_t sum = 0;
    int i;

    printf("Starting simple loop benchmark (MLP test)...\n");

    /*
     * The loop runs 10,000 times.
     * 
     * In each iteration, 'target_value' is loaded from a CONSTANT address.
     * Because the address does not depend on the previous iteration, the
     * Out-of-Order (O3) CPU can issue dozens of loads in parallel to the
     * L1 cache (Memory-Level Parallelism).
     * 
     * The latency of the L1 cache (2-3 cycles) is completely hidden by this
     * parallel prefetching. The only bottleneck is the ALU 'add' instruction
     * waiting for the previous 'add' (sum depends on sum).
     * 
     * Result:
     * Turning LVP ON or OFF yields the exact same execution time, because the
     * memory load is NOT on the critical path. The CPU executes this loop
     * at the maximum speed of the ALU (1 cycle per iteration), regardless of
     * the cache speed!
     */
    for (i = 0; i < 10000; i++) {
        sum += target_value;
    }

    printf("Benchmark complete. Sum = %d\n", sum);
    return 0;
}
