"""
Shared constants for hardware-loop microbenchmarks.

Must stay in sync with:
- hwloop_perf_src/hwloop_perf_common.hpp (kKernelIterations)
- run_hwloop_perf_compare.py (BENCHMARKS outer_repeats)
"""

KERNEL_ITERATIONS = 32


def expected_loop_backs(outer_repeats: int) -> int:
    """
    Logical loop-backs for the hwloop_redirect_hwloop kernel: each outer repeat
    runs the inner kernel with count kKernelIterations, which performs
    (kKernelIterations - 1) back-edges to the loop start.
    """
    return outer_repeats * (KERNEL_ITERATIONS - 1)
