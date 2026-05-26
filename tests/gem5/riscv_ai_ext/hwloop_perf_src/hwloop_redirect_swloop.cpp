#include "hwloop_perf_common.hpp"

namespace
{

void
runSoftwareLoopKernel(perf::hwloop::KernelState &state)
{
    const long loop_count = static_cast<long>(perf::hwloop::kKernelIterations);
    asm volatile(
        ".option push\n"
        ".option norvc\n"
        "mv t0, %[count]\n"
        "1:\n"
        HWLOOP_BODY_STEP
        "addiw t0, t0, -1\n"
        "bnez t0, 1b\n"
        ".option pop\n"
        : [acc0] "+r"(state.acc0),
          [acc1] "+r"(state.acc1),
          [acc2] "+r"(state.acc2),
          [acc3] "+r"(state.acc3)
        : [count] "r"(loop_count)
        : "memory", "t0");
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    const std::uint32_t checksum =
        perf::hwloop::runBenchmark(runSoftwareLoopKernel);
    perf::hwloop::printBenchmarkChecksum(checksum);
    perf::syscallExit(0);
}
