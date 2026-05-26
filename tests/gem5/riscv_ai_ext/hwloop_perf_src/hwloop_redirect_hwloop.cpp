#include "hwloop_perf_common.hpp"

namespace
{

void
runHardwareLoopKernel(perf::hwloop::KernelState &state)
{
    const long loop_count = static_cast<long>(perf::hwloop::kKernelIterations);
    asm volatile(
        ".option push\n"
        ".option norvc\n"
        ".insn i 0x0b, 0x2, x0, %[count], " HWLOOP_LP_SETUP_RAW_IMM "\n"
        HWLOOP_BODY_STEP
        ".option pop\n"
        : [acc0] "+r"(state.acc0),
          [acc1] "+r"(state.acc1),
          [acc2] "+r"(state.acc2),
          [acc3] "+r"(state.acc3)
        : [count] "r"(loop_count)
        : "memory");
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    const std::uint32_t checksum =
        perf::hwloop::runBenchmark(runHardwareLoopKernel);
    perf::hwloop::printBenchmarkChecksum(checksum);
    perf::syscallExit(0);
}
