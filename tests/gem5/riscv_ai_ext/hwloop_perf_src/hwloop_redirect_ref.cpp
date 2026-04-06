#include "hwloop_perf_common.hpp"

namespace
{

void
runRefKernel(perf::hwloop::KernelState &state)
{
    asm volatile(
        ".option push\n"
        ".option norvc\n"
        HWLOOP_BODY_X32
        ".option pop\n"
        : [acc0] "+r"(state.acc0),
          [acc1] "+r"(state.acc1),
          [acc2] "+r"(state.acc2),
          [acc3] "+r"(state.acc3)
        :
        : "memory");
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    const std::uint64_t checksum = perf::hwloop::runBenchmark(runRefKernel);
    perf::hwloop::printBenchmarkChecksum(checksum);
    perf::syscallExit(0);
}
