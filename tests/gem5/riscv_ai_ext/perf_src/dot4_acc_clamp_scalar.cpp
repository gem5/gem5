#include "perf_common.hpp"

namespace
{

alignas(64) static std::uint32_t gActivations[perf::kDotDataWords];
alignas(64) static std::uint32_t gWeights[perf::kDotDataWords];

std::uint32_t
runBenchmark()
{
    std::uint32_t checksum = perf::kDotChecksumSeed;

    for (std::size_t output_idx = 0; output_idx < perf::kDotOutputs;
         ++output_idx) {
        const std::size_t base = perf::dotBase(output_idx);
        std::int32_t accumulator = 0;

        for (std::size_t group = 0; group < perf::kDotGroups; ++group) {
            accumulator += perf::dot4ScalarStep(
                gActivations[base + group],
                gWeights[base + group]);
        }

        const std::uint32_t clamped =
            perf::clampScalar(accumulator, perf::kDotClampMax);
        checksum = perf::mix32(
            checksum,
            clamped ^ (static_cast<std::uint32_t>(output_idx) * 0x119de1f3U));
    }

    perf::gSink = checksum;
    return checksum;
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    perf::fillDot4Inputs(gActivations, gWeights);
    perf::beginKernelStats();
    const std::uint32_t checksum = runBenchmark();
    perf::endKernelStats();
    perf::printBenchmarkResult("dot4_acc_clamp_scalar", checksum);
    perf::syscallExit(0);
}
