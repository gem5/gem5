#include "custom_kernels.hpp"
#include "perf_common.hpp"

namespace
{

alignas(64) static std::uint32_t gActivations[perf::kDotTotalPacks];
alignas(64) static std::uint32_t gWeights[perf::kDotTotalPacks];
alignas(64) static std::uint32_t gOutputs[perf::kDotOutputs];

std::uint64_t
runBenchmark()
{
    perf::fillDot4Inputs(gActivations, gWeights);

    std::uint64_t checksum = perf::kChecksumSeed;

    for (std::size_t output_idx = 0; output_idx < perf::kDotOutputs;
         ++output_idx) {
        const std::size_t base = output_idx * perf::kDotPacksPerOutput;
        const std::uint64_t activated = perf::dot4CustomKernel(
            gActivations + base, gWeights + base,
            perf::kDotPacksPerOutput / perf::kCustomUnroll,
            perf::kDotClampMax);

        gOutputs[output_idx] = static_cast<std::uint32_t>(activated);
        checksum = perf::mixChecksum(checksum, activated ^ output_idx);
    }

    return checksum;
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    const std::uint64_t checksum = runBenchmark();
    perf::printBenchmarkResult("dot4_pipeline_custom", checksum);
    perf::syscallExit(0);
}
