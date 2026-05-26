#include "perf_common.hpp"

namespace
{

alignas(64) static std::int32_t gMacLhs[perf::kMacDataWords];
alignas(64) static std::int32_t gMacRhs[perf::kMacDataWords];

std::uint32_t
runBenchmark()
{
    std::uint32_t checksum = perf::kMacChecksumSeed;

    for (std::size_t output_idx = 0; output_idx < perf::kMacOutputs;
         ++output_idx) {
        std::int32_t accumulator =
            static_cast<std::int32_t>(output_idx & 15U) - 8;

        for (std::size_t tap = 0; tap < perf::kMacTaps; ++tap) {
            const std::size_t lhs_idx =
                (output_idx + tap * 3U) & (perf::kMacDataWords - 1U);
            const std::size_t rhs_idx =
                (output_idx * 5U + tap) & (perf::kMacDataWords - 1U);
            accumulator += gMacLhs[lhs_idx] * gMacRhs[rhs_idx];
        }

        const std::uint32_t clamped =
            perf::clampScalar(accumulator, perf::kMacClampMax);
        checksum = perf::mix32(
            checksum,
            clamped ^ (static_cast<std::uint32_t>(output_idx) * 0x45d9f3bU));
    }

    perf::gSink = checksum;
    return checksum;
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    perf::fillMacInputs(gMacLhs, gMacRhs);
    perf::beginKernelStats();
    const std::uint32_t checksum = runBenchmark();
    perf::endKernelStats();
    perf::printBenchmarkResult("mac_clamp_scalar", checksum);
    perf::syscallExit(0);
}
