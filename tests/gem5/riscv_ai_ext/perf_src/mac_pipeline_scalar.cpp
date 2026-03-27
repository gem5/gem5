#include "perf_common.hpp"

namespace
{

alignas(64) static std::uint32_t gOutputs[perf::kMacOutputs];

std::uint64_t
runBenchmark()
{
    std::uint64_t checksum = perf::kChecksumSeed;

    for (std::size_t block = 0; block < perf::kMacBlocks; ++block) {
        std::uint32_t lhs0 = 0x0010'0101U + static_cast<std::uint32_t>(block * 29U);
        std::uint32_t rhs0 = 0xff10'0003U - static_cast<std::uint32_t>(block * 31U);
        std::uint32_t lhs1 = 0x0008'1007U + static_cast<std::uint32_t>(block * 37U);
        std::uint32_t rhs1 = 0xff20'0011U - static_cast<std::uint32_t>(block * 41U);
        std::uint32_t lhs2 = 0x0004'200dU + static_cast<std::uint32_t>(block * 43U);
        std::uint32_t rhs2 = 0xff40'0023U - static_cast<std::uint32_t>(block * 47U);
        std::uint32_t lhs3 = 0x0002'4017U + static_cast<std::uint32_t>(block * 53U);
        std::uint32_t rhs3 = 0xff80'0041U - static_cast<std::uint32_t>(block * 59U);

        std::int64_t acc0 = 0;
        std::int64_t acc1 = 0;
        std::int64_t acc2 = 0;
        std::int64_t acc3 = 0;

        for (std::size_t iter = 0; iter < perf::kMacIterationsPerBlock; ++iter) {
            lhs0 += 13U;
            rhs0 -= 7U;
            acc0 += static_cast<std::int64_t>(perf::bitCastToInt32(lhs0)) *
                    static_cast<std::int64_t>(perf::bitCastToInt32(rhs0));

            lhs1 += 17U;
            rhs1 -= 11U;
            acc1 += static_cast<std::int64_t>(perf::bitCastToInt32(lhs1)) *
                    static_cast<std::int64_t>(perf::bitCastToInt32(rhs1));

            lhs2 += 19U;
            rhs2 -= 13U;
            acc2 += static_cast<std::int64_t>(perf::bitCastToInt32(lhs2)) *
                    static_cast<std::int64_t>(perf::bitCastToInt32(rhs2));

            lhs3 += 23U;
            rhs3 -= 17U;
            acc3 += static_cast<std::int64_t>(perf::bitCastToInt32(lhs3)) *
                    static_cast<std::int64_t>(perf::bitCastToInt32(rhs3));
        }

        const std::uint64_t out0 =
            perf::reluClampScalar(acc0, perf::kMacClampMax);
        const std::uint64_t out1 =
            perf::reluClampScalar(acc1, perf::kMacClampMax);
        const std::uint64_t out2 =
            perf::reluClampScalar(acc2, perf::kMacClampMax);
        const std::uint64_t out3 =
            perf::reluClampScalar(acc3, perf::kMacClampMax);

        const std::size_t out_base = block * 4;
        gOutputs[out_base + 0] = static_cast<std::uint32_t>(out0);
        gOutputs[out_base + 1] = static_cast<std::uint32_t>(out1);
        gOutputs[out_base + 2] = static_cast<std::uint32_t>(out2);
        gOutputs[out_base + 3] = static_cast<std::uint32_t>(out3);

        checksum = perf::mixChecksum(checksum, out0 ^ out_base);
        checksum = perf::mixChecksum(checksum, out1 ^ (out_base + 1));
        checksum = perf::mixChecksum(checksum, out2 ^ (out_base + 2));
        checksum = perf::mixChecksum(checksum, out3 ^ (out_base + 3));
    }

    return checksum;
}

} // namespace

extern "C" [[noreturn]] void
_start()
{
    const std::uint64_t checksum = runBenchmark();
    perf::printBenchmarkResult("mac_pipeline_scalar", checksum);
    perf::syscallExit(0);
}
