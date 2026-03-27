#include "custom_kernels.hpp"
#include "perf_common.hpp"

namespace
{

alignas(64) static std::uint32_t gOutputs[perf::kMacOutputs];

std::uint64_t
runBenchmark()
{
    std::uint64_t checksum = perf::kChecksumSeed;

    for (std::size_t block = 0; block < perf::kMacBlocks; ++block) {
        long lhs0 = static_cast<long>(
            perf::bitCastToInt32(0x0010'0101U + static_cast<std::uint32_t>(block * 29U)));
        long rhs0 = static_cast<long>(
            perf::bitCastToInt32(0xff10'0003U - static_cast<std::uint32_t>(block * 31U)));
        long lhs1 = static_cast<long>(
            perf::bitCastToInt32(0x0008'1007U + static_cast<std::uint32_t>(block * 37U)));
        long rhs1 = static_cast<long>(
            perf::bitCastToInt32(0xff20'0011U - static_cast<std::uint32_t>(block * 41U)));
        long lhs2 = static_cast<long>(
            perf::bitCastToInt32(0x0004'200dU + static_cast<std::uint32_t>(block * 43U)));
        long rhs2 = static_cast<long>(
            perf::bitCastToInt32(0xff40'0023U - static_cast<std::uint32_t>(block * 47U)));
        long lhs3 = static_cast<long>(
            perf::bitCastToInt32(0x0002'4017U + static_cast<std::uint32_t>(block * 53U)));
        long rhs3 = static_cast<long>(
            perf::bitCastToInt32(0xff80'0041U - static_cast<std::uint32_t>(block * 59U)));

        long acc0 = 0;
        long acc1 = 0;
        long acc2 = 0;
        long acc3 = 0;

        for (std::size_t iter = 0; iter < perf::kMacIterationsPerBlock; ++iter) {
            perf::macRoundCustom(
                acc0, lhs0, rhs0,
                acc1, lhs1, rhs1,
                acc2, lhs2, rhs2,
                acc3, lhs3, rhs3);
        }

        const std::uint64_t out0 =
            perf::reluClampCustom(acc0, perf::kMacClampMax);
        const std::uint64_t out1 =
            perf::reluClampCustom(acc1, perf::kMacClampMax);
        const std::uint64_t out2 =
            perf::reluClampCustom(acc2, perf::kMacClampMax);
        const std::uint64_t out3 =
            perf::reluClampCustom(acc3, perf::kMacClampMax);

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
    perf::printBenchmarkResult("mac_pipeline_custom", checksum);
    perf::syscallExit(0);
}
