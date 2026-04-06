#ifndef __RISCV_AI_EXT_HWLOOP_PERF_COMMON_HPP__
#define __RISCV_AI_EXT_HWLOOP_PERF_COMMON_HPP__

#include <cstddef>
#include <cstdint>

#include "perf_common.hpp"

#ifndef HWLOOP_OUTER_REPEATS
#error "HWLOOP_OUTER_REPEATS must be defined."
#endif

#ifndef HWLOOP_BENCH_NAME
#error "HWLOOP_BENCH_NAME must be defined."
#endif

namespace perf
{
namespace hwloop
{

constexpr std::size_t kKernelIterations = 32;
constexpr std::size_t kBodyInstructionCount = 8;
constexpr std::size_t kHwLoopImmediate = kBodyInstructionCount * 4;
constexpr std::size_t kOuterRepeats = HWLOOP_OUTER_REPEATS;

static_assert(kKernelIterations > 1);
static_assert(kHwLoopImmediate == 32);

struct KernelState
{
    long acc0;
    long acc1;
    long acc2;
    long acc3;
};

inline KernelState
seedState(std::size_t outer_idx)
{
    const long base = static_cast<long>(outer_idx + 1);
    return KernelState{
        0x0000'1101L + base * 13L,
        0x0000'2207L + base * 17L,
        0x0000'3311L + base * 19L,
        0x0000'4417L + base * 23L,
    };
}

inline std::uint64_t
finalizeState(const KernelState &state)
{
    std::uint64_t checksum = kChecksumSeed;
    checksum = mixChecksum(
        checksum, static_cast<std::uint64_t>(static_cast<std::uint32_t>(state.acc0)));
    checksum = mixChecksum(
        checksum, static_cast<std::uint64_t>(static_cast<std::uint32_t>(state.acc1)));
    checksum = mixChecksum(
        checksum, static_cast<std::uint64_t>(static_cast<std::uint32_t>(state.acc2)));
    checksum = mixChecksum(
        checksum, static_cast<std::uint64_t>(static_cast<std::uint32_t>(state.acc3)));
    return checksum;
}

template <typename Kernel>
inline std::uint64_t
runBenchmark(Kernel &&kernel)
{
    std::uint64_t checksum = kChecksumSeed;
    for (std::size_t outer_idx = 0; outer_idx < kOuterRepeats; ++outer_idx) {
        KernelState state = seedState(outer_idx);
        kernel(state);
        checksum = mixChecksum(checksum, finalizeState(state) ^ outer_idx);
    }
    return checksum;
}

inline void
printBenchmarkChecksum(std::uint64_t checksum)
{
    printBenchmarkResult(HWLOOP_BENCH_NAME, checksum);
}

} // namespace hwloop
} // namespace perf

#define HWLOOP_BODY_STEP \
    "addiw %[acc0], %[acc0], 13\n" \
    "xor %[acc1], %[acc1], %[acc0]\n" \
    "add %[acc2], %[acc2], %[acc1]\n" \
    "slliw %[acc3], %[acc3], 1\n" \
    "xori %[acc3], %[acc3], 7\n" \
    "add %[acc0], %[acc0], %[acc3]\n" \
    "xor %[acc2], %[acc2], %[acc0]\n" \
    "add %[acc1], %[acc1], %[acc2]\n"

#define HWLOOP_BODY_X1  HWLOOP_BODY_STEP
#define HWLOOP_BODY_X2  HWLOOP_BODY_X1 HWLOOP_BODY_X1
#define HWLOOP_BODY_X4  HWLOOP_BODY_X2 HWLOOP_BODY_X2
#define HWLOOP_BODY_X8  HWLOOP_BODY_X4 HWLOOP_BODY_X4
#define HWLOOP_BODY_X16 HWLOOP_BODY_X8 HWLOOP_BODY_X8
#define HWLOOP_BODY_X32 HWLOOP_BODY_X16 HWLOOP_BODY_X16

#endif // __RISCV_AI_EXT_HWLOOP_PERF_COMMON_HPP__
