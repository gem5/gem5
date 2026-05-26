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

#define HWLOOP_LP_SETUP_RAW_IMM_VALUE 9

namespace perf
{
namespace hwloop
{

constexpr std::size_t kKernelIterations = 32;
constexpr std::size_t kBodyInstructionCount = 8;
constexpr std::size_t kHwLoopRawImmediate = HWLOOP_LP_SETUP_RAW_IMM_VALUE;
constexpr std::size_t kOuterRepeats = HWLOOP_OUTER_REPEATS;
constexpr std::uint32_t kChecksumSeed = 0x48574c50U;

static_assert(kKernelIterations > 1);
static_assert(kHwLoopRawImmediate == kBodyInstructionCount + 1);

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

inline std::uint32_t
finalizeState(const KernelState &state)
{
    std::uint32_t checksum = kChecksumSeed;
    checksum = mix32(checksum, static_cast<std::uint32_t>(state.acc0));
    checksum = mix32(checksum, static_cast<std::uint32_t>(state.acc1));
    checksum = mix32(checksum, static_cast<std::uint32_t>(state.acc2));
    checksum = mix32(checksum, static_cast<std::uint32_t>(state.acc3));
    return checksum;
}

template <typename Kernel>
inline std::uint32_t
runBenchmark(Kernel &&kernel)
{
    std::uint32_t checksum = kChecksumSeed;
    for (std::size_t outer_idx = 0; outer_idx < kOuterRepeats; ++outer_idx) {
        KernelState state = seedState(outer_idx);
        kernel(state);
        checksum = mix32(checksum, finalizeState(state) ^
            static_cast<std::uint32_t>(outer_idx));
    }
    return checksum;
}

inline void
printBenchmarkChecksum(std::uint32_t checksum)
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

#define HWLOOP_STRINGIFY_DETAIL(value) #value
#define HWLOOP_STRINGIFY(value) HWLOOP_STRINGIFY_DETAIL(value)
#define HWLOOP_LP_SETUP_RAW_IMM HWLOOP_STRINGIFY(HWLOOP_LP_SETUP_RAW_IMM_VALUE)

#endif // __RISCV_AI_EXT_HWLOOP_PERF_COMMON_HPP__
