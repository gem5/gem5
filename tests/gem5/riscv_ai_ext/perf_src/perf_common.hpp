#ifndef __RISCV_AI_EXT_PERF_COMMON_HPP__
#define __RISCV_AI_EXT_PERF_COMMON_HPP__

#include <cstddef>
#include <cstdint>

#include <gem5/m5ops.h>

namespace perf
{

constexpr std::size_t kMacDataWords = 64;
constexpr std::size_t kMacOutputs = 512;
constexpr std::size_t kMacTaps = 32;
constexpr std::uint32_t kMacClampMax = 2047;
constexpr std::uint32_t kMacChecksumSeed = 0x31415926U;

constexpr std::size_t kDotDataWords = 256;
constexpr std::size_t kDotOutputs = 128;
constexpr std::size_t kDotGroups = 64;
constexpr std::uint32_t kDotClampMax = 255;
constexpr std::uint32_t kDotChecksumSeed = 0x27182818U;

static_assert(kMacDataWords == 64);
static_assert(kDotDataWords == 4 * kDotGroups);

inline std::size_t
cstringLen(const char *text)
{
    std::size_t len = 0;
    while (text[len] != '\0') {
        ++len;
    }
    return len;
}

inline long
syscallWrite(int fd, const void *buffer, std::size_t len)
{
    register long a0 asm("a0") = fd;
    register long a1 asm("a1") =
        reinterpret_cast<long>(buffer);
    register long a2 asm("a2") = static_cast<long>(len);
    register long a7 asm("a7") = 64;

    asm volatile(
        "ecall"
        : "+r"(a0)
        : "r"(a1), "r"(a2), "r"(a7)
        : "memory");

    return a0;
}

[[noreturn]] inline void
syscallExit(int code)
{
    register long a0 asm("a0") = code;
    register long a7 asm("a7") = 93;

    asm volatile(
        "ecall"
        :
        : "r"(a0), "r"(a7)
        : "memory");

    __builtin_unreachable();
}

inline void
writeBuffer(const char *buffer, std::size_t len)
{
    (void)syscallWrite(1, buffer, len);
}

inline void
appendText(char *buffer, std::size_t &len, const char *text)
{
    for (std::size_t i = 0; text[i] != '\0'; ++i) {
        buffer[len++] = text[i];
    }
}

inline void
appendHex32(char *buffer, std::size_t &len, std::uint32_t value)
{
    static constexpr char kHexDigits[] = "0123456789abcdef";

    buffer[len++] = '0';
    buffer[len++] = 'x';

    for (int nibble = 7; nibble >= 0; --nibble) {
        const std::uint64_t shift = static_cast<std::uint64_t>(nibble * 4);
        buffer[len++] = kHexDigits[(value >> shift) & 0xfU];
    }
}

inline void
printBenchmarkResult(const char *name, std::uint32_t checksum)
{
    char buffer[128];
    std::size_t len = 0;

    appendText(buffer, len, "BENCH_RESULT ");
    appendText(buffer, len, name);
    appendText(buffer, len, " ");
    appendHex32(buffer, len, checksum);
    buffer[len++] = '\n';

    writeBuffer(buffer, len);
}

inline void
compilerFence()
{
    asm volatile("" ::: "memory");
}

inline void
beginKernelStats()
{
    compilerFence();
    m5_reset_stats(0, 0);
    compilerFence();
}

inline void
endKernelStats()
{
    compilerFence();
    m5_dump_reset_stats(0, 0);
    compilerFence();
}

inline std::int32_t
wrapI8(std::uint32_t value, std::uint32_t mul, std::uint32_t add)
{
    return static_cast<std::int32_t>((value * mul + add) & 0x7fU) - 64;
}

inline std::uint32_t
packInt8x4(std::int32_t b0, std::int32_t b1,
    std::int32_t b2, std::int32_t b3)
{
    return (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b0)) << 0) |
           (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b1)) << 8) |
           (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b2)) << 16) |
           (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b3)) << 24);
}

inline void
fillMacInputs(std::int32_t *lhs, std::int32_t *rhs)
{
    for (std::size_t i = 0; i < kMacDataWords; ++i) {
        const auto idx = static_cast<std::uint32_t>(i);
        lhs[i] = wrapI8(idx, 13U, 5U);
        rhs[i] = wrapI8(idx, 17U, 9U);
    }
}

inline void
fillDot4Inputs(std::uint32_t *activations, std::uint32_t *weights)
{
    for (std::size_t i = 0; i < kDotDataWords; ++i) {
        const auto idx = static_cast<std::uint32_t>(i);

        activations[i] = packInt8x4(
            wrapI8(idx, 13U, 5U),
            wrapI8(idx, 17U, 9U),
            wrapI8(idx, 19U, 3U),
            wrapI8(idx, 23U, 11U));
        weights[i] = packInt8x4(
            wrapI8(idx, 29U, 7U),
            wrapI8(idx, 31U, 1U),
            wrapI8(idx, 37U, 13U),
            wrapI8(idx, 41U, 15U));
    }
}

inline std::size_t
dotBase(std::size_t output_idx)
{
    return ((output_idx * 37U) & 3U) * kDotGroups;
}

inline std::int32_t
dot4ScalarStep(std::uint32_t activation_word, std::uint32_t weight_word)
{
    std::int32_t dot = 0;
    for (int lane = 0; lane < 4; ++lane) {
        const std::int8_t activation =
            static_cast<std::int8_t>(activation_word >> (lane * 8));
        const std::int8_t weight =
            static_cast<std::int8_t>(weight_word >> (lane * 8));
        dot += static_cast<std::int32_t>(activation) *
               static_cast<std::int32_t>(weight);
    }
    return dot;
}

inline std::uint32_t
clampScalar(std::int32_t value, std::uint32_t upper_bound)
{
    if (value < 0) {
        return 0;
    }

    const std::uint32_t unsigned_value = static_cast<std::uint32_t>(value);
    return unsigned_value > upper_bound ? upper_bound : unsigned_value;
}

inline std::uint32_t
mix32(std::uint32_t state, std::uint32_t value)
{
    state ^= value + 0x9e3779b9U + (state << 6) + (state >> 2);
    state ^= state >> 16;
    state *= 0x7feb352dU;
    state ^= state >> 15;
    state *= 0x846ca68bU;
    state ^= state >> 16;
    return state;
}

inline volatile std::uint32_t gSink;

} // namespace perf

#endif // __RISCV_AI_EXT_PERF_COMMON_HPP__
