#ifndef __RISCV_AI_EXT_PERF_COMMON_HPP__
#define __RISCV_AI_EXT_PERF_COMMON_HPP__

#include <cstddef>
#include <cstdint>

namespace perf
{

constexpr std::size_t kDotOutputs = 256;
constexpr std::size_t kDotPacksPerOutput = 2048;
constexpr std::size_t kDotTotalPacks = kDotOutputs * kDotPacksPerOutput;
constexpr std::uint16_t kDotClampMax = 255;
constexpr std::size_t kCustomUnroll = 8;

constexpr std::size_t kMacBlocks = 256;
constexpr std::size_t kMacIterationsPerBlock = 4096;
constexpr std::size_t kMacOutputs = kMacBlocks * 4;
constexpr std::uint16_t kMacClampMax = 2047;

constexpr std::uint64_t kChecksumSeed = 0xcbf29ce484222325ULL;

static_assert(kDotPacksPerOutput % kCustomUnroll == 0);

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
appendHex(char *buffer, std::size_t &len, std::uint64_t value)
{
    static constexpr char kHexDigits[] = "0123456789abcdef";

    buffer[len++] = '0';
    buffer[len++] = 'x';

    for (int nibble = 15; nibble >= 0; --nibble) {
        const std::uint64_t shift = static_cast<std::uint64_t>(nibble * 4);
        buffer[len++] = kHexDigits[(value >> shift) & 0xfU];
    }
}

inline void
printBenchmarkResult(const char *name, std::uint64_t checksum)
{
    char buffer[128];
    std::size_t len = 0;

    appendText(buffer, len, "BENCH_RESULT ");
    appendText(buffer, len, name);
    appendText(buffer, len, " ");
    appendHex(buffer, len, checksum);
    buffer[len++] = '\n';

    writeBuffer(buffer, len);
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
fillDot4Inputs(std::uint32_t *activations, std::uint32_t *weights)
{
    for (std::size_t i = 0; i < kDotTotalPacks; ++i) {
        const std::int32_t base = static_cast<std::int32_t>(i);
        const std::int32_t a0 = ((base * 13 + 5) & 0x7f) - 64;
        const std::int32_t a1 = ((base * 17 + 9) & 0x7f) - 64;
        const std::int32_t a2 = ((base * 19 + 3) & 0x7f) - 64;
        const std::int32_t a3 = ((base * 23 + 11) & 0x7f) - 64;
        const std::int32_t w0 = ((base * 29 + 7) & 0x7f) - 64;
        const std::int32_t w1 = ((base * 31 + 1) & 0x7f) - 64;
        const std::int32_t w2 = ((base * 37 + 13) & 0x7f) - 64;
        const std::int32_t w3 = ((base * 41 + 15) & 0x7f) - 64;

        activations[i] = packInt8x4(a0, a1, a2, a3);
        weights[i] = packInt8x4(w0, w1, w2, w3);
    }
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

inline std::int32_t
bitCastToInt32(std::uint32_t value)
{
    return static_cast<std::int32_t>(value);
}

inline std::uint64_t
reluClampScalar(std::int64_t value, std::uint16_t upper_bound)
{
    if (value < 0) {
        return 0;
    }

    const std::uint64_t unsigned_value = static_cast<std::uint64_t>(value);
    return unsigned_value > upper_bound ? upper_bound : unsigned_value;
}

inline std::uint64_t
mixChecksum(std::uint64_t checksum, std::uint64_t value)
{
    checksum ^= value + 0x9e3779b97f4a7c15ULL + (checksum << 6) +
                (checksum >> 2);
    checksum *= 0x100000001b3ULL;
    return checksum;
}

} // namespace perf

#endif // __RISCV_AI_EXT_PERF_COMMON_HPP__
