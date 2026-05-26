#ifndef __RISCV_AI_EXT_CUSTOM_KERNELS_HPP__
#define __RISCV_AI_EXT_CUSTOM_KERNELS_HPP__

#include <cstdint>

namespace perf
{

inline std::int32_t
macCustom(std::int32_t accumulator, std::int32_t lhs, std::int32_t rhs)
{
    long acc_reg = accumulator;
    const long lhs_reg = lhs;
    const long rhs_reg = rhs;

    asm volatile(
        ".option push\n"
        ".option norvc\n"
        ".insn r 0x0b, 0x0, 0x0, %[acc], %[lhs], %[rhs]\n"
        ".option pop\n"
        : [acc] "+&r"(acc_reg)
        : [lhs] "r"(lhs_reg), [rhs] "r"(rhs_reg)
        : "memory");

    return static_cast<std::int32_t>(acc_reg);
}

inline std::int32_t
dot4AccCustom(std::int32_t accumulator, std::uint32_t activation_word,
    std::uint32_t weight_word)
{
    long acc_reg = accumulator;
    const long activation_reg = static_cast<long>(activation_word);
    const long weight_reg = static_cast<long>(weight_word);

    asm volatile(
        ".option push\n"
        ".option norvc\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[activation], %[weight]\n"
        ".option pop\n"
        : [acc] "+&r"(acc_reg)
        : [activation] "r"(activation_reg), [weight] "r"(weight_reg)
        : "memory");

    return static_cast<std::int32_t>(acc_reg);
}

inline std::uint32_t
clampCustom(std::int32_t value, std::uint32_t upper_bound)
{
    long out_reg;
    const long value_reg = value;
    const long upper_reg = static_cast<long>(upper_bound);

    asm volatile(
        ".option push\n"
        ".option norvc\n"
        ".insn r 0x0b, 0x1, 0x0, %[out], %[value], %[upper]\n"
        ".option pop\n"
        : [out] "=&r"(out_reg)
        : [value] "r"(value_reg), [upper] "r"(upper_reg)
        : "memory");

    return static_cast<std::uint32_t>(out_reg);
}

inline std::uint32_t
dot4PlwLpCustomKernel(const std::uint32_t *activations,
    const std::uint32_t *weights, std::uint32_t group_count,
    std::uint32_t clamp_max)
{
    std::uintptr_t activation_ptr =
        reinterpret_cast<std::uintptr_t>(activations);
    std::uintptr_t weight_ptr =
        reinterpret_cast<std::uintptr_t>(weights);
    long accumulator = 0;
    long activation_word;
    long weight_word;
    const long loop_count = group_count;

    asm volatile(
        ".option push\n"
        ".option norvc\n"
        ".balign 4\n"
        "mv t0, %[loop_count]\n"
        ".insn i 0x0b, 0x2, x0, t0, 4\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 4\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 4\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".option pop\n"
        : [acc] "+&r"(accumulator),
          [act_ptr] "+r"(activation_ptr),
          [weight_ptr] "+r"(weight_ptr),
          [act_word] "=&r"(activation_word),
          [weight_word] "=&r"(weight_word)
        : [loop_count] "r"(loop_count)
        : "memory", "t0");

    return clampCustom(static_cast<std::int32_t>(accumulator), clamp_max);
}

} // namespace perf

#endif // __RISCV_AI_EXT_CUSTOM_KERNELS_HPP__
