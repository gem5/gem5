#ifndef __RISCV_AI_EXT_CUSTOM_KERNELS_HPP__
#define __RISCV_AI_EXT_CUSTOM_KERNELS_HPP__

#include <cstdint>

namespace perf
{

inline std::uint64_t
reluClampCustom(long accumulator, std::uint16_t clamp_max)
{
    asm volatile(
        ".option push\n"
        ".option norvc\n"
        ".insn r 0x0b, 0x0, 0x2, %[acc], %[acc], x0\n"
        ".insn i 0x0b, 0x1, %[acc], %[acc], %[clamp]\n"
        ".option pop\n"
        : [acc] "+r"(accumulator)
        : [clamp] "i"(clamp_max)
        : "memory");

    return static_cast<std::uint64_t>(accumulator);
}

inline std::uint64_t
dot4CustomKernel(const std::uint32_t *activations,
    const std::uint32_t *weights, std::uint32_t group_count,
    std::uint16_t clamp_max)
{
    std::uintptr_t activation_ptr =
        reinterpret_cast<std::uintptr_t>(activations);
    std::uintptr_t weight_ptr =
        reinterpret_cast<std::uintptr_t>(weights);
    long accumulator = 0;
    long activation_word;
    long weight_word;

    asm volatile(
        ".option push\n"
        ".option norvc\n"
        "mv t0, %[group_count]\n"
        ".insn i 0x0b, 0x2, x0, t0, 96\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".insn i 0x0b, 0x3, %[act_word], %[act_ptr], 0\n"
        ".insn i 0x0b, 0x3, %[weight_word], %[weight_ptr], 0\n"
        ".insn r 0x0b, 0x0, 0x1, %[acc], %[act_word], %[weight_word]\n"
        ".option pop\n"
        : [acc] "+r"(accumulator),
          [act_ptr] "+r"(activation_ptr),
          [weight_ptr] "+r"(weight_ptr),
          [act_word] "=&r"(activation_word),
          [weight_word] "=&r"(weight_word)
        : [group_count] "r"(group_count)
        : "memory", "t0");

    return reluClampCustom(accumulator, clamp_max);
}

inline void
macRoundCustom(long &acc0, long &lhs0, long &rhs0,
    long &acc1, long &lhs1, long &rhs1,
    long &acc2, long &lhs2, long &rhs2,
    long &acc3, long &lhs3, long &rhs3)
{
    asm volatile(
        ".option push\n"
        ".option norvc\n"
        "addiw %[lhs0], %[lhs0], 13\n"
        "addiw %[rhs0], %[rhs0], -7\n"
        ".insn r 0x0b, 0x0, 0x0, %[acc0], %[lhs0], %[rhs0]\n"
        "addiw %[lhs1], %[lhs1], 17\n"
        "addiw %[rhs1], %[rhs1], -11\n"
        ".insn r 0x0b, 0x0, 0x0, %[acc1], %[lhs1], %[rhs1]\n"
        "addiw %[lhs2], %[lhs2], 19\n"
        "addiw %[rhs2], %[rhs2], -13\n"
        ".insn r 0x0b, 0x0, 0x0, %[acc2], %[lhs2], %[rhs2]\n"
        "addiw %[lhs3], %[lhs3], 23\n"
        "addiw %[rhs3], %[rhs3], -17\n"
        ".insn r 0x0b, 0x0, 0x0, %[acc3], %[lhs3], %[rhs3]\n"
        ".option pop\n"
        : [acc0] "+r"(acc0), [lhs0] "+r"(lhs0), [rhs0] "+r"(rhs0),
          [acc1] "+r"(acc1), [lhs1] "+r"(lhs1), [rhs1] "+r"(rhs1),
          [acc2] "+r"(acc2), [lhs2] "+r"(lhs2), [rhs2] "+r"(rhs2),
          [acc3] "+r"(acc3), [lhs3] "+r"(lhs3), [rhs3] "+r"(rhs3)
        :
        : "memory");
}

} // namespace perf

#endif // __RISCV_AI_EXT_CUSTOM_KERNELS_HPP__
