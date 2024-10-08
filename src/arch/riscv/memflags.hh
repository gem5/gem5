#ifndef __ARCH_RISCV_MEMFLAGS_HH__
#define __ARCH_RISCV_MEMFLAGS_HH__


namespace gem5
{


namespace RiscvISA {

    // We can only utilize the lower 8 bits of a
    // 64-bit value to encode these.
    // see src/mem/request.hh ARCH_BITS
    // Lower 3 bits are already used in mmu.hh
    // for alignment flags
    enum XlateFlags
    {
        HLVX = 1ULL << 3,
        FORCE_VIRT = 1ULL << 4,
        LR = 1ULL << 5,
    };

} // namespace RiscvISA
} // namespace gem5

#endif //__ARCH_RISCV_MEMFLAGS_HH__
