#ifndef __ARCH_RISCV_FAULT_CODES_HH__
#define __ARCH_RISCV_FAULT_CODES_HH__

#include "base/types.hh"

namespace gem5
{

namespace RiscvISA
{



enum FloatException : uint64_t
{
    FloatInexact = 0x1,
    FloatUnderflow = 0x2,
    FloatOverflow = 0x4,
    FloatDivZero = 0x8,
    FloatInvalid = 0x10
};

/*
 * In RISC-V, exception and interrupt codes share some values. They can be
 * differentiated by an 'Interrupt' flag that is enabled for interrupt faults
 * but not exceptions. The full fault cause can be computed by placing the
 * exception (or interrupt) code in the least significant bits of the CAUSE
 * CSR and then setting the highest bit of CAUSE with the 'Interrupt' flag.
 * For more details on exception causes, see Chapter 3.1.20 of the RISC-V
 * privileged specification v 1.10. Codes are enumerated in Table 3.6.
 */
enum ExceptionCode : uint64_t
{
    INST_ADDR_MISALIGNED = 0,
    INST_ACCESS = 1,
    INST_ILLEGAL = 2,
    BREAKPOINT = 3,
    LOAD_ADDR_MISALIGNED = 4,
    LOAD_ACCESS = 5,
    STORE_ADDR_MISALIGNED = 6,
    AMO_ADDR_MISALIGNED = 6,
    STORE_ACCESS = 7,
    AMO_ACCESS = 7,
    ECALL_USER = 8,
    ECALL_SUPER = 9,
    ECALL_VIRTUAL_SUPER = 10, // H-extension
    ECALL_MACHINE = 11,
    INST_PAGE = 12,
    LOAD_PAGE = 13,
    STORE_PAGE = 15,
    AMO_PAGE = 15,
    INST_GUEST_PAGE = 20, // H-extension
    LOAD_GUEST_PAGE = 21, // H-extension
    VIRTUAL_INST = 22,    // H-extension
    STORE_GUEST_PAGE = 23,// H-extension
    AMO_GUEST_PAGE = 23,  // H-extension

    INT_SOFTWARE_USER = 0,
    INT_SOFTWARE_SUPER = 1,
    INT_SOFTWARE_VIRTUAL_SUPER = 2, // H-extension
    INT_SOFTWARE_MACHINE = 3,
    INT_TIMER_USER = 4,
    INT_TIMER_SUPER = 5,
    INT_TIMER_VIRTUAL_SUPER = 6, // H-extension
    INT_TIMER_MACHINE = 7,
    INT_EXT_USER = 8,
    INT_EXT_SUPER = 9,
    INT_EXT_VIRTUAL_SUPER = 10, // H-extension
    INT_EXT_MACHINE = 11,
    INT_EXT_SUPER_GUEST = 12, // H-extension
    INT_LOCAL_0 = 16,
    INT_LOCAL_1 = 17,
    INT_LOCAL_2 = 18,
    INT_LOCAL_3 = 19,
    INT_LOCAL_4 = 20,
    INT_LOCAL_5 = 21,
    INT_LOCAL_6 = 22,
    INT_LOCAL_7 = 23,
    INT_LOCAL_8 = 24,
    INT_LOCAL_9 = 25,
    INT_LOCAL_10 = 26,
    INT_LOCAL_11 = 27,
    INT_LOCAL_12 = 28,
    INT_LOCAL_13 = 29,
    INT_LOCAL_14 = 30,
    INT_LOCAL_15 = 31,
    INT_LOCAL_16 = 32,
    INT_LOCAL_17 = 33,
    INT_LOCAL_18 = 34,
    INT_LOCAL_19 = 35,
    INT_LOCAL_20 = 36,
    INT_LOCAL_21 = 37,
    INT_LOCAL_22 = 38,
    INT_LOCAL_23 = 39,
    INT_LOCAL_24 = 40,
    INT_LOCAL_25 = 41,
    INT_LOCAL_26 = 42,
    INT_LOCAL_27 = 43,
    INT_LOCAL_28 = 44,
    INT_LOCAL_29 = 45,
    INT_LOCAL_30 = 46,
    INT_LOCAL_31 = 47,
    INT_LOCAL_32 = 48,
    INT_LOCAL_33 = 49,
    INT_LOCAL_34 = 50,
    INT_LOCAL_35 = 51,
    INT_LOCAL_36 = 52,
    INT_LOCAL_37 = 53,
    INT_LOCAL_38 = 54,
    INT_LOCAL_39 = 55,
    INT_LOCAL_40 = 56,
    INT_LOCAL_41 = 57,
    INT_LOCAL_42 = 58,
    INT_LOCAL_43 = 59,
    INT_LOCAL_44 = 60,
    INT_LOCAL_45 = 61,
    INT_LOCAL_46 = 62,
    INT_LOCAL_47 = 63,
    NumInterruptTypes,
    // INT_NMI does not exist in the spec, it's a modeling artifact for NMI. We
    // intentionally set it to be NumInterruptTypes so it can never conflict
    // with any real INT_NUM in used.
    INT_NMI = NumInterruptTypes,
};



// H-extension useful masks
const RegVal DELEGABLE_EXCPS = 0
    | (1ULL << INST_ADDR_MISALIGNED)
    | (1ULL << INST_ACCESS)
    | (1ULL << INST_ILLEGAL)
    | (1ULL << BREAKPOINT)
    | (1ULL << LOAD_ADDR_MISALIGNED)
    | (1ULL << LOAD_ACCESS)
    | (1ULL << STORE_ADDR_MISALIGNED)
    | (1ULL << STORE_ACCESS)
    | (1ULL << ECALL_USER)
    | (1ULL << ECALL_SUPER)
    | (1ULL << ECALL_MACHINE)
    | (1ULL << INST_PAGE)
    | (1ULL << LOAD_PAGE)
    | (1ULL << STORE_PAGE)
    ;



const RegVal DELEGABLE_EXCPS_WITH_RVH = DELEGABLE_EXCPS
    | (1ULL << ECALL_VIRTUAL_SUPER)
    | (1ULL << INST_GUEST_PAGE)
    | (1ULL << LOAD_GUEST_PAGE)
    | (1ULL << VIRTUAL_INST)
    | (1ULL << STORE_GUEST_PAGE)
    ;

const RegVal VS_DELEGABLE_EXCPS = DELEGABLE_EXCPS &
    ~((1ULL << ECALL_SUPER) |
    (1ULL << ECALL_VIRTUAL_SUPER) |
    (1ULL << ECALL_MACHINE) |
    (1ULL << INST_GUEST_PAGE) |
    (1ULL << LOAD_GUEST_PAGE) |
    (1ULL << VIRTUAL_INST) |
    (1ULL << STORE_GUEST_PAGE));


} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_FAULT_CODES_HH__
