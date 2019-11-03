#ifndef __ARCH_RISCV_BITFIELDS_HH__
#define __ARCH_RISCV_BITFIELDS_HH__

#include "base/bitfield.hh"

#define CSRIMM  bits(machInst, 19, 15)
#define FUNCT12 bits(machInst, 31, 20)
#define IMM5    bits(machInst, 11, 7)
#define IMM7    bits(machInst, 31, 25)
#define IMMSIGN bits(machInst, 31)
#define OPCODE  bits(machInst, 6, 0)

#endif // __ARCH_RISCV_BITFIELDS_HH__