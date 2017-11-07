#ifndef __ARCH_RISCV_BITFIELDS_HH__
#define __ARCH_RISCV_BITFIELDS_HH__

#include "base/bitfield.hh"

#define CSRIMM  bits(machInst, 19, 15)
#define FUNCT12 bits(machInst, 31, 20)

#endif // __ARCH_RISCV_BITFIELDS_HH__