#include "arch/riscv/insts/static_inst.hh"

#include "arch/riscv/types.hh"
#include "cpu/static_inst.hh"

namespace RiscvISA
{

void
RiscvMicroInst::advancePC(PCState &pcState) const
{
    if (flags[IsLastMicroop]) {
        pcState.uEnd();
    } else {
        pcState.uAdvance();
    }
}

} // namespace RiscvISA