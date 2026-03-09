#ifndef __ARCH_RISCV_MAC_INST_HH__
#define __ARCH_RISCV_MAC_INST_HH__

#include <type_traits>

#include "arch/riscv/insts/static_inst.hh"
#include "base/types.hh"

namespace gem5
{
namespace RiscvISA
{

/**
 * Static instruction class for MAC (Multiply-Accumulate) instruction.
 * Performs: rd = rd + (rs1 * rs2)
 */
class MAC_R : public RiscvStaticInst
{
  private:
    RegId srcRegIdxArr[3];
    RegId destRegIdxArr[1];

    const RegId rd;
    const RegId rs1;
    const RegId rs2;

  public:
    MAC_R(ExtMachInst machInst);

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    Fault execute(ExecContext *xc,
        trace::InstRecord *traceData) const override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_MAC_INST_HH__
