#ifndef __ARCH_RISCV_AI_EXT_INST_HH__
#define __ARCH_RISCV_AI_EXT_INST_HH__

#include <type_traits>

#include "arch/riscv/insts/static_inst.hh"
#include "base/types.hh"

namespace gem5
{
namespace RiscvISA
{

/**
 * mac (Multiply-Accumulate)
 * Semantics: rd = rd + (rs1 * rs2)
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

/**
 * dot4_acc (Packed Dot Product Accumulate)
 * Semantics: rd = rd + sum_{i=0..3}( (int8)rs1.byte[i] * (int8)rs2.byte[i] )
 * Packing: a0/b0 are bits [7:0] (LSB byte), then [15:8], [23:16], [31:24].
 */
class DOT4_ACC_R : public RiscvStaticInst
{
  private:
    RegId srcRegIdxArr[3];
    RegId destRegIdxArr[1];

    const RegId rd;
    const RegId rs1;
    const RegId rs2;

  public:
    DOT4_ACC_R(ExtMachInst machInst);

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    Fault execute(ExecContext *xc,
        trace::InstRecord *traceData) const override;
};

/**
 * relu
 * Semantics: rd = max(rs1, 0) (signed compare)
 */
class RELU_R : public RiscvStaticInst
{
  private:
    RegId srcRegIdxArr[1];
    RegId destRegIdxArr[1];

    const RegId rd;
    const RegId rs1;

  public:
    RELU_R(ExtMachInst machInst);

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    Fault execute(ExecContext *xc,
        trace::InstRecord *traceData) const override;
};

/**
 * clamp
 * Semantics:
 *   rd = (rs1 < 0) ? 0 : (rs1 > rs2) ? rs2 : rs1
 * rs2 is treated as an unsigned upper bound.
 */
class CLAMP_R : public RiscvStaticInst
{
  private:
    RegId srcRegIdxArr[2];
    RegId destRegIdxArr[1];

    const RegId rd;
    const RegId rs1;
    const RegId rs2;

  public:
    CLAMP_R(ExtMachInst machInst);

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    Fault execute(ExecContext *xc,
        trace::InstRecord *traceData) const override;
};

/**
 * lp.setup (I-type)
 * Semantics:
 *   lpcount = rs1
 *   lpstart = next PC
 *   lpend   = PC + (raw_imm << 2) - 4
 *   lpactive = (lpcount > 0)
 *   lpgen   = lpgen + 1   (monotonic loop-instance id for FE / debug)
 */
class LP_SETUP_I : public RiscvStaticInst
{
  private:
    RegId srcRegIdxArr[1];

    const RegId rs1;
    const uint16_t uimm12;

  public:
    LP_SETUP_I(ExtMachInst machInst);

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    Fault execute(ExecContext *xc,
        trace::InstRecord *traceData) const override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_AI_EXT_INST_HH__
