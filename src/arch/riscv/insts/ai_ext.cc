#include "arch/riscv/insts/ai_ext.hh"

#include <cstdint>
#include <sstream>
#include <string>

#include "arch/riscv/regs/int.hh"
#include "arch/riscv/utility.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "sim/faults.hh"

namespace gem5
{
namespace RiscvISA
{

MAC_R::MAC_R(ExtMachInst machInst)
    : RiscvStaticInst("mac", machInst, IntMultOp),
      rd(intRegClass[machInst.rd]),
      rs1(intRegClass[machInst.rs1]),
      rs2(intRegClass[machInst.rs2])
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;
    // Sources: rs1, rs2, and old value of rd (MAC reads & writes rd).
    setSrcRegIdx(_numSrcRegs++, rs1);
    setSrcRegIdx(_numSrcRegs++, rs2);
    setSrcRegIdx(_numSrcRegs++, rd);
    setDestRegIdx(_numDestRegs++, rd);

    flags[IsInteger] = true;
}

std::string
MAC_R::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << " "
       << registerName(rd) << ", "
       << registerName(rs1) << ", "
       << registerName(rs2);
    return ss.str();
}

Fault
MAC_R::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    const RegVal val_s1 = xc->getRegOperand(this, 0);
    const RegVal val_s2 = xc->getRegOperand(this, 1);
    const RegVal val_d_old = xc->getRegOperand(this, 2);

    RegVal result;
    if (machInst.rv_type == RV32) {
        int64_t s1 = (int32_t)val_s1;
        int64_t s2 = (int32_t)val_s2;
        int64_t acc = (int32_t)val_d_old;
        int64_t sum = acc + (s1 * s2);
        uint32_t res32 = (uint32_t)sum;
        result = (RegVal)rvSext((int64_t)(int32_t)res32);
    } else {
        __int128 s1 = (int64_t)val_s1;
        __int128 s2 = (int64_t)val_s2;
        __int128 acc = (int64_t)val_d_old;
        __int128 sum = acc + (s1 * s2);
        result = (RegVal)(uint64_t)sum;
    }

    xc->setRegOperand(this, 0, result);
    return NoFault;
}

DOT4_ACC_R::DOT4_ACC_R(ExtMachInst machInst)
    : RiscvStaticInst("dot4_acc", machInst, IntMultOp),
      rd(intRegClass[machInst.rd]),
      rs1(intRegClass[machInst.rs1]),
      rs2(intRegClass[machInst.rs2])
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;
    // rs1, rs2, and old rd (accumulator).
    setSrcRegIdx(_numSrcRegs++, rs1);
    setSrcRegIdx(_numSrcRegs++, rs2);
    setSrcRegIdx(_numSrcRegs++, rd);
    setDestRegIdx(_numDestRegs++, rd);

    flags[IsInteger] = true;
}

std::string
DOT4_ACC_R::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << " "
       << registerName(rd) << ", "
       << registerName(rs1) << ", "
       << registerName(rs2);
    return ss.str();
}

Fault
DOT4_ACC_R::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    const RegVal v1 = xc->getRegOperand(this, 0);
    const RegVal v2 = xc->getRegOperand(this, 1);
    const RegVal acc_old = xc->getRegOperand(this, 2);

    // Use the low 32 bits for the packed 4x int8 lanes.
    int32_t dot = 0;
    for (int i = 0; i < 4; ++i) {
        int8_t a = (int8_t)((uint32_t)v1 >> (8 * i));
        int8_t b = (int8_t)((uint32_t)v2 >> (8 * i));
        dot += (int32_t)a * (int32_t)b;
    }

    RegVal result;
    if (machInst.rv_type == RV32) {
        // Wrap to 32-bit then sign-extend to XLEN.
        uint32_t res32 = (uint32_t)acc_old + (uint32_t)dot;
        result = (RegVal)rvSext((int64_t)(int32_t)res32);
    } else {
        // XLEN=64. Operate on the full register width.
        result = acc_old + (int64_t)dot;
    }

    xc->setRegOperand(this, 0, result);
    return NoFault;
}

RELU_R::RELU_R(ExtMachInst machInst)
    : RiscvStaticInst("relu", machInst, IntAluOp),
      rd(intRegClass[machInst.rd]),
      rs1(intRegClass[machInst.rs1])
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;
    setSrcRegIdx(_numSrcRegs++, rs1);
    setDestRegIdx(_numDestRegs++, rd);

    flags[IsInteger] = true;
}

std::string
RELU_R::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << " "
       << registerName(rd) << ", "
       << registerName(rs1);
    return ss.str();
}

Fault
RELU_R::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    const RegVal v = xc->getRegOperand(this, 0);
    RegVal out;
    if (machInst.rv_type == RV32) {
        int32_t s = (int32_t)v;
        out = (s < 0) ? 0 : (RegVal)rvSext((int64_t)s);
    } else {
        int64_t s = (int64_t)v;
        out = (s < 0) ? 0 : (RegVal)s;
    }

    xc->setRegOperand(this, 0, out);
    return NoFault;
}

CLAMP_I::CLAMP_I(ExtMachInst machInst)
    : RiscvStaticInst("clamp", machInst, IntAluOp),
      rd(intRegClass[machInst.rd]),
      rs1(intRegClass[machInst.rs1]),
      uimm12((uint16_t)machInst.imm12)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;
    setSrcRegIdx(_numSrcRegs++, rs1);
    setDestRegIdx(_numDestRegs++, rd);

    flags[IsInteger] = true;
}

std::string
CLAMP_I::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << " "
       << registerName(rd) << ", "
       << registerName(rs1) << ", "
       << (uint32_t)uimm12;
    return ss.str();
}

Fault
CLAMP_I::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    const RegVal v = xc->getRegOperand(this, 0);
    const RegVal ub = (RegVal)uimm12; // unsigned upper bound, >= 0

    RegVal out;
    if (machInst.rv_type == RV32) {
        int32_t s = (int32_t)v;
        if (s < 0) {
            out = 0;
        } else if ((uint32_t)s > (uint32_t)ub) {
            out = (RegVal)rvSext((int64_t)(int32_t)(uint32_t)ub);
        } else {
            out = (RegVal)rvSext((int64_t)s);
        }
    } else {
        int64_t s = (int64_t)v;
        if (s < 0) {
            out = 0;
        } else if ((uint64_t)s > (uint64_t)ub) {
            out = ub;
        } else {
            out = (RegVal)s;
        }
    }

    xc->setRegOperand(this, 0, out);
    return NoFault;
}

} // namespace RiscvISA
} // namespace gem5
