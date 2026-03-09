#include "arch/riscv/insts/mac.hh"

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
    // Destination: rd.
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
    // Read sources via ExecContext
    RegVal val_s1 = xc->getRegOperand(this, 0);
    RegVal val_s2 = xc->getRegOperand(this, 1);
    RegVal val_d_old = xc->getRegOperand(this, 2);

    // Perform MAC operation: rd = rd + (rs1 * rs2)
    RegVal result;
    if (machInst.rv_type == RV32) {
        // RV32
        int64_t s1 = (int32_t)val_s1;
        int64_t s2 = (int32_t)val_s2;
        int64_t acc = (int32_t)val_d_old;
        int64_t sum = acc + (s1 * s2); // fits in int64_t
        uint32_t res32 = (uint32_t)sum; // modulo 2^32
        result = (RegVal)rvSext((int64_t)(int32_t)res32);
    } else {
        // RV64
        __int128 s1 = (int64_t)val_s1;
        __int128 s2 = (int64_t)val_s2;
        __int128 acc = (int64_t)val_d_old;
        __int128 sum = acc + (s1 * s2);
        result = (RegVal)(uint64_t)sum; // modulo 2^64
    }

    // Write result to destination register
    xc->setRegOperand(this, 0, result);

    return NoFault;
}

} // namespace RiscvISA
} // namespace gem5
