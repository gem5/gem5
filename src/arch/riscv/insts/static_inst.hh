/*
 * Copyright (c) 2015 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_RISCV_STATIC_INST_HH__
#define __ARCH_RISCV_STATIC_INST_HH__

#include <string>

#include "arch/riscv/pcstate.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/types.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "mem/packet.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * Base class for all RISC-V static instructions.
 */
class RiscvStaticInst : public StaticInst
{
  protected:
    RiscvStaticInst(const char *_mnemonic, ExtMachInst _machInst,
                    OpClass __opClass)
        : StaticInst(_mnemonic, __opClass), machInst(_machInst)
    {}

    template <typename T>
    T
    rvSelect(T v32, T v64) const
    {
        return (machInst.rv_type == RV32) ? v32 : v64;
    }

    template <typename T32, typename T64>
    T64
    rvExt(T64 x) const
    {
        return rvSelect((T64)(T32)x, x);
    }

    uint64_t
    rvZext(uint64_t x) const
    {
        return rvExt<uint32_t, uint64_t>(x);
    }

    int64_t
    rvSext(int64_t x) const
    {
        return rvExt<int32_t, int64_t>(x);
    }

  public:
    ExtMachInst machInst;

    void
    advancePC(PCStateBase &pc) const override
    {
        pc.as<PCState>().advance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        pc.advance();
        tc->pcState(pc);
    }

    std::unique_ptr<PCStateBase>
    buildRetPC(const PCStateBase &cur_pc,
               const PCStateBase &call_pc) const override
    {
        PCStateBase *ret_pc_ptr = call_pc.clone();
        auto &ret_pc = ret_pc_ptr->as<PCState>();
        ret_pc.advance();
        return std::unique_ptr<PCStateBase>{ ret_pc_ptr };
    }

    size_t
    asBytes(void *buf, size_t size) override
    {
        return simpleAsBytes(buf, size, machInst);
    }
};

/**
 * Base class for all RISC-V Macroops
 */
class RiscvMacroInst : public RiscvStaticInst
{
  protected:
    std::vector<StaticInstPtr> microops;

    RiscvMacroInst(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : RiscvStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMacroop] = true;
    }

    ~RiscvMacroInst() { microops.clear(); }

    StaticInstPtr
    fetchMicroop(MicroPC upc) const override
    {
        return microops[upc];
    }

    Fault
    initiateAcc(ExecContext *xc, trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    completeAcc(PacketPtr pkt, ExecContext *xc,
                trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    execute(ExecContext *xc, trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    void
    size(size_t newSize) override
    {
        for (int i = 0; i < microops.size(); i++) {
            microops[i]->size(newSize);
        }
        _size = newSize;
    }
};

/**
 * Base class for all RISC-V Microops
 */
class RiscvMicroInst : public RiscvStaticInst
{
  protected:
    RiscvMicroInst(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : RiscvStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMicroop] = true;
    }

    void advancePC(PCStateBase &pcState) const override;
    void advancePC(ThreadContext *tc) const override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_STATIC_INST_HH__
