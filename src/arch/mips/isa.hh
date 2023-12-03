/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#ifndef __ARCH_MIPS_ISA_HH__
#define __ARCH_MIPS_ISA_HH__

#include <queue>
#include <string>
#include <vector>

#include "arch/generic/isa.hh"
#include "arch/mips/pcstate.hh"
#include "arch/mips/regs/misc.hh"
#include "arch/mips/types.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class BaseCPU;
class Checkpoint;
struct MipsISAParams;
class ThreadContext;

namespace MipsISA
{
class ISA : public BaseISA
{
  public:
    // The MIPS name for this file is CP0 or Coprocessor 0
    typedef ISA CP0;

    using Params = MipsISAParams;

  protected:
    // Number of threads and vpes an individual ISA state can handle
    uint8_t numThreads;
    uint8_t numVpes;

    enum BankType
    {
        perProcessor,
        perThreadContext,
        perVirtProcessor
    };

    std::vector<std::vector<RegVal> > miscRegFile;
    std::vector<std::vector<RegVal> > miscRegFile_WriteMask;
    std::vector<BankType> bankType;

  public:
    void clear() override;

    PCStateBase *
    newPCState(Addr new_inst_addr = 0) const override
    {
        return new PCState(new_inst_addr);
    }

  public:
    void configCP();

    unsigned getVPENum(ThreadID tid) const;

    //////////////////////////////////////////////////////////
    //
    // READ/WRITE CP0 STATE
    //
    //
    //////////////////////////////////////////////////////////
    //@TODO: MIPS MT's register view automatically connects
    //       Status to TCStatus depending on current thread
    void
    updateCP0ReadView(RegIndex idx, ThreadID tid)
    {}

    RegVal readMiscRegNoEffect(RegIndex idx, ThreadID tid) const;

    RegVal
    readMiscRegNoEffect(RegIndex idx) const override
    {
        return readMiscRegNoEffect(idx, 0);
    }

    RegVal readMiscReg(RegIndex idx, ThreadID tid);

    RegVal
    readMiscReg(RegIndex idx) override
    {
        return readMiscReg(idx, 0);
    }

    RegVal filterCP0Write(RegIndex idx, int reg_sel, RegVal val);
    void setRegMask(RegIndex idx, RegVal val, ThreadID tid = 0);

    void setMiscRegNoEffect(RegIndex idx, RegVal val, ThreadID tid);

    void
    setMiscRegNoEffect(RegIndex idx, RegVal val) override
    {
        setMiscRegNoEffect(idx, val, 0);
    }

    void setMiscReg(RegIndex idx, RegVal val, ThreadID tid);

    void
    setMiscReg(RegIndex idx, RegVal val) override
    {
        setMiscReg(idx, val, 0);
    }

    //////////////////////////////////////////////////////////
    //
    // DECLARE INTERFACE THAT WILL ALLOW A MiscRegFile (Cop0)
    // TO SCHEDULE EVENTS
    //
    //////////////////////////////////////////////////////////

    // Flag that is set when CP0 state has been written to.
    bool cp0Updated;

    // Enumerated List of CP0 Event Types
    enum CP0EventType
    {
        UpdateCP0
    };

    /** Process a CP0 event */
    void processCP0Event(BaseCPU *cpu, CP0EventType);

    // Schedule a CP0 Update Event
    void scheduleCP0Update(BaseCPU *cpu, Cycles delay = Cycles(0));

    // If any changes have been made, then check the state for changes
    // and if necessary alert the CPU
    void updateCPU(BaseCPU *cpu);

    static std::string miscRegNames[misc_reg::NumRegs];

  public:
    ISA(const Params &p);

    bool
    inUserMode() const override
    {
        RegVal Stat = readMiscRegNoEffect(misc_reg::Status);
        RegVal Dbg = readMiscRegNoEffect(misc_reg::Debug);

        if ( // EXL, ERL or CU0 set, CP0 accessible
            (Stat & 0x10000006) == 0 &&
            // DM bit set, CP0 accessible
            (Dbg & 0x40000000) == 0 &&
            // KSU = 0, kernel mode is base mode
            (Stat & 0x00000018) != 0) {
            // Unable to use Status_CU0, etc directly,
            // using bitfields & masks.
            return true;
        } else {
            return false;
        }
    }

    void copyRegsFrom(ThreadContext *src) override;
};
} // namespace MipsISA
} // namespace gem5

#endif
