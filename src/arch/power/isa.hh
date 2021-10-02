/*
 * Copyright (c) 2009 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
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

#ifndef __ARCH_POWER_ISA_HH__
#define __ARCH_POWER_ISA_HH__

#include "arch/generic/isa.hh"
#include "arch/power/pcstate.hh"
#include "arch/power/regs/misc.hh"
#include "arch/power/types.hh"
#include "base/logging.hh"
#include "cpu/reg_class.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct PowerISAParams;
class ThreadContext;
class Checkpoint;
class EventManager;

namespace PowerISA
{

class ISA : public BaseISA
{
  protected:
    RegVal miscRegs[NUM_MISCREGS];

  public:
    PCStateBase *
    newPCState(Addr new_inst_addr=0) const override
    {
        return new PCState(new_inst_addr);
    }

    RegVal
    readMiscRegNoEffect(RegIndex idx) const override
    {
        fatal("Power does not currently have any misc regs defined\n");
    }

    RegVal
    readMiscReg(RegIndex idx) override
    {
        fatal("Power does not currently have any misc regs defined\n");
    }

    void
    setMiscRegNoEffect(RegIndex idx, RegVal val) override
    {
        fatal("Power does not currently have any misc regs defined\n");
    }

    void
    setMiscReg(RegIndex idx, RegVal val) override
    {
        fatal("Power does not currently have any misc regs defined\n");
    }

    bool
    inUserMode() const override
    {
        return false;
    }

    void copyRegsFrom(ThreadContext *src) override;

    using Params = PowerISAParams;

    ISA(const Params &p);
};

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_ISA_HH__
