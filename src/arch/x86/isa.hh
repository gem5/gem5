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

#ifndef __ARCH_X86_ISA_HH__
#define __ARCH_X86_ISA_HH__

#include <iostream>
#include <string>

#include "arch/generic/isa.hh"
#include "arch/x86/cpuid.hh"
#include "arch/x86/pcstate.hh"
#include "arch/x86/regs/ccr.hh"
#include "arch/x86/regs/float.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

class ThreadContext;
struct X86ISAParams;

namespace X86ISA
{

class ISA : public BaseISA
{
  private:
    RegVal regVal[misc_reg::NumRegs];
    void updateHandyM5Reg(Efer efer, CR0 cr0,
            SegAttr csAttr, SegAttr ssAttr, RFLAGS rflags);

    std::string vendorString;

  public:
    void clear() override;

    PCStateBase *
    newPCState(Addr new_inst_addr=0) const override
    {
        return new PCState(new_inst_addr);
    }

    using Params = X86ISAParams;

    ISA(const Params &p);

    RegVal readMiscRegNoEffect(RegIndex idx) const override;
    RegVal readMiscReg(RegIndex idx) override;

    void setMiscRegNoEffect(RegIndex idx, RegVal val) override;
    void setMiscReg(RegIndex idx, RegVal val) override;

    bool
    inUserMode() const override
    {
        HandyM5Reg m5reg = readMiscRegNoEffect(misc_reg::M5Reg);
        return m5reg.cpl == 3;
    }

    void copyRegsFrom(ThreadContext *src) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void setThreadContext(ThreadContext *_tc) override;

    std::string getVendorString() const;

    std::unique_ptr<X86CPUID> cpuid;
};

} // namespace X86ISA
} // namespace gem5

#endif
