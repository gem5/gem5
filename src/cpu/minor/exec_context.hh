/*
 * Copyright (c) 2011-2014 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Steve Reinhardt
 *          Dave Greene
 *          Nathan Binkert
 *          Andrew Bardsley
 */

/**
 * @file
 *
 *  ExecContext bears the exec_context interface for Minor.
 */

#ifndef __CPU_MINOR_EXEC_CONTEXT_HH__
#define __CPU_MINOR_EXEC_CONTEXT_HH__

#include "cpu/exec_context.hh"
#include "cpu/minor/execute.hh"
#include "cpu/minor/pipeline.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "debug/MinorExecute.hh"

namespace Minor
{

/* Forward declaration of Execute */
class Execute;

/** ExecContext bears the exec_context interface for Minor.  This nicely
 *  separates that interface from other classes such as Pipeline, MinorCPU
 *  and DynMinorInst and makes it easier to see what state is accessed by it.
 */
class ExecContext : public ::ExecContext
{
  public:
    MinorCPU &cpu;

    /** ThreadState object, provides all the architectural state. */
    SimpleThread &thread;

    /** The execute stage so we can peek at its contents. */
    Execute &execute;

    /** Instruction for the benefit of memory operations and for PC */
    MinorDynInstPtr inst;

    ExecContext (
        MinorCPU &cpu_,
        SimpleThread &thread_, Execute &execute_,
        MinorDynInstPtr inst_) :
        cpu(cpu_),
        thread(thread_),
        execute(execute_),
        inst(inst_)
    {
        DPRINTF(MinorExecute, "ExecContext setting PC: %s\n", inst->pc);
        pcState(inst->pc);
        setPredicate(true);
        thread.setIntReg(TheISA::ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
        thread.setFloatReg(TheISA::ZeroReg, 0.0);
#endif
    }

    Fault
    readMem(Addr addr, uint8_t *data, unsigned int size,
        unsigned int flags)
    {
        execute.getLSQ().pushRequest(inst, true /* load */, data,
            size, addr, flags, NULL);
        return NoFault;
    }

    Fault
    writeMem(uint8_t *data, unsigned int size, Addr addr,
        unsigned int flags, uint64_t *res)
    {
        execute.getLSQ().pushRequest(inst, false /* store */, data,
            size, addr, flags, res);
        return NoFault;
    }

    IntReg
    readIntRegOperand(const StaticInst *si, int idx)
    {
        return thread.readIntReg(si->srcRegIdx(idx));
    }

    TheISA::FloatReg
    readFloatRegOperand(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread.readFloatReg(reg_idx);
    }

    TheISA::FloatRegBits
    readFloatRegOperandBits(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread.readFloatRegBits(reg_idx);
    }

    void
    setIntRegOperand(const StaticInst *si, int idx, IntReg val)
    {
        thread.setIntReg(si->destRegIdx(idx), val);
    }

    void
    setFloatRegOperand(const StaticInst *si, int idx,
        TheISA::FloatReg val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread.setFloatReg(reg_idx, val);
    }

    void
    setFloatRegOperandBits(const StaticInst *si, int idx,
        TheISA::FloatRegBits val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread.setFloatRegBits(reg_idx, val);
    }

    bool
    readPredicate()
    {
        return thread.readPredicate();
    }

    void
    setPredicate(bool val)
    {
        thread.setPredicate(val);
    }

    TheISA::PCState
    pcState() const
    {
        return thread.pcState();
    }

    void
    pcState(const TheISA::PCState &val)
    {
        thread.pcState(val);
    }

    TheISA::MiscReg
    readMiscRegNoEffect(int misc_reg) const
    {
        return thread.readMiscRegNoEffect(misc_reg);
    }

    TheISA::MiscReg
    readMiscReg(int misc_reg)
    {
        return thread.readMiscReg(misc_reg);
    }

    void
    setMiscReg(int misc_reg, const TheISA::MiscReg &val)
    {
        thread.setMiscReg(misc_reg, val);
    }

    TheISA::MiscReg
    readMiscRegOperand(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::Misc_Reg_Base;
        return thread.readMiscReg(reg_idx);
    }

    void
    setMiscRegOperand(const StaticInst *si, int idx,
        const TheISA::MiscReg &val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::Misc_Reg_Base;
        return thread.setMiscReg(reg_idx, val);
    }

    Fault
    hwrei()
    {
#if THE_ISA == ALPHA_ISA
        return thread.hwrei();
#else
        return NoFault;
#endif
    }

    bool
    simPalCheck(int palFunc)
    {
#if THE_ISA == ALPHA_ISA
        return thread.simPalCheck(palFunc);
#else
        return false;
#endif
    }

    void
    syscall(int64_t callnum)
    {
        if (FullSystem)
            panic("Syscall emulation isn't available in FS mode.\n");

        thread.syscall(callnum);
    }

    ThreadContext *tcBase() { return thread.getTC(); }

    /* @todo, should make stCondFailures persistent somewhere */
    unsigned int readStCondFailures() const { return 0; }
    void setStCondFailures(unsigned int st_cond_failures) {}

    ContextID contextId() { return thread.contextId(); }
    /* ISA-specific (or at least currently ISA singleton) functions */

    /* X86: TLB twiddling */
    void
    demapPage(Addr vaddr, uint64_t asn)
    {
        thread.getITBPtr()->demapPage(vaddr, asn);
        thread.getDTBPtr()->demapPage(vaddr, asn);
    }

    TheISA::CCReg
    readCCRegOperand(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::CC_Reg_Base;
        return thread.readCCReg(reg_idx);
    }

    void
    setCCRegOperand(const StaticInst *si, int idx, TheISA::CCReg val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::CC_Reg_Base;
        thread.setCCReg(reg_idx, val);
    }

    void
    demapInstPage(Addr vaddr, uint64_t asn)
    {
        thread.getITBPtr()->demapPage(vaddr, asn);
    }

    void
    demapDataPage(Addr vaddr, uint64_t asn)
    {
        thread.getDTBPtr()->demapPage(vaddr, asn);
    }

    /* ALPHA/POWER: Effective address storage */
    void setEA(Addr ea)
    {
        inst->ea = ea;
    }

    BaseCPU *getCpuPtr() { return &cpu; }

    /* POWER: Effective address storage */
    Addr getEA() const
    {
        return inst->ea;
    }

    /* MIPS: other thread register reading/writing */
    uint64_t
    readRegOtherThread(int idx, ThreadID tid = InvalidThreadID)
    {
        SimpleThread *other_thread = (tid == InvalidThreadID
            ? &thread : cpu.threads[tid]);

        if (idx < TheISA::FP_Reg_Base) { /* Integer */
            return other_thread->readIntReg(idx);
        } else if (idx < TheISA::Misc_Reg_Base) { /* Float */
            return other_thread->readFloatRegBits(idx
                - TheISA::FP_Reg_Base);
        } else { /* Misc */
            return other_thread->readMiscReg(idx
                - TheISA::Misc_Reg_Base);
        }
    }

    void
    setRegOtherThread(int idx, const TheISA::MiscReg &val,
        ThreadID tid = InvalidThreadID)
    {
        SimpleThread *other_thread = (tid == InvalidThreadID
            ? &thread : cpu.threads[tid]);

        if (idx < TheISA::FP_Reg_Base) { /* Integer */
            return other_thread->setIntReg(idx, val);
        } else if (idx < TheISA::Misc_Reg_Base) { /* Float */
            return other_thread->setFloatRegBits(idx
                - TheISA::FP_Reg_Base, val);
        } else { /* Misc */
            return other_thread->setMiscReg(idx
                - TheISA::Misc_Reg_Base, val);
        }
    }

  public:
    // monitor/mwait funtions
    void armMonitor(Addr address) { getCpuPtr()->armMonitor(0, address); }
    bool mwait(PacketPtr pkt) { return getCpuPtr()->mwait(0, pkt); }
    void mwaitAtomic(ThreadContext *tc)
    { return getCpuPtr()->mwaitAtomic(0, tc, thread.dtb); }
    AddressMonitor *getAddrMonitor()
    { return getCpuPtr()->getCpuAddrMonitor(0); }
};

}

#endif /* __CPU_MINOR_EXEC_CONTEXT_HH__ */
