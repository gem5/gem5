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
#include "mem/request.hh"
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
    initiateMemRead(Addr addr, unsigned int size,
                    Request::Flags flags) override
    {
        execute.getLSQ().pushRequest(inst, true /* load */, nullptr,
            size, addr, flags, NULL);
        return NoFault;
    }

    Fault
    writeMem(uint8_t *data, unsigned int size, Addr addr,
             Request::Flags flags, uint64_t *res) override
    {
        execute.getLSQ().pushRequest(inst, false /* store */, data,
            size, addr, flags, res);
        return NoFault;
    }

    IntReg
    readIntRegOperand(const StaticInst *si, int idx) override
    {
        RegId reg = si->srcRegIdx(idx);
        assert(reg.regClass == IntRegClass);
        return thread.readIntReg(reg.regIdx);
    }

    TheISA::FloatReg
    readFloatRegOperand(const StaticInst *si, int idx) override
    {
        RegId reg = si->srcRegIdx(idx);
        assert(reg.regClass == FloatRegClass);
        return thread.readFloatReg(reg.regIdx);
    }

    TheISA::FloatRegBits
    readFloatRegOperandBits(const StaticInst *si, int idx) override
    {
        RegId reg = si->srcRegIdx(idx);
        assert(reg.regClass == FloatRegClass);
        return thread.readFloatRegBits(reg.regIdx);
    }

    void
    setIntRegOperand(const StaticInst *si, int idx, IntReg val) override
    {
        RegId reg = si->destRegIdx(idx);
        assert(reg.regClass == IntRegClass);
        thread.setIntReg(reg.regIdx, val);
    }

    void
    setFloatRegOperand(const StaticInst *si, int idx,
        TheISA::FloatReg val) override
    {
        RegId reg = si->destRegIdx(idx);
        assert(reg.regClass == FloatRegClass);
        thread.setFloatReg(reg.regIdx, val);
    }

    void
    setFloatRegOperandBits(const StaticInst *si, int idx,
        TheISA::FloatRegBits val) override
    {
        RegId reg = si->destRegIdx(idx);
        assert(reg.regClass == FloatRegClass);
        thread.setFloatRegBits(reg.regIdx, val);
    }

    bool
    readPredicate() override
    {
        return thread.readPredicate();
    }

    void
    setPredicate(bool val) override
    {
        thread.setPredicate(val);
    }

    TheISA::PCState
    pcState() const override
    {
        return thread.pcState();
    }

    void
    pcState(const TheISA::PCState &val) override
    {
        thread.pcState(val);
    }

    TheISA::MiscReg
    readMiscRegNoEffect(int misc_reg) const
    {
        return thread.readMiscRegNoEffect(misc_reg);
    }

    TheISA::MiscReg
    readMiscReg(int misc_reg) override
    {
        return thread.readMiscReg(misc_reg);
    }

    void
    setMiscReg(int misc_reg, const TheISA::MiscReg &val) override
    {
        thread.setMiscReg(misc_reg, val);
    }

    TheISA::MiscReg
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        RegId reg = si->srcRegIdx(idx);
        assert(reg.regClass == MiscRegClass);
        return thread.readMiscReg(reg.regIdx);
    }

    void
    setMiscRegOperand(const StaticInst *si, int idx,
        const TheISA::MiscReg &val) override
    {
        RegId reg = si->destRegIdx(idx);
        assert(reg.regClass == MiscRegClass);
        return thread.setMiscReg(reg.regIdx, val);
    }

    Fault
    hwrei() override
    {
#if THE_ISA == ALPHA_ISA
        return thread.hwrei();
#else
        return NoFault;
#endif
    }

    bool
    simPalCheck(int palFunc) override
    {
#if THE_ISA == ALPHA_ISA
        return thread.simPalCheck(palFunc);
#else
        return false;
#endif
    }

    void
    syscall(int64_t callnum, Fault *fault) override
     {
        if (FullSystem)
            panic("Syscall emulation isn't available in FS mode.\n");

        thread.syscall(callnum, fault);
    }

    ThreadContext *tcBase() override { return thread.getTC(); }

    /* @todo, should make stCondFailures persistent somewhere */
    unsigned int readStCondFailures() const override { return 0; }
    void setStCondFailures(unsigned int st_cond_failures) override {}

    ContextID contextId() { return thread.contextId(); }
    /* ISA-specific (or at least currently ISA singleton) functions */

    /* X86: TLB twiddling */
    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        thread.getITBPtr()->demapPage(vaddr, asn);
        thread.getDTBPtr()->demapPage(vaddr, asn);
    }

    TheISA::CCReg
    readCCRegOperand(const StaticInst *si, int idx) override
    {
        RegId reg = si->srcRegIdx(idx);
        assert(reg.regClass == CCRegClass);
        return thread.readCCReg(reg.regIdx);
    }

    void
    setCCRegOperand(const StaticInst *si, int idx, TheISA::CCReg val) override
    {
        RegId reg = si->destRegIdx(idx);
        assert(reg.regClass == CCRegClass);
        thread.setCCReg(reg.regIdx, val);
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
    void setEA(Addr ea) override
    {
        inst->ea = ea;
    }

    BaseCPU *getCpuPtr() { return &cpu; }

    /* POWER: Effective address storage */
    Addr getEA() const override
    {
        return inst->ea;
    }

    /* MIPS: other thread register reading/writing */
    uint64_t
    readRegOtherThread(RegId reg, ThreadID tid = InvalidThreadID)
    {
        SimpleThread *other_thread = (tid == InvalidThreadID
            ? &thread : cpu.threads[tid]);

        switch(reg.regClass) {
            case IntRegClass:
                return other_thread->readIntReg(reg.regIdx);
                break;
            case FloatRegClass:
                return other_thread->readFloatRegBits(reg.regIdx);
                break;
            case MiscRegClass:
                return other_thread->readMiscReg(reg.regIdx);
            default:
                panic("Unexpected reg class! (%s)",
                      RegClassStrings[reg.regClass]);
                return 0;
        }
    }

    void
    setRegOtherThread(RegId reg, const TheISA::MiscReg &val,
        ThreadID tid = InvalidThreadID)
    {
        SimpleThread *other_thread = (tid == InvalidThreadID
            ? &thread : cpu.threads[tid]);

         switch(reg.regClass) {
            case IntRegClass:
                return other_thread->setIntReg(reg.regIdx, val);
                break;
            case FloatRegClass:
                return other_thread->setFloatRegBits(reg.regIdx, val);
                break;
            case MiscRegClass:
                return other_thread->setMiscReg(reg.regIdx, val);
            default:
                panic("Unexpected reg class! (%s)",
                      RegClassStrings[reg.regClass]);
        }
    }

  public:
    // monitor/mwait funtions
    void armMonitor(Addr address) override
    { getCpuPtr()->armMonitor(inst->id.threadId, address); }

    bool mwait(PacketPtr pkt) override
    { return getCpuPtr()->mwait(inst->id.threadId, pkt); }

    void mwaitAtomic(ThreadContext *tc) override
    { return getCpuPtr()->mwaitAtomic(inst->id.threadId, tc, thread.dtb); }

    AddressMonitor *getAddrMonitor() override
    { return getCpuPtr()->getCpuAddrMonitor(inst->id.threadId); }
};

}

#endif /* __CPU_MINOR_EXEC_CONTEXT_HH__ */
