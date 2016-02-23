/*
 * Copyright (c) 2011 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#ifndef __CPU_CHECKER_CPU_HH__
#define __CPU_CHECKER_CPU_HH__

#include <list>
#include <map>
#include <queue>

#include "arch/types.hh"
#include "base/statistics.hh"
#include "cpu/base.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/exec_context.hh"
#include "cpu/pc_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "debug/Checker.hh"
#include "params/CheckerCPU.hh"
#include "sim/eventq.hh"

// forward declarations
namespace TheISA
{
    class TLB;
}

template <class>
class BaseDynInst;
class ThreadContext;
class Request;

/**
 * CheckerCPU class.  Dynamically verifies instructions as they are
 * completed by making sure that the instruction and its results match
 * the independent execution of the benchmark inside the checker.  The
 * checker verifies instructions in order, regardless of the order in
 * which instructions complete.  There are certain results that can
 * not be verified, specifically the result of a store conditional or
 * the values of uncached accesses.  In these cases, and with
 * instructions marked as "IsUnverifiable", the checker assumes that
 * the value from the main CPU's execution is correct and simply
 * copies that value.  It provides a CheckerThreadContext (see
 * checker/thread_context.hh) that provides hooks for updating the
 * Checker's state through any ThreadContext accesses.  This allows the
 * checker to be able to correctly verify instructions, even with
 * external accesses to the ThreadContext that change state.
 */
class CheckerCPU : public BaseCPU, public ExecContext
{
  protected:
    typedef TheISA::MachInst MachInst;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::MiscReg MiscReg;

    /** id attached to all issued requests */
    MasterID masterId;
  public:
    void init() override;

    typedef CheckerCPUParams Params;
    CheckerCPU(Params *p);
    virtual ~CheckerCPU();

    void setSystem(System *system);

    void setIcachePort(MasterPort *icache_port);

    void setDcachePort(MasterPort *dcache_port);

    MasterPort &getDataPort() override
    {
        // the checker does not have ports on its own so return the
        // data port of the actual CPU core
        assert(dcachePort);
        return *dcachePort;
    }

    MasterPort &getInstPort() override
    {
        // the checker does not have ports on its own so return the
        // data port of the actual CPU core
        assert(icachePort);
        return *icachePort;
    }

  protected:

    std::vector<Process*> workload;

    System *systemPtr;

    MasterPort *icachePort;
    MasterPort *dcachePort;

    ThreadContext *tc;

    TheISA::TLB *itb;
    TheISA::TLB *dtb;

    Addr dbg_vtophys(Addr addr);

    union Result {
        uint64_t integer;
        double dbl;
        void set(uint64_t i) { integer = i; }
        void set(double d) { dbl = d; }
        void get(uint64_t& i) { i = integer; }
        void get(double& d) { d = dbl; }
    };

    // ISAs like ARM can have multiple destination registers to check,
    // keep them all in a std::queue
    std::queue<Result> result;

    // Pointer to the one memory request.
    RequestPtr memReq;

    StaticInstPtr curStaticInst;
    StaticInstPtr curMacroStaticInst;

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;

    std::queue<int> miscRegIdxs;

  public:

    // Primary thread being run.
    SimpleThread *thread;

    TheISA::TLB* getITBPtr() { return itb; }
    TheISA::TLB* getDTBPtr() { return dtb; }

    virtual Counter totalInsts() const override
    {
        return 0;
    }

    virtual Counter totalOps() const override
    {
        return 0;
    }

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    // These functions are only used in CPU models that split
    // effective address computation from the actual memory access.
    void setEA(Addr EA) override
    { panic("CheckerCPU::setEA() not implemented\n"); }
    Addr getEA() const  override
    { panic("CheckerCPU::getEA() not implemented\n"); }

    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to redice overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

    IntReg readIntRegOperand(const StaticInst *si, int idx) override
    {
        return thread->readIntReg(si->srcRegIdx(idx));
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx) override
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread->readFloatReg(reg_idx);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si,
                                         int idx) override
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Reg_Base;
        return thread->readFloatRegBits(reg_idx);
    }

    CCReg readCCRegOperand(const StaticInst *si, int idx) override
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::CC_Reg_Base;
        return thread->readCCReg(reg_idx);
    }

    template <class T>
    void setResult(T t)
    {
        Result instRes;
        instRes.set(t);
        result.push(instRes);
    }

    void setIntRegOperand(const StaticInst *si, int idx,
                          IntReg val) override
    {
        thread->setIntReg(si->destRegIdx(idx), val);
        setResult<uint64_t>(val);
    }

    void setFloatRegOperand(const StaticInst *si, int idx,
                            FloatReg val) override
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread->setFloatReg(reg_idx, val);
        setResult<double>(val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val) override
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Reg_Base;
        thread->setFloatRegBits(reg_idx, val);
        setResult<uint64_t>(val);
    }

    void setCCRegOperand(const StaticInst *si, int idx, CCReg val) override
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::CC_Reg_Base;
        thread->setCCReg(reg_idx, val);
        setResult<uint64_t>(val);
    }

    bool readPredicate() override { return thread->readPredicate(); }
    void setPredicate(bool val) override
    {
        thread->setPredicate(val);
    }

    TheISA::PCState pcState() const override { return thread->pcState(); }
    void pcState(const TheISA::PCState &val) override
    {
        DPRINTF(Checker, "Changing PC to %s, old PC %s.\n",
                         val, thread->pcState());
        thread->pcState(val);
    }
    Addr instAddr() { return thread->instAddr(); }
    Addr nextInstAddr() { return thread->nextInstAddr(); }
    MicroPC microPC() { return thread->microPC(); }
    //////////////////////////////////////////

    MiscReg readMiscRegNoEffect(int misc_reg) const
    {
        return thread->readMiscRegNoEffect(misc_reg);
    }

    MiscReg readMiscReg(int misc_reg) override
    {
        return thread->readMiscReg(misc_reg);
    }

    void setMiscRegNoEffect(int misc_reg, const MiscReg &val)
    {
        DPRINTF(Checker, "Setting misc reg %d with no effect to check later\n", misc_reg);
        miscRegIdxs.push(misc_reg);
        return thread->setMiscRegNoEffect(misc_reg, val);
    }

    void setMiscReg(int misc_reg, const MiscReg &val) override
    {
        DPRINTF(Checker, "Setting misc reg %d with effect to check later\n", misc_reg);
        miscRegIdxs.push(misc_reg);
        return thread->setMiscReg(misc_reg, val);
    }

    MiscReg readMiscRegOperand(const StaticInst *si, int idx) override
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::Misc_Reg_Base;
        return thread->readMiscReg(reg_idx);
    }

    void setMiscRegOperand(const StaticInst *si, int idx,
                           const MiscReg &val) override
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::Misc_Reg_Base;
        return this->setMiscReg(reg_idx, val);
    }

#if THE_ISA == MIPS_ISA
    MiscReg readRegOtherThread(int misc_reg, ThreadID tid) override
    {
        panic("MIPS MT not defined for CheckerCPU.\n");
        return 0;
    }

    void setRegOtherThread(int misc_reg, MiscReg val, ThreadID tid) override
    {
        panic("MIPS MT not defined for CheckerCPU.\n");
    }
#endif

    /////////////////////////////////////////

    void recordPCChange(const TheISA::PCState &val)
    {
       changedPC = true;
       newPCState = val;
    }

    void demapPage(Addr vaddr, uint64_t asn) override
    {
        this->itb->demapPage(vaddr, asn);
        this->dtb->demapPage(vaddr, asn);
    }

    // monitor/mwait funtions
    void armMonitor(Addr address) override
    { BaseCPU::armMonitor(0, address); }
    bool mwait(PacketPtr pkt) override { return BaseCPU::mwait(0, pkt); }
    void mwaitAtomic(ThreadContext *tc) override
    { return BaseCPU::mwaitAtomic(0, tc, thread->dtb); }
    AddressMonitor *getAddrMonitor() override
    { return BaseCPU::getCpuAddrMonitor(0); }

    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        this->itb->demapPage(vaddr, asn);
    }

    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        this->dtb->demapPage(vaddr, asn);
    }

    Fault readMem(Addr addr, uint8_t *data, unsigned size,
                  unsigned flags) override;
    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res) override;

    unsigned int readStCondFailures() const override {
        return thread->readStCondFailures();
    }

    void setStCondFailures(unsigned int sc_failures) override
    {}
    /////////////////////////////////////////////////////

    Fault hwrei() override { return thread->hwrei(); }
    bool simPalCheck(int palFunc) override
    { return thread->simPalCheck(palFunc); }
    void wakeup(ThreadID tid) override { }
    // Assume that the normal CPU's call to syscall was successful.
    // The checker's state would have already been updated by the syscall.
    void syscall(int64_t callnum) override { }

    void handleError()
    {
        if (exitOnError)
            dumpAndExit();
    }

    bool checkFlags(Request *unverified_req, Addr vAddr,
                    Addr pAddr, int flags);

    void dumpAndExit();

    ThreadContext *tcBase() override { return tc; }
    SimpleThread *threadBase() { return thread; }

    Result unverifiedResult;
    Request *unverifiedReq;
    uint8_t *unverifiedMemData;

    bool changedPC;
    bool willChangePC;
    TheISA::PCState newPCState;
    bool exitOnError;
    bool updateOnError;
    bool warnOnlyOnLoadError;

    InstSeqNum youngestSN;
};

/**
 * Templated Checker class.  This Checker class is templated on the
 * DynInstPtr of the instruction type that will be verified.  Proper
 * template instantiations of the Checker must be placed at the bottom
 * of checker/cpu.cc.
 */
template <class Impl>
class Checker : public CheckerCPU
{
  private:
    typedef typename Impl::DynInstPtr DynInstPtr;

  public:
    Checker(Params *p)
        : CheckerCPU(p), updateThisCycle(false), unverifiedInst(NULL)
    { }

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    void advancePC(const Fault &fault);

    void verify(DynInstPtr &inst);

    void validateInst(DynInstPtr &inst);
    void validateExecution(DynInstPtr &inst);
    void validateState();

    void copyResult(DynInstPtr &inst, uint64_t mismatch_val, int start_idx);
    void handlePendingInt();

  private:
    void handleError(DynInstPtr &inst)
    {
        if (exitOnError) {
            dumpAndExit(inst);
        } else if (updateOnError) {
            updateThisCycle = true;
        }
    }

    void dumpAndExit(DynInstPtr &inst);

    bool updateThisCycle;

    DynInstPtr unverifiedInst;

    std::list<DynInstPtr> instList;
    typedef typename std::list<DynInstPtr>::iterator InstListIt;
    void dumpInsts();
};

#endif // __CPU_CHECKER_CPU_HH__
