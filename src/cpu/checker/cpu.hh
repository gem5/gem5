/*
 * Copyright (c) 2011, 2016-2018, 2020-2021 Arm Limited
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
 */

#ifndef __CPU_CHECKER_CPU_HH__
#define __CPU_CHECKER_CPU_HH__

#include <list>
#include <map>
#include <queue>

#include "arch/generic/pcstate.hh"
#include "base/statistics.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/inst_res.hh"
#include "cpu/pc_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "debug/Checker.hh"
#include "mem/request.hh"
#include "params/CheckerCPU.hh"
#include "sim/eventq.hh"

namespace gem5
{

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
    /** id attached to all issued requests */
    RequestorID requestorId;

  public:
    void init() override;

    PARAMS(CheckerCPU);
    CheckerCPU(const Params &p);
    virtual ~CheckerCPU();

    void setSystem(System *system);

    void setIcachePort(RequestPort *icache_port);

    void setDcachePort(RequestPort *dcache_port);

    Port &
    getDataPort() override
    {
        // the checker does not have ports on its own so return the
        // data port of the actual CPU core
        assert(dcachePort);
        return *dcachePort;
    }

    Port &
    getInstPort() override
    {
        // the checker does not have ports on its own so return the
        // data port of the actual CPU core
        assert(icachePort);
        return *icachePort;
    }

  protected:
    std::vector<Process *> workload;

    System *systemPtr;

    RequestPort *icachePort;
    RequestPort *dcachePort;

    ThreadContext *tc;

    BaseMMU *mmu;

    // ISAs like ARM can have multiple destination registers to check,
    // keep them all in a std::queue
    std::queue<InstResult> result;

    StaticInstPtr curStaticInst;
    StaticInstPtr curMacroStaticInst;

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;

    std::queue<int> miscRegIdxs;

  public:
    // Primary thread being run.
    SimpleThread *thread;

    BaseMMU *
    getMMUPtr()
    {
        return mmu;
    }

    virtual Counter
    totalInsts() const override
    {
        return 0;
    }

    virtual Counter
    totalOps() const override
    {
        return 0;
    }

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

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

    RegVal
    getRegOperand(const StaticInst *si, int idx) override
    {
        const RegId &id = si->srcRegIdx(idx);
        if (id.is(InvalidRegClass))
            return 0;
        return thread->getReg(id);
    }

    void
    getRegOperand(const StaticInst *si, int idx, void *val) override
    {
        thread->getReg(si->srcRegIdx(idx), val);
    }

    void *
    getWritableRegOperand(const StaticInst *si, int idx) override
    {
        return thread->getWritableReg(si->destRegIdx(idx));
    }

    void
    setRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId &id = si->destRegIdx(idx);
        if (id.is(InvalidRegClass))
            return;
        const RegId flat = id.flatten(*thread->getIsaPtr());
        thread->setReg(flat, val);
        result.emplace(flat.regClass(), val);
    }

    void
    setRegOperand(const StaticInst *si, int idx, const void *val) override
    {
        const RegId &id = si->destRegIdx(idx);
        if (id.is(InvalidRegClass))
            return;
        const RegId flat = id.flatten(*thread->getIsaPtr());
        thread->setReg(flat, val);
        result.emplace(flat.regClass(), val);
    }

    bool
    readPredicate() const override
    {
        return thread->readPredicate();
    }

    void
    setPredicate(bool val) override
    {
        thread->setPredicate(val);
    }

    bool
    readMemAccPredicate() const override
    {
        return thread->readMemAccPredicate();
    }

    void
    setMemAccPredicate(bool val) override
    {
        thread->setMemAccPredicate(val);
    }

    uint64_t
    getHtmTransactionUid() const override
    {
        panic("not yet supported!");
        return 0;
    };

    uint64_t
    newHtmTransactionUid() const override
    {
        panic("not yet supported!");
        return 0;
    };

    Fault
    initiateMemMgmtCmd(Request::Flags flags) override
    {
        panic("not yet supported!");
        return NoFault;
    }

    bool
    inHtmTransactionalState() const override
    {
        return (getHtmTransactionalDepth() > 0);
    }

    uint64_t
    getHtmTransactionalDepth() const override
    {
        assert(thread->htmTransactionStarts >= thread->htmTransactionStops);
        return (thread->htmTransactionStarts - thread->htmTransactionStops);
    }

    const PCStateBase &
    pcState() const override
    {
        return thread->pcState();
    }

    void
    pcState(const PCStateBase &val) override
    {
        DPRINTF(Checker, "Changing PC to %s, old PC %s.\n", val,
                thread->pcState());
        thread->pcState(val);
    }

    //////////////////////////////////////////

    RegVal
    readMiscRegNoEffect(int misc_reg) const
    {
        return thread->readMiscRegNoEffect(misc_reg);
    }

    RegVal
    readMiscReg(int misc_reg) override
    {
        return thread->readMiscReg(misc_reg);
    }

    void
    setMiscRegNoEffect(int misc_reg, RegVal val)
    {
        DPRINTF(Checker, "Setting misc reg %d with no effect to check later\n",
                misc_reg);
        miscRegIdxs.push(misc_reg);
        return thread->setMiscRegNoEffect(misc_reg, val);
    }

    void
    setMiscReg(int misc_reg, RegVal val) override
    {
        DPRINTF(Checker, "Setting misc reg %d with effect to check later\n",
                misc_reg);
        miscRegIdxs.push(misc_reg);
        return thread->setMiscReg(misc_reg, val);
    }

    RegVal
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        const RegId &reg = si->srcRegIdx(idx);
        assert(reg.is(MiscRegClass));
        return thread->readMiscReg(reg.index());
    }

    void
    setMiscRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId &reg = si->destRegIdx(idx);
        assert(reg.is(MiscRegClass));
        return this->setMiscReg(reg.index(), val);
    }

    /////////////////////////////////////////

    void
    recordPCChange(const PCStateBase &val)
    {
        changedPC = true;
        set(newPCState, val);
    }

    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        mmu->demapPage(vaddr, asn);
    }

    // monitor/mwait funtions
    void
    armMonitor(Addr address) override
    {
        BaseCPU::armMonitor(0, address);
    }

    bool
    mwait(PacketPtr pkt) override
    {
        return BaseCPU::mwait(0, pkt);
    }

    void
    mwaitAtomic(ThreadContext *tc) override
    {
        return BaseCPU::mwaitAtomic(0, tc, thread->mmu);
    }

    AddressMonitor *
    getAddrMonitor() override
    {
        return BaseCPU::getCpuAddrMonitor(0);
    }

    /**
     * Helper function used to generate the request for a single fragment of a
     * memory access.
     *
     * Takes care of setting up the appropriate byte-enable mask for the
     * fragment, given the mask for the entire memory access.
     *
     * @param frag_addr Start address of the fragment.
     * @param size Total size of the memory access in bytes.
     * @param flags Request flags.
     * @param byte_enable Byte-enable mask for the entire memory access.
     * @param[out] frag_size Fragment size.
     * @param[in,out] size_left Size left to be processed in the memory access.
     * @return Pointer to the allocated Request, nullptr if the byte-enable
     * mask is all-false for the fragment.
     */
    RequestPtr genMemFragmentRequest(Addr frag_addr, int size,
                                     Request::Flags flags,
                                     const std::vector<bool> &byte_enable,
                                     int &frag_size, int &size_left) const;

    Fault readMem(Addr addr, uint8_t *data, unsigned size,
                  Request::Flags flags,
                  const std::vector<bool> &byte_enable) override;

    Fault writeMem(uint8_t *data, unsigned size, Addr addr,
                   Request::Flags flags, uint64_t *res,
                   const std::vector<bool> &byte_enable) override;

    Fault
    amoMem(Addr addr, uint8_t *data, unsigned size, Request::Flags flags,
           AtomicOpFunctorPtr amo_op) override
    {
        panic("AMO is not supported yet in CPU checker\n");
    }

    unsigned int
    readStCondFailures() const override
    {
        return thread->readStCondFailures();
    }

    void
    setStCondFailures(unsigned int sc_failures) override
    {}

    /////////////////////////////////////////////////////

    void
    wakeup(ThreadID tid) override
    {}

    void
    handleError()
    {
        if (exitOnError)
            dumpAndExit();
    }

    bool checkFlags(const RequestPtr &unverified_req, Addr vAddr, Addr pAddr,
                    int flags);

    void dumpAndExit();

    ThreadContext *
    tcBase() const override
    {
        return tc;
    }

    SimpleThread *
    threadBase()
    {
        return thread;
    }

    InstResult unverifiedResult;
    RequestPtr unverifiedReq;
    uint8_t *unverifiedMemData;

    bool changedPC;
    bool willChangePC;
    std::unique_ptr<PCStateBase> newPCState;
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
template <class DynInstPtr>
class Checker : public CheckerCPU
{
  public:
    Checker(const Params &p)
        : CheckerCPU(p), updateThisCycle(false), unverifiedInst(NULL)
    {}

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    void advancePC(const Fault &fault);

    void verify(const DynInstPtr &inst);

    void validateInst(const DynInstPtr &inst);
    void validateExecution(const DynInstPtr &inst);
    void validateState();

    void copyResult(const DynInstPtr &inst, const InstResult &mismatch_val,
                    int start_idx);
    void handlePendingInt();

  private:
    void
    handleError(const DynInstPtr &inst)
    {
        if (exitOnError) {
            dumpAndExit(inst);
        } else if (updateOnError) {
            updateThisCycle = true;
        }
    }

    void dumpAndExit(const DynInstPtr &inst);

    bool updateThisCycle;

    DynInstPtr unverifiedInst;

    std::list<DynInstPtr> instList;
    typedef typename std::list<DynInstPtr>::iterator InstListIt;
    void dumpInsts();
};

} // namespace gem5

#endif // __CPU_CHECKER_CPU_HH__
