/*
 * Copyright (c) 2014-2018, 2020-2021 Arm Limited
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
 */

#ifndef __CPU_SIMPLE_EXEC_CONTEXT_HH__
#define __CPU_SIMPLE_EXEC_CONTEXT_HH__

#include "base/types.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/reg_class.hh"
#include "cpu/simple/base.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/translation.hh"
#include "mem/request.hh"

namespace gem5
{

class BaseSimpleCPU;

class SimpleExecContext : public ExecContext
{
  public:
    BaseSimpleCPU *cpu;
    SimpleThread* thread;

    // This is the offset from the current pc that fetch should be performed
    Addr fetchOffset;
    // This flag says to stay at the current pc. This is useful for
    // instructions which go beyond MachInst boundaries.
    bool stayAtPC;

    // Branch prediction
    std::unique_ptr<PCStateBase> predPC;

    /** PER-THREAD STATS */
    Counter numInst;
    Counter numOp;
    // Number of simulated loads
    Counter numLoad;
    // Number of cycles stalled for I-cache responses
    Counter lastIcacheStall;
    // Number of cycles stalled for D-cache responses
    Counter lastDcacheStall;

    struct ExecContextStats : public statistics::Group
    {
        ExecContextStats(BaseSimpleCPU *cpu, SimpleThread *thread)
            : statistics::Group(cpu,
                           csprintf("exec_context.thread_%i",
                                    thread->threadId()).c_str()),
              ADD_STAT(numInsts, statistics::units::Count::get(),
                       "Number of instructions committed"),
              ADD_STAT(numOps, statistics::units::Count::get(),
                       "Number of ops (including micro ops) committed"),
              ADD_STAT(numIntAluAccesses, statistics::units::Count::get(),
                       "Number of integer alu accesses"),
              ADD_STAT(numFpAluAccesses, statistics::units::Count::get(),
                       "Number of float alu accesses"),
              ADD_STAT(numVecAluAccesses, statistics::units::Count::get(),
                       "Number of vector alu accesses"),
              ADD_STAT(numMatAluAccesses, statistics::units::Count::get(),
                       "Number of matrix alu accesses"),
              ADD_STAT(numCallsReturns, statistics::units::Count::get(),
                       "Number of times a function call or return occured"),
              ADD_STAT(numCondCtrlInsts, statistics::units::Count::get(),
                       "Number of instructions that are conditional controls"),
              ADD_STAT(numIntInsts, statistics::units::Count::get(),
                       "Number of integer instructions"),
              ADD_STAT(numFpInsts, statistics::units::Count::get(),
                       "Number of float instructions"),
              ADD_STAT(numVecInsts, statistics::units::Count::get(),
                       "Number of vector instructions"),
              ADD_STAT(numMatInsts, statistics::units::Count::get(),
                       "Number of matrix instructions"),
              ADD_STAT(numIntRegReads, statistics::units::Count::get(),
                       "Number of times the integer registers were read"),
              ADD_STAT(numIntRegWrites, statistics::units::Count::get(),
                       "Number of times the integer registers were written"),
              ADD_STAT(numFpRegReads, statistics::units::Count::get(),
                       "Number of times the floating registers were read"),
              ADD_STAT(numFpRegWrites, statistics::units::Count::get(),
                       "Number of times the floating registers were written"),
              ADD_STAT(numVecRegReads, statistics::units::Count::get(),
                       "Number of times the vector registers were read"),
              ADD_STAT(numVecRegWrites, statistics::units::Count::get(),
                       "Number of times the vector registers were written"),
              ADD_STAT(numVecPredRegReads, statistics::units::Count::get(),
                       "Number of times the predicate registers were read"),
              ADD_STAT(numVecPredRegWrites, statistics::units::Count::get(),
                       "Number of times the predicate registers were written"),
              ADD_STAT(numCCRegReads, statistics::units::Count::get(),
                       "Number of times the CC registers were read"),
              ADD_STAT(numCCRegWrites, statistics::units::Count::get(),
                       "Number of times the CC registers were written"),
              ADD_STAT(numMiscRegReads, statistics::units::Count::get(),
                       "Number of times the Misc registers were read"),
              ADD_STAT(numMiscRegWrites, statistics::units::Count::get(),
                       "Number of times the Misc registers were written"),
              ADD_STAT(numMemRefs, statistics::units::Count::get(),
                       "Number of memory refs"),
              ADD_STAT(numLoadInsts, statistics::units::Count::get(),
                       "Number of load instructions"),
              ADD_STAT(numStoreInsts, statistics::units::Count::get(),
                       "Number of store instructions"),
              ADD_STAT(numIdleCycles, statistics::units::Cycle::get(),
                       "Number of idle cycles"),
              ADD_STAT(numBusyCycles, statistics::units::Cycle::get(),
                       "Number of busy cycles"),
              ADD_STAT(notIdleFraction, statistics::units::Ratio::get(),
                       "Percentage of non-idle cycles"),
              ADD_STAT(idleFraction, statistics::units::Ratio::get(),
                       "Percentage of idle cycles"),
              ADD_STAT(icacheStallCycles, statistics::units::Cycle::get(),
                       "ICache total stall cycles"),
              ADD_STAT(dcacheStallCycles, statistics::units::Cycle::get(),
                       "DCache total stall cycles"),
              ADD_STAT(numPredictedBranches, statistics::units::Count::get(),
                       "Number of branches predicted as taken"),
              ADD_STAT(numBranchMispred, statistics::units::Count::get(),
                       "Number of branch mispredictions"),
              ADD_STAT(statExecutedInstType, statistics::units::Count::get(),
                       "Class of executed instruction."),
              numRegReads{
                  &(cpu->executeStats[thread->threadId()]->numIntRegReads),
                  &(cpu->executeStats[thread->threadId()]->numFpRegReads),
                  &(cpu->executeStats[thread->threadId()]->numVecRegReads),
                  &(cpu->executeStats[thread->threadId()]->numVecRegReads),
                  &(cpu->executeStats[thread->threadId()]->numVecPredRegReads),
                  &(cpu->executeStats[thread->threadId()]->numCCRegReads),
                  &numMatRegReads
              },
              numRegWrites{
                  &(cpu->executeStats[thread->threadId()]->numIntRegWrites),
                  &(cpu->executeStats[thread->threadId()]->numFpRegWrites),
                  &(cpu->executeStats[thread->threadId()]->numVecRegWrites),
                  &(cpu->executeStats[thread->threadId()]->numVecRegWrites),
                  &(cpu->executeStats[thread->threadId()]
                        ->numVecPredRegWrites),
                  &(cpu->executeStats[thread->threadId()]->numCCRegWrites),
                  &numMatRegWrites
              }
        {
            numCCRegReads
                .flags(statistics::nozero);

            numCCRegWrites
                .flags(statistics::nozero);

            icacheStallCycles
                .prereq(icacheStallCycles);

            dcacheStallCycles
                .prereq(dcacheStallCycles);

            statExecutedInstType
                .init(enums::Num_OpClass)
                .flags(statistics::total | statistics::pdf | statistics::dist);

            for (unsigned i = 0; i < Num_OpClasses; ++i) {
                statExecutedInstType.subname(i, enums::OpClassStrings[i]);
            }

            idleFraction = statistics::constant(1.0) - notIdleFraction;
            numIdleCycles = idleFraction * cpu->baseStats.numCycles;
            numBusyCycles = notIdleFraction * cpu->baseStats.numCycles;

            numPredictedBranches
                .prereq(numPredictedBranches);

            numBranchMispred
                .prereq(numBranchMispred);
        }

        // Number of simulated instructions
        statistics::Scalar numInsts;
        statistics::Scalar numOps;

        // Number of integer alu accesses
        statistics::Scalar numIntAluAccesses;

        // Number of float alu accesses
        statistics::Scalar numFpAluAccesses;

        // Number of vector alu accesses
        statistics::Scalar numVecAluAccesses;

        // Number of matrix alu accesses
        statistics::Scalar numMatAluAccesses;

        // Number of function calls/returns
        statistics::Scalar numCallsReturns;

        // Conditional control instructions;
        statistics::Scalar numCondCtrlInsts;

        // Number of int instructions
        statistics::Scalar numIntInsts;

        // Number of float instructions
        statistics::Scalar numFpInsts;

        // Number of vector instructions
        statistics::Scalar numVecInsts;

        // Number of matrix instructions
        statistics::Scalar numMatInsts;

        // Number of integer register file accesses
        statistics::Scalar numIntRegReads;
        statistics::Scalar numIntRegWrites;

        // Number of float register file accesses
        statistics::Scalar numFpRegReads;
        statistics::Scalar numFpRegWrites;

        // Number of vector register file accesses
        mutable statistics::Scalar numVecRegReads;
        statistics::Scalar numVecRegWrites;

        // Number of predicate register file accesses
        mutable statistics::Scalar numVecPredRegReads;
        statistics::Scalar numVecPredRegWrites;

        // Number of matrix register file accesses
        mutable statistics::Scalar numMatRegReads;
        statistics::Scalar numMatRegWrites;

        // Number of condition code register file accesses
        statistics::Scalar numCCRegReads;
        statistics::Scalar numCCRegWrites;

        // Number of misc register file accesses
        statistics::Scalar numMiscRegReads;
        statistics::Scalar numMiscRegWrites;

        // Number of simulated memory references
        statistics::Scalar numMemRefs;
        statistics::Scalar numLoadInsts;
        statistics::Scalar numStoreInsts;

        // Number of idle cycles
        statistics::Formula numIdleCycles;

        // Number of busy cycles
        statistics::Formula numBusyCycles;

        // Number of idle cycles
        statistics::Average notIdleFraction;
        statistics::Formula idleFraction;

        // Number of cycles stalled for I-cache responses
        statistics::Scalar icacheStallCycles;

        // Number of cycles stalled for D-cache responses
        statistics::Scalar dcacheStallCycles;

        /// @{
        /// Number of branches predicted as taken
        statistics::Scalar numPredictedBranches;
        /// Number of misprediced branches
        statistics::Scalar numBranchMispred;
        /// @}

        // Instruction mix histogram by OpClass
        statistics::Vector statExecutedInstType;

        std::array<statistics::Scalar *, CCRegClass + 1> numRegReads;
        std::array<statistics::Scalar *, CCRegClass + 1> numRegWrites;

    } execContextStats;

  public:
    /** Constructor */
    SimpleExecContext(BaseSimpleCPU* _cpu, SimpleThread* _thread)
        : cpu(_cpu), thread(_thread), fetchOffset(0), stayAtPC(false),
        numInst(0), numOp(0), numLoad(0), lastIcacheStall(0),
        lastDcacheStall(0), execContextStats(cpu, thread)
    { }

    RegVal
    getRegOperand(const StaticInst *si, int idx) override
    {
        const RegId &reg = si->srcRegIdx(idx);
        if (reg.is(InvalidRegClass))
            return 0;
        (*execContextStats.numRegReads[reg.classValue()])++;
        return thread->getReg(reg);
    }

    void
    getRegOperand(const StaticInst *si, int idx, void *val) override
    {
        const RegId &reg = si->srcRegIdx(idx);
        (*execContextStats.numRegReads[reg.classValue()])++;
        thread->getReg(reg, val);
    }

    void *
    getWritableRegOperand(const StaticInst *si, int idx) override
    {
        const RegId &reg = si->destRegIdx(idx);
        (*execContextStats.numRegWrites[reg.classValue()])++;
        return thread->getWritableReg(reg);
    }

    void
    setRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId &reg = si->destRegIdx(idx);
        if (reg.is(InvalidRegClass))
            return;
        (*execContextStats.numRegWrites[reg.classValue()])++;
        thread->setReg(reg, val);
    }

    void
    setRegOperand(const StaticInst *si, int idx, const void *val) override
    {
        const RegId &reg = si->destRegIdx(idx);
        (*execContextStats.numRegWrites[reg.classValue()])++;
        thread->setReg(reg, val);
    }

    RegVal
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        // update both old and new stats
        execContextStats.numMiscRegReads++;
        cpu->executeStats[thread->threadId()]->numMiscRegReads++;
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.is(MiscRegClass));
        return thread->readMiscReg(reg.index());
    }

    void
    setMiscRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        // update both old and new stats
        execContextStats.numMiscRegWrites++;
        cpu->executeStats[thread->threadId()]->numMiscRegWrites++;
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.is(MiscRegClass));
        thread->setMiscReg(reg.index(), val);
    }

    /**
     * Reads a miscellaneous register, handling any architectural
     * side effects due to reading that register.
     */
    RegVal
    readMiscReg(int misc_reg) override
    {
        // update both old and new stats
        execContextStats.numMiscRegReads++;
        cpu->executeStats[thread->threadId()]->numMiscRegReads++;
        return thread->readMiscReg(misc_reg);
    }

    /**
     * Sets a miscellaneous register, handling any architectural
     * side effects due to writing that register.
     */
    void
    setMiscReg(int misc_reg, RegVal val) override
    {
        // update both old and new stats
        execContextStats.numMiscRegWrites++;
        cpu->executeStats[thread->threadId()]->numMiscRegWrites++;
        thread->setMiscReg(misc_reg, val);
    }

    const PCStateBase &
    pcState() const override
    {
        return thread->pcState();
    }

    void
    pcState(const PCStateBase &val) override
    {
        thread->pcState(val);
    }

    Fault
    readMem(Addr addr, uint8_t *data, unsigned int size,
            Request::Flags flags,
            const std::vector<bool>& byte_enable)
        override
    {
        assert(byte_enable.size() == size);
        return cpu->readMem(addr, data, size, flags, byte_enable);
    }

    Fault
    initiateMemRead(Addr addr, unsigned int size,
                    Request::Flags flags,
                    const std::vector<bool>& byte_enable)
        override
    {
        assert(byte_enable.size() == size);
        return cpu->initiateMemRead(addr, size, flags, byte_enable);
    }

    Fault
    writeMem(uint8_t *data, unsigned int size, Addr addr,
             Request::Flags flags, uint64_t *res,
             const std::vector<bool>& byte_enable)
        override
    {
        assert(byte_enable.size() == size);
        return cpu->writeMem(data, size, addr, flags, res,
            byte_enable);
    }

    Fault
    amoMem(Addr addr, uint8_t *data, unsigned int size,
           Request::Flags flags, AtomicOpFunctorPtr amo_op) override
    {
        return cpu->amoMem(addr, data, size, flags, std::move(amo_op));
    }

    Fault
    initiateMemAMO(Addr addr, unsigned int size,
                   Request::Flags flags,
                   AtomicOpFunctorPtr amo_op) override
    {
        return cpu->initiateMemAMO(addr, size, flags, std::move(amo_op));
    }

    Fault
    initiateMemMgmtCmd(Request::Flags flags) override
    {
        return cpu->initiateMemMgmtCmd(flags);
    }

    /**
     * Sets the number of consecutive store conditional failures.
     */
    void
    setStCondFailures(unsigned int sc_failures) override
    {
        thread->setStCondFailures(sc_failures);
    }

    /**
     * Returns the number of consecutive store conditional failures.
     */
    unsigned int
    readStCondFailures() const override
    {
        return thread->readStCondFailures();
    }

    /** Returns a pointer to the ThreadContext. */
    ThreadContext *tcBase() const override { return thread->getTC(); }

    bool
    readPredicate() const override
    {
        return thread->readPredicate();
    }

    void
    setPredicate(bool val) override
    {
        thread->setPredicate(val);

        if (cpu->traceData) {
            cpu->traceData->setPredicate(val);
        }
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
        return tcBase()->getHtmCheckpointPtr()->getHtmUid();
    }

    uint64_t
    newHtmTransactionUid() const override
    {
        return tcBase()->getHtmCheckpointPtr()->newHtmUid();
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

    /**
     * Invalidate a page in the DTLB <i>and</i> ITLB.
     */
    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        thread->demapPage(vaddr, asn);
    }

    void
    armMonitor(Addr address) override
    {
        cpu->armMonitor(thread->threadId(), address);
    }

    bool
    mwait(PacketPtr pkt) override
    {
        return cpu->mwait(thread->threadId(), pkt);
    }

    void
    mwaitAtomic(ThreadContext *tc) override
    {
        cpu->mwaitAtomic(thread->threadId(), tc, thread->mmu);
    }

    AddressMonitor *
    getAddrMonitor() override
    {
        return cpu->getCpuAddrMonitor(thread->threadId());
    }
};

} // namespace gem5

#endif // __CPU_EXEC_CONTEXT_HH__
