/*
 * Copyright (c) 2010-2012, 2015, 2017, 2018, 2020 ARM Limited
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
 */

#include "cpu/simple/base.hh"

#include "arch/generic/decoder.hh"
#include "base/cprintf.hh"
#include "base/inifile.hh"
#include "base/loader/symtab.hh"
#include "base/logging.hh"
#include "base/pollevent.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/null_static_inst.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/simple/exec_context.hh"
#include "cpu/simple_thread.hh"
#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/Decode.hh"
#include "debug/ExecFaulting.hh"
#include "debug/Fetch.hh"
#include "debug/HtmCpu.hh"
#include "debug/Quiesce.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/BaseSimpleCPU.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

BaseSimpleCPU::BaseSimpleCPU(const BaseSimpleCPUParams &p)
    : BaseCPU(p),
      curThread(0),
      branchPred(p.branchPred),
      traceData(NULL),
      _status(Idle)
{
    SimpleThread *thread;

    for (unsigned i = 0; i < numThreads; i++) {
        if (FullSystem) {
            thread = new SimpleThread(
                this, i, p.system, p.mmu, p.isa[i], p.decoder[i]);
        } else {
            thread = new SimpleThread(
                this, i, p.system, p.workload[i], p.mmu, p.isa[i],
                p.decoder[i]);
        }
        threadInfo.push_back(new SimpleExecContext(this, thread));
        ThreadContext *tc = thread->getTC();
        threadContexts.push_back(tc);
    }

    if (p.checker) {
        if (numThreads != 1)
            fatal("Checker currently does not support SMT");

        BaseCPU *temp_checker = p.checker;
        checker = dynamic_cast<CheckerCPU *>(temp_checker);
        checker->setSystem(p.system);
        // Manipulate thread context
        ThreadContext *cpu_tc = threadContexts[0];
        threadContexts[0] = new CheckerThreadContext<ThreadContext>(
                cpu_tc, this->checker);
    } else {
        checker = NULL;
    }
}

void
BaseSimpleCPU::checkPcEventQueue()
{
    Addr oldpc, pc = threadInfo[curThread]->thread->pcState().instAddr();
    do {
        oldpc = pc;
        threadInfo[curThread]->thread->pcEventQueue.service(
                oldpc, threadContexts[curThread]);
        pc = threadInfo[curThread]->thread->pcState().instAddr();
    } while (oldpc != pc);
}

void
BaseSimpleCPU::swapActiveThread()
{
    if (numThreads > 1) {
        if ((!curStaticInst || !curStaticInst->isDelayedCommit()) &&
             !threadInfo[curThread]->stayAtPC) {
            // Swap active threads
            if (!activeThreads.empty()) {
                curThread = activeThreads.front();
                activeThreads.pop_front();
                activeThreads.push_back(curThread);
            }
        }
    }
}

void
BaseSimpleCPU::countInst()
{
    SimpleExecContext& t_info = *threadInfo[curThread];

    if (!curStaticInst->isMicroop() || curStaticInst->isLastMicroop()) {
        t_info.numInst++;
    }
    t_info.numOp++;
}

void
BaseSimpleCPU::countFetchInst()
{
    SimpleExecContext& t_info = *threadInfo[curThread];

    if (!curStaticInst->isMicroop() || curStaticInst->isLastMicroop()) {
        // increment thread level numInsts fetched count
        fetchStats[t_info.thread->threadId()]->numInsts++;
    }
    // increment thread level numOps fetched count
    fetchStats[t_info.thread->threadId()]->numOps++;
}

void
BaseSimpleCPU::countCommitInst()
{
    SimpleExecContext& t_info = *threadInfo[curThread];

    if (!curStaticInst->isMicroop() || curStaticInst->isLastMicroop()) {
        // increment thread level and core level numInsts count
        commitStats[t_info.thread->threadId()]->numInsts++;
        baseStats.numInsts++;
    }
    // increment thread level numOps count
    commitStats[t_info.thread->threadId()]->numOps++;
}

Counter
BaseSimpleCPU::totalInsts() const
{
    Counter total_inst = 0;
    for (auto& t_info : threadInfo) {
        total_inst += t_info->numInst;
    }

    return total_inst;
}

Counter
BaseSimpleCPU::totalOps() const
{
    Counter total_op = 0;
    for (auto& t_info : threadInfo) {
        total_op += t_info->numOp;
    }

    return total_op;
}

BaseSimpleCPU::~BaseSimpleCPU()
{
}

void
BaseSimpleCPU::haltContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
    updateCycleCounters(BaseCPU::CPU_STATE_SLEEP);
}

void
BaseSimpleCPU::resetStats()
{
    BaseCPU::resetStats();
    for (auto &thread_info : threadInfo) {
        thread_info->execContextStats.notIdleFraction = (_status != Idle);
    }
}

void
BaseSimpleCPU::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    assert(_status == Idle || _status == Running);

    threadInfo[tid]->thread->serialize(cp);
}

void
BaseSimpleCPU::unserializeThread(CheckpointIn &cp, ThreadID tid)
{
    threadInfo[tid]->thread->unserialize(cp);
}

void
change_thread_state(ThreadID tid, int activate, int priority)
{
}

void
BaseSimpleCPU::wakeup(ThreadID tid)
{
    getCpuAddrMonitor(tid)->gotWakeup = true;

    if (threadInfo[tid]->thread->status() == ThreadContext::Suspended) {
        DPRINTF(Quiesce,"[tid:%d] Suspended Processor awoke\n", tid);
        threadInfo[tid]->thread->activate();
    }
}

void
BaseSimpleCPU::traceFault()
{
    if (debug::ExecFaulting) {
        traceData->setFaulting(true);
    } else {
        delete traceData;
        traceData = NULL;
    }
}

void
BaseSimpleCPU::checkForInterrupts()
{
    SimpleExecContext&t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;
    ThreadContext* tc = thread->getTC();

    if (checkInterrupts(curThread)) {
        Fault interrupt = interrupts[curThread]->getInterrupt();

        if (interrupt != NoFault) {
            // hardware transactional memory
            // Postpone taking interrupts while executing transactions.
            assert(!std::dynamic_pointer_cast<GenericHtmFailureFault>(
                interrupt));
            if (t_info.inHtmTransactionalState()) {
                DPRINTF(HtmCpu, "Deferring pending interrupt - %s -"
                    "due to transactional state\n",
                    interrupt->name());
                return;
            }

            t_info.fetchOffset = 0;
            interrupts[curThread]->updateIntrInfo();
            interrupt->invoke(tc);
            thread->decoder->reset();
        }
    }
}


void
BaseSimpleCPU::setupFetchRequest(const RequestPtr &req)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    auto &decoder = thread->decoder;
    Addr instAddr = thread->pcState().instAddr();
    Addr fetchPC = (instAddr & decoder->pcMask()) + t_info.fetchOffset;

    // set up memory request for instruction fetch
    DPRINTF(Fetch, "Fetch: Inst PC:%08p, Fetch PC:%08p\n", instAddr, fetchPC);

    req->setVirt(fetchPC, decoder->moreBytesSize(), Request::INST_FETCH,
                 instRequestorId(), instAddr);
}

void
BaseSimpleCPU::serviceInstCountEvents()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    t_info.thread->comInstEventQueue.serviceEvents(t_info.numInst);
}

void
BaseSimpleCPU::preExecute()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    // resets predicates
    t_info.setPredicate(true);
    t_info.setMemAccPredicate(true);

    // decode the instruction
    set(preExecuteTempPC, thread->pcState());
    auto &pc_state = *preExecuteTempPC;

    auto &decoder = thread->decoder;

    if (isRomMicroPC(pc_state.microPC())) {
        t_info.stayAtPC = false;
        curStaticInst = decoder->fetchRomMicroop(
                pc_state.microPC(), curMacroStaticInst);
    } else if (!curMacroStaticInst) {
        //We're not in the middle of a macro instruction
        StaticInstPtr instPtr = NULL;

        //Predecode, ie bundle up an ExtMachInst
        //If more fetch data is needed, pass it in.
        Addr fetch_pc =
            (pc_state.instAddr() & decoder->pcMask()) + t_info.fetchOffset;

        decoder->moreBytes(pc_state, fetch_pc);

        //Decode an instruction if one is ready. Otherwise, we'll have to
        //fetch beyond the MachInst at the current pc.
        instPtr = decoder->decode(pc_state);
        if (instPtr) {
            t_info.stayAtPC = false;
            thread->pcState(pc_state);
        } else {
            t_info.stayAtPC = true;
            t_info.fetchOffset += decoder->moreBytesSize();
        }

        //If we decoded an instruction and it's microcoded, start pulling
        //out micro ops
        if (instPtr && instPtr->isMacroop()) {
            curMacroStaticInst = instPtr;
            curStaticInst =
                curMacroStaticInst->fetchMicroop(pc_state.microPC());
        } else {
            curStaticInst = instPtr;
        }
    } else {
        //Read the next micro op from the macro op
        curStaticInst = curMacroStaticInst->fetchMicroop(pc_state.microPC());
    }

    //If we decoded an instruction this "tick", record information about it.
    if (curStaticInst) {
#if TRACING_ON
        traceData = tracer->getInstRecord(curTick(), thread->getTC(),
                curStaticInst, thread->pcState(), curMacroStaticInst);
#endif // TRACING_ON
    }

    if (branchPred && curStaticInst &&
        curStaticInst->isControl()) {
        // Use a fake sequence number since we only have one
        // instruction in flight at the same time.
        const InstSeqNum cur_sn(0);
        set(t_info.predPC, thread->pcState());
        const bool predict_taken(
            branchPred->predict(curStaticInst, cur_sn, *t_info.predPC,
                curThread));

        if (predict_taken)
            ++t_info.execContextStats.numPredictedBranches;
    }

    // increment the fetch instruction stat counters
    if (curStaticInst) {
        countFetchInst();
    }
}

void
BaseSimpleCPU::postExecute()
{
    SimpleExecContext &t_info = *threadInfo[curThread];

    assert(curStaticInst);

    Addr instAddr = threadContexts[curThread]->pcState().instAddr();

    if (curStaticInst->isMemRef()) {
        executeStats[t_info.thread->threadId()]->numMemRefs++;
    }

    if (curStaticInst->isLoad()) {
        ++t_info.numLoad;
    }

    if (curStaticInst->isControl()) {
        ++fetchStats[t_info.thread->threadId()]->numBranches;
    }

    /* Power model statistics */
    //integer alu accesses
    if (curStaticInst->isInteger()){
        executeStats[t_info.thread->threadId()]->numIntAluAccesses++;
        commitStats[t_info.thread->threadId()]->numIntInsts++;
    }

    //float alu accesses
    if (curStaticInst->isFloating()){
        executeStats[t_info.thread->threadId()]->numFpAluAccesses++;
        commitStats[t_info.thread->threadId()]->numFpInsts++;
    }

    //vector alu accesses
    if (curStaticInst->isVector()){
        executeStats[t_info.thread->threadId()]->numVecAluAccesses++;
        commitStats[t_info.thread->threadId()]->numVecInsts++;
    }

    //Matrix alu accesses
    if (curStaticInst->isMatrix()){
        t_info.execContextStats.numMatAluAccesses++;
        t_info.execContextStats.numMatInsts++;
    }

    //number of function calls/returns to get window accesses
    if (curStaticInst->isCall() || curStaticInst->isReturn()){
        t_info.execContextStats.numCallsReturns++;
    }

    //result bus acceses
    if (curStaticInst->isLoad()){
        commitStats[t_info.thread->threadId()]->numLoadInsts++;
    }

    if (curStaticInst->isStore() || curStaticInst->isAtomic()){
        commitStats[t_info.thread->threadId()]->numStoreInsts++;
    }
    /* End power model statistics */

    commitStats[t_info.thread->threadId()]
        ->committedInstType[curStaticInst->opClass()]++;
    commitStats[t_info.thread->threadId()]->updateComCtrlStats(curStaticInst);

    /* increment the committed numInsts and numOps stats */
    countCommitInst();

    if (FullSystem)
        traceFunctions(instAddr);

    if (traceData) {
        traceData->dump();
        delete traceData;
        traceData = NULL;
    }

    // Call CPU instruction commit probes
    probeInstCommit(curStaticInst, instAddr);
}

void
BaseSimpleCPU::advancePC(const Fault &fault)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const bool branching = thread->pcState().branching();

    //Since we're moving to a new pc, zero out the offset
    t_info.fetchOffset = 0;
    if (fault != NoFault) {
        curMacroStaticInst = nullStaticInstPtr;
        fault->invoke(threadContexts[curThread], curStaticInst);
        thread->decoder->reset();
    } else {
        if (curStaticInst) {
            if (curStaticInst->isLastMicroop())
                curMacroStaticInst = nullStaticInstPtr;
            curStaticInst->advancePC(thread);
        }
    }

    if (branchPred && curStaticInst && curStaticInst->isControl()) {
        // Use a fake sequence number since we only have one
        // instruction in flight at the same time.
        const InstSeqNum cur_sn(0);

        if (*t_info.predPC != thread->pcState()) {

            // Mis-predicted branch
            branchPred->squash(cur_sn, thread->pcState(), branching,
                                curThread);
            ++t_info.execContextStats.numBranchMispred;
        }
        // Update the branch predictor, this is done whether the
        // prediction was correct or not.
        branchPred->update(cur_sn, curThread);
    }
}

} // namespace gem5
