/*
 * Copyright (c) 2010-2012,2015 ARM Limited
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
 */

#include "arch/kernel_stats.hh"
#include "arch/stacktrace.hh"
#include "arch/tlb.hh"
#include "arch/utility.hh"
#include "arch/vtophys.hh"
#include "base/loader/symtab.hh"
#include "base/cp_annotate.hh"
#include "base/cprintf.hh"
#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/simple/base.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/profile.hh"
#include "cpu/simple/exec_context.hh"
#include "cpu/simple_thread.hh"
#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/Decode.hh"
#include "debug/Fetch.hh"
#include "debug/Quiesce.hh"
#include "mem/mem_object.hh"
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

using namespace std;
using namespace TheISA;

BaseSimpleCPU::BaseSimpleCPU(BaseSimpleCPUParams *p)
    : BaseCPU(p),
      curThread(0),
      branchPred(p->branchPred),
      traceData(NULL),
      inst(),
      _status(Idle)
{
    SimpleThread *thread;

    for (unsigned i = 0; i < numThreads; i++) {
        if (FullSystem) {
            thread = new SimpleThread(this, i, p->system,
                                      p->itb, p->dtb, p->isa[i]);
        } else {
            thread = new SimpleThread(this, i, p->system, p->workload[i],
                                      p->itb, p->dtb, p->isa[i]);
        }
        threadInfo.push_back(new SimpleExecContext(this, thread));
        ThreadContext *tc = thread->getTC();
        threadContexts.push_back(tc);
    }

    if (p->checker) {
        if (numThreads != 1)
            fatal("Checker currently does not support SMT");

        BaseCPU *temp_checker = p->checker;
        checker = dynamic_cast<CheckerCPU *>(temp_checker);
        checker->setSystem(p->system);
        // Manipulate thread context
        ThreadContext *cpu_tc = threadContexts[0];
        threadContexts[0] = new CheckerThreadContext<ThreadContext>(cpu_tc, this->checker);
    } else {
        checker = NULL;
    }
}

void
BaseSimpleCPU::init()
{
    BaseCPU::init();

    for (auto tc : threadContexts) {
        // Initialise the ThreadContext's memory proxies
        tc->initMemProxies(tc);

        if (FullSystem && !params()->switched_out) {
            // initialize CPU, including PC
            TheISA::initCPU(tc, tc->contextId());
        }
    }
}

void
BaseSimpleCPU::checkPcEventQueue()
{
    Addr oldpc, pc = threadInfo[curThread]->thread->instAddr();
    do {
        oldpc = pc;
        system->pcEventQueue.service(threadContexts[curThread]);
        pc = threadInfo[curThread]->thread->instAddr();
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
        t_info.numInsts++;
    }
    t_info.numOp++;
    t_info.numOps++;

    system->totalNumInsts++;
    t_info.thread->funcExeInst++;
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
}


void
BaseSimpleCPU::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        SimpleExecContext& t_info = *threadInfo[tid];

        std::string thread_str = name();
        if (numThreads > 1)
            thread_str += ".thread" + std::to_string(tid);

        t_info.numInsts
            .name(thread_str + ".committedInsts")
            .desc("Number of instructions committed")
            ;

        t_info.numOps
            .name(thread_str + ".committedOps")
            .desc("Number of ops (including micro ops) committed")
            ;

        t_info.numIntAluAccesses
            .name(thread_str + ".num_int_alu_accesses")
            .desc("Number of integer alu accesses")
            ;

        t_info.numFpAluAccesses
            .name(thread_str + ".num_fp_alu_accesses")
            .desc("Number of float alu accesses")
            ;

        t_info.numCallsReturns
            .name(thread_str + ".num_func_calls")
            .desc("number of times a function call or return occured")
            ;

        t_info.numCondCtrlInsts
            .name(thread_str + ".num_conditional_control_insts")
            .desc("number of instructions that are conditional controls")
            ;

        t_info.numIntInsts
            .name(thread_str + ".num_int_insts")
            .desc("number of integer instructions")
            ;

        t_info.numFpInsts
            .name(thread_str + ".num_fp_insts")
            .desc("number of float instructions")
            ;

        t_info.numIntRegReads
            .name(thread_str + ".num_int_register_reads")
            .desc("number of times the integer registers were read")
            ;

        t_info.numIntRegWrites
            .name(thread_str + ".num_int_register_writes")
            .desc("number of times the integer registers were written")
            ;

        t_info.numFpRegReads
            .name(thread_str + ".num_fp_register_reads")
            .desc("number of times the floating registers were read")
            ;

        t_info.numFpRegWrites
            .name(thread_str + ".num_fp_register_writes")
            .desc("number of times the floating registers were written")
            ;

        t_info.numCCRegReads
            .name(thread_str + ".num_cc_register_reads")
            .desc("number of times the CC registers were read")
            .flags(nozero)
            ;

        t_info.numCCRegWrites
            .name(thread_str + ".num_cc_register_writes")
            .desc("number of times the CC registers were written")
            .flags(nozero)
            ;

        t_info.numMemRefs
            .name(thread_str + ".num_mem_refs")
            .desc("number of memory refs")
            ;

        t_info.numStoreInsts
            .name(thread_str + ".num_store_insts")
            .desc("Number of store instructions")
            ;

        t_info.numLoadInsts
            .name(thread_str + ".num_load_insts")
            .desc("Number of load instructions")
            ;

        t_info.notIdleFraction
            .name(thread_str + ".not_idle_fraction")
            .desc("Percentage of non-idle cycles")
            ;

        t_info.idleFraction
            .name(thread_str + ".idle_fraction")
            .desc("Percentage of idle cycles")
            ;

        t_info.numBusyCycles
            .name(thread_str + ".num_busy_cycles")
            .desc("Number of busy cycles")
            ;

        t_info.numIdleCycles
            .name(thread_str + ".num_idle_cycles")
            .desc("Number of idle cycles")
            ;

        t_info.icacheStallCycles
            .name(thread_str + ".icache_stall_cycles")
            .desc("ICache total stall cycles")
            .prereq(t_info.icacheStallCycles)
            ;

        t_info.dcacheStallCycles
            .name(thread_str + ".dcache_stall_cycles")
            .desc("DCache total stall cycles")
            .prereq(t_info.dcacheStallCycles)
            ;

        t_info.statExecutedInstType
            .init(Enums::Num_OpClass)
            .name(thread_str + ".op_class")
            .desc("Class of executed instruction")
            .flags(total | pdf | dist)
            ;

        for (unsigned i = 0; i < Num_OpClasses; ++i) {
            t_info.statExecutedInstType.subname(i, Enums::OpClassStrings[i]);
        }

        t_info.idleFraction = constant(1.0) - t_info.notIdleFraction;
        t_info.numIdleCycles = t_info.idleFraction * numCycles;
        t_info.numBusyCycles = t_info.notIdleFraction * numCycles;

        t_info.numBranches
            .name(thread_str + ".Branches")
            .desc("Number of branches fetched")
            .prereq(t_info.numBranches);

        t_info.numPredictedBranches
            .name(thread_str + ".predictedBranches")
            .desc("Number of branches predicted as taken")
            .prereq(t_info.numPredictedBranches);

        t_info.numBranchMispred
            .name(thread_str + ".BranchMispred")
            .desc("Number of branch mispredictions")
            .prereq(t_info.numBranchMispred);
    }
}

void
BaseSimpleCPU::resetStats()
{
    for (auto &thread_info : threadInfo) {
        thread_info->notIdleFraction = (_status != Idle);
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

Addr
BaseSimpleCPU::dbg_vtophys(Addr addr)
{
    return vtophys(threadContexts[curThread], addr);
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
BaseSimpleCPU::checkForInterrupts()
{
    SimpleExecContext&t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;
    ThreadContext* tc = thread->getTC();

    if (checkInterrupts(tc)) {
        Fault interrupt = interrupts[curThread]->getInterrupt(tc);

        if (interrupt != NoFault) {
            t_info.fetchOffset = 0;
            interrupts[curThread]->updateIntrInfo(tc);
            interrupt->invoke(tc);
            thread->decoder.reset();
        }
    }
}


void
BaseSimpleCPU::setupFetchRequest(Request *req)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Addr instAddr = thread->instAddr();

    // set up memory request for instruction fetch
    DPRINTF(Fetch, "Fetch: PC:%08p\n", instAddr);

    Addr fetchPC = (instAddr & PCMask) + t_info.fetchOffset;
    req->setVirt(0, fetchPC, sizeof(MachInst), Request::INST_FETCH, instMasterId(),
            instAddr);
}


void
BaseSimpleCPU::preExecute()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    // maintain $r0 semantics
    thread->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
    thread->setFloatReg(ZeroReg, 0.0);
#endif // ALPHA_ISA

    // check for instruction-count-based events
    comInstEventQueue[curThread]->serviceEvents(t_info.numInst);
    system->instEventQueue.serviceEvents(system->totalNumInsts);

    // decode the instruction
    inst = gtoh(inst);

    TheISA::PCState pcState = thread->pcState();

    if (isRomMicroPC(pcState.microPC())) {
        t_info.stayAtPC = false;
        curStaticInst = microcodeRom.fetchMicroop(pcState.microPC(),
                                                  curMacroStaticInst);
    } else if (!curMacroStaticInst) {
        //We're not in the middle of a macro instruction
        StaticInstPtr instPtr = NULL;

        TheISA::Decoder *decoder = &(thread->decoder);

        //Predecode, ie bundle up an ExtMachInst
        //If more fetch data is needed, pass it in.
        Addr fetchPC = (pcState.instAddr() & PCMask) + t_info.fetchOffset;
        //if(decoder->needMoreBytes())
            decoder->moreBytes(pcState, fetchPC, inst);
        //else
        //    decoder->process();

        //Decode an instruction if one is ready. Otherwise, we'll have to
        //fetch beyond the MachInst at the current pc.
        instPtr = decoder->decode(pcState);
        if (instPtr) {
            t_info.stayAtPC = false;
            thread->pcState(pcState);
        } else {
            t_info.stayAtPC = true;
            t_info.fetchOffset += sizeof(MachInst);
        }

        //If we decoded an instruction and it's microcoded, start pulling
        //out micro ops
        if (instPtr && instPtr->isMacroop()) {
            curMacroStaticInst = instPtr;
            curStaticInst =
                curMacroStaticInst->fetchMicroop(pcState.microPC());
        } else {
            curStaticInst = instPtr;
        }
    } else {
        //Read the next micro op from the macro op
        curStaticInst = curMacroStaticInst->fetchMicroop(pcState.microPC());
    }

    //If we decoded an instruction this "tick", record information about it.
    if (curStaticInst) {
#if TRACING_ON
        traceData = tracer->getInstRecord(curTick(), thread->getTC(),
                curStaticInst, thread->pcState(), curMacroStaticInst);

        DPRINTF(Decode,"Decode: Decoded %s instruction: %#x\n",
                curStaticInst->getName(), curStaticInst->machInst);
#endif // TRACING_ON
    }

    if (branchPred && curStaticInst &&
        curStaticInst->isControl()) {
        // Use a fake sequence number since we only have one
        // instruction in flight at the same time.
        const InstSeqNum cur_sn(0);
        t_info.predPC = thread->pcState();
        const bool predict_taken(
            branchPred->predict(curStaticInst, cur_sn, t_info.predPC,
                                curThread));

        if (predict_taken)
            ++t_info.numPredictedBranches;
    }
}

void
BaseSimpleCPU::postExecute()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    assert(curStaticInst);

    TheISA::PCState pc = threadContexts[curThread]->pcState();
    Addr instAddr = pc.instAddr();
    if (FullSystem && thread->profile) {
        bool usermode = TheISA::inUserMode(threadContexts[curThread]);
        thread->profilePC = usermode ? 1 : instAddr;
        ProfileNode *node = thread->profile->consume(threadContexts[curThread],
                                                     curStaticInst);
        if (node)
            thread->profileNode = node;
    }

    if (curStaticInst->isMemRef()) {
        t_info.numMemRefs++;
    }

    if (curStaticInst->isLoad()) {
        ++t_info.numLoad;
        comLoadEventQueue[curThread]->serviceEvents(t_info.numLoad);
    }

    if (CPA::available()) {
        CPA::cpa()->swAutoBegin(threadContexts[curThread], pc.nextInstAddr());
    }

    if (curStaticInst->isControl()) {
        ++t_info.numBranches;
    }

    /* Power model statistics */
    //integer alu accesses
    if (curStaticInst->isInteger()){
        t_info.numIntAluAccesses++;
        t_info.numIntInsts++;
    }

    //float alu accesses
    if (curStaticInst->isFloating()){
        t_info.numFpAluAccesses++;
        t_info.numFpInsts++;
    }

    //number of function calls/returns to get window accesses
    if (curStaticInst->isCall() || curStaticInst->isReturn()){
        t_info.numCallsReturns++;
    }

    //the number of branch predictions that will be made
    if (curStaticInst->isCondCtrl()){
        t_info.numCondCtrlInsts++;
    }

    //result bus acceses
    if (curStaticInst->isLoad()){
        t_info.numLoadInsts++;
    }

    if (curStaticInst->isStore()){
        t_info.numStoreInsts++;
    }
    /* End power model statistics */

    t_info.statExecutedInstType[curStaticInst->opClass()]++;

    if (FullSystem)
        traceFunctions(instAddr);

    if (traceData) {
        traceData->dump();
        delete traceData;
        traceData = NULL;
    }

    // Call CPU instruction commit probes
    probeInstCommit(curStaticInst);
}

void
BaseSimpleCPU::advancePC(const Fault &fault)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const bool branching(thread->pcState().branching());

    //Since we're moving to a new pc, zero out the offset
    t_info.fetchOffset = 0;
    if (fault != NoFault) {
        curMacroStaticInst = StaticInst::nullStaticInstPtr;
        fault->invoke(threadContexts[curThread], curStaticInst);
        thread->decoder.reset();
    } else {
        if (curStaticInst) {
            if (curStaticInst->isLastMicroop())
                curMacroStaticInst = StaticInst::nullStaticInstPtr;
            TheISA::PCState pcState = thread->pcState();
            TheISA::advancePC(pcState, curStaticInst);
            thread->pcState(pcState);
        }
    }

    if (branchPred && curStaticInst && curStaticInst->isControl()) {
        // Use a fake sequence number since we only have one
        // instruction in flight at the same time.
        const InstSeqNum cur_sn(0);

        if (t_info.predPC == thread->pcState()) {
            // Correctly predicted branch
            branchPred->update(cur_sn, curThread);
        } else {
            // Mis-predicted branch
            branchPred->squash(cur_sn, thread->pcState(), branching, curThread);
            ++t_info.numBranchMispred;
        }
    }
}

void
BaseSimpleCPU::startup()
{
    BaseCPU::startup();
    for (auto& t_info : threadInfo)
        t_info->thread->startup();
}
