/*
 * Copyright (c) 2010-2012 ARM Limited
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
      branchPred(p->branchPred),
      traceData(NULL), thread(NULL)
{
    if (FullSystem)
        thread = new SimpleThread(this, 0, p->system, p->itb, p->dtb,
                                  p->isa[0]);
    else
        thread = new SimpleThread(this, /* thread_num */ 0, p->system,
                                  p->workload[0], p->itb, p->dtb, p->isa[0]);

    thread->setStatus(ThreadContext::Halted);

    tc = thread->getTC();

    if (p->checker) {
        BaseCPU *temp_checker = p->checker;
        checker = dynamic_cast<CheckerCPU *>(temp_checker);
        checker->setSystem(p->system);
        // Manipulate thread context
        ThreadContext *cpu_tc = tc;
        tc = new CheckerThreadContext<ThreadContext>(cpu_tc, this->checker);
    } else {
        checker = NULL;
    }

    numInst = 0;
    startNumInst = 0;
    numOp = 0;
    startNumOp = 0;
    numLoad = 0;
    startNumLoad = 0;
    lastIcacheStall = 0;
    lastDcacheStall = 0;

    threadContexts.push_back(tc);


    fetchOffset = 0;
    stayAtPC = false;
}

BaseSimpleCPU::~BaseSimpleCPU()
{
}

void
BaseSimpleCPU::deallocateContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
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

    numInsts
        .name(name() + ".committedInsts")
        .desc("Number of instructions committed")
        ;

    numOps
        .name(name() + ".committedOps")
        .desc("Number of ops (including micro ops) committed")
        ;

    numIntAluAccesses
        .name(name() + ".num_int_alu_accesses")
        .desc("Number of integer alu accesses")
        ;

    numFpAluAccesses
        .name(name() + ".num_fp_alu_accesses")
        .desc("Number of float alu accesses")
        ;

    numCallsReturns
        .name(name() + ".num_func_calls")
        .desc("number of times a function call or return occured")
        ;

    numCondCtrlInsts
        .name(name() + ".num_conditional_control_insts")
        .desc("number of instructions that are conditional controls")
        ;

    numIntInsts
        .name(name() + ".num_int_insts")
        .desc("number of integer instructions")
        ;

    numFpInsts
        .name(name() + ".num_fp_insts")
        .desc("number of float instructions")
        ;

    numIntRegReads
        .name(name() + ".num_int_register_reads")
        .desc("number of times the integer registers were read")
        ;

    numIntRegWrites
        .name(name() + ".num_int_register_writes")
        .desc("number of times the integer registers were written")
        ;

    numFpRegReads
        .name(name() + ".num_fp_register_reads")
        .desc("number of times the floating registers were read")
        ;

    numFpRegWrites
        .name(name() + ".num_fp_register_writes")
        .desc("number of times the floating registers were written")
        ;

    numCCRegReads
        .name(name() + ".num_cc_register_reads")
        .desc("number of times the CC registers were read")
        .flags(nozero)
        ;

    numCCRegWrites
        .name(name() + ".num_cc_register_writes")
        .desc("number of times the CC registers were written")
        .flags(nozero)
        ;

    numMemRefs
        .name(name()+".num_mem_refs")
        .desc("number of memory refs")
        ;

    numStoreInsts
        .name(name() + ".num_store_insts")
        .desc("Number of store instructions")
        ;

    numLoadInsts
        .name(name() + ".num_load_insts")
        .desc("Number of load instructions")
        ;

    notIdleFraction
        .name(name() + ".not_idle_fraction")
        .desc("Percentage of non-idle cycles")
        ;

    idleFraction
        .name(name() + ".idle_fraction")
        .desc("Percentage of idle cycles")
        ;

    numBusyCycles
        .name(name() + ".num_busy_cycles")
        .desc("Number of busy cycles")
        ;

    numIdleCycles
        .name(name()+".num_idle_cycles")
        .desc("Number of idle cycles")
        ;

    icacheStallCycles
        .name(name() + ".icache_stall_cycles")
        .desc("ICache total stall cycles")
        .prereq(icacheStallCycles)
        ;

    dcacheStallCycles
        .name(name() + ".dcache_stall_cycles")
        .desc("DCache total stall cycles")
        .prereq(dcacheStallCycles)
        ;

    icacheRetryCycles
        .name(name() + ".icache_retry_cycles")
        .desc("ICache total retry cycles")
        .prereq(icacheRetryCycles)
        ;

    dcacheRetryCycles
        .name(name() + ".dcache_retry_cycles")
        .desc("DCache total retry cycles")
        .prereq(dcacheRetryCycles)
        ;

    idleFraction = constant(1.0) - notIdleFraction;
    numIdleCycles = idleFraction * numCycles;
    numBusyCycles = (notIdleFraction)*numCycles;

    numBranches
        .name(name() + ".Branches")
        .desc("Number of branches fetched")
        .prereq(numBranches);

    numPredictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches predicted as taken")
        .prereq(numPredictedBranches);

    numBranchMispred
        .name(name() + ".BranchMispred")
        .desc("Number of branch mispredictions")
        .prereq(numBranchMispred);
}

void
BaseSimpleCPU::resetStats()
{
//    startNumInst = numInst;
     notIdleFraction = (_status != Idle);
}

void
BaseSimpleCPU::serializeThread(ostream &os, ThreadID tid)
{
    assert(_status == Idle || _status == Running);
    assert(tid == 0);

    thread->serialize(os);
}

void
BaseSimpleCPU::unserializeThread(Checkpoint *cp, const string &section,
                                 ThreadID tid)
{
    if (tid != 0)
        fatal("Trying to load more than one thread into a SimpleCPU\n");
    thread->unserialize(cp, section);
}

void
change_thread_state(ThreadID tid, int activate, int priority)
{
}

Addr
BaseSimpleCPU::dbg_vtophys(Addr addr)
{
    return vtophys(tc, addr);
}

void
BaseSimpleCPU::wakeup()
{
    if (thread->status() != ThreadContext::Suspended)
        return;

    DPRINTF(Quiesce,"Suspended Processor awoke\n");
    thread->activate();
}

void
BaseSimpleCPU::checkForInterrupts()
{
    if (checkInterrupts(tc)) {
        Fault interrupt = interrupts->getInterrupt(tc);

        if (interrupt != NoFault) {
            fetchOffset = 0;
            interrupts->updateIntrInfo(tc);
            interrupt->invoke(tc);
            thread->decoder.reset();
        }
    }
}


void
BaseSimpleCPU::setupFetchRequest(Request *req)
{
    Addr instAddr = thread->instAddr();

    // set up memory request for instruction fetch
    DPRINTF(Fetch, "Fetch: PC:%08p\n", instAddr);

    Addr fetchPC = (instAddr & PCMask) + fetchOffset;
    req->setVirt(0, fetchPC, sizeof(MachInst), Request::INST_FETCH, instMasterId(),
            instAddr);
}


void
BaseSimpleCPU::preExecute()
{
    // maintain $r0 semantics
    thread->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
    thread->setFloatReg(ZeroReg, 0.0);
#endif // ALPHA_ISA

    // check for instruction-count-based events
    comInstEventQueue[0]->serviceEvents(numInst);
    system->instEventQueue.serviceEvents(system->totalNumInsts);

    // decode the instruction
    inst = gtoh(inst);

    TheISA::PCState pcState = thread->pcState();

    if (isRomMicroPC(pcState.microPC())) {
        stayAtPC = false;
        curStaticInst = microcodeRom.fetchMicroop(pcState.microPC(),
                                                  curMacroStaticInst);
    } else if (!curMacroStaticInst) {
        //We're not in the middle of a macro instruction
        StaticInstPtr instPtr = NULL;

        TheISA::Decoder *decoder = &(thread->decoder);

        //Predecode, ie bundle up an ExtMachInst
        //If more fetch data is needed, pass it in.
        Addr fetchPC = (pcState.instAddr() & PCMask) + fetchOffset;
        //if(decoder->needMoreBytes())
            decoder->moreBytes(pcState, fetchPC, inst);
        //else
        //    decoder->process();

        //Decode an instruction if one is ready. Otherwise, we'll have to
        //fetch beyond the MachInst at the current pc.
        instPtr = decoder->decode(pcState);
        if (instPtr) {
            stayAtPC = false;
            thread->pcState(pcState);
        } else {
            stayAtPC = true;
            fetchOffset += sizeof(MachInst);
        }

        //If we decoded an instruction and it's microcoded, start pulling
        //out micro ops
        if (instPtr && instPtr->isMacroop()) {
            curMacroStaticInst = instPtr;
            curStaticInst = curMacroStaticInst->fetchMicroop(pcState.microPC());
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
        traceData = tracer->getInstRecord(curTick(), tc,
                curStaticInst, thread->pcState(), curMacroStaticInst);

        DPRINTF(Decode,"Decode: Decoded %s instruction: %#x\n",
                curStaticInst->getName(), curStaticInst->machInst);
#endif // TRACING_ON
    }

    if (branchPred && curStaticInst && curStaticInst->isControl()) {
        // Use a fake sequence number since we only have one
        // instruction in flight at the same time.
        const InstSeqNum cur_sn(0);
        const ThreadID tid(0);
        pred_pc = thread->pcState();
        const bool predict_taken(
            branchPred->predict(curStaticInst, cur_sn, pred_pc, tid));

        if (predict_taken)
            ++numPredictedBranches;
    }
}

void
BaseSimpleCPU::postExecute()
{
    assert(curStaticInst);

    TheISA::PCState pc = tc->pcState();
    Addr instAddr = pc.instAddr();
    if (FullSystem && thread->profile) {
        bool usermode = TheISA::inUserMode(tc);
        thread->profilePC = usermode ? 1 : instAddr;
        ProfileNode *node = thread->profile->consume(tc, curStaticInst);
        if (node)
            thread->profileNode = node;
    }

    if (curStaticInst->isMemRef()) {
        numMemRefs++;
    }

    if (curStaticInst->isLoad()) {
        ++numLoad;
        comLoadEventQueue[0]->serviceEvents(numLoad);
    }

    if (CPA::available()) {
        CPA::cpa()->swAutoBegin(tc, pc.nextInstAddr());
    }

    if (curStaticInst->isControl()) {
        ++numBranches;
    }

    /* Power model statistics */
    //integer alu accesses
    if (curStaticInst->isInteger()){
        numIntAluAccesses++;
        numIntInsts++;
    }

    //float alu accesses
    if (curStaticInst->isFloating()){
        numFpAluAccesses++;
        numFpInsts++;
    }
    
    //number of function calls/returns to get window accesses
    if (curStaticInst->isCall() || curStaticInst->isReturn()){
        numCallsReturns++;
    }
    
    //the number of branch predictions that will be made
    if (curStaticInst->isCondCtrl()){
        numCondCtrlInsts++;
    }
    
    //result bus acceses
    if (curStaticInst->isLoad()){
        numLoadInsts++;
    }
    
    if (curStaticInst->isStore()){
        numStoreInsts++;
    }
    /* End power model statistics */

    if (FullSystem)
        traceFunctions(instAddr);

    if (traceData) {
        traceData->dump();
        delete traceData;
        traceData = NULL;
    }
}

void
BaseSimpleCPU::advancePC(Fault fault)
{
    const bool branching(thread->pcState().branching());

    //Since we're moving to a new pc, zero out the offset
    fetchOffset = 0;
    if (fault != NoFault) {
        curMacroStaticInst = StaticInst::nullStaticInstPtr;
        fault->invoke(tc, curStaticInst);
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
        const ThreadID tid(0);

        if (pred_pc == thread->pcState()) {
            // Correctly predicted branch
            branchPred->update(cur_sn, tid);
        } else {
            // Mis-predicted branch
            branchPred->squash(cur_sn, pcState(),
                               branching, tid);
            ++numBranchMispred;
        }
    }
}

void
BaseSimpleCPU::startup()
{
    BaseCPU::startup();
    thread->startup();
}
