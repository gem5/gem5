/*
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
#ifndef __CPU_OZONE_BACK_END_IMPL_HH__
#define __CPU_OZONE_BACK_END_IMPL_HH__

#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/ozone/front_end.hh"
#include "cpu/exetrace.hh"
#include "cpu/thread_context.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/faults.hh"

using namespace TheISA;

template<class Impl>
Tick
FrontEnd<Impl>::IcachePort::recvAtomic(PacketPtr pkt)
{
    panic("FrontEnd doesn't expect recvAtomic callback!");
    return curTick();
}

template<class Impl>
void
FrontEnd<Impl>::IcachePort::recvFunctional(PacketPtr pkt)
{
    warn("FrontEnd doesn't update state from functional calls");
}

template<class Impl>
bool
FrontEnd<Impl>::IcachePort::recvTiming(PacketPtr pkt)
{
    fe->processCacheCompletion(pkt);
    return true;
}

template<class Impl>
void
FrontEnd<Impl>::IcachePort::recvRetry()
{
    fe->recvRetry();
}

template <class Impl>
FrontEnd<Impl>::FrontEnd(Params *params)
    : branchPred(params),
      icachePort(this),
      numInstsReady(params->frontEndLatency, 0),
      instBufferSize(0),
      maxInstBufferSize(params->maxInstBufferSize),
      latency(params->frontEndLatency),
      width(params->frontEndWidth),
      freeRegs(params->numPhysicalRegs),
      numPhysRegs(params->numPhysicalRegs),
      serializeNext(false),
      interruptPending(false)
{
    switchedOut = false;

    status = Idle;

    memReq = NULL;
    // Size of cache block.
    cacheBlkSize = 64;

    assert(isPowerOf2(cacheBlkSize));

    // Create mask to get rid of offset bits.
    cacheBlkMask = (cacheBlkSize - 1);

    // Create space to store a cache line.
    cacheData = new uint8_t[cacheBlkSize];

    fetchCacheLineNextCycle = true;

    cacheBlkValid = cacheBlocked = false;

    retryPkt = NULL;

    fetchFault = NoFault;
}

template <class Impl>
std::string
FrontEnd<Impl>::name() const
{
    return cpu->name() + ".frontend";
}

template <class Impl>
void
FrontEnd<Impl>::setCPU(CPUType *cpu_ptr)
{
    cpu = cpu_ptr;

    icachePort.setName(this->name() + "-iport");

    if (cpu->checker) {
        cpu->checker->setIcachePort(&icachePort);
    }
}

template <class Impl>
void
FrontEnd<Impl>::setCommBuffer(TimeBuffer<CommStruct> *_comm)
{
    comm = _comm;
    // @todo: Hardcoded for now.  Allow this to be set by a latency.
    fromCommit = comm->getWire(-1);
}

template <class Impl>
void
FrontEnd<Impl>::setTC(ThreadContext *tc_ptr)
{
    tc = tc_ptr;
}

template <class Impl>
void
FrontEnd<Impl>::regStats()
{
    icacheStallCycles
        .name(name() + ".icacheStallCycles")
        .desc("Number of cycles fetch is stalled on an Icache miss")
        .prereq(icacheStallCycles);

    fetchedInsts
        .name(name() + ".fetchedInsts")
        .desc("Number of instructions fetch has processed")
        .prereq(fetchedInsts);

    fetchedBranches
        .name(name() + ".fetchedBranches")
        .desc("Number of fetched branches")
        .prereq(fetchedBranches);

    predictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches that fetch has predicted taken")
        .prereq(predictedBranches);

    fetchCycles
        .name(name() + ".fetchCycles")
        .desc("Number of cycles fetch has run and was not squashing or"
              " blocked")
        .prereq(fetchCycles);

    fetchIdleCycles
        .name(name() + ".fetchIdleCycles")
        .desc("Number of cycles fetch was idle")
        .prereq(fetchIdleCycles);

    fetchSquashCycles
        .name(name() + ".fetchSquashCycles")
        .desc("Number of cycles fetch has spent squashing")
        .prereq(fetchSquashCycles);

    fetchBlockedCycles
        .name(name() + ".fetchBlockedCycles")
        .desc("Number of cycles fetch has spent blocked")
        .prereq(fetchBlockedCycles);

    fetchedCacheLines
        .name(name() + ".fetchedCacheLines")
        .desc("Number of cache lines fetched")
        .prereq(fetchedCacheLines);

    fetchIcacheSquashes
        .name(name() + ".fetchIcacheSquashes")
        .desc("Number of outstanding Icache misses that were squashed")
        .prereq(fetchIcacheSquashes);

    fetchNisnDist
        .init(/* base value */ 0,
              /* last value */ width,
              /* bucket size */ 1)
        .name(name() + ".rateDist")
        .desc("Number of instructions fetched each cycle (Total)")
        .flags(Stats::pdf);

    idleRate
        .name(name() + ".idleRate")
        .desc("Percent of cycles fetch was idle")
        .prereq(idleRate);
    idleRate = fetchIdleCycles * 100 / cpu->numCycles;

    branchRate
        .name(name() + ".branchRate")
        .desc("Number of branch fetches per cycle")
        .flags(Stats::total);
    branchRate = fetchedBranches / cpu->numCycles;

    fetchRate
        .name(name() + ".rate")
        .desc("Number of inst fetches per cycle")
        .flags(Stats::total);
    fetchRate = fetchedInsts / cpu->numCycles;

    IFQCount
        .name(name() + ".IFQ:count")
        .desc("cumulative IFQ occupancy")
        ;

    IFQFcount
        .name(name() + ".IFQ:fullCount")
        .desc("cumulative IFQ full count")
        .flags(Stats::total)
        ;

    IFQOccupancy
        .name(name() + ".IFQ:occupancy")
        .desc("avg IFQ occupancy (inst's)")
        ;
    IFQOccupancy = IFQCount / cpu->numCycles;

    IFQLatency
        .name(name() + ".IFQ:latency")
        .desc("avg IFQ occupant latency (cycle's)")
        .flags(Stats::total)
        ;

    IFQFullRate
        .name(name() + ".IFQ:fullRate")
        .desc("fraction of time (cycles) IFQ was full")
        .flags(Stats::total);
        ;
    IFQFullRate = IFQFcount * Stats::constant(100) / cpu->numCycles;

    dispatchCountStat
        .name(name() + ".DIS:count")
        .desc("cumulative count of dispatched insts")
        .flags(Stats::total)
        ;

    dispatchedSerializing
        .name(name() + ".DIS:serializingInsts")
        .desc("count of serializing insts dispatched")
        .flags(Stats::total)
        ;

    dispatchedTempSerializing
        .name(name() + ".DIS:tempSerializingInsts")
        .desc("count of temporary serializing insts dispatched")
        .flags(Stats::total)
        ;

    dispatchSerializeStallCycles
        .name(name() + ".DIS:serializeStallCycles")
        .desc("count of cycles dispatch stalled for serializing inst")
        .flags(Stats::total)
        ;

    dispatchRate
        .name(name() + ".DIS:rate")
        .desc("dispatched insts per cycle")
        .flags(Stats::total)
        ;
    dispatchRate = dispatchCountStat / cpu->numCycles;

    regIntFull
        .name(name() + ".REG:int:full")
        .desc("number of cycles where there were no INT registers")
        ;

    regFpFull
        .name(name() + ".REG:fp:full")
        .desc("number of cycles where there were no FP registers")
        ;
    IFQLatency = IFQOccupancy / dispatchRate;

    branchPred.regStats();
}

template <class Impl>
void
FrontEnd<Impl>::tick()
{
    if (switchedOut)
        return;

    for (int insts_to_queue = numInstsReady[-latency];
         !instBuffer.empty() && insts_to_queue;
         --insts_to_queue)
    {
        DPRINTF(FE, "Transferring instruction [sn:%lli] to the feBuffer\n",
                instBuffer.front()->seqNum);
        feBuffer.push_back(instBuffer.front());
        instBuffer.pop_front();
    }

    numInstsReady.advance();

    // @todo: Maybe I want to just have direct communication...
    if (fromCommit->doneSeqNum) {
        branchPred.update(fromCommit->doneSeqNum, 0);
    }

    IFQCount += instBufferSize;
    IFQFcount += instBufferSize == maxInstBufferSize;

    // Fetch cache line
    if (status == IcacheAccessComplete) {
        cacheBlkValid = true;

        status = Running;
//        if (barrierInst)
//            status = SerializeBlocked;
        if (freeRegs <= 0)
            status = RenameBlocked;
        checkBE();
    } else if (status == IcacheWaitResponse || status == IcacheWaitRetry) {
        DPRINTF(FE, "Still in Icache wait.\n");
        icacheStallCycles++;
        return;
    }

    if (status == RenameBlocked || status == SerializeBlocked ||
        status == TrapPending || status == BEBlocked) {
        // Will cause a one cycle bubble between changing state and
        // restarting.
        DPRINTF(FE, "In blocked status.\n");

        fetchBlockedCycles++;

        if (status == SerializeBlocked) {
            dispatchSerializeStallCycles++;
        }
        updateStatus();
        return;
    } else if (status == QuiescePending) {
        DPRINTF(FE, "Waiting for quiesce to execute or get squashed.\n");
        return;
    } else if (status != IcacheAccessComplete) {
        if (fetchCacheLineNextCycle) {
            Fault fault = fetchCacheLine();
            if (fault != NoFault) {
                handleFault(fault);
                fetchFault = fault;
                return;
            }
            fetchCacheLineNextCycle = false;
        }
        // If miss, stall until it returns.
        if (status == IcacheWaitResponse || status == IcacheWaitRetry) {
            // Tell CPU to not tick me for now.
            return;
        }
    }

    fetchCycles++;

    int num_inst = 0;

    // Otherwise loop and process instructions.
    // One way to hack infinite width is to set width and maxInstBufferSize
    // both really high.  Inelegant, but probably will work.
    while (num_inst < width &&
           instBufferSize < maxInstBufferSize) {
        // Get instruction from cache line.
        DynInstPtr inst = getInstFromCacheline();

        if (!inst) {
            // PC is no longer in the cache line, end fetch.
            // Might want to check this at the end of the cycle so that
            // there's no cycle lost to checking for a new cache line.
            DPRINTF(FE, "Need to get new cache line\n");
            fetchCacheLineNextCycle = true;
            break;
        }

        processInst(inst);

        if (status == SerializeBlocked) {
            break;
        }

        // Possibly push into a time buffer that estimates the front end
        // latency
        instBuffer.push_back(inst);
        ++instBufferSize;
        numInstsReady[0]++;
        ++num_inst;

        if (inst->isQuiesce()) {
            status = QuiescePending;
            break;
        }

        if (inst->predTaken()) {
            // Start over with tick?
            break;
        } else if (freeRegs <= 0) {
            DPRINTF(FE, "Ran out of free registers to rename to!\n");
            status = RenameBlocked;
            break;
        } else if (serializeNext) {
            break;
        }
    }

    fetchNisnDist.sample(num_inst);
    checkBE();

    DPRINTF(FE, "Num insts processed: %i, Inst Buffer size: %i, Free "
            "Regs %i\n", num_inst, instBufferSize, freeRegs);
}

template <class Impl>
Fault
FrontEnd<Impl>::fetchCacheLine()
{
    // Read a cache line, based on the current PC.
    Fault fault = NoFault;

    //AlphaDep
    if (interruptPending && (PC & 0x3)) {
        return fault;
    }

    // Align the fetch PC so it's at the start of a cache block.
    Addr fetch_PC = icacheBlockAlignPC(PC);

    DPRINTF(FE, "Fetching cache line starting at %#x.\n", fetch_PC);

    // Setup the memReq to do a read of the first isntruction's address.
    // Set the appropriate read size and flags as well.
    memReq = new Request(0, fetch_PC, cacheBlkSize, 0,
                         PC, cpu->thread->contextId());

    // Translate the instruction request.
    fault = cpu->itb->translateAtomic(memReq, thread, false, true);

    // Now do the timing access to see whether or not the instruction
    // exists within the cache.
    if (fault == NoFault) {
#if 0
        if (cpu->system->memctrl->badaddr(memReq->paddr) ||
            memReq->isUncacheable()) {
            DPRINTF(FE, "Fetch: Bad address %#x (hopefully on a "
                    "misspeculating path!",
                    memReq->paddr);
            return TheISA::genMachineCheckFault();
        }
#endif

        // Build packet here.
        PacketPtr data_pkt = new Packet(memReq, Packet::ReadReq);
        data_pkt->dataStatic(cacheData);

        if (!icachePort.sendTiming(data_pkt)) {
            assert(retryPkt == NULL);
            DPRINTF(Fetch, "Out of MSHRs!\n");
            status = IcacheWaitRetry;
            retryPkt = data_pkt;
            cacheBlocked = true;
            return NoFault;
        }

        status = IcacheWaitResponse;
    }

    // Note that this will set the cache block PC a bit earlier than it should
    // be set.
    cacheBlkPC = fetch_PC;

    ++fetchedCacheLines;

    DPRINTF(FE, "Done fetching cache line.\n");

    return fault;
}

template <class Impl>
void
FrontEnd<Impl>::processInst(DynInstPtr &inst)
{
    if (processBarriers(inst)) {
        return;
    }

    Addr inst_PC = inst->readPC();

    if (!inst->isControl()) {
        inst->setPredTarg(inst->readNextPC());
    } else {
        fetchedBranches++;
        if (branchPred.predict(inst, inst_PC, inst->threadNumber)) {
            predictedBranches++;
        }
    }

    Addr next_PC = inst->readPredTarg();

    DPRINTF(FE, "[sn:%lli] Predicted and processed inst PC %#x, next PC "
            "%#x\n", inst->seqNum, inst_PC, next_PC);

//    inst->setNextPC(next_PC);

    // Not sure where I should set this
    PC = next_PC;

    renameInst(inst);
}

template <class Impl>
bool
FrontEnd<Impl>::processBarriers(DynInstPtr &inst)
{
    if (serializeNext) {
        inst->setSerializeBefore();
        serializeNext = false;
    } else if (!inst->isSerializing() &&
               !inst->isIprAccess() &&
               !inst->isStoreConditional()) {
        return false;
    }

    if ((inst->isIprAccess() || inst->isSerializeBefore()) &&
        !inst->isSerializeHandled()) {
        DPRINTF(FE, "Serialize before instruction encountered.\n");

        if (!inst->isTempSerializeBefore()) {
            dispatchedSerializing++;
            inst->setSerializeHandled();
        } else {
            dispatchedTempSerializing++;
        }

        // Change status over to SerializeBlocked so that other stages know
        // what this is blocked on.
//        status = SerializeBlocked;

//        barrierInst = inst;
//        return true;
    } else if ((inst->isStoreConditional() || inst->isSerializeAfter())
               && !inst->isSerializeHandled()) {
        DPRINTF(FE, "Serialize after instruction encountered.\n");

        inst->setSerializeHandled();

        dispatchedSerializing++;

        serializeNext = true;
        return false;
    }
    return false;
}

template <class Impl>
void
FrontEnd<Impl>::handleFault(Fault &fault)
{
    DPRINTF(FE, "Fault at fetch, telling commit\n");

    // We're blocked on the back end until it handles this fault.
    status = TrapPending;

    // Get a sequence number.
    InstSeqNum inst_seq = getAndIncrementInstSeq();
    // We will use a nop in order to carry the fault.
    ExtMachInst ext_inst = TheISA::NoopMachInst;

    // Create a new DynInst from the dummy nop.
    DynInstPtr instruction = new DynInst(ext_inst, PC,
                                         PC+sizeof(MachInst),
                                         inst_seq, cpu);
    instruction->setPredTarg(instruction->readNextPC());
//    instruction->setThread(tid);

//    instruction->setASID(tid);

    instruction->setThreadState(thread);

    instruction->traceData = NULL;

    instruction->fault = fault;
    instruction->setCanIssue();
    instBuffer.push_back(instruction);
    numInstsReady[0]++;
    ++instBufferSize;
}

template <class Impl>
void
FrontEnd<Impl>::squash(const InstSeqNum &squash_num, const Addr &next_PC,
                       const bool is_branch, const bool branch_taken)
{
    DPRINTF(FE, "Squashing from [sn:%lli], setting PC to %#x\n",
            squash_num, next_PC);

    if (fetchFault != NoFault)
        fetchFault = NoFault;

    while (!instBuffer.empty() &&
           instBuffer.back()->seqNum > squash_num) {
        DynInstPtr inst = instBuffer.back();

        DPRINTF(FE, "Squashing instruction [sn:%lli] PC %#x\n",
                inst->seqNum, inst->readPC());

        inst->clearDependents();

        instBuffer.pop_back();
        --instBufferSize;

        freeRegs+= inst->numDestRegs();
    }

    while (!feBuffer.empty() &&
           feBuffer.back()->seqNum > squash_num) {
        DynInstPtr inst = feBuffer.back();

        DPRINTF(FE, "Squashing instruction [sn:%lli] PC %#x\n",
                inst->seqNum, inst->readPC());

        inst->clearDependents();

        feBuffer.pop_back();
        --instBufferSize;

        freeRegs+= inst->numDestRegs();
    }

    // Copy over rename table from the back end.
    renameTable.copyFrom(backEnd->renameTable);

    PC = next_PC;

    // Update BP with proper information.
    if (is_branch) {
        branchPred.squash(squash_num, next_PC, branch_taken, 0);
    } else {
        branchPred.squash(squash_num, 0);
    }

    // Clear the icache miss if it's outstanding.
    if (status == IcacheWaitResponse) {
        DPRINTF(FE, "Squashing outstanding Icache access.\n");
        memReq = NULL;
    }
/*
    if (status == SerializeBlocked) {
        assert(barrierInst->seqNum > squash_num);
        barrierInst = NULL;
    }
*/
    // Unless this squash originated from the front end, we're probably
    // in running mode now.
    // Actually might want to make this latency dependent.
    status = Running;
    fetchCacheLineNextCycle = true;
}

template <class Impl>
typename Impl::DynInstPtr
FrontEnd<Impl>::getInst()
{
    if (feBuffer.empty()) {
        return NULL;
    }

    DynInstPtr inst = feBuffer.front();

    if (inst->isSerializeBefore() || inst->isIprAccess()) {
        DPRINTF(FE, "Back end is getting a serialize before inst\n");
        if (!backEnd->robEmpty()) {
            DPRINTF(FE, "Rob is not empty yet, not returning inst\n");
            return NULL;
        }
        inst->clearSerializeBefore();
    }

    feBuffer.pop_front();

    --instBufferSize;

    dispatchCountStat++;

    return inst;
}

template <class Impl>
void
FrontEnd<Impl>::processCacheCompletion(PacketPtr pkt)
{
    DPRINTF(FE, "Processing cache completion\n");

    // Do something here.
    if (status != IcacheWaitResponse ||
        pkt->req != memReq ||
        switchedOut) {
        DPRINTF(FE, "Previous fetch was squashed.\n");
        fetchIcacheSquashes++;
        delete pkt->req;
        delete pkt;
        return;
    }

    status = IcacheAccessComplete;

/*    if (checkStall(tid)) {
        fetchStatus[tid] = Blocked;
    } else {
        fetchStatus[tid] = IcacheMissComplete;
    }
*/
//    memcpy(cacheData, memReq->data, memReq->size);

    // Reset the completion event to NULL.
//    memReq->completionEvent = NULL;
    delete pkt->req;
    delete pkt;
    memReq = NULL;
}

template <class Impl>
void
FrontEnd<Impl>::addFreeRegs(int num_freed)
{
    if (status == RenameBlocked && freeRegs + num_freed > 0) {
        status = Running;
    }

    DPRINTF(FE, "Adding %i freed registers\n", num_freed);

    freeRegs+= num_freed;

//    assert(freeRegs <= numPhysRegs);
    if (freeRegs > numPhysRegs)
        freeRegs = numPhysRegs;
}

template <class Impl>
void
FrontEnd<Impl>::recvRetry()
{
    assert(cacheBlocked);
    if (retryPkt != NULL) {
        assert(status == IcacheWaitRetry);

        if (icachePort.sendTiming(retryPkt)) {
            status = IcacheWaitResponse;
            retryPkt = NULL;
            cacheBlocked = false;
        }
    } else {
        // Access has been squashed since it was sent out.  Just clear
        // the cache being blocked.
        cacheBlocked = false;
    }

}

template <class Impl>
bool
FrontEnd<Impl>::updateStatus()
{
    bool serialize_block = !backEnd->robEmpty() || instBufferSize;
    bool be_block = cpu->decoupledFrontEnd ? false : backEnd->isBlocked();
    bool ret_val = false;

    if (status == SerializeBlocked && !serialize_block) {
        status = SerializeComplete;
        ret_val = true;
    }

    if (status == BEBlocked && !be_block) {
//        if (barrierInst) {
//            status = SerializeBlocked;
//        } else {
            status = Running;
//        }
        ret_val = true;
    }
    return ret_val;
}

template <class Impl>
void
FrontEnd<Impl>::checkBE()
{
    bool be_block = cpu->decoupledFrontEnd ? false : backEnd->isBlocked();
    if (be_block) {
        if (status == Running || status == Idle) {
            status = BEBlocked;
        }
    }
}

template <class Impl>
typename Impl::DynInstPtr
FrontEnd<Impl>::getInstFromCacheline()
{
/*
    if (status == SerializeComplete) {
        DynInstPtr inst = barrierInst;
        status = Running;
        barrierInst = NULL;
        inst->clearSerializeBefore();
        return inst;
    }
*/
    InstSeqNum inst_seq;
    MachInst inst;
    // @todo: Fix this magic number used here to handle word offset (and
    // getting rid of PAL bit)
    unsigned offset = (PC & cacheBlkMask) & ~3;

    // PC of inst is not in this cache block
    if (PC >= (cacheBlkPC + cacheBlkSize) || PC < cacheBlkPC || !cacheBlkValid) {
        return NULL;
    }

    //////////////////////////
    // Fetch one instruction
    //////////////////////////

    // Get a sequence number.
    inst_seq = getAndIncrementInstSeq();

    // Make sure this is a valid index.
    assert(offset <= cacheBlkSize - sizeof(MachInst));

    // Get the instruction from the array of the cache line.
    inst = htog(*reinterpret_cast<MachInst *>(&cacheData[offset]));

#if THE_ISA == ALPHA_ISA
    ExtMachInst decode_inst = TheISA::makeExtMI(inst, PC);
#elif THE_ISA == SPARC_ISA
    ExtMachInst decode_inst = TheISA::makeExtMI(inst, tc);
#endif

    // Create a new DynInst from the instruction fetched.
    DynInstPtr instruction = new DynInst(decode_inst, PC, PC+sizeof(MachInst),
                                         inst_seq, cpu);

    instruction->setThreadState(thread);

    DPRINTF(FE, "Instruction [sn:%lli] created, with PC %#x\n%s\n",
            inst_seq, instruction->readPC(),
            instruction->staticInst->disassemble(PC));

    instruction->traceData =
        Trace::getInstRecord(curTick(), tc,
                             instruction->staticInst,
                             instruction->readPC());

    // Increment stat of fetched instructions.
    ++fetchedInsts;

    return instruction;
}

template <class Impl>
void
FrontEnd<Impl>::renameInst(DynInstPtr &inst)
{
    DynInstPtr src_inst = NULL;
    int num_src_regs = inst->numSrcRegs();
    if (num_src_regs == 0) {
        inst->setCanIssue();
    } else {
        for (int i = 0; i < num_src_regs; ++i) {
            src_inst = renameTable[inst->srcRegIdx(i)];

            inst->setSrcInst(src_inst, i);

            DPRINTF(FE, "[sn:%lli]: Src reg %i is inst [sn:%lli]\n",
                    inst->seqNum, (int)inst->srcRegIdx(i), src_inst->seqNum);

            if (src_inst->isResultReady()) {
                DPRINTF(FE, "Reg ready.\n");
                inst->markSrcRegReady(i);
            } else {
                DPRINTF(FE, "Adding to dependent list.\n");
                src_inst->addDependent(inst);
            }
        }
    }

    for (int i = 0; i < inst->numDestRegs(); ++i) {
        RegIndex idx = inst->destRegIdx(i);

        DPRINTF(FE, "Dest reg %i is now inst [sn:%lli], was previously "
                "[sn:%lli]\n",
                (int)inst->destRegIdx(i), inst->seqNum,
                renameTable[idx]->seqNum);

        inst->setPrevDestInst(renameTable[idx], i);

        renameTable[idx] = inst;
        --freeRegs;
    }
}

template <class Impl>
void
FrontEnd<Impl>::wakeFromQuiesce()
{
    DPRINTF(FE, "Waking up from quiesce\n");
    // Hopefully this is safe
    status = Running;
}

template <class Impl>
void
FrontEnd<Impl>::switchOut()
{
    switchedOut = true;
    cpu->signalSwitched();
}

template <class Impl>
void
FrontEnd<Impl>::doSwitchOut()
{
    memReq = NULL;
    squash(0, 0);
    instBuffer.clear();
    instBufferSize = 0;
    feBuffer.clear();
    status = Idle;
}

template <class Impl>
void
FrontEnd<Impl>::takeOverFrom(ThreadContext *old_tc)
{
    assert(freeRegs == numPhysRegs);
    fetchCacheLineNextCycle = true;

    cacheBlkValid = false;

    fetchFault = NoFault;
    serializeNext = false;
    barrierInst = NULL;
    status = Running;
    switchedOut = false;
    interruptPending = false;
}

template <class Impl>
void
FrontEnd<Impl>::dumpInsts()
{
    cprintf("instBuffer size: %i\n", instBuffer.size());

    InstBuffIt buff_it = instBuffer.begin();

    for (int num = 0; buff_it != instBuffer.end(); num++) {
        cprintf("Instruction:%i\nPC:%#x\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*buff_it)->readPC(), (*buff_it)->threadNumber,
                (*buff_it)->seqNum, (*buff_it)->isIssued(),
                (*buff_it)->isSquashed());
        buff_it++;
    }
}

#endif//__CPU_OZONE_BACK_END_IMPL_HH__
