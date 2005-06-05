/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

// Remove this later; used only for debugging.
#define OPCODE(X)                       (X >> 26) & 0x3f


#include "arch/alpha/byte_swap.hh"
#include "cpu/exetrace.hh"
#include "mem/base_mem.hh"
#include "mem/mem_interface.hh"
#include "mem/mem_req.hh"
#include "cpu/o3/fetch.hh"

#include "sim/root.hh"

template<class Impl>
SimpleFetch<Impl>::CacheCompletionEvent
::CacheCompletionEvent(SimpleFetch *_fetch)
    : Event(&mainEventQueue),
      fetch(_fetch)
{
}

template<class Impl>
void
SimpleFetch<Impl>::CacheCompletionEvent::process()
{
    fetch->processCacheCompletion();
}

template<class Impl>
const char *
SimpleFetch<Impl>::CacheCompletionEvent::description()
{
    return "SimpleFetch cache completion event";
}

template<class Impl>
SimpleFetch<Impl>::SimpleFetch(Params &params)
    : icacheInterface(params.icacheInterface),
      branchPred(params),
      decodeToFetchDelay(params.decodeToFetchDelay),
      renameToFetchDelay(params.renameToFetchDelay),
      iewToFetchDelay(params.iewToFetchDelay),
      commitToFetchDelay(params.commitToFetchDelay),
      fetchWidth(params.fetchWidth)
{
    DPRINTF(Fetch, "Fetch: Fetch constructor called\n");

    // Set status to idle.
    _status = Idle;

    // Create a new memory request.
    memReq = new MemReq();
    // Not sure of this parameter.  I think it should be based on the
    // thread number.
#ifndef FULL_SYSTEM
    memReq->asid = 0;
#else
    memReq->asid = 0;
#endif // FULL_SYSTEM
    memReq->data = new uint8_t[64];

    // Size of cache block.
    cacheBlkSize = icacheInterface ? icacheInterface->getBlockSize() : 64;

    // Create mask to get rid of offset bits.
    cacheBlkMask = (cacheBlkSize - 1);

    // Get the size of an instruction.
    instSize = sizeof(MachInst);

    // Create space to store a cache line.
    cacheData = new uint8_t[cacheBlkSize];
}

template <class Impl>
void
SimpleFetch<Impl>::regStats()
{
    icacheStallCycles
        .name(name() + ".icacheStallCycles")
        .desc("Number of cycles fetch is stalled on an Icache miss")
        .prereq(icacheStallCycles);

    fetchedInsts
        .name(name() + ".fetchedInsts")
        .desc("Number of instructions fetch has processed")
        .prereq(fetchedInsts);
    predictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches that fetch has predicted taken")
        .prereq(predictedBranches);
    fetchCycles
        .name(name() + ".fetchCycles")
        .desc("Number of cycles fetch has run and was not squashing or"
              " blocked")
        .prereq(fetchCycles);
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

    fetch_nisn_dist
        .init(/* base value */ 0,
              /* last value */ fetchWidth,
              /* bucket size */ 1)
        .name(name() + ".FETCH:rate_dist")
        .desc("Number of instructions fetched each cycle (Total)")
        .flags(Stats::pdf)
        ;

    branchPred.regStats();
}

template<class Impl>
void
SimpleFetch<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Fetch, "Fetch: Setting the CPU pointer.\n");
    cpu = cpu_ptr;
    // This line will be removed eventually.
    memReq->xc = cpu->xcBase();
}

template<class Impl>
void
SimpleFetch<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    DPRINTF(Fetch, "Fetch: Setting the time buffer pointer.\n");
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

template<class Impl>
void
SimpleFetch<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    DPRINTF(Fetch, "Fetch: Setting the fetch queue pointer.\n");
    fetchQueue = fq_ptr;

    // Create wire to write information to proper place in fetch queue.
    toDecode = fetchQueue->getWire(0);
}

template<class Impl>
void
SimpleFetch<Impl>::processCacheCompletion()
{
    DPRINTF(Fetch, "Fetch: Waking up from cache miss.\n");

    // Only change the status if it's still waiting on the icache access
    // to return.
    // Can keep track of how many cache accesses go unused due to
    // misspeculation here.
    if (_status == IcacheMissStall)
        _status = IcacheMissComplete;
}

template <class Impl>
bool
SimpleFetch<Impl>::lookupAndUpdateNextPC(DynInstPtr &inst, Addr &next_PC)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    bool predict_taken;

    if (!inst->isControl()) {
        next_PC = next_PC + instSize;
        inst->setPredTarg(next_PC);
        return false;
    }

    predict_taken = branchPred.predict(inst, next_PC);

    if (predict_taken) {
        ++predictedBranches;
    }

    return predict_taken;
}

template <class Impl>
Fault
SimpleFetch<Impl>::fetchCacheLine(Addr fetch_PC)
{
    // Check if the instruction exists within the cache.
    // If it does, then proceed on to read the instruction and the rest
    // of the instructions in the cache line until either the end of the
    // cache line or a predicted taken branch is encountered.

#ifdef FULL_SYSTEM
    // Flag to say whether or not address is physical addr.
    unsigned flags = cpu->inPalMode() ? PHYSICAL : 0;
#else
    unsigned flags = 0;
#endif // FULL_SYSTEM

    Fault fault = No_Fault;

    // Align the fetch PC so it's at the start of a cache block.
    fetch_PC = icacheBlockAlignPC(fetch_PC);

    // Setup the memReq to do a read of the first isntruction's address.
    // Set the appropriate read size and flags as well.
    memReq->cmd = Read;
    memReq->reset(fetch_PC, cacheBlkSize, flags);

    // Translate the instruction request.
    // Should this function be
    // in the CPU class ?  Probably...ITB/DTB should exist within the
    // CPU.

    fault = cpu->translateInstReq(memReq);

    // In the case of faults, the fetch stage may need to stall and wait
    // on what caused the fetch (ITB or Icache miss).

    // If translation was successful, attempt to read the first
    // instruction.
    if (fault == No_Fault) {
        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");
        fault = cpu->mem->read(memReq, cacheData);
        // This read may change when the mem interface changes.

        fetchedCacheLines++;
    }

    // Now do the timing access to see whether or not the instruction
    // exists within the cache.
    if (icacheInterface && fault == No_Fault) {
        DPRINTF(Fetch, "Fetch: Doing timing memory access.\n");
        memReq->completionEvent = NULL;

        memReq->time = curTick;

        MemAccessResult result = icacheInterface->access(memReq);

        // If the cache missed (in this model functional and timing
        // memories are different), then schedule an event to wake
        // up this stage once the cache miss completes.
        if (result != MA_HIT && icacheInterface->doEvents()) {
            memReq->completionEvent = new CacheCompletionEvent(this);

            // How does current model work as far as individual
            // stages scheduling/unscheduling?
            // Perhaps have only the main CPU scheduled/unscheduled,
            // and have it choose what stages to run appropriately.

            DPRINTF(Fetch, "Fetch: Stalling due to icache miss.\n");
            _status = IcacheMissStall;
        }
    }

    return fault;
}

template <class Impl>
inline void
SimpleFetch<Impl>::doSquash(const Addr &new_PC)
{
    DPRINTF(Fetch, "Fetch: Squashing, setting PC to: %#x.\n", new_PC);

    cpu->setNextPC(new_PC + instSize);
    cpu->setPC(new_PC);

    // Clear the icache miss if it's outstanding.
    if (_status == IcacheMissStall && icacheInterface) {
        DPRINTF(Fetch, "Fetch: Squashing outstanding Icache miss.\n");
        // @todo: Use an actual thread number here.
        icacheInterface->squash(0);
    }

    _status = Squashing;

    ++fetchSquashCycles;
}

template<class Impl>
void
SimpleFetch<Impl>::squashFromDecode(const Addr &new_PC,
                                    const InstSeqNum &seq_num)
{
    DPRINTF(Fetch, "Fetch: Squashing from decode.\n");

    doSquash(new_PC);

    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num);
}

template <class Impl>
void
SimpleFetch<Impl>::squash(const Addr &new_PC)
{
    DPRINTF(Fetch, "Fetch: Squash from commit.\n");

    doSquash(new_PC);

    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB();
}

template<class Impl>
void
SimpleFetch<Impl>::tick()
{
    // Check squash signals from commit.
    if (fromCommit->commitInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from commit.\n");

        // In any case, squash.
        squash(fromCommit->commitInfo.nextPC);

        // Also check if there's a mispredict that happened.
        if (fromCommit->commitInfo.branchMispredict) {
            branchPred.squash(fromCommit->commitInfo.doneSeqNum,
                              fromCommit->commitInfo.nextPC,
                              fromCommit->commitInfo.branchTaken);
        } else {
            branchPred.squash(fromCommit->commitInfo.doneSeqNum);
        }

        return;
    } else if (fromCommit->commitInfo.doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was braodcasted.
        branchPred.update(fromCommit->commitInfo.doneSeqNum);
    }

    // Check ROB squash signals from commit.
    if (fromCommit->commitInfo.robSquashing) {
        DPRINTF(Fetch, "Fetch: ROB is still squashing.\n");

        // Continue to squash.
        _status = Squashing;

        ++fetchSquashCycles;
        return;
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from decode.\n");

        // Update the branch predictor.
        if (fromDecode->decodeInfo.branchMispredict) {
            branchPred.squash(fromDecode->decodeInfo.doneSeqNum,
                              fromDecode->decodeInfo.nextPC,
                              fromDecode->decodeInfo.branchTaken);
        } else {
            branchPred.squash(fromDecode->decodeInfo.doneSeqNum);
        }

        if (_status != Squashing) {
            // Squash unless we're already squashing?
            squashFromDecode(fromDecode->decodeInfo.nextPC,
                             fromDecode->decodeInfo.doneSeqNum);
            return;
        }
    }

    // Check if any of the stall signals are high.
    if (fromDecode->decodeInfo.stall ||
        fromRename->renameInfo.stall ||
        fromIEW->iewInfo.stall ||
        fromCommit->commitInfo.stall)
    {
        // Block stage, regardless of current status.

        DPRINTF(Fetch, "Fetch: Stalling stage.\n");
        DPRINTF(Fetch, "Fetch: Statuses: Decode: %i Rename: %i IEW: %i "
                "Commit: %i\n",
                fromDecode->decodeInfo.stall,
                fromRename->renameInfo.stall,
                fromIEW->iewInfo.stall,
                fromCommit->commitInfo.stall);

        _status = Blocked;

        ++fetchBlockedCycles;
        return;
    } else if (_status == Blocked) {
        // Unblock stage if status is currently blocked and none of the
        // stall signals are being held high.
        _status = Running;

        ++fetchBlockedCycles;
        return;
    }

    // If fetch has reached this point, then there are no squash signals
    // still being held high.  Check if fetch is in the squashing state;
    // if so, fetch can switch to running.
    // Similarly, there are no blocked signals still being held high.
    // Check if fetch is in the blocked state; if so, fetch can switch to
    // running.
    if (_status == Squashing) {
        DPRINTF(Fetch, "Fetch: Done squashing, switching to running.\n");

        // Switch status to running
        _status = Running;

        ++fetchCycles;

        fetch();
    } else if (_status != IcacheMissStall) {
        DPRINTF(Fetch, "Fetch: Running stage.\n");

        ++fetchCycles;

        fetch();
    }
}

template<class Impl>
void
SimpleFetch<Impl>::fetch()
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////

    // The current PC.
    Addr fetch_PC = cpu->readPC();

    // Fault code for memory access.
    Fault fault = No_Fault;

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    if (_status == IcacheMissComplete) {
        DPRINTF(Fetch, "Fetch: Icache miss is complete.\n");

        // Reset the completion event to NULL.
        memReq->completionEvent = NULL;

        _status = Running;
    } else {
        DPRINTF(Fetch, "Fetch: Attempting to translate and read "
                       "instruction, starting at PC %08p.\n",
                fetch_PC);

        fault = fetchCacheLine(fetch_PC);
    }

    // If we had a stall due to an icache miss, then return.  It'd
    // be nicer if this were handled through the kind of fault that
    // is returned by the function.
    if (_status == IcacheMissStall) {
        return;
    }

    // As far as timing goes, the CPU will need to send an event through
    // the MemReq in order to be woken up once the memory access completes.
    // Probably have a status on a per thread basis so each thread can
    // block independently and be woken up independently.

    Addr next_PC = fetch_PC;
    InstSeqNum inst_seq;
    MachInst inst;
    unsigned offset = fetch_PC & cacheBlkMask;
    unsigned fetched;

    if (fault == No_Fault) {
        // If the read of the first instruction was successful, then grab the
        // instructions from the rest of the cache line and put them into the
        // queue heading to decode.

        DPRINTF(Fetch, "Fetch: Adding instructions to queue to decode.\n");

        //////////////////////////
        // Fetch first instruction
        //////////////////////////

        // Need to keep track of whether or not a predicted branch
        // ended this fetch block.
        bool predicted_branch = false;

        for (fetched = 0;
             offset < cacheBlkSize &&
                 fetched < fetchWidth &&
                 !predicted_branch;
             ++fetched)
        {

            // Get a sequence number.
            inst_seq = cpu->getAndIncrementInstSeq();

            // Make sure this is a valid index.
            assert(offset <= cacheBlkSize - instSize);

            // Get the instruction from the array of the cache line.
            inst = htoa(*reinterpret_cast<MachInst *>
                        (&cacheData[offset]));

            // Create a new DynInst from the instruction fetched.
            DynInstPtr instruction = new DynInst(inst, fetch_PC, next_PC,
                                                 inst_seq, cpu);

            DPRINTF(Fetch, "Fetch: Instruction %i created, with PC %#x\n",
                    inst_seq, instruction->readPC());

            DPRINTF(Fetch, "Fetch: Instruction opcode is: %03p\n",
                    OPCODE(inst));

            instruction->traceData =
                Trace::getInstRecord(curTick, cpu->xcBase(), cpu,
                                     instruction->staticInst,
                                     instruction->readPC(), 0);

            predicted_branch = lookupAndUpdateNextPC(instruction, next_PC);

            // Add instruction to the CPU's list of instructions.
            cpu->addInst(instruction);

            // Write the instruction to the first slot in the queue
            // that heads to decode.
            toDecode->insts[fetched] = instruction;

            toDecode->size++;

            // Increment stat of fetched instructions.
            ++fetchedInsts;

            // Move to the next instruction, unless we have a branch.
            fetch_PC = next_PC;

            offset+= instSize;
        }

        fetch_nisn_dist.sample(fetched);
    }

    // Now that fetching is completed, update the PC to signify what the next
    // cycle will be.  Might want to move this to the beginning of this
    // function so that the PC updates at the beginning of everything.
    // Or might want to leave setting the PC to the main CPU, with fetch
    // only changing the nextPC (will require correct determination of
    // next PC).
    if (fault == No_Fault) {
        DPRINTF(Fetch, "Fetch: Setting PC to %08p.\n", next_PC);
        cpu->setPC(next_PC);
        cpu->setNextPC(next_PC + instSize);
    } else {
        // If the issue was an icache miss, then we can just return and
        // wait until it is handled.
        if (_status == IcacheMissStall) {
            return;
        }

        // Handle the fault.
        // This stage will not be able to continue until all the ROB
        // slots are empty, at which point the fault can be handled.
        // The only other way it can wake up is if a squash comes along
        // and changes the PC.  Not sure how to handle that case...perhaps
        // have it handled by the upper level CPU class which peeks into the
        // time buffer and sees if a squash comes along, in which case it
        // changes the status.

        DPRINTF(Fetch, "Fetch: Blocked, need to handle the trap.\n");

        _status = Blocked;
#ifdef FULL_SYSTEM
//        cpu->trap(fault);
        // Send a signal to the ROB indicating that there's a trap from the
        // fetch stage that needs to be handled.  Need to indicate that
        // there's a fault, and the fault type.
#else // !FULL_SYSTEM
        fatal("fault (%d) detected @ PC %08p", fault, cpu->readPC());
#endif // FULL_SYSTEM
    }
}
