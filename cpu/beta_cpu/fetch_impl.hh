// Todo: Rewrite this.  Add in branch prediction.  Fix up if squashing comes
// from decode; only the correct instructions should be killed.  This will
// probably require changing the CPU's instList functions to take a seqNum
// instead of a dyninst.  With probe path, should be able to specify
// size of data to fetch.  Will be able to get full cache line.

// Remove this later.
#define OPCODE(X)                       (X >> 26) & 0x3f

#include "cpu/exetrace.hh"
#include "mem/base_mem.hh"
#include "mem/mem_interface.hh"
#include "mem/mem_req.hh"
#include "cpu/beta_cpu/fetch.hh"

#include "sim/universe.hh"

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
    : cacheCompletionEvent(this),
      icacheInterface(params.icacheInterface),
      decodeToFetchDelay(params.decodeToFetchDelay),
      renameToFetchDelay(params.renameToFetchDelay),
      iewToFetchDelay(params.iewToFetchDelay),
      commitToFetchDelay(params.commitToFetchDelay),
      fetchWidth(params.fetchWidth),
      inst(0)
{
    // Set status to idle.
    _status = Idle;

    // Create a new memory request.
    memReq = new MemReq();
    // Not sure of this parameter.  I think it should be based on the
    // thread number.
#ifndef FULL_SYSTEM
    memReq->asid = params.asid;
#else
    memReq->asid = 0;
#endif // FULL_SYSTEM
    memReq->data = new uint8_t[64];

    // Size of cache block.
    blkSize = icacheInterface ? icacheInterface->getBlockSize() : 64;

    // Create mask to get rid of offset bits.
    cacheBlockMask = ~((int)log2(blkSize) - 1);

    // Get the size of an instruction.
    instSize = sizeof(MachInst);
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
    // How to handle an outstanding miss which gets cancelled due to squash,
    // then a new icache miss gets scheduled?
    if (_status == IcacheMissStall)
        _status = IcacheMissComplete;
}

// Note that in the SimpleFetch<>, will most likely have to provide the
// template parameters to BP and BTB.
template<class Impl>
void
SimpleFetch<Impl>::squash(Addr new_PC)
{
    DPRINTF(Fetch, "Fetch: Squashing, setting PC to: %#x.\n", new_PC);
    cpu->setNextPC(new_PC + instSize);
    cpu->setPC(new_PC);

    _status = Squashing;

    // Clear out the instructions that are no longer valid.
    // Actually maybe slightly unrealistic to kill instructions that are
    // in flight like that between stages.  Perhaps just have next
    // stage ignore those instructions or something.  In the cycle where it's
    // returning from squashing, the other stages can just ignore the inputs
    // for that cycle.

    // Tell the CPU to remove any instructions that aren't currently
    // in the ROB (instructions in flight that were killed).
    cpu->removeInstsNotInROB();
}

template<class Impl>
void
SimpleFetch<Impl>::tick()
{
#if 0
    if (fromCommit->commitInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from commit.\n");

        // In any case, squash.
        squash(fromCommit->commitInfo.nextPC);
        return;
    }

    if (fromDecode->decodeInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from decode.\n");

        // Squash unless we're already squashing?
        squash(fromDecode->decodeInfo.nextPC);
        return;
    }

    if (fromCommit->commitInfo.robSquashing) {
        DPRINTF(Fetch, "Fetch: ROB is still squashing.\n");

        // Continue to squash.
        _status = Squashing;
        return;
    }

    if (fromDecode->decodeInfo.stall ||
        fromRename->renameInfo.stall ||
        fromIEW->iewInfo.stall ||
        fromCommit->commitInfo.stall)
    {
        DPRINTF(Fetch, "Fetch: Stalling stage.\n");
        DPRINTF(Fetch, "Fetch: Statuses: Decode: %i Rename: %i IEW: %i "
                "Commit: %i\n",
                fromDecode->decodeInfo.stall,
                fromRename->renameInfo.stall,
                fromIEW->iewInfo.stall,
                fromCommit->commitInfo.stall);
        // What to do if we're already in an icache stall?
    }
#endif

    if (_status != Blocked &&
        _status != Squashing &&
        _status != IcacheMissStall) {
        DPRINTF(Fetch, "Fetch: Running stage.\n");

        fetch();
    } else if (_status == Blocked) {
        // If still being told to stall, do nothing.
        if (fromDecode->decodeInfo.stall ||
            fromRename->renameInfo.stall ||
            fromIEW->iewInfo.stall ||
            fromCommit->commitInfo.stall)
        {
            DPRINTF(Fetch, "Fetch: Stalling stage.\n");
            DPRINTF(Fetch, "Fetch: Statuses: Decode: %i Rename: %i IEW: %i "
                    "Commit: %i\n",
                    fromDecode->decodeInfo.stall,
                    fromRename->renameInfo.stall,
                    fromIEW->iewInfo.stall,
                    fromCommit->commitInfo.stall);
        } else {

            DPRINTF(Fetch, "Fetch: Done blocking.\n");
            _status = Running;
        }

        if (fromCommit->commitInfo.squash) {
            DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                    "from commit.\n");
            squash(fromCommit->commitInfo.nextPC);
            return;
        } else if (fromDecode->decodeInfo.squash) {
            DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                    "from decode.\n");
            squash(fromDecode->decodeInfo.nextPC);
            return;
        } else if (fromCommit->commitInfo.robSquashing) {
            DPRINTF(Fetch, "Fetch: ROB is still squashing.\n");
            _status = Squashing;
            return;
        }
    } else if (_status == Squashing) {
        // If there are no squash signals then change back to running.
        // Note that when a squash starts happening, commitInfo.squash will
        // be high.  But if the squash is still in progress, then only
        // commitInfo.robSquashing will be high.
        if (!fromCommit->commitInfo.squash &&
            !fromCommit->commitInfo.robSquashing) {

            DPRINTF(Fetch, "Fetch: Done squashing.\n");
            _status = Running;
        } else if (fromCommit->commitInfo.squash) {
            // If there's a new squash, then start squashing again.
            squash(fromCommit->commitInfo.nextPC);
        } else {
            // Purely a debugging statement.
            DPRINTF(Fetch, "Fetch: ROB still squashing.\n");
        }
    }

}

template<class Impl>
void
SimpleFetch<Impl>::fetch()
{
    //////////////////////////////////////////
    // Check backwards communication
    //////////////////////////////////////////

    // If branch prediction is incorrect, squash any instructions,
    // update PC, and do not fetch anything this cycle.

    // Might want to put all the PC changing stuff in one area.
    // Normally should also check here to see if there is branch
    // misprediction info to update with.
    if (fromCommit->commitInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from commit.\n");
        squash(fromCommit->commitInfo.nextPC);
        return;
    } else if (fromDecode->decodeInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from decode.\n");
        squash(fromDecode->decodeInfo.nextPC);
        return;
    } else if (fromCommit->commitInfo.robSquashing) {
        DPRINTF(Fetch, "Fetch: ROB still squashing.\n");
        _status = Squashing;
        return;
    }

    // If being told to stall, do nothing.
    if (fromDecode->decodeInfo.stall ||
        fromRename->renameInfo.stall ||
        fromIEW->iewInfo.stall ||
        fromCommit->commitInfo.stall)
    {
        DPRINTF(Fetch, "Fetch: Stalling stage.\n");
        DPRINTF(Fetch, "Fetch: Statuses: Decode: %i Rename: %i IEW: %i "
                "Commit: %i\n",
                fromDecode->decodeInfo.stall,
                fromRename->renameInfo.stall,
                fromIEW->iewInfo.stall,
                fromCommit->commitInfo.stall);
        _status = Blocked;
        return;
    }

    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////

    // If nothing else outstanding, attempt to read instructions.

#ifdef FULL_SYSTEM
    // Flag to say whether or not address is physical addr.
    unsigned flags = cpu->inPalMode() ? PHYSICAL : 0;
#else
    unsigned flags = 0;
#endif // FULL_SYSTEM

    // The current PC.
    Addr PC = cpu->readPC();

    // Fault code for memory access.
    Fault fault = No_Fault;

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.
    if (_status == IcacheMissComplete) {
        DPRINTF(Fetch, "Fetch: Icache miss is complete.\n");

        // Reset the completion event to NULL.
        memReq->completionEvent = NULL;

        _status = Running;
    } else {
        DPRINTF(Fetch, "Fetch: Attempting to translate and read "
                       "instruction, starting at PC %08p.\n",
                PC);

        // Otherwise check if the instruction exists within the cache.
        // If it does, then proceed on to read the instruction and the rest
        // of the instructions in the cache line until either the end of the
        // cache line or a predicted taken branch is encountered.
        // Note that this simply checks if the first instruction exists
        // within the cache, assuming the rest of the cache line also exists
        // within the cache.

        // Setup the memReq to do a read of the first isntruction's address.
        // Set the appropriate read size and flags as well.
        memReq->cmd = Read;
        memReq->reset(PC, instSize, flags);

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
            fault = cpu->mem->read(memReq, inst);
            // This read may change when the mem interface changes.
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
                memReq->completionEvent = &cacheCompletionEvent;
//        	    lastIcacheStall = curTick;

                // How does current model work as far as individual
                // stages scheduling/unscheduling?
                // Perhaps have only the main CPU scheduled/unscheduled,
                // and have it choose what stages to run appropriately.

                DPRINTF(Fetch, "Fetch: Stalling due to icache miss.\n");
                _status = IcacheMissStall;
                return;
            }
        }
    }

    // As far as timing goes, the CPU will need to send an event through
    // the MemReq in order to be woken up once the memory access completes.
    // Probably have a status on a per thread basis so each thread can
    // block independently and be woken up independently.

    Addr next_PC = 0;
    InstSeqNum inst_seq;

    // If the read of the first instruction was successful, then grab the
    // instructions from the rest of the cache line and put them into the
    // queue heading to decode.
    if (fault == No_Fault) {
        DPRINTF(Fetch, "Fetch: Adding instructions to queue to decode.\n");

        // Need to keep track of whether or not a predicted branch
        // ended this fetch block.
        bool predicted_branch = false;

        // Might want to keep track of various stats.
//        numLinesFetched++;

        // Get a sequence number.
        inst_seq = cpu->getAndIncrementInstSeq();

        // Because the first instruction was already fetched, create the
        // DynInst and put it into the queue to decode.
        DynInst *instruction = new DynInst(inst, PC, PC+instSize, inst_seq,
                                           cpu);
        DPRINTF(Fetch, "Fetch: Instruction %i created, with PC %#x\n",
                instruction, instruction->readPC());
        DPRINTF(Fetch, "Fetch: Instruction opcode is: %03p\n",
                OPCODE(inst));

        instruction->traceData =
            Trace::getInstRecord(curTick, cpu->xcBase(), cpu,
                                 instruction->staticInst,
                                 instruction->readPC(), 0);

        cpu->addInst(instruction);

        // Write the instruction to the first slot in the queue
        // that heads to decode.
        toDecode->insts[0] = instruction;

        // Now update the PC to fetch the next instruction in the cache
        // line.
        PC = PC + instSize;

        // Obtain the index into the cache line by getting only the low
        // order bits.
        int line_index = PC & cacheBlockMask;

        // Take instructions and put them into the queue heading to decode.
        // Then read the next instruction in the cache line.  Continue
        // until either all of the fetch bandwidth is used (not an issue for
        // non-SMT), or the end of the cache line is reached.  Note that
        // this assumes standard cachelines, and not something like a trace
        // cache where lines might not end at cache-line size aligned
        // addresses.
        // @todo: Fix the horrible amount of translates/reads that must
        // take place due to reading an entire cacheline.  Ideally it
        // should all take place at once, return an array of binary
        // instructions, which can then be used to get all the instructions
        // needed.  Figure out if I can roll it back into one loop.
        for (int fetched = 1;
             line_index < blkSize && fetched < fetchWidth;
             line_index+=instSize, ++fetched)
        {
            // Reset the mem request to setup the read of the next
            // instruction.
            memReq->reset(PC, instSize, flags);

            // Translate the instruction request.
            fault = cpu->translateInstReq(memReq);

            // Read instruction.
            if (fault == No_Fault) {
                fault = cpu->mem->read(memReq, inst);
            }

            // Check if there was a fault.
            if (fault != No_Fault) {
                panic("Fetch: Read of instruction faulted when it should "
                      "succeed; most likely exceeding cache line.\n");
            }

            // Get a sequence number.
            inst_seq = cpu->getAndIncrementInstSeq();

            // Create the actual DynInst.  Parameters are:
            // DynInst(instruction, PC, predicted PC, CPU pointer).
            // Because this simple model has no branch prediction, the
            // predicted PC will simply be PC+sizeof(MachInst).
            // Update to actually use a branch predictor to predict the
            // target in the future.
            DynInst *instruction = new DynInst(inst, PC, PC+instSize,
                                               inst_seq, cpu);
            DPRINTF(Fetch, "Fetch: Instruction %i created, with PC %#x\n",
                    instruction, instruction->readPC());
            DPRINTF(Fetch, "Fetch: Instruction opcode is: %03p\n",
                    OPCODE(inst));

            cpu->addInst(instruction);

            // Write the instruction to the proper slot in the queue
            // that heads to decode.
            toDecode->insts[fetched] = instruction;

            // Might want to keep track of various stats.
//             numInstsFetched++;

            // Now update the PC to fetch the next instruction in the cache
            // line.
            PC = PC + instSize;
        }

        // If no branches predicted taken, then increment PC with
        // fall-through path.  This simple model always predicts not
        // taken.
        if (!predicted_branch) {
            next_PC = PC;
        }
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
        // Trap will probably need a pointer to the CPU to do accessing.
        // Or an exec context. --Write ProxyExecContext eventually.
        // Avoid using this for now as the xc really shouldn't be in here.
        cpu->trap(fault);
#else // !FULL_SYSTEM
        fatal("fault (%d) detected @ PC %08p", fault, cpu->readPC());
#endif // FULL_SYSTEM
    }
}
