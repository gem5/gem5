// Todo: Add in branch prediction.  With probe path, should
// be able to specify
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
      branchPred(params),
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
    cacheBlockMask = (blkSize - 1);

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

template<class Impl>
bool
SimpleFetch<Impl>::lookupAndUpdateNextPC(Addr &next_PC)
{
#if 1
    // Do branch prediction check here.
    bool predict_taken =  branchPred.BPLookup(next_PC);
    Addr predict_target;

    DPRINTF(Fetch, "Fetch: Branch predictor predicts taken? %i\n",
            predict_taken);

    if (branchPred.BTBValid(next_PC)) {
        predict_target = branchPred.BTBLookup(next_PC);
        DPRINTF(Fetch, "Fetch: BTB target is %#x.\n", predict_target);
    } else {
        predict_taken = false;
        DPRINTF(Fetch, "Fetch: BTB does not have a valid entry.\n");
    }

    // Now update the PC to fetch the next instruction in the cache
    // line.
    if (!predict_taken) {
        next_PC = next_PC + instSize;
        return false;
    } else {
        next_PC = predict_target;
        return true;
    }
#endif

#if 0
    next_PC = next_PC + instSize;
    return false;
#endif
}

template<class Impl>
void
SimpleFetch<Impl>::squash(Addr new_PC)
{
    DPRINTF(Fetch, "Fetch: Squashing, setting PC to: %#x.\n", new_PC);

    cpu->setNextPC(new_PC + instSize);
    cpu->setPC(new_PC);

    _status = Squashing;

    // Clear the icache miss if it's outstanding.
    if (_status == IcacheMissStall && icacheInterface) {
        // @todo: Use an actual thread number here.
        icacheInterface->squash(0);
    }

    // Tell the CPU to remove any instructions that aren't currently
    // in the ROB (instructions in flight that were killed).
    cpu->removeInstsNotInROB();
}

template<class Impl>
void
SimpleFetch<Impl>::tick()
{
#if 1
    // Check squash signals from commit.
    if (fromCommit->commitInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from commit.\n");

        // In any case, squash.
        squash(fromCommit->commitInfo.nextPC);

        // Also check if there's a mispredict that happened.
        if (fromCommit->commitInfo.branchMispredict) {
            branchPred.BPUpdate(fromCommit->commitInfo.mispredPC,
                                 fromCommit->commitInfo.branchTaken);
            branchPred.BTBUpdate(fromCommit->commitInfo.mispredPC,
                                  fromCommit->commitInfo.nextPC);
        }

        return;
    }

    // Check ROB squash signals from commit.
    if (fromCommit->commitInfo.robSquashing) {
        DPRINTF(Fetch, "Fetch: ROB is still squashing.\n");

        // Continue to squash.
        _status = Squashing;
        return;
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo.squash) {
        DPRINTF(Fetch, "Fetch: Squashing instructions due to squash "
                "from decode.\n");

        // Update the branch predictor.
        if (fromCommit->decodeInfo.branchMispredict) {
            branchPred.BPUpdate(fromDecode->decodeInfo.mispredPC,
                                 fromDecode->decodeInfo.branchTaken);
            branchPred.BTBUpdate(fromDecode->decodeInfo.mispredPC,
                                  fromDecode->decodeInfo.nextPC);
        }

        if (_status != Squashing) {
            // Squash unless we're already squashing?
            squash(fromDecode->decodeInfo.nextPC);
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
        return;
    } else if (_status == Blocked) {
        // Unblock stage if status is currently blocked and none of the
        // stall signals are being held high.
        _status = Running;

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
    } else if (_status != IcacheMissStall) {
        DPRINTF(Fetch, "Fetch: Running stage.\n");

        fetch();
    }
#endif

#if 0
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
#endif
}

template<class Impl>
void
SimpleFetch<Impl>::fetch()
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////

#ifdef FULL_SYSTEM
    // Flag to say whether or not address is physical addr.
    unsigned flags = cpu->inPalMode() ? PHYSICAL : 0;
#else
    unsigned flags = 0;
#endif // FULL_SYSTEM

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
        memReq->reset(fetch_PC, instSize, flags);

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

    Addr next_PC = fetch_PC;
    InstSeqNum inst_seq;

    // If the read of the first instruction was successful, then grab the
    // instructions from the rest of the cache line and put them into the
    // queue heading to decode.
    if (fault == No_Fault) {
        DPRINTF(Fetch, "Fetch: Adding instructions to queue to decode.\n");

        //////////////////////////
        // Fetch first instruction
        //////////////////////////

        // Need to keep track of whether or not a predicted branch
        // ended this fetch block.
        bool predicted_branch = false;

        // Might want to keep track of various stats.
//        numLinesFetched++;

        // Get a sequence number.
        inst_seq = cpu->getAndIncrementInstSeq();

        // Update the next PC; it either is PC+sizeof(MachInst), or
        // branch_target.  Check whether or not a branch was taken.
        predicted_branch = lookupAndUpdateNextPC(next_PC);

        // Because the first instruction was already fetched, create the
        // DynInst and put it into the queue to decode.
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

        cpu->addInst(instruction);

        // Write the instruction to the first slot in the queue
        // that heads to decode.
        toDecode->insts[0] = instruction;

        toDecode->size++;

        fetch_PC = next_PC;

        //////////////////////////
        // Fetch other instructions
        //////////////////////////

        // Obtain the index into the cache line by getting only the low
        // order bits.  Will need to do shifting as well.
        int line_index = fetch_PC & cacheBlockMask;

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
             line_index < blkSize &&
                 fetched < fetchWidth &&
                 !predicted_branch;
             line_index+=instSize, ++fetched)
        {
            // Reset the mem request to setup the read of the next
            // instruction.
            memReq->reset(fetch_PC, instSize, flags);

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

            predicted_branch = lookupAndUpdateNextPC(next_PC);

            // Create the actual DynInst.  Parameters are:
            // DynInst(instruction, PC, predicted PC, CPU pointer).
            // Because this simple model has no branch prediction, the
            // predicted PC will simply be PC+sizeof(MachInst).
            // Update to actually use a branch predictor to predict the
            // target in the future.
            DynInstPtr instruction =
                new DynInst(inst, fetch_PC, next_PC, inst_seq, cpu);

            instruction->traceData =
                Trace::getInstRecord(curTick, cpu->xcBase(), cpu,
                                     instruction->staticInst,
                                     instruction->readPC(), 0);

            DPRINTF(Fetch, "Fetch: Instruction %i created, with PC %#x\n",
                    inst_seq, instruction->readPC());
            DPRINTF(Fetch, "Fetch: Instruction opcode is: %03p\n",
                    OPCODE(inst));

            cpu->addInst(instruction);

            // Write the instruction to the proper slot in the queue
            // that heads to decode.
            toDecode->insts[fetched] = instruction;

            toDecode->size++;

            // Might want to keep track of various stats.
//             numInstsFetched++;

            // Update the PC with the next PC.
            fetch_PC = next_PC;
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
//        cpu->trap(fault);
        // Send a signal to the ROB indicating that there's a trap from the
        // fetch stage that needs to be handled.  Need to indicate that
        // there's a fault, and the fault type.
#else // !FULL_SYSTEM
        fatal("fault (%d) detected @ PC %08p", fault, cpu->readPC());
#endif // FULL_SYSTEM
    }
}
