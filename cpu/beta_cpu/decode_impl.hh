#ifndef __SIMPLE_DECODE_CC__
#define __SIMPLE_DECODE_CC__

#include "cpu/beta_cpu/decode.hh"

template<class Impl>
SimpleDecode<Impl>::SimpleDecode(Params &params)
    : renameToDecodeDelay(params.renameToDecodeDelay),
      iewToDecodeDelay(params.iewToDecodeDelay),
      commitToDecodeDelay(params.commitToDecodeDelay),
      fetchToDecodeDelay(params.fetchToDecodeDelay),
      decodeWidth(params.decodeWidth)
{
    DPRINTF(Decode, "Decode: decodeWidth=%i.\n", decodeWidth);
    _status = Idle;
}

template<class Impl>
void
SimpleDecode<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Decode, "Decode: Setting CPU pointer.\n");
    cpu = cpu_ptr;
}

template<class Impl>
void
SimpleDecode<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(Decode, "Decode: Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to write information back to fetch.
    toFetch = timeBuffer->getWire(0);

    // Create wires to get information from proper places in time buffer.
    fromRename = timeBuffer->getWire(-renameToDecodeDelay);
    fromIEW = timeBuffer->getWire(-iewToDecodeDelay);
    fromCommit = timeBuffer->getWire(-commitToDecodeDelay);
}

template<class Impl>
void
SimpleDecode<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    DPRINTF(Decode, "Decode: Setting decode queue pointer.\n");
    decodeQueue = dq_ptr;

    // Setup wire to write information to proper place in decode queue.
    toRename = decodeQueue->getWire(0);
}

template<class Impl>
void
SimpleDecode<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    DPRINTF(Decode, "Decode: Setting fetch queue pointer.\n");
    fetchQueue = fq_ptr;

    // Setup wire to read information from fetch queue.
    fromFetch = fetchQueue->getWire(-fetchToDecodeDelay);
}

template<class Impl>
void
SimpleDecode<Impl>::block()
{
    DPRINTF(Decode, "Decode: Blocking.\n");

    // Set the status to Blocked.
    _status = Blocked;

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidBuffer.push(*fromFetch);

    // Note that this stage only signals previous stages to stall when
    // it is the cause of the stall originates at this stage.  Otherwise
    // the previous stages are expected to check all possible stall signals.
}

template<class Impl>
inline void
SimpleDecode<Impl>::unblock()
{
    DPRINTF(Decode, "Decode: Unblocking, going to remove "
            "instructions from skid buffer.\n");
    // Remove the now processed instructions from the skid buffer.
    skidBuffer.pop();

    // If there's still information in the skid buffer, then
    // continue to tell previous stages to stall.  They will be
            // able to restart once the skid buffer is empty.
    if (!skidBuffer.empty()) {
        toFetch->decodeInfo.stall = true;
    } else {
        DPRINTF(Decode, "Decode: Finished unblocking.\n");
        _status = Running;
    }
}

// This squash is specifically for when Decode detects a PC-relative branch
// was predicted incorrectly.
template<class Impl>
void
SimpleDecode<Impl>::squash(DynInst *inst)
{
    DPRINTF(Decode, "Decode: Squashing due to incorrect branch prediction "
                    "detected at decode.\n");
    Addr new_PC = inst->nextPC;

    toFetch->decodeInfo.predIncorrect = true;
    toFetch->decodeInfo.squash = true;
    toFetch->decodeInfo.nextPC = new_PC;

    // Set status to squashing.
    _status = Squashing;

    // Maybe advance the time buffer?  Not sure what to do in the normal
    // case.

    // Clear the skid buffer in case it has any data in it.
    while (!skidBuffer.empty())
    {
        skidBuffer.pop();
    }
}

template<class Impl>
void
SimpleDecode<Impl>::squash()
{
    DPRINTF(Decode, "Decode: Squashing.\n");
    // Set status to squashing.
    _status = Squashing;

    // Maybe advance the time buffer?  Not sure what to do in the normal
    // case.

    // Clear the skid buffer in case it has any data in it.
    while (!skidBuffer.empty())
    {
        skidBuffer.pop();
    }
}

template<class Impl>
void
SimpleDecode<Impl>::tick()
{
    // Decode should try to execute as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (_status != Blocked && _status != Squashing) {
        DPRINTF(Decode, "Decode: Not blocked, so attempting to run "
                        "stage.\n");
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(_status == Unblocking ? !skidBuffer.empty() : 1);

        decode();

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        if (_status == Unblocking) {
            unblock();
        }
    } else if (_status == Blocked) {
        if (fromFetch->insts[0] != NULL) {
            block();
        }

        if (!fromRename->renameInfo.stall &&
                   !fromIEW->iewInfo.stall &&
                   !fromCommit->commitInfo.stall) {
            DPRINTF(Decode, "Decode: Stall signals cleared, going to "
                    "unblock.\n");
            _status = Unblocking;

            // Continue to tell previous stage to block until this
            // stage is done unblocking.
            toFetch->decodeInfo.stall = true;
        } else {
            DPRINTF(Decode, "Decode: Still blocked.\n");
            toFetch->decodeInfo.stall = true;
        }

        if (fromCommit->commitInfo.squash ||
            fromCommit->commitInfo.robSquashing) {
            squash();
        }
    } else if (_status == Squashing) {
        if (!fromCommit->commitInfo.squash &&
            !fromCommit->commitInfo.robSquashing) {
            _status = Running;
        } else if (fromCommit->commitInfo.squash) {
            squash();
        }
    }
}

template<class Impl>
void
SimpleDecode<Impl>::decode()
{
    // Check time buffer if being told to squash.
    if (/* fromRename->renameInfo.squash || */
        /* fromIEW->iewInfo.squash || */
        fromCommit->commitInfo.squash) {
        squash();
        return;
    }

    // Check time buffer if being told to stall.
    if (fromRename->renameInfo.stall ||
        fromIEW->iewInfo.stall ||
        fromCommit->commitInfo.stall)
    {
        block();
        return;
    }

    // Check fetch queue to see if instructions are available.
    // If no available instructions, do nothing, unless this stage is
    // currently unblocking.
    if (fromFetch->insts[0] == NULL && _status != Unblocking) {
        DPRINTF(Decode, "Decode: Nothing to do, breaking out early.\n");
        // Should I change the status to idle?
        return;
    }

    DynInst *inst;
    // Instead have a class member variable that records which instruction
    // was the last one that was ended on.  At the tick() stage, it can
    // check if that's equal to 0.  If not, then don't pop stuff off.
    unsigned num_inst = 0;
    bool insts_available = _status == Unblocking ?
        skidBuffer.front().insts[num_inst] != NULL :
        fromFetch->insts[num_inst] != NULL;

    // Debug block...
#if 0
    if (insts_available) {
        DPRINTF(Decode, "Decode: Instructions available.\n");
    } else {
        if (_status == Unblocking && skidBuffer.empty()) {
            DPRINTF(Decode, "Decode: No instructions available, skid buffer "
                    "empty.\n");
        } else if (_status != Unblocking &&
                   fromFetch->insts[0] == NULL) {
            DPRINTF(Decode, "Decode: No instructions available, fetch queue "
                    "empty.\n");
        } else {
            panic("Decode: No instructions available, unexpected condition!"
                  "\n");
        }
    }
#endif

    // Check to make sure that instructions coming from fetch are valid.
    // Normally at this stage the branch target of PC-relative branches
    // should be computed here.  However in this simple model all
    // computation will take place at execute.  Hence doneTargCalc()
    // will always be false.
     while (num_inst < decodeWidth &&
            insts_available)
     {
        DPRINTF(Decode, "Decode: Sending instruction to rename.\n");
        // Might create some sort of accessor to get an instruction
        // on a per thread basis.  Or might be faster to just get
        // a pointer to an array or list of instructions and use that
        // within this code.
        inst = _status == Unblocking ? skidBuffer.front().insts[num_inst] :
               fromFetch->insts[num_inst];
        DPRINTF(Decode, "Decode: Processing instruction %i with PC %#x\n",
                inst, inst->readPC());

        // This current instruction is valid, so add it into the decode
        // queue.  The next instruction may not be valid, so check to
        // see if branches were predicted correctly.
        toRename->insts[num_inst] = inst;

        // Ensure that if it was predicted as a branch, it really is a
        // branch.  This case should never happen in this model.
        if (inst->predTaken() && !inst->isControl()) {
            panic("Instruction predicted as a branch!");

            // Might want to set some sort of boolean and just do
            // a check at the end
            squash(inst);
            break;
        }

        // Ensure that the predicted branch target is the actual branch
        // target if possible (branches that are PC relative).
        if (inst->isControl() && inst->doneTargCalc()) {
            if (inst->mispredicted()) {
                // Might want to set some sort of boolean and just do
                // a check at the end
                squash(inst);
                break;
            }
        }

        // Also check if instructions have no source registers.  Mark
        // them as ready to issue at any time.  Not sure if this check
        // should exist here or at a later stage; however it doesn't matter
        // too much for function correctness.
        if (inst->numSrcRegs() == 0) {
            inst->setCanIssue();
        }

        // Increment which instruction we're looking at.
        ++num_inst;

        // Check whether or not there are instructions available.
        // Either need to check within the skid buffer, or the fetch
        // queue, depending if this stage is unblocking or not.
        insts_available = _status == Unblocking ?
                           skidBuffer.front().insts[num_inst] == NULL :
                           fromFetch->insts[num_inst] == NULL;
    }
}

#endif // __SIMPLE_DECODE_CC__
