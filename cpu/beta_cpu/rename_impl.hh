#include <list>

#include "cpu/beta_cpu/rename.hh"

template<class Impl>
SimpleRename<Impl>::SimpleRename(Params &params)
    : iewToRenameDelay(params.iewToRenameDelay),
      decodeToRenameDelay(params.decodeToRenameDelay),
      commitToRenameDelay(params.commitToRenameDelay),
      renameWidth(params.renameWidth),
      commitWidth(params.commitWidth)
{
    _status = Idle;
}

template<class Impl>
void
SimpleRename<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Rename, "Rename: Setting CPU pointer.\n");
    cpu = cpu_ptr;
}

template<class Impl>
void
SimpleRename<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(Rename, "Rename: Setting time buffer pointer.\n");
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from IEW stage.
    fromIEW = timeBuffer->getWire(-iewToRenameDelay);

    // Setup wire to read infromation from time buffer, from commit stage.
    fromCommit = timeBuffer->getWire(-commitToRenameDelay);

    // Setup wire to write information to previous stages.
    toDecode = timeBuffer->getWire(0);
}

template<class Impl>
void
SimpleRename<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    DPRINTF(Rename, "Rename: Setting rename queue pointer.\n");
    renameQueue = rq_ptr;

    // Setup wire to write information to future stages.
    toIEW = renameQueue->getWire(0);
}

template<class Impl>
void
SimpleRename<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    DPRINTF(Rename, "Rename: Setting decode queue pointer.\n");
    decodeQueue = dq_ptr;

    // Setup wire to get information from decode.
    fromDecode = decodeQueue->getWire(-decodeToRenameDelay);

}

template<class Impl>
void
SimpleRename<Impl>::setRenameMap(RenameMap *rm_ptr)
{
    DPRINTF(Rename, "Rename: Setting rename map pointer.\n");
    renameMap = rm_ptr;
}

template<class Impl>
void
SimpleRename<Impl>::setFreeList(FreeList *fl_ptr)
{
    DPRINTF(Rename, "Rename: Setting free list pointer.\n");
    freeList = fl_ptr;
}

template<class Impl>
void
SimpleRename<Impl>::dumpHistory()
{
    typename list<RenameHistory>::iterator buf_it = historyBuffer.begin();

    while (buf_it != historyBuffer.end())
    {
        cprintf("Seq num: %i\nArch reg: %i New phys reg: %i Old phys "
                "reg: %i\n", (*buf_it).instSeqNum, (int)(*buf_it).archReg,
                (int)(*buf_it).newPhysReg, (int)(*buf_it).prevPhysReg);

        buf_it++;
    }
}

template<class Impl>
void
SimpleRename<Impl>::block()
{
    DPRINTF(Rename, "Rename: Blocking.\n");
    // Set status to Blocked.
    _status = Blocked;

    // Add the current inputs onto the skid buffer, so they can be
    // reprocessed when this stage unblocks.
    skidBuffer.push(*fromDecode);

    // Note that this stage only signals previous stages to stall when
    // it is the cause of the stall originates at this stage.  Otherwise
    // the previous stages are expected to check all possible stall signals.
}

template<class Impl>
inline void
SimpleRename<Impl>::unblock()
{
    DPRINTF(Rename, "Rename: Reading instructions out of skid "
            "buffer.\n");
    // Remove the now processed instructions from the skid buffer.
    skidBuffer.pop();

    // If there's still information in the skid buffer, then
    // continue to tell previous stages to stall.  They will be
    // able to restart once the skid buffer is empty.
    if (!skidBuffer.empty()) {
                toDecode->renameInfo.stall = true;
    } else {
        DPRINTF(Rename, "Rename: Done unblocking.\n");
        _status = Running;
    }
}

template<class Impl>
void
SimpleRename<Impl>::doSquash()
{
    typename list<RenameHistory>::iterator hb_it = historyBuffer.begin();
    typename list<RenameHistory>::iterator delete_it;

    InstSeqNum squashed_seq_num = fromCommit->commitInfo.doneSeqNum;

#ifdef FULL_SYSTEM
    assert(!historyBuffer.empty());
#else
    // After a syscall squashes everything, the history buffer may be empty
    // but the ROB may still be squashing instructions.
    if (historyBuffer.empty()) {
        return;
    }
#endif // FULL_SYSTEM

    // Go through the most recent instructions, undoing the mappings
    // they did and freeing up the registers.
    while ((*hb_it).instSeqNum > squashed_seq_num)
    {
        DPRINTF(Rename, "Rename: Removing history entry with sequence "
                "number %i.\n", (*hb_it).instSeqNum);

        // If it's not simply a place holder, then add the registers.
        if (!(*hb_it).placeHolder) {
            // Tell the rename map to set the architected register to the
            // previous physical register that it was renamed to.
            renameMap->setEntry(hb_it->archReg, hb_it->prevPhysReg);

            // Put the renamed physical register back on the free list.
            freeList->addReg(hb_it->newPhysReg);
        }

        delete_it = hb_it;

        hb_it++;

        historyBuffer.erase(delete_it);
    }
}

template<class Impl>
void
SimpleRename<Impl>::squash()
{
    DPRINTF(Rename, "Rename: Squashing instructions.\n");
    // Set the status to Squashing.
    _status = Squashing;

    // Clear the skid buffer in case it has any data in it.
    while (!skidBuffer.empty())
    {
        skidBuffer.pop();
    }

    doSquash();
}

// In the future, when a SmartPtr is used for DynInst, then this function
// itself can handle returning the instruction's physical registers to
// the free list.
template<class Impl>
void
SimpleRename<Impl>::removeFromHistory(InstSeqNum inst_seq_num)
{
    DPRINTF(Rename, "Rename: Removing a committed instruction from the "
            "history buffer, sequence number %lli.\n", inst_seq_num);
    typename list<RenameHistory>::iterator hb_it = historyBuffer.end();

    hb_it--;

    if (hb_it->instSeqNum > inst_seq_num) {
        DPRINTF(Rename, "Rename: Old sequence number encountered.  Ensure "
                "that a syscall happened recently.\n");
        return;
    }

    for ( ; hb_it->instSeqNum != inst_seq_num; hb_it--)
    {
        // Make sure we haven't gone off the end of the list.
        assert(hb_it != historyBuffer.end());

        // In theory instructions at the end of the history buffer
        // should be older than the instruction being removed, which
        // means they will have a lower sequence number.  Also the
        // instruction being removed from the history really should
        // be the last instruction in the list, as it is the instruction
        // that was just committed that is being removed.
        assert(hb_it->instSeqNum < inst_seq_num);
        DPRINTF(Rename, "Rename: Committed instruction is not the last "
                "entry in the history buffer.\n");
    }

    if (!(*hb_it).placeHolder) {
        freeList->addReg(hb_it->prevPhysReg);
    }

    historyBuffer.erase(hb_it);

}

template<class Impl>
void
SimpleRename<Impl>::tick()
{
    // Rename will need to try to rename as many instructions as it
    // has bandwidth, unless it is blocked.

    // Check if _status is BarrierStall.  If so, then check if the number
    // of free ROB entries is equal to the number of total ROB entries.
    // Once equal then wake this stage up.  Set status to unblocking maybe.

    if (_status != Blocked && _status != Squashing) {
        DPRINTF(Rename, "Rename: Status is not blocked, will attempt to "
                        "run stage.\n");
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(_status == Unblocking ? !skidBuffer.empty() : 1);

        rename();

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        if (_status == Unblocking) {
            unblock();
        }
    } else if (_status == Blocked) {
        // If stage is blocked and still receiving valid instructions,
        // make sure to store them in the skid buffer.
        if (fromDecode->insts[0] != NULL) {

            block();

            // Continue to tell previous stage to stall.
            toDecode->renameInfo.stall = true;
        }

        if (!fromIEW->iewInfo.stall &&
            !fromCommit->commitInfo.stall &&
            fromCommit->commitInfo.freeROBEntries != 0 &&
            fromIEW->iewInfo.freeIQEntries != 0) {

            // Need to be sure to check all blocking conditions above.
            // If they have cleared, then start unblocking.
            DPRINTF(Rename, "Rename: Stall signals cleared, going to "
                    "unblock.\n");
            _status = Unblocking;

            // Continue to tell previous stage to block until this stage
            // is done unblocking.
            toDecode->renameInfo.stall = true;
        } else {
            // Otherwise no conditions have changed.  Tell previous
            // stage to continue blocking.
            toDecode->renameInfo.stall = true;
        }

        if (fromCommit->commitInfo.squash ||
            fromCommit->commitInfo.robSquashing) {
            squash();
            return;
        }
    } else if (_status == Squashing) {
        if (fromCommit->commitInfo.squash) {
            squash();
        } else if (!fromCommit->commitInfo.squash &&
                   !fromCommit->commitInfo.robSquashing) {

            DPRINTF(Rename, "Rename: Done squashing, going to running.\n");
            _status = Running;
        } else {
            doSquash();
        }
    }

    // Ugly code, revamp all of the tick() functions eventually.
    if (fromCommit->commitInfo.doneSeqNum != 0 && _status != Squashing) {
        removeFromHistory(fromCommit->commitInfo.doneSeqNum);
    }

    // Perhaps put this outside of this function, since this will
    // happen regardless of whether or not the stage is blocked or
    // squashing.
    // Read from the time buffer any necessary data.
    // Read registers that are freed, and add them to the freelist.
    // This is unnecessary due to the history buffer (assuming the history
    // buffer works properly).
/*
    while(!fromCommit->commitInfo.freeRegs.empty())
    {
        PhysRegIndex freed_reg = fromCommit->commitInfo.freeRegs.back();
        DPRINTF(Rename, "Rename: Adding freed register %i to freelist.\n",
                (int)freed_reg);
        freeList->addReg(freed_reg);

        fromCommit->commitInfo.freeRegs.pop_back();
    }
*/

}

template<class Impl>
void
SimpleRename<Impl>::rename()
{
    // Check if any of the stages ahead of rename are telling rename
    // to squash.  The squash() function will also take care of fixing up
    // the rename map and the free list.
    if (fromCommit->commitInfo.squash ||
        fromCommit->commitInfo.robSquashing) {
        squash();
        return;
    }

    // Check if time buffer is telling this stage to stall.
    if (fromIEW->iewInfo.stall ||
        fromCommit->commitInfo.stall) {
        DPRINTF(Rename, "Rename: Receiving signal from IEW/Commit to "
                        "stall.\n");
        block();
        return;
    }

    // Check if the current status is squashing.  If so, set its status
    // to running and resume execution the next cycle.
    if (_status == Squashing) {
        DPRINTF(Rename, "Rename: Done squashing.\n");
        _status = Running;
        return;
    }

    // Check the decode queue to see if instructions are available.
    // If there are no available instructions to rename, then do nothing.
    // Or, if the stage is currently unblocking, then go ahead and run it.
    if (fromDecode->insts[0] == NULL && _status != Unblocking) {
        DPRINTF(Rename, "Rename: Nothing to do, breaking out early.\n");
        // Should I change status to idle?
        return;
    }

    DynInst *inst;
    unsigned num_inst = 0;

    bool insts_available = _status == Unblocking ?
        skidBuffer.front().insts[num_inst] != NULL :
        fromDecode->insts[num_inst] != NULL;

    typename SimpleRenameMap::RenameInfo rename_result;

    unsigned num_src_regs;
    unsigned num_dest_regs;

    // Will have to do a different calculation for the number of free
    // entries.  Number of free entries recorded on this cycle -
    // renameWidth * renameToDecodeDelay
    // Can I avoid a multiply?
    unsigned free_rob_entries =
        fromCommit->commitInfo.freeROBEntries - iewToRenameDelay;
    DPRINTF(Rename, "Rename: ROB has %d free entries.\n",
            free_rob_entries);
    unsigned free_iq_entries =
        fromIEW->iewInfo.freeIQEntries - iewToRenameDelay;

    // Check if there's any space left.
    if (free_rob_entries == 0 || free_iq_entries == 0) {
        DPRINTF(Rename, "Rename: Blocking due to no free ROB or IQ "
                "entries.\n"
                "Rename: ROB has %d free entries.\n"
                "Rename: IQ has %d free entries.\n",
                free_rob_entries,
                free_iq_entries);
        block();
        // Tell previous stage to stall.
        toDecode->renameInfo.stall = true;

        return;
    }

    unsigned min_iq_rob = min(free_rob_entries, free_iq_entries);
    unsigned num_insts_to_rename = min(min_iq_rob, renameWidth);

    while (insts_available &&
           num_inst < num_insts_to_rename) {
        DPRINTF(Rename, "Rename: Sending instructions to iew.\n");

        // Get the next instruction either from the skid buffer or the
        // decode queue.
        inst = _status == Unblocking ? skidBuffer.front().insts[num_inst] :
               fromDecode->insts[num_inst];

        DPRINTF(Rename, "Rename: Processing instruction %i with PC %#x.\n",
                inst, inst->readPC());

        // If it's a trap instruction, then it needs to wait here within
        // rename until the ROB is empty.  Needs a way to detect that the
        // ROB is empty.  Maybe an event?
        // Would be nice if it could be avoided putting this into a
        // specific stage and instead just put it into the AlphaFullCPU.
        // Might not really be feasible though...
        // (EXCB, TRAPB)
        if (inst->isSerializing()) {
            panic("Rename: Serializing instruction encountered.\n");
            DPRINTF(Rename, "Rename: Serializing instruction "
                            "encountered.\n");
            block();

            // Change status over to BarrierStall so that other stages know
            // what this is blocked on.
            _status = BarrierStall;

            // Tell the previous stage to stall.
            toDecode->renameInfo.stall = true;

            break;
        }

        // Make sure there's enough room in the ROB and the IQ.
        // This doesn't really need to be done dynamically; consider
        // moving outside of this function.
        if (free_rob_entries == 0 || free_iq_entries == 0) {
            DPRINTF(Rename, "Rename: Blocking due to lack of ROB or IQ "
                            "entries.\n");
            // Call some sort of function to handle all the setup of being
            // blocked.
            block();

            // Not really sure how to schedule an event properly, but an
            // event must be scheduled such that upon freeing a ROB entry,
            // this stage will restart up.  Perhaps add in a ptr to an Event
            // within the ROB that will be able to execute that Event
            // if a free register is added to the freelist.

            // Tell the previous stage to stall.
            toDecode->renameInfo.stall = true;

            break;
        }

        // Temporary variables to hold number of source and destination regs.
        num_src_regs = inst->numSrcRegs();
        num_dest_regs = inst->numDestRegs();

        // Check here to make sure there are enough destination registers
        // to rename to.  Otherwise block.
        if (renameMap->numFreeEntries() < num_dest_regs)
        {
            DPRINTF(Rename, "Rename: Blocking due to lack of free "
                            "physical registers to rename to.\n");
            // Call function to handle blocking.
            block();

            // Need some sort of event based on a register being freed.

            // Tell the previous stage to stall.
            toDecode->renameInfo.stall = true;

            // Break out of rename loop.
            break;
        }

        // Get the architectual register numbers from the source and
        // destination operands, and redirect them to the right register.
        // Will need to mark dependencies though.
        for (int src_idx = 0; src_idx < num_src_regs; src_idx++)
        {
            RegIndex src_reg = inst->srcRegIdx(src_idx);

            // Look up the source registers to get the phys. register they've
            // been renamed to, and set the sources to those registers.
            RegIndex renamed_reg = renameMap->lookup(src_reg);

            DPRINTF(Rename, "Rename: Looking up arch reg %i, got "
                    "physical reg %i.\n", (int)src_reg, (int)renamed_reg);

            inst->renameSrcReg(src_idx, renamed_reg);

            // Either incorporate it into the info passed back,
            // or make another function call to see if that register is
            // ready or not.
            if (renameMap->isReady(renamed_reg)) {
                DPRINTF(Rename, "Rename: Register is ready.\n");

                inst->markSrcRegReady(src_idx);
            }
        }

        // Rename the destination registers.
        for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++)
        {
            RegIndex dest_reg = inst->destRegIdx(dest_idx);

            // Get the physical register that the destination will be
            // renamed to.
            rename_result = renameMap->rename(dest_reg);

            DPRINTF(Rename, "Rename: Renaming arch reg %i to physical "
                    "register %i.\n", (int)dest_reg,
                    (int)rename_result.first);

            // Record the rename information so that a history can be kept.
            RenameHistory hb_entry(inst->seqNum, dest_reg,
                                   rename_result.first,
                                   rename_result.second);

            historyBuffer.push_front(hb_entry);

            DPRINTF(Rename, "Rename: Adding instruction to history buffer, "
                    "sequence number %lli.\n", inst->seqNum);

            // Tell the instruction to rename the appropriate destination
            // register (dest_idx) to the new physical register
            // (rename_result.first), and record the previous physical
            // register that the same logical register was renamed to
            // (rename_result.second).
            inst->renameDestReg(dest_idx,
                                rename_result.first,
                                rename_result.second);
        }

        // If it's an instruction with no destination registers, then put
        // a placeholder within the history buffer.  It might be better
        // to not put it in the history buffer at all (other than branches,
        // which always need at least a place holder), and differentiate
        // between instructions with and without destination registers
        // when getting from commit the instructions that committed.
        if (num_dest_regs == 0) {
            RenameHistory hb_entry(inst->seqNum);

            historyBuffer.push_front(hb_entry);

            DPRINTF(Rename, "Rename: Adding placeholder instruction to "
                    "history buffer, sequence number %lli.\n",
                    inst->seqNum);
        }

        // Put instruction in rename queue.
        toIEW->insts[num_inst] = inst;

        // Decrease the number of free ROB and IQ entries.
        --free_rob_entries;
        --free_iq_entries;

        // Increment which instruction we're on.
        ++num_inst;

        // Check whether or not there are instructions available.
        // Either need to check within the skid buffer, or the decode
        // queue, depending if this stage is unblocking or not.
        // Hmm, dangerous check.  Can touch memory not allocated.  Might
        // be better to just do check at beginning of loop.  Or better
        // yet actually pass the number of instructions issued.
        insts_available = _status == Unblocking ?
                           skidBuffer.front().insts[num_inst] != NULL :
                           fromDecode->insts[num_inst] != NULL;
    }

}
