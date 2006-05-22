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

#include <list>

#include "config/full_system.hh"
#include "cpu/o3/rename.hh"

template <class Impl>
SimpleRename<Impl>::SimpleRename(Params &params)
    : iewToRenameDelay(params.iewToRenameDelay),
      decodeToRenameDelay(params.decodeToRenameDelay),
      commitToRenameDelay(params.commitToRenameDelay),
      renameWidth(params.renameWidth),
      commitWidth(params.commitWidth),
      numInst(0)
{
    _status = Idle;
}

template <class Impl>
void
SimpleRename<Impl>::regStats()
{
    renameSquashCycles
        .name(name() + ".renameSquashCycles")
        .desc("Number of cycles rename is squashing")
        .prereq(renameSquashCycles);
    renameIdleCycles
        .name(name() + ".renameIdleCycles")
        .desc("Number of cycles rename is idle")
        .prereq(renameIdleCycles);
    renameBlockCycles
        .name(name() + ".renameBlockCycles")
        .desc("Number of cycles rename is blocking")
        .prereq(renameBlockCycles);
    renameUnblockCycles
        .name(name() + ".renameUnblockCycles")
        .desc("Number of cycles rename is unblocking")
        .prereq(renameUnblockCycles);
    renameRenamedInsts
        .name(name() + ".renameRenamedInsts")
        .desc("Number of instructions processed by rename")
        .prereq(renameRenamedInsts);
    renameSquashedInsts
        .name(name() + ".renameSquashedInsts")
        .desc("Number of squashed instructions processed by rename")
        .prereq(renameSquashedInsts);
    renameROBFullEvents
        .name(name() + ".renameROBFullEvents")
        .desc("Number of times rename has considered the ROB 'full'")
        .prereq(renameROBFullEvents);
    renameIQFullEvents
        .name(name() + ".renameIQFullEvents")
        .desc("Number of times rename has considered the IQ 'full'")
        .prereq(renameIQFullEvents);
    renameFullRegistersEvents
        .name(name() + ".renameFullRegisterEvents")
        .desc("Number of times there has been no free registers")
        .prereq(renameFullRegistersEvents);
    renameRenamedOperands
        .name(name() + ".renameRenamedOperands")
        .desc("Number of destination operands rename has renamed")
        .prereq(renameRenamedOperands);
    renameRenameLookups
        .name(name() + ".renameRenameLookups")
        .desc("Number of register rename lookups that rename has made")
        .prereq(renameRenameLookups);
    renameHBPlaceHolders
        .name(name() + ".renameHBPlaceHolders")
        .desc("Number of place holders added to the history buffer")
        .prereq(renameHBPlaceHolders);
    renameCommittedMaps
        .name(name() + ".renameCommittedMaps")
        .desc("Number of HB maps that are committed")
        .prereq(renameCommittedMaps);
    renameUndoneMaps
        .name(name() + ".renameUndoneMaps")
        .desc("Number of HB maps that are undone due to squashing")
        .prereq(renameUndoneMaps);
    renameValidUndoneMaps
        .name(name() + ".renameValidUndoneMaps")
        .desc("Number of HB maps that are undone, and are not place holders")
        .prereq(renameValidUndoneMaps);
}

template <class Impl>
void
SimpleRename<Impl>::setCPU(FullCPU *cpu_ptr)
{
    DPRINTF(Rename, "Rename: Setting CPU pointer.\n");
    cpu = cpu_ptr;
}

template <class Impl>
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

template <class Impl>
void
SimpleRename<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    DPRINTF(Rename, "Rename: Setting rename queue pointer.\n");
    renameQueue = rq_ptr;

    // Setup wire to write information to future stages.
    toIEW = renameQueue->getWire(0);
}

template <class Impl>
void
SimpleRename<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    DPRINTF(Rename, "Rename: Setting decode queue pointer.\n");
    decodeQueue = dq_ptr;

    // Setup wire to get information from decode.
    fromDecode = decodeQueue->getWire(-decodeToRenameDelay);
}

template <class Impl>
void
SimpleRename<Impl>::setRenameMap(RenameMap *rm_ptr)
{
    DPRINTF(Rename, "Rename: Setting rename map pointer.\n");
    renameMap = rm_ptr;
}

template <class Impl>
void
SimpleRename<Impl>::setFreeList(FreeList *fl_ptr)
{
    DPRINTF(Rename, "Rename: Setting free list pointer.\n");
    freeList = fl_ptr;
}

template <class Impl>
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

template <class Impl>
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

template <class Impl>
inline void
SimpleRename<Impl>::unblock()
{
    DPRINTF(Rename, "Rename: Read instructions out of skid buffer this "
            "cycle.\n");
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

template <class Impl>
void
SimpleRename<Impl>::doSquash()
{
    typename list<RenameHistory>::iterator hb_it = historyBuffer.begin();

    InstSeqNum squashed_seq_num = fromCommit->commitInfo.doneSeqNum;

#if FULL_SYSTEM
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
        assert(hb_it != historyBuffer.end());

        DPRINTF(Rename, "Rename: Removing history entry with sequence "
                "number %i.\n", (*hb_it).instSeqNum);

        // If it's not simply a place holder, then add the registers.
        if (!(*hb_it).placeHolder) {
            // Tell the rename map to set the architected register to the
            // previous physical register that it was renamed to.
            renameMap->setEntry(hb_it->archReg, hb_it->prevPhysReg);

            // Put the renamed physical register back on the free list.
            freeList->addReg(hb_it->newPhysReg);

            ++renameValidUndoneMaps;
        }

        historyBuffer.erase(hb_it++);

        ++renameUndoneMaps;
    }
}

template <class Impl>
void
SimpleRename<Impl>::squash()
{
    DPRINTF(Rename, "Rename: Squashing instructions.\n");
    // Set the status to Squashing.
    _status = Squashing;

    numInst = 0;

    // Clear the skid buffer in case it has any data in it.
    while (!skidBuffer.empty())
    {
        skidBuffer.pop();
    }

    doSquash();
}

template<class Impl>
void
SimpleRename<Impl>::removeFromHistory(InstSeqNum inst_seq_num)
{
    DPRINTF(Rename, "Rename: Removing a committed instruction from the "
            "history buffer, until sequence number %lli.\n", inst_seq_num);
    typename list<RenameHistory>::iterator hb_it = historyBuffer.end();

    --hb_it;

    if (hb_it->instSeqNum > inst_seq_num) {
        DPRINTF(Rename, "Rename: Old sequence number encountered.  Ensure "
                "that a syscall happened recently.\n");
        return;
    }

    while ((*hb_it).instSeqNum != inst_seq_num)
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
        DPRINTF(Rename, "Rename: Freeing up older rename of reg %i, sequence"
                " number %i.\n",
                (*hb_it).prevPhysReg, (*hb_it).instSeqNum);

        if (!(*hb_it).placeHolder) {
            freeList->addReg((*hb_it).prevPhysReg);
            ++renameCommittedMaps;
        }

        historyBuffer.erase(hb_it--);
    }

    // Finally free up the previous register of the finished instruction
    // itself.
    if (!(*hb_it).placeHolder) {
        freeList->addReg(hb_it->prevPhysReg);
        ++renameCommittedMaps;
    }

    historyBuffer.erase(hb_it);
}

template <class Impl>
inline void
SimpleRename<Impl>::renameSrcRegs(DynInstPtr &inst)
{
    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // destination operands, and redirect them to the right register.
    // Will need to mark dependencies though.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++)
    {
        RegIndex src_reg = inst->srcRegIdx(src_idx);

        // Look up the source registers to get the phys. register they've
        // been renamed to, and set the sources to those registers.
        PhysRegIndex renamed_reg = renameMap->lookup(src_reg);

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

        ++renameRenameLookups;
    }
}

template <class Impl>
inline void
SimpleRename<Impl>::renameDestRegs(DynInstPtr &inst)
{
    typename SimpleRenameMap::RenameInfo rename_result;

    unsigned num_dest_regs = inst->numDestRegs();

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

        ++renameHBPlaceHolders;
    } else {

        // Rename the destination registers.
        for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++)
        {
            RegIndex dest_reg = inst->destRegIdx(dest_idx);

            // Get the physical register that the destination will be
            // renamed to.
            rename_result = renameMap->rename(dest_reg);

            DPRINTF(Rename, "Rename: Renaming arch reg %i to physical "
                    "reg %i.\n", (int)dest_reg,
                    (int)rename_result.first);

            // Record the rename information so that a history can be kept.
            RenameHistory hb_entry(inst->seqNum, dest_reg,
                                   rename_result.first,
                                   rename_result.second);

            historyBuffer.push_front(hb_entry);

            DPRINTF(Rename, "Rename: Adding instruction to history buffer, "
                    "sequence number %lli.\n",
                    (*historyBuffer.begin()).instSeqNum);

            // Tell the instruction to rename the appropriate destination
            // register (dest_idx) to the new physical register
            // (rename_result.first), and record the previous physical
            // register that the same logical register was renamed to
            // (rename_result.second).
            inst->renameDestReg(dest_idx,
                                rename_result.first,
                                rename_result.second);

            ++renameRenamedOperands;
        }
    }
}

template <class Impl>
inline int
SimpleRename<Impl>::calcFreeROBEntries()
{
    return fromCommit->commitInfo.freeROBEntries -
        renameWidth * iewToRenameDelay;
}

template <class Impl>
inline int
SimpleRename<Impl>::calcFreeIQEntries()
{
    return fromIEW->iewInfo.freeIQEntries - renameWidth * iewToRenameDelay;
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
            ++renameUnblockCycles;

            if (fromDecode->size > 0) {
                // Add the current inputs onto the skid buffer, so they can be
                // reprocessed when this stage unblocks.
                skidBuffer.push(*fromDecode);
            }

            unblock();
        }
    } else if (_status == Blocked) {
        ++renameBlockCycles;

        // If stage is blocked and still receiving valid instructions,
        // make sure to store them in the skid buffer.
        if (fromDecode->size > 0) {

            block();

            // Continue to tell previous stage to stall.
            toDecode->renameInfo.stall = true;
        }

        if (!fromIEW->iewInfo.stall &&
            !fromCommit->commitInfo.stall &&
            calcFreeROBEntries() > 0 &&
            calcFreeIQEntries() > 0 &&
            renameMap->numFreeEntries() > 0) {

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
        ++renameSquashCycles;

        if (fromCommit->commitInfo.squash) {
            squash();
        } else if (!fromCommit->commitInfo.squash &&
                   !fromCommit->commitInfo.robSquashing) {

            DPRINTF(Rename, "Rename: Done squashing, going to running.\n");
            _status = Running;
            rename();
        } else {
            doSquash();
        }
    }

    // Ugly code, revamp all of the tick() functions eventually.
    if (fromCommit->commitInfo.doneSeqNum != 0 && _status != Squashing) {
#if !FULL_SYSTEM
        if (!fromCommit->commitInfo.squash) {
            removeFromHistory(fromCommit->commitInfo.doneSeqNum);
        }
#else
        removeFromHistory(fromCommit->commitInfo.doneSeqNum);
#endif
    }

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
        DPRINTF(Rename, "Rename: Receiving signal from Commit to squash.\n");
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
    if (fromDecode->size == 0 && _status != Unblocking) {
        DPRINTF(Rename, "Rename: Nothing to do, breaking out early.\n");
        // Should I change status to idle?
        return;
    }

    ////////////////////////////////////
    // Actual rename part.
    ////////////////////////////////////

    DynInstPtr inst;

    // If we're unblocking, then we may be in the middle of an instruction
    // group.  Subtract off numInst to get the proper number of instructions
    // left.
    int insts_available = _status == Unblocking ?
        skidBuffer.front().size - numInst :
        fromDecode->size;

    bool block_this_cycle = false;

    // Will have to do a different calculation for the number of free
    // entries.  Number of free entries recorded on this cycle -
    // renameWidth * renameToDecodeDelay
    int free_rob_entries = calcFreeROBEntries();
    int free_iq_entries = calcFreeIQEntries();
    int min_iq_rob = min(free_rob_entries, free_iq_entries);

    unsigned to_iew_index = 0;

    // Check if there's any space left.
    if (min_iq_rob <= 0) {
        DPRINTF(Rename, "Rename: Blocking due to no free ROB or IQ "
                "entries.\n"
                "Rename: ROB has %d free entries.\n"
                "Rename: IQ has %d free entries.\n",
                free_rob_entries,
                free_iq_entries);
        block();
        // Tell previous stage to stall.
        toDecode->renameInfo.stall = true;

        if (free_rob_entries <= 0) {
            ++renameROBFullEvents;
        } else {
            ++renameIQFullEvents;
        }

        return;
    } else if (min_iq_rob < insts_available) {
        DPRINTF(Rename, "Rename: Will have to block this cycle.  Only "
                "%i insts can be renamed due to IQ/ROB limits.\n",
                min_iq_rob);

        insts_available = min_iq_rob;

        block_this_cycle = true;

        if (free_rob_entries < free_iq_entries) {
            ++renameROBFullEvents;
        } else {
            ++renameIQFullEvents;
        }
    }

    while (insts_available > 0) {
        DPRINTF(Rename, "Rename: Sending instructions to iew.\n");

        // Get the next instruction either from the skid buffer or the
        // decode queue.
        inst = _status == Unblocking ? skidBuffer.front().insts[numInst] :
               fromDecode->insts[numInst];

        if (inst->isSquashed()) {
            DPRINTF(Rename, "Rename: instruction %i with PC %#x is "
                    "squashed, skipping.\n",
                    inst->seqNum, inst->readPC());

            // Go to the next instruction.
            ++numInst;

            ++renameSquashedInsts;

            // Decrement how many instructions are available.
            --insts_available;

            continue;
        }

        DPRINTF(Rename, "Rename: Processing instruction %i with PC %#x.\n",
                inst->seqNum, inst->readPC());

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

            // Change status over to BarrierStall so that other stages know
            // what this is blocked on.
            _status = BarrierStall;

            block_this_cycle = true;

            break;
        }

        // Check here to make sure there are enough destination registers
        // to rename to.  Otherwise block.
        if (renameMap->numFreeEntries() < inst->numDestRegs())
        {
            DPRINTF(Rename, "Rename: Blocking due to lack of free "
                            "physical registers to rename to.\n");
            // Need some sort of event based on a register being freed.

            block_this_cycle = true;

            ++renameFullRegistersEvents;

            break;
        }

        renameSrcRegs(inst);

        renameDestRegs(inst);

        // Put instruction in rename queue.
        toIEW->insts[to_iew_index] = inst;
        ++(toIEW->size);

        // Decrease the number of free ROB and IQ entries.
        --free_rob_entries;
        --free_iq_entries;

        // Increment which instruction we're on.
        ++to_iew_index;
        ++numInst;

        ++renameRenamedInsts;

        // Decrement how many instructions are available.
        --insts_available;
    }

    // Check if there's any instructions left that haven't yet been renamed.
    // If so then block.
    if (block_this_cycle) {
        block();

        toDecode->renameInfo.stall = true;
    } else {
        // If we had a successful rename and didn't have to exit early, then
        // reset numInst so it will refer to the correct instruction on next
        // run.
        numInst = 0;
    }
}
