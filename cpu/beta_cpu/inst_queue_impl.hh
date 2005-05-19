// Todo:
// Current ordering allows for 0 cycle added-to-scheduled.  Could maybe fake
// it; either do in reverse order, or have added instructions put into a
// different ready queue that, in scheduleRreadyInsts(), gets put onto the
// normal ready queue.  This would however give only a one cycle delay,
// but probably is more flexible to actually add in a delay parameter than
// just running it backwards.

#include <vector>

#include "sim/universe.hh"
#include "cpu/beta_cpu/inst_queue.hh"

// Either compile error or max int due to sign extension.
// Blatant hack to avoid compile warnings.
const InstSeqNum MaxInstSeqNum = 0 - 1;

template <class Impl>
InstructionQueue<Impl>::InstructionQueue(Params &params)
    : memDepUnit(params),
      numEntries(params.numIQEntries),
      intWidth(params.executeIntWidth),
      floatWidth(params.executeFloatWidth),
      branchWidth(params.executeBranchWidth),
      memoryWidth(params.executeMemoryWidth),
      totalWidth(params.issueWidth),
      numPhysIntRegs(params.numPhysIntRegs),
      numPhysFloatRegs(params.numPhysFloatRegs),
      commitToIEWDelay(params.commitToIEWDelay)
{
    // Initialize the number of free IQ entries.
    freeEntries = numEntries;

    // Set the number of physical registers as the number of int + float
    numPhysRegs = numPhysIntRegs + numPhysFloatRegs;

    DPRINTF(IQ, "IQ: There are %i physical registers.\n", numPhysRegs);

    //Create an entry for each physical register within the
    //dependency graph.
    dependGraph = new DependencyEntry[numPhysRegs];

    // Resize the register scoreboard.
    regScoreboard.resize(numPhysRegs);

    // Initialize all the head pointers to point to NULL, and all the
    // entries as unready.
    // Note that in actuality, the registers corresponding to the logical
    // registers start off as ready.  However this doesn't matter for the
    // IQ as the instruction should have been correctly told if those
    // registers are ready in rename.  Thus it can all be initialized as
    // unready.
    for (int i = 0; i < numPhysRegs; ++i)
    {
        dependGraph[i].next = NULL;
        dependGraph[i].inst = NULL;
        regScoreboard[i] = false;
    }

}

template <class Impl>
void
InstructionQueue<Impl>::regStats()
{
    iqInstsAdded
        .name(name() + ".iqInstsAdded")
        .desc("Number of instructions added to the IQ (excludes non-spec)")
        .prereq(iqInstsAdded);

    iqNonSpecInstsAdded
        .name(name() + ".iqNonSpecInstsAdded")
        .desc("Number of non-speculative instructions added to the IQ")
        .prereq(iqNonSpecInstsAdded);

//    iqIntInstsAdded;

    iqIntInstsIssued
        .name(name() + ".iqIntInstsIssued")
        .desc("Number of integer instructions issued")
        .prereq(iqIntInstsIssued);

//    iqFloatInstsAdded;

    iqFloatInstsIssued
        .name(name() + ".iqFloatInstsIssued")
        .desc("Number of float instructions issued")
        .prereq(iqFloatInstsIssued);

//    iqBranchInstsAdded;

    iqBranchInstsIssued
        .name(name() + ".iqBranchInstsIssued")
        .desc("Number of branch instructions issued")
        .prereq(iqBranchInstsIssued);

//    iqMemInstsAdded;

    iqMemInstsIssued
        .name(name() + ".iqMemInstsIssued")
        .desc("Number of memory instructions issued")
        .prereq(iqMemInstsIssued);

//    iqMiscInstsAdded;

    iqMiscInstsIssued
        .name(name() + ".iqMiscInstsIssued")
        .desc("Number of miscellaneous instructions issued")
        .prereq(iqMiscInstsIssued);

    iqSquashedInstsIssued
        .name(name() + ".iqSquashedInstsIssued")
        .desc("Number of squashed instructions issued")
        .prereq(iqSquashedInstsIssued);

    iqLoopSquashStalls
        .name(name() + ".iqLoopSquashStalls")
        .desc("Number of times issue loop had to restart due to squashed "
              "inst; mainly for profiling")
        .prereq(iqLoopSquashStalls);

    iqSquashedInstsExamined
        .name(name() + ".iqSquashedInstsExamined")
        .desc("Number of squashed instructions iterated over during squash;"
              " mainly for profiling")
        .prereq(iqSquashedInstsExamined);

    iqSquashedOperandsExamined
        .name(name() + ".iqSquashedOperandsExamined")
        .desc("Number of squashed operands that are examined and possibly "
              "removed from graph")
        .prereq(iqSquashedOperandsExamined);

    iqSquashedNonSpecRemoved
        .name(name() + ".iqSquashedNonSpecRemoved")
        .desc("Number of squashed non-spec instructions that were removed")
        .prereq(iqSquashedNonSpecRemoved);

    // Tell mem dependence unit to reg stats as well.
    memDepUnit.regStats();
}

template <class Impl>
void
InstructionQueue<Impl>::setCPU(FullCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    tail = cpu->instList.begin();
}

template <class Impl>
void
InstructionQueue<Impl>::setIssueToExecuteQueue(
                        TimeBuffer<IssueStruct> *i2e_ptr)
{
    DPRINTF(IQ, "IQ: Set the issue to execute queue.\n");
    issueToExecuteQueue = i2e_ptr;
}

template <class Impl>
void
InstructionQueue<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(IQ, "IQ: Set the time buffer.\n");
    timeBuffer = tb_ptr;

    fromCommit = timeBuffer->getWire(-commitToIEWDelay);
}

template <class Impl>
unsigned
InstructionQueue<Impl>::numFreeEntries()
{
    return freeEntries;
}

// Might want to do something more complex if it knows how many instructions
// will be issued this cycle.
template <class Impl>
bool
InstructionQueue<Impl>::isFull()
{
    if (freeEntries == 0) {
        return(true);
    } else {
        return(false);
    }
}

template <class Impl>
void
InstructionQueue<Impl>::insert(DynInstPtr &new_inst)
{
    // Make sure the instruction is valid
    assert(new_inst);

    DPRINTF(IQ, "IQ: Adding instruction PC %#x to the IQ.\n",
            new_inst->readPC());

    // Check if there are any free entries.  Panic if there are none.
    // Might want to have this return a fault in the future instead of
    // panicing.
    assert(freeEntries != 0);

    // If the IQ currently has nothing in it, then there's a possibility
    // that the tail iterator is invalid (might have been pointing at an
    // instruction that was retired).  Reset the tail iterator.
    if (freeEntries == numEntries) {
        tail = cpu->instList.begin();
    }

    // Move the tail iterator.  Instructions may not have been issued
    // to the IQ, so we may have to increment the iterator more than once.
    while ((*tail) != new_inst) {
        tail++;

        // Make sure the tail iterator points at something legal.
        assert(tail != cpu->instList.end());
    }


    // Decrease the number of free entries.
    --freeEntries;

    // Look through its source registers (physical regs), and mark any
    // dependencies.
    addToDependents(new_inst);

    // Have this instruction set itself as the producer of its destination
    // register(s).
    createDependency(new_inst);

    // If it's a memory instruction, add it to the memory dependency
    // unit.
    if (new_inst->isMemRef()) {
        memDepUnit.insert(new_inst);
        // Uh..forgot to look it up and put it on the proper dependency list
        // if the instruction should not go yet.
    } else {
        // If the instruction is ready then add it to the ready list.
        addIfReady(new_inst);
    }

    ++iqInstsAdded;

    assert(freeEntries == (numEntries - countInsts()));
}

template <class Impl>
void
InstructionQueue<Impl>::insertNonSpec(DynInstPtr &inst)
{
    nonSpecInsts[inst->seqNum] = inst;

    // @todo: Clean up this code; can do it by setting inst as unable
    // to issue, then calling normal insert on the inst.

    // Make sure the instruction is valid
    assert(inst);

    DPRINTF(IQ, "IQ: Adding instruction PC %#x to the IQ.\n",
            inst->readPC());

    // Check if there are any free entries.  Panic if there are none.
    // Might want to have this return a fault in the future instead of
    // panicing.
    assert(freeEntries != 0);

    // If the IQ currently has nothing in it, then there's a possibility
    // that the tail iterator is invalid (might have been pointing at an
    // instruction that was retired).  Reset the tail iterator.
    if (freeEntries == numEntries) {
        tail = cpu->instList.begin();
    }

    // Move the tail iterator.  Instructions may not have been issued
    // to the IQ, so we may have to increment the iterator more than once.
    while ((*tail) != inst) {
        tail++;

        // Make sure the tail iterator points at something legal.
        assert(tail != cpu->instList.end());
    }

    // Decrease the number of free entries.
    --freeEntries;

    // Have this instruction set itself as the producer of its destination
    // register(s).
    createDependency(inst);

    // If it's a memory instruction, add it to the memory dependency
    // unit.
    if (inst->isMemRef()) {
        memDepUnit.insertNonSpec(inst);
    }

    ++iqNonSpecInstsAdded;
}

// Slightly hack function to advance the tail iterator in the case that
// the IEW stage issues an instruction that is not added to the IQ.  This
// is needed in case a long chain of such instructions occurs.
// I don't think this is used anymore.
template <class Impl>
void
InstructionQueue<Impl>::advanceTail(DynInstPtr &inst)
{
    // Make sure the instruction is valid
    assert(inst);

    DPRINTF(IQ, "IQ: Adding instruction PC %#x to the IQ.\n",
            inst->readPC());

    // Check if there are any free entries.  Panic if there are none.
    // Might want to have this return a fault in the future instead of
    // panicing.
    assert(freeEntries != 0);

    // If the IQ currently has nothing in it, then there's a possibility
    // that the tail iterator is invalid (might have been pointing at an
    // instruction that was retired).  Reset the tail iterator.
    if (freeEntries == numEntries) {
        tail = cpu->instList.begin();
    }

    // Move the tail iterator.  Instructions may not have been issued
    // to the IQ, so we may have to increment the iterator more than once.
    while ((*tail) != inst) {
        tail++;

        // Make sure the tail iterator points at something legal.
        assert(tail != cpu->instList.end());
    }

    assert(freeEntries <= numEntries);

    // Have this instruction set itself as the producer of its destination
    // register(s).
    createDependency(inst);
}

// Need to make sure the number of float and integer instructions
// issued does not exceed the total issue bandwidth.
// @todo: Figure out a better way to remove the squashed items from the
// lists.  Checking the top item of each list to see if it's squashed
// wastes time and forces jumps.
template <class Impl>
void
InstructionQueue<Impl>::scheduleReadyInsts()
{
    DPRINTF(IQ, "IQ: Attempting to schedule ready instructions from "
                "the IQ.\n");

    int int_issued = 0;
    int float_issued = 0;
    int branch_issued = 0;
    int memory_issued = 0;
    int squashed_issued = 0;
    int total_issued = 0;

    IssueStruct *i2e_info = issueToExecuteQueue->access(0);

    bool insts_available = !readyBranchInsts.empty() ||
        !readyIntInsts.empty() ||
        !readyFloatInsts.empty() ||
        !memDepUnit.empty() ||
        !readyMiscInsts.empty() ||
        !squashedInsts.empty();

    // Note: Requires a globally defined constant.
    InstSeqNum oldest_inst = MaxInstSeqNum;
    InstList list_with_oldest = None;

    // Temporary values.
    DynInstPtr int_head_inst;
    DynInstPtr float_head_inst;
    DynInstPtr branch_head_inst;
    DynInstPtr mem_head_inst;
    DynInstPtr misc_head_inst;
    DynInstPtr squashed_head_inst;

    // Somewhat nasty code to look at all of the lists where issuable
    // instructions are located, and choose the oldest instruction among
    // those lists.  Consider a rewrite in the future.
    while (insts_available && total_issued < totalWidth)
    {
        // Set this to false.  Each if-block is required to set it to true
        // if there were instructions available this check.  This will cause
        // this loop to run once more than necessary, but avoids extra calls.
        insts_available = false;

        oldest_inst = MaxInstSeqNum;

        list_with_oldest = None;

        if (!readyIntInsts.empty() &&
            int_issued < intWidth) {

            insts_available = true;

            int_head_inst = readyIntInsts.top();

            if (int_head_inst->isSquashed()) {
                readyIntInsts.pop();

                ++iqLoopSquashStalls;

                continue;
            }

            oldest_inst = int_head_inst->seqNum;

            list_with_oldest = Int;
        }

        if (!readyFloatInsts.empty() &&
            float_issued < floatWidth) {

            insts_available = true;

            float_head_inst = readyFloatInsts.top();

            if (float_head_inst->isSquashed()) {
                readyFloatInsts.pop();

                ++iqLoopSquashStalls;

                continue;
            } else if (float_head_inst->seqNum < oldest_inst) {
                oldest_inst = float_head_inst->seqNum;

                list_with_oldest = Float;
            }
        }

        if (!readyBranchInsts.empty() &&
            branch_issued < branchWidth) {

            insts_available = true;

            branch_head_inst = readyBranchInsts.top();

            if (branch_head_inst->isSquashed()) {
                readyBranchInsts.pop();

                ++iqLoopSquashStalls;

                continue;
            } else if (branch_head_inst->seqNum < oldest_inst) {
                oldest_inst = branch_head_inst->seqNum;

                list_with_oldest = Branch;
            }

        }

        if (!memDepUnit.empty() &&
            memory_issued < memoryWidth) {

            insts_available = true;

            mem_head_inst = memDepUnit.top();

            if (mem_head_inst->isSquashed()) {
                memDepUnit.pop();

                ++iqLoopSquashStalls;

                continue;
            } else if (mem_head_inst->seqNum < oldest_inst) {
                oldest_inst = mem_head_inst->seqNum;

                list_with_oldest = Memory;
            }
        }

        if (!readyMiscInsts.empty()) {

            insts_available = true;

            misc_head_inst = readyMiscInsts.top();

            if (misc_head_inst->isSquashed()) {
                readyMiscInsts.pop();

                ++iqLoopSquashStalls;

                continue;
            } else if (misc_head_inst->seqNum < oldest_inst) {
                oldest_inst = misc_head_inst->seqNum;

                list_with_oldest = Misc;
            }
        }

        if (!squashedInsts.empty()) {

            insts_available = true;

            squashed_head_inst = squashedInsts.top();

            if (squashed_head_inst->seqNum < oldest_inst) {
                list_with_oldest = Squashed;
            }

        }

        DynInstPtr issuing_inst = NULL;

        switch (list_with_oldest) {
          case None:
            DPRINTF(IQ, "IQ: Not able to schedule any instructions. Issuing "
                    "inst is %#x.\n", issuing_inst);
            break;

          case Int:
            issuing_inst = int_head_inst;
            readyIntInsts.pop();
            ++int_issued;
            DPRINTF(IQ, "IQ: Issuing integer instruction PC %#x.\n",
                    issuing_inst->readPC());
            break;

          case Float:
            issuing_inst = float_head_inst;
            readyFloatInsts.pop();
            ++float_issued;
            DPRINTF(IQ, "IQ: Issuing float instruction PC %#x.\n",
                    issuing_inst->readPC());
            break;

          case Branch:
            issuing_inst = branch_head_inst;
            readyBranchInsts.pop();
            ++branch_issued;
            DPRINTF(IQ, "IQ: Issuing branch instruction PC %#x.\n",
                    issuing_inst->readPC());
            break;

          case Memory:
            issuing_inst = mem_head_inst;

            memDepUnit.pop();
            ++memory_issued;
            DPRINTF(IQ, "IQ: Issuing memory instruction PC %#x.\n",
                    issuing_inst->readPC());
            break;

          case Misc:
            issuing_inst = misc_head_inst;
            readyMiscInsts.pop();

            ++iqMiscInstsIssued;

            DPRINTF(IQ, "IQ: Issuing a miscellaneous instruction PC %#x.\n",
                    issuing_inst->readPC());
            break;

          case Squashed:
            assert(0 && "Squashed insts should not issue any more!");
            squashedInsts.pop();
            // Set the squashed instruction as able to commit so that commit
            // can just drop it from the ROB.  This is a bit faked.
            ++squashed_issued;
            ++freeEntries;

            DPRINTF(IQ, "IQ: Issuing squashed instruction PC %#x.\n",
                    squashed_head_inst->readPC());
            break;
        }

        if (list_with_oldest != None && list_with_oldest != Squashed) {
            i2e_info->insts[total_issued] = issuing_inst;
            i2e_info->size++;

            issuing_inst->setIssued();

            ++freeEntries;
            ++total_issued;
        }

        assert(freeEntries == (numEntries - countInsts()));
    }

    iqIntInstsIssued += int_issued;
    iqFloatInstsIssued += float_issued;
    iqBranchInstsIssued += branch_issued;
    iqMemInstsIssued += memory_issued;
    iqSquashedInstsIssued += squashed_issued;
}

template <class Impl>
void
InstructionQueue<Impl>::scheduleNonSpec(const InstSeqNum &inst)
{
    DPRINTF(IQ, "IQ: Marking nonspeculative instruction with sequence "
            "number %i as ready to execute.\n", inst);

    non_spec_it_t inst_it = nonSpecInsts.find(inst);

    assert(inst_it != nonSpecInsts.end());

    // Mark this instruction as ready to issue.
    (*inst_it).second->setCanIssue();

    // Now schedule the instruction.
    if (!(*inst_it).second->isMemRef()) {
        addIfReady((*inst_it).second);
    } else {
        memDepUnit.nonSpecInstReady((*inst_it).second);
    }

    nonSpecInsts.erase(inst_it);
}

template <class Impl>
void
InstructionQueue<Impl>::wakeDependents(DynInstPtr &completed_inst)
{
    DPRINTF(IQ, "IQ: Waking dependents of completed instruction.\n");
    //Look at the physical destination register of the DynInst
    //and look it up on the dependency graph.  Then mark as ready
    //any instructions within the instruction queue.
    DependencyEntry *curr;

    // Tell the memory dependence unit to wake any dependents on this
    // instruction if it is a memory instruction.

    if (completed_inst->isMemRef()) {
        memDepUnit.wakeDependents(completed_inst);
    }

    for (int dest_reg_idx = 0;
         dest_reg_idx < completed_inst->numDestRegs();
         dest_reg_idx++)
    {
        PhysRegIndex dest_reg =
            completed_inst->renamedDestRegIdx(dest_reg_idx);

        // Special case of uniq or control registers.  They are not
        // handled by the IQ and thus have no dependency graph entry.
        // @todo Figure out a cleaner way to handle this.
        if (dest_reg >= numPhysRegs) {
            continue;
        }

        DPRINTF(IQ, "IQ: Waking any dependents on register %i.\n",
                (int) dest_reg);

        //Maybe abstract this part into a function.
        //Go through the dependency chain, marking the registers as ready
        //within the waiting instructions.
        while (dependGraph[dest_reg].next) {

            curr = dependGraph[dest_reg].next;

            DPRINTF(IQ, "IQ: Waking up a dependent instruction, PC%#x.\n",
                    curr->inst->readPC());

            // Might want to give more information to the instruction
            // so that it knows which of its source registers is ready.
            // However that would mean that the dependency graph entries
            // would need to hold the src_reg_idx.
            curr->inst->markSrcRegReady();

            addIfReady(curr->inst);

            dependGraph[dest_reg].next = curr->next;

            DependencyEntry::mem_alloc_counter--;

            curr->inst = NULL;

            delete curr;
        }

        // Reset the head node now that all of its dependents have been woken
        // up.
        dependGraph[dest_reg].next = NULL;
        dependGraph[dest_reg].inst = NULL;

        // Mark the scoreboard as having that register ready.
        regScoreboard[dest_reg] = true;
    }
}

template <class Impl>
void
InstructionQueue<Impl>::violation(DynInstPtr &store,
                                  DynInstPtr &faulting_load)
{
    memDepUnit.violation(store, faulting_load);
}

template <class Impl>
void
InstructionQueue<Impl>::squash()
{
    DPRINTF(IQ, "IQ: Starting to squash instructions in the IQ.\n");

    // Read instruction sequence number of last instruction out of the
    // time buffer.
    squashedSeqNum = fromCommit->commitInfo.doneSeqNum;

    // Setup the squash iterator to point to the tail.
    squashIt = tail;

    // Call doSquash if there are insts in the IQ
    if (freeEntries != numEntries) {
        doSquash();
    }

    // Also tell the memory dependence unit to squash.
    memDepUnit.squash(squashedSeqNum);
}

template <class Impl>
void
InstructionQueue<Impl>::doSquash()
{
    // Make sure the squash iterator isn't pointing to nothing.
    assert(squashIt != cpu->instList.end());
    // Make sure the squashed sequence number is valid.
    assert(squashedSeqNum != 0);

    DPRINTF(IQ, "IQ: Squashing instructions in the IQ.\n");

    // Squash any instructions younger than the squashed sequence number
    // given.
    while ((*squashIt)->seqNum > squashedSeqNum) {
        DynInstPtr squashed_inst = (*squashIt);

        // Only handle the instruction if it actually is in the IQ and
        // hasn't already been squashed in the IQ.
        if (!squashed_inst->isIssued() &&
            !squashed_inst->isSquashedInIQ()) {

            // Remove the instruction from the dependency list.
            // Hack for now: These below don't add themselves to the
            // dependency list, so don't try to remove them.
            if (!squashed_inst->isNonSpeculative()/* &&
                                                     !squashed_inst->isStore()*/
                ) {

                for (int src_reg_idx = 0;
                     src_reg_idx < squashed_inst->numSrcRegs();
                     src_reg_idx++)
                {
                    PhysRegIndex src_reg =
                        squashed_inst->renamedSrcRegIdx(src_reg_idx);

                    // Only remove it from the dependency graph if it was
                    // placed there in the first place.
                    // HACK: This assumes that instructions woken up from the
                    // dependency chain aren't informed that a specific src
                    // register has become ready.  This may not always be true
                    // in the future.
                    if (!squashed_inst->isReadySrcRegIdx(src_reg_idx) &&
                        src_reg < numPhysRegs) {
                        dependGraph[src_reg].remove(squashed_inst);
                    }

                    ++iqSquashedOperandsExamined;
                }

                // Might want to remove producers as well.
            } else {
                nonSpecInsts[squashed_inst->seqNum] = NULL;

                nonSpecInsts.erase(squashed_inst->seqNum);

                ++iqSquashedNonSpecRemoved;
            }

            // Might want to also clear out the head of the dependency graph.

            // Mark it as squashed within the IQ.
            squashed_inst->setSquashedInIQ();

//            squashedInsts.push(squashed_inst);
            squashed_inst->setIssued();
            squashed_inst->setCanCommit();

            ++freeEntries;

            DPRINTF(IQ, "IQ: Instruction PC %#x squashed.\n",
                    squashed_inst->readPC());
        }

        --squashIt;
        ++iqSquashedInstsExamined;
    }

    assert(freeEntries <= numEntries);

    if (freeEntries == numEntries) {
        tail = cpu->instList.end();
    }

}

template <class Impl>
void
InstructionQueue<Impl>::stopSquash()
{
    // Clear up the squash variables to ensure that squashing doesn't
    // get called improperly.
    squashedSeqNum = 0;

    squashIt = cpu->instList.end();
}

template <class Impl>
void
InstructionQueue<Impl>::DependencyEntry::insert(DynInstPtr &new_inst)
{
    //Add this new, dependent instruction at the head of the dependency
    //chain.

    // First create the entry that will be added to the head of the
    // dependency chain.
    DependencyEntry *new_entry = new DependencyEntry;
    new_entry->next = this->next;
    new_entry->inst = new_inst;

    // Then actually add it to the chain.
    this->next = new_entry;

    ++mem_alloc_counter;
}

template <class Impl>
void
InstructionQueue<Impl>::DependencyEntry::remove(DynInstPtr &inst_to_remove)
{
    DependencyEntry *prev = this;
    DependencyEntry *curr = this->next;

    // Make sure curr isn't NULL.  Because this instruction is being
    // removed from a dependency list, it must have been placed there at
    // an earlier time.  The dependency chain should not be empty,
    // unless the instruction dependent upon it is already ready.
    if (curr == NULL) {
        return;
    }

    // Find the instruction to remove within the dependency linked list.
    while(curr->inst != inst_to_remove)
    {
        prev = curr;
        curr = curr->next;

        assert(curr != NULL);
    }

    // Now remove this instruction from the list.
    prev->next = curr->next;

    --mem_alloc_counter;

    // Could push this off to the destructor of DependencyEntry
    curr->inst = NULL;

    delete curr;
}

template <class Impl>
bool
InstructionQueue<Impl>::addToDependents(DynInstPtr &new_inst)
{
    // Loop through the instruction's source registers, adding
    // them to the dependency list if they are not ready.
    int8_t total_src_regs = new_inst->numSrcRegs();
    bool return_val = false;

    for (int src_reg_idx = 0;
         src_reg_idx < total_src_regs;
         src_reg_idx++)
    {
        // Only add it to the dependency graph if it's not ready.
        if (!new_inst->isReadySrcRegIdx(src_reg_idx)) {
            PhysRegIndex src_reg = new_inst->renamedSrcRegIdx(src_reg_idx);

            // Check the IQ's scoreboard to make sure the register
            // hasn't become ready while the instruction was in flight
            // between stages.  Only if it really isn't ready should
            // it be added to the dependency graph.
            if (src_reg >= numPhysRegs) {
                continue;
            } else if (regScoreboard[src_reg] == false) {
                DPRINTF(IQ, "IQ: Instruction PC %#x has src reg %i that "
                        "is being added to the dependency chain.\n",
                        new_inst->readPC(), src_reg);

                dependGraph[src_reg].insert(new_inst);

                // Change the return value to indicate that something
                // was added to the dependency graph.
                return_val = true;
            } else {
                DPRINTF(IQ, "IQ: Instruction PC %#x has src reg %i that "
                        "became ready before it reached the IQ.\n",
                        new_inst->readPC(), src_reg);
                // Mark a register ready within the instruction.
                new_inst->markSrcRegReady();
            }
        }
    }

    return return_val;
}

template <class Impl>
void
InstructionQueue<Impl>::createDependency(DynInstPtr &new_inst)
{
    //Actually nothing really needs to be marked when an
    //instruction becomes the producer of a register's value,
    //but for convenience a ptr to the producing instruction will
    //be placed in the head node of the dependency links.
    int8_t total_dest_regs = new_inst->numDestRegs();

    for (int dest_reg_idx = 0;
         dest_reg_idx < total_dest_regs;
         dest_reg_idx++)
    {
        PhysRegIndex dest_reg = new_inst->renamedDestRegIdx(dest_reg_idx);

        // Instructions that use the misc regs will have a reg number
        // higher than the normal physical registers.  In this case these
        // registers are not renamed, and there is no need to track
        // dependencies as these instructions must be executed at commit.
        if (dest_reg >= numPhysRegs) {
            continue;
        }

        dependGraph[dest_reg].inst = new_inst;

        if (dependGraph[dest_reg].next) {
            dumpDependGraph();
            panic("IQ: Dependency graph not empty!");
        }

        // Mark the scoreboard to say it's not yet ready.
        regScoreboard[dest_reg] = false;
    }
}

template <class Impl>
void
InstructionQueue<Impl>::addIfReady(DynInstPtr &inst)
{
    //If the instruction now has all of its source registers
    // available, then add it to the list of ready instructions.
    if (inst->readyToIssue()) {

        //Add the instruction to the proper ready list.
        if (inst->isControl()) {

            DPRINTF(IQ, "IQ: Branch instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x.\n",
                    inst->readPC());
            readyBranchInsts.push(inst);

        } else if (inst->isMemRef()) {

            DPRINTF(IQ, "IQ: Checking if memory instruction can issue.\n");

            // Message to the mem dependence unit that this instruction has
            // its registers ready.

            memDepUnit.regsReady(inst);

#if 0
            if (memDepUnit.readyToIssue(inst)) {
                DPRINTF(IQ, "IQ: Memory instruction is ready to issue, "
                        "putting it onto the ready list, PC %#x.\n",
                        inst->readPC());
                readyMemInsts.push(inst);
            } else {
                // Make dependent on the store.
                // Will need some way to get the store instruction it should
                // be dependent upon; then when the store issues it can
                // put the instruction on the ready list.
                // Yet another tree?
                assert(0 && "Instruction has no way to actually issue");
            }
#endif

        } else if (inst->isInteger()) {

            DPRINTF(IQ, "IQ: Integer instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x.\n",
                    inst->readPC());
            readyIntInsts.push(inst);

        } else if (inst->isFloating()) {

            DPRINTF(IQ, "IQ: Floating instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x.\n",
                    inst->readPC());
            readyFloatInsts.push(inst);

        } else {
            DPRINTF(IQ, "IQ: Miscellaneous instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x..\n",
                    inst->readPC());

            readyMiscInsts.push(inst);
        }
    }
}

/*
 * Caution, this function must not be called prior to tail being updated at
 * least once, otherwise it will fail the assertion.  This is because
 * instList.begin() actually changes upon the insertion of an element into the
 * list when the list is empty.
 */
template <class Impl>
int
InstructionQueue<Impl>::countInsts()
{
    ListIt count_it = cpu->instList.begin();
    int total_insts = 0;

    if (tail == cpu->instList.end())
        return 0;

    while (count_it != tail) {
        if (!(*count_it)->isIssued()) {
            ++total_insts;
        }

        ++count_it;

        assert(count_it != cpu->instList.end());
    }

    // Need to count the tail iterator as well.
    if (count_it != cpu->instList.end() &&
        (*count_it) &&
        !(*count_it)->isIssued()) {
        ++total_insts;
    }

    return total_insts;
}

template <class Impl>
void
InstructionQueue<Impl>::dumpDependGraph()
{
    DependencyEntry *curr;

    for (int i = 0; i < numPhysRegs; ++i)
    {
        curr = &dependGraph[i];

        if (curr->inst) {
            cprintf("dependGraph[%i]: producer: %#x consumer: ", i,
                    curr->inst->readPC());
        } else {
            cprintf("dependGraph[%i]: No producer. consumer: ", i);
        }

        while (curr->next != NULL) {
            curr = curr->next;

            cprintf("%#x ", curr->inst->readPC());
        }

        cprintf("\n");
    }
}

template <class Impl>
void
InstructionQueue<Impl>::dumpLists()
{
    cprintf("Ready integer list size: %i\n", readyIntInsts.size());

    cprintf("Ready float list size: %i\n", readyFloatInsts.size());

    cprintf("Ready branch list size: %i\n", readyBranchInsts.size());

    cprintf("Ready misc list size: %i\n", readyMiscInsts.size());

    cprintf("Squashed list size: %i\n", squashedInsts.size());

    cprintf("Non speculative list size: %i\n", nonSpecInsts.size());

    non_spec_it_t non_spec_it = nonSpecInsts.begin();

    cprintf("Non speculative list: ");

    while (non_spec_it != nonSpecInsts.end()) {
        cprintf("%#x ", (*non_spec_it).second->readPC());
        ++non_spec_it;
    }

    cprintf("\n");

}
