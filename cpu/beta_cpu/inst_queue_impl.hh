#ifndef __INST_QUEUE_IMPL_HH__
#define __INST_QUEUE_IMPL_HH__

// Todo: Fix up consistency errors about back of the ready list being
// the oldest instructions in the queue.  When woken up from the dependency
// graph they will be the oldest, but when they are immediately executable
// newer instructions will mistakenly get inserted onto the back.  Also
// current ordering allows for 0 cycle added-to-scheduled.  Could maybe fake
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

template<class Impl>
InstructionQueue<Impl>::InstructionQueue(Params &params)
    : numEntries(params.numIQEntries),
      intWidth(params.executeIntWidth),
      floatWidth(params.executeFloatWidth),
      numPhysIntRegs(params.numPhysIntRegs),
      numPhysFloatRegs(params.numPhysFloatRegs),
      commitToIEWDelay(params.commitToIEWDelay)
{
    // HACK: HARDCODED NUMBER.  REMOVE LATER AND ADD TO PARAMETER.
    totalWidth = 1;
    branchWidth = 1;
    DPRINTF(IQ, "IQ: Int width is %i.\n", params.executeIntWidth);

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

template<class Impl>
void
InstructionQueue<Impl>::setCPU(FullCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    tail = cpu->instList.begin();
}

template<class Impl>
void
InstructionQueue<Impl>::setIssueToExecuteQueue(
                        TimeBuffer<IssueStruct> *i2e_ptr)
{
    DPRINTF(IQ, "IQ: Set the issue to execute queue.\n");
    issueToExecuteQueue = i2e_ptr;
}

template<class Impl>
void
InstructionQueue<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(IQ, "IQ: Set the time buffer.\n");
    timeBuffer = tb_ptr;

    fromCommit = timeBuffer->getWire(-commitToIEWDelay);
}

// Might want to do something more complex if it knows how many instructions
// will be issued this cycle.
template<class Impl>
bool
InstructionQueue<Impl>::isFull()
{
    if (freeEntries == 0) {
        return(true);
    } else {
        return(false);
    }
}

template<class Impl>
unsigned
InstructionQueue<Impl>::numFreeEntries()
{
    return freeEntries;
}

template<class Impl>
void
InstructionQueue<Impl>::insert(DynInst *new_inst)
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

    // If the instruction is ready then add it to the ready list.
    addIfReady(new_inst);

    assert(freeEntries == (numEntries - countInsts()));
}

// Slightly hack function to advance the tail iterator in the case that
// the IEW stage issues an instruction that is not added to the IQ.  This
// is needed in case a long chain of such instructions occurs.
template<class Impl>
void
InstructionQueue<Impl>::advanceTail(DynInst *inst)
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
// issued does not exceed the total issue bandwidth.  Probably should
// have some sort of limit of total number of branches that can be issued
// as well.
template<class Impl>
void
InstructionQueue<Impl>::scheduleReadyInsts()
{
    DPRINTF(IQ, "IQ: Attempting to schedule ready instructions from "
                "the IQ.\n");

    int int_issued = 0;
    int float_issued = 0;
    int branch_issued = 0;
    int squashed_issued = 0;
    int total_issued = 0;

    IssueStruct *i2e_info = issueToExecuteQueue->access(0);

    bool insts_available = !readyBranchInsts.empty() ||
        !readyIntInsts.empty() ||
        !readyFloatInsts.empty() ||
        !squashedInsts.empty();

    // Note: Requires a globally defined constant.
    InstSeqNum oldest_inst = MaxInstSeqNum;
    InstList list_with_oldest = None;

    // Temporary values.
    DynInst *int_head_inst;
    DynInst *float_head_inst;
    DynInst *branch_head_inst;
    DynInst *squashed_head_inst;

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

            int_head_inst = readyIntInsts.top().inst;

            if (int_head_inst->isSquashed()) {
                readyIntInsts.pop();
                continue;
            }

            oldest_inst = int_head_inst->seqNum;

            list_with_oldest = Int;
        }

        if (!readyFloatInsts.empty() &&
            float_issued < floatWidth) {

            insts_available = true;

            float_head_inst = readyFloatInsts.top().inst;

            if (float_head_inst->isSquashed()) {
                readyFloatInsts.pop();
                continue;
            } else if (float_head_inst->seqNum < oldest_inst) {
                oldest_inst = float_head_inst->seqNum;

                list_with_oldest = Float;
            }
        }

        if (!readyBranchInsts.empty() &&
            branch_issued < branchWidth) {

            insts_available = true;

            branch_head_inst = readyBranchInsts.top().inst;

            if (branch_head_inst->isSquashed()) {
                readyBranchInsts.pop();
                continue;
            } else if (branch_head_inst->seqNum < oldest_inst) {
                oldest_inst = branch_head_inst->seqNum;

                list_with_oldest = Branch;
            }

        }

        if (!squashedInsts.empty()) {

            insts_available = true;

            squashed_head_inst = squashedInsts.top().inst;

            if (squashed_head_inst->seqNum < oldest_inst) {
                list_with_oldest = Squashed;
            }

        }

        DynInst *issuing_inst = NULL;

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
          case Squashed:
            issuing_inst = squashed_head_inst;
            squashedInsts.pop();
            ++squashed_issued;
            DPRINTF(IQ, "IQ: Issuing squashed instruction PC %#x.\n",
                    issuing_inst->readPC());
            break;
        }

        if (list_with_oldest != None) {
            i2e_info->insts[total_issued] = issuing_inst;

            issuing_inst->setIssued();

            ++freeEntries;
            ++total_issued;
        }

        assert(freeEntries == (numEntries - countInsts()));
    }
}

template<class Impl>
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
        DynInst *squashed_inst = (*squashIt);

        // Only handle the instruction if it actually is in the IQ and
        // hasn't already been squashed in the IQ.
        if (!squashed_inst->isIssued() &&
            !squashed_inst->isSquashedInIQ()) {
            // Remove the instruction from the dependency list.
            int8_t total_src_regs = squashed_inst->numSrcRegs();

            for (int src_reg_idx = 0;
                 src_reg_idx < total_src_regs;
                 src_reg_idx++)
            {
                // Only remove it from the dependency graph if it was
                // placed there in the first place.
                // HACK: This assumes that instructions woken up from the
                // dependency chain aren't informed that a specific src
                // register has become ready.  This may not always be true
                // in the future.
                if (!squashed_inst->isReadySrcRegIdx(src_reg_idx)) {
                    int8_t src_reg =
                        squashed_inst->renamedSrcRegIdx(src_reg_idx);
                    dependGraph[src_reg].remove(squashed_inst);
                }
            }

            // Mark it as squashed within the IQ.
            squashed_inst->setSquashedInIQ();

            ReadyEntry temp(squashed_inst);

            squashedInsts.push(temp);

            DPRINTF(IQ, "IQ: Instruction PC %#x squashed.\n",
                    squashed_inst->readPC());
        }
        squashIt--;
    }
}

template<class Impl>
void
InstructionQueue<Impl>::squash()
{
    DPRINTF(IQ, "IQ: Starting to squash instructions in the IQ.\n");

    // Read instruction sequence number of last instruction out of the
    // time buffer.
    squashedSeqNum = fromCommit->commitInfo.doneSeqNum;

    // Setup the squash iterator to point to the tail.
    squashIt = tail;

    // Call doSquash.
    doSquash();
}

template<class Impl>
void
InstructionQueue<Impl>::stopSquash()
{
    // Clear up the squash variables to ensure that squashing doesn't
    // get called improperly.
    squashedSeqNum = 0;

    squashIt = cpu->instList.end();
}

template<class Impl>
int
InstructionQueue<Impl>::countInsts()
{
    ListIt count_it = cpu->instList.begin();
    int total_insts = 0;

    while (count_it != tail) {
        if (!(*count_it)->isIssued()) {
            ++total_insts;
        }

        count_it++;

        assert(count_it != cpu->instList.end());
    }

    // Need to count the tail iterator as well.
    if (count_it != cpu->instList.end() &&
        (*count_it) != NULL &&
        !(*count_it)->isIssued()) {
        ++total_insts;
    }

    return total_insts;
}

template<class Impl>
void
InstructionQueue<Impl>::wakeDependents(DynInst *completed_inst)
{
    DPRINTF(IQ, "IQ: Waking dependents of completed instruction.\n");
    //Look at the physical destination register of the DynInst
    //and look it up on the dependency graph.  Then mark as ready
    //any instructions within the instruction queue.
    int8_t total_dest_regs = completed_inst->numDestRegs();

    DependencyEntry *curr;

    for (int dest_reg_idx = 0;
         dest_reg_idx < total_dest_regs;
         dest_reg_idx++)
    {
        PhysRegIndex dest_reg =
            completed_inst->renamedDestRegIdx(dest_reg_idx);

        // Special case of uniq or control registers.  They are not
        // handled by the IQ and thus have no dependency graph entry.
        // @todo Figure out a cleaner way to handle thie.
        if (dest_reg >= numPhysRegs) {
            continue;
        }

        DPRINTF(IQ, "IQ: Waking any dependents on register %i.\n",
                (int) dest_reg);

        //Maybe abstract this part into a function.
        //Go through the dependency chain, marking the registers as ready
        //within the waiting instructions.
        while (dependGraph[dest_reg].next != NULL) {

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

template<class Impl>
bool
InstructionQueue<Impl>::addToDependents(DynInst *new_inst)
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
            if (regScoreboard[src_reg] == false) {
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

template<class Impl>
void
InstructionQueue<Impl>::createDependency(DynInst *new_inst)
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
        int8_t dest_reg = new_inst->renamedDestRegIdx(dest_reg_idx);
        dependGraph[dest_reg].inst = new_inst;
        if (dependGraph[dest_reg].next != NULL) {
            panic("Dependency chain is not empty.\n");
        }

        // Mark the scoreboard to say it's not yet ready.
        regScoreboard[dest_reg] = false;
    }
}

template<class Impl>
void
InstructionQueue<Impl>::DependencyEntry::insert(DynInst *new_inst)
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
}

template<class Impl>
void
InstructionQueue<Impl>::DependencyEntry::remove(DynInst *inst_to_remove)
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
    }

    // Now remove this instruction from the list.
    prev->next = curr->next;

    delete curr;
}

template<class Impl>
void
InstructionQueue<Impl>::addIfReady(DynInst *inst)
{
    //If the instruction now has all of its source registers
    // available, then add it to the list of ready instructions.
    if (inst->readyToIssue()) {
        ReadyEntry to_add(inst);
        //Add the instruction to the proper ready list.
        if (inst->isInteger()) {
            DPRINTF(IQ, "IQ: Integer instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x.\n",
                    inst->readPC());
            readyIntInsts.push(to_add);
        } else if (inst->isFloating()) {
            DPRINTF(IQ, "IQ: Floating instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x.\n",
                    inst->readPC());
            readyFloatInsts.push(to_add);
        } else if (inst->isControl()) {
            DPRINTF(IQ, "IQ: Branch instruction is ready to issue, "
                    "putting it onto the ready list, PC %#x.\n",
                    inst->readPC());
            readyBranchInsts.push(to_add);
        } else {
            panic("IQ: Instruction not an expected type.\n");
        }
    }
}

#endif // __INST_QUEUE_IMPL_HH__
