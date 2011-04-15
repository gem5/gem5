/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

// Todo:
// Current ordering allows for 0 cycle added-to-scheduled.  Could maybe fake
// it; either do in reverse order, or have added instructions put into a
// different ready queue that, in scheduleRreadyInsts(), gets put onto the
// normal ready queue.  This would however give only a one cycle delay,
// but probably is more flexible to actually add in a delay parameter than
// just running it backwards.

#include <vector>

#include "cpu/ozone/inst_queue.hh"
#include "sim/core.hh"
#if 0
template <class Impl>
InstQueue<Impl>::FUCompletion::FUCompletion(DynInstPtr &_inst,
                                                   int fu_idx,
                                                   InstQueue<Impl> *iq_ptr)
    : Event(&mainEventQueue, Stat_Event_Pri),
      inst(_inst), fuIdx(fu_idx), iqPtr(iq_ptr)
{
    this->setFlags(Event::AutoDelete);
}

template <class Impl>
void
InstQueue<Impl>::FUCompletion::process()
{
    iqPtr->processFUCompletion(inst, fuIdx);
}


template <class Impl>
const char *
InstQueue<Impl>::FUCompletion::description() const
{
    return "Functional unit completion";
}
#endif
template <class Impl>
InstQueue<Impl>::InstQueue(Params *params)
    : dcacheInterface(params->dcacheInterface),
//      fuPool(params->fuPool),
      numEntries(params->numIQEntries),
      totalWidth(params->issueWidth),
//      numPhysIntRegs(params->numPhysIntRegs),
//      numPhysFloatRegs(params->numPhysFloatRegs),
      commitToIEWDelay(params->commitToIEWDelay)
{
//    assert(fuPool);

//    numThreads = params->numberOfThreads;
    numThreads = 1;

    //Initialize thread IQ counts
    for (ThreadID tid = 0; tid <numThreads; tid++) {
        count[tid] = 0;
    }

    // Initialize the number of free IQ entries.
    freeEntries = numEntries;

    // Set the number of physical registers as the number of int + float
//    numPhysRegs = numPhysIntRegs + numPhysFloatRegs;

//    DPRINTF(IQ, "There are %i physical registers.\n", numPhysRegs);

    //Create an entry for each physical register within the
    //dependency graph.
//    dependGraph = new DependencyEntry[numPhysRegs];

    // Resize the register scoreboard.
//    regScoreboard.resize(numPhysRegs);
/*
    //Initialize Mem Dependence Units
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        memDepUnit[tid].init(params, tid);
        memDepUnit[tid].setIQ(this);
    }

    // Initialize all the head pointers to point to NULL, and all the
    // entries as unready.
    // Note that in actuality, the registers corresponding to the logical
    // registers start off as ready.  However this doesn't matter for the
    // IQ as the instruction should have been correctly told if those
    // registers are ready in rename.  Thus it can all be initialized as
    // unready.
    for (int i = 0; i < numPhysRegs; ++i) {
        dependGraph[i].next = NULL;
        dependGraph[i].inst = NULL;
        regScoreboard[i] = false;
    }
*/
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        squashedSeqNum[tid] = 0;
    }
/*
    for (int i = 0; i < Num_OpClasses; ++i) {
        queueOnList[i] = false;
        readyIt[i] = listOrder.end();
    }

    string policy = params->smtIQPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Figure out resource sharing policy
    if (policy == "dynamic") {
        iqPolicy = Dynamic;

        //Set Max Entries to Total ROB Capacity
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = numEntries;
        }

    } else if (policy == "partitioned") {
        iqPolicy = Partitioned;

        //@todo:make work if part_amt doesnt divide evenly.
        int part_amt = numEntries / numThreads;

        //Divide ROB up evenly
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = part_amt;
        }

        DPRINTF(Fetch, "IQ sharing policy set to Partitioned:"
                "%i entries per thread.\n",part_amt);

    } else if (policy == "threshold") {
        iqPolicy = Threshold;

        double threshold =  (double)params->smtIQThreshold / 100;

        int thresholdIQ = (int)((double)threshold * numEntries);

        //Divide up by threshold amount
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = thresholdIQ;
        }

        DPRINTF(Fetch, "IQ sharing policy set to Threshold:"
                "%i entries per thread.\n",thresholdIQ);
   } else {
       assert(0 && "Invalid IQ Sharing Policy.Options Are:{Dynamic,"
              "Partitioned, Threshold}");
   }
*/
}

template <class Impl>
InstQueue<Impl>::~InstQueue()
{
    // Clear the dependency graph
/*
    DependencyEntry *curr;
    DependencyEntry *prev;

    for (int i = 0; i < numPhysRegs; ++i) {
        curr = dependGraph[i].next;

        while (curr) {
            DependencyEntry::mem_alloc_counter--;

            prev = curr;
            curr = prev->next;
            prev->inst = NULL;

            delete prev;
        }

        if (dependGraph[i].inst) {
            dependGraph[i].inst = NULL;
        }

        dependGraph[i].next = NULL;
    }

    assert(DependencyEntry::mem_alloc_counter == 0);

    delete [] dependGraph;
*/
}

template <class Impl>
std::string
InstQueue<Impl>::name() const
{
    return cpu->name() + ".iq";
}

template <class Impl>
void
InstQueue<Impl>::regStats()
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
/*
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        // Tell mem dependence unit to reg stats as well.
        memDepUnit[tid].regStats();
    }
*/
}
/*
template <class Impl>
void
InstQueue<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(IQ, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}
*/
template <class Impl>
void
InstQueue<Impl>::setIssueToExecuteQueue(TimeBuffer<IssueStruct> *i2e_ptr)
{
    DPRINTF(IQ, "Set the issue to execute queue.\n");
    issueToExecuteQueue = i2e_ptr;
}
/*
template <class Impl>
void
InstQueue<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    DPRINTF(IQ, "Set the time buffer.\n");
    timeBuffer = tb_ptr;

    fromCommit = timeBuffer->getWire(-commitToIEWDelay);
}

template <class Impl>
int
InstQueue<Impl>::entryAmount(ThreadID num_threads)
{
    if (iqPolicy == Partitioned) {
        return numEntries / num_threads;
    } else {
        return 0;
    }
}


template <class Impl>
void
InstQueue<Impl>::resetEntries()
{
    if (iqPolicy != Dynamic || numThreads > 1) {
        int active_threads = activeThreads->size();

        std::list<unsigned>::iterator threads = activeThreads->begin();
        std::list<unsigned>::iterator end = activeThreads->end();

        while (threads != end) {
            ThreadID tid = *threads++;

            if (iqPolicy == Partitioned) {
                maxEntries[tid] = numEntries / active_threads;
            } else if (iqPolicy == Threshold && active_threads == 1) {
                maxEntries[tid] = numEntries;
            }
        }
    }
}
*/
template <class Impl>
unsigned
InstQueue<Impl>::numFreeEntries()
{
    return freeEntries;
}

template <class Impl>
unsigned
InstQueue<Impl>::numFreeEntries(ThreadID tid)
{
    return maxEntries[tid] - count[tid];
}

// Might want to do something more complex if it knows how many instructions
// will be issued this cycle.
template <class Impl>
bool
InstQueue<Impl>::isFull()
{
    if (freeEntries == 0) {
        return(true);
    } else {
        return(false);
    }
}

template <class Impl>
bool
InstQueue<Impl>::isFull(ThreadID tid)
{
    if (numFreeEntries(tid) == 0) {
        return(true);
    } else {
        return(false);
    }
}

template <class Impl>
bool
InstQueue<Impl>::hasReadyInsts()
{
/*
    if (!listOrder.empty()) {
        return true;
    }

    for (int i = 0; i < Num_OpClasses; ++i) {
        if (!readyInsts[i].empty()) {
            return true;
        }
    }

    return false;
*/
    return readyInsts.empty();
}

template <class Impl>
void
InstQueue<Impl>::insert(DynInstPtr &new_inst)
{
    // Make sure the instruction is valid
    assert(new_inst);

    DPRINTF(IQ, "Adding instruction PC %#x to the IQ.\n",
            new_inst->readPC());

    // Check if there are any free entries.  Panic if there are none.
    // Might want to have this return a fault in the future instead of
    // panicing.
    assert(freeEntries != 0);

    instList[new_inst->threadNumber].push_back(new_inst);

    // Decrease the number of free entries.
    --freeEntries;

    //Mark Instruction as in IQ
//    new_inst->setInIQ();
/*
    // Look through its source registers (physical regs), and mark any
    // dependencies.
    addToDependents(new_inst);

    // Have this instruction set itself as the producer of its destination
    // register(s).
    createDependency(new_inst);
*/
    // If it's a memory instruction, add it to the memory dependency
    // unit.
//    if (new_inst->isMemRef()) {
//        memDepUnit[new_inst->threadNumber].insert(new_inst);
//    } else {
        // If the instruction is ready then add it to the ready list.
        addIfReady(new_inst);
//    }

    ++iqInstsAdded;


    //Update Thread IQ Count
    count[new_inst->threadNumber]++;

    assert(freeEntries == (numEntries - countInsts()));
}

template <class Impl>
void
InstQueue<Impl>::insertNonSpec(DynInstPtr &new_inst)
{
    nonSpecInsts[new_inst->seqNum] = new_inst;

    // @todo: Clean up this code; can do it by setting inst as unable
    // to issue, then calling normal insert on the inst.

    // Make sure the instruction is valid
    assert(new_inst);

    DPRINTF(IQ, "Adding instruction PC %#x to the IQ.\n",
            new_inst->readPC());

    // Check if there are any free entries.  Panic if there are none.
    // Might want to have this return a fault in the future instead of
    // panicing.
    assert(freeEntries != 0);

    instList[new_inst->threadNumber].push_back(new_inst);

    // Decrease the number of free entries.
    --freeEntries;

    //Mark Instruction as in IQ
//    new_inst->setInIQ();
/*
    // Have this instruction set itself as the producer of its destination
    // register(s).
    createDependency(new_inst);

    // If it's a memory instruction, add it to the memory dependency
    // unit.
    if (new_inst->isMemRef()) {
        memDepUnit[new_inst->threadNumber].insertNonSpec(new_inst);
    }
*/
    ++iqNonSpecInstsAdded;

    //Update Thread IQ Count
    count[new_inst->threadNumber]++;

    assert(freeEntries == (numEntries - countInsts()));
}
/*
template <class Impl>
void
InstQueue<Impl>::advanceTail(DynInstPtr &inst)
{
    // Have this instruction set itself as the producer of its destination
    // register(s).
    createDependency(inst);
}

template <class Impl>
void
InstQueue<Impl>::addToOrderList(OpClass op_class)
{
    assert(!readyInsts[op_class].empty());

    ListOrderEntry queue_entry;

    queue_entry.queueType = op_class;

    queue_entry.oldestInst = readyInsts[op_class].top()->seqNum;

    ListOrderIt list_it = listOrder.begin();
    ListOrderIt list_end_it = listOrder.end();

    while (list_it != list_end_it) {
        if ((*list_it).oldestInst > queue_entry.oldestInst) {
            break;
        }

        list_it++;
    }

    readyIt[op_class] = listOrder.insert(list_it, queue_entry);
    queueOnList[op_class] = true;
}

template <class Impl>
void
InstQueue<Impl>::moveToYoungerInst(ListOrderIt list_order_it)
{
    // Get iterator of next item on the list
    // Delete the original iterator
    // Determine if the next item is either the end of the list or younger
    // than the new instruction.  If so, then add in a new iterator right here.
    // If not, then move along.
    ListOrderEntry queue_entry;
    OpClass op_class = (*list_order_it).queueType;
    ListOrderIt next_it = list_order_it;

    ++next_it;

    queue_entry.queueType = op_class;
    queue_entry.oldestInst = readyInsts[op_class].top()->seqNum;

    while (next_it != listOrder.end() &&
           (*next_it).oldestInst < queue_entry.oldestInst) {
        ++next_it;
    }

    readyIt[op_class] = listOrder.insert(next_it, queue_entry);
}

template <class Impl>
void
InstQueue<Impl>::processFUCompletion(DynInstPtr &inst, int fu_idx)
{
    // The CPU could have been sleeping until this op completed (*extremely*
    // long latency op).  Wake it if it was.  This may be overkill.
    iewStage->wakeCPU();

    fuPool->freeUnit(fu_idx);

    int &size = issueToExecuteQueue->access(0)->size;

    issueToExecuteQueue->access(0)->insts[size++] = inst;
}
*/
// @todo: Figure out a better way to remove the squashed items from the
// lists.  Checking the top item of each list to see if it's squashed
// wastes time and forces jumps.
template <class Impl>
void
InstQueue<Impl>::scheduleReadyInsts()
{
    DPRINTF(IQ, "Attempting to schedule ready instructions from "
            "the IQ.\n");

//    IssueStruct *i2e_info = issueToExecuteQueue->access(0);
/*
    // Will need to reorder the list if either a queue is not on the list,
    // or it has an older instruction than last time.
    for (int i = 0; i < Num_OpClasses; ++i) {
        if (!readyInsts[i].empty()) {
            if (!queueOnList[i]) {
                addToOrderList(OpClass(i));
            } else if (readyInsts[i].top()->seqNum  <
                       (*readyIt[i]).oldestInst) {
                listOrder.erase(readyIt[i]);
                addToOrderList(OpClass(i));
            }
        }
    }

    // Have iterator to head of the list
    // While I haven't exceeded bandwidth or reached the end of the list,
    // Try to get a FU that can do what this op needs.
    // If successful, change the oldestInst to the new top of the list, put
    // the queue in the proper place in the list.
    // Increment the iterator.
    // This will avoid trying to schedule a certain op class if there are no
    // FUs that handle it.
    ListOrderIt order_it = listOrder.begin();
    ListOrderIt order_end_it = listOrder.end();
    int total_issued = 0;
    int exec_queue_slot = i2e_info->size;

    while (exec_queue_slot < totalWidth && order_it != order_end_it) {
        OpClass op_class = (*order_it).queueType;

        assert(!readyInsts[op_class].empty());

        DynInstPtr issuing_inst = readyInsts[op_class].top();

        assert(issuing_inst->seqNum == (*order_it).oldestInst);

        if (issuing_inst->isSquashed()) {
            readyInsts[op_class].pop();

            if (!readyInsts[op_class].empty()) {
                moveToYoungerInst(order_it);
            } else {
                readyIt[op_class] = listOrder.end();
                queueOnList[op_class] = false;
            }

            listOrder.erase(order_it++);

            ++iqSquashedInstsIssued;

            continue;
        }

        int idx = fuPool->getUnit(op_class);

        if (idx != -1) {
            int op_latency = fuPool->getOpLatency(op_class);

            if (op_latency == 1) {
                i2e_info->insts[exec_queue_slot++] = issuing_inst;
                i2e_info->size++;

                // Add the FU onto the list of FU's to be freed next cycle.
                fuPool->freeUnit(idx);
            } else {
                int issue_latency = fuPool->getIssueLatency(op_class);

                if (issue_latency > 1) {
                    // Generate completion event for the FU
                    FUCompletion *execution = new FUCompletion(issuing_inst,
                                                               idx, this);

                    execution->schedule(curTick() + issue_latency - 1);
                } else {
                    i2e_info->insts[exec_queue_slot++] = issuing_inst;
                    i2e_info->size++;

                    // Add the FU onto the list of FU's to be freed next cycle.
                    fuPool->freeUnit(idx);
                }
            }

            DPRINTF(IQ, "Thread %i: Issuing instruction PC %#x "
                    "[sn:%lli]\n",
                    issuing_inst->threadNumber, issuing_inst->readPC(),
                    issuing_inst->seqNum);

            readyInsts[op_class].pop();

            if (!readyInsts[op_class].empty()) {
                moveToYoungerInst(order_it);
            } else {
                readyIt[op_class] = listOrder.end();
                queueOnList[op_class] = false;
            }

            issuing_inst->setIssued();
            ++total_issued;

            if (!issuing_inst->isMemRef()) {
                // Memory instructions can not be freed from the IQ until they
                // complete.
                ++freeEntries;
                count[issuing_inst->threadNumber]--;
                issuing_inst->removeInIQ();
            } else {
                memDepUnit[issuing_inst->threadNumber].issue(issuing_inst);
            }

            listOrder.erase(order_it++);
        } else {
            ++order_it;
        }
    }

    if (total_issued) {
        cpu->activityThisCycle();
    } else {
        DPRINTF(IQ, "Not able to schedule any instructions.\n");
    }
*/
}

template <class Impl>
void
InstQueue<Impl>::scheduleNonSpec(const InstSeqNum &inst)
{
    DPRINTF(IQ, "Marking nonspeculative instruction with sequence "
            "number %i as ready to execute.\n", inst);

    NonSpecMapIt inst_it = nonSpecInsts.find(inst);

    assert(inst_it != nonSpecInsts.end());

//    ThreadID tid = (*inst_it).second->threadNumber;

    // Mark this instruction as ready to issue.
    (*inst_it).second->setCanIssue();

    // Now schedule the instruction.
//    if (!(*inst_it).second->isMemRef()) {
        addIfReady((*inst_it).second);
//    } else {
//        memDepUnit[tid].nonSpecInstReady((*inst_it).second);
//    }

    nonSpecInsts.erase(inst_it);
}

template <class Impl>
void
InstQueue<Impl>::commit(const InstSeqNum &inst, ThreadID tid)
{
    /*Need to go through each thread??*/
    DPRINTF(IQ, "[tid:%i]: Committing instructions older than [sn:%i]\n",
            tid,inst);

    ListIt iq_it = instList[tid].begin();

    while (iq_it != instList[tid].end() &&
           (*iq_it)->seqNum <= inst) {
        ++iq_it;
        instList[tid].pop_front();
    }

    assert(freeEntries == (numEntries - countInsts()));
}

template <class Impl>
void
InstQueue<Impl>::wakeDependents(DynInstPtr &completed_inst)
{
    DPRINTF(IQ, "Waking dependents of completed instruction.\n");
    // Look at the physical destination register of the DynInst
    // and look it up on the dependency graph.  Then mark as ready
    // any instructions within the instruction queue.
/*
    DependencyEntry *curr;
    DependencyEntry *prev;
*/
    // Tell the memory dependence unit to wake any dependents on this
    // instruction if it is a memory instruction.  Also complete the memory
    // instruction at this point since we know it executed fine.
    // @todo: Might want to rename "completeMemInst" to
    // something that indicates that it won't need to be replayed, and call
    // this earlier.  Might not be a big deal.
    if (completed_inst->isMemRef()) {
//        memDepUnit[completed_inst->threadNumber].wakeDependents(completed_inst);
        completeMemInst(completed_inst);
    }
    completed_inst->wakeDependents();
/*
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

        DPRINTF(IQ, "Waking any dependents on register %i.\n",
                (int) dest_reg);

        //Maybe abstract this part into a function.
        //Go through the dependency chain, marking the registers as ready
        //within the waiting instructions.

        curr = dependGraph[dest_reg].next;

        while (curr) {
            DPRINTF(IQ, "Waking up a dependent instruction, PC%#x.\n",
                    curr->inst->readPC());

            // Might want to give more information to the instruction
            // so that it knows which of its source registers is ready.
            // However that would mean that the dependency graph entries
            // would need to hold the src_reg_idx.
            curr->inst->markSrcRegReady();

            addIfReady(curr->inst);

            DependencyEntry::mem_alloc_counter--;

            prev = curr;
            curr = prev->next;
            prev->inst = NULL;

            delete prev;
        }

        // Reset the head node now that all of its dependents have been woken
        // up.
        dependGraph[dest_reg].next = NULL;
        dependGraph[dest_reg].inst = NULL;

        // Mark the scoreboard as having that register ready.
        regScoreboard[dest_reg] = true;
    }
*/
}

template <class Impl>
void
InstQueue<Impl>::addReadyMemInst(DynInstPtr &ready_inst)
{
//    OpClass op_class = ready_inst->opClass();

    readyInsts.push(ready_inst);

    DPRINTF(IQ, "Instruction is ready to issue, putting it onto "
            "the ready list, PC %#x opclass:%i [sn:%lli].\n",
            ready_inst->readPC(), ready_inst->opClass(), ready_inst->seqNum);
}
/*
template <class Impl>
void
InstQueue<Impl>::rescheduleMemInst(DynInstPtr &resched_inst)
{
    memDepUnit[resched_inst->threadNumber].reschedule(resched_inst);
}

template <class Impl>
void
InstQueue<Impl>::replayMemInst(DynInstPtr &replay_inst)
{
    memDepUnit[replay_inst->threadNumber].replay(replay_inst);
}
*/
template <class Impl>
void
InstQueue<Impl>::completeMemInst(DynInstPtr &completed_inst)
{
    ThreadID tid = completed_inst->threadNumber;

    DPRINTF(IQ, "Completing mem instruction PC:%#x [sn:%lli]\n",
            completed_inst->readPC(), completed_inst->seqNum);

    ++freeEntries;

//    completed_inst->memOpDone = true;

//    memDepUnit[tid].completed(completed_inst);

    count[tid]--;
}
/*
template <class Impl>
void
InstQueue<Impl>::violation(DynInstPtr &store,
                                  DynInstPtr &faulting_load)
{
    memDepUnit[store->threadNumber].violation(store, faulting_load);
}
*/
template <class Impl>
void
InstQueue<Impl>::squash(ThreadID tid)
{
    DPRINTF(IQ, "[tid:%i]: Starting to squash instructions in "
            "the IQ.\n", tid);

    // Read instruction sequence number of last instruction out of the
    // time buffer.
//    squashedSeqNum[tid] = fromCommit->commitInfo[tid].doneSeqNum;

    // Setup the squash iterator to point to the tail.
    squashIt[tid] = instList[tid].end();
    --squashIt[tid];

    // Call doSquash if there are insts in the IQ
    if (count[tid] > 0) {
        doSquash(tid);
    }

    // Also tell the memory dependence unit to squash.
//    memDepUnit[tid].squash(squashedSeqNum[tid], tid);
}

template <class Impl>
void
InstQueue<Impl>::doSquash(ThreadID tid)
{
    // Make sure the squashed sequence number is valid.
    assert(squashedSeqNum[tid] != 0);

    DPRINTF(IQ, "[tid:%i]: Squashing until sequence number %i!\n",
            tid, squashedSeqNum[tid]);

    // Squash any instructions younger than the squashed sequence number
    // given.
    while (squashIt[tid] != instList[tid].end() &&
           (*squashIt[tid])->seqNum > squashedSeqNum[tid]) {

        DynInstPtr squashed_inst = (*squashIt[tid]);

        // Only handle the instruction if it actually is in the IQ and
        // hasn't already been squashed in the IQ.
        if (squashed_inst->threadNumber != tid ||
            squashed_inst->isSquashedInIQ()) {
            --squashIt[tid];
            continue;
        }

        if (!squashed_inst->isIssued() ||
            (squashed_inst->isMemRef()/* &&
                                         !squashed_inst->memOpDone*/)) {

            // Remove the instruction from the dependency list.
            if (!squashed_inst->isNonSpeculative()) {
/*
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
                    // Instead of doing a linked list traversal, we can just
                    // remove these squashed instructions either at issue time,
                    // or when the register is overwritten.  The only downside
                    // to this is it leaves more room for error.

                    if (!squashed_inst->isReadySrcRegIdx(src_reg_idx) &&
                        src_reg < numPhysRegs) {
                        dependGraph[src_reg].remove(squashed_inst);
                    }


                    ++iqSquashedOperandsExamined;
                }
*/
                // Might want to remove producers as well.
            } else {
                nonSpecInsts[squashed_inst->seqNum] = NULL;

                nonSpecInsts.erase(squashed_inst->seqNum);

                ++iqSquashedNonSpecRemoved;
            }

            // Might want to also clear out the head of the dependency graph.

            // Mark it as squashed within the IQ.
            squashed_inst->setSquashedInIQ();

            // @todo: Remove this hack where several statuses are set so the
            // inst will flow through the rest of the pipeline.
            squashed_inst->setIssued();
            squashed_inst->setCanCommit();
//            squashed_inst->removeInIQ();

            //Update Thread IQ Count
            count[squashed_inst->threadNumber]--;

            ++freeEntries;

            if (numThreads > 1) {
                DPRINTF(IQ, "[tid:%i]: Instruction PC %#x squashed.\n",
                        tid, squashed_inst->readPC());
            } else {
                DPRINTF(IQ, "Instruction PC %#x squashed.\n",
                        squashed_inst->readPC());
            }
        }

        --squashIt[tid];
        ++iqSquashedInstsExamined;
    }
}
/*
template <class Impl>
void
InstQueue<Impl>::DependencyEntry::insert(DynInstPtr &new_inst)
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
InstQueue<Impl>::DependencyEntry::remove(DynInstPtr &inst_to_remove)
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
    while (curr->inst != inst_to_remove) {
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
InstQueue<Impl>::addToDependents(DynInstPtr &new_inst)
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
                DPRINTF(IQ, "Instruction PC %#x has src reg %i that "
                        "is being added to the dependency chain.\n",
                        new_inst->readPC(), src_reg);

                dependGraph[src_reg].insert(new_inst);

                // Change the return value to indicate that something
                // was added to the dependency graph.
                return_val = true;
            } else {
                DPRINTF(IQ, "Instruction PC %#x has src reg %i that "
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
InstQueue<Impl>::createDependency(DynInstPtr &new_inst)
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

        if (dependGraph[dest_reg].next) {
            dumpDependGraph();
            panic("Dependency graph %i not empty!", dest_reg);
        }

        dependGraph[dest_reg].inst = new_inst;

        // Mark the scoreboard to say it's not yet ready.
        regScoreboard[dest_reg] = false;
    }
}
*/
template <class Impl>
void
InstQueue<Impl>::addIfReady(DynInstPtr &inst)
{
    //If the instruction now has all of its source registers
    // available, then add it to the list of ready instructions.
    if (inst->readyToIssue()) {

        //Add the instruction to the proper ready list.
        if (inst->isMemRef()) {

            DPRINTF(IQ, "Checking if memory instruction can issue.\n");

            // Message to the mem dependence unit that this instruction has
            // its registers ready.

//            memDepUnit[inst->threadNumber].regsReady(inst);

            return;
        }

//        OpClass op_class = inst->opClass();

        DPRINTF(IQ, "Instruction is ready to issue, putting it onto "
                "the ready list, PC %#x opclass:%i [sn:%lli].\n",
                inst->readPC(), inst->opClass(), inst->seqNum);

        readyInsts.push(inst);
    }
}

template <class Impl>
int
InstQueue<Impl>::countInsts()
{
    //ksewell:This works but definitely could use a cleaner write
    //with a more intuitive way of counting. Right now it's
    //just brute force ....

#if 0
    int total_insts = 0;

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        ListIt count_it = instList[tid].begin();

        while (count_it != instList[tid].end()) {
            if (!(*count_it)->isSquashed() && !(*count_it)->isSquashedInIQ()) {
                if (!(*count_it)->isIssued()) {
                    ++total_insts;
                } else if ((*count_it)->isMemRef() &&
                           !(*count_it)->memOpDone) {
                    // Loads that have not been marked as executed still count
                    // towards the total instructions.
                    ++total_insts;
                }
            }

            ++count_it;
        }
    }

    return total_insts;
#else
    return numEntries - freeEntries;
#endif
}
/*
template <class Impl>
void
InstQueue<Impl>::dumpDependGraph()
{
    DependencyEntry *curr;

    for (int i = 0; i < numPhysRegs; ++i)
    {
        curr = &dependGraph[i];

        if (curr->inst) {
            cprintf("dependGraph[%i]: producer: %#x [sn:%lli] consumer: ",
                    i, curr->inst->readPC(), curr->inst->seqNum);
        } else {
            cprintf("dependGraph[%i]: No producer. consumer: ", i);
        }

        while (curr->next != NULL) {
            curr = curr->next;

            cprintf("%#x [sn:%lli] ",
                    curr->inst->readPC(), curr->inst->seqNum);
        }

        cprintf("\n");
    }
}
*/
template <class Impl>
void
InstQueue<Impl>::dumpLists()
{
    for (int i = 0; i < Num_OpClasses; ++i) {
        cprintf("Ready list %i size: %i\n", i, readyInsts.size());

        cprintf("\n");
    }

    cprintf("Non speculative list size: %i\n", nonSpecInsts.size());

    NonSpecMapIt non_spec_it = nonSpecInsts.begin();
    NonSpecMapIt non_spec_end_it = nonSpecInsts.end();

    cprintf("Non speculative list: ");

    while (non_spec_it != non_spec_end_it) {
        cprintf("%#x [sn:%lli]", (*non_spec_it).second->readPC(),
                (*non_spec_it).second->seqNum);
        ++non_spec_it;
    }

    cprintf("\n");
/*
    ListOrderIt list_order_it = listOrder.begin();
    ListOrderIt list_order_end_it = listOrder.end();
    int i = 1;

    cprintf("List order: ");

    while (list_order_it != list_order_end_it) {
        cprintf("%i OpClass:%i [sn:%lli] ", i, (*list_order_it).queueType,
                (*list_order_it).oldestInst);

        ++list_order_it;
        ++i;
    }
*/
    cprintf("\n");
}


template <class Impl>
void
InstQueue<Impl>::dumpInsts()
{
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
//        int num = 0;
//        int valid_num = 0;
/*
      ListIt inst_list_it = instList[tid].begin();

        while (inst_list_it != instList[tid].end())
        {
            cprintf("Instruction:%i\n",
                    num);
            if (!(*inst_list_it)->isSquashed()) {
                if (!(*inst_list_it)->isIssued()) {
                    ++valid_num;
                    cprintf("Count:%i\n", valid_num);
                } else if ((*inst_list_it)->isMemRef() &&
                           !(*inst_list_it)->memOpDone) {
                    // Loads that have not been marked as executed still count
                    // towards the total instructions.
                    ++valid_num;
                    cprintf("Count:%i\n", valid_num);
                }
            }

            cprintf("PC:%#x\n[sn:%lli]\n[tid:%i]\n"
                    "Issued:%i\nSquashed:%i\n",
                    (*inst_list_it)->readPC(),
                    (*inst_list_it)->seqNum,
                    (*inst_list_it)->threadNumber,
                    (*inst_list_it)->isIssued(),
                    (*inst_list_it)->isSquashed());

            if ((*inst_list_it)->isMemRef()) {
                cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone);
            }

            cprintf("\n");

            inst_list_it++;
            ++num;
        }
*/
    }
}
