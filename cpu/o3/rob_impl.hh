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
 */

#include "config/full_system.hh"
#include "cpu/o3/rob.hh"

using namespace std;

template <class Impl>
ROB<Impl>::ROB(unsigned _numEntries, unsigned _squashWidth,
               string _smtROBPolicy, unsigned _smtROBThreshold,
               unsigned _numThreads)
    : numEntries(_numEntries),
      squashWidth(_squashWidth),
      numInstsInROB(0),
      squashedSeqNum(0),
      numThreads(_numThreads)
{
    for (int tid=0; tid  < numThreads; tid++) {
        doneSquashing[tid] = true;
        threadEntries[tid] = 0;
    }

    string policy = _smtROBPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Figure out rob policy
    if (policy == "dynamic") {
        robPolicy = Dynamic;

        //Set Max Entries to Total ROB Capacity
        for (int i = 0; i < numThreads; i++) {
            maxEntries[i]=numEntries;
        }

    } else if (policy == "partitioned") {
        robPolicy = Partitioned;
        DPRINTF(Fetch, "ROB sharing policy set to Partitioned\n");

        //@todo:make work if part_amt doesnt divide evenly.
        int part_amt = numEntries / numThreads;

        //Divide ROB up evenly
        for (int i = 0; i < numThreads; i++) {
            maxEntries[i]=part_amt;
        }

    } else if (policy == "threshold") {
        robPolicy = Threshold;
        DPRINTF(Fetch, "ROB sharing policy set to Threshold\n");

        int threshold =  _smtROBThreshold;;

        //Divide up by threshold amount
        for (int i = 0; i < numThreads; i++) {
            maxEntries[i]=threshold;
        }
    } else {
        assert(0 && "Invalid ROB Sharing Policy.Options Are:{Dynamic,"
                    "Partitioned, Threshold}");
    }
}

template <class Impl>
std::string
ROB<Impl>::name() const
{
    return cpu->name() + ".rob";
}

template <class Impl>
void
ROB<Impl>::setCPU(FullCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    // Set the per-thread iterators to the end of the instruction list.
    for (int i=0; i < numThreads;i++) {
        squashIt[i] = instList[i].end();
    }

    // Initialize the "universal" ROB head & tail point to invalid
    // pointers
    head = instList[0].end();
    tail = instList[0].end();
}

template <class Impl>
void
ROB<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(ROB, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

template <class Impl>
void
ROB<Impl>::switchOut()
{
    for (int tid = 0; tid < numThreads; tid++) {
        instList[tid].clear();
    }
}

template <class Impl>
void
ROB<Impl>::takeOverFrom()
{
    for (int tid=0; tid  < numThreads; tid++) {
        doneSquashing[tid] = true;
        threadEntries[tid] = 0;
        squashIt[tid] = instList[tid].end();
    }
    numInstsInROB = 0;

    // Initialize the "universal" ROB head & tail point to invalid
    // pointers
    head = instList[0].end();
    tail = instList[0].end();
}

template <class Impl>
void
ROB<Impl>::resetEntries()
{
    if (robPolicy != Dynamic || numThreads > 1) {
        int active_threads = (*activeThreads).size();

        list<unsigned>::iterator threads  = (*activeThreads).begin();
        list<unsigned>::iterator list_end = (*activeThreads).end();

        while (threads != list_end) {
            if (robPolicy == Partitioned) {
                maxEntries[*threads++] = numEntries / active_threads;
            } else if (robPolicy == Threshold && active_threads == 1) {
                maxEntries[*threads++] = numEntries;
            }
        }
    }
}

template <class Impl>
int
ROB<Impl>::entryAmount(int num_threads)
{
    if (robPolicy == Partitioned) {
        return numEntries / num_threads;
    } else {
        return 0;
    }
}

template <class Impl>
int
ROB<Impl>::countInsts()
{
    int total=0;

    for (int i=0;i < numThreads;i++)
        total += countInsts(i);

    return total;
}

template <class Impl>
int
ROB<Impl>::countInsts(unsigned tid)
{
    return instList[tid].size();
}

template <class Impl>
void
ROB<Impl>::insertInst(DynInstPtr &inst)
{
    //assert(numInstsInROB == countInsts());
    assert(inst);

    DPRINTF(ROB, "Adding inst PC %#x to the ROB.\n", inst->readPC());

    assert(numInstsInROB != numEntries);

    int tid = inst->threadNumber;

    instList[tid].push_back(inst);

    //Set Up head iterator if this is the 1st instruction in the ROB
    if (numInstsInROB == 0) {
        head = instList[tid].begin();
        assert((*head) == inst);
    }

    //Must Decrement for iterator to actually be valid  since __.end()
    //actually points to 1 after the last inst
    tail = instList[tid].end();
    tail--;

    inst->setInROB();

    ++numInstsInROB;
    ++threadEntries[tid];

    assert((*tail) == inst);

    DPRINTF(ROB, "[tid:%i] Now has %d instructions.\n", tid, threadEntries[tid]);
}

// Whatever calls this function needs to ensure that it properly frees up
// registers prior to this function.
/*
template <class Impl>
void
ROB<Impl>::retireHead()
{
    //assert(numInstsInROB == countInsts());
    assert(numInstsInROB > 0);

    int tid = (*head)->threadNumber;

    retireHead(tid);

    if (numInstsInROB == 0) {
        tail = instList[tid].end();
    }
}
*/

template <class Impl>
void
ROB<Impl>::retireHead(unsigned tid)
{
    //assert(numInstsInROB == countInsts());
    assert(numInstsInROB > 0);

    // Get the head ROB instruction.
    InstIt head_it = instList[tid].begin();

    DynInstPtr head_inst = (*head_it);

    assert(head_inst->readyToCommit());

    DPRINTF(ROB, "[tid:%u]: Retiring head instruction, "
            "instruction PC %#x,[sn:%lli]\n", tid, head_inst->readPC(),
            head_inst->seqNum);

    --numInstsInROB;
    --threadEntries[tid];

    head_inst->removeInROB();
    head_inst->setCommitted();

    instList[tid].erase(head_it);

    //Update "Global" Head of ROB
    updateHead();

    // @todo: A special case is needed if the instruction being
    // retired is the only instruction in the ROB; otherwise the tail
    // iterator will become invalidated.
    cpu->removeFrontInst(head_inst);
}
/*
template <class Impl>
bool
ROB<Impl>::isHeadReady()
{
    if (numInstsInROB != 0) {
        return (*head)->readyToCommit();
    }

    return false;
}
*/
template <class Impl>
bool
ROB<Impl>::isHeadReady(unsigned tid)
{
    if (threadEntries[tid] != 0) {
        return instList[tid].front()->readyToCommit();
    }

    return false;
}

template <class Impl>
bool
ROB<Impl>::canCommit()
{
    //@todo: set ActiveThreads through ROB or CPU
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (isHeadReady(tid)) {
            return true;
        }
    }

    return false;
}

template <class Impl>
unsigned
ROB<Impl>::numFreeEntries()
{
    //assert(numInstsInROB == countInsts());

    return numEntries - numInstsInROB;
}

template <class Impl>
unsigned
ROB<Impl>::numFreeEntries(unsigned tid)
{
    return maxEntries[tid] - threadEntries[tid];
}

template <class Impl>
void
ROB<Impl>::doSquash(unsigned tid)
{
    DPRINTF(ROB, "[tid:%u]: Squashing instructions until [sn:%i].\n",
            tid, squashedSeqNum);

    assert(squashIt[tid] != instList[tid].end());

    if ((*squashIt[tid])->seqNum < squashedSeqNum) {
        DPRINTF(ROB, "[tid:%u]: Done squashing instructions.\n",
                tid);

        squashIt[tid] = instList[tid].end();

        doneSquashing[tid] = true;
        return;
    }

    bool robTailUpdate = false;

    for (int numSquashed = 0;
         numSquashed < squashWidth &&
         squashIt[tid] != instList[tid].end() &&
         (*squashIt[tid])->seqNum > squashedSeqNum;
         ++numSquashed)
    {
        DPRINTF(ROB, "[tid:%u]: Squashing instruction PC %#x, seq num %i.\n",
                (*squashIt[tid])->threadNumber,
                (*squashIt[tid])->readPC(),
                (*squashIt[tid])->seqNum);

        // Mark the instruction as squashed, and ready to commit so that
        // it can drain out of the pipeline.
        (*squashIt[tid])->setSquashed();

        (*squashIt[tid])->setCanCommit();


        if (squashIt[tid] == instList[tid].begin()) {
            DPRINTF(ROB, "Reached head of instruction list while "
                    "squashing.\n");

            squashIt[tid] = instList[tid].end();

            doneSquashing[tid] = true;

            return;
        }

        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        if ((*squashIt[tid]) == (*tail_thread))
            robTailUpdate = true;

        squashIt[tid]--;
    }


    // Check if ROB is done squashing.
    if ((*squashIt[tid])->seqNum <= squashedSeqNum) {
        DPRINTF(ROB, "[tid:%u]: Done squashing instructions.\n",
                tid);

        squashIt[tid] = instList[tid].end();

        doneSquashing[tid] = true;
    }

    if (robTailUpdate) {
        updateTail();
    }
}


template <class Impl>
void
ROB<Impl>::updateHead()
{
    DynInstPtr head_inst;
    InstSeqNum lowest_num = 0;
    bool first_valid = true;

    // @todo: set ActiveThreads through ROB or CPU
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned thread_num = *threads++;

        if (instList[thread_num].empty())
            continue;

        if (first_valid) {
            head = instList[thread_num].begin();
            lowest_num = (*head)->seqNum;
            first_valid = false;
            continue;
        }

        InstIt head_thread = instList[thread_num].begin();

        DynInstPtr head_inst = (*head_thread);

        assert(head_inst != 0);

        if (head_inst->seqNum < lowest_num) {
            head = head_thread;
            lowest_num = head_inst->seqNum;
        }
    }

    if (first_valid) {
        head = instList[0].end();
    }

}

template <class Impl>
void
ROB<Impl>::updateTail()
{
    tail = instList[0].end();
    bool first_valid = true;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        if (instList[tid].empty()) {
            continue;
        }

        // If this is the first valid then assign w/out
        // comparison
        if (first_valid) {
            tail = instList[tid].end();
            tail--;
            first_valid = false;
            continue;
        }

        // Assign new tail if this thread's tail is younger
        // than our current "tail high"
        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        if ((*tail_thread)->seqNum > (*tail)->seqNum) {
            tail = tail_thread;
        }
    }
}


template <class Impl>
void
ROB<Impl>::squash(InstSeqNum squash_num,unsigned tid)
{
    if (isEmpty()) {
        DPRINTF(ROB, "Does not need to squash due to being empty "
                "[sn:%i]\n",
                squash_num);

        return;
    }

    DPRINTF(ROB, "Starting to squash within the ROB.\n");

    robStatus[tid] = ROBSquashing;

    doneSquashing[tid] = false;

    squashedSeqNum = squash_num;

    if (!instList[tid].empty()) {
        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        squashIt[tid] = tail_thread;

        doSquash(tid);
    }
}
/*
template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readHeadInst()
{
    if (numInstsInROB != 0) {
        assert((*head)->isInROB()==true);
        return *head;
    } else {
        return dummyInst;
    }
}
*/
template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readHeadInst(unsigned tid)
{
    if (threadEntries[tid] != 0) {
        InstIt head_thread = instList[tid].begin();

        assert((*head_thread)->isInROB()==true);

        return *head_thread;
    } else {
        return dummyInst;
    }
}
/*
template <class Impl>
uint64_t
ROB<Impl>::readHeadPC()
{
    //assert(numInstsInROB == countInsts());

    DynInstPtr head_inst = *head;

    return head_inst->readPC();
}

template <class Impl>
uint64_t
ROB<Impl>::readHeadPC(unsigned tid)
{
    //assert(numInstsInROB == countInsts());
    InstIt head_thread = instList[tid].begin();

    return (*head_thread)->readPC();
}


template <class Impl>
uint64_t
ROB<Impl>::readHeadNextPC()
{
    //assert(numInstsInROB == countInsts());

    DynInstPtr head_inst = *head;

    return head_inst->readNextPC();
}

template <class Impl>
uint64_t
ROB<Impl>::readHeadNextPC(unsigned tid)
{
    //assert(numInstsInROB == countInsts());
    InstIt head_thread = instList[tid].begin();

    return (*head_thread)->readNextPC();
}

template <class Impl>
InstSeqNum
ROB<Impl>::readHeadSeqNum()
{
    //assert(numInstsInROB == countInsts());
    DynInstPtr head_inst = *head;

    return head_inst->seqNum;
}

template <class Impl>
InstSeqNum
ROB<Impl>::readHeadSeqNum(unsigned tid)
{
    InstIt head_thread = instList[tid].begin();

    return ((*head_thread)->seqNum);
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readTailInst()
{
    //assert(numInstsInROB == countInsts());
    //assert(tail != instList[0].end());

    return (*tail);
}
*/
template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readTailInst(unsigned tid)
{
    //assert(tail_thread[tid] != instList[tid].end());

    InstIt tail_thread = instList[tid].end();
    tail_thread--;

    return *tail_thread;
}

/*
template <class Impl>
uint64_t
ROB<Impl>::readTailPC()
{
    //assert(numInstsInROB == countInsts());

    //assert(tail != instList[0].end());

    return (*tail)->readPC();
}

template <class Impl>
uint64_t
ROB<Impl>::readTailPC(unsigned tid)
{
    //assert(tail_thread[tid] != instList[tid].end());

    InstIt tail_thread = instList[tid].end();
    tail_thread--;

    return (*tail_thread)->readPC();
}

template <class Impl>
InstSeqNum
ROB<Impl>::readTailSeqNum()
{
    // Return the last sequence number that has not been squashed.  Other
    // stages can use it to squash any instructions younger than the current
    // tail.
    return (*tail)->seqNum;
}

template <class Impl>
InstSeqNum
ROB<Impl>::readTailSeqNum(unsigned tid)
{
    // Return the last sequence number that has not been squashed.  Other
    // stages can use it to squash any instructions younger than the current
    // tail.
    //    assert(tail_thread[tid] != instList[tid].end());

    InstIt tail_thread = instList[tid].end();
    tail_thread--;

    return (*tail_thread)->seqNum;
}
*/
