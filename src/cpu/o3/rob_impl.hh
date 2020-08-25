/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#ifndef __CPU_O3_ROB_IMPL_HH__
#define __CPU_O3_ROB_IMPL_HH__

#include <list>

#include "base/logging.hh"
#include "cpu/o3/rob.hh"
#include "debug/Fetch.hh"
#include "debug/ROB.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template <class Impl>
ROB<Impl>::ROB(O3CPU *_cpu, DerivO3CPUParams *params)
    : robPolicy(params->smtROBPolicy),
      cpu(_cpu),
      numEntries(params->numROBEntries),
      squashWidth(params->squashWidth),
      numInstsInROB(0),
      numThreads(params->numThreads),
      stats(_cpu)
{
    //Figure out rob policy
    if (robPolicy == SMTQueuePolicy::Dynamic) {
        //Set Max Entries to Total ROB Capacity
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = numEntries;
        }

    } else if (robPolicy == SMTQueuePolicy::Partitioned) {
        DPRINTF(Fetch, "ROB sharing policy set to Partitioned\n");

        //@todo:make work if part_amt doesnt divide evenly.
        int part_amt = numEntries / numThreads;

        //Divide ROB up evenly
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = part_amt;
        }

    } else if (robPolicy == SMTQueuePolicy::Threshold) {
        DPRINTF(Fetch, "ROB sharing policy set to Threshold\n");

        int threshold =  params->smtROBThreshold;;

        //Divide up by threshold amount
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = threshold;
        }
    }

    for (ThreadID tid = numThreads; tid < Impl::MaxThreads; tid++) {
        maxEntries[tid] = 0;
    }

    resetState();
}

template <class Impl>
void
ROB<Impl>::resetState()
{
    for (ThreadID tid = 0; tid  < Impl::MaxThreads; tid++) {
        threadEntries[tid] = 0;
        squashIt[tid] = instList[tid].end();
        squashedSeqNum[tid] = 0;
        doneSquashing[tid] = true;
    }
    numInstsInROB = 0;

    // Initialize the "universal" ROB head & tail point to invalid
    // pointers
    head = instList[0].end();
    tail = instList[0].end();
}

template <class Impl>
std::string
ROB<Impl>::name() const
{
    return cpu->name() + ".rob";
}

template <class Impl>
void
ROB<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    DPRINTF(ROB, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

template <class Impl>
void
ROB<Impl>::drainSanityCheck() const
{
    for (ThreadID tid = 0; tid  < numThreads; tid++)
        assert(instList[tid].empty());
    assert(isEmpty());
}

template <class Impl>
void
ROB<Impl>::takeOverFrom()
{
    resetState();
}

template <class Impl>
void
ROB<Impl>::resetEntries()
{
    if (robPolicy != SMTQueuePolicy::Dynamic || numThreads > 1) {
        auto active_threads = activeThreads->size();

        list<ThreadID>::iterator threads = activeThreads->begin();
        list<ThreadID>::iterator end = activeThreads->end();

        while (threads != end) {
            ThreadID tid = *threads++;

            if (robPolicy == SMTQueuePolicy::Partitioned) {
                maxEntries[tid] = numEntries / active_threads;
            } else if (robPolicy == SMTQueuePolicy::Threshold &&
                       active_threads == 1) {
                maxEntries[tid] = numEntries;
            }
        }
    }
}

template <class Impl>
int
ROB<Impl>::entryAmount(ThreadID num_threads)
{
    if (robPolicy == SMTQueuePolicy::Partitioned) {
        return numEntries / num_threads;
    } else {
        return 0;
    }
}

template <class Impl>
int
ROB<Impl>::countInsts()
{
    int total = 0;

    for (ThreadID tid = 0; tid < numThreads; tid++)
        total += countInsts(tid);

    return total;
}

template <class Impl>
size_t
ROB<Impl>::countInsts(ThreadID tid)
{
    return instList[tid].size();
}

template <class Impl>
void
ROB<Impl>::insertInst(const DynInstPtr &inst)
{
    assert(inst);

    stats.writes++;

    DPRINTF(ROB, "Adding inst PC %s to the ROB.\n", inst->pcState());

    assert(numInstsInROB != numEntries);

    ThreadID tid = inst->threadNumber;

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

template <class Impl>
void
ROB<Impl>::retireHead(ThreadID tid)
{
    stats.writes++;

    assert(numInstsInROB > 0);

    // Get the head ROB instruction by copying it and remove it from the list
    InstIt head_it = instList[tid].begin();

    DynInstPtr head_inst = std::move(*head_it);
    instList[tid].erase(head_it);

    assert(head_inst->readyToCommit());

    DPRINTF(ROB, "[tid:%i] Retiring head instruction, "
            "instruction PC %s, [sn:%llu]\n", tid, head_inst->pcState(),
            head_inst->seqNum);

    --numInstsInROB;
    --threadEntries[tid];

    head_inst->clearInROB();
    head_inst->setCommitted();

    //Update "Global" Head of ROB
    updateHead();

    // @todo: A special case is needed if the instruction being
    // retired is the only instruction in the ROB; otherwise the tail
    // iterator will become invalidated.
    cpu->removeFrontInst(head_inst);
}

template <class Impl>
bool
ROB<Impl>::isHeadReady(ThreadID tid)
{
    stats.reads++;
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
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

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
    return numEntries - numInstsInROB;
}

template <class Impl>
unsigned
ROB<Impl>::numFreeEntries(ThreadID tid)
{
    return maxEntries[tid] - threadEntries[tid];
}

template <class Impl>
void
ROB<Impl>::doSquash(ThreadID tid)
{
    stats.writes++;
    DPRINTF(ROB, "[tid:%i] Squashing instructions until [sn:%llu].\n",
            tid, squashedSeqNum[tid]);

    assert(squashIt[tid] != instList[tid].end());

    if ((*squashIt[tid])->seqNum < squashedSeqNum[tid]) {
        DPRINTF(ROB, "[tid:%i] Done squashing instructions.\n",
                tid);

        squashIt[tid] = instList[tid].end();

        doneSquashing[tid] = true;
        return;
    }

    bool robTailUpdate = false;

    for (int numSquashed = 0;
         numSquashed < squashWidth &&
         squashIt[tid] != instList[tid].end() &&
         (*squashIt[tid])->seqNum > squashedSeqNum[tid];
         ++numSquashed)
    {
        DPRINTF(ROB, "[tid:%i] Squashing instruction PC %s, seq num %i.\n",
                (*squashIt[tid])->threadNumber,
                (*squashIt[tid])->pcState(),
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
    if ((*squashIt[tid])->seqNum <= squashedSeqNum[tid]) {
        DPRINTF(ROB, "[tid:%i] Done squashing instructions.\n",
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
    InstSeqNum lowest_num = 0;
    bool first_valid = true;

    // @todo: set ActiveThreads through ROB or CPU
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (instList[tid].empty())
            continue;

        if (first_valid) {
            head = instList[tid].begin();
            lowest_num = (*head)->seqNum;
            first_valid = false;
            continue;
        }

        InstIt head_thread = instList[tid].begin();

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

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

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
ROB<Impl>::squash(InstSeqNum squash_num, ThreadID tid)
{
    if (isEmpty(tid)) {
        DPRINTF(ROB, "Does not need to squash due to being empty "
                "[sn:%llu]\n",
                squash_num);

        return;
    }

    DPRINTF(ROB, "Starting to squash within the ROB.\n");

    robStatus[tid] = ROBSquashing;

    doneSquashing[tid] = false;

    squashedSeqNum[tid] = squash_num;

    if (!instList[tid].empty()) {
        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        squashIt[tid] = tail_thread;

        doSquash(tid);
    }
}

template <class Impl>
const typename Impl::DynInstPtr&
ROB<Impl>::readHeadInst(ThreadID tid)
{
    if (threadEntries[tid] != 0) {
        InstIt head_thread = instList[tid].begin();

        assert((*head_thread)->isInROB());

        return *head_thread;
    } else {
        return dummyInst;
    }
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readTailInst(ThreadID tid)
{
    InstIt tail_thread = instList[tid].end();
    tail_thread--;

    return *tail_thread;
}

template <class Impl>
ROB<Impl>::ROBStats::ROBStats(Stats::Group *parent)
    : Stats::Group(parent, "rob"),
      ADD_STAT(reads, "The number of ROB reads"),
      ADD_STAT(writes, "The number of ROB writes")
{
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::findInst(ThreadID tid, InstSeqNum squash_inst)
{
    for (InstIt it = instList[tid].begin(); it != instList[tid].end(); it++) {
        if ((*it)->seqNum == squash_inst) {
            return *it;
        }
    }
    return NULL;
}

#endif//__CPU_O3_ROB_IMPL_HH__
