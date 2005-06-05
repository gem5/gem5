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

#ifndef __CPU_O3_CPU_ROB_IMPL_HH__
#define __CPU_O3_CPU_ROB_IMPL_HH__

#include "cpu/o3/rob.hh"

template <class Impl>
ROB<Impl>::ROB(unsigned _numEntries, unsigned _squashWidth)
    : numEntries(_numEntries),
      squashWidth(_squashWidth),
      numInstsInROB(0),
      squashedSeqNum(0)
{
    doneSquashing = true;
}

template <class Impl>
void
ROB<Impl>::setCPU(FullCPU *cpu_ptr)
{
    cpu = cpu_ptr;

    // Set the tail to the beginning of the CPU instruction list so that
    // upon the first instruction being inserted into the ROB, the tail
    // iterator can simply be incremented.
    tail = cpu->instList.begin();

    // Set the squash iterator to the end of the instruction list.
    squashIt = cpu->instList.end();
}

template <class Impl>
int
ROB<Impl>::countInsts()
{
    // Start at 1; if the tail matches cpu->instList.begin(), then there is
    // one inst in the ROB.
    int return_val = 1;

    // There are quite a few special cases.  Do not use this function other
    // than for debugging purposes.
    if (cpu->instList.begin() == cpu->instList.end()) {
        // In this case there are no instructions in the list.  The ROB
        // must be empty.
        return 0;
    } else if (tail == cpu->instList.end()) {
        // In this case, the tail is not yet pointing to anything valid.
        // The ROB must be empty.
        return 0;
    }

    // Iterate through the ROB from the head to the tail, counting the
    // entries.
    for (InstIt_t i = cpu->instList.begin(); i != tail; ++i)
    {
        assert(i != cpu->instList.end());
        ++return_val;
    }

    return return_val;

    // Because the head won't be tracked properly until the ROB gets the
    // first instruction, and any time that the ROB is empty and has not
    // yet gotten the instruction, this function doesn't work.
//    return numInstsInROB;
}

template <class Impl>
void
ROB<Impl>::insertInst(DynInstPtr &inst)
{
    // Make sure we have the right number of instructions.
    assert(numInstsInROB == countInsts());
    // Make sure the instruction is valid.
    assert(inst);

    DPRINTF(ROB, "ROB: Adding inst PC %#x to the ROB.\n", inst->readPC());

    // If the ROB is full then exit.
    assert(numInstsInROB != numEntries);

    ++numInstsInROB;

    // Increment the tail iterator, moving it one instruction back.
    // There is a special case if the ROB was empty prior to this insertion,
    // in which case the tail will be pointing at instList.end().  If that
    // happens, then reset the tail to the beginning of the list.
    if (tail != cpu->instList.end()) {
        ++tail;
    } else {
        tail = cpu->instList.begin();
    }

    // Make sure the tail iterator is actually pointing at the instruction
    // added.
    assert((*tail) == inst);

    DPRINTF(ROB, "ROB: Now has %d instructions.\n", numInstsInROB);

}

// Whatever calls this function needs to ensure that it properly frees up
// registers prior to this function.
template <class Impl>
void
ROB<Impl>::retireHead()
{
    assert(numInstsInROB == countInsts());
    assert(numInstsInROB > 0);

    // Get the head ROB instruction.
    DynInstPtr head_inst = cpu->instList.front();

    // Make certain this can retire.
    assert(head_inst->readyToCommit());

    DPRINTF(ROB, "ROB: Retiring head instruction of the ROB, "
            "instruction PC %#x, seq num %i\n", head_inst->readPC(),
            head_inst->seqNum);

    // Keep track of how many instructions are in the ROB.
    --numInstsInROB;

    // Tell CPU to remove the instruction from the list of instructions.
    // A special case is needed if the instruction being retired is the
    // only instruction in the ROB; otherwise the tail iterator will become
    // invalidated.
    cpu->removeFrontInst(head_inst);

    if (numInstsInROB == 0) {
        tail = cpu->instList.end();
    }
}

template <class Impl>
bool
ROB<Impl>::isHeadReady()
{
    if (numInstsInROB != 0) {
        return cpu->instList.front()->readyToCommit();
    }

    return false;
}

template <class Impl>
unsigned
ROB<Impl>::numFreeEntries()
{
    assert(numInstsInROB == countInsts());

    return numEntries - numInstsInROB;
}

template <class Impl>
void
ROB<Impl>::doSquash()
{
    DPRINTF(ROB, "ROB: Squashing instructions.\n");

    assert(squashIt != cpu->instList.end());

    for (int numSquashed = 0;
         numSquashed < squashWidth && (*squashIt)->seqNum != squashedSeqNum;
         ++numSquashed)
    {
        // Ensure that the instruction is younger.
        assert((*squashIt)->seqNum > squashedSeqNum);

        DPRINTF(ROB, "ROB: Squashing instruction PC %#x, seq num %i.\n",
                (*squashIt)->readPC(), (*squashIt)->seqNum);

        // Mark the instruction as squashed, and ready to commit so that
        // it can drain out of the pipeline.
        (*squashIt)->setSquashed();

        (*squashIt)->setCanCommit();

        // Special case for when squashing due to a syscall.  It's possible
        // that the squash happened after the head instruction was already
        // committed, meaning that (*squashIt)->seqNum != squashedSeqNum
        // will never be false.  Normally the squash would never be able
        // to go past the head of the ROB; in this case it might, so it
        // must be handled otherwise it will segfault.
#ifndef FULL_SYSTEM
        if (squashIt == cpu->instList.begin()) {
            DPRINTF(ROB, "ROB: Reached head of instruction list while "
                    "squashing.\n");

            squashIt = cpu->instList.end();

            doneSquashing = true;

            return;
        }
#endif

        // Move the tail iterator to the next instruction.
        squashIt--;
    }


    // Check if ROB is done squashing.
    if ((*squashIt)->seqNum == squashedSeqNum) {
        DPRINTF(ROB, "ROB: Done squashing instructions.\n");

        squashIt = cpu->instList.end();

        doneSquashing = true;
    }
}

template <class Impl>
void
ROB<Impl>::squash(InstSeqNum squash_num)
{
    DPRINTF(ROB, "ROB: Starting to squash within the ROB.\n");
    doneSquashing = false;

    squashedSeqNum = squash_num;

    assert(tail != cpu->instList.end());

    squashIt = tail;

    doSquash();
}

template <class Impl>
uint64_t
ROB<Impl>::readHeadPC()
{
    assert(numInstsInROB == countInsts());

    DynInstPtr head_inst = cpu->instList.front();

    return head_inst->readPC();
}

template <class Impl>
uint64_t
ROB<Impl>::readHeadNextPC()
{
    assert(numInstsInROB == countInsts());

    DynInstPtr head_inst = cpu->instList.front();

    return head_inst->readNextPC();
}

template <class Impl>
InstSeqNum
ROB<Impl>::readHeadSeqNum()
{
    // Return the last sequence number that has not been squashed.  Other
    // stages can use it to squash any instructions younger than the current
    // tail.
    DynInstPtr head_inst = cpu->instList.front();

    return head_inst->seqNum;
}

template <class Impl>
uint64_t
ROB<Impl>::readTailPC()
{
    assert(numInstsInROB == countInsts());

    assert(tail != cpu->instList.end());

    return (*tail)->readPC();
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

#endif // __CPU_O3_CPU_ROB_IMPL_HH__
