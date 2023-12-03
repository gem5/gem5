/*
 * Copyright (c) 2022-2023 The University of Edinburgh
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

#include "cpu/pred/simple_btb.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/BTB.hh"

namespace gem5
{

namespace branch_prediction
{

SimpleBTB::SimpleBTB(const SimpleBTBParams &p)
    : BranchTargetBuffer(p),
      numEntries(p.numEntries),
      tagBits(p.tagBits),
      instShiftAmt(p.instShiftAmt),
      log2NumThreads(floorLog2(p.numThreads))
{
    DPRINTF(BTB, "BTB: Creating BTB object.\n");

    if (!isPowerOf2(numEntries)) {
        fatal("BTB entries is not a power of 2!");
    }

    btb.resize(numEntries);

    for (unsigned i = 0; i < numEntries; ++i) {
        btb[i].valid = false;
    }

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + floorLog2(numEntries);
}

void
SimpleBTB::memInvalidate()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        btb[i].valid = false;
    }
}

inline unsigned
SimpleBTB::getIndex(Addr instPC, ThreadID tid)
{
    // Need to shift PC over by the word offset.
    return ((instPC >> instShiftAmt) ^
            (tid << (tagShiftAmt - instShiftAmt - log2NumThreads))) &
           idxMask;
}

inline Addr
SimpleBTB::getTag(Addr instPC)
{
    return (instPC >> tagShiftAmt) & tagMask;
}

SimpleBTB::BTBEntry *
SimpleBTB::findEntry(Addr instPC, ThreadID tid)
{
    unsigned btb_idx = getIndex(instPC, tid);
    Addr inst_tag = getTag(instPC);

    assert(btb_idx < numEntries);

    if (btb[btb_idx].valid && inst_tag == btb[btb_idx].tag &&
        btb[btb_idx].tid == tid) {
        return &btb[btb_idx];
    }

    return nullptr;
}

bool
SimpleBTB::valid(ThreadID tid, Addr instPC)
{
    BTBEntry *entry = findEntry(instPC, tid);

    return entry != nullptr;
}

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
const PCStateBase *
SimpleBTB::lookup(ThreadID tid, Addr instPC, BranchType type)
{
    stats.lookups[type]++;

    BTBEntry *entry = findEntry(instPC, tid);

    if (entry) {
        return entry->target.get();
    }
    stats.misses[type]++;
    return nullptr;
}

const StaticInstPtr
SimpleBTB::getInst(ThreadID tid, Addr instPC)
{
    BTBEntry *entry = findEntry(instPC, tid);

    if (entry) {
        return entry->inst;
    }
    return nullptr;
}

void
SimpleBTB::update(ThreadID tid, Addr instPC, const PCStateBase &target,
                  BranchType type, StaticInstPtr inst)
{
    unsigned btb_idx = getIndex(instPC, tid);

    assert(btb_idx < numEntries);

    stats.updates[type]++;

    btb[btb_idx].tid = tid;
    btb[btb_idx].valid = true;
    set(btb[btb_idx].target, target);
    btb[btb_idx].tag = getTag(instPC);
    btb[btb_idx].inst = inst;
}

} // namespace branch_prediction
} // namespace gem5
