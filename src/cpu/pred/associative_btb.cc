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

#include "cpu/pred/associative_btb.hh"

#include "base/intmath.hh"
#include "base/trace.hh"
#include "debug/BTB.hh"
#include "mem/cache/prefetch/associative_set_impl.hh"

namespace gem5
{

namespace branch_prediction
{

AssociativeBTB::AssociativeBTB(const AssociativeBTBParams &p)
    : BranchTargetBuffer(p),
        btb(p.assoc, p.numEntries, p.indexing_policy,
                p.replacement_policy),
        numEntries(p.numEntries),
        assoc(p.assoc),
        tagBits(p.tagBits), compressedTags(p.useTagCompression),
        numSets(numEntries/assoc),
        setShift(0), setMask(numSets-1),
        tagShift(floorLog2(numSets)),
        instShiftAmt(p.instShiftAmt),
        log2NumThreads(floorLog2(p.numThreads)),
        assocStats(this)
{
    // The number of entries is divided into n ways.
    uint64_t setBits = floorLog2(numSets);

    if (!isPowerOf2(numSets)) {
        fatal("Number of sets is not a power of 2!");
    }

    idxBits = tagBits + setBits;
    idxMask = (idxBits < 64) ? (1ULL << idxBits) - 1 : (uint64_t)(-1);

    DPRINTF(BTB, "BTB: Creating Associative BTB (entries:%i, assoc:%i, "
                "tagBits:%i/comp:%i, idx mask:%x, numSets:%i)\n",
                numEntries, assoc, tagBits, compressedTags, idxMask, numSets);
}

void
AssociativeBTB::memInvalidate()
{
    DPRINTF(BTB, "BTB: Invalidate all entries\n");

    for (auto &entry : btb) {
        entry.invalidate();
    }
}

uint64_t
AssociativeBTB::getIndex(ThreadID tid, Addr instPC)
{
    /**
     * Compute the index into the BTB.
     * - Shift PC over by the word offset
     * - Mask the address to use only the specified number of TAG bits
     *   plus the bits to for the set index.
     *
     *  64                          0
     *  | xxx |   TAG    |  Set  |bb|
     *         \_____BTB idx____/
     *
     * The TID will be hashed with as MSBs of the PC
     */
    uint64_t idx = (instPC >> instShiftAmt)
                 ^ (tid << (idxBits - log2NumThreads -1));

    if (compressedTags) {
        /* For compressed tags the lower 8bit of the tag remain the original
         * the upper bits of the PC are hased together to form the upper
         * 8 bits of the tag.
         * Details https://ieeexplore.ieee.org/document/9528930
         */
        uint64_t tag = (idx >> tagShift);

        uint64_t upper = (tag>>16) ^ (tag>>24) ^ (tag>>32)
                                   ^ (tag>>40) ^ (tag>>48);
        tag ^= upper << 8;
        idx = (tag << tagShift) | (idx & setMask);
    }
    return idx & idxMask;
}

bool
AssociativeBTB::valid(ThreadID tid, Addr instPC, BranchType type)
{
    uint64_t idx = getIndex(tid, instPC);
    BTBEntry * entry = btb.findEntry(idx, /* unused */ false);

    if (entry != nullptr && entry->tid == tid) {
        return true;
    }
    return false;
}

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
const PCStateBase *
AssociativeBTB::lookup(ThreadID tid, Addr instPC, BranchType type)
{
    stats.lookups++;
    if (type != BranchType::NoBranch) {
        stats.lookupType[type]++;
    }


    uint64_t idx = getIndex(tid, instPC);
    BTBEntry * entry = btb.findEntry(idx, /* unused */ false);

    if (entry != nullptr && entry->tid == tid) {
        // PC is different -> conflict hit.
        if (entry->pc != instPC) {
            assocStats.conflict++;
        }

        entry->accesses++;
        btb.accessEntry(entry);
        return entry->target;
    }
    stats.misses++;
    if (type != BranchType::NoBranch) {
        stats.missType[type]++;
    }
    return nullptr;
}

const StaticInstPtr
AssociativeBTB::lookupInst(ThreadID tid, Addr instPC)
{
    uint64_t idx = getIndex(tid, instPC);
    BTBEntry * entry = btb.findEntry(idx, /* unused */ false);

    if (entry != nullptr && entry->tid == tid) {
        return entry->inst;
    }
    return nullptr;
}

void
AssociativeBTB::update(ThreadID tid, Addr instPC,
                    const PCStateBase &target,
                    BranchType type, StaticInstPtr inst)
{
    uint64_t idx = getIndex(tid, instPC);
    BTBEntry * entry = btb.findEntry(idx, /* unused */ false);

    updateEntry(entry, tid, instPC, target, type, inst);
}

void
AssociativeBTB::updateEntry(BTBEntry* &entry, ThreadID tid, Addr instPC,
                    const PCStateBase &target, BranchType type,
                    StaticInstPtr inst)
{
    if (type != BranchType::NoBranch) {
        stats.updates[type]++;
    }

    uint64_t idx = getIndex(tid, instPC);

    if (entry != nullptr && entry->tid == tid) {
        DPRINTF(BTB, "BTB::%s: Updated existing entry. PC:%#x, idx:%#x \n",
                     __func__, instPC, idx);
        btb.accessEntry(entry);
        entry->accesses++;
        if (entry->pc != instPC)
            assocStats.conflict++;

    } else {
        uint64_t set = (idx >> setShift) & setMask;
        DPRINTF(BTB, "BTB::%s: Replace entry. PC:%#x, idx:%#x, set:%i\n",
                     __func__, instPC, idx, set);
        stats.evictions++;
        entry = btb.findVictim(idx);
        assert(entry != nullptr);
        btb.insertEntry(idx, false, entry);

        // Measure the number of accesses.
        assocStats.accesses.sample(entry->accesses == 0 ? 0
                                : floorLog2(entry->accesses));
        entry->accesses = 0;
    }

    entry->pc = instPC;
    entry->tid = tid;
    set(entry->target, &target);
    entry->inst = inst;
}


AssociativeBTB::AssociativeBTBStats::AssociativeBTBStats(
                                                AssociativeBTB *parent)
    : statistics::Group(parent),
    ADD_STAT(accesses, statistics::units::Count::get(),
             "Distribution of accesses (log2) per allocated entry."),
    ADD_STAT(conflict, statistics::units::Ratio::get(),
             "Number of conflicts. Tag hit but PC different.")
{
    using namespace statistics;
    accesses
        .init(8)
        .flags(pdf);
}


} // namespace branch_prediction
} // namespace gem5
