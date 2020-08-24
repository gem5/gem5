/*
 * Copyright (c) 2013,2016,2018-2019 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

/**
 * @file
 * Definitions of BaseTags.
 */

#include "mem/cache/tags/base.hh"

#include <cassert>

#include "base/types.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/request.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

BaseTags::BaseTags(const Params *p)
    : ClockedObject(p), blkSize(p->block_size), blkMask(blkSize - 1),
      size(p->size), lookupLatency(p->tag_latency),
      system(p->system), indexingPolicy(p->indexing_policy),
      warmupBound((p->warmup_percentage/100.0) * (p->size / p->block_size)),
      warmedUp(false), numBlocks(p->size / p->block_size),
      dataBlks(new uint8_t[p->size]), // Allocate data storage in one big chunk
      stats(*this)
{
    registerExitCallback([this]() { cleanupRefs(); });
}

ReplaceableEntry*
BaseTags::findBlockBySetAndWay(int set, int way) const
{
    return indexingPolicy->getEntry(set, way);
}

CacheBlk*
BaseTags::findBlock(Addr addr, bool is_secure) const
{
    // Extract block tag
    Addr tag = extractTag(addr);

    // Find possible entries that may contain the given address
    const std::vector<ReplaceableEntry*> entries =
        indexingPolicy->getPossibleEntries(addr);

    // Search for block
    for (const auto& location : entries) {
        CacheBlk* blk = static_cast<CacheBlk*>(location);
        if ((blk->tag == tag) && blk->isValid() &&
            (blk->isSecure() == is_secure)) {
            return blk;
        }
    }

    // Did not find block
    return nullptr;
}

void
BaseTags::insertBlock(const PacketPtr pkt, CacheBlk *blk)
{
    assert(!blk->isValid());

    // Previous block, if existed, has been removed, and now we have
    // to insert the new one

    // Deal with what we are bringing in
    RequestorID requestor_id = pkt->req->requestorId();
    assert(requestor_id < system->maxRequestors());
    stats.occupancies[requestor_id]++;

    // Insert block with tag, src requestor id and task id
    blk->insert(extractTag(pkt->getAddr()), pkt->isSecure(), requestor_id,
                pkt->req->taskId());

    // Check if cache warm up is done
    if (!warmedUp && stats.tagsInUse.value() >= warmupBound) {
        warmedUp = true;
        stats.warmupCycle = curTick();
    }

    // We only need to write into one tag and one data block.
    stats.tagAccesses += 1;
    stats.dataAccesses += 1;
}

Addr
BaseTags::extractTag(const Addr addr) const
{
    return indexingPolicy->extractTag(addr);
}

void
BaseTags::cleanupRefsVisitor(CacheBlk &blk)
{
    if (blk.isValid()) {
        stats.totalRefs += blk.refCount;
        ++stats.sampledRefs;
    }
}

void
BaseTags::cleanupRefs()
{
    forEachBlk([this](CacheBlk &blk) { cleanupRefsVisitor(blk); });
}

void
BaseTags::computeStatsVisitor(CacheBlk &blk)
{
    if (blk.isValid()) {
        assert(blk.task_id < ContextSwitchTaskId::NumTaskId);
        stats.occupanciesTaskId[blk.task_id]++;
        assert(blk.tickInserted <= curTick());
        Tick age = curTick() - blk.tickInserted;

        int age_index;
        if (age / SimClock::Int::us < 10) { // <10us
            age_index = 0;
        } else if (age / SimClock::Int::us < 100) { // <100us
            age_index = 1;
        } else if (age / SimClock::Int::ms < 1) { // <1ms
            age_index = 2;
        } else if (age / SimClock::Int::ms < 10) { // <10ms
            age_index = 3;
        } else
            age_index = 4; // >10ms

        stats.ageTaskId[blk.task_id][age_index]++;
    }
}

void
BaseTags::computeStats()
{
    for (unsigned i = 0; i < ContextSwitchTaskId::NumTaskId; ++i) {
        stats.occupanciesTaskId[i] = 0;
        for (unsigned j = 0; j < 5; ++j) {
            stats.ageTaskId[i][j] = 0;
        }
    }

    forEachBlk([this](CacheBlk &blk) { computeStatsVisitor(blk); });
}

std::string
BaseTags::print()
{
    std::string str;

    auto print_blk = [&str](CacheBlk &blk) {
        if (blk.isValid())
            str += csprintf("\tBlock: %s\n", blk.print());
    };
    forEachBlk(print_blk);

    if (str.empty())
        str = "no valid tags\n";

    return str;
}

BaseTags::BaseTagStats::BaseTagStats(BaseTags &_tags)
    : Stats::Group(&_tags),
    tags(_tags),

    tagsInUse(this, "tagsinuse",
              "Cycle average of tags in use"),
    totalRefs(this, "total_refs",
              "Total number of references to valid blocks."),
    sampledRefs(this, "sampled_refs",
                "Sample count of references to valid blocks."),
    avgRefs(this, "avg_refs",
            "Average number of references to valid blocks."),
    warmupCycle(this, "warmup_cycle",
                "Cycle when the warmup percentage was hit."),
    occupancies(this, "occ_blocks",
                "Average occupied blocks per requestor"),
    avgOccs(this, "occ_percent",
            "Average percentage of cache occupancy"),
    occupanciesTaskId(this, "occ_task_id_blocks",
                      "Occupied blocks per task id"),
    ageTaskId(this, "age_task_id_blocks", "Occupied blocks per task id"),
    percentOccsTaskId(this, "occ_task_id_percent",
                      "Percentage of cache occupancy per task id"),
    tagAccesses(this, "tag_accesses", "Number of tag accesses"),
    dataAccesses(this, "data_accesses", "Number of data accesses")
{
}

void
BaseTags::BaseTagStats::regStats()
{
    using namespace Stats;

    Stats::Group::regStats();

    System *system = tags.system;

    avgRefs = totalRefs / sampledRefs;

    occupancies
        .init(system->maxRequestors())
        .flags(nozero | nonan)
        ;
    for (int i = 0; i < system->maxRequestors(); i++) {
        occupancies.subname(i, system->getRequestorName(i));
    }

    avgOccs.flags(nozero | total);
    for (int i = 0; i < system->maxRequestors(); i++) {
        avgOccs.subname(i, system->getRequestorName(i));
    }

    avgOccs = occupancies / Stats::constant(tags.numBlocks);

    occupanciesTaskId
        .init(ContextSwitchTaskId::NumTaskId)
        .flags(nozero | nonan)
        ;

    ageTaskId
        .init(ContextSwitchTaskId::NumTaskId, 5)
        .flags(nozero | nonan)
        ;

    percentOccsTaskId.flags(nozero);

    percentOccsTaskId = occupanciesTaskId / Stats::constant(tags.numBlocks);
}

void
BaseTags::BaseTagStats::preDumpStats()
{
    Stats::Group::preDumpStats();

    tags.computeStats();
}
