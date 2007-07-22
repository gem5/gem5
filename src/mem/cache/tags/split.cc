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
 *
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Definitions of split cache tag store.
 */

#include <string>
#include <iostream>
#include <fstream>

#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "mem/cache/base_cache.hh"
#include "mem/cache/tags/split.hh"
#include "mem/cache/tags/split_lifo.hh"
#include "mem/cache/tags/split_lru.hh"


using namespace std;
using namespace TheISA;

// create and initialize a partitioned cache structure
Split::Split(int _numSets, int _blkSize, int total_ways, int LRU1_assoc,
             bool _lifo, bool _two_queue, int _hit_latency) :
    numSets(_numSets), blkSize(_blkSize), lifo(_lifo), hitLatency(_hit_latency)
{
    DPRINTF(Split, "new split cache!!\n");

    DPRINTF(Split, "lru has %d numSets, %d blkSize, %d assoc, and %d hit_latency\n",
            numSets, blkSize, LRU1_assoc, hitLatency);

    lru = new SplitLRU(_numSets, _blkSize, LRU1_assoc, _hit_latency, 1);

    if (total_ways - LRU1_assoc == 0) {
        lifo_net = NULL;
        lru_net = NULL;
    } else {
        if (lifo) {
            DPRINTF(Split, "Other partition is a LIFO with size %d in bytes. it gets %d ways\n",
                    (total_ways - LRU1_assoc)*_numSets*_blkSize, (total_ways - LRU1_assoc));
            lifo_net = new SplitLIFO(_blkSize, (total_ways - LRU1_assoc)*_numSets*_blkSize,
                                     (total_ways - LRU1_assoc), _hit_latency, _two_queue, 2);
            lru_net = NULL;
        }
        else {
            DPRINTF(Split, "other LRU gets %d ways\n", total_ways - LRU1_assoc);
            lru_net = new SplitLRU(_numSets, _blkSize, total_ways - LRU1_assoc, _hit_latency, 2);
            lifo_net = NULL;
        }
    }

    blkMask = blkSize - 1;

    if (!isPowerOf2(total_ways))
        warn("total cache ways/columns %d should be power of 2",
             total_ways);

    warmedUp = false;
    /** @todo Make warmup percentage a parameter. */
    warmupBound = numSets * total_ways;

}

Split::~Split()
{
    delete lru;
    if (lifo)
        delete lifo_net;
    else
        delete lru_net;
}

void
Split::regStats(const string &name)
{
    using namespace Stats;

    BaseTags::regStats(name);

    usedEvictDist.init(0,3000,40);
    unusedEvictDist.init(0,3000,40);
    useByCPUCycleDist.init(0,35,1);

    nic_repl
        .name(name + ".nic_repl")
        .desc("number of replacements in the nic partition")
        .precision(0)
        ;

    cpu_repl
        .name(name + ".cpu_repl")
        .desc("number of replacements in the cpu partition")
        .precision(0)
        ;

    lru->regStats(name + ".lru");

    if (lifo && lifo_net) {
        lifo_net->regStats(name + ".lifo_net");
    } else if (lru_net) {
        lru_net->regStats(name + ".lru_net");
    }

    nicUsedWhenEvicted
        .name(name + ".nicUsedWhenEvicted")
        .desc("number of NIC blks that were used before evicted")
        ;

    nicUsedTotLatency
        .name(name + ".nicUsedTotLatency")
        .desc("total cycles before eviction of used NIC blks")
        ;

    nicUsedTotEvicted
        .name(name + ".nicUsedTotEvicted")
        .desc("total number of used NIC blks evicted")
        ;

    nicUsedAvgLatency
        .name(name + ".nicUsedAvgLatency")
        .desc("avg number of cycles a used NIC blk is in cache")
        .precision(0)
        ;
    nicUsedAvgLatency = nicUsedTotLatency / nicUsedTotEvicted;

    usedEvictDist
        .name(name + ".usedEvictDist")
        .desc("distribution of used NIC blk eviction times")
        .flags(pdf | cdf)
        ;

    nicUnusedWhenEvicted
        .name(name + ".nicUnusedWhenEvicted")
        .desc("number of NIC blks that were unused when evicted")
        ;

    nicUnusedTotLatency
        .name(name + ".nicUnusedTotLatency")
        .desc("total cycles before eviction of unused NIC blks")
        ;

    nicUnusedTotEvicted
        .name(name + ".nicUnusedTotEvicted")
        .desc("total number of unused NIC blks evicted")
        ;

    nicUnusedAvgLatency
        .name(name + ".nicUnusedAvgLatency")
        .desc("avg number of cycles an unused NIC blk is in cache")
        .precision(0)
        ;
    nicUnusedAvgLatency = nicUnusedTotLatency / nicUnusedTotEvicted;

    unusedEvictDist
        .name(name + ".unusedEvictDist")
        .desc("distribution of unused NIC blk eviction times")
        .flags(pdf | cdf)
        ;

    nicUseByCPUCycleTotal
        .name(name + ".nicUseByCPUCycleTotal")
        .desc("total latency of NIC blks til usage time")
        ;

    nicBlksUsedByCPU
        .name(name + ".nicBlksUsedByCPU")
        .desc("total number of NIC blks used")
        ;

    nicAvgUsageByCPULatency
        .name(name + ".nicAvgUsageByCPULatency")
        .desc("average number of cycles before a NIC blk that is used gets used")
        .precision(0)
        ;
    nicAvgUsageByCPULatency = nicUseByCPUCycleTotal / nicBlksUsedByCPU;

    useByCPUCycleDist
        .name(name + ".useByCPUCycleDist")
        .desc("the distribution of cycle time in cache before NIC blk is used")
        .flags(pdf | cdf)
        ;

    cpuUsedBlks
        .name(name + ".cpuUsedBlks")
        .desc("number of cpu blks that were used before evicted")
        ;

    cpuUnusedBlks
        .name(name + ".cpuUnusedBlks")
        .desc("number of cpu blks that were unused before evicted")
        ;

    nicAvgLatency
        .name(name + ".nicAvgLatency")
        .desc("avg number of cycles a NIC blk is in cache before evicted")
        .precision(0)
        ;
    nicAvgLatency = (nicUnusedTotLatency + nicUsedTotLatency) /
        (nicUnusedTotEvicted + nicUsedTotEvicted);

    NR_CP_hits
        .name(name + ".NR_CP_hits")
        .desc("NIC requests hitting in CPU Partition")
        ;

    NR_NP_hits
        .name(name + ".NR_NP_hits")
        .desc("NIC requests hitting in NIC Partition")
        ;

    CR_CP_hits
        .name(name + ".CR_CP_hits")
        .desc("CPU requests hitting in CPU partition")
        ;

    CR_NP_hits
        .name(name + ".CR_NP_hits")
        .desc("CPU requests hitting in NIC partition")
        ;

}

// probe cache for presence of given block.
bool
Split::probe(Addr addr) const
{
    bool success = lru->probe(addr);
    if (!success) {
        if (lifo && lifo_net)
            success = lifo_net->probe(addr);
        else if (lru_net)
            success = lru_net->probe(addr);
    }

    return success;
}


SplitBlk*
Split::findBlock(Addr addr, int &lat)
{
    SplitBlk *blk = lru->findBlock(addr, lat);
    if (!blk) {
        if (lifo && lifo_net) {
            blk = lifo_net->findBlock(addr, lat);
        } else if (lru_net) {
            blk = lru_net->findBlock(addr, lat);
        }
    }

    return blk;
}

SplitBlk*
Split::findBlock(Addr addr) const
{
    SplitBlk *blk = lru->findBlock(addr);
    if (!blk) {
        if (lifo && lifo_net) {
            blk = lifo_net->findBlock(addr);
        } else if (lru_net) {
            blk = lru_net->findBlock(addr);
        }
    }

    return blk;
}

SplitBlk*
Split::findReplacement(Addr addr, PacketList &writebacks)
{
    SplitBlk *blk = NULL;

    assert(0);
#if 0
    if (pkt->nic_pkt()) {
        DPRINTF(Split, "finding a replacement for nic_req\n");
        nic_repl++;
        if (lifo && lifo_net)
            blk = lifo_net->findReplacement(addr, writebacks);
        else if (lru_net)
            blk = lru_net->findReplacement(addr, writebacks);
        // in this case, this is an LRU only cache, it's non partitioned
        else
            blk = lru->findReplacement(addr, writebacks);
    } else {
        DPRINTF(Split, "finding replacement for cpu_req\n");
        blk = lru->findReplacement(addr, writebacks);
        cpu_repl++;
    }

    Tick latency = curTick - blk->ts;
    if (blk->isNIC) {
        if (blk->isUsed) {
            nicUsedWhenEvicted++;
            usedEvictDist.sample(latency);
            nicUsedTotLatency += latency;
            nicUsedTotEvicted++;
        } else {
            nicUnusedWhenEvicted++;
            unusedEvictDist.sample(latency);
            nicUnusedTotLatency += latency;
            nicUnusedTotEvicted++;
        }
    } else {
        if (blk->isUsed) {
            cpuUsedBlks++;
        } else {
            cpuUnusedBlks++;
        }
    }

    // blk attributes for the new blk coming IN
    blk->ts = curTick;
    blk->isNIC = (pkt->nic_pkt()) ? true : false;
#endif

    return blk;
}

void
Split::invalidateBlk(Split::BlkType *blk)
{
    if (!blk) {
        fatal("FIXME!\n");
#if 0
        if (lifo && lifo_net)
            blk = lifo_net->findBlock(addr);
        else if (lru_net)
            blk = lru_net->findBlock(addr);
#endif

        if (!blk)
            return;
    }

    blk->status = 0;
    blk->isTouched = false;
    tagsInUse--;
}

void
Split::cleanupRefs()
{
    lru->cleanupRefs();
    if (lifo && lifo_net)
        lifo_net->cleanupRefs();
    else if (lru_net)
        lru_net->cleanupRefs();

    ofstream memPrint(simout.resolve("memory_footprint.txt").c_str(),
                      ios::trunc);

    // this shouldn't be here but it happens at the end, which is what i want
    memIter end = memHash.end();
    for (memIter iter = memHash.begin(); iter != end; ++iter) {
        ccprintf(memPrint, "%8x\t%d\n", (*iter).first, (*iter).second);
    }
}

Addr
Split::regenerateBlkAddr(Addr tag, int set) const
{
    if (lifo_net)
        return lifo_net->regenerateBlkAddr(tag, set);
    else
        return lru->regenerateBlkAddr(tag, set);
}

Addr
Split::extractTag(Addr addr) const
{
    // need to fix this if we want to use it... old interface of
    // passing in blk was too weird
    assert(0);
    return 0;
/*
    if (blk->part == 2) {
        if (lifo_net)
            return lifo_net->extractTag(addr);
        else if (lru_net)
            return lru_net->extractTag(addr);
        else
            panic("this shouldn't happen");
    } else
        return lru->extractTag(addr);
*/
}

