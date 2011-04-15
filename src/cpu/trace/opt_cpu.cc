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
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definition of a memory trace CPU object for optimal caches. Uses a memory
 * trace to access a fully associative cache with optimal replacement.
 */

#include <algorithm> // For heap functions.

#include "cpu/trace/reader/mem_trace_reader.hh"
#include "cpu/trace/opt_cpu.hh"
#include "params/OptCPU.hh"
#include "sim/sim_events.hh"

using namespace std;

OptCPU::OptCPU(const string &name,
               MemTraceReader *_trace,
               int block_size,
               int cache_size,
               int _assoc)
    : SimObject(name), tickEvent(this), trace(_trace),
      numBlks(cache_size/block_size), assoc(_assoc), numSets(numBlks/assoc),
      setMask(numSets - 1)
{
    int log_block_size = 0;
    int tmp_block_size = block_size;
    while (tmp_block_size > 1) {
        ++log_block_size;
        tmp_block_size = tmp_block_size >> 1;
    }
    assert(1<<log_block_size == block_size);
    MemReqPtr req;
    trace->getNextReq(req);
    refInfo.resize(numSets);
    while (req) {
        RefInfo temp;
        temp.addr = req->paddr >> log_block_size;
        int set = temp.addr & setMask;
        refInfo[set].push_back(temp);
        trace->getNextReq(req);
    }

    // Initialize top level of lookup table.
    lookupTable.resize(16);

    // Annotate references with next ref time.
    for (int k = 0; k < numSets; ++k) {
        for (RefIndex i = refInfo[k].size() - 1; i >= 0; --i) {
            Addr addr = refInfo[k][i].addr;
            initTable(addr, InfiniteRef);
            refInfo[k][i].nextRefTime = lookupValue(addr);
            setValue(addr, i);
        }
    }

    // Reset the lookup table
    for (int j = 0; j < 16; ++j) {
        if (lookupTable[j].size() == (1<<16)) {
            for (int k = 0; k < (1<<16); ++k) {
                if (lookupTable[j][k].size() == (1<<16)) {
                    for (int l = 0; l < (1<<16); ++l) {
                        lookupTable[j][k][l] = -1;
                    }
                }
            }
        }
    }

    tickEvent.schedule(0);

    hits = 0;
    misses = 0;
}

void
OptCPU::processSet(int set)
{
    // Initialize cache
    int blks_in_cache = 0;
    RefIndex i = 0;
    cacheHeap.clear();
    cacheHeap.resize(assoc);

    while (blks_in_cache < assoc) {
        RefIndex cache_index = lookupValue(refInfo[set][i].addr);
        if (cache_index == -1) {
            // First reference to this block
            misses++;
            cache_index = blks_in_cache++;
            setValue(refInfo[set][i].addr, cache_index);
        } else {
            hits++;
        }
        // update cache heap to most recent reference
        cacheHeap[cache_index] = i;
        if (++i >= refInfo[set].size()) {
            return;
        }
    }
    for (int start = assoc/2; start >= 0; --start) {
        heapify(set,start);
    }
    //verifyHeap(set,0);

    for (; i < refInfo[set].size(); ++i) {
        RefIndex cache_index = lookupValue(refInfo[set][i].addr);
        if (cache_index == -1) {
            // miss
            misses++;
            // replace from cacheHeap[0]
            // mark replaced block as absent
            setValue(refInfo[set][cacheHeap[0]].addr, -1);
            setValue(refInfo[set][i].addr, 0);
            cacheHeap[0] = i;
            heapify(set, 0);
            // Make sure its in the cache
            assert(lookupValue(refInfo[set][i].addr) != -1);
        } else {
            // hit
            hits++;
            assert(refInfo[set][cacheHeap[cache_index]].addr ==
                   refInfo[set][i].addr);
            assert(refInfo[set][cacheHeap[cache_index]].nextRefTime == i);
            assert(heapLeft(cache_index) >= assoc);

            cacheHeap[cache_index] = i;
            processRankIncrease(set, cache_index);
            assert(lookupValue(refInfo[set][i].addr) != -1);
        }
    }
}
void
OptCPU::tick()
{
    // Do opt simulation

    int references = 0;
    for (int set = 0; set < numSets; ++set) {
        if (!refInfo[set].empty()) {
            processSet(set);
        }
        references += refInfo[set].size();
    }
    // exit;
    fprintf(stderr,"sys.cpu.misses %d #opt cache misses\n",misses);
    fprintf(stderr,"sys.cpu.hits %d #opt cache hits\n", hits);
    fprintf(stderr,"sys.cpu.accesses %d #opt cache acceses\n", references);
    exitSimLoop("end of memory trace reached");
}

void
OptCPU::initTable(Addr addr, RefIndex index)
{
    int l1_index = (addr >> 32) & 0x0f;
    int l2_index = (addr >> 16) & 0xffff;
    assert(l1_index == addr >> 32);
    if (lookupTable[l1_index].size() != (1<<16)) {
        lookupTable[l1_index].resize(1<<16);
    }
    if (lookupTable[l1_index][l2_index].size() != (1<<16)) {
        lookupTable[l1_index][l2_index].resize(1<<16, index);
    }
}

OptCPU::TickEvent::TickEvent(OptCPU *c)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c)
{
}

void
OptCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
OptCPU::TickEvent::description() const
{
    return "OptCPU tick";
}


OptCPU *
OptCPUParams::create()
{
    return new OptCPU(name, data_trace, block_size, size, assoc);
}
