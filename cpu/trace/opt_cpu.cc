
/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 * Definition of a memory trace CPU object for optimal caches. Uses a memory
 * trace to access a fully associative cache with optimal replacement.
 */

#include <algorithm> // For heap functions.

#include "cpu/trace/opt_cpu.hh"
#include "cpu/trace/reader/mem_trace_reader.hh"

#include "sim/builder.hh"
#include "sim/sim_events.hh"

using namespace std;

OptCPU::OptCPU(const string &name,
              MemTraceReader *_trace,
              int log_block_size,
              int cache_size)
    : BaseCPU(name,1), tickEvent(this), trace(_trace),
      numBlks(cache_size/(1<<log_block_size))
{
    MemReqPtr req;
    trace->getNextReq(req);
    assert(log_block_size >= 4);
    assert(refInfo.size() == 0);
    while (req && (refInfo.size() < 60000000)) {
        RefInfo temp;
        temp.addr = req->paddr >> log_block_size;
        refInfo.push_back(temp);
        trace->getNextReq(req);
    }
    // Can't handle more references than "infinity"
    assert(refInfo.size() < InfiniteRef);

    // Initialize top level of lookup table.
    lookupTable.resize(16);

    // Annotate references with next ref time.
    for (RefIndex i = refInfo.size() - 1; i >= 0; --i) {
        Addr addr = refInfo[i].addr;
        initTable(addr, InfiniteRef);
        refInfo[i].nextRefTime = lookupValue(addr);
        setValue(addr, i);
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


    cacheHeap.resize(numBlks);

    tickEvent.schedule(0);

    hits = 0;
    misses = 0;
}

void
OptCPU::tick()
{
    // Do opt simulation

    // Initialize cache
    int blks_in_cache = 0;
    RefIndex i = 0;

    while (blks_in_cache < numBlks) {
        RefIndex cache_index = lookupValue(refInfo[i].addr);
        if (cache_index == -1) {
            // First reference to this block
            misses++;
            cache_index = blks_in_cache++;
            setValue(refInfo[i].addr, cache_index);
        } else {
            hits++;
        }
        // update cache heap to most recent reference
        cacheHeap[cache_index] = i;
        if (++i >= refInfo.size()) {
            // exit
        }
    }
    for (int start = numBlks/2; start >= 0; --start) {
        heapify(start);
    }
    //verifyHeap(0);

    for (; i < refInfo.size(); ++i) {
        RefIndex cache_index = lookupValue(refInfo[i].addr);
        if (cache_index == -1) {
            // miss
            misses++;
            // replace from cacheHeap[0]
            // mark replaced block as absent
            setValue(refInfo[cacheHeap[0]].addr, -1);
            cacheHeap[0] = i;
            heapify(0);
        } else {
            // hit
            hits++;
            assert(refInfo[cacheHeap[cache_index]].addr == refInfo[i].addr);
            assert(refInfo[cacheHeap[cache_index]].nextRefTime == i);
            assert(heapLeft(cache_index) >= numBlks);
        }
        cacheHeap[cache_index] = i;
        processRankIncrease(cache_index);
    }
    // exit;
    fprintf(stderr, "%d, %d, %d\n", misses, hits, refInfo.size());
    new SimExitEvent("Finshed Memory Trace");
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
OptCPU::TickEvent::description()
{
    return "OptCPU tick event";
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(OptCPU)

    SimObjectParam<MemTraceReader *> trace;
    Param<int> size;
    Param<int> log_block_size;

END_DECLARE_SIM_OBJECT_PARAMS(OptCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(OptCPU)

    INIT_PARAM_DFLT(trace, "instruction cache", NULL),
    INIT_PARAM(size, "cache size"),
    INIT_PARAM(log_block_size, "log base 2 of block size")

END_INIT_SIM_OBJECT_PARAMS(OptCPU)

CREATE_SIM_OBJECT(OptCPU)
{
    return new OptCPU(getInstanceName(),
                      trace,
                      log_block_size,
                      size);
}

REGISTER_SIM_OBJECT("OptCPU", OptCPU)
