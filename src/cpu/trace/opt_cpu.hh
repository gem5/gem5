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
 * Declaration of a memory trace CPU object for optimal caches. Uses a memory
 * trace to access a fully associative cache with optimal replacement.
 */

#ifndef __CPU_TRACE_OPT_CPU_HH__
#define __CPU_TRACE_OPT_CPU_HH__

#include <vector>

#include "mem/mem_req.hh" // for MemReqPtr
#include "sim/eventq.hh" // for Event
#include "sim/sim_object.hh"

// Forward Declaration
class MemTraceReader;

/**
 * A CPU object to simulate a fully-associative cache with optimal replacement.
 */
class OptCPU : public SimObject
{
  private:
    typedef int RefIndex;

    typedef std::vector<RefIndex> L3Table;
    typedef std::vector<L3Table> L2Table;
    typedef std::vector<L2Table> L1Table;

    /**
     * Event to call OptCPU::tick
     */
    class TickEvent : public Event
    {
      private:
        /** The associated CPU */
        OptCPU *cpu;

      public:
        /**
         * Construct this event;
         */
        TickEvent(OptCPU *c);

        /**
         * Call the tick function.
         */
        void process();

        /**
         * Return a string description of this event.
         */
        const char *description() const;
    };

    TickEvent tickEvent;

    class RefInfo
    {
      public:
        RefIndex nextRefTime;
        Addr addr;
    };

    /** Reference Information, per set. */
    std::vector<std::vector<RefInfo> > refInfo;

    /** Lookup table to track blocks in the cache heap */
    L1Table lookupTable;

    /**
     * Return the correct value in the lookup table.
     */
    RefIndex lookupValue(Addr addr)
    {
        int l1_index = (addr >> 32) & 0x0f;
        int l2_index = (addr >> 16) & 0xffff;
        int l3_index = addr & 0xffff;
        assert(l1_index == addr >> 32);
        return lookupTable[l1_index][l2_index][l3_index];
    }

    /**
     * Set the value in the lookup table.
     */
    void setValue(Addr addr, RefIndex index)
    {
        int l1_index = (addr >> 32) & 0x0f;
        int l2_index = (addr >> 16) & 0xffff;
        int l3_index = addr & 0xffff;
        assert(l1_index == addr >> 32);
        lookupTable[l1_index][l2_index][l3_index]=index;
    }

    /**
     * Initialize the lookup table to the given value.
     */
    void initTable(Addr addr, RefIndex index);

    void heapSwap(int set, int a, int b) {
        RefIndex tmp = cacheHeap[a];
        cacheHeap[a] = cacheHeap[b];
        cacheHeap[b] = tmp;

        setValue(refInfo[set][cacheHeap[a]].addr, a);
        setValue(refInfo[set][cacheHeap[b]].addr, b);
    }

    int heapLeft(int index) { return index + index + 1; }
    int heapRight(int index) { return index + index + 2; }
    int heapParent(int index) { return (index - 1) >> 1; }

    RefIndex heapRank(int set, int index) {
        return refInfo[set][cacheHeap[index]].nextRefTime;
    }

    void heapify(int set, int start){
        int left = heapLeft(start);
        int right = heapRight(start);
        int max = start;
        if (left < assoc && heapRank(set, left) > heapRank(set, start)) {
            max = left;
        }
        if (right < assoc && heapRank(set, right) >  heapRank(set, max)) {
            max = right;
        }

        if (max != start) {
            heapSwap(set, start, max);
            heapify(set, max);
        }
    }

    void verifyHeap(int set, int start) {
        int left = heapLeft(start);
        int right = heapRight(start);

        if (left < assoc) {
            assert(heapRank(set, start) >= heapRank(set, left));
            verifyHeap(set, left);
        }
        if (right < assoc) {
            assert(heapRank(set, start) >= heapRank(set, right));
            verifyHeap(set, right);
        }
    }

    void processRankIncrease(int set, int start) {
        int parent = heapParent(start);
        while (start > 0 && heapRank(set,parent) < heapRank(set,start)) {
            heapSwap(set, parent, start);
            start = parent;
            parent = heapParent(start);
        }
    }

    void processSet(int set);

    static const RefIndex InfiniteRef = 0x7fffffff;

    /** Memory reference trace. */
    MemTraceReader *trace;

    /** Cache heap for replacement. */
    std::vector<RefIndex> cacheHeap;

    /** The number of blocks in the cache. */
    const int numBlks;

    const int assoc;
    const int numSets;
    const int setMask;


    int misses;
    int hits;

  public:
    /**
     * Construct a OptCPU object.
     */
    OptCPU(const std::string &name,
           MemTraceReader *_trace,
           int block_size,
           int cache_size,
           int assoc);

    /**
     * Perform the optimal replacement simulation.
     */
    void tick();
};

#endif // __CPU_TRACE_OPT_CPU_HH__
