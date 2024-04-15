/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_FU_POOL_HH__
#define __CPU_O3_FU_POOL_HH__

#include <array>
#include <bitset>
#include <list>
#include <string>
#include <vector>

#include "cpu/op_class.hh"
#include "params/FUPool.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class FUDesc;
class FuncUnit;

namespace o3
{

/**
 * Pool of FU's, specific to the new CPU model. The old FU pool had lists of
 * free units and busy units, and whenever a FU was needed it would iterate
 * through the free units to find a FU that provided the capability. This pool
 * has lists of units specific to each of the capabilities, and whenever a FU
 * is needed, it iterates through that list to find a free unit. The previous
 * FU pool would have to be ticked each cycle to update which units became
 * free. This FU pool lets the IEW stage handle freeing units, which frees
 * them as their scheduled execution events complete. This limits units in this
 * model to either have identical issue and op latencies, or 1 cycle issue
 * latencies.
 */
class FUPool : public SimObject
{
  private:
    /** Maximum op execution latencies, per op class. */
    std::array<Cycles, Num_OpClasses> maxOpLatencies;
    /** Whether op is pipelined or not. */
    std::array<bool, Num_OpClasses> pipelined;

    /** Bitvector listing capabilities of this FU pool. */
    std::bitset<Num_OpClasses> capabilityList;

    /** Bitvector listing which FUs are busy. */
    std::vector<bool> unitBusy;

    /** List of units to be freed at the end of this cycle. */
    std::vector<int> unitsToBeFreed;

    /**
     * Class that implements a circular queue to hold FU indices. The hope is
     * that FUs that have been just used will be moved to the end of the queue
     * by iterating through it, thus leaving free units at the head of the
     * queue.
     */
    class FUIdxQueue
    {
      public:
        /** Constructs a circular queue of FU indices. */
        FUIdxQueue() : idx(0), size(0) {}

        /** Adds a FU to the queue. */
        inline void addFU(int fu_idx);

        /** Returns the index of the FU at the head of the queue, and changes
         *  the index to the next element.
         */
        inline int getFU();

      private:
        /** Circular queue index. */
        int idx;

        /** Size of the queue. */
        int size;

        /** Queue of FU indices. */
        std::vector<int> funcUnitsIdx;
    };

    /** Per op class queues of FUs that provide that capability. */
    FUIdxQueue fuPerCapList[Num_OpClasses];

    /** Number of FUs. */
    int numFU;

    /** Functional units. */
    std::vector<FuncUnit *> funcUnits;

    typedef std::vector<FuncUnit *>::iterator fuListIterator;

  public:
    typedef FUPoolParams Params;
    /** Constructs a FU pool. */
    FUPool(const Params &p);
    ~FUPool();

    static constexpr auto NoCapableFU = -2;
    static constexpr auto NoFreeFU = -1;
    /**
     * Gets a FU providing the requested capability. Will mark the
     * unit as busy, but leaves the freeing of the unit up to the IEW
     * stage.
     *
     * @param capability The capability requested.
     * @return Returns NoCapableFU if the FU pool does not have the
     * capability, NoFreeFU if there is no free FU, and the FU's index
     * otherwise.
     */
    int getUnit(OpClass capability);

    /** Frees a FU at the end of this cycle. */
    void freeUnitNextCycle(int fu_idx);

    /** Frees all FUs on the list. */
    void processFreeUnits();

    /** Returns the total number of FUs. */
    int
    size()
    {
        return numFU;
    }

    /** Debugging function used to dump FU information. */
    void dump();

    /** Returns the operation execution latency of the given capability. */
    Cycles
    getOpLatency(OpClass capability)
    {
        return maxOpLatencies[capability];
    }

    /** Returns the issue latency of the given capability. */
    bool
    isPipelined(OpClass capability)
    {
        return pipelined[capability];
    }

    /** Have all the FUs drained? */
    bool isDrained() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom(){};
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_FU_POOL_HH__
