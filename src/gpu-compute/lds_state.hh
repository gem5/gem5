/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LDS_STATE_HH__
#define __LDS_STATE_HH__

#include <array>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gpu-compute/misc.hh"
#include "mem/port.hh"
#include "params/LdsState.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class ComputeUnit;

/**
 * this represents a slice of the overall LDS, intended to be associated with
 * an individual workgroup
 */
class LdsChunk
{
  public:
    LdsChunk(const uint32_t x_size):
        chunk(x_size)
    {
    }

    LdsChunk() {}

    /**
     * a read operation
     */
    template<class T>
    T
    read(const uint32_t index)
    {
        /**
         * For reads that are outside the bounds of the LDS
         * chunk allocated to this WG we return 0.
         */
        if (index >= chunk.size()) {
            return (T)0;
        }

        T *p0 = (T *) (&(chunk.at(index)));
        return *p0;
    }

    /**
     * a write operation
     */
    template<class T>
    void
    write(const uint32_t index, const T value)
    {
        /**
         * Writes that are outside the bounds of the LDS
         * chunk allocated to this WG are dropped.
         */
        if (index >= chunk.size()) {
            return;
        }

        T *p0 = (T *) (&(chunk.at(index)));
        *p0 = value;
    }

    /**
     * an atomic operation
     */
    template<class T>
    T
    atomic(const uint32_t index, AtomicOpFunctorPtr amoOp)
    {
        /**
         * Atomics that are outside the bounds of the LDS
         * chunk allocated to this WG are dropped.
         */
        if (index >= chunk.size()) {
            return (T)0;
        }
        T *p0 = (T *) (&(chunk.at(index)));
        T tmp = *p0;

       (*amoOp)((uint8_t *)p0);
        return tmp;
    }

    /**
     * get the size of this chunk
     */
    std::vector<uint8_t>::size_type
    size() const
    {
        return chunk.size();
    }

  protected:
    // the actual data store for this slice of the LDS
    std::vector<uint8_t> chunk;
};

// Local Data Share (LDS) State per Wavefront (contents of the LDS region
// allocated to the WorkGroup of this Wavefront)
class LdsState: public ClockedObject
{
  protected:

    /**
     * an event to allow event-driven execution
     */
    class TickEvent: public Event
    {
      protected:

        LdsState *ldsState = nullptr;

        Tick nextTick = 0;

      public:

        TickEvent(LdsState *_ldsState) :
            ldsState(_ldsState)
        {
        }

        virtual void
        process();

        void
        schedule(Tick when)
        {
            mainEventQueue[0]->schedule(this, when);
        }

        void
        deschedule()
        {
            mainEventQueue[0]->deschedule(this);
        }
    };

    /**
     * CuSidePort is the LDS Port closer to the CU side
     */
    class CuSidePort: public ResponsePort
    {
      public:
        CuSidePort(const std::string &_name, LdsState *_ownerLds) :
                ResponsePort(_name), ownerLds(_ownerLds)
        {
        }

      protected:
        LdsState *ownerLds;

        virtual bool
        recvTimingReq(PacketPtr pkt);

        virtual Tick
        recvAtomic(PacketPtr pkt)
        {
          return 0;
        }

        virtual void
        recvFunctional(PacketPtr pkt);

        virtual void
        recvRangeChange()
        {
        }

        virtual void
        recvRetry();

        virtual void
        recvRespRetry();

        virtual AddrRangeList
        getAddrRanges() const
        {
          AddrRangeList ranges;
          ranges.push_back(ownerLds->getAddrRange());
          return ranges;
        }

        template<typename T>
        void
        loadData(PacketPtr packet);

        template<typename T>
        void
        storeData(PacketPtr packet);

        template<typename T>
        void
        atomicOperation(PacketPtr packet);
    };

  protected:

    /**
     * the lds reference counter
     * The key is the workgroup ID and dispatch ID
     * The value is the number of wavefronts that reference this LDS, as
     * wavefronts are launched, the counter goes up for that workgroup and when
     * they return it decreases, once it reaches 0 then this chunk of the LDS
     * is returned to the available pool. However,it is deallocated on the 1->0
     * transition, not whenever the counter is 0 as it always starts with 0
     * when the workgroup asks for space
     */
    std::unordered_map<uint32_t,
                       std::unordered_map<uint32_t, int32_t>> refCounter;

    // the map that allows workgroups to access their own chunk of the LDS
    std::unordered_map<uint32_t,
                       std::unordered_map<uint32_t, LdsChunk>> chunkMap;

    // an event to allow the LDS to wake up at a specified time
    TickEvent tickEvent;

    // the queue of packets that are going back to the CU after a
    // read/write/atomic op
    // TODO need to make this have a maximum size to create flow control
    std::queue<std::pair<Tick, PacketPtr>> returnQueue;

    // whether or not there are pending responses
    bool retryResp = false;

    bool
    process();

    GPUDynInstPtr
    getDynInstr(PacketPtr packet);

    bool
    processPacket(PacketPtr packet);

    unsigned
    countBankConflicts(PacketPtr packet, unsigned *bankAccesses);

    unsigned
    countBankConflicts(GPUDynInstPtr gpuDynInst,
                       unsigned *numBankAccesses);

  public:
    using Params = LdsStateParams;

    LdsState(const Params &params);

    // prevent copy construction
    LdsState(const LdsState&) = delete;

    ~LdsState()
    {
        parent = nullptr;
    }

    bool
    isRetryResp() const
    {
        return retryResp;
    }

    void
    setRetryResp(const bool value)
    {
        retryResp = value;
    }

    // prevent assignment
    LdsState &
    operator=(const LdsState &) = delete;

    /**
     * use the dynamic wave id to create or just increase the reference count
     */
    int
    increaseRefCounter(const uint32_t dispatchId, const uint32_t wgId)
    {
        int refCount = getRefCounter(dispatchId, wgId);
        fatal_if(refCount < 0,
                 "reference count should not be below zero");
        return ++refCounter[dispatchId][wgId];
    }

    /**
     * decrease the reference count after making sure it is in the list
     * give back this chunk if the ref counter has reached 0
     */
    int
    decreaseRefCounter(const uint32_t dispatchId, const uint32_t wgId)
    {
      int refCount = getRefCounter(dispatchId, wgId);

      fatal_if(refCount <= 0,
              "reference count should not be below zero or at zero to"
              "decrement");

      refCounter[dispatchId][wgId]--;

      if (refCounter[dispatchId][wgId] == 0) {
        releaseSpace(dispatchId, wgId);
        return 0;
      } else {
        return refCounter[dispatchId][wgId];
      }
    }

    /**
     * return the current reference count for this workgroup id
     */
    int
    getRefCounter(const uint32_t dispatchId, const uint32_t wgId) const
    {
      auto dispatchIter = chunkMap.find(dispatchId);
      fatal_if(dispatchIter == chunkMap.end(),
               "could not locate this dispatch id [%d]", dispatchId);

      auto workgroup = dispatchIter->second.find(wgId);
      fatal_if(workgroup == dispatchIter->second.end(),
               "could not find this workgroup id within this dispatch id"
               " did[%d] wgid[%d]", dispatchId, wgId);

      auto refCountIter = refCounter.find(dispatchId);
      if (refCountIter == refCounter.end()) {
        fatal("could not locate this dispatch id [%d]", dispatchId);
      } else {
        auto workgroup = refCountIter->second.find(wgId);
        if (workgroup == refCountIter->second.end()) {
          fatal("could not find this workgroup id within this dispatch id"
                  " did[%d] wgid[%d]", dispatchId, wgId);
        } else {
          return refCounter.at(dispatchId).at(wgId);
        }
      }

      fatal("should not reach this point");
      return 0;
    }

    /**
     * assign a parent and request this amount of space be set aside
     * for this wgid
     */
    LdsChunk *
    reserveSpace(const uint32_t dispatchId, const uint32_t wgId,
            const uint32_t size)
    {
        if (chunkMap.find(dispatchId) != chunkMap.end()) {
            panic_if(
                chunkMap[dispatchId].find(wgId) != chunkMap[dispatchId].end(),
                "duplicate workgroup ID asking for space in the LDS "
                "did[%d] wgid[%d]", dispatchId, wgId);
        }

        if (bytesAllocated + size > maximumSize) {
            return nullptr;
        } else {
            bytesAllocated += size;

            auto value = chunkMap[dispatchId].emplace(wgId, LdsChunk(size));
            panic_if(!value.second, "was unable to allocate a new chunkMap");

            // make an entry for this workgroup
            refCounter[dispatchId][wgId] = 0;

            return &chunkMap[dispatchId][wgId];
        }
    }

    /*
     * return pointer to lds chunk for wgid
     */
    LdsChunk *
    getLdsChunk(const uint32_t dispatchId, const uint32_t wgId)
    {
      fatal_if(chunkMap.find(dispatchId) == chunkMap.end(),
          "fetch for unknown dispatch ID did[%d]", dispatchId);

      fatal_if(chunkMap[dispatchId].find(wgId) == chunkMap[dispatchId].end(),
          "fetch for unknown workgroup ID wgid[%d] in dispatch ID did[%d]",
          wgId, dispatchId);

      return &chunkMap[dispatchId][wgId];
    }

    bool
    returnQueuePush(std::pair<Tick, PacketPtr> thePair);

    Tick
    earliestReturnTime() const
    {
        // TODO set to max(lastCommand+1, curTick())
        return returnQueue.empty() ? curTick() : returnQueue.back().first;
    }

    void
    setParent(ComputeUnit *x_parent);

    // accessors
    ComputeUnit *
    getParent() const
    {
        return parent;
    }

    std::string
    getName()
    {
        return _name;
    }

    int
    getBanks() const
    {
        return banks;
    }

    ComputeUnit *
    getComputeUnit() const
    {
        return parent;
    }

    int
    getBankConflictPenalty() const
    {
        return bankConflictPenalty;
    }

    /**
     * get the allocated size for this workgroup
     */
    std::size_t
    ldsSize(const uint32_t x_wgId)
    {
        return chunkMap[x_wgId].size();
    }

    AddrRange
    getAddrRange() const
    {
        return range;
    }

    Port &
    getPort(const std::string &if_name, PortID idx)
    {
        if (if_name == "cuPort") {
            // TODO need to set name dynamically at this point?
            return cuPort;
        } else {
            fatal("cannot resolve the port name " + if_name);
        }
    }

    /**
     * can this much space be reserved for a workgroup?
     */
    bool
    canReserve(uint32_t x_size) const
    {
      return bytesAllocated + x_size <= maximumSize;
    }

  private:
    /**
     * give back the space
     */
    bool
    releaseSpace(const uint32_t x_dispatchId, const uint32_t x_wgId)
    {
        auto dispatchIter = chunkMap.find(x_dispatchId);

        if (dispatchIter == chunkMap.end()) {
          fatal("dispatch id not found [%d]", x_dispatchId);
        } else {
          auto workgroupIter = dispatchIter->second.find(x_wgId);
          if (workgroupIter == dispatchIter->second.end()) {
            fatal("workgroup id [%d] not found in dispatch id [%d]",
                    x_wgId, x_dispatchId);
          }
        }

        fatal_if(bytesAllocated < chunkMap[x_dispatchId][x_wgId].size(),
                 "releasing more space than was allocated");

        bytesAllocated -= chunkMap[x_dispatchId][x_wgId].size();
        chunkMap[x_dispatchId].erase(chunkMap[x_dispatchId].find(x_wgId));
        return true;
    }

    // the port that connects this LDS to its owner CU
    CuSidePort cuPort;

    ComputeUnit* parent = nullptr;

    std::string _name;

    // the number of bytes currently reserved by all workgroups
    int bytesAllocated = 0;

    // the size of the LDS, the most bytes available
    int maximumSize;

    // Address range of this memory
    AddrRange range;

    // the penalty, in cycles, for each LDS bank conflict
    int bankConflictPenalty = 0;

    // the number of banks in the LDS underlying data store
    int banks = 0;
};

} // namespace gem5

#endif // __LDS_STATE_HH__
