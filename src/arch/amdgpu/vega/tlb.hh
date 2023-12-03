/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_AMDGPU_VEGA_TLB_HH__
#define __ARCH_AMDGPU_VEGA_TLB_HH__

#include <list>
#include <queue>
#include <string>
#include <vector>

#include "arch/amdgpu/vega/pagetable.hh"
#include "arch/generic/mmu.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/VegaGPUTLB.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class BaseMMU;
class Packet;
class AMDGPUDevice;
class ThreadContext;

namespace VegaISA
{

class Walker;

class GpuTLB : public ClockedObject
{
  public:
    GpuTLB(const VegaGPUTLBParams &p);
    ~GpuTLB();

    typedef enum BaseMMU::Mode Mode;

    class Translation
    {
      public:
        virtual ~Translation() {}

        /**
         * Signal that the translation has been delayed due to a hw page
         * table walk.
         */
        virtual void markDelayed() = 0;

        /**
         * The memory for this object may be dynamically allocated, and it
         * may be responsible for cleaning itslef up which will happen in
         * this function. Once it's called the object is no longer valid.
         */
        virtual void finish(Fault fault, const RequestPtr &req, Mode mode) = 0;

        /** This function is used by the page table walker to determine if
         * it should translate the a pending request or if the underlying
         * request has been squashed.
         * @ return Is the instruction that requested this translation
         * squashed?
         */
        virtual bool
        squashed() const
        {
            return false;
        }
    };

    Addr pageAlign(Addr vaddr);
    void dumpAll();
    VegaTlbEntry *lookup(Addr va, bool update_lru = true);

    Walker *getWalker();
    void invalidateAll();
    void demapPage(Addr va, uint64_t asn);

  protected:
    typedef std::list<VegaTlbEntry *> EntryList;
    EntryList::iterator lookupIt(Addr va, bool update_lru = true);
    Walker *walker;
    AMDGPUDevice *gpuDevice;

    int size;
    int assoc;
    int numSets;

    /**
     *  true if this is a fully-associative TLB
     */
    bool FA;
    Addr setMask;

    /**
     * Allocation Policy: true if we always allocate on a hit, false
     * otherwise. Default is true.
     */
    bool allocationPolicy;

    /**
     * if true, then this is not the last level TLB
     */
    bool hasMemSidePort;

    std::vector<VegaTlbEntry> tlb;

    /*
     * It's a per-set list. As long as we have not reached
     * the full capacity of the given set, grab an entry from
     * the freeList.
     */
    std::vector<EntryList> freeList;

    /**
     * An entryList per set is the equivalent of an LRU stack;
     * it's used to guide replacement decisions. The head of the list
     * contains the MRU TLB entry of the given set. If the freeList
     * for this set is empty, the last element of the list
     * is evicted (i.e., dropped on the floor).
     */
    std::vector<EntryList> entryList;

  public:
    // latencies for a TLB hit, miss and page fault
    int hitLatency;
    int missLatency1;
    int missLatency2;

    struct VegaTLBStats : public statistics::Group
    {
        VegaTLBStats(statistics::Group *parent);

        statistics::Scalar maxDownstreamReached;
        statistics::Scalar outstandingReqsMax;

        // local_stats are as seen from the TLB
        // without taking into account coalescing
        statistics::Scalar localNumTLBAccesses;
        statistics::Scalar localNumTLBHits;
        statistics::Scalar localNumTLBMisses;
        statistics::Formula localTLBMissRate;

        // global_stats are as seen from the
        // CU's perspective taking into account
        // all coalesced requests.
        statistics::Scalar globalNumTLBAccesses;
        statistics::Scalar globalNumTLBHits;
        statistics::Scalar globalNumTLBMisses;
        statistics::Formula globalTLBMissRate;

        // from the CU perspective (global)
        statistics::Scalar accessCycles;
        statistics::Scalar pageTableCycles;

        // from the perspective of this TLB
        statistics::Scalar localCycles;
        statistics::Formula localLatency;
    } stats;

    VegaTlbEntry *insert(Addr vpn, VegaTlbEntry &entry);

    // Checkpointing
    virtual void serialize(CheckpointOut &cp) const override;
    virtual void unserialize(CheckpointIn &cp) override;
    void issueTranslation();

    enum tlbOutcome
    {
        TLB_HIT,
        TLB_MISS,
        PAGE_WALK,
        MISS_RETURN
    };

    VegaTlbEntry *tlbLookup(const RequestPtr &req, bool update_stats);

    void walkerResponse(VegaTlbEntry &entry, PacketPtr pkt);
    void handleTranslationReturn(Addr addr, tlbOutcome outcome, PacketPtr pkt);

    void handleFuncTranslationReturn(PacketPtr pkt, tlbOutcome outcome);

    void pagingProtectionChecks(PacketPtr pkt, VegaTlbEntry *tlb_entry,
                                Mode mode);

    void updatePhysAddresses(Addr virt_page_addr, VegaTlbEntry *tlb_entry,
                             Addr phys_page_addr);

    void issueTLBLookup(PacketPtr pkt);

    // CpuSidePort is the TLB Port closer to the CPU/CU side
    class CpuSidePort : public ResponsePort
    {
      public:
        CpuSidePort(const std::string &_name, GpuTLB *gpu_TLB, PortID _index)
            : ResponsePort(_name), tlb(gpu_TLB), index(_index)
        {}

      protected:
        GpuTLB *tlb;
        int index;

        virtual bool recvTimingReq(PacketPtr pkt);

        virtual Tick
        recvAtomic(PacketPtr pkt)
        {
            return 0;
        }

        virtual void recvFunctional(PacketPtr pkt);

        virtual void
        recvRangeChange()
        {}

        virtual void recvReqRetry();

        virtual void
        recvRespRetry()
        {
            panic("recvRespRetry called");
        }

        virtual AddrRangeList getAddrRanges() const;
    };

    /**
     * MemSidePort is the TLB Port closer to the memory side
     * If this is a last level TLB then this port will not be connected.
     *
     * Future action item: if we ever do real page walks, then this port
     * should be connected to a RubyPort.
     */
    class MemSidePort : public RequestPort
    {
      public:
        MemSidePort(const std::string &_name, GpuTLB *gpu_TLB, PortID _index)
            : RequestPort(_name), tlb(gpu_TLB), index(_index)
        {}

        std::deque<PacketPtr> retries;

      protected:
        GpuTLB *tlb;
        int index;

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual Tick
        recvAtomic(PacketPtr pkt)
        {
            return 0;
        }

        virtual void
        recvFunctional(PacketPtr pkt)
        {}

        virtual void
        recvRangeChange()
        {}

        virtual void recvReqRetry();
    };

    // TLB ports on the cpu Side
    std::vector<CpuSidePort *> cpuSidePort;
    // TLB ports on the memory side
    std::vector<MemSidePort *> memSidePort;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    Fault createPagefault(Addr vaddr, Mode mode);

    // maximum number of permitted coalesced requests per cycle
    int maxCoalescedReqs;

    // Current number of outstandings coalesced requests.
    // Should be <= maxCoalescedReqs
    int outstandingReqs;

    /**
     * A TLBEvent is scheduled after the TLB lookup and helps us take the
     * appropriate actions:
     *  (e.g., update TLB on a hit,
     *  send request to lower level TLB on a miss,
     *  or start a page walk if this was the last-level TLB).
     */
    void translationReturn(Addr virtPageAddr, tlbOutcome outcome,
                           PacketPtr pkt);

    class TLBEvent : public Event
    {
      private:
        GpuTLB *tlb;
        Addr virtPageAddr;
        /**
         * outcome can be TLB_HIT, TLB_MISS, or PAGE_WALK
         */
        tlbOutcome outcome;
        PacketPtr pkt;

      public:
        TLBEvent(GpuTLB *_tlb, Addr _addr, tlbOutcome outcome, PacketPtr _pkt);

        void process();
        const char *description() const;

        // updateOutcome updates the tlbOutcome of a TLBEvent
        void updateOutcome(tlbOutcome _outcome);
        Addr getTLBEventVaddr();
    };

    std::unordered_map<Addr, TLBEvent *> translationReturnEvent;

    // this FIFO queue keeps track of the virt. page addresses
    // that are pending cleanup
    std::queue<Addr> cleanupQueue;

    // the cleanupEvent is scheduled after a TLBEvent triggers in order to
    // free memory and do the required clean-up
    void cleanup();

    EventFunctionWrapper cleanupEvent;
};

} // namespace VegaISA

} // namespace gem5

#endif // __ARCH_AMDGPU_VEGA_TLB_HH__
