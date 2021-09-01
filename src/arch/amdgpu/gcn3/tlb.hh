/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#ifndef __GPU_TLB_HH__
#define __GPU_TLB_HH__

#include <fstream>
#include <list>
#include <queue>
#include <string>
#include <vector>

#include "arch/generic/tlb.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/regs/segment.hh"
#include "base/callback.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "gpu-compute/compute_unit.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/X86GPUTLB.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class BaseTLB;
class Packet;
class ThreadContext;

namespace X86ISA
{
    class GpuTLB : public ClockedObject
    {
      protected:
        friend class Walker;

        typedef std::list<TlbEntry*> EntryList;

        uint32_t configAddress;

      public:
        typedef X86GPUTLBParams Params;
        GpuTLB(const Params &p);
        ~GpuTLB();

        typedef enum BaseMMU::Mode Mode;

        class Translation
        {
          public:
            virtual ~Translation() { }

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
            virtual void finish(Fault fault, const RequestPtr &req,
                                ThreadContext *tc, Mode mode) = 0;
        };

        void dumpAll();
        TlbEntry *lookup(Addr va, bool update_lru=true);
        void setConfigAddress(uint32_t addr);

      protected:
        EntryList::iterator lookupIt(Addr va, bool update_lru=true);
        Walker *walker;

      public:
        Walker *getWalker();
        void invalidateAll();
        void invalidateNonGlobal();
        void demapPage(Addr va, uint64_t asn);

      protected:
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

        /**
         * Print out accessDistance stats. One stat file
         * per TLB.
         */
        bool accessDistance;

        std::vector<TlbEntry> tlb;

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

        Fault translateInt(bool read, const RequestPtr &req,
                           ThreadContext *tc);

        Fault translate(const RequestPtr &req, ThreadContext *tc,
                Translation *translation, Mode mode, bool &delayedResponse,
                bool timing, int &latency);

      public:
        // latencies for a TLB hit, miss and page fault
        int hitLatency;
        int missLatency1;
        int missLatency2;

        void updatePageFootprint(Addr virt_page_addr);
        void printAccessPattern();


        Fault translateAtomic(const RequestPtr &req, ThreadContext *tc,
                              Mode mode, int &latency);

        void translateTiming(const RequestPtr &req, ThreadContext *tc,
                             Translation *translation, Mode mode,
                             int &latency);

        Tick doMmuRegRead(ThreadContext *tc, Packet *pkt);
        Tick doMmuRegWrite(ThreadContext *tc, Packet *pkt);

        TlbEntry *insert(Addr vpn, TlbEntry &entry);

        // Checkpointing
        virtual void serialize(CheckpointOut& cp) const override;
        virtual void unserialize(CheckpointIn& cp) override;
        void issueTranslation();
        enum tlbOutcome {TLB_HIT, TLB_MISS, PAGE_WALK, MISS_RETURN};
        bool tlbLookup(const RequestPtr &req,
                       ThreadContext *tc, bool update_stats);

        void handleTranslationReturn(Addr addr, tlbOutcome outcome,
                                     PacketPtr pkt);

        void handleFuncTranslationReturn(PacketPtr pkt, tlbOutcome outcome);

        void pagingProtectionChecks(ThreadContext *tc, PacketPtr pkt,
                                    TlbEntry *tlb_entry, Mode mode);

        void updatePhysAddresses(Addr virt_page_addr, TlbEntry *tlb_entry,
                                 Addr phys_page_addr);

        void issueTLBLookup(PacketPtr pkt);

        // CpuSidePort is the TLB Port closer to the CPU/CU side
        class CpuSidePort : public ResponsePort
        {
          public:
            CpuSidePort(const std::string &_name, GpuTLB * gpu_TLB,
                        PortID _index)
                : ResponsePort(_name, gpu_TLB), tlb(gpu_TLB), index(_index) { }

          protected:
            GpuTLB *tlb;
            int index;

            virtual bool recvTimingReq(PacketPtr pkt);
            virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
            virtual void recvFunctional(PacketPtr pkt);
            virtual void recvRangeChange() { }
            virtual void recvReqRetry();
            virtual void recvRespRetry() { panic("recvRespRetry called"); }
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
            MemSidePort(const std::string &_name, GpuTLB * gpu_TLB,
                        PortID _index)
                : RequestPort(_name, gpu_TLB), tlb(gpu_TLB), index(_index) { }

            std::deque<PacketPtr> retries;

          protected:
            GpuTLB *tlb;
            int index;

            virtual bool recvTimingResp(PacketPtr pkt);
            virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
            virtual void recvFunctional(PacketPtr pkt) { }
            virtual void recvRangeChange() { }
            virtual void recvReqRetry();
        };

        // TLB ports on the cpu Side
        std::vector<CpuSidePort*> cpuSidePort;
        // TLB ports on the memory side
        std::vector<MemSidePort*> memSidePort;

        Port &getPort(const std::string &if_name,
                      PortID idx=InvalidPortID) override;

        /**
         * TLB TranslationState: this currently is a somewhat bastardization of
         * the usage of SenderState, whereby the receiver of a packet is not
         * usually supposed to need to look at the contents of the senderState,
         * you're really only supposed to look at what you pushed on, pop it
         * off, and send it back.
         *
         * However, since there is state that we want to pass to the TLBs using
         * the send/recv Timing/Functional/etc. APIs, which don't allow for new
         * arguments, we need a common TLB senderState to pass between TLBs,
         * both "forwards" and "backwards."
         *
         * So, basically, the rule is that any packet received by a TLB port
         * (cpuside OR memside) must be safely castable to a TranslationState.
         */

        struct TranslationState : public Packet::SenderState
        {
            // TLB mode, read or write
            Mode tlbMode;
            // Thread context associated with this req
            ThreadContext *tc;

            /*
            * TLB entry to be populated and passed back and filled in
            * previous TLBs.  Equivalent to the data cache concept of
            * "data return."
            */
            TlbEntry *tlbEntry;
            // Is this a TLB prefetch request?
            bool isPrefetch;
            // When was the req for this translation issued
            uint64_t issueTime;
            // Remember where this came from
            std::vector<ResponsePort*>ports;

            // keep track of #uncoalesced reqs per packet per TLB level;
            // reqCnt per level >= reqCnt higher level
            std::vector<int> reqCnt;
            // TLB level this packet hit in; 0 if it hit in the page table
            int hitLevel;
            Packet::SenderState *saved;

            TranslationState(Mode tlb_mode, ThreadContext *_tc,
                             bool is_prefetch=false,
                             Packet::SenderState *_saved=nullptr)
                : tlbMode(tlb_mode), tc(_tc), tlbEntry(nullptr),
                  isPrefetch(is_prefetch), issueTime(0),
                  hitLevel(0),saved(_saved) { }
        };

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
                TLBEvent(GpuTLB *_tlb, Addr _addr, tlbOutcome outcome,
                        PacketPtr _pkt);

                void process();
                const char *description() const;

                // updateOutcome updates the tlbOutcome of a TLBEvent
                void updateOutcome(tlbOutcome _outcome);
                Addr getTLBEventVaddr();
        };

        std::unordered_map<Addr, TLBEvent*> translationReturnEvent;

        // this FIFO queue keeps track of the virt. page addresses
        // that are pending cleanup
        std::queue<Addr> cleanupQueue;

        // the cleanupEvent is scheduled after a TLBEvent triggers in order to
        // free memory and do the required clean-up
        void cleanup();

        EventFunctionWrapper cleanupEvent;

        /**
         * This hash map will use the virtual page address as a key
         * and will keep track of total number of accesses per page
         */

        struct AccessInfo
        {
            unsigned int lastTimeAccessed; // last access to this page
            unsigned int accessesPerPage;
            // need to divide it by accessesPerPage at the end
            unsigned int totalReuseDistance;

            /**
             * The field below will help us compute the access distance,
             * that is the number of (coalesced) TLB accesses that
             * happened in between each access to this page
             *
             * localTLBAccesses[x] is the value of localTLBNumAccesses
             * when the page <Addr> was accessed for the <x>th time
             */
            std::vector<unsigned int> localTLBAccesses;
            unsigned int sumDistance;
            unsigned int meanDistance;
        };

        typedef std::unordered_map<Addr, AccessInfo> AccessPatternTable;
        AccessPatternTable TLBFootprint;

        // Called at the end of simulation to dump page access stats.
        void exitCallback();

        EventFunctionWrapper exitEvent;

      protected:
        struct GpuTLBStats : public statistics::Group
        {
            GpuTLBStats(statistics::Group *parent);

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
            // from the CU perspective (global)
            statistics::Scalar pageTableCycles;
            statistics::Scalar numUniquePages;
            // from the perspective of this TLB
            statistics::Scalar localCycles;
            // from the perspective of this TLB
            statistics::Formula localLatency;
            // I take the avg. per page and then
            // the avg. over all pages.
            statistics::Scalar avgReuseDistance;
        } stats;
    };
}

using GpuTranslationState = X86ISA::GpuTLB::TranslationState;

} // namespace gem5

#endif // __GPU_TLB_HH__
