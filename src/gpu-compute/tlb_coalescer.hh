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

#ifndef __TLB_COALESCER_HH__
#define __TLB_COALESCER_HH__

#include <list>
#include <queue>
#include <string>
#include <vector>

#include "arch/generic/tlb.hh"
#include "arch/isa.hh"
#include "arch/isa_traits.hh"
#include "arch/x86/pagetable.hh"
#include "arch/x86/regs/segment.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "gpu-compute/gpu_tlb.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/TLBCoalescer.hh"
#include "sim/clocked_object.hh"

class BaseTLB;
class Packet;
class ThreadContext;

/**
 * The TLBCoalescer is a ClockedObject sitting on the front side (CPUSide) of
 * each TLB. It receives packets and issues coalesced requests to the
 * TLB below it. It controls how requests are coalesced (the rules)
 * and the permitted number of TLB probes per cycle (i.e., how many
 * coalesced requests it feeds the TLB per cycle).
 */
class TLBCoalescer : public ClockedObject
{
   protected:
    // TLB clock: will inherit clock from shader's clock period in terms
    // of nuber of ticks of curTime (aka global simulation clock)
    // The assignment of TLB clock from shader clock is done in the
    // python config files.
    int clock;

  public:
    typedef TLBCoalescerParams Params;
    TLBCoalescer(const Params *p);
    ~TLBCoalescer() { }

    // Number of TLB probes per cycle. Parameterizable - default 2.
    int TLBProbesPerCycle;

    // Consider coalescing across that many ticks.
    // Paraemterizable - default 1.
    int coalescingWindow;

    // Each coalesced request consists of multiple packets
    // that all fall within the same virtual page
    typedef std::vector<PacketPtr> coalescedReq;

    // disables coalescing when true
    bool disableCoalescing;

    /*
     * This is a hash map with <tick_index> as a key.
     * It contains a vector of coalescedReqs per <tick_index>.
     * Requests are buffered here until they can be issued to
     * the TLB, at which point they are copied to the
     * issuedTranslationsTable hash map.
     *
     * In terms of coalescing, we coalesce requests in a given
     * window of x cycles by using tick_index = issueTime/x as a
     * key, where x = coalescingWindow. issueTime is the issueTime
     * of the pkt from the ComputeUnit's perspective, but another
     * option is to change it to curTick(), so we coalesce based
     * on the receive time.
     */
    typedef std::unordered_map<int64_t, std::vector<coalescedReq>> CoalescingFIFO;

    CoalescingFIFO coalescerFIFO;

    /*
     * issuedTranslationsTabler: a hash_map indexed by virtual page
     * address. Each hash_map entry has a vector of PacketPtr associated
     * with it denoting the different packets that share an outstanding
     * coalesced translation request for the same virtual page.
     *
     * The rules that determine which requests we can coalesce are
     * specified in the canCoalesce() method.
     */
    typedef std::unordered_map<Addr, coalescedReq> CoalescingTable;

    CoalescingTable issuedTranslationsTable;

    // number of packets the coalescer receives
    Stats::Scalar uncoalescedAccesses;
    // number packets the coalescer send to the TLB
    Stats::Scalar coalescedAccesses;

    // Number of cycles the coalesced requests spend waiting in
    // coalescerFIFO. For each packet the coalescer receives we take into
    // account the number of all uncoalesced requests this pkt "represents"
    Stats::Scalar queuingCycles;

    // On average how much time a request from the
    // uncoalescedAccesses that reaches the TLB
    // spends waiting?
    Stats::Scalar localqueuingCycles;
    // localqueuingCycles/uncoalescedAccesses
    Stats::Formula localLatency;

    bool canCoalesce(PacketPtr pkt1, PacketPtr pkt2);
    void updatePhysAddresses(PacketPtr pkt);
    void regStats() override;

    // Clock related functions. Maps to-and-from
    // Simulation ticks and object clocks.
    Tick frequency() const { return SimClock::Frequency / clock; }
    Tick ticks(int numCycles) const { return (Tick)clock * numCycles; }
    Tick curCycle() const { return curTick() / clock; }
    Tick tickToCycles(Tick val) const { return val / clock;}

    class CpuSidePort : public SlavePort
    {
      public:
        CpuSidePort(const std::string &_name, TLBCoalescer *tlb_coalescer,
                    PortID _index)
            : SlavePort(_name, tlb_coalescer), coalescer(tlb_coalescer),
              index(_index) { }

      protected:
        TLBCoalescer *coalescer;
        int index;

        virtual bool recvTimingReq(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
        virtual void recvFunctional(PacketPtr pkt);
        virtual void recvRangeChange() { }
        virtual void recvReqRetry();

        virtual void
        recvRespRetry()
        {
            fatal("recvRespRetry() is not implemented in the TLB coalescer.\n");
        }

        virtual AddrRangeList getAddrRanges() const;
    };

    class MemSidePort : public MasterPort
    {
      public:
        MemSidePort(const std::string &_name, TLBCoalescer *tlb_coalescer,
                    PortID _index)
            : MasterPort(_name, tlb_coalescer), coalescer(tlb_coalescer),
              index(_index) { }

        std::deque<PacketPtr> retries;

      protected:
        TLBCoalescer *coalescer;
        int index;

        virtual bool recvTimingResp(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt) { return 0; }
        virtual void recvFunctional(PacketPtr pkt);
        virtual void recvRangeChange() { }
        virtual void recvReqRetry();

        virtual void
        recvRespRetry()
        {
            fatal("recvRespRetry() not implemented in TLB coalescer");
        }
    };

    // Coalescer slave ports on the cpu Side
    std::vector<CpuSidePort*> cpuSidePort;
    // Coalescer master ports on the memory side
    std::vector<MemSidePort*> memSidePort;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void processProbeTLBEvent();
    /// This event issues the TLB probes
    EventFunctionWrapper probeTLBEvent;

    void processCleanupEvent();
    /// The cleanupEvent is scheduled after a TLBEvent triggers
    /// in order to free memory and do the required clean-up
    EventFunctionWrapper cleanupEvent;

    // this FIFO queue keeps track of the virt. page
    // addresses that are pending cleanup
    std::queue<Addr> cleanupQueue;
};

#endif // __TLB_COALESCER_HH__
