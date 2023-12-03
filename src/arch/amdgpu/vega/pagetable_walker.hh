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

#ifndef __DEV_AMDGPU_PAGETABLE_WALKER_HH__
#define __DEV_AMDGPU_PAGETABLE_WALKER_HH__

#include <vector>

#include "arch/amdgpu/vega/pagetable.hh"
#include "arch/amdgpu/vega/tlb.hh"
#include "base/types.hh"
#include "debug/GPUPTWalker.hh"
#include "mem/packet.hh"
#include "params/VegaPagetableWalker.hh"
#include "sim/clocked_object.hh"
#include "sim/system.hh"

namespace gem5
{

class ThreadContext;

namespace VegaISA
{

class Walker : public ClockedObject
{
  protected:
    // Port for accessing memory
    class WalkerPort : public RequestPort
    {
      public:
        WalkerPort(const std::string &_name, Walker *_walker)
            : RequestPort(_name), walker(_walker)
        {}

      protected:
        Walker *walker;

        bool recvTimingResp(PacketPtr pkt);
        void recvReqRetry();
    };

    friend class WalkerPort;
    WalkerPort port;

    // State to track each walk of the page table
    class WalkerState
    {
        friend class Walker;

      private:
        enum State
        {
            Ready,
            Waiting,
            PDE2,
            PDE1,
            PDE0,
            PTE
        };

      protected:
        Walker *walker;
        State state;
        State nextState;
        int dataSize;
        bool enableNX;
        VegaTlbEntry entry;
        PacketPtr read;
        Fault timingFault;
        BaseMMU::Mode mode;
        bool retrying;
        bool started;
        bool timing;
        PacketPtr tlbPkt;
        int blockFragmentSize;

      public:
        WalkerState(Walker *_walker, PacketPtr pkt, bool is_functional = false)
            : walker(_walker),
              state(Ready),
              nextState(Ready),
              dataSize(8),
              enableNX(true),
              retrying(false),
              started(false),
              tlbPkt(pkt),
              blockFragmentSize(0)
        {
            DPRINTF(GPUPTWalker, "Walker::WalkerState %p %p %d\n", this,
                    walker, state);
        }

        void initState(BaseMMU::Mode _mode, Addr baseAddr, Addr vaddr,
                       bool is_functional = false);
        void startWalk();
        Fault startFunctional(Addr base, Addr vaddr, PageTableEntry &pte,
                              unsigned &logBytes);

        bool isRetrying();
        void retry();

        std::string
        name() const
        {
            return walker->name();
        }

        Walker *
        getWalker() const
        {
            return walker;
        }

      private:
        Fault stepWalk();
        void stepTimingWalk();
        void walkStateMachine(PageTableEntry &pte, Addr &nextRead,
                              bool &doEndWalk, Fault &fault);
        void sendPackets();
        void endWalk();
        Fault pageFault(bool present);
        uint64_t offsetFunc(Addr logicalAddr, int top, int lsb);
    };

    friend class WalkerState;
    // State for timing and atomic accesses (need multiple per walker in
    // the case of multiple outstanding requests in timing mode)
    std::list<WalkerState *> currStates;
    // State for functional accesses (only need one of these per walker)
    WalkerState funcState;

    struct WalkerSenderState : public Packet::SenderState
    {
        WalkerState *senderWalk;

        WalkerSenderState(WalkerState *_senderWalk) : senderWalk(_senderWalk)
        {}
    };

  public:
    // Kick off the state machine.
    void startTiming(PacketPtr pkt, Addr base, Addr vaddr, BaseMMU::Mode mode);
    Fault startFunctional(Addr base, Addr vaddr, PageTableEntry &pte,
                          unsigned &logBytes, BaseMMU::Mode mode);
    Fault startFunctional(Addr base, Addr &addr, unsigned &logBytes,
                          BaseMMU::Mode mode, bool &isSystem);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    Addr
    getBaseAddr() const
    {
        return baseAddr;
    }

    void
    setBaseAddr(Addr ta)
    {
        baseAddr = ta;
    }

    void
    setDevRequestor(RequestorID mid)
    {
        deviceRequestorId = mid;
    }

    RequestorID
    getDevRequestor() const
    {
        return deviceRequestorId;
    }

  protected:
    // The TLB we're supposed to load.
    GpuTLB *tlb;
    RequestorID requestorId;

    // Base address set by MAP_PROCESS packet
    Addr baseAddr;
    RequestorID deviceRequestorId;

    // Functions for dealing with packets.
    void recvTimingResp(PacketPtr pkt);
    void recvReqRetry();
    bool sendTiming(WalkerState *sendingState, PacketPtr pkt);

    void walkerResponse(WalkerState *state, VegaTlbEntry &entry,
                        PacketPtr pkt);

    // System pointer for functional accesses
    System *system;

  public:
    void
    setTLB(GpuTLB *_tlb)
    {
        assert(tlb == nullptr); // only set it once
        tlb = _tlb;
    }

    Walker(const VegaPagetableWalkerParams &p)
        : ClockedObject(p),
          port(name() + ".port", this),
          funcState(this, nullptr, true),
          tlb(nullptr),
          requestorId(p.system->getRequestorId(this)),
          deviceRequestorId(999),
          system(p.system)
    {
        DPRINTF(GPUPTWalker, "Walker::Walker %p\n", this);
    }
};

} // namespace VegaISA
} // namespace gem5

#endif // __DEV_AMDGPU_PAGETABLE_WALKER_HH__
