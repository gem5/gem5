/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __SALAM_REGISTER_BANK_HH__
#define __SALAM_REGISTER_BANK_HH__

#include <vector>

#include "mem/abstract_mem.hh"
#include "mem/port.hh"
#include "mem/tport.hh"

using namespace gem5;
using namespace memory;

#include "params/RegisterBank.hh"

class RegisterBank : public AbstractMemory
{
  public:
    PARAMS(RegisterBank);
    RegisterBank(const RegisterBankParams &p);
    void registerAccess(PacketPtr pkt);

  private:
    class DeferredPacket
    {
      public:
        const Tick tick;
        const PacketPtr pkt;

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt) {}
    };

    class RegPort : public ResponsePort
    {
      private:
        RegisterBank *memory;

      public:
        RegPort(const std::string &_name, RegisterBank *_memory,
                PortID id = InvalidPortID)
            : ResponsePort(_name, id), memory(_memory)
        {}

      protected:
        Tick
        recvAtomic(PacketPtr pkt) override
        {
            return memory->recvAtomic(pkt);
        };
        Tick
        recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor) override
        {
            return memory->recvAtomicBackdoor(pkt, _backdoor);
        };
        void
        recvFunctional(PacketPtr pkt) override
        {
            memory->recvFunctional(pkt);
        };
        bool
        recvTimingReq(PacketPtr pkt) override
        {
            return memory->recvTimingReq(pkt);
        };
        void
        recvRespRetry() override
        {
            memory->recvRespRetry();
        };
        AddrRangeList
        getAddrRanges() const override
        {
            AddrRangeList ranges;
            ranges.push_back(memory->getAddrRange());
            return ranges;
        }
    };

    RegPort port;

    class LoadPort : public SimpleTimingPort
    {
      private:
        RegisterBank *memory;

      protected:
        Tick
        recvAtomic(PacketPtr pkt)
        {
            memory->registerAccess(pkt);
            return memory->getDeltaTime();
        }
        AddrRangeList
        getAddrRanges() const override
        {
            AddrRangeList ranges;
            ranges.push_back(memory->getAddrRange());
            return ranges;
        }

      public:
        LoadPort(const std::string &_name, RegisterBank *_memory,
                 PortID id = InvalidPortID)
            : SimpleTimingPort(_name, _memory), memory(_memory)
        {}
    };

    LoadPort load;

    /**
     * Container for register deltas. Copied to pmem on delta events.
     */
    uint8_t *deltaAddr;
    /**
     * Latency of a delta cycle in the register bank
     */
    const Tick deltaTime;
    /**
     * Internal (unbounded) storage to mimic the delay caused by the
     * delta timing of writes. Note that this is where the packet spends
     * the memory latency.
     */
    std::list<DeferredPacket> packetQueue;

    /**
     * Remember if we failed to send a response and are awaiting a
     * retry. This is only used as a check.
     */
    bool retryResp;

    /**
     * Dequeue a packet from our internal packet queue and move it to
     * the port where it will be sent as soon as possible.
     */
    void dequeue();
    EventFunctionWrapper dequeueEvent;

    /**
     * Handle the delta cycle of the registers in the bank.
     * Copies updated data at deltaAddr to the storage buffer in pmemAddr.
     * Calls dequeue() to send packet responses.
     */
    void delta();
    EventFunctionWrapper deltaEvent;

  public:
    DrainState drain() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void init() override;
    Tick
    getDeltaTime()
    {
        return deltaTime;
    }

  protected:
    Tick recvAtomic(PacketPtr pkt);
    Tick recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();
};
#endif //__SALAM_REGISTER_BANK_HH__
