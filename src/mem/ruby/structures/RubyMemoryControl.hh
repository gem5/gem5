/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * Copyright (c) 2012 Advanced Micro Devices, Inc.
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

#ifndef __MEM_RUBY_STRUCTURES_MEMORY_CONTROL_HH__
#define __MEM_RUBY_STRUCTURES_MEMORY_CONTROL_HH__

#include <iostream>
#include <list>
#include <string>

#include "mem/abstract_mem.hh"
#include "mem/protocol/MemoryMsg.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/profiler/MemCntrlProfiler.hh"
#include "mem/ruby/structures/MemoryNode.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/RubyMemoryControl.hh"

// This constant is part of the definition of tFAW; see
// the comments in header to RubyMemoryControl.cc
#define ACTIVATE_PER_TFAW 4

//////////////////////////////////////////////////////////////////////////////

class RubyMemoryControl : public AbstractMemory, public Consumer
{
  public:
    typedef RubyMemoryControlParams Params;
    RubyMemoryControl(const Params *p);
    void init() override;
    void reset();

    ~RubyMemoryControl();

    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID) override;
    DrainState drain() override;
    void wakeup() override;

    void setDescription(const std::string& name) { m_description = name; };
    std::string getDescription() { return m_description; };

    // Called from the directory:
    bool recvTimingReq(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    void enqueueMemRef(MemoryNode *memRef);
    bool areNSlotsAvailable(int n) { return true; };  // infinite queue length

    void print(std::ostream& out) const override;
    void regStats() override;

    const int getBank(const Addr addr) const;
    const int getRank(const Addr addr) const;

    // not used in Ruby memory controller
    const int getChannel(const Addr addr) const;
    const int getRow(const Addr addr) const;

    //added by SS
    int getBanksPerRank() { return m_banks_per_rank; };
    int getRanksPerDimm() { return m_ranks_per_dimm; };
    int getDimmsPerChannel() { return m_dimms_per_channel; }

    bool functionalRead(Packet *pkt);
    uint32_t functionalWrite(Packet *pkt);

  private:
    void enqueueToDirectory(MemoryNode *req, Cycles latency);
    const int getRank(int bank) const;
    bool queueReady(int bank);
    void issueRequest(int bank);
    bool issueRefresh(int bank);
    void markTfaw(int rank);
    void executeCycle();

    // Private copy constructor and assignment operator
    RubyMemoryControl (const RubyMemoryControl& obj);
    RubyMemoryControl& operator=(const RubyMemoryControl& obj);

  private:
    // For now, make use of a queued slave port to avoid dealing with
    // flow control for the responses being sent back
    class MemoryPort : public QueuedSlavePort
    {
        RespPacketQueue queue;
        RubyMemoryControl& memory;

      public:
        MemoryPort(const std::string& name, RubyMemoryControl& _memory);

      protected:
        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt);

        bool recvTimingReq(PacketPtr);

        virtual AddrRangeList getAddrRanges() const;
    };

    /**
     * Our incoming port, for a multi-ported controller add a crossbar
     * in front of it
     */
    MemoryPort port;

    // data members
    std::string m_description;
    int m_msg_counter;

    int m_banks_per_rank;
    int m_ranks_per_dimm;
    int m_dimms_per_channel;
    int m_bank_bit_0;
    int m_rank_bit_0;
    int m_dimm_bit_0;
    unsigned int m_bank_queue_size;
    int m_bank_busy_time;
    int m_rank_rank_delay;
    int m_read_write_delay;
    int m_basic_bus_busy_time;
    Cycles m_mem_ctl_latency;
    int m_refresh_period;
    int m_mem_random_arbitrate;
    int m_tFaw;
    Cycles m_mem_fixed_delay;

    int m_total_banks;
    int m_total_ranks;
    int m_refresh_period_system;

    // queues where memory requests live
    std::list<MemoryNode *> m_response_queue;
    std::list<MemoryNode *> m_input_queue;
    std::list<MemoryNode *>* m_bankQueues;

    // Each entry indicates number of address-bus cycles until bank
    // is reschedulable:
    int *m_bankBusyCounter;
    int *m_oldRequest;

    uint64_t *m_tfaw_shift;
    int *m_tfaw_count;

    // Each of these indicates number of address-bus cycles until
    // we can issue a new request of the corresponding type:
    int m_busBusyCounter_Write;
    int m_busBusyCounter_ReadNewRank;
    int m_busBusyCounter_Basic;

    int m_busBusy_WhichRank;  // which rank last granted
    int m_roundRobin;         // which bank queue was last granted
    int m_refresh_count;      // cycles until next refresh
    int m_need_refresh;       // set whenever m_refresh_count goes to zero
    int m_refresh_bank;       // which bank to refresh next
    int m_ageCounter;         // age of old requests; to detect starvation
    int m_idleCount;          // watchdog timer for shutting down

    MemCntrlProfiler *m_profiler_ptr;

    class MemCntrlEvent : public Event
    {
      public:
        MemCntrlEvent(RubyMemoryControl *_mem_cntrl)
        {
            mem_cntrl = _mem_cntrl;
        }
      private:
        void process() { mem_cntrl->wakeup(); }

        RubyMemoryControl* mem_cntrl;
    };

    MemCntrlEvent m_event;
};

std::ostream& operator<<(std::ostream& out, const RubyMemoryControl& obj);

#endif // __MEM_RUBY_STRUCTURES_MEMORY_CONTROL_HH__
