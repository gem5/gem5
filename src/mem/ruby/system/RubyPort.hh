/*
 * Copyright (c) 2012-2013,2019 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2009-2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2011 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SYSTEM_RUBYPORT_HH__
#define __MEM_RUBY_SYSTEM_RUBYPORT_HH__

#include <cassert>
#include <string>

#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/protocol/RequestStatus.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/tport.hh"
#include "params/RubyPort.hh"
#include "sim/clocked_object.hh"

class AbstractController;

class RubyPort : public ClockedObject
{
  public:
    class MemRequestPort : public QueuedRequestPort
    {
      private:
        ReqPacketQueue reqQueue;
        SnoopRespPacketQueue snoopRespQueue;

      public:
        MemRequestPort(const std::string &_name, RubyPort *_port);

      protected:
        bool recvTimingResp(PacketPtr pkt);
        void recvRangeChange() {}
    };

    class MemResponsePort : public QueuedResponsePort
    {
      private:
        RespPacketQueue queue;
        bool access_backing_store;
        bool no_retry_on_stall;

      public:
        MemResponsePort(const std::string &_name, RubyPort *_port,
                     bool _access_backing_store,
                     PortID id, bool _no_retry_on_stall);
        void hitCallback(PacketPtr pkt);
        void evictionCallback(Addr address);

      protected:
        bool recvTimingReq(PacketPtr pkt);

        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt);

        AddrRangeList getAddrRanges() const
        { AddrRangeList ranges; return ranges; }

        void addToRetryList();

      private:
        bool isPhysMemAddress(PacketPtr pkt) const;
    };

    class PioRequestPort : public QueuedRequestPort
    {
      private:
        ReqPacketQueue reqQueue;
        SnoopRespPacketQueue snoopRespQueue;

      public:
        PioRequestPort(const std::string &_name, RubyPort *_port);

      protected:
        bool recvTimingResp(PacketPtr pkt);
        void recvRangeChange();
    };

    class PioResponsePort : public QueuedResponsePort
    {
      private:
        RespPacketQueue queue;

      public:
        PioResponsePort(const std::string &_name, RubyPort *_port);

      protected:
        bool recvTimingReq(PacketPtr pkt);

        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt)
        { panic("recvFunctional should never be called on pio response "
                "port!"); }

        AddrRangeList getAddrRanges() const;
    };

    struct SenderState : public Packet::SenderState
    {
        MemResponsePort *port;
        SenderState(MemResponsePort * _port) : port(_port)
        {}
     };

    typedef RubyPortParams Params;
    RubyPort(const Params *p);
    virtual ~RubyPort() {}

    void init() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    virtual RequestStatus makeRequest(PacketPtr pkt) = 0;
    virtual int outstandingCount() const = 0;
    virtual bool isDeadlockEventScheduled() const = 0;
    virtual void descheduleDeadlockEvent() = 0;

    //
    // Called by the controller to give the sequencer a pointer.
    // A pointer to the controller is needed for atomic support.
    //
    void setController(AbstractController* _cntrl) { m_controller = _cntrl; }
    uint32_t getId() { return m_version; }
    DrainState drain() override;

    bool isCPUSequencer() { return m_isCPUSequencer; }

    virtual int functionalWrite(Packet *func_pkt);

  protected:
    void trySendRetries();
    void ruby_hit_callback(PacketPtr pkt);
    void testDrainComplete();
    void ruby_eviction_callback(Addr address);

    /**
     * Called by the PIO port when receiving a timing response.
     *
     * @param pkt Response packet
     * @param request_port_id Port id of the PIO port
     *
     * @return Whether successfully sent
     */
    bool recvTimingResp(PacketPtr pkt, PortID request_port_id);

    RubySystem *m_ruby_system;
    uint32_t m_version;
    AbstractController* m_controller;
    MessageBuffer* m_mandatory_q_ptr;
    bool m_usingRubyTester;
    System* system;

    std::vector<MemResponsePort *> response_ports;

  private:
    bool onRetryList(MemResponsePort * port)
    {
        return (std::find(retryList.begin(), retryList.end(), port) !=
                retryList.end());
    }
    void addToRetryList(MemResponsePort * port)
    {
        if (onRetryList(port)) return;
        retryList.push_back(port);
    }

    PioRequestPort pioRequestPort;
    PioResponsePort pioResponsePort;
    MemRequestPort memRequestPort;
    MemResponsePort memResponsePort;
    unsigned int gotAddrRanges;

    /** Vector of M5 Ports attached to this Ruby port. */
    typedef std::vector<MemResponsePort *>::iterator CpuPortIter;
    std::vector<PioRequestPort *> request_ports;

    //
    // Based on similar code in the M5 bus.  Stores pointers to those ports
    // that should be called when the Sequencer becomes available after a stall.
    //
    std::vector<MemResponsePort *> retryList;

    bool m_isCPUSequencer;
};

#endif // __MEM_RUBY_SYSTEM_RUBYPORT_HH__
