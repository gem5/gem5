/*
 * Copyright (c) 2009 Advanced Micro Devices, Inc.
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

#include "mem/mem_object.hh"
#include "mem/physical.hh"
#include "mem/protocol/RequestStatus.hh"
#include "mem/ruby/libruby.hh"
#include "mem/ruby/system/System.hh"
#include "mem/tport.hh"
#include "params/RubyPort.hh"

class MessageBuffer;
class AbstractController;

class RubyPort : public MemObject
{
  public:
    class M5Port : public SimpleTimingPort
    {
      private:
        RubyPort *ruby_port;
        bool _onRetryList;

      public:
        M5Port(const std::string &_name, RubyPort *_port);
        bool sendTiming(PacketPtr pkt);
        void hitCallback(PacketPtr pkt);
        unsigned deviceBlockSize() const;
        
        bool onRetryList() 
        { return _onRetryList; }
        
        void onRetryList(bool newVal)
        { _onRetryList = newVal; }

      protected:
        virtual bool recvTiming(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt);

      private:
        bool isPhysMemAddress(Addr addr);
    };

    friend class M5Port;

    class PioPort : public SimpleTimingPort
    {
      private:
        RubyPort *ruby_port;

      public:
        PioPort(const std::string &_name, RubyPort *_port);
        bool sendTiming(PacketPtr pkt);

      protected:
        virtual bool recvTiming(PacketPtr pkt);
        virtual Tick recvAtomic(PacketPtr pkt);
    };

    friend class PioPort;

    struct SenderState : public Packet::SenderState
    {
        M5Port* port;
        Packet::SenderState *saved;

        SenderState(M5Port* _port, Packet::SenderState *sender_state = NULL)
            : port(_port), saved(sender_state)
        {}
    };

    typedef RubyPortParams Params;
    RubyPort(const Params *p);
    virtual ~RubyPort() {}

    void init();

    Port *getPort(const std::string &if_name, int idx);

    virtual RequestStatus makeRequest(const RubyRequest & request) = 0;

    //
    // Called by the controller to give the sequencer a pointer.
    // A pointer to the controller is needed for atomic support.
    //
    void setController(AbstractController* _cntrl) { m_controller = _cntrl; }

  protected:
    const std::string m_name;
    void ruby_hit_callback(PacketPtr pkt);
    void hit(PacketPtr pkt);

    int m_version;
    AbstractController* m_controller;
    MessageBuffer* m_mandatory_q_ptr;
    PioPort* pio_port;
    bool m_usingRubyTester;

  private:
    void addToRetryList(M5Port * port)
    {
        if (!port->onRetryList()) {
            port->onRetryList(true);
            retryList.push_back(port);
            waitingOnSequencer = true;
        }
    }

    uint16_t m_port_id;
    uint64_t m_request_cnt;

    M5Port* physMemPort;

    PhysicalMemory* physmem;

    //
    // Based on similar code in the M5 bus.  Stores pointers to those ports
    // that should be called when the Sequencer becomes available after a stall.
    //
    std::list<M5Port*> retryList;

    bool waitingOnSequencer;
};

#endif // __MEM_RUBY_SYSTEM_RUBYPORT_HH__
