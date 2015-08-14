/*
 * Copyright (c) 2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_SYSTEM_DMASEQUENCER_HH__
#define __MEM_RUBY_SYSTEM_DMASEQUENCER_HH__

#include <memory>
#include <ostream>

#include "mem/mem_object.hh"
#include "mem/protocol/DMASequencerRequestType.hh"
#include "mem/protocol/RequestStatus.hh"
#include "mem/ruby/common/DataBlock.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/simple_mem.hh"
#include "mem/tport.hh"
#include "params/DMASequencer.hh"

class AbstractController;

struct DMARequest
{
    uint64_t start_paddr;
    int len;
    bool write;
    int bytes_completed;
    int bytes_issued;
    uint8_t *data;
    PacketPtr pkt;
};

class DMASequencer : public MemObject
{
  public:
    typedef DMASequencerParams Params;
    DMASequencer(const Params *);
    void init();
    RubySystem *m_ruby_system;

  public:
    class MemSlavePort : public QueuedSlavePort
    {
      private:
        RespPacketQueue queue;
        RubySystem* m_ruby_system;
        bool access_backing_store;

      public:
        MemSlavePort(const std::string &_name, DMASequencer *_port,
                     PortID id, RubySystem *_ruby_system,
                     bool _access_backing_store);
        void hitCallback(PacketPtr pkt);
        void evictionCallback(Addr address);

      protected:
        bool recvTimingReq(PacketPtr pkt);

        Tick recvAtomic(PacketPtr pkt)
        { panic("DMASequencer::MemSlavePort::recvAtomic() not implemented!\n"); }

        void recvFunctional(PacketPtr pkt)
        { panic("DMASequencer::MemSlavePort::recvFunctional() not implemented!\n"); }

        AddrRangeList getAddrRanges() const
        { AddrRangeList ranges; return ranges; }

      private:
        bool isPhysMemAddress(Addr addr) const;
    };

    BaseSlavePort &getSlavePort(const std::string &if_name,
                                PortID idx = InvalidPortID);

    /* external interface */
    RequestStatus makeRequest(PacketPtr pkt);
    bool busy() { return m_is_busy;}
    int outstandingCount() const { return (m_is_busy ? 1 : 0); }
    bool isDeadlockEventScheduled() const { return false; }
    void descheduleDeadlockEvent() {}

    // Called by the controller to give the sequencer a pointer.
    // A pointer to the controller is needed for atomic support.
    void setController(AbstractController* _cntrl) { m_controller = _cntrl; }
    uint32_t getId() { return m_version; }
    DrainState drain() M5_ATTR_OVERRIDE;

    /* SLICC callback */
    void dataCallback(const DataBlock & dblk);
    void ackCallback();

    void recordRequestType(DMASequencerRequestType requestType);

  private:
    void issueNext();
    void ruby_hit_callback(PacketPtr pkt);
    void testDrainComplete();

    /**
     * Called by the PIO port when receiving a timing response.
     *
     * @param pkt Response packet
     * @param master_port_id Port id of the PIO port
     *
     * @return Whether successfully sent
     */
    bool recvTimingResp(PacketPtr pkt, PortID master_port_id);
    unsigned int getChildDrainCount();

  private:
    uint32_t m_version;
    AbstractController* m_controller;
    MessageBuffer* m_mandatory_q_ptr;
    bool m_usingRubyTester;

    MemSlavePort slave_port;

    System* system;

    bool retry;
    bool m_is_busy;
    uint64_t m_data_block_mask;
    DMARequest active_request;
};

#endif // __MEM_RUBY_SYSTEM_DMASEQUENCER_HH__
