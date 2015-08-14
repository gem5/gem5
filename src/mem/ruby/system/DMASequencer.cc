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

#include <memory>

#include "debug/Config.hh"
#include "debug/Drain.hh"
#include "debug/RubyDma.hh"
#include "debug/RubyStats.hh"
#include "mem/protocol/SequencerMsg.hh"
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/system/System.hh"
#include "sim/system.hh"

DMASequencer::DMASequencer(const Params *p)
    : MemObject(p), m_ruby_system(p->ruby_system), m_version(p->version),
      m_controller(NULL), m_mandatory_q_ptr(NULL),
      m_usingRubyTester(p->using_ruby_tester),
      slave_port(csprintf("%s.slave", name()), this, 0, p->ruby_system,
                 p->ruby_system->getAccessBackingStore()),
      system(p->system), retry(false)
{
    assert(m_version != -1);
}

void
DMASequencer::init()
{
    MemObject::init();
    assert(m_controller != NULL);
    m_mandatory_q_ptr = m_controller->getMandatoryQueue();
    m_mandatory_q_ptr->setSender(this);
    m_is_busy = false;
    m_data_block_mask = ~ (~0 << RubySystem::getBlockSizeBits());

    slave_port.sendRangeChange();
}

BaseSlavePort &
DMASequencer::getSlavePort(const std::string &if_name, PortID idx)
{
    // used by the CPUs to connect the caches to the interconnect, and
    // for the x86 case also the interrupt master
    if (if_name != "slave") {
        // pass it along to our super class
        return MemObject::getSlavePort(if_name, idx);
    } else {
        return slave_port;
    }
}

DMASequencer::MemSlavePort::MemSlavePort(const std::string &_name,
    DMASequencer *_port, PortID id, RubySystem* _ruby_system,
    bool _access_backing_store)
    : QueuedSlavePort(_name, _port, queue, id), queue(*_port, *this),
      m_ruby_system(_ruby_system), access_backing_store(_access_backing_store)
{
    DPRINTF(RubyDma, "Created slave memport on ruby sequencer %s\n", _name);
}

bool
DMASequencer::MemSlavePort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(RubyDma, "Timing request for address %#x on port %d\n",
            pkt->getAddr(), id);
    DMASequencer *seq = static_cast<DMASequencer *>(&owner);

    if (pkt->memInhibitAsserted())
        panic("DMASequencer should never see an inhibited request\n");

    assert(isPhysMemAddress(pkt->getAddr()));
    assert(getOffset(pkt->getAddr()) + pkt->getSize() <=
           RubySystem::getBlockSizeBytes());

    // Submit the ruby request
    RequestStatus requestStatus = seq->makeRequest(pkt);

    // If the request successfully issued then we should return true.
    // Otherwise, we need to tell the port to retry at a later point
    // and return false.
    if (requestStatus == RequestStatus_Issued) {
        DPRINTF(RubyDma, "Request %s 0x%x issued\n", pkt->cmdString(),
                pkt->getAddr());
        return true;
    }

    // Unless one is using the ruby tester, record the stalled M5 port for
    // later retry when the sequencer becomes free.
    if (!seq->m_usingRubyTester) {
        seq->retry = true;
    }

    DPRINTF(RubyDma, "Request for address %#x did not issued because %s\n",
            pkt->getAddr(), RequestStatus_to_string(requestStatus));

    return false;
}

void
DMASequencer::ruby_hit_callback(PacketPtr pkt)
{
    DPRINTF(RubyDma, "Hit callback for %s 0x%x\n", pkt->cmdString(),
            pkt->getAddr());

    // The packet was destined for memory and has not yet been turned
    // into a response
    assert(system->isMemAddr(pkt->getAddr()));
    assert(pkt->isRequest());
    slave_port.hitCallback(pkt);

    // If we had to stall the slave ports, wake it up because
    // the sequencer likely has free resources now.
    if (retry) {
        retry = false;
        DPRINTF(RubyDma,"Sequencer may now be free.  SendRetry to port %s\n",
                slave_port.name());
        slave_port.sendRetryReq();
    }

    testDrainComplete();
}

void
DMASequencer::testDrainComplete()
{
    //If we weren't able to drain before, we might be able to now.
    if (drainState() == DrainState::Draining) {
        unsigned int drainCount = outstandingCount();
        DPRINTF(Drain, "Drain count: %u\n", drainCount);
        if (drainCount == 0) {
            DPRINTF(Drain, "DMASequencer done draining, signaling drain done\n");
            signalDrainDone();
        }
    }
}

DrainState
DMASequencer::drain()
{
    if (isDeadlockEventScheduled()) {
        descheduleDeadlockEvent();
    }

    // If the DMASequencer is not empty, then it needs to clear all outstanding
    // requests before it should call signalDrainDone()
    DPRINTF(Config, "outstanding count %d\n", outstandingCount());

    // Set status
    if (outstandingCount() > 0) {
        DPRINTF(Drain, "DMASequencer not drained\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

void
DMASequencer::MemSlavePort::hitCallback(PacketPtr pkt)
{
    bool needsResponse = pkt->needsResponse();
    assert(!pkt->isLLSC());
    assert(!pkt->isFlush());

    DPRINTF(RubyDma, "Hit callback needs response %d\n", needsResponse);

    // turn packet around to go back to requester if response expected

    if (access_backing_store) {
        m_ruby_system->getPhysMem()->access(pkt);
    } else if (needsResponse) {
        pkt->makeResponse();
    }

    if (needsResponse) {
        DPRINTF(RubyDma, "Sending packet back over port\n");
        // send next cycle
        DMASequencer *seq = static_cast<DMASequencer *>(&owner);
        RubySystem *rs = seq->m_ruby_system;
        schedTimingResp(pkt, curTick() + rs->clockPeriod());
    } else {
        delete pkt;
    }

    DPRINTF(RubyDma, "Hit callback done!\n");
}

bool
DMASequencer::MemSlavePort::isPhysMemAddress(Addr addr) const
{
    DMASequencer *seq = static_cast<DMASequencer *>(&owner);
    return seq->system->isMemAddr(addr);
}

RequestStatus
DMASequencer::makeRequest(PacketPtr pkt)
{
    if (m_is_busy) {
        return RequestStatus_BufferFull;
    }

    Addr paddr = pkt->getAddr();
    uint8_t* data =  pkt->getPtr<uint8_t>();
    int len = pkt->getSize();
    bool write = pkt->isWrite();

    assert(!m_is_busy);  // only support one outstanding DMA request
    m_is_busy = true;

    active_request.start_paddr = paddr;
    active_request.write = write;
    active_request.data = data;
    active_request.len = len;
    active_request.bytes_completed = 0;
    active_request.bytes_issued = 0;
    active_request.pkt = pkt;

    std::shared_ptr<SequencerMsg> msg =
        std::make_shared<SequencerMsg>(clockEdge());
    msg->getPhysicalAddress() = paddr;
    msg->getLineAddress() = makeLineAddress(msg->getPhysicalAddress());
    msg->getType() = write ? SequencerRequestType_ST : SequencerRequestType_LD;
    int offset = paddr & m_data_block_mask;

    msg->getLen() = (offset + len) <= RubySystem::getBlockSizeBytes() ?
        len : RubySystem::getBlockSizeBytes() - offset;

    if (write && (data != NULL)) {
        if (active_request.data != NULL) {
            msg->getDataBlk().setData(data, offset, msg->getLen());
        }
    }

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg);
    active_request.bytes_issued += msg->getLen();

    return RequestStatus_Issued;
}

void
DMASequencer::issueNext()
{
    assert(m_is_busy);
    active_request.bytes_completed = active_request.bytes_issued;
    if (active_request.len == active_request.bytes_completed) {
        //
        // Must unset the busy flag before calling back the dma port because
        // the callback may cause a previously nacked request to be reissued
        //
        DPRINTF(RubyDma, "DMA request completed\n");
        m_is_busy = false;
        ruby_hit_callback(active_request.pkt);
        return;
    }

    std::shared_ptr<SequencerMsg> msg =
        std::make_shared<SequencerMsg>(clockEdge());
    msg->getPhysicalAddress() = active_request.start_paddr +
                                active_request.bytes_completed;

    assert((msg->getPhysicalAddress() & m_data_block_mask) == 0);
    msg->getLineAddress() = makeLineAddress(msg->getPhysicalAddress());

    msg->getType() = (active_request.write ? SequencerRequestType_ST :
                     SequencerRequestType_LD);

    msg->getLen() =
        (active_request.len -
         active_request.bytes_completed < RubySystem::getBlockSizeBytes() ?
         active_request.len - active_request.bytes_completed :
         RubySystem::getBlockSizeBytes());

    if (active_request.write) {
        msg->getDataBlk().
            setData(&active_request.data[active_request.bytes_completed],
                    0, msg->getLen());
    }

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg);
    active_request.bytes_issued += msg->getLen();
    DPRINTF(RubyDma,
            "DMA request bytes issued %d, bytes completed %d, total len %d\n",
            active_request.bytes_issued, active_request.bytes_completed,
            active_request.len);
}

void
DMASequencer::dataCallback(const DataBlock & dblk)
{
    assert(m_is_busy);
    int len = active_request.bytes_issued - active_request.bytes_completed;
    int offset = 0;
    if (active_request.bytes_completed == 0)
        offset = active_request.start_paddr & m_data_block_mask;
    assert(!active_request.write);
    if (active_request.data != NULL) {
        memcpy(&active_request.data[active_request.bytes_completed],
               dblk.getData(offset, len), len);
    }
    issueNext();
}

void
DMASequencer::ackCallback()
{
    issueNext();
}

void
DMASequencer::recordRequestType(DMASequencerRequestType requestType)
{
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            DMASequencerRequestType_to_string(requestType));
}

DMASequencer *
DMASequencerParams::create()
{
    return new DMASequencer(this);
}
