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

#include "mem/ruby/system/DMASequencer.hh"

#include <memory>

#include "debug/RubyDma.hh"
#include "debug/RubyStats.hh"
#include "mem/ruby/protocol/SequencerMsg.hh"
#include "mem/ruby/protocol/SequencerRequestType.hh"
#include "mem/ruby/system/RubySystem.hh"

DMARequest::DMARequest(uint64_t start_paddr, int len, bool write,
                       int bytes_completed, int bytes_issued, uint8_t *data,
                       PacketPtr pkt)
    : start_paddr(start_paddr), len(len), write(write),
      bytes_completed(bytes_completed), bytes_issued(bytes_issued), data(data),
      pkt(pkt)
{
}

DMASequencer::DMASequencer(const Params *p)
    : RubyPort(p), m_outstanding_count(0),
      m_max_outstanding_requests(p->max_outstanding_requests)
{
}

void
DMASequencer::init()
{
    RubyPort::init();
    m_data_block_mask = mask(RubySystem::getBlockSizeBits());

    for (const auto &s_port : slave_ports)
        s_port->sendRangeChange();
}

RequestStatus
DMASequencer::makeRequest(PacketPtr pkt)
{
    if (m_outstanding_count == m_max_outstanding_requests) {
        return RequestStatus_BufferFull;
    }

    Addr paddr = pkt->getAddr();
    uint8_t* data =  pkt->getPtr<uint8_t>();
    int len = pkt->getSize();
    bool write = pkt->isWrite();

    assert(m_outstanding_count < m_max_outstanding_requests);
    Addr line_addr = makeLineAddress(paddr);
    auto emplace_pair =
        m_RequestTable.emplace(std::piecewise_construct,
                               std::forward_as_tuple(line_addr),
                               std::forward_as_tuple(paddr, len, write, 0,
                                                     0, data, pkt));
    DMARequest& active_request = emplace_pair.first->second;

    // This is pretty conservative.  A regular Sequencer with a  more beefy
    // request table that can track multiple requests for a cache line should
    // be used if a more aggressive policy is needed.
    if (!emplace_pair.second) {
            DPRINTF(RubyDma, "DMA aliased: addr %p, len %d\n", line_addr, len);
            return RequestStatus_Aliased;
    }

    DPRINTF(RubyDma, "DMA req created: addr %p, len %d\n", line_addr, len);

    std::shared_ptr<SequencerMsg> msg =
        std::make_shared<SequencerMsg>(clockEdge());
    msg->getPhysicalAddress() = paddr;
    msg->getLineAddress() = line_addr;
    msg->getType() = write ? SequencerRequestType_ST : SequencerRequestType_LD;
    int offset = paddr & m_data_block_mask;

    msg->getLen() = (offset + len) <= RubySystem::getBlockSizeBytes() ?
        len : RubySystem::getBlockSizeBytes() - offset;

    if (write && (data != NULL)) {
        if (active_request.data != NULL) {
            msg->getDataBlk().setData(data, offset, msg->getLen());
        }
    }

    m_outstanding_count++;

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)));
    active_request.bytes_issued += msg->getLen();

    return RequestStatus_Issued;
}

void
DMASequencer::issueNext(const Addr& address)
{
    RequestTable::iterator i = m_RequestTable.find(address);
    assert(i != m_RequestTable.end());

    DMARequest &active_request = i->second;

    assert(m_outstanding_count <= m_max_outstanding_requests);
    active_request.bytes_completed = active_request.bytes_issued;
    if (active_request.len == active_request.bytes_completed) {
        DPRINTF(RubyDma, "DMA request completed: addr %p, size %d\n",
                address, active_request.len);
        m_outstanding_count--;
        PacketPtr pkt = active_request.pkt;
        m_RequestTable.erase(i);
        ruby_hit_callback(pkt);
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
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)));
    active_request.bytes_issued += msg->getLen();
    DPRINTF(RubyDma,
            "DMA request bytes issued %d, bytes completed %d, total len %d\n",
            active_request.bytes_issued, active_request.bytes_completed,
            active_request.len);
}

void
DMASequencer::dataCallback(const DataBlock & dblk, const Addr& address)
{

    RequestTable::iterator i = m_RequestTable.find(address);
    assert(i != m_RequestTable.end());

    DMARequest &active_request = i->second;
    int len = active_request.bytes_issued - active_request.bytes_completed;
    int offset = 0;
    if (active_request.bytes_completed == 0)
        offset = active_request.start_paddr & m_data_block_mask;
    assert(!active_request.write);
    if (active_request.data != NULL) {
        memcpy(&active_request.data[active_request.bytes_completed],
               dblk.getData(offset, len), len);
    }
    issueNext(address);
}

void
DMASequencer::ackCallback(const Addr& address)
{
    assert(m_RequestTable.find(address) != m_RequestTable.end());
    issueNext(address);
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
