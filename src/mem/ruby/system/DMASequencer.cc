/*
 * Copyright (c) 2021 ARM Limited
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

namespace gem5
{

namespace ruby
{

DMARequest::DMARequest(uint64_t start_paddr, int len, bool write,
                       int bytes_completed, int bytes_issued, uint8_t *data,
                       PacketPtr pkt)
    : start_paddr(start_paddr), len(len), write(write),
      bytes_completed(bytes_completed), bytes_issued(bytes_issued), data(data),
      pkt(pkt)
{
}

DMASequencer::DMASequencer(const Params &p)
    : RubyPort(p), m_outstanding_count(0),
      m_max_outstanding_requests(p.max_outstanding_requests)
{
}

void
DMASequencer::init()
{
    RubyPort::init();
    m_data_block_mask = mask(m_ruby_system->getBlockSizeBits());
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

    // Should DMA be allowed to generate this ?
    assert(!pkt->isMaskedWrite());

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

    int blk_size = m_ruby_system->getBlockSizeBytes();

    std::shared_ptr<SequencerMsg> msg =
        std::make_shared<SequencerMsg>(clockEdge(), blk_size);
    msg->getPhysicalAddress() = paddr;
    msg->getLineAddress() = line_addr;

    if (pkt->req->isAtomic()) {
        msg->setType(SequencerRequestType_ATOMIC);

        // While regular LD/ST can support DMAs spanning multiple cache lines,
        // atomic requests are only supported within a single cache line. The
        // atomic request will end upon atomicCallback and not call issueNext.
        int block_size = m_ruby_system->getBlockSizeBytes();
        int atomic_offset = pkt->getAddr() - line_addr;
        std::vector<bool> access_mask(block_size, false);
        assert(atomic_offset + pkt->getSize() <= block_size);

        for (int idx = 0; idx < pkt->getSize(); ++idx) {
            access_mask[atomic_offset + idx] = true;
        }

        std::vector<std::pair<int, AtomicOpFunctor*>> atomic_ops;
        std::pair<int, AtomicOpFunctor*>
            atomic_op(atomic_offset, pkt->getAtomicOp());

        atomic_ops.emplace_back(atomic_op);
        msg->getwriteMask().setAtomicOps(atomic_ops);
    } else if (write) {
        msg->setType(SequencerRequestType_ST);
    } else {
        assert(pkt->isRead());
        msg->setType(SequencerRequestType_LD);
    }

    int offset = paddr & m_data_block_mask;

    msg->getLen() = (offset + len) <= m_ruby_system->getBlockSizeBytes() ?
        len : m_ruby_system->getBlockSizeBytes() - offset;

    if (write && (data != NULL)) {
        if (active_request.data != NULL) {
            msg->getDataBlk().setData(data, offset, msg->getLen());
        }
    }

    m_outstanding_count++;

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)),
        m_ruby_system->getRandomization(), m_ruby_system->getWarmupEnabled());
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

    int blk_size = m_ruby_system->getBlockSizeBytes();

    std::shared_ptr<SequencerMsg> msg =
        std::make_shared<SequencerMsg>(clockEdge(), blk_size);
    msg->getPhysicalAddress() = active_request.start_paddr +
                                active_request.bytes_completed;

    assert((msg->getPhysicalAddress() & m_data_block_mask) == 0);
    msg->getLineAddress() = makeLineAddress(msg->getPhysicalAddress());

    msg->getType() = (active_request.write ? SequencerRequestType_ST :
                     SequencerRequestType_LD);

    msg->getLen() =
        (active_request.len -
         active_request.bytes_completed < m_ruby_system->getBlockSizeBytes() ?
         active_request.len - active_request.bytes_completed :
         m_ruby_system->getBlockSizeBytes());

    if (active_request.write) {
        msg->getDataBlk().
            setData(&active_request.data[active_request.bytes_completed],
                    0, msg->getLen());
    }

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)),
        m_ruby_system->getRandomization(), m_ruby_system->getWarmupEnabled());
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
DMASequencer::atomicCallback(const DataBlock& dblk, const Addr& address)
{
    RequestTable::iterator i = m_RequestTable.find(address);
    assert(i != m_RequestTable.end());

    DMARequest &active_request = i->second;
    PacketPtr pkt = active_request.pkt;

    int offset = active_request.start_paddr & m_data_block_mask;
    memcpy(pkt->getPtr<uint8_t>(), dblk.getData(offset, pkt->getSize()),
           pkt->getSize());

    ruby_hit_callback(pkt);

    m_outstanding_count--;
    m_RequestTable.erase(i);
}

void
DMASequencer::recordRequestType(DMASequencerRequestType requestType)
{
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            DMASequencerRequestType_to_string(requestType));
}

} // namespace ruby
} // namespace gem5
