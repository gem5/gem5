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

#include "debug/RubyDma.hh"
#include "debug/RubyStats.hh"
#include "mem/protocol/SequencerMsg.hh"
#include "mem/protocol/SequencerRequestType.hh"
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/system/RubySystem.hh"

DMASequencer::DMASequencer(const Params *p)
    : RubyPort(p)
{
}

void
DMASequencer::init()
{
    RubyPort::init();
    m_is_busy = false;
    m_data_block_mask = mask(RubySystem::getBlockSizeBits());

    for (const auto &s_port : slave_ports)
        s_port->sendRangeChange();
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
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)));
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
    m_mandatory_q_ptr->enqueue(msg, clockEdge(), cyclesToTicks(Cycles(1)));
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
