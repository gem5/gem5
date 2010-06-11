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

#include "mem/protocol/SequencerMsg.hh"
#include "mem/protocol/SequencerRequestType.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/system/System.hh"

DMASequencer::DMASequencer(const Params *p)
    : RubyPort(p)
{
}

void
DMASequencer::init()
{
    RubyPort::init();
    m_is_busy = false;
    m_data_block_mask = ~ (~0 << RubySystem::getBlockSizeBits());
}

RequestStatus
DMASequencer::makeRequest(const RubyRequest &request)
{
    uint64_t paddr = request.paddr;
    uint8_t* data = request.data;
    int len = request.len;
    bool write = false;
    switch(request.type) {
      case RubyRequestType_LD:
        write = false;
        break;
      case RubyRequestType_ST:
        write = true;
        break;
      case RubyRequestType_NULL:
      case RubyRequestType_IFETCH:
      case RubyRequestType_Locked_Read:
      case RubyRequestType_Locked_Write:
      case RubyRequestType_RMW_Read:
      case RubyRequestType_RMW_Write:
      case RubyRequestType_NUM:
        panic("DMASequencer::makeRequest does not support RubyRequestType");
        return RequestStatus_NULL;
    }

    assert(!m_is_busy);  // only support one outstanding DMA request
    m_is_busy = true;

    active_request.start_paddr = paddr;
    active_request.write = write;
    active_request.data = data;
    active_request.len = len;
    active_request.bytes_completed = 0;
    active_request.bytes_issued = 0;
    active_request.pkt = request.pkt;

    SequencerMsg *msg = new SequencerMsg;
    msg->getPhysicalAddress() = Address(paddr);
    msg->getLineAddress() = line_address(msg->getPhysicalAddress());
    msg->getType() = write ? SequencerRequestType_ST : SequencerRequestType_LD;
    int offset = paddr & m_data_block_mask;

    msg->getLen() = (offset + len) <= RubySystem::getBlockSizeBytes() ?
        len : RubySystem::getBlockSizeBytes() - offset;

    if (write) {
        msg->getDataBlk().setData(data, offset, msg->getLen());
    }

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg);
    active_request.bytes_issued += msg->getLen();

    return RequestStatus_Issued;
}

void
DMASequencer::issueNext()
{
    assert(m_is_busy == true);
    active_request.bytes_completed = active_request.bytes_issued;
    if (active_request.len == active_request.bytes_completed) {
        ruby_hit_callback(active_request.pkt);
        m_is_busy = false;
        return;
    }

    SequencerMsg *msg = new SequencerMsg;
    msg->getPhysicalAddress() = Address(active_request.start_paddr +
                                       active_request.bytes_completed);

    assert((msg->getPhysicalAddress().getAddress() & m_data_block_mask) == 0);
    msg->getLineAddress() = line_address(msg->getPhysicalAddress());

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
        msg->getType() = SequencerRequestType_ST;
    } else {
        msg->getType() = SequencerRequestType_LD;
    }

    assert(m_mandatory_q_ptr != NULL);
    m_mandatory_q_ptr->enqueue(msg);
    active_request.bytes_issued += msg->getLen();
}

void
DMASequencer::dataCallback(const DataBlock & dblk)
{
    assert(m_is_busy == true);
    int len = active_request.bytes_issued - active_request.bytes_completed;
    int offset = 0;
    if (active_request.bytes_completed == 0)
        offset = active_request.start_paddr & m_data_block_mask;
    assert(active_request.write == false);
    memcpy(&active_request.data[active_request.bytes_completed],
           dblk.getData(offset, len), len);
    issueNext();
}

void
DMASequencer::ackCallback()
{
    issueNext();
}

void
DMASequencer::printConfig(std::ostream & out)
{
}

DMASequencer *
DMASequencerParams::create()
{
    return new DMASequencer(this);
}
