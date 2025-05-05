/*
 * Copyright (c) 2023 ARM Limited
 * All rights reserved
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

#ifndef __MEM_RUBY_PROTOCOL_CHI_CHIGenericController_HH__
#define __MEM_RUBY_PROTOCOL_CHI_CHIGenericController_HH__

#include <iostream>
#include <sstream>
#include <string>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/CacheRecorder.hh"
#include "params/CHIGenericController.hh"

// Generated from SLICC
#include "mem/ruby/protocol/CHI/CHIDataMsg.hh"
#include "mem/ruby/protocol/CHI/CHIRequestMsg.hh"
#include "mem/ruby/protocol/CHI/CHIResponseMsg.hh"
#include "mem/ruby/protocol/CHI/Cache_Controller.hh"

namespace gem5
{

namespace ruby
{

class CHIGenericController : public AbstractController
{
  public:
    PARAMS(CHIGenericController);
    CHIGenericController(const Params &p);

    void init() override;

    MessageBuffer *getMandatoryQueue() const override;
    MessageBuffer *getMemReqQueue() const override;
    MessageBuffer *getMemRespQueue() const override;
    void initNetQueues() override;

    void print(std::ostream& out) const override;
    void wakeup() override;
    void resetStats() override;
    void regStats() override;
    void collateStats() override;

    void recordCacheTrace(int cntrl, CacheRecorder* tr) override;
    Sequencer* getCPUSequencer() const override;
    DMASequencer* getDMASequencer() const override;
    GPUCoalescer* getGPUCoalescer() const override;

    void addSequencer(RubyPort* seq);

    bool functionalReadBuffers(PacketPtr&) override;
    bool functionalReadBuffers(PacketPtr&, WriteMask&) override;
    int functionalWriteBuffers(PacketPtr&) override;

    AccessPermission getAccessPermission(const Addr& param_addr) override;

    void functionalRead(const Addr& param_addr, Packet* param_pkt,
                        WriteMask& param_mask) override;
    int functionalWrite(const Addr& param_addr, Packet* param_pkt) override;

  protected:
    MessageBuffer* const reqOut;
    MessageBuffer* const snpOut;
    MessageBuffer* const rspOut;
    MessageBuffer* const datOut;
    MessageBuffer* const reqIn;
    MessageBuffer* const snpIn;
    MessageBuffer* const rspIn;
    MessageBuffer* const datIn;

    std::vector<RubyPort*> sequencers;

  public:
    const int cacheLineSize;
    const int cacheLineBits;
    const int dataChannelSize;
    const int dataMsgsPerLine;

    // interface to generic requesters and responders

    enum CHIChannel
    {
        CHI_REQ = 0,
        CHI_SNP = 1,
        CHI_RSP = 2,
        CHI_DAT = 3
    };

    using CHIRequestMsg = CHI::CHIRequestMsg;
    using CHIResponseMsg = CHI::CHIResponseMsg;
    using CHIDataMsg = CHI::CHIDataMsg;
    using CHIRequestMsgPtr = std::shared_ptr<CHIRequestMsg>;
    using CHIResponseMsgPtr = std::shared_ptr<CHIResponseMsg>;
    using CHIDataMsgPtr = std::shared_ptr<CHIDataMsg>;

    bool
    sendRequestMsg(CHIRequestMsgPtr msg)
    {
        return sendMessage(msg, reqOut);
    }

    bool
    sendSnoopMsg(CHIRequestMsgPtr msg)
    {
        return sendMessage(msg, snpOut);
    }

    bool
    sendResponseMsg(CHIResponseMsgPtr msg)
    {
        return sendMessage(msg, rspOut);
    }

    bool
    sendDataMsg(CHIDataMsgPtr msg)
    {
        return sendMessage(msg, datOut);
    }

  protected:
    virtual bool recvRequestMsg(const CHIRequestMsg *msg) = 0;
    virtual bool recvSnoopMsg(const CHIRequestMsg *msg) = 0;
    virtual bool recvResponseMsg(const CHIResponseMsg *msg) = 0;
    virtual bool recvDataMsg(const CHIDataMsg *msg) = 0;

  private:
    template<typename MsgType>
    bool receiveAllRdyMessages(MessageBuffer *buffer,
                        const std::function<bool(const MsgType*)> &callback)
    {
        bool pending = false;
        Tick cur_tick = curTick();
        while (buffer->isReady(cur_tick)) {
            const MsgType *msg =
                dynamic_cast<const MsgType*>(buffer->peek());
            assert(msg);
            if (callback(msg))
                buffer->dequeue(cur_tick);
            else {
                pending = true;
                break;
            }
        }
        return pending;
    }

    template<typename MessageType>
    bool sendMessage(MessageType &msg, MessageBuffer *buffer)
    {
        Tick cur_tick = curTick();
        if (buffer->areNSlotsAvailable(1, cur_tick)) {
            buffer->enqueue(msg, curTick(), cyclesToTicks(Cycles(1)),
                m_ruby_system->getRandomization(),
                m_ruby_system->getWarmupEnabled());
            return true;
        } else {
            return false;
        }
    }

};

} // namespace ruby
} // namespace gem5

#endif // __CHIGenericController_H__
