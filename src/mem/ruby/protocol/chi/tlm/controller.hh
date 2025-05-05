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

#ifndef __MEM_RUBY_PROTOCOL_CHI_TLM_CONTROLLER_HH__
#define __MEM_RUBY_PROTOCOL_CHI_TLM_CONTROLLER_HH__

#include <ARM/TLM/arm_chi.h>

#include "mem/ruby/protocol/chi/generic/CHIGenericController.hh"
#include "mem/ruby/protocol/CHI/CHIDataType.hh"
#include "mem/ruby/protocol/CHI/CHIRequestType.hh"
#include "mem/ruby/protocol/CHI/CHIResponseType.hh"
#include "mem/ruby/protocol/RequestStatus.hh"
#include "mem/ruby/protocol/WriteMask.hh"
#include "params/TlmController.hh"

namespace gem5 {

namespace ruby {
class CHIRequestMsg;
class CHIResponseMsg;
class CHIDataMsg;
}

namespace tlm::chi {

/**
 * The tlm::chi::CacheController is a ruby CacheController which acts as
 * a bridge between the AMBA TLM 2.0 implementation of CHI [1][2] with
 * the gem5 (ruby) one.
 *
 * In other words it translates AMBA CHI transactions into ruby
 * messages (which are then forwarded to the MessageQueues)
 * and viceversa.
 *
 * ARM::CHI::Payload,        CHIRequestMsg
 *                     <-->  CHIDataMsg
 * ARM::CHI::Phase           CHIResponseMsg
 *                           CHIDataMsg
 *
 * To connect the tlm::chi::CacheController in python is relatively
 * straightforward. The upstream initiator/RNF needs to have a pointer
 * to the controller, and this can be done for example with
 * SimObject params.
 * Example:
 *
 * class MyRNF():
 *     chi_controller = Param.TlmController("TLM-to-Ruby CacheController")
 *
 * The RNF C++ code would then have to set the
 * tlm::chi::CacheController::bw callback to implement the backward path
 * to send data from downstream to upstream.
 *
 * [1]: https://developer.arm.com/documentation/101459/latest
 * [2]: https://developer.arm.com/Architectures/AMBA#Downloads
 */
class CacheController : public ruby::CHIGenericController
{
  public:
    PARAMS(TlmController);
    CacheController(const Params &p);

    /** Set this to send data upstream */
    std::function<void(ARM::CHI::Payload* payload, ARM::CHI::Phase* phase)> bw;

    bool recvRequestMsg(const CHIRequestMsg *msg) override;
    bool recvSnoopMsg(const CHIRequestMsg *msg) override;
    bool recvResponseMsg(const CHIResponseMsg *msg) override;
    bool recvDataMsg(const CHIDataMsg *msg) override;

    void sendMsg(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase);
    using CHIGenericController::sendRequestMsg;
    void sendRequestMsg(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase);
    using CHIGenericController::sendResponseMsg;
    void sendResponseMsg(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase);
    void sendCompAck(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase);
    using CHIGenericController::sendDataMsg;
    void sendDataMsg(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase);

    Addr reqAddr(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase) const;
    Addr reqSize(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase) const;

    void pCreditGrant(const CHIResponseMsg *msg);

    struct Transaction
    {
        enum class Type
        {
            READ,
            WRITE,
            DATALESS
        };

        Transaction(CacheController *parent,
            ARM::CHI::Payload &_payload,
            ARM::CHI::Phase &_phase);
        ~Transaction();

        static std::unique_ptr<Transaction> gen(CacheController *parent,
            ARM::CHI::Payload &_payload,
            ARM::CHI::Phase &_phase);

        virtual bool
        handle(const CHIDataMsg *msg)
        {
            panic("Unimplemented");
        }

        virtual bool handle(const CHIResponseMsg *msg);

        CacheController *controller;
        ARM::CHI::Payload *payload;
        ARM::CHI::Phase phase;
    };
    struct ReadTransaction : public Transaction
    {
        using Transaction::Transaction;
        bool handle(const CHIDataMsg *msg) override;
        bool handle(const CHIResponseMsg *msg) override;
        bool forward(const CHIDataMsg *msg);

        uint8_t dataMsgCnt = 0;
    };
    struct DatalessTransaction : public Transaction
    {
        using Transaction::Transaction;
        bool handle(const CHIResponseMsg *msg) override;
    };
    struct WriteTransaction : public Transaction
    {
        using Transaction::Transaction;
        bool handle(const CHIResponseMsg *msg) override;
        bool recvComp = false;
        bool recvDBID = false;
    };
    std::unordered_map<uint16_t, std::unique_ptr<Transaction>> pendingTransactions;
};

} // namespace tlm::chi

} // namespace gem5

#endif // __MEM_RUBY_PROTOCOL_CHI_TLM_CONTROLLER_HH__
