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

#include "mem/ruby/protocol/chi/tlm/controller.hh"

#include "debug/TLM.hh"
#include "mem/ruby/protocol/CHI/CHIDataMsg.hh"
#include "mem/ruby/protocol/CHI/CHIRequestMsg.hh"
#include "mem/ruby/protocol/CHI/CHIResponseMsg.hh"
#include "mem/ruby/protocol/chi/tlm/utils.hh"

namespace gem5 {

namespace tlm::chi {

using namespace ruby::CHI;

CacheController::CacheController(const Params &p)
  : CHIGenericController(p)
{
}

bool
CacheController::recvRequestMsg(const CHIRequestMsg *msg)
{
    panic("recvRequestMsg");
    return true;
}

bool
CacheController::recvSnoopMsg(const CHIRequestMsg *msg)
{
    ARM::CHI::Phase phase;
    ARM::CHI::Payload *payload = ARM::CHI::Payload::new_payload();

    payload->address = msg->m_addr;
    payload->ns = msg->m_ns;
    phase.channel = ARM::CHI::CHANNEL_SNP;
    phase.snp_opcode = ruby_to_tlm::snpOpcode(msg->m_type);
    phase.txn_id = msg->m_txnId % 1024;

    bw(payload, &phase);

    // payload->unref(); ??
    return true;
}

void
CacheController::pCreditGrant(const CHIResponseMsg *msg)
{
    ARM::CHI::Phase phase;
    ARM::CHI::Payload *payload = ARM::CHI::Payload::new_payload();
    phase.channel = ARM::CHI::CHANNEL_RSP;
    phase.rsp_opcode = ARM::CHI::RSP_OPCODE_PCRD_GRANT;
    phase.pcrd_type = 0; // TODO: set this one depending on allow retry

    bw(payload, &phase);

    payload->unref(); // TODO: check this
}

bool
CacheController::recvResponseMsg(const CHIResponseMsg *msg)
{

    if (msg->m_type == CHIResponseType_PCrdGrant) {
        // P-credit grant does not refer to a specific transaction id
        pCreditGrant(msg);
        return true;
    }

    auto txn_id = msg->m_txnId;
    if (auto it = pendingTransactions.find(txn_id);
        it != pendingTransactions.end()) {

        auto& transaction = it->second;
        if (transaction->handle(msg))
            pendingTransactions.erase(it);
    } else {
        panic("recvResponseMsg");
    }
    return true;
}

bool
CacheController::recvDataMsg(const CHIDataMsg *msg)
{
    auto txn_id = msg->m_txnId;
    if (auto it = pendingTransactions.find(txn_id);
        it != pendingTransactions.end()) {
        auto& transaction = it->second;
        if (transaction->handle(msg))
            pendingTransactions.erase(it);
    } else {
        panic("Not able to find transaction");
    }
    return true;
}

bool
CacheController::Transaction::handle(const CHIResponseMsg *msg)
{
    const auto opcode = ruby_to_tlm::rspOpcode(msg->m_type);
    phase.channel = ARM::CHI::CHANNEL_RSP;
    phase.rsp_opcode = opcode;
    phase.resp = ruby_to_tlm::rspResp(msg->m_type);
    phase.txn_id = msg->m_txnId % 1024;

    controller->bw(payload, &phase);
    return opcode != ARM::CHI::RSP_OPCODE_RETRY_ACK;
}

bool
CacheController::ReadTransaction::handle(const CHIDataMsg *msg)
{
    dataMsgCnt++;

    for (auto byte = 0; byte < controller->cacheLineSize; byte++) {
        if (msg->m_bitMask.test(byte))
            payload->data[byte] = msg->m_dataBlk.getByte(byte);
    }

    // ARM::CHI::Phase phase;
    phase.channel = ARM::CHI::CHANNEL_DAT;
    phase.dat_opcode = ruby_to_tlm::datOpcode(msg->m_type);
    phase.resp = ruby_to_tlm::datResp(msg->m_type);
    phase.txn_id = msg->m_txnId % 1024;
    phase.data_id = dataId(msg->m_addr + msg->m_bitMask.firstBitSet(true));

    // This is a hack, we should fix it on the ruby side
    if (forward(msg)) {
        controller->bw(payload, &phase);
    }

    if (dataMsgCnt == controller->dataMsgsPerLine) {
        if (phase.exp_comp_ack == false) {
            // The client is not sending a CompAck but ruby is
            // expecting it so we send it anyway
            controller->sendCompAck(*payload, phase);
        }
        return true;
    } else {
        return false;
    }
}

bool
CacheController::ReadTransaction::handle(const CHIResponseMsg *msg)
{
    /// TODO: remove this, DBID is not sent
    phase.dbid = msg->m_dbid % 1024;
    return Transaction::handle(msg);
}

bool
CacheController::ReadTransaction::forward(const CHIDataMsg *msg)
{
    if (payload->size == 6) {
        return true;
    } else {
        if (msg->m_bitMask.test(payload->address - msg->m_addr)) {
            return true;
        } else {
            // This is not the packet holding the original address
            // Do not forward it back (just drop it)
            return false;
        }
    }
}

bool
CacheController::DatalessTransaction::handle(const CHIResponseMsg *msg)
{
    const auto opcode = ruby_to_tlm::rspOpcode(msg->m_type);
    assert(opcode == ARM::CHI::RSP_OPCODE_COMP ||
           opcode == ARM::CHI::RSP_OPCODE_RETRY_ACK);

    return Transaction::handle(msg);
}

bool
CacheController::WriteTransaction::handle(const CHIResponseMsg *msg)
{
    const auto opcode = ruby_to_tlm::rspOpcode(msg->m_type);
    if (opcode == ARM::CHI::RSP_OPCODE_COMP_DBID_RESP) {
        recvComp = true;
        recvDBID = true;
    }
    if (opcode == ARM::CHI::RSP_OPCODE_COMP) {
        recvComp = true;
    }
    if (opcode == ARM::CHI::RSP_OPCODE_DBID_RESP) {
        recvDBID = true;
    }

    phase.dbid = msg->m_dbid % 1024;
    Transaction::handle(msg);

    return recvComp && recvDBID;
}

void
CacheController::sendCompAck(ARM::CHI::Payload &payload,
                             ARM::CHI::Phase &phase)
{
    auto res_msg = std::make_shared<CHIResponseMsg>(
        curTick(), cacheLineSize, m_ruby_system);

    res_msg->m_addr = ruby::makeLineAddress(payload.address, cacheLineBits);
    res_msg->m_type = CHIResponseType_CompAck;
    res_msg->m_Destination.add(mapAddressToDownstreamMachine(payload.address));
    res_msg->m_responder = getMachineID();

    res_msg->m_txnId = phase.txn_id;

    sendResponseMsg(res_msg);
}

void
CacheController::sendMsg(ARM::CHI::Payload &payload, ARM::CHI::Phase &phase)
{
    switch (phase.channel) {
      case ARM::CHI::CHANNEL_REQ:
        sendRequestMsg(payload, phase);
        break;
      case ARM::CHI::CHANNEL_DAT:
        sendDataMsg(payload, phase);
        break;
      case ARM::CHI::CHANNEL_RSP:
        sendResponseMsg(payload, phase);
        break;
      default:
        panic("Unimplemented channel: %d", phase.channel);
    }
}

Addr
CacheController::reqAddr(ARM::CHI::Payload &payload,
                         ARM::CHI::Phase &phase) const
{
    switch (phase.req_opcode) {
      case ARM::CHI::REQ_OPCODE_WRITE_NO_SNP_PTL:
        return ruby::makeLineAddress(payload.address, cacheLineBits) +
            ctz64(payload.byte_enable);
      default:
        return payload.address;
    }
}

Addr
CacheController::reqSize(ARM::CHI::Payload &payload,
                         ARM::CHI::Phase &phase) const
{
    switch (phase.req_opcode) {
      case ARM::CHI::REQ_OPCODE_WRITE_NO_SNP_PTL:
        assert(transactionSize(payload.size) >= popCount(payload.byte_enable));
        return popCount(payload.byte_enable);
      default:
        return transactionSize(payload.size);
    }
}

void
CacheController::sendRequestMsg(ARM::CHI::Payload &payload,
                                ARM::CHI::Phase &phase)
{
    auto req_msg = std::make_shared<CHIRequestMsg>(
        curTick(), cacheLineSize, m_ruby_system);

    req_msg->m_addr = ruby::makeLineAddress(payload.address, cacheLineBits);
    req_msg->m_accAddr = reqAddr(payload, phase);
    req_msg->m_accSize = reqSize(payload, phase);
    req_msg->m_requestor = getMachineID();
    req_msg->m_fwdRequestor = getMachineID();
    req_msg->m_dataToFwdRequestor = false;
    req_msg->m_type = tlm_to_ruby::reqOpcode(phase.req_opcode);
    req_msg->m_isSeqReqValid = false;
    req_msg->m_is_local_pf = false;
    req_msg->m_is_remote_pf = false;
    req_msg->m_allowRetry = phase.allow_retry;
    req_msg->m_Destination.add(mapAddressToDownstreamMachine(payload.address));

    req_msg->m_txnId = phase.txn_id + (payload.lpid * 1024);
    req_msg->m_ns = payload.ns;

    sendRequestMsg(req_msg);

    pendingTransactions[req_msg->m_txnId] = Transaction::gen(
        this, payload, phase);
}

void
CacheController::sendDataMsg(ARM::CHI::Payload &payload,
                             ARM::CHI::Phase &phase)
{
    auto data_msg = std::make_shared<CHIDataMsg>(
        curTick(), cacheLineSize, m_ruby_system);

    data_msg->m_addr = ruby::makeLineAddress(payload.address, cacheLineBits);
    data_msg->m_responder = getMachineID();
    data_msg->m_type = tlm_to_ruby::datOpcode(phase.dat_opcode, phase.resp);
    data_msg->m_Destination.add(
        mapAddressToDownstreamMachine(payload.address));
    data_msg->m_txnId = phase.txn_id + (payload.lpid * 1024);
    data_msg->m_dataBlk.setData(payload.data, 0, cacheLineSize);

    std::vector<bool> byte_enabled(cacheLineSize, false);

    const Addr data_id_offset = phase.data_id * 16;
    for (int bit = data_id_offset; bit < data_id_offset + 32; bit++) {
        if (bits(payload.byte_enable, bit)) {
            byte_enabled[bit] = true;
        }
    }

    data_msg->m_bitMask = ruby::WriteMask(byte_enabled.size(), byte_enabled);

    sendDataMsg(data_msg);
}

void
CacheController::sendResponseMsg(ARM::CHI::Payload &payload,
                                 ARM::CHI::Phase &phase)
{
    auto res_msg = std::make_shared<CHIResponseMsg>(
        curTick(), cacheLineSize, m_ruby_system);

    res_msg->m_addr = ruby::makeLineAddress(payload.address, cacheLineBits);
    res_msg->m_responder = getMachineID();
    res_msg->m_type = tlm_to_ruby::rspOpcode(phase.rsp_opcode, phase.resp);
    res_msg->m_Destination.add(mapAddressToDownstreamMachine(payload.address));
    res_msg->m_txnId = phase.txn_id + (payload.lpid * 1024);

    sendResponseMsg(res_msg);
}

CacheController::Transaction::Transaction(CacheController *_controller,
    ARM::CHI::Payload &_payload,
    ARM::CHI::Phase &_phase)
  : controller(_controller), payload(&_payload), phase(_phase)
{
    payload->ref();
}

CacheController::Transaction::~Transaction()
{
    payload->unref();
}

std::unique_ptr<CacheController::Transaction>
CacheController::Transaction::gen(CacheController *controller,
    ARM::CHI::Payload &payload,
    ARM::CHI::Phase &phase)
{
    switch (phase.req_opcode) {
      case ARM::CHI::REQ_OPCODE_READ_SHARED:
      case ARM::CHI::REQ_OPCODE_READ_CLEAN:
      case ARM::CHI::REQ_OPCODE_READ_ONCE:
      case ARM::CHI::REQ_OPCODE_READ_NO_SNP:
      case ARM::CHI::REQ_OPCODE_READ_UNIQUE:
      case ARM::CHI::REQ_OPCODE_READ_NOT_SHARED_DIRTY:
      case ARM::CHI::REQ_OPCODE_READ_PREFER_UNIQUE:
      case ARM::CHI::REQ_OPCODE_MAKE_READ_UNIQUE:
        return std::make_unique<ReadTransaction>(controller, payload, phase);
      case ARM::CHI::REQ_OPCODE_WRITE_NO_SNP_PTL:
      case ARM::CHI::REQ_OPCODE_WRITE_NO_SNP_FULL:
      case ARM::CHI::REQ_OPCODE_WRITE_UNIQUE_ZERO:
      case ARM::CHI::REQ_OPCODE_WRITE_UNIQUE_FULL:
      case ARM::CHI::REQ_OPCODE_WRITE_BACK_FULL:
      // This is a write transaction for now. Will
      // this still be the case once WriteEvictOrEvict will also be supported
      // in ruby
      case ARM::CHI::REQ_OPCODE_WRITE_EVICT_OR_EVICT:
        return std::make_unique<WriteTransaction>(
            controller, payload, phase);
      case ARM::CHI::REQ_OPCODE_CLEAN_UNIQUE:
      case ARM::CHI::REQ_OPCODE_MAKE_UNIQUE:
      case ARM::CHI::REQ_OPCODE_EVICT:
      case ARM::CHI::REQ_OPCODE_STASH_ONCE_SEP_SHARED:
      case ARM::CHI::REQ_OPCODE_STASH_ONCE_SEP_UNIQUE:
        return std::make_unique<DatalessTransaction>(
            controller, payload, phase);
      default:
        panic("Impossible to generate transaction object");
    }
}

} // namespace tlm::chi

} // namespace gem5
