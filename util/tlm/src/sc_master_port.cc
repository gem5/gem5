/*
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sstream>

#include "master_transactor.hh"
#include "params/ExternalMaster.hh"
#include "sc_ext.hh"
#include "sc_master_port.hh"
#include "sim/core.hh"
#include "sim/system.hh"

namespace Gem5SystemC
{

gem5::PacketPtr
SCMasterPort::generatePacket(tlm::tlm_generic_payload &trans)
{
    gem5::Request::Flags flags;
    auto req = std::make_shared<gem5::Request>(
        trans.get_address(), trans.get_data_length(), flags, owner.id);

    gem5::MemCmd cmd;

    switch (trans.get_command()) {
    case tlm::TLM_READ_COMMAND:
        cmd = gem5::MemCmd::ReadReq;
        break;
    case tlm::TLM_WRITE_COMMAND:
        cmd = gem5::MemCmd::WriteReq;
        break;
    default:
        SC_REPORT_FATAL("SCMasterPort",
                        "received transaction with unsupported command");
    }

    /*
     * Allocate a new Packet. The packet will be deleted when it returns from
     * the gem5 world as a response.
     */
    auto pkt = new gem5::Packet(req, cmd);
    pkt->dataStatic(trans.get_data_ptr());

    return pkt;
}

void
SCMasterPort::destroyPacket(gem5::PacketPtr pkt)
{
    delete pkt;
}

SCMasterPort::SCMasterPort(const std::string &name_,
                           const std::string &systemc_name,
                           gem5::ExternalMaster &owner_,
                           Gem5SimControl &simControl)
    : gem5::ExternalMaster::ExternalPort(name_, owner_),
      peq(this, &SCMasterPort::peq_cb),
      waitForRetry(false),
      pendingRequest(nullptr),
      pendingPacket(nullptr),
      needToSendRetry(false),
      responseInProgress(false),
      transactor(nullptr),
      simControl(simControl)
{
    system = dynamic_cast<const gem5::ExternalMasterParams &>(owner_.params())
                 .system;
}

void
SCMasterPort::bindToTransactor(Gem5MasterTransactor *transactor)
{
    sc_assert(this->transactor == nullptr);

    this->transactor = transactor;

    /*
     * Register the TLM non-blocking interface when using gem5 Timing mode and
     * the TLM blocking interface when using the gem5 Atomic mode.
     * Then the magic (TM) in simple_target_socket automatically transforms
     * non-blocking in blocking transactions and vice versa.
     *
     * NOTE: The mode may change during execution.
     */
    if (system->isTimingMode()) {
        SC_REPORT_INFO("SCMasterPort", "register non-blocking interface");
        transactor->socket.register_nb_transport_fw(
            this, &SCMasterPort::nb_transport_fw);
    } else if (system->isAtomicMode()) {
        SC_REPORT_INFO("SCMasterPort", "register blocking interface");
        transactor->socket.register_b_transport(this,
                                                &SCMasterPort::b_transport);
    } else {
        panic("gem5 operates neither in Timing nor in Atomic mode");
    }

    transactor->socket.register_transport_dbg(this,
                                              &SCMasterPort::transport_dbg);
}

void
SCMasterPort::checkTransaction(tlm::tlm_generic_payload &trans)
{
    if (trans.is_response_error()) {
        std::stringstream ss;
        ss << "Transaction returned with error, response status = "
           << trans.get_response_string();
        SC_REPORT_ERROR("TLM-2", ss.str().c_str());
    }
}

tlm::tlm_sync_enum
SCMasterPort::nb_transport_fw(tlm::tlm_generic_payload &trans,
                              tlm::tlm_phase &phase, sc_core::sc_time &delay)
{
    uint64_t adr = trans.get_address();
    unsigned len = trans.get_data_length();
    unsigned char *byteEnable = trans.get_byte_enable_ptr();
    unsigned width = trans.get_streaming_width();

    // check the transaction attributes for unsupported features ...
    if (byteEnable != 0) {
        trans.set_response_status(tlm::TLM_BYTE_ENABLE_ERROR_RESPONSE);
        return tlm::TLM_COMPLETED;
    }
    if (width < len) { // is this a burst request?
        trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
        return tlm::TLM_COMPLETED;
    }

    // ... and queue the valid transaction
    trans.acquire();
    peq.notify(trans, phase, delay);
    return tlm::TLM_ACCEPTED;
}

void
SCMasterPort::peq_cb(tlm::tlm_generic_payload &trans,
                     const tlm::tlm_phase &phase)
{
    // catch up with SystemC time
    simControl.catchup();
    assert(gem5::curTick() == sc_core::sc_time_stamp().value());

    switch (phase) {
    case tlm::BEGIN_REQ:
        handleBeginReq(trans);
        break;
    case tlm::END_RESP:
        handleEndResp(trans);
        break;
    default:
        panic("unimplemented phase in callback");
    }

    // the functions called above may have scheduled gem5 events
    // -> notify the event loop handler
    simControl.notify();
}

void
SCMasterPort::handleBeginReq(tlm::tlm_generic_payload &trans)
{
    sc_assert(!waitForRetry);
    sc_assert(pendingRequest == nullptr);
    sc_assert(pendingPacket == nullptr);

    trans.acquire();

    gem5::PacketPtr pkt = nullptr;

    Gem5Extension *extension = nullptr;
    trans.get_extension(extension);

    // If there is an extension, this transaction was initiated by the gem5
    // world and we can pipe through the original packet. Otherwise, we
    // generate a new packet based on the transaction.
    if (extension != nullptr) {
        pkt = extension->getPacket();
    } else {
        pkt = generatePacket(trans);
    }

    auto tlmSenderState = new TlmSenderState(trans);
    pkt->pushSenderState(tlmSenderState);

    if (sendTimingReq(pkt)) { // port is free -> send END_REQ immediately
        sendEndReq(trans);
        trans.release();
    } else { // port is blocked -> wait for retry before sending END_REQ
        waitForRetry = true;
        pendingRequest = &trans;
        pendingPacket = pkt;
    }
}

void
SCMasterPort::handleEndResp(tlm::tlm_generic_payload &trans)
{
    sc_assert(responseInProgress);

    responseInProgress = false;

    checkTransaction(trans);

    if (needToSendRetry) {
        sendRetryResp();
        needToSendRetry = false;
    }
}

void
SCMasterPort::sendEndReq(tlm::tlm_generic_payload &trans)
{
    tlm::tlm_phase phase = tlm::END_REQ;
    auto delay = sc_core::SC_ZERO_TIME;

    auto status = transactor->socket->nb_transport_bw(trans, phase, delay);
    panic_if(status != tlm::TLM_ACCEPTED,
             "Unexpected status after sending END_REQ");
}

void
SCMasterPort::b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &t)
{
    Gem5Extension *extension = nullptr;
    trans.get_extension(extension);

    gem5::PacketPtr pkt = nullptr;

    // If there is an extension, this transaction was initiated by the gem5
    // world and we can pipe through the original packet.
    if (extension != nullptr) {
        pkt = extension->getPacket();
    } else {
        pkt = generatePacket(trans);
    }

    gem5::Tick ticks = sendAtomic(pkt);

    // send an atomic request to gem5
    panic_if(pkt->needsResponse() && !pkt->isResponse(),
             "Packet sending failed!\n");

    // one tick is a pico second
    auto delay = sc_core::sc_time(
        (double)(ticks / gem5::sim_clock::as_int::ps), sc_core::SC_PS);

    // update time
    t += delay;

    if (extension == nullptr)
        destroyPacket(pkt);

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int
SCMasterPort::transport_dbg(tlm::tlm_generic_payload &trans)
{
    Gem5Extension *extension = nullptr;
    trans.get_extension(extension);

    // If there is an extension, this transaction was initiated by the gem5
    // world and we can pipe through the original packet.
    if (extension != nullptr) {
        sendFunctional(extension->getPacket());
    } else {
        auto pkt = generatePacket(trans);
        sendFunctional(pkt);
        destroyPacket(pkt);
    }

    return trans.get_data_length();
}

bool
SCMasterPort::get_direct_mem_ptr(tlm::tlm_generic_payload &trans,
                                 tlm::tlm_dmi &dmi_data)
{
    return false;
}

bool
SCMasterPort::recvTimingResp(gem5::PacketPtr pkt)
{
    // exclusion rule
    // We need to Wait for END_RESP before sending next BEGIN_RESP
    if (responseInProgress) {
        sc_assert(!needToSendRetry);
        needToSendRetry = true;
        return false;
    }

    sc_assert(pkt->isResponse());

    /*
     * Pay for annotated transport delays.
     *
     * See recvTimingReq in sc_slave_port.cc for a detailed description.
     */
    auto delay = sc_core::sc_time::from_value(pkt->payloadDelay);
    // reset the delays
    pkt->payloadDelay = 0;
    pkt->headerDelay = 0;

    auto tlmSenderState =
        dynamic_cast<TlmSenderState *>(pkt->popSenderState());
    sc_assert(tlmSenderState != nullptr);

    auto &trans = tlmSenderState->trans;

    Gem5Extension *extension = nullptr;
    trans.get_extension(extension);

    // clean up
    delete tlmSenderState;

    // If there is an extension the packet was piped through and we must not
    // delete it. The packet travels back with the transaction.
    if (extension == nullptr)
        destroyPacket(pkt);

    sendBeginResp(trans, delay);
    trans.release();

    return true;
}

void
SCMasterPort::sendBeginResp(tlm::tlm_generic_payload &trans,
                            sc_core::sc_time &delay)
{
    tlm::tlm_phase phase = tlm::BEGIN_RESP;

    trans.set_response_status(tlm::TLM_OK_RESPONSE);

    auto status = transactor->socket->nb_transport_bw(trans, phase, delay);

    if (status == tlm::TLM_COMPLETED ||
        status == tlm::TLM_UPDATED && phase == tlm::END_RESP) {
        // transaction completed -> no need to wait for tlm::END_RESP
        responseInProgress = false;
    } else if (status == tlm::TLM_ACCEPTED) {
        // we need to wait for tlm::END_RESP
        responseInProgress = true;
    } else {
        panic("Unexpected status after sending BEGIN_RESP");
    }
}

void
SCMasterPort::recvReqRetry()
{
    sc_assert(waitForRetry);
    sc_assert(pendingRequest != nullptr);
    sc_assert(pendingPacket != nullptr);

    if (sendTimingReq(pendingPacket)) {
        waitForRetry = false;
        pendingPacket = nullptr;

        auto &trans = *pendingRequest;
        sendEndReq(trans);
        trans.release();

        pendingRequest = nullptr;
    }
}

void
SCMasterPort::recvRangeChange()
{
    SC_REPORT_WARNING("SCMasterPort",
                      "received address range change but ignored it");
}

gem5::ExternalMaster::ExternalPort *
SCMasterPortHandler::getExternalPort(const std::string &name,
                                     gem5::ExternalMaster &owner,
                                     const std::string &port_data)
{
    // Create and register a new SystemC master port
    auto *port = new SCMasterPort(name, port_data, owner, control);

    control.registerMasterPort(port_data, port);

    return port;
}

} // namespace Gem5SystemC
