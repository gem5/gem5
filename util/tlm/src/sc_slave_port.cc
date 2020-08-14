/*
 * Copyright (c) 2015, University of Kaiserslautern
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

#include "sc_ext.hh"
#include "sc_mm.hh"
#include "sc_slave_port.hh"
#include "slave_transactor.hh"

namespace Gem5SystemC
{

/**
 * Instantiate a tlm memory manager that takes care about all the
 * tlm transactions in the system
 */
MemoryManager mm;

/**
 * Convert a gem5 packet to a TLM payload by copying all the relevant
 * information to a previously allocated tlm payload
 */
void
packet2payload(PacketPtr packet, tlm::tlm_generic_payload &trans)
{
    trans.set_address(packet->getAddr());

    /* Check if this transaction was allocated by mm */
    sc_assert(trans.has_mm());

    unsigned int size = packet->getSize();
    unsigned char *data = packet->getPtr<unsigned char>();

    trans.set_data_length(size);
    trans.set_streaming_width(size);
    trans.set_data_ptr(data);

    if (packet->isRead()) {
        trans.set_command(tlm::TLM_READ_COMMAND);
    }
    else if (packet->isInvalidate()) {
        /* Do nothing */
    } else if (packet->isWrite()) {
        trans.set_command(tlm::TLM_WRITE_COMMAND);
    } else {
        SC_REPORT_FATAL("SCSlavePort", "No R/W packet");
    }
}

/**
 * Similar to TLM's blocking transport (LT)
 */
Tick
SCSlavePort::recvAtomic(PacketPtr packet)
{
    CAUGHT_UP;
    SC_REPORT_INFO("SCSlavePort", "recvAtomic hasn't been tested much");

    panic_if(packet->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(packet->isRead() || packet->isWrite()),
             "Should only see read and writes at TLM memory\n");


    sc_core::sc_time delay = sc_core::SC_ZERO_TIME;


    /* Prepare the transaction */
    tlm::tlm_generic_payload * trans = mm.allocate();
    trans->acquire();
    packet2payload(packet, *trans);

    /* Attach the packet pointer to the TLM transaction to keep track */
    Gem5Extension* extension = new Gem5Extension(packet);
    trans->set_auto_extension(extension);

    /* Execute b_transport: */
    if (packet->cmd == MemCmd::SwapReq) {
        SC_REPORT_FATAL("SCSlavePort", "SwapReq not supported");
    } else if (packet->isRead()) {
        transactor->socket->b_transport(*trans, delay);
    } else if (packet->isInvalidate()) {
        // do nothing
    } else if (packet->isWrite()) {
        transactor->socket->b_transport(*trans, delay);
    } else {
        SC_REPORT_FATAL("SCSlavePort", "Typo of request not supported");
    }

    if (packet->needsResponse()) {
        packet->makeResponse();
    }

    trans->release();

    return delay.value();
}

/**
 * Similar to TLM's debug transport
 */
void
SCSlavePort::recvFunctional(PacketPtr packet)
{
    /* Prepare the transaction */
    tlm::tlm_generic_payload * trans = mm.allocate();
    trans->acquire();
    packet2payload(packet, *trans);

    /* Attach the packet pointer to the TLM transaction to keep track */
    Gem5Extension* extension = new Gem5Extension(packet);
    trans->set_auto_extension(extension);

    /* Execute Debug Transport: */
    unsigned int bytes = transactor->socket->transport_dbg(*trans);
    if (bytes != trans->get_data_length()) {
        SC_REPORT_FATAL("SCSlavePort","debug transport was not completed");
    }

    trans->release();
}

bool
SCSlavePort::recvTimingSnoopResp(PacketPtr packet)
{
    /* Snooping should be implemented with tlm_dbg_transport */
    SC_REPORT_FATAL("SCSlavePort","unimplemented func.: recvTimingSnoopResp");
    return false;
}

void
SCSlavePort::recvFunctionalSnoop(PacketPtr packet)
{
    /* Snooping should be implemented with tlm_dbg_transport */
    SC_REPORT_FATAL("SCSlavePort","unimplemented func.: recvFunctionalSnoop");
}

/**
 *  Similar to TLM's non-blocking transport (AT)
 */
bool
SCSlavePort::recvTimingReq(PacketPtr packet)
{
    CAUGHT_UP;

    panic_if(packet->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(packet->isRead() || packet->isWrite()),
             "Should only see read and writes at TLM memory\n");


    /* We should never get a second request after noting that a retry is
     * required */
    sc_assert(!needToSendRequestRetry);

    /* Remember if a request comes in while we're blocked so that a retry
     * can be sent to gem5 */
    if (blockingRequest) {
        needToSendRequestRetry = true;
        return false;
    }

    /*  NOTE: normal tlm is blocking here. But in our case we return false
     *  and tell gem5 when a retry can be done. This is the main difference
     *  in the protocol:
     *  if (requestInProgress)
     *  {
     *      wait(endRequestEvent);
     *  }
     *  requestInProgress = trans;
    */

    /* Prepare the transaction */
    tlm::tlm_generic_payload * trans = mm.allocate();
    trans->acquire();
    packet2payload(packet, *trans);

    /* Attach the packet pointer to the TLM transaction to keep track */
    Gem5Extension* extension = new Gem5Extension(packet);
    trans->set_auto_extension(extension);

    /*
     * Pay for annotated transport delays.
     *
     * The header delay marks the point in time, when the packet first is seen
     * by the transactor. This is the point int time, when the transactor needs
     * to send the BEGIN_REQ to the SystemC world.
     *
     * NOTE: We drop the payload delay here. Normally, the receiver would be
     *       responsible for handling the payload delay. In this case, however,
     *       the receiver is a SystemC module and has no notion of the gem5
     *       transport protocol and we cannot simply forward the
     *       payload delay to the receiving module. Instead, we expect the
     *       receiving SystemC module to model the payload delay by deferring
     *       the END_REQ. This could lead to incorrect delays, if the XBar
     *       payload delay is longer than the time the receiver needs to accept
     *       the request (time between BEGIN_REQ and END_REQ).
     *
     * TODO: We could detect the case described above by remembering the
     *       payload delay and comparing it to the time between BEGIN_REQ and
     *       END_REQ. Then, a warning should be printed.
     */
    auto delay = sc_core::sc_time::from_value(packet->payloadDelay);
    // reset the delays
    packet->payloadDelay = 0;
    packet->headerDelay = 0;

    /* Starting TLM non-blocking sequence (AT) Refer to IEEE1666-2011 SystemC
     * Standard Page 507 for a visualisation of the procedure */
    tlm::tlm_phase phase = tlm::BEGIN_REQ;
    tlm::tlm_sync_enum status;
    status = transactor->socket->nb_transport_fw(*trans, phase, delay);
    /* Check returned value: */
    if (status == tlm::TLM_ACCEPTED) {
        sc_assert(phase == tlm::BEGIN_REQ);
        /* Accepted but is now blocking until END_REQ (exclusion rule)*/
        blockingRequest = trans;
    } else if (status == tlm::TLM_UPDATED) {
        /* The Timing annotation must be honored: */
        sc_assert(phase == tlm::END_REQ || phase == tlm::BEGIN_RESP);

        PayloadEvent<SCSlavePort> * pe;
        pe = new PayloadEvent<SCSlavePort>(*this,
            &SCSlavePort::pec, "PEQ");
        pe->notify(*trans, phase, delay);
    } else if (status == tlm::TLM_COMPLETED) {
        /* Transaction is over nothing has do be done. */
        sc_assert(phase == tlm::END_RESP);
        trans->release();
    }

    return true;
}

void
SCSlavePort::pec(
    PayloadEvent<SCSlavePort> * pe,
    tlm::tlm_generic_payload& trans,
    const tlm::tlm_phase& phase)
{
    sc_time delay;

    if (phase == tlm::END_REQ ||
            &trans == blockingRequest && phase == tlm::BEGIN_RESP) {
        sc_assert(&trans == blockingRequest);
        blockingRequest = NULL;

        /* Did another request arrive while blocked, schedule a retry */
        if (needToSendRequestRetry) {
            needToSendRequestRetry = false;
            sendRetryReq();
        }
    }
    if (phase == tlm::BEGIN_RESP)
    {
        CAUGHT_UP;

        auto& extension = Gem5Extension::getExtension(trans);
        auto packet = extension.getPacket();

        sc_assert(!blockingResponse);

        bool need_retry = false;

        /*
         * If the packet was piped through and needs a response, we don't need
         * to touch the packet and can forward it directly as a response.
         * Otherwise, we need to make a response and send the transformed
         * packet.
         */
        if (extension.isPipeThrough()) {
            if (packet->isResponse()) {
                need_retry = !sendTimingResp(packet);
            }
        } else if (packet->needsResponse()) {
            packet->makeResponse();
            need_retry = !sendTimingResp(packet);
        }

        if (need_retry) {
            blockingResponse = &trans;
        } else {
            if (phase == tlm::BEGIN_RESP) {
                /* Send END_RESP and we're finished: */
                tlm::tlm_phase fw_phase = tlm::END_RESP;
                sc_time delay = SC_ZERO_TIME;
                transactor->socket->nb_transport_fw(trans, fw_phase, delay);
                /* Release the transaction with all the extensions */
                trans.release();
            }
        }
    }
    delete pe;
}

void
SCSlavePort::recvRespRetry()
{
    CAUGHT_UP;

    /* Retry a response */
    sc_assert(blockingResponse);

    tlm::tlm_generic_payload *trans = blockingResponse;
    blockingResponse = NULL;
    PacketPtr packet = Gem5Extension::getExtension(trans).getPacket();

    bool need_retry = !sendTimingResp(packet);

    sc_assert(!need_retry);

    sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
    tlm::tlm_phase phase = tlm::END_RESP;
    transactor->socket->nb_transport_fw(*trans, phase, delay);
    // Release transaction with all the extensions
    trans->release();
}

tlm::tlm_sync_enum
SCSlavePort::nb_transport_bw(tlm::tlm_generic_payload& trans,
    tlm::tlm_phase& phase,
    sc_core::sc_time& delay)
{
    PayloadEvent<SCSlavePort> * pe;
    pe = new PayloadEvent<SCSlavePort>(*this, &SCSlavePort::pec, "PE");
    pe->notify(trans, phase, delay);
    return tlm::TLM_ACCEPTED;
}

SCSlavePort::SCSlavePort(const std::string &name_,
    const std::string &systemc_name,
    ExternalSlave &owner_) :
    ExternalSlave::ExternalPort(name_, owner_),
    blockingRequest(NULL),
    needToSendRequestRetry(false),
    blockingResponse(NULL),
    transactor(nullptr)
{
}

void
SCSlavePort::bindToTransactor(Gem5SlaveTransactor* transactor)
{
    sc_assert(this->transactor == nullptr);

    this->transactor = transactor;

    transactor->socket.register_nb_transport_bw(this,
                                                &SCSlavePort::nb_transport_bw);
}

ExternalSlave::ExternalPort*
SCSlavePortHandler::getExternalPort(const std::string &name,
                                    ExternalSlave &owner,
                                    const std::string &port_data)
{
    // Create and register a new SystemC slave port
    auto* port = new SCSlavePort(name, port_data, owner);

    control.registerSlavePort(port_data, port);

    return port;
}

}
