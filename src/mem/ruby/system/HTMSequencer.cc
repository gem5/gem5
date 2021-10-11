/*
 * Copyright (c) 2020 ARM Limited
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

#include "mem/ruby/system/HTMSequencer.hh"

#include "debug/HtmMem.hh"
#include "debug/RubyPort.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

HtmCacheFailure
HTMSequencer::htmRetCodeConversion(
    const HtmFailedInCacheReason ruby_ret_code)
{
    switch (ruby_ret_code) {
      case HtmFailedInCacheReason_NO_FAIL:
        return HtmCacheFailure::NO_FAIL;
      case HtmFailedInCacheReason_FAIL_SELF:
        return HtmCacheFailure::FAIL_SELF;
      case HtmFailedInCacheReason_FAIL_REMOTE:
        return HtmCacheFailure::FAIL_REMOTE;
      case HtmFailedInCacheReason_FAIL_OTHER:
        return HtmCacheFailure::FAIL_OTHER;
      default:
        panic("Invalid htm return code\n");
    }
}

HTMSequencer::HTMSequencer(const RubyHTMSequencerParams &p)
    : Sequencer(p),
      ADD_STAT(m_htm_transaction_cycles, "number of cycles spent in an outer "
                                         "transaction"),
      ADD_STAT(m_htm_transaction_instructions, "number of instructions spent "
                                               "in an outer transaction"),
      ADD_STAT(m_htm_transaction_abort_cause, "cause of htm transaction abort")
{
    m_htmstart_tick = 0;
    m_htmstart_instruction = 0;

    // hardware transactional memory
    m_htm_transaction_cycles
        .init(10)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan)
        ;
    m_htm_transaction_instructions
        .init(10)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan)
        ;
    auto num_causes = static_cast<int>(HtmFailureFaultCause::NUM_CAUSES);
    m_htm_transaction_abort_cause
        .init(num_causes)
        .flags(statistics::total | statistics::pdf | statistics::dist |
            statistics::nozero)
        ;

    for (unsigned cause_idx = 0; cause_idx < num_causes; ++cause_idx) {
        m_htm_transaction_abort_cause.subname(
            cause_idx,
            htmFailureToStr(HtmFailureFaultCause(cause_idx)));
    }

}

HTMSequencer::~HTMSequencer()
{
}

void
HTMSequencer::htmCallback(Addr address,
                       const HtmCallbackMode mode,
                       const HtmFailedInCacheReason htm_return_code)
{
    // mode=0: HTM command
    // mode=1: transaction failed - inform via LD
    // mode=2: transaction failed - inform via ST

    if (mode == HtmCallbackMode_HTM_CMD) {
        SequencerRequest* request = nullptr;

        assert(m_htmCmdRequestTable.size() > 0);

        request = m_htmCmdRequestTable.front();
        m_htmCmdRequestTable.pop_front();

        assert(isHtmCmdRequest(request->m_type));

        PacketPtr pkt = request->pkt;
        delete request;

        // valid responses have zero as the payload
        uint8_t* dataptr = pkt->getPtr<uint8_t>();
        memset(dataptr, 0, pkt->getSize());
        *dataptr = (uint8_t) htm_return_code;

        // record stats
        if (htm_return_code == HtmFailedInCacheReason_NO_FAIL) {
            if (pkt->req->isHTMStart()) {
                m_htmstart_tick = pkt->req->time();
                m_htmstart_instruction = pkt->req->getInstCount();
                DPRINTF(HtmMem, "htmStart - htmUid=%u\n",
                        pkt->getHtmTransactionUid());
            } else if (pkt->req->isHTMCommit()) {
                Tick transaction_ticks = pkt->req->time() - m_htmstart_tick;
                Cycles transaction_cycles = ticksToCycles(transaction_ticks);
                m_htm_transaction_cycles.sample(transaction_cycles);
                m_htmstart_tick = 0;
                Counter transaction_instructions =
                    pkt->req->getInstCount() - m_htmstart_instruction;
                m_htm_transaction_instructions.sample(
                  transaction_instructions);
                m_htmstart_instruction = 0;
                DPRINTF(HtmMem, "htmCommit - htmUid=%u\n",
                        pkt->getHtmTransactionUid());
            } else if (pkt->req->isHTMAbort()) {
                HtmFailureFaultCause cause = pkt->req->getHtmAbortCause();
                assert(cause != HtmFailureFaultCause::INVALID);
                auto cause_idx = static_cast<int>(cause);
                m_htm_transaction_abort_cause[cause_idx]++;
                DPRINTF(HtmMem, "htmAbort - reason=%s - htmUid=%u\n",
                        htmFailureToStr(cause),
                        pkt->getHtmTransactionUid());
            }
        } else {
            DPRINTF(HtmMem, "HTM_CMD: fail - htmUid=%u\n",
                pkt->getHtmTransactionUid());
        }

        rubyHtmCallback(pkt, htm_return_code);
        testDrainComplete();
    } else if (mode == HtmCallbackMode_LD_FAIL ||
               mode == HtmCallbackMode_ST_FAIL) {
        // transaction failed
        assert(address == makeLineAddress(address));
        assert(m_RequestTable.find(address) != m_RequestTable.end());

        auto &seq_req_list = m_RequestTable[address];
        while (!seq_req_list.empty()) {
            SequencerRequest &request = seq_req_list.front();

            PacketPtr pkt = request.pkt;
            markRemoved();

            // TODO - atomics

            // store conditionals should indicate failure
            if (request.m_type == RubyRequestType_Store_Conditional) {
                pkt->req->setExtraData(0);
            }

            DPRINTF(HtmMem, "%s_FAIL: size=%d - "
                            "addr=0x%lx - htmUid=%d\n",
                            (mode == HtmCallbackMode_LD_FAIL) ? "LD" : "ST",
                            pkt->getSize(),
                            address, pkt->getHtmTransactionUid());

            rubyHtmCallback(pkt, htm_return_code);
            testDrainComplete();
            pkt = nullptr;
            seq_req_list.pop_front();
        }
        // free all outstanding requests corresponding to this address
        if (seq_req_list.empty()) {
            m_RequestTable.erase(address);
        }
    } else {
        panic("unrecognised HTM callback mode\n");
    }
}

void
HTMSequencer::rubyHtmCallback(PacketPtr pkt,
                          const HtmFailedInCacheReason htm_return_code)
{
    // The packet was destined for memory and has not yet been turned
    // into a response
    assert(system->isMemAddr(pkt->getAddr()) || system->isDeviceMemAddr(pkt));
    assert(pkt->isRequest());

    // First retrieve the request port from the sender State
    RubyPort::SenderState *senderState =
        safe_cast<RubyPort::SenderState *>(pkt->popSenderState());

    MemResponsePort *port = safe_cast<MemResponsePort*>(senderState->port);
    assert(port != nullptr);
    delete senderState;

    //port->htmCallback(pkt, htm_return_code);
    DPRINTF(HtmMem, "HTM callback: start=%d, commit=%d, "
                    "cancel=%d, rc=%d\n",
            pkt->req->isHTMStart(), pkt->req->isHTMCommit(),
            pkt->req->isHTMCancel(), htm_return_code);

    // turn packet around to go back to requestor if response expected
    if (pkt->needsResponse()) {
        DPRINTF(RubyPort, "Sending packet back over port\n");
        pkt->makeHtmTransactionalReqResponse(
            htmRetCodeConversion(htm_return_code));
        port->schedTimingResp(pkt, curTick());
    } else {
        delete pkt;
    }

    trySendRetries();
}

void
HTMSequencer::wakeup()
{
    Sequencer::wakeup();

    // Check for deadlock of any of the requests
    Cycles current_time = curCycle();

    // hardware transactional memory commands
    std::deque<SequencerRequest*>::iterator htm =
      m_htmCmdRequestTable.begin();
    std::deque<SequencerRequest*>::iterator htm_end =
      m_htmCmdRequestTable.end();

    for (; htm != htm_end; ++htm) {
        SequencerRequest* request = *htm;
        if (current_time - request->issue_time < m_deadlock_threshold)
            continue;

        panic("Possible Deadlock detected. Aborting!\n"
              "version: %d m_htmCmdRequestTable: %d "
              "current time: %u issue_time: %d difference: %d\n",
              m_version, m_htmCmdRequestTable.size(),
              current_time * clockPeriod(),
              request->issue_time * clockPeriod(),
              (current_time * clockPeriod()) -
              (request->issue_time * clockPeriod()));
    }
}

bool
HTMSequencer::empty() const
{
    return Sequencer::empty() && m_htmCmdRequestTable.empty();
}

template <class VALUE>
std::ostream &
operator<<(std::ostream &out, const std::deque<VALUE> &queue)
{
    auto i = queue.begin();
    auto end = queue.end();

    out << "[";
    for (; i != end; ++i)
        out << " " << *i;
    out << " ]";

    return out;
}

void
HTMSequencer::print(std::ostream& out) const
{
    Sequencer::print(out);

    out << "+ [HTMSequencer: " << m_version
        << ", htm cmd request table: " << m_htmCmdRequestTable
        << "]";
}

// Insert the request in the request table. Return RequestStatus_Aliased
// if the entry was already present.
RequestStatus
HTMSequencer::insertRequest(PacketPtr pkt, RubyRequestType primary_type,
                            RubyRequestType secondary_type)
{
    if (isHtmCmdRequest(primary_type)) {
        // for the moment, allow just one HTM cmd into the cache controller.
        // Later this can be adjusted for optimization, e.g.
        // back-to-back HTM_Starts.
        if ((m_htmCmdRequestTable.size() > 0) && !pkt->req->isHTMAbort())
            return RequestStatus_BufferFull;

        // insert request into HtmCmd queue
        SequencerRequest* htmReq =
            new SequencerRequest(pkt, primary_type, secondary_type,
                curCycle());
        assert(htmReq);
        m_htmCmdRequestTable.push_back(htmReq);
        return RequestStatus_Ready;
    } else {
        return Sequencer::insertRequest(pkt, primary_type, secondary_type);
    }
}

} // namespace ruby
} // namespace gem5
