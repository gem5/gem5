/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/testers/gpu_ruby_test/gpu_wavefront.hh"

#include "debug/ProtocolTest.hh"

namespace gem5
{

GpuWavefront::GpuWavefront(const Params &p)
      : TesterThread(p), cuId(p.cu_id)
{
    threadName = "GpuWavefront(TesterThread ID = " + std::to_string(threadId) +
                 ", CU ID = " + std::to_string(cuId) + ")";
    threadEvent.setDesc("GpuWavefront tick");
}

GpuWavefront::~GpuWavefront()
{

}

void
GpuWavefront::issueLoadOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::LOAD);
    // we should not have any outstanding fence or atomic op at this point
    assert(pendingFenceCount == 0);
    assert(pendingAtomicCount == 0);

    for (int lane = 0; lane < numLanes; ++lane) {
        Location location = curAction->getLocation(lane);
        assert(location >= AddressManager::INVALID_LOCATION);

        // Make a request if we do not get an INVALID_LOCATION for this lane.
        if (location >= 0) {
            Addr address = addrManager->getAddress(location);
            DPRINTF(ProtocolTest, "%s Episode %d: Issuing Load - Addr %s\n",
                    this->getName(), curEpisode->getEpisodeId(),
                    printAddress(address));

            int load_size = sizeof(Value);

            // for now, assert address is 4-byte aligned
            assert(address % load_size == 0);

            auto req = std::make_shared<Request>(address, load_size,
                                                 0, tester->requestorId(),
                                                 0, threadId, nullptr);
            req->setPaddr(address);
            req->setReqInstSeqNum(tester->getActionSeqNum());
            // set protocol-specific flags
            setExtraRequestFlags(req);

            PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
            uint8_t* data = new uint8_t[load_size];
            pkt->dataDynamic(data);
            pkt->senderState = new ProtocolTester::SenderState(this);

            // increment the number of outstanding ld_st requests
            pendingLdStCount++;

            if (!port->sendTimingReq(pkt)) {
                panic("Not expected failed sendTimingReq\n");
            }

            // insert an outstanding load
            addOutstandingReqs(outstandingLoads, address, lane, location);
        }
    }
}

void
GpuWavefront::issueStoreOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::STORE);
    // we should not have any outstanding fence or atomic op at this point
    assert(pendingFenceCount == 0);
    assert(pendingAtomicCount == 0);

    for (int lane = 0; lane < numLanes; ++lane) {
        Location location = curAction->getLocation(lane);
        assert(location >= AddressManager::INVALID_LOCATION);

        // Make a request if we do not get an INVALID_LOCATION for this lane.
        if (location >= 0) {
            // prepare the next value to store
            Value new_value = addrManager->getLoggedValue(location) + 1;

            Addr address = addrManager->getAddress(location);
            // must be aligned with store size
            assert(address % sizeof(Value) == 0);

            DPRINTF(ProtocolTest, "%s Episode %d: Issuing Store - Addr %s - "
                    "Value %d\n", this->getName(),
                    curEpisode->getEpisodeId(), printAddress(address),
                    new_value);

            auto req = std::make_shared<Request>(address, sizeof(Value),
                                                 0, tester->requestorId(), 0,
                                                 threadId, nullptr);
            req->setPaddr(address);
            req->setReqInstSeqNum(tester->getActionSeqNum());
            // set protocol-specific flags
            setExtraRequestFlags(req);

            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
            uint8_t *writeData = new uint8_t[sizeof(Value)];
            for (int j = 0; j < sizeof(Value); ++j) {
                writeData[j] = ((uint8_t*)&new_value)[j];
            }
            pkt->dataDynamic(writeData);
            pkt->senderState = new ProtocolTester::SenderState(this);

            // increment the number of outstanding ld_st requests
            pendingLdStCount++;

            if (!port->sendTimingReq(pkt)) {
                panic("Not expecting a failed sendTimingReq\n");
            }

            // add an outstanding store
            addOutstandingReqs(outstandingStores, address, lane, location,
                               new_value);
        }
    }
}

void
GpuWavefront::issueAtomicOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::ATOMIC);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    // we use atomic_inc in the tester
    Request::Flags flags = Request::ATOMIC_RETURN_OP;

    for (int lane = 0; lane < numLanes; ++lane) {
        Location location = curAction->getLocation(lane);
        assert(location >= 0);

        Addr address = addrManager->getAddress(location);

        DPRINTF(ProtocolTest, "%s Episode %d: Issuing Atomic_Inc - Addr %s\n",
                this->getName(), curEpisode->getEpisodeId(),
                printAddress(address));

        // must be aligned with store size
        assert(address % sizeof(Value) == 0);
        AtomicOpFunctor *amo_op = new AtomicOpInc<Value>();
        auto req = std::make_shared<Request>(address, sizeof(Value),
                                             flags, tester->requestorId(),
                                             0, threadId,
                                             AtomicOpFunctorPtr(amo_op));
        req->setPaddr(address);
        req->setReqInstSeqNum(tester->getActionSeqNum());
        // set protocol-specific flags
        setExtraRequestFlags(req);

        PacketPtr pkt = new Packet(req, MemCmd::SwapReq);
        uint8_t* data = new uint8_t[sizeof(Value)];
        pkt->dataDynamic(data);
        pkt->senderState = new ProtocolTester::SenderState(this);

        if (!port->sendTimingReq(pkt)) {
            panic("Not expecting failed sendTimingReq\n");
        }

        // increment the number of outstanding atomic ops
        pendingAtomicCount++;

        // add an outstanding atomic
        addOutstandingReqs(outstandingAtomics, address, lane, location);
    }
}

void
GpuWavefront::issueAcquireOp()
{
    DPRINTF(ProtocolTest, "%s Episode %d: Issuing Acquire\n", this->getName(),
            curEpisode->getEpisodeId());

    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::ACQUIRE);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    auto acq_req = std::make_shared<Request>(0, 0, 0,
                                             tester->requestorId(), 0,
                                             threadId, nullptr);
    acq_req->setPaddr(0);
    acq_req->setReqInstSeqNum(tester->getActionSeqNum());
    acq_req->setCacheCoherenceFlags(Request::INV_L1);
    // set protocol-specific flags
    setExtraRequestFlags(acq_req);

    PacketPtr pkt = new Packet(acq_req, MemCmd::MemSyncReq);
    pkt->senderState = new ProtocolTester::SenderState(this);

    // increment the number of outstanding fence requests
    pendingFenceCount++;

    if (!port->sendTimingReq(pkt)) {
        panic("Not expecting failed sendTimingReq\n");
    }
}

void
GpuWavefront::issueReleaseOp()
{
    DPRINTF(ProtocolTest, "%s Episode %d: Issuing Release\n", this->getName(),
            curEpisode->getEpisodeId());

    // A release fence simply waits for all previous stores to complete. All
    // previous loads and stores were done before this release operation is
    // issued, so issueReleaseOp is just a no-op in this tester.

    // we may be able to issue an action. Let's check
    if (!threadEvent.scheduled()) {
        scheduleWakeup();
    }
}

void
GpuWavefront::hitCallback(PacketPtr pkt)
{
    assert(pkt);
    MemCmd resp_cmd = pkt->cmd;
    Addr addr = (resp_cmd == MemCmd::WriteCompleteResp) ? 0 : pkt->getAddr();

    DPRINTF(ProtocolTest, "%s Episode %d: hitCallback - Command %s - "
                    "Addr %s\n", this->getName(),
                    curEpisode->getEpisodeId(), resp_cmd.toString(),
                    printAddress(addr));

    // whether the transaction is done after this hitCallback
    bool isTransactionDone = true;

    if (resp_cmd == MemCmd::MemSyncResp) {
        // response to a pending fence
        // no validation needed for fence responses
        assert(pendingFenceCount > 0);
        assert(pendingLdStCount == 0);
        assert(pendingAtomicCount == 0);
        pendingFenceCount--;
    } else if (resp_cmd == MemCmd::ReadResp) {
        // response to a pending read
        assert(pendingLdStCount > 0);
        assert(pendingAtomicCount == 0);
        assert(outstandingLoads.count(addr) > 0);

        // get return data
        Value value = *(pkt->getPtr<Value>());
        OutstandingReq req = popOutstandingReq(outstandingLoads, addr);
        validateLoadResp(req.origLoc, req.lane, value);

        // this Read is done
        pendingLdStCount--;
    } else if (resp_cmd == MemCmd::WriteResp) {
        // response to a pending write
        assert(pendingLdStCount > 0);
        assert(pendingAtomicCount == 0);

        // no need to validate Write response
        // just pop it from the outstanding req table so that subsequent
        // requests dependent on this write can proceed
        // note that we don't decrement pendingLdStCount here yet since
        // the write is not yet completed in downstream memory. Instead, we
        // decrement the counter when we receive the write completion ack
        assert(outstandingStores.count(addr) > 0);
        OutstandingReq req = popOutstandingReq(outstandingStores, addr);
        assert(req.storedValue != AddressManager::INVALID_VALUE);

        // update log table
        addrManager->updateLogTable(req.origLoc, threadId,
                                    curEpisode->getEpisodeId(),
                                    req.storedValue,
                                    curTick(),
                                    cuId);

        // the transaction is not done yet. Waiting for write completion ack
        isTransactionDone = false;
    } else if (resp_cmd == MemCmd::SwapResp) {
        // response to a pending atomic
        assert(pendingAtomicCount > 0);
        assert(pendingLdStCount == 0);
        assert(outstandingAtomics.count(addr) > 0);

        // get return data
        Value value = *(pkt->getPtr<Value>());

        // validate atomic op return
        OutstandingReq req = popOutstandingReq(outstandingAtomics, addr);
        validateAtomicResp(req.origLoc, req.lane, value);

        // update log table
        addrManager->updateLogTable(req.origLoc, threadId,
                                    curEpisode->getEpisodeId(), value,
                                    curTick(),
                                    cuId);

        // this Atomic is done
        pendingAtomicCount--;
    } else if (resp_cmd == MemCmd::WriteCompleteResp) {
        // write completion ACK
        assert(pendingLdStCount > 0);
        assert(pendingAtomicCount == 0);

        // the Write is now done
        pendingLdStCount--;
    } else {
        panic("Unsupported MemCmd response type");
    }

    if (isTransactionDone) {
        // no need to keep senderState and request around
        delete pkt->senderState;
    }

    delete pkt;

    // record the last active cycle to check for deadlock
    lastActiveCycle = curCycle();

    // we may be able to issue an action. Let's check
    if (!threadEvent.scheduled()) {
        scheduleWakeup();
    }
}

void
GpuWavefront::setExtraRequestFlags(RequestPtr req)
{
    // No extra request flag is set
}

} // namespace gem5
