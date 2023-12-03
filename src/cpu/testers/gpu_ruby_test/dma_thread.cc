/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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

#include "cpu/testers/gpu_ruby_test/dma_thread.hh"

#include "debug/ProtocolTest.hh"

namespace gem5
{

DmaThread::DmaThread(const Params &_params) : TesterThread(_params)
{
    threadName = "DmaThread(Thread ID " + std::to_string(threadId) + ")";
    threadEvent.setDesc("DmaThread tick");
    assert(numLanes == 1);
}

DmaThread::~DmaThread() {}

void
DmaThread::issueLoadOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::LOAD);
    // we should not have any outstanding fence or atomic op at this point
    assert(pendingFenceCount == 0);
    assert(pendingAtomicCount == 0);

    // DMA thread is a scalar thread so always set lane to zero. This allows
    // us to reuse the API for GPU threads rather than have a specific API
    // for scalar tester threads
    int lane = 0;

    Location location = curAction->getLocation(lane);
    assert(location >= AddressManager::INVALID_LOCATION);

    if (location >= 0) {
        Addr address = addrManager->getAddress(location);
        DPRINTF(ProtocolTest, "%s Episode %d: Issuing Load - Addr %s\n",
                this->getName(), curEpisode->getEpisodeId(),
                ruby::printAddress(address));

        int load_size = sizeof(Value);

        // for now, assert address is 4-byte aligned
        assert(address % load_size == 0);

        auto req = std::make_shared<Request>(address, load_size, 0,
                                             tester->requestorId(), 0,
                                             threadId, nullptr);
        req->setPaddr(address);
        req->setReqInstSeqNum(tester->getActionSeqNum());

        PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
        uint8_t *data = new uint8_t[load_size];
        pkt->dataDynamic(data);
        pkt->senderState = new ProtocolTester::SenderState(this);

        if (!port->sendTimingReq(pkt)) {
            panic("Not expected failed sendTimingReq\n");
        }

        // insert an outstanding load
        addOutstandingReqs(outstandingLoads, address, lane, location);

        // increment the number of outstanding ld_st requests
        pendingLdStCount++;
    }
}

void
DmaThread::issueStoreOps()
{
    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::STORE);
    // we should not have any outstanding fence or atomic op at this point
    assert(pendingFenceCount == 0);
    assert(pendingAtomicCount == 0);

    // DMA thread is a scalar thread so always set lane to zero. This allows
    // us to reuse the API for GPU threads rather than have a specific API
    // for scalar tester threads
    int lane = 0;

    Location location = curAction->getLocation(lane);
    assert(location >= AddressManager::INVALID_LOCATION);

    if (location >= 0) {
        // prepare the next value to store
        Value new_value = addrManager->getLoggedValue(location) + 1;

        Addr address = addrManager->getAddress(location);
        // must be aligned with store size
        assert(address % sizeof(Value) == 0);

        DPRINTF(ProtocolTest,
                "%s Episode %d: Issuing Store - Addr %s - "
                "Value %d\n",
                this->getName(), curEpisode->getEpisodeId(),
                ruby::printAddress(address), new_value);

        auto req = std::make_shared<Request>(address, sizeof(Value), 0,
                                             tester->requestorId(), 0,
                                             threadId, nullptr);
        req->setPaddr(address);
        req->setReqInstSeqNum(tester->getActionSeqNum());

        PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
        uint8_t *writeData = new uint8_t[sizeof(Value)];
        for (int j = 0; j < sizeof(Value); ++j) {
            writeData[j] = ((uint8_t *)&new_value)[j];
        }
        pkt->dataDynamic(writeData);
        pkt->senderState = new ProtocolTester::SenderState(this);

        if (!port->sendTimingReq(pkt)) {
            panic("Not expecting a failed sendTimingReq\n");
        }

        // add an outstanding store
        addOutstandingReqs(outstandingStores, address, lane, location,
                           new_value);

        // increment the number of outstanding ld_st requests
        pendingLdStCount++;
    }
}

void
DmaThread::issueAtomicOps()
{
    DPRINTF(ProtocolTest, "Issuing Atomic Op ...\n");

    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::ATOMIC);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    // no-op: No DMA protocol exists with Atomics
}

void
DmaThread::issueAcquireOp()
{
    DPRINTF(ProtocolTest, "Issuing Acquire Op ...\n");

    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::ACQUIRE);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    // no-op: Acquire does not apply to DMA threads
}

void
DmaThread::issueReleaseOp()
{
    DPRINTF(ProtocolTest, "Issuing Release Op ...\n");

    assert(curAction);
    assert(curAction->getType() == Episode::Action::Type::RELEASE);
    // we should not have any outstanding ops at this point
    assert(pendingFenceCount == 0);
    assert(pendingLdStCount == 0);
    assert(pendingAtomicCount == 0);

    // no-op: Release does not apply to DMA threads
}

void
DmaThread::hitCallback(PacketPtr pkt)
{
    assert(pkt);
    MemCmd resp_cmd = pkt->cmd;
    Addr addr = pkt->getAddr();

    DPRINTF(ProtocolTest,
            "%s Episode %d: hitCallback - Command %s -"
            " Addr %s\n",
            this->getName(), curEpisode->getEpisodeId(), resp_cmd.toString(),
            ruby::printAddress(addr));

    if (resp_cmd == MemCmd::SwapResp) {
        // response to a pending atomic
        assert(pendingAtomicCount > 0);
        assert(pendingLdStCount == 0);
        assert(outstandingAtomics.count(addr) > 0);

        // get return data
        Value value = *(pkt->getPtr<Value>());

        // validate atomic op return
        OutstandingReq req = popOutstandingReq(outstandingAtomics, addr);
        assert(req.lane == 0);
        validateAtomicResp(req.origLoc, req.lane, value);

        // update log table
        addrManager->updateLogTable(req.origLoc, threadId,
                                    curEpisode->getEpisodeId(), value,
                                    curTick(), 0);

        // this Atomic is done
        pendingAtomicCount--;
    } else if (resp_cmd == MemCmd::ReadResp) {
        // response to a pending read
        assert(pendingLdStCount > 0);
        assert(pendingAtomicCount == 0);
        assert(outstandingLoads.count(addr) > 0);

        // get return data
        Value value = *(pkt->getPtr<Value>());
        OutstandingReq req = popOutstandingReq(outstandingLoads, addr);
        assert(req.lane == 0);
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
        // note that unlike GpuWavefront we do decrement pendingLdStCount here
        // since the write is guaranteed to be completed in downstream memory.
        assert(outstandingStores.count(addr) > 0);
        OutstandingReq req = popOutstandingReq(outstandingStores, addr);
        assert(req.storedValue != AddressManager::INVALID_VALUE);

        // update log table
        addrManager->updateLogTable(req.origLoc, threadId,
                                    curEpisode->getEpisodeId(),
                                    req.storedValue, curTick(), 0);

        // the Write is now done
        pendingLdStCount--;
    } else {
        panic("UnsupportedMemCmd response type: %s",
              resp_cmd.toString().c_str());
    }

    delete pkt->senderState;
    delete pkt;

    // record the last active cycle to check for deadlock
    lastActiveCycle = curCycle();

    // we may be able to issue an action. Let's check
    if (!threadEvent.scheduled()) {
        scheduleWakeup();
    }
}

} // namespace gem5
