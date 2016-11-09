/*
 * Copyright (c) 2012-2014 ARM Limited
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
 *
 * Authors: Thomas Grass
 *          Andreas Hansson
 *          Marco Elver
 */

#include "mem/mem_checker_monitor.hh"

#include <memory>

#include "base/output.hh"
#include "base/trace.hh"
#include "debug/MemCheckerMonitor.hh"

using namespace std;

MemCheckerMonitor::MemCheckerMonitor(Params* params)
    : MemObject(params),
      masterPort(name() + "-master", *this),
      slavePort(name() + "-slave", *this),
      warnOnly(params->warn_only),
      memchecker(params->memchecker)
{}

MemCheckerMonitor::~MemCheckerMonitor()
{}

MemCheckerMonitor*
MemCheckerMonitorParams::create()
{
    return new MemCheckerMonitor(this);
}

void
MemCheckerMonitor::init()
{
    // make sure both sides of the monitor are connected
    if (!slavePort.isConnected() || !masterPort.isConnected())
        fatal("Communication monitor is not connected on both sides.\n");
}

BaseMasterPort&
MemCheckerMonitor::getMasterPort(const std::string& if_name, PortID idx)
{
    if (if_name == "master" || if_name == "mem_side") {
        return masterPort;
    } else {
        return MemObject::getMasterPort(if_name, idx);
    }
}

BaseSlavePort&
MemCheckerMonitor::getSlavePort(const std::string& if_name, PortID idx)
{
    if (if_name == "slave" || if_name == "cpu_side") {
        return slavePort;
    } else {
        return MemObject::getSlavePort(if_name, idx);
    }
}

void
MemCheckerMonitor::recvFunctional(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();
    unsigned size = pkt->getSize();

    // Conservatively reset this address-range. Alternatively we could try to
    // update the values seen by the memchecker, however, there may be other
    // reads/writes to these location from other devices we do not see.
    memchecker->reset(addr, size);

    masterPort.sendFunctional(pkt);

    DPRINTF(MemCheckerMonitor,
            "Forwarded functional access: addr = %#llx, size = %d\n",
            addr, size);
}

void
MemCheckerMonitor::recvFunctionalSnoop(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();
    unsigned size = pkt->getSize();

    // See above.
    memchecker->reset(addr, size);

    slavePort.sendFunctionalSnoop(pkt);

    DPRINTF(MemCheckerMonitor,
            "Received functional snoop: addr = %#llx, size = %d\n",
            addr, size);
}

Tick
MemCheckerMonitor::recvAtomic(PacketPtr pkt)
{
    assert(false && "Atomic not supported");
    return masterPort.sendAtomic(pkt);
}

Tick
MemCheckerMonitor::recvAtomicSnoop(PacketPtr pkt)
{
    assert(false && "Atomic not supported");
    return slavePort.sendAtomicSnoop(pkt);
}

bool
MemCheckerMonitor::recvTimingReq(PacketPtr pkt)
{
    // should always see a request
    assert(pkt->isRequest());

    // Store relevant fields of packet, because packet may be modified
    // or even deleted when sendTiming() is called.
    //
    // For reads we are only interested in real reads, and not prefetches, as
    // it is not guaranteed that the prefetch returns any useful data.
    bool is_read = pkt->isRead() && !pkt->req->isPrefetch();
    bool is_write = pkt->isWrite();
    unsigned size = pkt->getSize();
    Addr addr = pkt->getAddr();
    bool expects_response = pkt->needsResponse() && !pkt->cacheResponding();
    std::unique_ptr<uint8_t[]> pkt_data;
    MemCheckerMonitorSenderState* state = NULL;

    if (expects_response && is_write) {
        // On receipt of a request, only need to allocate pkt_data if this is a
        // write. For reads, we have no data yet, so it doesn't make sense to
        // allocate.
        pkt_data.reset(new uint8_t[size]);
        memcpy(pkt_data.get(), pkt->getConstPtr<uint8_t*>(), size);
    }

    // If a cache miss is served by a cache, a monitor near the memory
    // would see a request which needs a response, but this response
    // would not come back from the memory. Therefore
    // we additionally have to check the inhibit flag.
    if (expects_response && (is_read || is_write)) {
        state = new MemCheckerMonitorSenderState(0);
        pkt->pushSenderState(state);
    }

    // Attempt to send the packet
    bool successful = masterPort.sendTimingReq(pkt);

    // If not successful, restore the sender state
    if (!successful && expects_response && (is_read || is_write)) {
        delete pkt->popSenderState();
    }

    if (successful && expects_response) {
        if (is_read) {
            MemChecker::Serial serial = memchecker->startRead(curTick(),
                                                              addr,
                                                              size);

            // At the time where we push the sender-state, we do not yet know
            // the serial the MemChecker class will assign to this request. We
            // cannot call startRead at the time we push the sender-state, as
            // the masterPort may not be successful in executing sendTimingReq,
            // and in case of a failure, we must not modify the state of the
            // MemChecker.
            //
            // Once we know that sendTimingReq was successful, we can set the
            // serial of the newly constructed sender-state. This is legal, as
            // we know that nobody else will touch nor is responsible for
            // deletion of our sender-state.
            state->serial = serial;

            DPRINTF(MemCheckerMonitor,
                    "Forwarded read request: serial = %d, addr = %#llx, "
                    "size = %d\n",
                    serial, addr, size);
        } else if (is_write) {
            MemChecker::Serial serial = memchecker->startWrite(curTick(),
                                                               addr,
                                                               size,
                                                               pkt_data.get());

            state->serial = serial;

            DPRINTF(MemCheckerMonitor,
                    "Forwarded write request: serial = %d, addr = %#llx, "
                    "size = %d\n",
                    serial, addr, size);
        } else {
            DPRINTF(MemCheckerMonitor,
                    "Forwarded non read/write request: addr = %#llx\n", addr);
        }
    } else if (successful) {
        DPRINTF(MemCheckerMonitor,
                "Forwarded request marked for cache response: addr = %#llx\n",
                addr);
    }

    return successful;
}

bool
MemCheckerMonitor::recvTimingResp(PacketPtr pkt)
{
    // should always see responses
    assert(pkt->isResponse());

    // Store relevant fields of packet, because packet may be modified
    // or even deleted when sendTiming() is called.
    bool is_read = pkt->isRead() && !pkt->req->isPrefetch();
    bool is_write = pkt->isWrite();
    bool is_failed_LLSC = pkt->isLLSC() && pkt->req->getExtraData() == 0;
    unsigned size = pkt->getSize();
    Addr addr = pkt->getAddr();
    std::unique_ptr<uint8_t[]> pkt_data;
    MemCheckerMonitorSenderState* received_state = NULL;

    if (is_read) {
        // On receipt of a response, only need to allocate pkt_data if this is
        // a read. For writes, we have already given the MemChecker the data on
        // the request, so it doesn't make sense to allocate on write.
        pkt_data.reset(new uint8_t[size]);
        memcpy(pkt_data.get(), pkt->getConstPtr<uint8_t*>(), size);
    }

    if (is_read || is_write) {
        received_state =
            dynamic_cast<MemCheckerMonitorSenderState*>(pkt->senderState);

        // Restore initial sender state
        panic_if(received_state == NULL,
                 "Monitor got a response without monitor sender state\n");

        // Restore the state
        pkt->senderState = received_state->predecessor;
    }

    // Attempt to send the packet
    bool successful = slavePort.sendTimingResp(pkt);

    // If packet successfully send, complete transaction in MemChecker
    // instance, and delete sender state, otherwise restore state.
    if (successful) {
        if (is_read) {
            DPRINTF(MemCheckerMonitor,
                    "Received read response: serial = %d, addr = %#llx, "
                    "size = %d\n",
                    received_state->serial, addr, size);

            bool result = memchecker->completeRead(received_state->serial,
                                                   curTick(),
                                                   addr,
                                                   size,
                                                   pkt_data.get());

            if (!result) {
                warn("%s: read of %#llx @ cycle %d failed:\n%s\n",
                     name(),
                     addr, curTick(),
                     memchecker->getErrorMessage().c_str());

                panic_if(!warnOnly, "MemChecker violation!");
            }

            delete received_state;
        } else if (is_write) {
            DPRINTF(MemCheckerMonitor,
                    "Received write response: serial = %d, addr = %#llx, "
                    "size = %d\n",
                    received_state->serial, addr, size);

            if (is_failed_LLSC) {
                // The write was not successful, let MemChecker know.
                memchecker->abortWrite(received_state->serial,
                                       addr,
                                       size);
            } else {
                memchecker->completeWrite(received_state->serial,
                                          curTick(),
                                          addr,
                                          size);
            }

            delete received_state;
        } else {
            DPRINTF(MemCheckerMonitor,
                    "Received non read/write response: addr = %#llx\n", addr);
        }
    } else if (is_read || is_write) {
        // Don't delete anything and let the packet look like we
        // did not touch it
        pkt->senderState = received_state;
    }

    return successful;
}

void
MemCheckerMonitor::recvTimingSnoopReq(PacketPtr pkt)
{
    slavePort.sendTimingSnoopReq(pkt);
}

bool
MemCheckerMonitor::recvTimingSnoopResp(PacketPtr pkt)
{
    return masterPort.sendTimingSnoopResp(pkt);
}

bool
MemCheckerMonitor::isSnooping() const
{
    // check if the connected master port is snooping
    return slavePort.isSnooping();
}

AddrRangeList
MemCheckerMonitor::getAddrRanges() const
{
    // get the address ranges of the connected slave port
    return masterPort.getAddrRanges();
}

void
MemCheckerMonitor::recvReqRetry()
{
    slavePort.sendRetryReq();
}

void
MemCheckerMonitor::recvRespRetry()
{
    masterPort.sendRetryResp();
}

void
MemCheckerMonitor::recvRangeChange()
{
    slavePort.sendRangeChange();
}
