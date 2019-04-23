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
 * Authors: Andrew Bardsley
 */

#include "mem/external_slave.hh"

#include <cctype>
#include <iomanip>

#include "base/trace.hh"
#include "debug/ExternalPort.hh"

/** Implement a `stub' port which just responds to requests by printing
 *  a message.  The stub port can be used to configure and test a system
 *  where the external port is used for a peripheral before connecting
 *  the external port */
class StubSlavePort : public ExternalSlave::ExternalPort
{
  public:
    void processResponseEvent();

    EventFunctionWrapper responseEvent;

    /** Stub can handle a single request at a time.  This will be
     *  NULL when no packet is in flight */
    PacketPtr responsePacket;

    /** Received a new request while processing a first.  Need to ask for
     *  a retry after completing this packet */
    bool mustRetry;

    StubSlavePort(const std::string &name_,
        ExternalSlave &owner_) :
        ExternalSlave::ExternalPort(name_, owner_),
        responseEvent([this]{ processResponseEvent(); }, name()),
        responsePacket(NULL), mustRetry(false)
    { }

    Tick recvAtomic(PacketPtr packet);
    void recvFunctional(PacketPtr packet);
    bool recvTimingReq(PacketPtr packet);
    bool recvTimingSnoopResp(PacketPtr packet);
    void recvRespRetry();
    void recvFunctionalSnoop(PacketPtr packet);
};

class StubSlavePortHandler : public
    ExternalSlave::Handler
{
  public:
    ExternalSlave::ExternalPort *getExternalPort(
        const std::string &name_,
        ExternalSlave &owner,
        const std::string &port_data)
    {
        StringWrap name(name_);

        DPRINTF(ExternalPort, "finding stub port '%s'\n", port_data);
        return new StubSlavePort(name_, owner);
    }
};

Tick
StubSlavePort::recvAtomic(PacketPtr packet)
{
    if (DTRACE(ExternalPort)) {
        unsigned int M5_VAR_USED size = packet->getSize();

        DPRINTF(ExternalPort, "StubSlavePort: recvAtomic a: 0x%x size: %d"
            " data: ...\n", packet->getAddr(), size);
        DDUMP(ExternalPort, packet->getConstPtr<uint8_t>(), size);
    }

    return 0;
}

void
StubSlavePort::recvFunctional(PacketPtr packet)
{
    recvAtomic(packet);
}

void
StubSlavePort::processResponseEvent()
{
    responsePacket->makeResponse();
    responsePacket->headerDelay = 0;
    responsePacket->payloadDelay = 0;

    if (sendTimingResp(responsePacket)) {
        responsePacket = NULL;

        if (mustRetry)
            sendRetryReq();
        mustRetry = false;
    }
}

bool
StubSlavePort::recvTimingReq(PacketPtr packet)
{
    if (responsePacket) {
        mustRetry = true;

        return false;
    } else {
        recvAtomic(packet);

        responsePacket = packet;
        owner.schedule(responseEvent, curTick());

        return true;
    }
}

bool
StubSlavePort::recvTimingSnoopResp(PacketPtr packet)
{
    fatal("StubSlavePort: function: %s\n", __func__);
    return false;
}

void
StubSlavePort::recvRespRetry()
{
    assert(responsePacket);
    /* Stub handles only one response at a time so responseEvent should never
     *  be scheduled at this point.  Retrys shouldn't need to schedule, we
     *  can safely send the response here */
    responseEvent.process();
}

void
StubSlavePort::recvFunctionalSnoop(PacketPtr packet)
{
    fatal("StubSlavePort: unimplemented function: %s\n", __func__);
}

std::map<std::string, ExternalSlave::Handler *>
    ExternalSlave::portHandlers;

AddrRangeList
ExternalSlave::ExternalPort::getAddrRanges() const
{
    return owner.addrRanges;
}

ExternalSlave::ExternalSlave(ExternalSlaveParams *params) :
    SimObject(params),
    externalPort(NULL),
    portName(params->name + ".port"),
    portType(params->port_type),
    portData(params->port_data),
    addrRanges(params->addr_ranges.begin(), params->addr_ranges.end())
{
    /* Register the stub handler if it hasn't already been registered */
    if (portHandlers.find("stub") == portHandlers.end())
        registerHandler("stub", new StubSlavePortHandler);
}

Port &
ExternalSlave::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port") {
        DPRINTF(ExternalPort, "Trying to bind external port: %s %s\n",
            portType, portName);

        if (!externalPort) {
            auto handlerIter = portHandlers.find(portType);

            if (handlerIter == portHandlers.end())
                fatal("Can't find port handler type '%s'\n", portType);

            externalPort = portHandlers[portType]->getExternalPort(portName,
                *this, portData);

            if (!externalPort) {
                fatal("%s: Can't find external port type: %s"
                    " port_data: '%s'\n", portName, portType, portData);
            }
        }
        return *externalPort;
    } else {
        return SimObject::getPort(if_name, idx);
    }
}

void
ExternalSlave::init()
{
    if (!externalPort) {
        fatal("ExternalSlave %s: externalPort not set!\n", name());
    } else if (!externalPort->isConnected()) {
        fatal("ExternalSlave %s is unconnected!\n", name());
    } else {
        externalPort->sendRangeChange();
    }
}

ExternalSlave *
ExternalSlaveParams::create()
{
    return new ExternalSlave(this);
}

void
ExternalSlave::registerHandler(const std::string &handler_name,
    Handler *handler)
{
    portHandlers[handler_name] = handler;
}
