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

#include "dev/arm/css/scmi_platform.hh"

#include <stddef.h>

#include "debug/SCMI.hh"
#include "dev/arm/doorbell.hh"
#include "mem/packet_access.hh"

namespace gem5
{

using namespace scmi;

AgentChannel::AgentChannel(const ScmiChannelParams &p)
    : VirtualChannel(p),
      readLengthEvent([this] { readLength(); }, name()),
      readMessageEvent([this] { readMessage(); }, name()),
      handleMessageEvent([this] { handleMessage(); }, name())
{}

void
AgentChannel::initiateRead()
{
    if (!pendingMessage) {
        pendingMessage = true;
        msgBuffer = Message();
        readStatus();
    } else {
        DPRINTF(SCMI, "Pending message\n");
    }
}

void
AgentChannel::readStatus()
{
    const auto offset = offsetof(Message, channelStatus);
    const Addr address = shmem.start() + offset;

    // Reading the the mailbox to check the
    // channel status. The value will be handled by the readLength
    // event/method
    dmaPort->dmaAction(MemCmd::ReadReq, address, sizeof(uint32_t),
                       &readLengthEvent, (uint8_t *)&msgBuffer.channelStatus,
                       0, Request::UNCACHEABLE);
}

void
AgentChannel::readLength()
{
    DPRINTF(SCMI, "SCMI Virtual channel %u, channel.status: %u\n", virtID,
            msgBuffer.channelStatus);

    // Check if the channel is busy. If it is busy it means there is a
    // message so we need to process it. Abort the reads otherwise
    if (msgBuffer.channelStatus & 0x1) {
        // Channel is free: Terminate: reset message buffer
        pendingMessage = false;
        msgBuffer = Message();
    } else {
        // Read mailbox length
        const auto offset = offsetof(Message, length);
        const Addr address = shmem.start() + offset;

        dmaPort->dmaAction(MemCmd::ReadReq, address, sizeof(msgBuffer.length),
                           &readMessageEvent, (uint8_t *)&msgBuffer.length, 0,
                           Request::UNCACHEABLE);
    }
}

void
AgentChannel::readMessage()
{
    const auto offset = offsetof(Message, header);
    const Addr address = shmem.start() + offset;

    DPRINTF(SCMI, "SCMI Virtual channel %u, message.length: %u\n", virtID,
            msgBuffer.length);

    dmaPort->dmaAction(MemCmd::ReadReq, address, msgBuffer.length,
                       &handleMessageEvent, (uint8_t *)&msgBuffer.header, 0,
                       Request::UNCACHEABLE);
}

void
AgentChannel::handleMessage()
{
    DPRINTF(SCMI, "SCMI Virtual channel %u, message.header: %#x\n", virtID,
            msgBuffer.header);

    // Send the message to the platform which is gonna handle it
    // We are also forwarding a pointer to the agent channel so
    // the platform can retrieve the platform channel
    platform->handleMessage(this, msgBuffer);
}

PlatformChannel::PlatformChannel(const ScmiChannelParams &p)
    : VirtualChannel(p),
      clearDoorbellEvent([this] { clearDoorbell(); }, name()),
      notifyAgentEvent([this] { notifyAgent(); }, name()),
      completeEvent([this] { complete(); }, name()),
      agentDoorbellVal(0),
      platformDoorbellVal(0)
{}

void
PlatformChannel::writeBackMessage(const Message &msg)
{
    DPRINTF(SCMI,
            "SCMI Virtual channel %u, writing back message %u"
            " with status code: %d\n",
            virtID, Platform::messageID(msg), msg.payload.status);

    // Field by field copy of the message
    msgBuffer = msg;

    // Mark the channel as free in the message buffer
    msgBuffer.channelStatus = 0x1;

    dmaPort->dmaAction(MemCmd::WriteReq, shmem.start(), sizeof(msgBuffer),
                       &clearDoorbellEvent, (uint8_t *)&msgBuffer, 0,
                       Request::UNCACHEABLE);
}

void
PlatformChannel::clearDoorbell()
{
    DPRINTF(SCMI, "SCMI Virtual channel %u, clearing doorbell\n", virtID);

    AgentChannel *agent_ch = platform->find(this);
    agent_ch->pendingMessage = false;

    agentDoorbellVal = 0xffffffff;
    dmaPort->dmaAction(MemCmd::WriteReq, agent_ch->doorbell->clearAddress(),
                       sizeof(uint32_t), &notifyAgentEvent,
                       (uint8_t *)&agentDoorbellVal, 0, Request::UNCACHEABLE);
}

void
PlatformChannel::notifyAgent()
{
    DPRINTF(SCMI, "SCMI Virtual channel %u, notifying agent\n", virtID);

    platformDoorbellVal = 1 << virtID;
    dmaPort->dmaAction(MemCmd::WriteReq, doorbell->setAddress(),
                       sizeof(uint32_t), &completeEvent,
                       (uint8_t *)&platformDoorbellVal, 0,
                       Request::UNCACHEABLE);
}

void
PlatformChannel::complete()
{
    pendingMessage = false;
    msgBuffer = Message();
}

Platform::Platform(const ScmiPlatformParams &p)
    : Scp(p),
      comms(p.comms),
      agents(p.agents),
      protocols({ { BASE, new BaseProtocol(*this) } }),
      dmaPort(this, p.sys)
{
    for (auto comm : comms) {
        comm->agentChan->dmaPort = &dmaPort;
        comm->agentChan->setPlatform(this);

        comm->platformChan->dmaPort = &dmaPort;
        comm->platformChan->setPlatform(this);
    }

    fatal_if(numProtocols() >= PROTOCOL_MAX,
             "The number of instantiated protocols are not matching the"
             " architected limit");
}

Platform::~Platform()
{
    for (auto &kv : protocols) {
        delete kv.second;
    }
}

Port &
Platform::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma") {
        return dmaPort;
    }
    return Scp::getPort(if_name, idx);
}

void
Platform::handleMessage(AgentChannel *agent_ch, Message &msg)
{
    auto prot_id = protocolID(msg);

    auto it = protocols.find(prot_id);

    panic_if(it == protocols.end(), "Unimplemented SCMI protocol: %u\n",
             prot_id);

    Protocol *protocol = it->second;
    protocol->handleMessage(msg);

    // Find the platform channel
    PlatformChannel *platform_ch = find(agent_ch);

    // Send the message back to the platform channel
    platform_ch->writeBackMessage(msg);
}

void
Platform::raiseInterrupt(const Doorbell *doorbell)
{
    DPRINTF(SCMI, "Raise interrupt in SCMI platform\n");

    // Now we need to read the physical channel in the mailbox
    // to get the virtual channel, we avoid this

    // Select the associated virtual channel with the doorbell
    for (auto comm : comms) {
        auto channel = comm->agentChan;
        if (channel->doorbell == doorbell) {
            // There is a matching virtual channel: make it
            // start reading the message the shared memory area
            channel->initiateRead();
            return;
        }
    }

    panic("No matching virtual channel\n");
}

void
Platform::clearInterrupt(const Doorbell *doorbell)
{
    DPRINTF(SCMI, "Clear interrupt in SCMI platform\n");
}

AgentChannel *
Platform::find(PlatformChannel *platform) const
{
    for (auto comm : comms) {
        if (comm->platformChan == platform) {
            return comm->agentChan;
        }
    }

    return nullptr;
}

PlatformChannel *
Platform::find(AgentChannel *agent) const
{
    for (auto comm : comms) {
        if (comm->agentChan == agent) {
            return comm->platformChan;
        }
    }

    return nullptr;
}

} // namespace gem5
