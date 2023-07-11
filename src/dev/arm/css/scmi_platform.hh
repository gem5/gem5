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

#ifndef __DEV_ARM_CSS_SCMI_PLATFORM_H__
#define __DEV_ARM_CSS_SCMI_PLATFORM_H__

#include "base/bitunion.hh"
#include "dev/arm/css/scmi_protocols.hh"
#include "dev/arm/css/scp.hh"
#include "dev/dma_device.hh"
#include "params/ScmiPlatform.hh"

namespace gem5
{

class Doorbell;

namespace scmi
{

class Platform;

// Maximum number of protocols defined by the SCMI specification
static const uint8_t PROTOCOL_MAX = 6;

enum ProtocolID : uint8_t
{
    BASE = 0x10,
    START = 0x11,
    POWER_DOMAIN = START,
    SYSTEM_POWER = 0x12,
    PERFORMANCE_DOMAIN = 0x13,
    CLOCK = 0x14,
    SENSOR = 0x15,
    END = SENSOR
};

enum class MessageType
{
    COMMANDS = 0,
    DELAYED_RESPONSES = 2,
    NOTIFICATIONS = 3
};

BitUnion32(MessageHeader)
    Bitfield<27,18> token;
    Bitfield<17,10> protocolId;
    Bitfield<9,8> messageType;
    Bitfield<7,0> messageId;
EndBitUnion(MessageHeader)

union Payload
{
    struct
    {
        int32_t status;
    } invalidCommand;

    struct
    {
        int32_t status;
        uint32_t version;
    } baseProtocolVersion;

    struct
    {
        int32_t status;
        uint32_t attributes;
    } baseProtocolAttributes;

    struct
    {
        union
        {
            int32_t status;
            uint32_t messageId;
        };
        uint32_t attributes;
    } baseProtocolMessageAttributes;

    struct
    {
        int32_t status;
        uint8_t vendorIdentifier[Protocol::MAX_STRING_SIZE + 1];
    } baseDiscoverVendor;

    struct
    {
        int32_t status;
        uint8_t vendorIdentifier[Protocol::MAX_STRING_SIZE + 1];
    } baseDiscoverSubVendor;

    struct
    {
        int32_t status;
        uint32_t implementationVersion;
    } baseDiscoverImplementationVersion;

    struct
    {
        union
        {
            uint32_t skip;
            int32_t status;
        };
        uint32_t numProtocols;
        uint32_t protocols[(PROTOCOL_MAX - 1)/ 4];
    } baseDiscoverListProtocols;

    struct
    {
        union
        {
            uint32_t agentId;
            int32_t status;
        };
        uint8_t name[Protocol::MAX_STRING_SIZE + 1];
    } baseDiscoverAgent;

    int32_t status;
};

struct Message
{
    uint32_t reserved0;
    uint32_t channelStatus;
    uint64_t reserved1;
    uint32_t mailboxFlags;
    uint32_t length;
    uint32_t header;
    Payload payload;
};

/**
 * Generic communication channel between the Agent and the Platform
 */
class VirtualChannel : public SimObject
{
  public:
    VirtualChannel(const ScmiChannelParams &p)
      : SimObject(p),
        msgBuffer(), pendingMessage(false), shmem(p.shmem_range),
        physID(p.phys_id), virtID(p.virt_id),
        doorbell(p.doorbell)
    {}

    /** Set a pointer to the SCMI platform */
    void
    setPlatform(Platform *_platform)
    {
        platform = _platform;
    }

    Message msgBuffer;
    bool pendingMessage;

    const AddrRange shmem;

    const uint32_t physID;
    const uint32_t virtID;

    DmaPort *dmaPort;
    Doorbell *doorbell;
    Platform *platform;

  private:
    static const int dmaSize = 8; // 64 bits
};

/**
 * This is a Agent to Platform channel (The agent is the initiator)
 */
class AgentChannel : public VirtualChannel
{
  public:
    AgentChannel(const ScmiChannelParams &p);

    void initiateRead();

    void readStatus();
    void readLength();
    void readMessage();
    void handleMessage();

    EventFunctionWrapper readLengthEvent;
    EventFunctionWrapper readMessageEvent;
    EventFunctionWrapper handleMessageEvent;
};

/**
 * This is a Platform to Agent channel (The platform is the initiator)
 */
class PlatformChannel : public VirtualChannel
{
  public:
    PlatformChannel(const ScmiChannelParams &p);

    void writeBackMessage(const Message &msg);
    void notifyAgent();
    void clearDoorbell();
    void complete();

    EventFunctionWrapper clearDoorbellEvent;
    EventFunctionWrapper notifyAgentEvent;
    EventFunctionWrapper completeEvent;

  protected:
    uint32_t agentDoorbellVal;
    uint32_t platformDoorbellVal;
};

/**
 * The SCMI Communication class models a bidirectional
 * communication between the SCMI platform and the agent.
 * As such it has a ScmiAgentChannel and a ScmiPlatformChannel
 * object as members.
 */
class Communication : public SimObject
{
  public:
    Communication(const ScmiCommunicationParams &p)
      : SimObject(p), platformChan(p.platform_channel),
        agentChan(p.agent_channel)
    {}

    PlatformChannel *platformChan;
    AgentChannel *agentChan;
};

class Platform : public Scp
{
  public:
    using ProtocolList = std::unordered_map<uint8_t, Protocol *>;

    PARAMS(ScmiPlatform);
    Platform(const Params &p);
    ~Platform();

    void handleMessage(AgentChannel *ch, Message &msg);

    /** Returns the number of agents in the system */
    uint32_t numAgents() const { return agents.size(); }

    /** Returns the name of an agent given an index */
    const char*
    getAgent(unsigned index) const
    {
        return agents[index].c_str();
    }

    /**
     * Returns the number of protocols implemented, except for
     * the base protocol
     */
    uint32_t numProtocols() const { return protocols.size() - 1; }

    Port& getPort(const std::string &if_name, PortID idx) override;

    void raiseInterrupt(const Doorbell *doorbell) override;
    void clearInterrupt(const Doorbell *doorbell) override;

    static uint32_t
    protocolID(const Message &msg)
    {
        return bits(msg.header, 17, 10);
    }

    static uint32_t
    messageID(const Message &msg)
    {
        return bits(msg.header, 7, 0);
    }

    static uint32_t
    messageType(const Message &msg)
    {
        return bits(msg.header, 9, 8);
    }

    const ProtocolList&
    protocolList() const
    {
        return protocols;
    }

    AgentChannel* find(PlatformChannel* platform) const;
    PlatformChannel* find(AgentChannel* agent) const;

  private:
    std::vector<Communication *> comms;
    const std::vector<std::string> agents;

    ProtocolList protocols;

    DmaPort dmaPort;
};

} // namespace scmi
} // namespace gem5

#endif // __DEV_ARM_CSS_SCMI_PLATFORM_H__
