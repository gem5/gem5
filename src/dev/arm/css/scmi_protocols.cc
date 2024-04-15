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

#include "dev/arm/css/scmi_protocols.hh"

#include "debug/SCMI.hh"
#include "dev/arm/css/scmi_platform.hh"

namespace gem5
{

using namespace scmi;

const std::string
Protocol::name() const
{
    return platform.name();
}

BaseProtocol::BaseProtocol(Platform &_platform)
    : Protocol(_platform),
      vendor(platform.params().base_vendor),
      subvendor(platform.params().base_subvendor),
      implementationVersion(platform.params().base_impl_version)
{
    fatal_if(vendor.length() > MAX_STRING_SIZE,
             "Invalid BASE_PROTOCOL VENDOR size\n");
    fatal_if(subvendor.length() > MAX_STRING_SIZE,
             "Invalid BASE_PROTOCOL SUBVENDOR size\n");
}

void
BaseProtocol::handleMessage(Message &msg)
{
    auto message_id = Platform::messageID(msg);

    DPRINTF(SCMI, "Handling SCMI message:\n");
    DPRINTF(SCMI, "# Message Protocol = BASE_PROTOCOL\n");
    DPRINTF(SCMI, "# Message ID = %u\n", message_id);

    switch (static_cast<Commands>(message_id)) {
    case Commands::VERSION:
        version(msg);
        break;
    case Commands::ATTRIBUTES:
        attributes(msg);
        break;
    case Commands::MESSAGE_ATTRIBUTES:
        messageAttributes(msg);
        break;
    case Commands::DISCOVER_VENDOR:
        discoverVendor(msg);
        break;
    case Commands::DISCOVER_SUB_VENDOR:
        discoverSubVendor(msg);
        break;
    case Commands::DISCOVER_IMPLEMENTATION_VERSION:
        discoverImplVersion(msg);
        break;
    case Commands::DISCOVER_LIST_PROTOCOLS:
        discoverListProtocols(msg);
        break;
    case Commands::DISCOVER_AGENT:
        discoverAgent(msg);
        break;
    case Commands::NOTIFY_ERRORS:
    case Commands::SET_DEVICE_PERMISSIONS:
    case Commands::SET_PROTOCOL_PERMISSIONS:
    case Commands::RESET_AGENT_CONFIGURATION:
    default:
        invalidCommand(msg);
        warn("Unimplemented SCMI command: %u\n", message_id);
    }
}

void
BaseProtocol::version(Message &msg)
{
    auto &payload = msg.payload.baseProtocolVersion;
    payload.status = SUCCESS;
    payload.version = PROTOCOL_VERSION;

    // header + status + return
    msg.length = sizeof(uint32_t) * 3;
}

void
BaseProtocol::attributes(Message &msg)
{
    uint32_t _attributes = 0;

    replaceBits(_attributes, 15, 8, platform.numAgents());
    replaceBits(_attributes, 7, 0, platform.numProtocols());

    auto &payload = msg.payload.baseProtocolAttributes;
    payload.status = SUCCESS;
    payload.attributes = _attributes;

    // header + status + return
    msg.length = sizeof(uint32_t) * 3;
}

bool
BaseProtocol::implementedProtocol(Commands message_id) const
{
    switch (message_id) {
    case Commands::VERSION:
    case Commands::ATTRIBUTES:
    case Commands::MESSAGE_ATTRIBUTES:
    case Commands::DISCOVER_VENDOR:
    case Commands::DISCOVER_SUB_VENDOR:
    case Commands::DISCOVER_IMPLEMENTATION_VERSION:
    case Commands::DISCOVER_LIST_PROTOCOLS:
    case Commands::DISCOVER_AGENT:
        return true;
    default:
        return false;
    }
}

void
BaseProtocol::messageAttributes(Message &msg)
{
    auto &payload = msg.payload.baseProtocolMessageAttributes;
    const auto message_id = static_cast<Commands>(payload.messageId);

    if (!implementedProtocol(message_id)) {
        payload.status = NOT_FOUND;
    } else {
        payload.status = SUCCESS;
    }

    // For all messages in the Base protocol, 0 must be returned
    payload.attributes = 0;

    // header + status + return
    msg.length = sizeof(uint32_t) * 3;
}

void
BaseProtocol::discoverVendor(Message &msg)
{
    auto &payload = msg.payload.baseDiscoverVendor;
    payload.status = SUCCESS;

    auto vendor_size =
        vendor.copy((char *)&payload.vendorIdentifier, MAX_STRING_SIZE);

    // header + status + payload
    msg.length = sizeof(uint32_t) * 2 + vendor_size;
}

void
BaseProtocol::discoverSubVendor(Message &msg)
{
    auto &payload = msg.payload.baseDiscoverSubVendor;
    payload.status = SUCCESS;

    auto subvendor_size =
        subvendor.copy((char *)&payload.vendorIdentifier, MAX_STRING_SIZE);

    // header + status + payload
    msg.length = sizeof(uint32_t) * 2 + subvendor_size;
}

void
BaseProtocol::discoverImplVersion(Message &msg)
{
    auto &payload = msg.payload.baseDiscoverImplementationVersion;
    payload.status = SUCCESS;
    payload.implementationVersion = implementationVersion;

    // header + status + return
    msg.length = sizeof(uint32_t) * 3;
}

void
BaseProtocol::discoverListProtocols(Message &msg)
{
    auto &payload = msg.payload.baseDiscoverListProtocols;
    const uint32_t skip = payload.skip;
    const auto num_protocols = platform.numProtocols();

    if (skip > num_protocols) {
        payload.status = INVALID_PARAMETERS;
        msg.length = sizeof(uint32_t) * 2;

    } else {
        const auto &protocol_list = platform.protocolList();
        auto *protocols = (uint8_t *)payload.protocols;
        uint32_t num_implemented = 0;

        for (auto protoc_id = START + skip; protoc_id <= END; protoc_id++) {
            auto it = protocol_list.find(protoc_id);
            if (it != protocol_list.end()) {
                num_implemented++;

                *protocols = it->first;
                protocols++;
            }
        }

        payload.status = SUCCESS;
        payload.numProtocols = num_implemented;

        // header + status + return
        msg.length = sizeof(uint32_t) * 3;
    }
}

void
BaseProtocol::discoverAgent(Message &msg)
{
    auto &payload = msg.payload.baseDiscoverAgent;
    const uint32_t agent_id = payload.agentId;

    if (agent_id > platform.numAgents()) {
        payload.status = NOT_FOUND;
        msg.length = sizeof(uint32_t) * 2;

    } else {
        auto agent_size = 0;
        auto agent_name = std::string();

        if (agent_id) {
            // Subtracting one to the agent_id, since agent_id 0 is reserved
            // for the platform.
            agent_name = platform.getAgent(agent_id - 1);
        } else {
            agent_name = "platform";
        }

        agent_size = agent_name.length();

        strncpy((char *)&payload.name, agent_name.c_str(), agent_size);

        payload.status = SUCCESS;
        // header + status + payload
        msg.length = sizeof(uint32_t) * 2 + agent_size;
    }
}

void
BaseProtocol::invalidCommand(Message &msg)
{
    auto &payload = msg.payload.invalidCommand;
    payload.status = NOT_FOUND;
    msg.length = sizeof(uint32_t) * 2;
}

} // namespace gem5
