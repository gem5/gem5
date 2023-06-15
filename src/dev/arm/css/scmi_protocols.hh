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

#ifndef __DEV_ARM_CSS_SCMI_PROTOCOLS_H__
#define __DEV_ARM_CSS_SCMI_PROTOCOLS_H__

#include <cstdint>
#include <string>

#include "base/compiler.hh"

namespace gem5
{

namespace scmi
{

class Platform;
struct Message;

enum StatusCode
{
    SUCCESS = 0,
    NOT_SUPPORTED = -1,
    INVALID_PARAMETERS = -2,
    DENIED = -3,
    NOT_FOUND = -4,
    OUT_OF_RANGE = -5,
    BUSY = -6,
    COMMS_ERROR = -7,
    GENERIC_ERROR = -8,
    HARDWARE_ERROR = -9,
    PROTOCOL_ERROR = -10
};

class Protocol
{
  public:
    // All agent-platform communications in the SCMI protocol
    // are using 15 as a maximum string size, considering the
    // 16th byte is used for the NULL terminator
    static const uint32_t MAX_STRING_SIZE = 15;

    Protocol(Platform &_platform)
      : platform(_platform)
    {}

    virtual ~Protocol() {}

    virtual void handleMessage(Message &msg) = 0;

    virtual void version(Message &msg) = 0;

    virtual void attributes(Message &msg) = 0;

    virtual void messageAttributes(Message &msg) = 0;

    const std::string name() const;

  protected:
    Platform &platform;
};

/**
 * This protocol describes the properties of the implementation and provides
 * generic error management. The Base protocol provides commands to:
 *   - Describe protocol version
 *   - Discover implementation attributes and vendor identification.
 *   - Discover which protocols are implemented.
 *   - Discover which agents are in the system.
 *   - Register for notifications of platform errors.
 *   - Configure the platform in order to control and modify an agent
 *     visibility of platform resources and commands.
 * This protocol is mandatory.
 */
class BaseProtocol : public Protocol
{
    static const uint32_t PROTOCOL_VERSION = 0x10000;

  public:
    explicit BaseProtocol(Platform &_platform);

    enum class Commands
    {
        VERSION = 0x0,
        ATTRIBUTES = 0x1,
        MESSAGE_ATTRIBUTES = 0x2,
        DISCOVER_VENDOR = 0x3,
        DISCOVER_SUB_VENDOR = 0x4,
        DISCOVER_IMPLEMENTATION_VERSION = 0x5,
        DISCOVER_LIST_PROTOCOLS = 0x6,
        DISCOVER_AGENT = 0x7,
        NOTIFY_ERRORS = 0x8,
        SET_DEVICE_PERMISSIONS = 0x9,
        SET_PROTOCOL_PERMISSIONS = 0xa,
        RESET_AGENT_CONFIGURATION = 0xb
    };

    // Commands
    void handleMessage(Message &msg) override;
    void version(Message &msg) override;
    void attributes(Message &msg) override;
    void messageAttributes(Message &msg) override;
    void discoverVendor(Message &msg);
    void discoverSubVendor(Message &msg);
    void discoverImplVersion(Message &msg);
    void discoverListProtocols(Message &msg);
    void discoverAgent(Message &msg);

    // Invalid Command
    void invalidCommand(Message &msg);

  protected:
    bool implementedProtocol(Commands message_id) const;

    const std::string vendor;
    const std::string subvendor;
    const uint32_t implementationVersion;

};

} // namespace scmi
} // namespace gem5

#endif
