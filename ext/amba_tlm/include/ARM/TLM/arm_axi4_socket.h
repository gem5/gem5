/*
 * The Clear BSD License
 *
 * Copyright (c) 2015-2021 Arm Limited.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARM_AXI4_SOCKET_H
#define ARM_AXI4_SOCKET_H

#include "arm_axi4_payload.h"
#include "arm_axi4_phase.h"
#include "arm_tlm_socket.h"

namespace ARM
{
/** AMBA AXI protocol support. */
namespace AXI4
{

/**
 * Payload and Phase grouped for use with socket templates in a similar
 * way to tlm::tlm_base_protocol_types.
 */
class ProtocolType
{
public:
    typedef Payload tlm_payload_type;
    typedef Phase tlm_phase_type;
};

/** ARM::TLM::SimpleInitiatorSocket specialised for AXI4 payloads/phases. */
template <typename Module, typename Types = ProtocolType>
class SimpleInitiatorSocket : public ARM::TLM::SimpleInitiatorSocket <Module, Types>
{
private:
    typedef typename ARM::TLM::SimpleInitiatorSocket<Module, Types> BaseType;

public:
    SimpleInitiatorSocket(const char* name_, Module& t,
        typename BaseType::NBFunc bw,
        TLM::Protocol protocol_, unsigned width_) :
        TLM::SimpleInitiatorSocket <Module, Types>(name_, t, bw, protocol_, width_)
    {}
};

/** ARM::TLM::SimpleTargetSocket specialised for AXI4 payloads/phases. */
template <typename Module, typename Types = ProtocolType>
class SimpleTargetSocket : public ARM::TLM::SimpleTargetSocket <Module, Types>
{
private:
    typedef typename ARM::TLM::SimpleTargetSocket<Module, Types> BaseType;

public:
    SimpleTargetSocket(const char* name_, Module& t,
        typename BaseType::NBFunc fw,
        TLM::Protocol protocol_, unsigned width_,
        typename BaseType::DebugFunc dbg = nullptr) :
        TLM::SimpleTargetSocket <Module, Types>(name_, t, fw,
            protocol_, width_, dbg)
    {}
};

}
}

#endif /* ARM_AXI4_SOCKET_H */
