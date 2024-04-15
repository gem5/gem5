/*
 * Copyright 2021 Google, Inc.
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

#include "mem/sys_bridge.hh"

#include "sim/system.hh"

namespace gem5
{

SysBridge::PacketData
SysBridge::BridgingPort::replaceReqID(PacketPtr pkt)
{
    RequestPtr old_req = pkt->req;
    RequestPtr new_req = std::make_shared<Request>(
        old_req->getPaddr(), old_req->getSize(), old_req->getFlags(), id);
    pkt->req = new_req;
    return { old_req };
}

SysBridge::SysBridge(const SysBridgeParams &p)
    : SimObject(p),
      sourcePort(p.name + ".source_port", &targetPort,
                 p.target->getRequestorId(this)),
      targetPort(p.name + ".target_port", &sourcePort,
                 p.source->getRequestorId(this))
{}

Port &
SysBridge::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "source_port")
        return sourcePort;
    else if (if_name == "target_port")
        return targetPort;
    else
        return SimObject::getPort(if_name, idx);
}

} // namespace gem5
