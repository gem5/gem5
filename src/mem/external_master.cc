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
 */

#include "mem/external_master.hh"

#include <cctype>
#include <iomanip>

#include "base/trace.hh"
#include "debug/ExternalPort.hh"
#include "sim/system.hh"

std::map<std::string, ExternalMaster::Handler *>
    ExternalMaster::portHandlers;

ExternalMaster::ExternalMaster(ExternalMasterParams *params) :
    SimObject(params),
    externalPort(NULL),
    portName(params->name + ".port"),
    portType(params->port_type),
    portData(params->port_data),
    id(params->system->getRequestorId(this))
{}

Port &
ExternalMaster::getPort(const std::string &if_name, PortID idx)
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
ExternalMaster::init()
{
    if (!externalPort) {
        fatal("ExternalMaster %s: externalPort not set!\n", name());
    } else if (!externalPort->isConnected()) {
        fatal("ExternalMaster %s is unconnected!\n", name());
    }
}

ExternalMaster *
ExternalMasterParams::create()
{
    return new ExternalMaster(this);
}

void
ExternalMaster::registerHandler(const std::string &handler_name,
    Handler *handler)
{
    portHandlers[handler_name] = handler;
}
