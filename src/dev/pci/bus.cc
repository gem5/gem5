/*
 * Copyright (c) 2025 REDS institute of the HEIG-VD
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

#include "dev/pci/bus.hh"

#include "debug/AddrRanges.hh"
#include "mem/noncoherent_xbar.hh"
#include "params/PciBus.hh"

namespace gem5
{

PciBus::PciBus(const Params &p) : NoncoherentXBar(p)
{
    // create a default port that will be used for configuration cycles
    configErrorPortID = memSidePorts.size();
    std::string portName = name() + ".config_error_port";
    RequestPort *bp =
        new NoncoherentXBarRequestPort(portName, *this, configErrorPortID);
    memSidePorts.push_back(bp);
    reqLayers.push_back(
        new ReqLayer(*bp, *this, csprintf("reqLayer%d", configErrorPortID)));
}

Port &
PciBus::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "config_error_port") {
        return *memSidePorts[configErrorPortID];
    } else {
        return NoncoherentXBar::getPort(if_name, idx);
    }
}

void
PciBus::recvRangeChange(PortID mem_side_port_id)
{
    if (mem_side_port_id == configErrorPortID) {
        // Configuration port is used as a default port for a give range. It
        // doesn't require any check for overlapping ranges and isn't needed
        // for gotAllAddrRanges.
        configRange = memSidePorts[mem_side_port_id]->getAddrRanges();
        DPRINTF(AddrRanges, "Got new config range: %s\n",
                configRange.to_string());
    } else {
        NoncoherentXBar::recvRangeChange(mem_side_port_id);
    }
}

PortID
PciBus::findPort(AddrRange addr_range, PacketPtr pkt)
{
    PortID port_id = NoncoherentXBar::findPort(addr_range, pkt);

    // Configuration error port overrides default port for a given range.
    if (port_id == defaultPortID && addr_range.isSubset(configRange)) {
        return configErrorPortID;
    }

    return port_id;
}

} // namespace gem5
