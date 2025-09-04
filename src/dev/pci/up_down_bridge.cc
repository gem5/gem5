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

#include "dev/pci/up_down_bridge.hh"

#include "base/addr_range.hh"
#include "base/logging.hh"
#include "debug/PciBridge.hh"
#include "params/PciUpDownBridge.hh"

namespace gem5
{

PciUpDownBridge::PciUpDownBridge(const Params &p)
    : BridgeBase(p), memSideRanges(), configRange()
{}

void
PciUpDownBridge::setConfigRange(AddrRange config_range)
{
    this->configRange = config_range;
    cpuSidePort.sendRangeChange();
}

AddrRangeList
PciUpDownBridge::getAddrRanges() const
{
    AddrRangeList ranges = memSideRanges;

    if (configRange.valid()) {
        // Add whole configuration range, but avoid range duplication for
        // existing PCI devices.
        ranges -= configRange;
        ranges.push_back(configRange);
    }

    return ranges;
}

void
PciUpDownBridge::recvRangeChange()
{
    AddrRangeList new_ranges = memSidePort.getAddrRanges();

    // Avoid potential loop of range change.
    if (new_ranges != memSideRanges) {
        DPRINTF(PciBridge, "Received range change\n");
        for (const auto &r : new_ranges) {
            DPRINTF(PciBridge, "-- %s\n", r.to_string());
        }

        memSideRanges = new_ranges;
        cpuSidePort.sendRangeChange();
    }
}

} // namespace gem5
