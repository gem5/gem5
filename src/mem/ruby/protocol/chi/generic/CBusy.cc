/*
 * Copyright (c) 2026 Arm Limited
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

#include "mem/ruby/protocol/chi/generic/CBusy.hh"

#include "mem/ruby/protocol/CHI/Cache_Controller.hh"
#include "mem/ruby/protocol/CHI/Memory_Controller.hh"

namespace gem5
{

namespace ruby
{

namespace CHI
{

BaseCBusy::BaseCBusy(const Params &p)
    : BackpressureGen(p),
      threshold0(p.threshold_0),
      threshold1(p.threshold_1),
      threshold2(p.threshold_2)
{}

int
CBusy::generate(AbstractController *ctrl, const Message &msg) const
{
    return generateFromTbeOccupancy<CHI::Cache_Controller>(ctrl);
}

CBusyTracker::CBusyTracker(const Params &p)
    : BackpressureTracker(p), stats(this)
{}

void
CBusyTracker::update(const MachineID &mach_id, int cbusy)
{
    entries.updateCBusy(mach_id, cbusy);
    stats.avgCBusy = entries.avgCBusy();
}

void
CBusyTracker::updateDMT(const MachineID &mach_id, int cbusy)
{
    dmtEntries.updateCBusy(mach_id, cbusy);
    stats.avgCBusyDMT = dmtEntries.avgCBusy();
}

void
CBusyTracker::print(std::ostream &out) const
{
    out << "AvgCBusy=" << entries.avgCBusy() << "/" << dmtEntries.avgCBusy();
}

void
CBusyTracker::CBusyTrackerHelper::updateCBusy(const MachineID &mach_id,
                                              int cbusy)
{
    auto aux = cbusyEntries.insert(std::make_pair(mach_id, 0));
    int &curr_cbusy = aux.first->second;
    assert(curr_cbusy <= cbusySum);
    cbusySum -= curr_cbusy;
    cbusySum += cbusy;
    curr_cbusy = cbusy;
}

int
CBusyTracker::CBusyTrackerHelper::avgCBusy() const
{
    return (cbusyEntries.size() != 0)
               ? (cbusySum + (cbusyEntries.size() / 2)) / cbusyEntries.size()
               : 0;
}

int
CBusyTracker::CBusyTrackerHelper::maxCBusy() const
{
    int max = 0;
    for (const auto &e : cbusyEntries) {
        if (e.second > max) {
            max = e.second;
        }
    }
    return max;
}

CBusyTracker::CBusyTrackerStats::CBusyTrackerStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(avgCBusy, "Avg CBusy value on request responses"),
      ADD_STAT(avgCBusyDMT, "Avg CBusy value on DMT request responses")
{}

int
SnfCBusy::generate(AbstractController *ctrl, const Message &msg) const
{
    return generateFromTbeOccupancy<CHI::Memory_Controller>(ctrl);
}

} // namespace CHI
} // namespace ruby
} // namespace gem5
