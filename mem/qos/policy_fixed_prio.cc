/*
 * Copyright (c) 2018 ARM Limited
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

#include "mem/qos/policy_fixed_prio.hh"

#include <algorithm>
#include <functional>

#include "base/trace.hh"
#include "debug/QOS.hh"
#include "mem/request.hh"
#include "params/QoSFixedPriorityPolicy.hh"

namespace gem5
{

namespace memory
{

namespace qos
{

FixedPriorityPolicy::FixedPriorityPolicy(const Params &p)
  : Policy(p), defaultPriority(p.qos_fixed_prio_default_prio)
{}

FixedPriorityPolicy::~FixedPriorityPolicy()
{}

void
FixedPriorityPolicy::init()
{
}

void
FixedPriorityPolicy::initRequestorName(std::string requestor, uint8_t priority)
{
    priorityMap.insert(
        this->pair<std::string, uint8_t>(requestor, priority));
}

void
FixedPriorityPolicy::initRequestorObj(const SimObject* requestor,
                                   uint8_t priority)
{
    priorityMap.insert(
        this->pair<const SimObject*, uint8_t>(requestor, priority));
}

uint8_t
FixedPriorityPolicy::schedule(const RequestorID id, const uint64_t data)
{
    // Reads a packet's RequestorID contained in its encapsulated request
    // if a match is found in the configured priority map, returns the
    // matching priority, else returns zero

    auto ret = priorityMap.find(id);

    if (ret != priorityMap.end()) {
        return ret->second;
    } else {
        DPRINTF(QOS, "Requestor %s (RequestorID %d) not present in "
                     "priorityMap, assigning default priority %d\n",
                      memCtrl->system()->getRequestorName(id),
                      id, defaultPriority);
        return defaultPriority;
    }
}

} // namespace qos
} // namespace memory
} // namespace gem5
