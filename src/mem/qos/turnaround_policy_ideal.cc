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
 *
 * Authors: Matteo Andreozzi
 */

#include "turnaround_policy_ideal.hh"

#include "params/QoSTurnaroundPolicyIdeal.hh"

namespace QoS {

TurnaroundPolicyIdeal::TurnaroundPolicyIdeal(const Params* p)
  : TurnaroundPolicy(p)
{}

TurnaroundPolicyIdeal::~TurnaroundPolicyIdeal()
{}

MemCtrl::BusState
TurnaroundPolicyIdeal::selectBusState()
{
    auto bus_state = memCtrl->getBusState();
    const auto num_priorities = memCtrl->numPriorities();

    // QoS-aware turnaround policy
    // Loop for every queue in the memory controller.
    for (uint8_t i = 0; i < num_priorities; i++) {

        // Starting from top priority queues first
        uint8_t queue_idx = num_priorities - i - 1;

        const uint64_t readq_size = memCtrl->getReadQueueSize(queue_idx);
        const uint64_t writeq_size = memCtrl->getWriteQueueSize(queue_idx);

        // No data for current priority: both the read queue
        // and write queue are empty.
        if ((readq_size == 0) && (writeq_size == 0)) {
            continue;
        }

        // Data found - select state
        if (readq_size == 0) {
            bus_state = MemCtrl::WRITE;
        } else if (writeq_size == 0) {
            bus_state = MemCtrl::READ;
        } else {
            // readq_size > 0 && writeq_size > 0
            bus_state = ((memCtrl->getBusState() == MemCtrl::READ) ?
                    MemCtrl::WRITE : MemCtrl::READ);
        }

        DPRINTF(QOS,
                "QoSMemoryTurnaround::QoSTurnaroundPolicyIdeal - "
                "QoS priority %d queues %d, %d triggering bus %s "
                "in state %s\n", queue_idx, readq_size, writeq_size,
                (bus_state != memCtrl->getBusState()) ?
                "turnaround" : "staying",
                (bus_state == MemCtrl::READ)? "READ" : "WRITE");
        // State selected - exit loop
        break;
    }

    return bus_state;
}

} // namespace QoS

QoS::TurnaroundPolicyIdeal *
QoSTurnaroundPolicyIdealParams::create()
{
    return new QoS::TurnaroundPolicyIdeal(this);
}
