/*
 * Copyright (c) 2017-2020 ARM Limited
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

#include "mem/qos/mem_ctrl.hh"

#include "mem/qos/policy.hh"
#include "mem/qos/q_policy.hh"
#include "mem/qos/turnaround_policy.hh"
#include "sim/core.hh"

namespace gem5
{

namespace memory
{

namespace qos
{

MemCtrl::MemCtrl(const QoSMemCtrlParams &p)
  : ClockedObject(p),
    policy(p.qos_policy),
    turnPolicy(p.qos_turnaround_policy),
    queuePolicy(QueuePolicy::create(p)),
    _numPriorities(p.qos_priorities),
    qosPriorityEscalation(p.qos_priority_escalation),
    qosSyncroScheduler(p.qos_syncro_scheduler),
    totalReadQueueSize(0), totalWriteQueueSize(0),
    busState(READ), busStateNext(READ),
    stats(*this),
    _system(p.system)
{
    // Set the priority policy
    if (policy) {
        policy->setMemCtrl(this);
    }

    // Set the queue priority policy
    if (queuePolicy) {
        queuePolicy->setMemCtrl(this);
    }

    // Set the bus turnaround policy
    if (turnPolicy) {
        turnPolicy->setMemCtrl(this);
    }

    readQueueSizes.resize(_numPriorities);
    writeQueueSizes.resize(_numPriorities);
    serviceTick.resize(_numPriorities);
}

MemCtrl::~MemCtrl()
{}

void
MemCtrl::logRequest(BusState dir, RequestorID id, uint8_t _qos,
                    Addr addr, uint64_t entries)
{
    // If needed, initialize all counters and statistics
    // for this requestor
    addRequestor(id);

    DPRINTF(QOS,
            "qos::MemCtrl::logRequest REQUESTOR %s [id %d] address %#x"
            " prio %d this requestor q packets %d"
            " - queue size %d - requested entries %d\n",
            requestors[id], id, addr, _qos, packetPriorities[id][_qos],
            (dir == READ) ? readQueueSizes[_qos]: writeQueueSizes[_qos],
            entries);

    if (dir == READ) {
        readQueueSizes[_qos] += entries;
        totalReadQueueSize += entries;
    } else if (dir == WRITE) {
        writeQueueSizes[_qos] += entries;
        totalWriteQueueSize += entries;
    }

    packetPriorities[id][_qos] += entries;
    for (auto j = 0; j < entries; ++j) {
        requestTimes[id][addr].push_back(curTick());
    }

    // Record statistics
    stats.avgPriority[id].sample(_qos);

    // Compute avg priority distance

    for (uint8_t i = 0; i < packetPriorities[id].size(); ++i) {
        uint8_t distance =
            (abs(int(_qos) - int(i))) * packetPriorities[id][i];

        if (distance > 0) {
            stats.avgPriorityDistance[id].sample(distance);
            DPRINTF(QOS,
                    "qos::MemCtrl::logRequest REQUESTOR %s [id %d]"
                    " registering priority distance %d for priority %d"
                    " (packets %d)\n",
                    requestors[id], id, distance, i,
                    packetPriorities[id][i]);
        }
    }

    DPRINTF(QOS,
            "qos::MemCtrl::logRequest REQUESTOR %s [id %d] prio %d "
            "this requestor q packets %d - new queue size %d\n",
            requestors[id], id, _qos, packetPriorities[id][_qos],
            (dir == READ) ? readQueueSizes[_qos]: writeQueueSizes[_qos]);

}

void
MemCtrl::logResponse(BusState dir, RequestorID id, uint8_t _qos,
                     Addr addr, uint64_t entries, double delay)
{
    panic_if(!hasRequestor(id),
        "Logging response with invalid requestor\n");

    DPRINTF(QOS,
            "qos::MemCtrl::logResponse REQUESTOR %s [id %d] address %#x prio"
            " %d this requestor q packets %d"
            " - queue size %d - requested entries %d\n",
            requestors[id], id, addr, _qos, packetPriorities[id][_qos],
            (dir == READ) ? readQueueSizes[_qos]: writeQueueSizes[_qos],
            entries);

    if (dir == READ) {
        readQueueSizes[_qos] -= entries;
        totalReadQueueSize -= entries;
    } else if (dir == WRITE) {
        writeQueueSizes[_qos] -= entries;
        totalWriteQueueSize -= entries;
    }

    panic_if(packetPriorities[id][_qos] == 0,
             "qos::MemCtrl::logResponse requestor %s negative packets "
             "for priority %d", requestors[id], _qos);

    packetPriorities[id][_qos] -= entries;

    for (auto j = 0; j < entries; ++j) {
        auto it = requestTimes[id].find(addr);
        panic_if(it == requestTimes[id].end(),
                 "qos::MemCtrl::logResponse requestor %s unmatched response "
                 "for address %#x received", requestors[id], addr);

        // Load request time
        uint64_t requestTime = it->second.front();

        // Remove request entry
        it->second.pop_front();

        // Remove whole address entry if last one
        if (it->second.empty()) {
            requestTimes[id].erase(it);
        }
        // Compute latency
        double latency = (double) (curTick() + delay - requestTime)
                / sim_clock::as_float::s;

        if (latency > 0) {
            // Record per-priority latency stats
            if (stats.priorityMaxLatency[_qos].value() < latency) {
                stats.priorityMaxLatency[_qos] = latency;
            }

            if (stats.priorityMinLatency[_qos].value() > latency
                    || stats.priorityMinLatency[_qos].value() == 0) {
                stats.priorityMinLatency[_qos] = latency;
            }
        }
    }

    DPRINTF(QOS,
            "qos::MemCtrl::logResponse REQUESTOR %s [id %d] prio %d "
            "this requestor q packets %d - new queue size %d\n",
            requestors[id], id, _qos, packetPriorities[id][_qos],
            (dir == READ) ? readQueueSizes[_qos]: writeQueueSizes[_qos]);
}

uint8_t
MemCtrl::schedule(RequestorID id, uint64_t data)
{
    if (policy) {
        return policy->schedule(id, data);
    } else {
        DPRINTF(QOS,
                "qos::MemCtrl::schedule requestor id [%d] "
                "data received [%d], but QoS scheduler not initialized\n",
                id,data);
        return 0;
    }
}

uint8_t
MemCtrl::schedule(const PacketPtr pkt)
{
    assert(pkt->req);

    if (policy) {
        return schedule(pkt->req->requestorId(), pkt->getSize());
    } else {
        DPRINTF(QOS, "qos::MemCtrl::schedule Packet received [Qv %d], "
                "but QoS scheduler not initialized\n",
                pkt->qosValue());
        return pkt->qosValue();
    }
}

MemCtrl::BusState
MemCtrl::selectNextBusState()
{
    auto bus_state = getBusState();

    if (turnPolicy) {
        DPRINTF(QOS,
                "qos::MemCtrl::selectNextBusState running policy %s\n",
                turnPolicy->name());

        bus_state = turnPolicy->selectBusState();
    } else {
        DPRINTF(QOS,
                "qos::MemCtrl::selectNextBusState running "
                "default bus direction selection policy\n");

        if ((!getTotalReadQueueSize() && bus_state == MemCtrl::READ) ||
            (!getTotalWriteQueueSize() && bus_state == MemCtrl::WRITE)) {
            // READ/WRITE turnaround
            bus_state = (bus_state == MemCtrl::READ) ? MemCtrl::WRITE :
                                                       MemCtrl::READ;

        }
    }

    return bus_state;
}

void
MemCtrl::addRequestor(RequestorID id)
{
    if (!hasRequestor(id)) {
        requestors.emplace(id, _system->getRequestorName(id));
        packetPriorities[id].resize(numPriorities(), 0);

        DPRINTF(QOS,
                "qos::MemCtrl::addRequestor registering"
                " Requestor %s [id %d]\n",
                requestors[id], id);
    }
}

MemCtrl::MemCtrlStats::MemCtrlStats(MemCtrl &mc)
    : statistics::Group(&mc),
    memCtrl(mc),

    ADD_STAT(avgPriority, statistics::units::Count::get(),
             "Average QoS priority value for accepted requests"),
    ADD_STAT(avgPriorityDistance, statistics::units::Count::get(),
             "Average QoS priority distance between assigned and queued "
             "values"),

    ADD_STAT(priorityMinLatency, statistics::units::Second::get(),
             "per QoS priority minimum request to response latency"),
    ADD_STAT(priorityMaxLatency, statistics::units::Second::get(),
             "per QoS priority maximum request to response latency"),
    ADD_STAT(numReadWriteTurnArounds, statistics::units::Count::get(),
             "Number of turnarounds from READ to WRITE"),
    ADD_STAT(numWriteReadTurnArounds, statistics::units::Count::get(),
             "Number of turnarounds from WRITE to READ"),
    ADD_STAT(numStayReadState, statistics::units::Count::get(),
             "Number of times bus staying in READ state"),
    ADD_STAT(numStayWriteState, statistics::units::Count::get(),
             "Number of times bus staying in WRITE state")
{
}

void
MemCtrl::MemCtrlStats::regStats()
{
    statistics::Group::regStats();

    using namespace statistics;

    System *system = memCtrl._system;
    const auto max_requestors = system->maxRequestors();
    const auto num_priorities = memCtrl.numPriorities();

    // Initializes per requestor statistics
    avgPriority
        .init(max_requestors)
        .flags(nozero | nonan)
        .precision(2)
        ;

    avgPriorityDistance
        .init(max_requestors)
        .flags(nozero | nonan)
        ;

    priorityMinLatency
        .init(num_priorities)
        .precision(12)
        ;

    priorityMaxLatency
        .init(num_priorities)
        .precision(12)
        ;

    for (int i = 0; i < max_requestors; i++) {
        const std::string name = system->getRequestorName(i);
        avgPriority.subname(i, name);
        avgPriorityDistance.subname(i, name);
    }

    for (int j = 0; j < num_priorities; ++j) {
        priorityMinLatency.subname(j, std::to_string(j));
        priorityMaxLatency.subname(j, std::to_string(j));
    }
}

void
MemCtrl::recordTurnaroundStats()
{
    if (busStateNext != busState) {
        if (busState == READ) {
            stats.numWriteReadTurnArounds++;
        } else if (busState == WRITE) {
            stats.numReadWriteTurnArounds++;
        }
    } else {
        if (busState == READ) {
            stats.numStayReadState++;
        } else if (busState == WRITE) {
            stats.numStayWriteState++;
        }
    }
}

} // namespace qos
} // namespace memory
} // namespace gem5
