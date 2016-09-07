/*
 * Copyright (c) 2017-2018 ARM Limited
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

#include "mem_ctrl.hh"

#include "turnaround_policy.hh"

namespace QoS {

MemCtrl::MemCtrl(const QoSMemCtrlParams * p)
  : AbstractMemory(p),
    policy(p->qos_policy),
    turnPolicy(p->qos_turnaround_policy),
    queuePolicy(QueuePolicy::create(p)),
    _numPriorities(p->qos_priorities),
    qosPriorityEscalation(p->qos_priority_escalation),
    qosSyncroScheduler(p->qos_syncro_scheduler),
    totalReadQueueSize(0), totalWriteQueueSize(0),
    busState(READ), busStateNext(READ)
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
MemCtrl::init()
{
    AbstractMemory::init();
}

void
MemCtrl::logRequest(BusState dir, MasterID m_id, uint8_t qos,
                    Addr addr, uint64_t entries)
{
    // If needed, initialize all counters and statistics
    // for this master
    addMaster(m_id);

    DPRINTF(QOS,
            "QoSMemCtrl::logRequest MASTER %s [id %d] address %d"
            " prio %d this master q packets %d"
            " - queue size %d - requested entries %d\n",
            masters[m_id], m_id, addr, qos, packetPriorities[m_id][qos],
            (dir == READ) ? readQueueSizes[qos]: writeQueueSizes[qos],
            entries);

    if (dir == READ) {
        readQueueSizes[qos] += entries;
        totalReadQueueSize += entries;
    } else if (dir == WRITE) {
        writeQueueSizes[qos] += entries;
        totalWriteQueueSize += entries;
    }

    packetPriorities[m_id][qos] += entries;
    for (auto j = 0; j < entries; ++j) {
        requestTimes[m_id][addr].push_back(curTick());
    }

    // Record statistics
    avgPriority[m_id].sample(qos);

    // Compute avg priority distance

    for (uint8_t i = 0; i < packetPriorities[m_id].size(); ++i) {
        uint8_t distance =
            (abs(int(qos) - int(i))) * packetPriorities[m_id][i];

        if (distance > 0) {
            avgPriorityDistance[m_id].sample(distance);
            DPRINTF(QOS,
                    "QoSMemCtrl::logRequest MASTER %s [id %d]"
                    " registering priority distance %d for priority %d"
                    " (packets %d)\n",
                    masters[m_id], m_id, distance, i,
                    packetPriorities[m_id][i]);
        }
    }

    DPRINTF(QOS,
            "QoSMemCtrl::logRequest MASTER %s [id %d] prio %d "
            "this master q packets %d - new queue size %d\n",
            masters[m_id], m_id, qos, packetPriorities[m_id][qos],
            (dir == READ) ? readQueueSizes[qos]: writeQueueSizes[qos]);

}

void
MemCtrl::logResponse(BusState dir, MasterID m_id, uint8_t qos,
                     Addr addr, uint64_t entries, double delay)
{
    panic_if(!hasMaster(m_id),
        "Logging response with invalid master\n");

    DPRINTF(QOS,
            "QoSMemCtrl::logResponse MASTER %s [id %d] address %d prio"
            " %d this master q packets %d"
            " - queue size %d - requested entries %d\n",
            masters[m_id], m_id, addr, qos, packetPriorities[m_id][qos],
            (dir == READ) ? readQueueSizes[qos]: writeQueueSizes[qos],
            entries);

    if (dir == READ) {
        readQueueSizes[qos] -= entries;
        totalReadQueueSize -= entries;
    } else if (dir == WRITE) {
        writeQueueSizes[qos] -= entries;
        totalWriteQueueSize -= entries;
    }

    panic_if(packetPriorities[m_id][qos] == 0,
             "QoSMemCtrl::logResponse master %s negative packets for priority"
             " %d", masters[m_id], qos);

    packetPriorities[m_id][qos] -= entries;

    for (auto j = 0; j < entries; ++j) {
        auto it = requestTimes[m_id].find(addr);
        panic_if(it == requestTimes[m_id].end(),
                 "QoSMemCtrl::logResponse master %s unmatched response for"
                 " address %d received", masters[m_id], addr);

        // Load request time
        uint64_t requestTime = it->second.front();

        // Remove request entry
        it->second.pop_front();

        // Remove whole address entry if last one
        if (it->second.empty()) {
            requestTimes[m_id].erase(it);
        }
        // Compute latency
        double latency = (double) (curTick() + delay - requestTime)
                / SimClock::Float::s;

        if (latency > 0) {
            // Record per-priority latency stats
            if (priorityMaxLatency[qos].value() < latency) {
                priorityMaxLatency[qos] = latency;
            }

            if (priorityMinLatency[qos].value() > latency
                    || priorityMinLatency[qos].value() == 0) {
                priorityMinLatency[qos] = latency;
            }
        }
    }

    DPRINTF(QOS,
            "QoSMemCtrl::logResponse MASTER %s [id %d] prio %d "
            "this master q packets %d - new queue size %d\n",
            masters[m_id], m_id, qos, packetPriorities[m_id][qos],
            (dir == READ) ? readQueueSizes[qos]: writeQueueSizes[qos]);
}

uint8_t
MemCtrl::schedule(MasterID m_id, uint64_t data)
{
    if (policy) {
        return policy->schedule(m_id, data);
    } else {
        DPRINTF(QOS,
                "QoSScheduler::schedule master ID [%d] "
                "data received [%d], but QoS scheduler not initialized\n",
                m_id,data);
        return 0;
    }
}

uint8_t
MemCtrl::schedule(const PacketPtr pkt)
{
    assert(pkt->req);

    if (policy) {
        return schedule(pkt->req->masterId(), pkt->getSize());
    } else {
        DPRINTF(QOS, "QoSScheduler::schedule Packet received [Qv %d], "
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
                "QoSMemoryTurnaround::selectBusState running policy %s\n",
                turnPolicy->name());

        bus_state = turnPolicy->selectBusState();
    } else {
        DPRINTF(QOS,
                "QoSMemoryTurnaround::selectBusState running "
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
MemCtrl::addMaster(MasterID m_id)
{
    if (!hasMaster(m_id)) {
        masters.emplace(m_id, _system->getMasterName(m_id));
        packetPriorities[m_id].resize(numPriorities(), 0);

        DPRINTF(QOS,
                "QoSMemCtrl::addMaster registering"
                " Master %s [id %d]\n",
                masters[m_id], m_id);
    }
}

void
MemCtrl::regStats()
{
    AbstractMemory::regStats();

    using namespace Stats;

    // Initializes per master statistics
    avgPriority.init(_system->maxMasters()).name(name() + ".avgPriority")
        .desc("Average QoS priority value for accepted requests")
        .flags(nozero | nonan).precision(2);

    avgPriorityDistance.init(_system->maxMasters())
        .name(name() + ".avgPriorityDistance")
        .desc("Average QoS priority distance between assigned and "
        "queued values").flags(nozero | nonan);

    priorityMinLatency.init(numPriorities())
        .name(name() + ".priorityMinLatency")
        .desc("per QoS priority minimum request to response latency (s)")
        .precision(12);

    priorityMaxLatency.init(numPriorities())
        .name(name() + ".priorityMaxLatency")
        .desc("per QoS priority maximum request to response latency (s)")
        .precision(12);

    numReadWriteTurnArounds.name(name() + ".numReadWriteTurnArounds")
        .desc("Number of turnarounds from READ to WRITE");

    numWriteReadTurnArounds.name(name() + ".numWriteReadTurnArounds")
        .desc("Number of turnarounds from WRITE to READ");

    numStayReadState.name(name() + ".numStayReadState")
        .desc("Number of times bus staying in READ state");

    numStayWriteState.name(name() + ".numStayWriteState")
        .desc("Number of times bus staying in WRITE state");

    for (int i = 0; i < _system->maxMasters(); i++) {
        const std::string master = _system->getMasterName(i);
        avgPriority.subname(i, master);
        avgPriorityDistance.subname(i, master);
    }

    for (int j = 0; j < numPriorities(); ++j) {
        priorityMinLatency.subname(j, std::to_string(j));
        priorityMaxLatency.subname(j, std::to_string(j));
    }
}

void
MemCtrl::recordTurnaroundStats()
{
    if (busStateNext != busState) {
        if (busState == READ) {
            numWriteReadTurnArounds++;
        } else if (busState == WRITE) {
            numReadWriteTurnArounds++;
        }
    } else {
        if (busState == READ) {
            numStayReadState++;
        } else if (busState == WRITE) {
            numStayWriteState++;
        }
    }
}

} // namespace QoS
