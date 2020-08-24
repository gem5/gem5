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
 * Author: Matteo Andreozzi
 */

#include "mem/qos/q_policy.hh"

#include <unordered_map>
#include <utility>

#include "debug/QOS.hh"
#include "enums/QoSQPolicy.hh"
#include "mem/qos/mem_ctrl.hh"

namespace QoS {

QueuePolicy*
QueuePolicy::create(const QoSMemCtrlParams* p)
{
    switch (p->qos_q_policy) {
      case Enums::QoSQPolicy::fifo:
        return new FifoQueuePolicy(p);
      case Enums::QoSQPolicy::lrg:
        return new LrgQueuePolicy(p);
      case Enums::QoSQPolicy::lifo:
      default:
        return new LifoQueuePolicy(p);
    }
}

QueuePolicy::PacketQueue::iterator
LrgQueuePolicy::selectPacket(PacketQueue* q)
{
    QueuePolicy::PacketQueue::iterator ret = q->end();

    // Tracks one packet per requestor in the queue
    std::unordered_map<RequestorID, QueuePolicy::PacketQueue::iterator> track;

    // Cycle queue only once
    for (auto pkt_it = q->begin(); pkt_it != q->end(); ++pkt_it) {

        const auto& pkt = *pkt_it;

        panic_if(!pkt->req,
                 "QoSQPolicy::lrg detected packet without request");

        // Get Request RequestorID
        RequestorID requestor_id = pkt->req->requestorId();
        DPRINTF(QOS, "QoSQPolicy::lrg checking packet "
                     "from queue with id %d\n", requestor_id);

        // Check if this is a known requestor.
        panic_if(memCtrl->hasRequestor(requestor_id),
                 "%s: Unrecognized Requestor\n", __func__);

        panic_if(toServe.size() > 0,
                 "%s: toServe list is empty\n", __func__);

        if (toServe.front() == requestor_id) {
            DPRINTF(QOS, "QoSQPolicy::lrg matched to served "
                         "requestor id %d\n", requestor_id);
            // This packet matches the RequestorID to be served next
            // move toServe front to back
            toServe.push_back(requestor_id);
            toServe.pop_front();

            return pkt_it;
        }

        // The requestor generating the packet is not first in the toServe list
        // (Doesn't have the highest priority among requestors)
        // Check if this is the first packet seen with its requestor id
        // and remember it. Then keep looping over the remaining packets
        // in the queue.
        if (track.find(requestor_id) == track.end()) {
            track[requestor_id] = pkt_it;
            DPRINTF(QOS, "QoSQPolicy::lrg tracking a packet for "
                         "requestor id %d\n", requestor_id);
        }
    }

    // If here, the current requestor to be serviced doesn't have a pending
    // packet in the queue: look for the next requestor in the list.
    for (const auto& requestorId : toServe) {
        DPRINTF(QOS, "QoSQPolicy::lrg evaluating alternative "
                     "requestor id %d\n", requestorId);

        if (track.find(requestorId) != track.end()) {
            ret = track[requestorId];
            DPRINTF(QOS, "QoSQPolicy::lrg requestor id "
                         "%d selected for service\n", requestorId);

            return ret;
        }
    }

    DPRINTF(QOS, "QoSQPolicy::lrg no packet was serviced\n");

    // Ret will be : packet to serve if any found or queue begin
    // (end if queue is empty)
    return ret;
}

void
LrgQueuePolicy::enqueuePacket(PacketPtr pkt)
{
    RequestorID requestor_id = pkt->requestorId();
    if (!memCtrl->hasRequestor(requestor_id)) {
        toServe.push_back(requestor_id);
    }
};

} // namespace QoS
