/*
 * Copyright (c) 2018, 2024 ARM Limited
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

#ifndef __MEM_QOS_Q_POLICY_HH__
#define __MEM_QOS_Q_POLICY_HH__

#include <deque>
#include <list>
#include <unordered_set>

#include "base/compiler.hh"
#include "mem/packet.hh"
#include "mem/qos/mem_ctrl.hh"
#include "params/QoSMemCtrl.hh"

namespace gem5
{

namespace memory
{

namespace qos
{

/**
 * QoS Queue Policy
 *
 * The QoS Queue Policy class implements algorithms to schedule packets
 * within the same QoS priority queue
 */
class QueuePolicy
{
  public:
    typedef std::deque<PacketPtr> PacketQueue;

    /**
     * This factory method is used for generating the queue policy. It takes
     * the memory controller params as an argument and returns the related
     * QueuePolicy object.  If no particular QueuePolicy has been specified in
     * the QoSMemCtrlParams, the method will default to a LIFO queue policy.
     *
     * @param p qos::MemCtrl parameter variable
     * @return Pointer to the QueuePolicy
     */
    static QueuePolicy* create(const QoSMemCtrlParams &p);

    /**
     * This method is called by the memory controller after it enqueues a
     * packet. It is a way for the queue policy to hook into the packet
     * enqueuing and update relevant metadata. This will then be used once
     * the QueuePolicy::selectPacket will be called.
     *
     * @param pkt Enqueued packet
     */
    virtual void enqueuePacket(PacketPtr pkt) {};

    /**
     * Policy selector.
     * The implementation of this virtual method selects the packet
     * to be serviced from the packet queue passed as an argument.
     *
     * @param queue Non-empty packet queue to select a packet from
     * @return Iterator pointing to the packet in the queue to be
     *         serviced
     */
    virtual PacketQueue::iterator selectPacket(PacketQueue* queue) = 0;

    /**
     * Setting a pointer to the Memory Controller implementing
     * the policy.
     */
    void setMemCtrl(MemCtrl* mem) { memCtrl = mem; };

    virtual ~QueuePolicy() {};

  protected:
    QueuePolicy(const QoSMemCtrlParams &p)
      : memCtrl(nullptr)
    {}

    /** Pointer to parent memory controller implementing the policy */
    MemCtrl* memCtrl;
};

/** Last In First Out Queue Policy */
class LifoQueuePolicy : public QueuePolicy
{
  public:
    LifoQueuePolicy(const QoSMemCtrlParams &p)
      : QueuePolicy(p)
    {}

    /**
     * Implements LIFO packet select policy
     *
     * @param queue The non-empty queue from which to select a packet
     * @return Iterator to the selected packet
     */
    PacketQueue::iterator
    selectPacket(PacketQueue* queue) override;
};

/** First In First Out Queue Policy */
class FifoQueuePolicy : public QueuePolicy
{
  public:
    FifoQueuePolicy(const QoSMemCtrlParams &p)
      : QueuePolicy(p)
    {}

    /**
     * Implements FCFS packet select policy
     *
     * @param queue The non-empty queue from which to select a packet
     * @return Iterator to the selected packet
     */
    PacketQueue::iterator
    selectPacket(PacketQueue* queue) override;
};

/**
 * Least Recently Granted Queue Policy
 * It selects packets from the queue with a round
 * robin-like policy: using the requestor id as a switching
 * parameter rather than switching over a time quantum.
 */
class LrgQueuePolicy : public QueuePolicy
{
  public:
    LrgQueuePolicy(const QoSMemCtrlParams &p)
      : QueuePolicy(p)
    {}

    void enqueuePacket(PacketPtr pkt) override;

    /**
     * Implements LRG packet select policy
     *
     * @param queue The non-empty queue from which to select a packet
     * @return Iterator to the selected packet
     */
    PacketQueue::iterator
    selectPacket(PacketQueue* queue) override;

  protected:
    /**
     * Support structure for lrg algorithms:
     * keeps track of serviced requestors,
     * always serve the front element.
     */
    std::list<RequestorID> toServe;
};

} // namespace qos
} // namespace memory
} // namespace gem5

#endif /* __MEM_QOS_Q_POLICY_HH__ */
