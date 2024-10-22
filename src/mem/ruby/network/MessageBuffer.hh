/*
 * Copyright (c) 2019-2021 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
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

/*
 * Unordered buffer of messages that can be inserted such
 * that they can be dequeued after a given delta time has expired.
 */

#ifndef __MEM_RUBY_NETWORK_MESSAGEBUFFER_HH__
#define __MEM_RUBY_NETWORK_MESSAGEBUFFER_HH__

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/trace.hh"
#include "debug/RubyQueue.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/dummy_port.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/MessageBuffer.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

class MessageBuffer : public SimObject
{
  public:
    typedef MessageBufferParams Params;
    MessageBuffer(const Params &p);

    void reanalyzeMessages(Addr addr, Tick current_time);
    void reanalyzeAllMessages(Tick current_time);
    void stallMessage(Addr addr, Tick current_time);
    // return true if the stall map has a message of this address
    bool hasStalledMsg(Addr addr) const;

    // TRUE if head of queue timestamp <= SystemTime
    bool isReady(Tick current_time) const;

    // earliest tick the head of queue will be ready, or MaxTick if empty
    Tick readyTime() const;

    void
    delayHead(Tick current_time, Tick delta, bool ruby_is_random,
              bool ruby_warmup)
    {
        MsgPtr m = m_prio_heap.front();
        std::pop_heap(m_prio_heap.begin(), m_prio_heap.end(),
                      std::greater<MsgPtr>());
        m_prio_heap.pop_back();
        enqueue(m, current_time, delta, ruby_is_random, ruby_warmup);
    }

    bool areNSlotsAvailable(unsigned int n, Tick curTime);
    int getPriority() { return m_priority_rank; }
    void setPriority(int rank) { m_priority_rank = rank; }
    void setConsumer(Consumer* consumer)
    {
        DPRINTF(RubyQueue, "Setting consumer: %s\n", *consumer);
        if (m_consumer != NULL) {
            fatal("Trying to connect %s to MessageBuffer %s. \
                  \n%s already connected. Check the cntrl_id's.\n",
                  *consumer, *this, *m_consumer);
        }
        m_consumer = consumer;
    }

    Consumer* getConsumer() { return m_consumer; }

    bool getOrdered() { return m_strict_fifo; }

    //! Function for extracting the message at the head of the
    //! message queue.  The function assumes that the queue is nonempty.
    const Message* peek() const;

    const MsgPtr &peekMsgPtr() const { return m_prio_heap.front(); }

    void enqueue(MsgPtr message, Tick curTime, Tick delta,
                bool ruby_is_random, bool ruby_warmup,
                bool bypassStrictFIFO = false);

    // Defer enqueueing a message to a later cycle by putting it aside and not
    // enqueueing it in this cycle
    // The corresponding controller will need to explicitly enqueue the
    // deferred message into the message buffer. Otherwise, the message will
    // be lost.
    void deferEnqueueingMessage(Addr addr, MsgPtr message);

    // enqueue all previously deferred messages that are associated with the
    // input address
    void enqueueDeferredMessages(Addr addr, Tick curTime, Tick delay,
                                 bool ruby_is_random, bool ruby_warmup);
    bool isDeferredMsgMapEmpty(Addr addr) const;

    //! Updates the delay cycles of the message at the head of the queue,
    //! removes it from the queue and returns its total delay.
    Tick dequeue(Tick current_time, bool decrement_messages = true);

    void registerDequeueCallback(std::function<void()> callback);
    void unregisterDequeueCallback();

    void recycle(Tick current_time, Tick recycle_latency);
    bool isEmpty() const { return m_prio_heap.size() == 0; }
    bool isStallMapEmpty() { return m_stall_msg_map.size() == 0; }
    unsigned int getStallMapSize() { return m_stall_msg_map.size(); }

    unsigned int getSize(Tick curTime);

    void clear();
    void print(std::ostream& out) const;
    void clearStats() { m_not_avail_count = 0; m_msg_counter = 0; }

    void setIncomingLink(int link_id) { m_input_link_id = link_id; }
    void setVnet(int net) { m_vnet_id = net; }

    int getIncomingLink() const { return m_input_link_id; }
    int getVnet() const { return m_vnet_id; }

    Port &
    getPort(const std::string &, PortID idx=InvalidPortID) override
    {
        return RubyDummyPort::instance();
    }

    // Function for figuring out if any of the messages in the buffer need
    // to be updated with the data from the packet.
    // Return value indicates the number of messages that were updated.
    uint32_t functionalWrite(Packet *pkt)
    {
        return functionalAccess(pkt, false, nullptr);
    }

    // Function for figuring if message in the buffer has valid data for
    // the packet.
    // Returns true only if a message was found with valid data and the
    // read was performed.
    bool functionalRead(Packet *pkt)
    {
        return functionalAccess(pkt, true, nullptr) == 1;
    }

    // Functional read with mask
    bool functionalRead(Packet *pkt, WriteMask &mask)
    {
        return functionalAccess(pkt, true, &mask) == 1;
    }

    int routingPriority() const { return m_routing_priority; }

  private:
    void reanalyzeList(std::list<MsgPtr> &, Tick);

    uint32_t functionalAccess(Packet *pkt, bool is_read, WriteMask *mask);

  private:
    // Data Members (m_ prefix)
    //! Consumer to signal a wakeup(), can be NULL
    Consumer* m_consumer;
    std::vector<MsgPtr> m_prio_heap;

    std::function<void()> m_dequeue_callback;

    // use a std::map for the stalled messages as this container is
    // sorted and ensures a well-defined iteration order
    typedef std::map<Addr, std::list<MsgPtr> > StallMsgMapType;

    /**
     * A map from line addresses to lists of stalled messages for that line.
     * If this buffer allows the receiver to stall messages, on a stall
     * request, the stalled message is removed from the m_prio_heap and placed
     * in the m_stall_msg_map. Messages are held there until the receiver
     * requests they be reanalyzed, at which point they are moved back to
     * m_prio_heap.
     *
     * NOTE: The stall map holds messages in the order in which they were
     * initially received, and when a line is unblocked, the messages are
     * moved back to the m_prio_heap in the same order. This prevents starving
     * older requests with younger ones.
     */
    StallMsgMapType m_stall_msg_map;

    /**
     * A map from line addresses to corresponding vectors of messages that
     * are deferred for enqueueing. Messages in this map are waiting to be
     * enqueued into the message buffer.
     */
    typedef std::unordered_map<Addr, std::vector<MsgPtr>> DeferredMsgMapType;
    DeferredMsgMapType m_deferred_msg_map;

    /**
     * Current size of the stall map.
     * Track the number of messages held in stall map lists. This is used to
     * ensure that if the buffer is finite-sized, it blocks further requests
     * when the m_prio_heap and m_stall_msg_map contain m_max_size messages.
     */
    int m_stall_map_size;

    /**
     * The maximum capacity. For finite-sized buffers, m_max_size stores a
     * number greater than 0 to indicate the maximum allowed number of messages
     * in the buffer at any time. To get infinitely-sized buffers, set buffer
     * size: m_max_size = 0
     */
    const unsigned int m_max_size;

    /**
     * When != 0, isReady returns false once m_max_dequeue_rate
     * messages have been dequeued in the same cycle.
     */
    const unsigned int m_max_dequeue_rate;

    unsigned int m_dequeues_this_cy;

    Tick m_time_last_time_size_checked;
    unsigned int m_size_last_time_size_checked;

    // variables used so enqueues appear to happen immediately, while
    // pop happen the next cycle
    Tick m_time_last_time_enqueue;
    Tick m_time_last_time_pop;
    Tick m_last_arrival_time;

    unsigned int m_size_at_cycle_start;
    unsigned int m_stalled_at_cycle_start;
    unsigned int m_msgs_this_cycle;

    uint64_t m_msg_counter;
    int m_priority_rank;

    bool m_last_message_strict_fifo_bypassed;

    const bool m_strict_fifo;
    const MessageRandomization m_randomization;
    const bool m_allow_zero_latency;

    const int m_routing_priority;

    int m_input_link_id;
    int m_vnet_id;

    // Count the # of times I didn't have N slots available
    statistics::Scalar m_not_avail_count;
    statistics::Scalar m_msg_count;
    statistics::Average m_buf_msgs;
    statistics::Scalar m_stall_time;
    statistics::Scalar m_stall_count;
    statistics::Formula m_avg_stall_time;
    statistics::Formula m_occupancy;
};

Tick random_time();

inline std::ostream&
operator<<(std::ostream& out, const MessageBuffer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_NETWORK_MESSAGEBUFFER_HH__
