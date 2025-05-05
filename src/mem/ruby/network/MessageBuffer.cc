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

#include "mem/ruby/network/MessageBuffer.hh"

#include <cassert>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyQueue.hh"

namespace gem5
{

namespace ruby
{

using stl_helpers::operator<<;

MessageBuffer::MessageBuffer(const Params &p)
    : SimObject(p), m_stall_map_size(0), m_max_size(p.buffer_size),
    m_max_dequeue_rate(p.max_dequeue_rate), m_dequeues_this_cy(0),
    m_time_last_time_size_checked(0),
    m_time_last_time_enqueue(0), m_time_last_time_pop(0),
    m_last_arrival_time(0), m_last_message_strict_fifo_bypassed(false),
    m_strict_fifo(p.ordered),
    m_randomization(p.randomization),
    m_allow_zero_latency(p.allow_zero_latency),
    m_routing_priority(p.routing_priority),
    ADD_STAT(m_not_avail_count, statistics::units::Count::get(),
             "Number of times this buffer did not have N slots available"),
    ADD_STAT(m_msg_count, statistics::units::Count::get(),
             "Number of messages passed the buffer"),
    ADD_STAT(m_buf_msgs, statistics::units::Rate<
                statistics::units::Count, statistics::units::Tick>::get(),
             "Average number of messages in buffer"),
    ADD_STAT(m_stall_time, statistics::units::Tick::get(),
             "Total number of ticks messages were stalled in this buffer"),
    ADD_STAT(m_stall_count, statistics::units::Count::get(),
             "Number of times messages were stalled"),
    ADD_STAT(m_avg_stall_time, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average stall ticks per message"),
    ADD_STAT(m_occupancy, statistics::units::Rate<
                statistics::units::Ratio, statistics::units::Tick>::get(),
             "Average occupancy of buffer capacity")
{
    m_msg_counter = 0;
    m_consumer = NULL;
    m_size_last_time_size_checked = 0;
    m_size_at_cycle_start = 0;
    m_stalled_at_cycle_start = 0;
    m_msgs_this_cycle = 0;
    m_priority_rank = 0;

    m_stall_msg_map.clear();
    m_input_link_id = 0;
    m_vnet_id = 0;

    m_buf_msgs = 0;
    m_stall_time = 0;

    m_dequeue_callback = nullptr;

    // stats
    m_not_avail_count
        .flags(statistics::nozero);

    m_msg_count
        .flags(statistics::nozero);

    m_buf_msgs
        .flags(statistics::nozero);

    m_stall_count
        .flags(statistics::nozero);

    m_avg_stall_time
        .flags(statistics::nozero | statistics::nonan);

    m_occupancy
        .flags(statistics::nozero);

    m_stall_time
        .flags(statistics::nozero);

    if (m_max_size > 0) {
        m_occupancy = m_buf_msgs / m_max_size;
    } else {
        m_occupancy = 0;
    }

    m_avg_stall_time = m_stall_time / m_msg_count;
}

unsigned int
MessageBuffer::getSize(Tick curTime)
{
    if (m_time_last_time_size_checked != curTime) {
        m_time_last_time_size_checked = curTime;
        m_size_last_time_size_checked = m_prio_heap.size();
    }

    return m_size_last_time_size_checked;
}

bool
MessageBuffer::areNSlotsAvailable(unsigned int n, Tick current_time)
{

    // fast path when message buffers have infinite size
    if (m_max_size == 0) {
        return true;
    }

    // determine the correct size for the current cycle
    // pop operations shouldn't effect the network's visible size
    // until schd cycle, but enqueue operations effect the visible
    // size immediately
    unsigned int current_size = 0;
    unsigned int current_stall_size = 0;

    if (m_time_last_time_pop < current_time) {
        // no pops this cycle - heap and stall queue size is correct
        current_size = m_prio_heap.size();
        current_stall_size = m_stall_map_size;
    } else {
        if (m_time_last_time_enqueue < current_time) {
            // no enqueues this cycle - m_size_at_cycle_start is correct
            current_size = m_size_at_cycle_start;
        } else {
            // both pops and enqueues occured this cycle - add new
            // enqueued msgs to m_size_at_cycle_start
            current_size = m_size_at_cycle_start + m_msgs_this_cycle;
        }

        // Stall queue size at start is considered
        current_stall_size = m_stalled_at_cycle_start;
    }

    // now compare the new size with our max size
    if (current_size + current_stall_size + n <= m_max_size) {
        return true;
    } else {
        DPRINTF(RubyQueue, "n: %d, current_size: %d, heap size: %d, "
                "m_max_size: %d\n",
                n, current_size + current_stall_size,
                m_prio_heap.size(), m_max_size);
        m_not_avail_count++;
        return false;
    }
}

const Message*
MessageBuffer::peek() const
{
    DPRINTF(RubyQueue, "Peeking at head of queue.\n");
    const Message* msg_ptr = m_prio_heap.front().get();
    assert(msg_ptr);

    DPRINTF(RubyQueue, "Message: %s\n", (*msg_ptr));
    return msg_ptr;
}

// FIXME - move me somewhere else
Tick
random_time()
{
    static Random::RandomPtr rng(Random::genRandom());
    Tick time = 1;
    time += rng->random(0, 3);  // [0...3]
    if (rng->random(0, 7) == 0) {  // 1 in 8 chance
        time += 100 + rng->random(1, 15); // 100 + [1...15]
    }
    return time;
}

void
MessageBuffer::enqueue(MsgPtr message, Tick current_time, Tick delta,
                       bool ruby_is_random, bool ruby_warmup,
                       bool bypassStrictFIFO)
{
    // record current time incase we have a pop that also adjusts my size
    if (m_time_last_time_enqueue < current_time) {
        m_msgs_this_cycle = 0;  // first msg this cycle
        m_time_last_time_enqueue = current_time;
    }

    m_msg_counter++;
    m_msgs_this_cycle++;

    // Calculate the arrival time of the message, that is, the first
    // cycle the message can be dequeued.
    panic_if((delta == 0) && !m_allow_zero_latency,
           "Delta equals zero and allow_zero_latency is false during enqueue");
    Tick arrival_time = 0;

    // random delays are inserted if the RubySystem level randomization flag
    // is turned on and this buffer allows it
    if ((m_randomization == MessageRandomization::disabled) ||
        ((m_randomization == MessageRandomization::ruby_system) &&
          !ruby_is_random)) {
        // No randomization
        arrival_time = current_time + delta;
    } else {
        // Randomization - ignore delta
        if (m_strict_fifo) {
            if (m_last_arrival_time < current_time) {
                m_last_arrival_time = current_time;
            }
            arrival_time = m_last_arrival_time + random_time();
        } else {
            arrival_time = current_time + random_time();
        }
    }

    // Check the arrival time
    assert(arrival_time >= current_time);
    if (m_strict_fifo &&
        !(bypassStrictFIFO || m_last_message_strict_fifo_bypassed)) {
        if (arrival_time < m_last_arrival_time) {
            panic("FIFO ordering violated: %s name: %s current time: %d "
                  "delta: %d arrival_time: %d last arrival_time: %d\n",
                  *this, name(), current_time, delta, arrival_time,
                  m_last_arrival_time);
        }
    }

    // If running a cache trace, don't worry about the last arrival checks
    if (!ruby_warmup) {
        m_last_arrival_time = arrival_time;
    }

    m_last_message_strict_fifo_bypassed = bypassStrictFIFO;

    // compute the delay cycles and set enqueue time
    Message* msg_ptr = message.get();
    assert(msg_ptr != NULL);

    assert(current_time >= msg_ptr->getLastEnqueueTime() &&
           "ensure we aren't dequeued early");

    msg_ptr->updateDelayedTicks(current_time);
    msg_ptr->setLastEnqueueTime(arrival_time);
    msg_ptr->setMsgCounter(m_msg_counter);

    // Insert the message into the priority heap
    m_prio_heap.push_back(message);
    push_heap(m_prio_heap.begin(), m_prio_heap.end(), std::greater<MsgPtr>());
    // Increment the number of messages statistic
    m_buf_msgs++;

    assert((m_max_size == 0) ||
           ((m_prio_heap.size() + m_stall_map_size) <= m_max_size));

    DPRINTF(RubyQueue, "Enqueue arrival_time: %lld, Message: %s\n",
            arrival_time, *(message.get()));

    // Schedule the wakeup
    assert(m_consumer != NULL);
    m_consumer->scheduleEventAbsolute(arrival_time);
    m_consumer->storeEventInfo(m_vnet_id);
}

Tick
MessageBuffer::dequeue(Tick current_time, bool decrement_messages)
{
    DPRINTF(RubyQueue, "Popping\n");
    assert(isReady(current_time));

    // get MsgPtr of the message about to be dequeued
    MsgPtr message = m_prio_heap.front();

    // get the delay cycles
    message->updateDelayedTicks(current_time);
    Tick delay = message->getDelayedTicks();

    // record previous size and time so the current buffer size isn't
    // adjusted until schd cycle
    if (m_time_last_time_pop < current_time) {
        m_size_at_cycle_start = m_prio_heap.size();
        m_stalled_at_cycle_start = m_stall_map_size;
        m_time_last_time_pop = current_time;
        m_dequeues_this_cy = 0;
    }
    ++m_dequeues_this_cy;

    pop_heap(m_prio_heap.begin(), m_prio_heap.end(), std::greater<MsgPtr>());
    m_prio_heap.pop_back();
    if (decrement_messages) {
        // Record how much time is passed since the message was enqueued
        m_stall_time += curTick() - message->getLastEnqueueTime();
        m_msg_count++;

        // If the message will be removed from the queue, decrement the
        // number of message in the queue.
        m_buf_msgs--;
    }

    // if a dequeue callback was requested, call it now
    if (m_dequeue_callback) {
        m_dequeue_callback();
    }

    return delay;
}

void
MessageBuffer::registerDequeueCallback(std::function<void()> callback)
{
    m_dequeue_callback = callback;
}

void
MessageBuffer::unregisterDequeueCallback()
{
    m_dequeue_callback = nullptr;
}

void
MessageBuffer::clear()
{
    m_prio_heap.clear();

    m_msg_counter = 0;
    m_time_last_time_enqueue = 0;
    m_time_last_time_pop = 0;
    m_size_at_cycle_start = 0;
    m_stalled_at_cycle_start = 0;
    m_msgs_this_cycle = 0;
}

void
MessageBuffer::recycle(Tick current_time, Tick recycle_latency)
{
    DPRINTF(RubyQueue, "Recycling.\n");
    assert(isReady(current_time));
    MsgPtr node = m_prio_heap.front();
    pop_heap(m_prio_heap.begin(), m_prio_heap.end(), std::greater<MsgPtr>());

    Tick future_time = current_time + recycle_latency;
    node->setLastEnqueueTime(future_time);

    m_prio_heap.back() = node;
    push_heap(m_prio_heap.begin(), m_prio_heap.end(), std::greater<MsgPtr>());
    m_consumer->scheduleEventAbsolute(future_time);
}

void
MessageBuffer::reanalyzeList(std::list<MsgPtr> &lt, Tick schdTick)
{
    while (!lt.empty()) {
        MsgPtr m = lt.front();
        assert(m->getLastEnqueueTime() <= schdTick);

        m_prio_heap.push_back(m);
        push_heap(m_prio_heap.begin(), m_prio_heap.end(),
                  std::greater<MsgPtr>());

        m_consumer->scheduleEventAbsolute(schdTick);

        DPRINTF(RubyQueue, "Requeue arrival_time: %lld, Message: %s\n",
            schdTick, *(m.get()));

        lt.pop_front();
    }
}

void
MessageBuffer::reanalyzeMessages(Addr addr, Tick current_time)
{
    DPRINTF(RubyQueue, "ReanalyzeMessages %#x\n", addr);
    assert(m_stall_msg_map.count(addr) > 0);

    //
    // Put all stalled messages associated with this address back on the
    // prio heap.  The reanalyzeList call will make sure the consumer is
    // scheduled for the current cycle so that the previously stalled messages
    // will be observed before any younger messages that may arrive this cycle
    //
    m_stall_map_size -= m_stall_msg_map[addr].size();
    assert(m_stall_map_size >= 0);
    reanalyzeList(m_stall_msg_map[addr], current_time);
    m_stall_msg_map.erase(addr);
}

void
MessageBuffer::reanalyzeAllMessages(Tick current_time)
{
    DPRINTF(RubyQueue, "ReanalyzeAllMessages\n");

    //
    // Put all stalled messages associated with this address back on the
    // prio heap.  The reanalyzeList call will make sure the consumer is
    // scheduled for the current cycle so that the previously stalled messages
    // will be observed before any younger messages that may arrive this cycle.
    //
    for (StallMsgMapType::iterator map_iter = m_stall_msg_map.begin();
         map_iter != m_stall_msg_map.end(); ++map_iter) {
        m_stall_map_size -= map_iter->second.size();
        assert(m_stall_map_size >= 0);
        reanalyzeList(map_iter->second, current_time);
    }
    m_stall_msg_map.clear();
}

void
MessageBuffer::stallMessage(Addr addr, Tick current_time)
{
    DPRINTF(RubyQueue, "Stalling due to %#x\n", addr);
    assert(isReady(current_time));
    MsgPtr message = m_prio_heap.front();

    // Since the message will just be moved to stall map, indicate that the
    // buffer should not decrement the m_buf_msgs statistic
    dequeue(current_time, false);

    //
    // Note: no event is scheduled to analyze the map at a later time.
    // Instead the controller is responsible to call reanalyzeMessages when
    // these addresses change state.
    //
    (m_stall_msg_map[addr]).push_back(message);
    m_stall_map_size++;
    m_stall_count++;
}

bool
MessageBuffer::hasStalledMsg(Addr addr) const
{
    return (m_stall_msg_map.count(addr) != 0);
}

void
MessageBuffer::deferEnqueueingMessage(Addr addr, MsgPtr message)
{
    DPRINTF(RubyQueue, "Deferring enqueueing message: %s, Address %#x\n",
            *(message.get()), addr);
    (m_deferred_msg_map[addr]).push_back(message);
}

void
MessageBuffer::enqueueDeferredMessages(Addr addr, Tick curTime, Tick delay,
                                       bool ruby_is_random, bool ruby_warmup)
{
    assert(!isDeferredMsgMapEmpty(addr));
    std::vector<MsgPtr>& msg_vec = m_deferred_msg_map[addr];
    assert(msg_vec.size() > 0);

    // enqueue all deferred messages associated with this address
    for (MsgPtr m : msg_vec) {
        enqueue(m, curTime, delay, ruby_is_random, ruby_warmup);
    }

    msg_vec.clear();
    m_deferred_msg_map.erase(addr);
}

bool
MessageBuffer::isDeferredMsgMapEmpty(Addr addr) const
{
    return m_deferred_msg_map.count(addr) == 0;
}

void
MessageBuffer::print(std::ostream& out) const
{
    ccprintf(out, "[MessageBuffer: ");
    if (m_consumer != NULL) {
        ccprintf(out, " consumer-yes ");
    }

    std::vector<MsgPtr> copy(m_prio_heap);
    std::sort_heap(copy.begin(), copy.end(), std::greater<MsgPtr>());
    ccprintf(out, "%s] %s", copy, name());
}

bool
MessageBuffer::isReady(Tick current_time) const
{
    assert(m_time_last_time_pop <= current_time);
    bool can_dequeue = (m_max_dequeue_rate == 0) ||
                       (m_time_last_time_pop < current_time) ||
                       (m_dequeues_this_cy < m_max_dequeue_rate);
    bool is_ready = (m_prio_heap.size() > 0) &&
                   (m_prio_heap.front()->getLastEnqueueTime() <= current_time);
    if (!can_dequeue && is_ready) {
        // Make sure the Consumer executes next cycle to dequeue the ready msg
        m_consumer->scheduleEvent(Cycles(1));
    }
    return can_dequeue && is_ready;
}

Tick
MessageBuffer::readyTime() const
{
    if (m_prio_heap.empty())
        return MaxTick;
    else
        return m_prio_heap.front()->getLastEnqueueTime();
}

uint32_t
MessageBuffer::functionalAccess(Packet *pkt, bool is_read, WriteMask *mask)
{
    DPRINTF(RubyQueue, "functional %s for %#x\n",
            is_read ? "read" : "write", pkt->getAddr());

    uint32_t num_functional_accesses = 0;

    // Check the priority heap and write any messages that may
    // correspond to the address in the packet.
    for (unsigned int i = 0; i < m_prio_heap.size(); ++i) {
        Message *msg = m_prio_heap[i].get();
        if (is_read && !mask && msg->functionalRead(pkt))
            return 1;
        else if (is_read && mask && msg->functionalRead(pkt, *mask))
            num_functional_accesses++;
        else if (!is_read && msg->functionalWrite(pkt))
            num_functional_accesses++;
    }

    // Check the stall queue and write any messages that may
    // correspond to the address in the packet.
    for (StallMsgMapType::iterator map_iter = m_stall_msg_map.begin();
         map_iter != m_stall_msg_map.end();
         ++map_iter) {

        for (std::list<MsgPtr>::iterator it = (map_iter->second).begin();
            it != (map_iter->second).end(); ++it) {

            Message *msg = (*it).get();
            if (is_read && !mask && msg->functionalRead(pkt))
                return 1;
            else if (is_read && mask && msg->functionalRead(pkt, *mask))
                num_functional_accesses++;
            else if (!is_read && msg->functionalWrite(pkt))
                num_functional_accesses++;
        }
    }

    return num_functional_accesses;
}

} // namespace ruby
} // namespace gem5
