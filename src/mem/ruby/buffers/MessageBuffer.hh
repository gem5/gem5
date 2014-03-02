/*
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

#ifndef __MEM_RUBY_BUFFERS_MESSAGEBUFFER_HH__
#define __MEM_RUBY_BUFFERS_MESSAGEBUFFER_HH__

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "mem/packet.hh"
#include "mem/ruby/buffers/MessageBufferNode.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/slicc_interface/Message.hh"

class MessageBuffer
{
  public:
    MessageBuffer(const std::string &name = "");

    std::string name() const { return m_name; }

    void setRecycleLatency(Cycles recycle_latency)
    { m_recycle_latency = recycle_latency; }

    void reanalyzeMessages(const Address& addr);
    void reanalyzeAllMessages();
    void stallMessage(const Address& addr);

    // TRUE if head of queue timestamp <= SystemTime
    bool isReady() const;

    void
    delayHead()
    {
        MessageBufferNode node = m_prio_heap.front();
        std::pop_heap(m_prio_heap.begin(), m_prio_heap.end(),
                      std::greater<MessageBufferNode>());
        m_prio_heap.pop_back();
        enqueue(node.m_msgptr, Cycles(1));
    }

    bool areNSlotsAvailable(unsigned int n);
    int getPriority() { return m_priority_rank; }
    void setPriority(int rank) { m_priority_rank = rank; }
    void setConsumer(Consumer* consumer)
    {
        if (m_consumer != NULL) {
            fatal("Trying to connect %s to MessageBuffer %s. \
                  \n%s already connected. Check the cntrl_id's.\n",
                  *consumer, *this, *m_consumer);
        }
        m_consumer = consumer;
    }

    void setSender(ClockedObject* obj)
    {
        assert(m_sender == NULL || m_sender == obj);
        m_sender = obj;
    }

    void setReceiver(ClockedObject* obj)
    {
        assert(m_receiver == NULL || m_receiver == obj);
        m_receiver = obj;
    }

    void setDescription(const std::string& name) { m_name = name; }
    std::string getDescription() { return m_name;}

    Consumer* getConsumer() { return m_consumer; }

    //! Function for extracting the message at the head of the
    //! message queue.  The function assumes that the queue is nonempty.
    const Message* peek() const;

    const MsgPtr&
    peekMsgPtr() const
    {
        assert(isReady());
        return m_prio_heap.front().m_msgptr;
    }

    void enqueue(MsgPtr message) { enqueue(message, Cycles(1)); }
    void enqueue(MsgPtr message, Cycles delta);

    //! Updates the delay cycles of the message at the of the queue,
    //! removes it from the queue and returns its total delay.
    Cycles dequeue_getDelayCycles();

    void dequeue();

    void recycle();
    bool isEmpty() const { return m_prio_heap.size() == 0; }

    void
    setOrdering(bool order)
    {
        m_strict_fifo = order;
        m_ordering_set = true;
    }

    void resize(unsigned int size) { m_max_size = size; }
    unsigned int getSize();
    void setRandomization(bool random_flag) { m_randomization = random_flag; }

    void clear();
    void print(std::ostream& out) const;
    void clearStats() { m_not_avail_count = 0; m_msg_counter = 0; }

    void setIncomingLink(int link_id) { m_input_link_id = link_id; }
    void setVnet(int net) { m_vnet_id = net; }

    // Function for figuring out if any of the messages in the buffer can
    // satisfy the read request for the address in the packet.
    // Return value, if true, indicates that the request was fulfilled.
    bool functionalRead(Packet *pkt);

    // Function for figuring out if any of the messages in the buffer need
    // to be updated with the data from the packet.
    // Return value indicates the number of messages that were updated.
    // This required for debugging the code.
    uint32_t functionalWrite(Packet *pkt);

  private:
    void reanalyzeList(std::list<MsgPtr> &, Tick);

  private:
    //added by SS
    Cycles m_recycle_latency;

    // Data Members (m_ prefix)
    //! The two ends of the buffer.
    ClockedObject* m_sender;
    ClockedObject* m_receiver;

    //! Consumer to signal a wakeup(), can be NULL
    Consumer* m_consumer;
    std::vector<MessageBufferNode> m_prio_heap;

    // use a std::map for the stalled messages as this container is
    // sorted and ensures a well-defined iteration order
    typedef std::map< Address, std::list<MsgPtr> > StallMsgMapType;

    StallMsgMapType m_stall_msg_map;
    std::string m_name;

    unsigned int m_max_size;
    Cycles m_time_last_time_size_checked;
    unsigned int m_size_last_time_size_checked;

    // variables used so enqueues appear to happen imediately, while
    // pop happen the next cycle
    Cycles m_time_last_time_enqueue;
    Cycles m_time_last_time_pop;
    unsigned int m_size_at_cycle_start;
    unsigned int m_msgs_this_cycle;

    int m_not_avail_count;  // count the # of times I didn't have N
                            // slots available
    uint64 m_msg_counter;
    int m_priority_rank;
    bool m_strict_fifo;
    bool m_ordering_set;
    bool m_randomization;

    Tick m_last_arrival_time;

    int m_input_link_id;
    int m_vnet_id;
};

Cycles random_time();

inline std::ostream&
operator<<(std::ostream& out, const MessageBuffer& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_BUFFERS_MESSAGEBUFFER_HH__
