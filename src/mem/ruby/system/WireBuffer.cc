/*
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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
 *
 * Author:  Lisa Hsu
 *
 */

#include <algorithm>
#include <functional>

#include "base/cprintf.hh"
#include "base/stl_helpers.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/system/WireBuffer.hh"

using namespace std;

// Output operator definition

ostream&
operator<<(ostream& out, const WireBuffer& obj)
{
    obj.print(out);
    out << flush;
    return out;
}


// ****************************************************************

// CONSTRUCTOR
WireBuffer::WireBuffer(const Params *p)
    : SimObject(p)
{
    m_msg_counter = 0;
}

void
WireBuffer::init()
{
}

WireBuffer::~WireBuffer()
{
}

void
WireBuffer::enqueue(MsgPtr message, Cycles latency)
{
    m_msg_counter++;
    Cycles current_time = g_system_ptr->curCycle();
    Cycles arrival_time = current_time + latency;
    assert(arrival_time > current_time);

    MessageBufferNode thisNode(arrival_time, m_msg_counter, message);
    m_message_queue.push_back(thisNode);
    if (m_consumer_ptr != NULL) {
        m_consumer_ptr->
            scheduleEventAbsolute(g_system_ptr->clockPeriod() * arrival_time);
    } else {
        panic("No Consumer for WireBuffer! %s\n", *this);
    }
}

void
WireBuffer::dequeue()
{
    assert(isReady());
    pop_heap(m_message_queue.begin(), m_message_queue.end(),
        greater<MessageBufferNode>());
    m_message_queue.pop_back();
}

const Message*
WireBuffer::peek()
{
    MessageBufferNode node = peekNode();
    Message* msg_ptr = node.m_msgptr.get();
    assert(msg_ptr != NULL);
    return msg_ptr;
}

MessageBufferNode
WireBuffer::peekNode()
{
    assert(isReady());
    MessageBufferNode req = m_message_queue.front();
    return req;
}

void
WireBuffer::recycle()
{
    // Because you don't want anything reordered, make sure the recycle latency
    // is just 1 cycle. As a result, you really want to use this only in
    // Wire-like situations because you don't want to deadlock as a result of
    // being stuck behind something if you're not actually supposed to.
    assert(isReady());
    MessageBufferNode node = m_message_queue.front();
    pop_heap(m_message_queue.begin(), m_message_queue.end(),
        greater<MessageBufferNode>());

    node.m_time = g_system_ptr->curCycle() + Cycles(1);
    m_message_queue.back() = node;
    push_heap(m_message_queue.begin(), m_message_queue.end(),
        greater<MessageBufferNode>());
    m_consumer_ptr->
        scheduleEventAbsolute(g_system_ptr->clockPeriod() * node.m_time);
}

bool
WireBuffer::isReady()
{
    return ((!m_message_queue.empty()) &&
            (m_message_queue.front().m_time <= g_system_ptr->curCycle()));
}

void
WireBuffer::print(ostream& out) const
{
}

void
WireBuffer::wakeup()
{
}

WireBuffer *
RubyWireBufferParams::create()
{
    return new WireBuffer(this);
}

