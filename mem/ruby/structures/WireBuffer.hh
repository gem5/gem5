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
 * Author: Lisa Hsu
 *
 */

#ifndef __MEM_RUBY_STRUCTURES_WIREBUFFER_HH__
#define __MEM_RUBY_STRUCTURES_WIREBUFFER_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/RubyWireBuffer.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

//////////////////////////////////////////////////////////////////////////////
// This object was written to literally mimic a Wire in Ruby, in the sense
// that there is no way for messages to get reordered en route on the WireBuffer.
// With Message Buffers, even if randomization is off and ordered is on,
// messages can arrive in different orders than they were sent because of
// network issues. This mimics a Wire, such that that is not possible. This can
// allow for messages between closely coupled controllers that are not actually
// separated by a network in real systems to simplify coherence.
/////////////////////////////////////////////////////////////////////////////

class Message;

class WireBuffer : public SimObject
{
  public:
    typedef RubyWireBufferParams Params;
    WireBuffer(const Params &p);
    void init();

    ~WireBuffer();

    void wakeup();

    void setConsumer(Consumer* consumer_ptr)
    {
        m_consumer_ptr = consumer_ptr;
    }
    Consumer* getConsumer() { return m_consumer_ptr; };
    void setDescription(const std::string& name) { m_description = name; };
    std::string getDescription() { return m_description; };

    void enqueue(MsgPtr message, Tick current_time, Tick delta);
    void dequeue(Tick current_time);
    const Message* peek();
    void recycle(Tick current_time, Tick recycle_latency);
    bool isReady(Tick current_time);
    // infinite queue length
    bool areNSlotsAvailable(int n, Tick current_time) { return true; };

    void print(std::ostream& out) const;
    uint64_t m_msg_counter;

  private:
    // Private copy constructor and assignment operator
    WireBuffer (const WireBuffer& obj);
    WireBuffer& operator=(const WireBuffer& obj);

    // data members
    Consumer* m_consumer_ptr;  // Consumer to signal a wakeup()
    std::string m_description;

    // queues where memory requests live
    std::vector<MsgPtr> m_message_queue;
};

std::ostream& operator<<(std::ostream& out, const WireBuffer& obj);

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_WireBuffer_HH__
