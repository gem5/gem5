/*
 * Copyright (c) 2021 ARM Limited
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

#ifndef __MEM_RUBY_COMMON_QUEUE_HH__
#define __MEM_RUBY_COMMON_QUEUE_HH__

#include <deque>
#include <iostream>

namespace gem5
{

namespace ruby
{

// TriggerQueue helper class is used keep a list of events that trigger the
// actions that need to be executed before an ouststanding transaction
// completes in the CHI protocol. When a transaction no longer has pending
// respose or data messages, this queue is checked and the event at the head
// of the queue is triggered. If the queue is empty, the transactions is
// finalized. Events can be marked as NB (non-blocking). NB are triggered by
// the protocol even if the transactions has pending data/responses.

template<typename T>
class TriggerQueue
{
  private:
    struct ValType
    {
      T val;
      bool non_blocking;
    };
    std::deque<ValType> queue;

  public:
    // Returns the head of the queue
    const T& front() const { return queue.front().val; }

    // Returns the head of the queue
    // NOTE: SLICC won't allow to reuse front() or different
    // values of the template parameter, thus we use an additional
    // def. to workaround that
    const T& next() const { return queue.front().val; }

    // Returns the end of the queue
    const T& back() const { return queue.back().val; }

    // Is the head event non-blocking ?
    bool frontNB() const { return queue.front().non_blocking; }

    // Is the last event non-blocking ?
    bool backNB() const { return queue.back().non_blocking; }

    // Is the queue empty ?
    bool empty() const { return queue.empty(); }

    // put an event at the end of the queue
    void push(const T &elem) { queue.push_back({elem,false}); }

    // emplace an event at the end of the queue
    template<typename... Ts>
    void
    emplace(Ts&&... args)
    {
        queue.push_back({T(std::forward<Ts>(args)...),false});
    }

    // put an event at the head of the queue
    void pushFront(const T &elem) { queue.push_front({elem,false}); }

    // put a non-blocking event at the end of the queue
    void pushNB(const T &elem) { queue.push_back({elem,true}); }

    // put a non-blocking event at the head of the queue
    void pushFrontNB(const T &elem) { queue.push_front({elem,true}); }

    // pop the head of the queue
    void pop() { queue.pop_front(); }

    void print(std::ostream& out) const;
};

template<class T>
inline std::ostream&
operator<<(std::ostream& out, const TriggerQueue<T>& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

template<class T>
inline void
TriggerQueue<T>::print(std::ostream& out) const
{
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_COMMON_QUEUE_HH__
