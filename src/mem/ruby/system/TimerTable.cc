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

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/system/TimerTable.hh"

TimerTable::TimerTable()
{
    m_consumer_ptr  = NULL;
    m_next_valid = false;
    m_next_address = Address(0);
    m_next_time = 0;
}

bool
TimerTable::isReady() const
{
    if (m_map.size() == 0) {
        return false;
    }

    if (!m_next_valid) {
        updateNext();
    }
    assert(m_next_valid);
    return (g_eventQueue_ptr->getTime() >= m_next_time);
}

const Address&
TimerTable::readyAddress() const
{
    assert(isReady());

    if (!m_next_valid) {
        updateNext();
    }
    assert(m_next_valid);
    return m_next_address;
}

void
TimerTable::set(const Address& address, Time relative_latency)
{
    assert(address == line_address(address));
    assert(relative_latency > 0);
    assert(m_map.exist(address) == false);
    Time ready_time = g_eventQueue_ptr->getTime() + relative_latency;
    m_map.add(address, ready_time);
    assert(m_consumer_ptr != NULL);
    g_eventQueue_ptr->scheduleEventAbsolute(m_consumer_ptr, ready_time);
    m_next_valid = false;

    // Don't always recalculate the next ready address
    if (ready_time <= m_next_time) {
        m_next_valid = false;
    }
}

void
TimerTable::unset(const Address& address)
{
    assert(address == line_address(address));
    assert(m_map.exist(address) == true);
    m_map.remove(address);

    // Don't always recalculate the next ready address
    if (address == m_next_address) {
        m_next_valid = false;
    }
}

void
TimerTable::print(std::ostream& out) const
{
}

void
TimerTable::updateNext() const
{
    if (m_map.size() == 0) {
        assert(m_next_valid == false);
        return;
    }

    std::vector<Address> addresses = m_map.keys();
    m_next_address = addresses[0];
    m_next_time = m_map.lookup(m_next_address);

    // Search for the minimum time
    int size = addresses.size();
    for (int i=1; i<size; i++) {
        Address maybe_next_address = addresses[i];
        Time maybe_next_time = m_map.lookup(maybe_next_address);
        if (maybe_next_time < m_next_time) {
            m_next_time = maybe_next_time;
            m_next_address= maybe_next_address;
        }
    }
    m_next_valid = true;
}
