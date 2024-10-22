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

#include "mem/ruby/structures/TimerTable.hh"

#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

TimerTable::TimerTable()
    : m_next_time(0)
{
    m_consumer_ptr  = NULL;
    m_next_valid = false;
    m_next_address = 0;
}

bool
TimerTable::isReady(Tick curTime) const
{
    if (m_map.empty())
        return false;

    if (!m_next_valid) {
        updateNext();
    }
    assert(m_next_valid);
    return (curTime >= m_next_time);
}

Addr
TimerTable::nextAddress() const
{
    if (!m_next_valid) {
        updateNext();
    }
    assert(m_next_valid);
    return m_next_address;
}

void
TimerTable::set(Addr address, Tick ready_time)
{
    assert(address == makeLineAddress(address, m_block_size_bits));
    assert(!m_map.count(address));

    m_map[address] = ready_time;
    assert(m_consumer_ptr != NULL);
    m_consumer_ptr->scheduleEventAbsolute(ready_time);
    m_next_valid = false;

    // Don't always recalculate the next ready address
    if (ready_time <= m_next_time) {
        m_next_valid = false;
    }
}

void
TimerTable::unset(Addr address)
{
    assert(address == makeLineAddress(address, m_block_size_bits));
    assert(m_map.count(address));
    m_map.erase(address);

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
    if (m_map.empty()) {
        assert(!m_next_valid);
        return;
    }

    AddressMap::const_iterator i = m_map.begin();
    AddressMap::const_iterator end = m_map.end();

    m_next_address = i->first;
    m_next_time = i->second;
    ++i;

    for (; i != end; ++i) {
        if (i->second < m_next_time) {
            m_next_address = i->first;
            m_next_time = i->second;
        }
    }

    m_next_valid = true;
}

} // namespace ruby
} // namespace gem5
