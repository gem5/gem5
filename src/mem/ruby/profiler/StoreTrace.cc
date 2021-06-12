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

#include "mem/ruby/profiler/StoreTrace.hh"

#include "sim/cur_tick.hh"

namespace gem5
{

namespace ruby
{

bool StoreTrace::s_init = false; // Total number of store lifetimes of
                                 // all lines
int64_t StoreTrace::s_total_samples = 0; // Total number of store
                                       // lifetimes of all lines
Histogram* StoreTrace::s_store_count_ptr = NULL;
Histogram* StoreTrace::s_store_first_to_stolen_ptr = NULL;
Histogram* StoreTrace::s_store_last_to_stolen_ptr = NULL;
Histogram* StoreTrace::s_store_first_to_last_ptr = NULL;

StoreTrace::StoreTrace(Addr addr)
    : m_store_count(-1), m_store_first_to_stolen(-1),
      m_store_last_to_stolen(-1), m_store_first_to_last(-1)
{
    StoreTrace::initSummary();
    m_addr = addr;
    m_total_samples = 0;

    // Really -1 isn't valid, so this will trigger the initilization code
    m_last_writer = -1;
    m_stores_this_interval = 0;
}

StoreTrace::~StoreTrace()
{
}

void
StoreTrace::print(std::ostream& out) const
{
    out << m_addr
        << " total_samples: " << m_total_samples << std::endl
        << "store_count: " << m_store_count << std::endl
        << "store_first_to_stolen: " << m_store_first_to_stolen << std::endl
        << "store_last_to_stolen: " << m_store_last_to_stolen << std::endl
        << "store_first_to_last: " << m_store_first_to_last  << std::endl;
}

void
StoreTrace::initSummary()
{
    if (!s_init) {
        s_total_samples = 0;
        s_store_count_ptr = new Histogram(-1);
        s_store_first_to_stolen_ptr = new Histogram(-1);
        s_store_last_to_stolen_ptr = new Histogram(-1);
        s_store_first_to_last_ptr = new Histogram(-1);
    }
    s_init = true;
}

void
StoreTrace::printSummary(std::ostream& out)
{
    out << "total_samples: " << s_total_samples << std::endl;
    out << "store_count: " << (*s_store_count_ptr) << std::endl;
    out << "store_first_to_stolen: "
        << (*s_store_first_to_stolen_ptr) << std::endl;
    out << "store_last_to_stolen: "
        << (*s_store_last_to_stolen_ptr) << std::endl;
    out << "store_first_to_last: " << (*s_store_first_to_last_ptr)
        << std::endl;
}

void
StoreTrace::clearSummary()
{
    StoreTrace::initSummary();
    s_total_samples = 0;
    s_store_count_ptr->clear();
    s_store_first_to_stolen_ptr->clear();
    s_store_last_to_stolen_ptr->clear();
    s_store_first_to_last_ptr->clear();
}

void
StoreTrace::store(NodeID node)
{
    Tick current = curTick();

    assert((m_last_writer == -1) || (m_last_writer == node));

    m_last_writer = node;
    if (m_last_writer == -1) {
        assert(m_stores_this_interval == 0);
    }

    if (m_stores_this_interval == 0) {
        // A new proessor just wrote the line, so reset the stats
        m_first_store = current;
    }

    m_last_store = current;
    m_stores_this_interval++;
}

void
StoreTrace::downgrade(NodeID node)
{
    if (node == m_last_writer) {
        Tick current = curTick();
        assert(m_stores_this_interval != 0);
        assert(m_last_store != 0);
        assert(m_first_store != 0);
        assert(m_last_writer != -1);

        // Per line stats
        m_store_first_to_stolen.add(current - m_first_store);
        m_store_count.add(m_stores_this_interval);
        m_store_last_to_stolen.add(current - m_last_store);
        m_store_first_to_last.add(m_last_store - m_first_store);
        m_total_samples++;

        // Global stats
        assert(s_store_first_to_stolen_ptr != NULL);
        s_store_first_to_stolen_ptr->add(current - m_first_store);
        s_store_count_ptr->add(m_stores_this_interval);
        s_store_last_to_stolen_ptr->add(current - m_last_store);
        s_store_first_to_last_ptr->add(m_last_store - m_first_store);
        s_total_samples++;

        // Initilize for next go round
        m_stores_this_interval = 0;
        m_last_store = 0;
        m_first_store = 0;
        m_last_writer = -1;
    }
}

} // namespace ruby
} // namespace gem5
