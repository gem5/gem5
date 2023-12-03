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

#include "mem/ruby/profiler/AccessTraceForAddress.hh"

#include "mem/ruby/common/Histogram.hh"

namespace gem5
{

namespace ruby
{

AccessTraceForAddress::~AccessTraceForAddress()
{
    if (m_histogram_ptr) {
        delete m_histogram_ptr;
        m_histogram_ptr = NULL;
    }
}

void
AccessTraceForAddress::print(std::ostream &out) const
{
    out << m_addr;

    if (m_histogram_ptr == NULL) {
        out << " " << m_total;
        out << " | " << m_loads;
        out << " " << m_stores;
        out << " " << m_atomics;
        out << " | " << m_user;
        out << " " << m_total - m_user;
        out << " | " << m_sharing;
        out << " | " << m_touched_by.count();
    } else {
        assert(m_total == 0);
        out << " " << (*m_histogram_ptr);
    }
}

void
AccessTraceForAddress::update(RubyRequestType type, RubyAccessMode access_mode,
                              NodeID cpu, bool sharing_miss)
{
    m_touched_by.add(cpu);
    m_total++;
    if (type == RubyRequestType_ATOMIC) {
        m_atomics++;
    } else if (type == RubyRequestType_LD) {
        m_loads++;
    } else if (type == RubyRequestType_ST) {
        m_stores++;
    } else {
        //  ERROR_MSG("Trying to add invalid access to trace");
    }

    if (access_mode == RubyAccessMode_User) {
        m_user++;
    }

    if (sharing_miss) {
        m_sharing++;
    }
}

int
AccessTraceForAddress::getTotal() const
{
    if (m_histogram_ptr == NULL) {
        return m_total;
    } else {
        return m_histogram_ptr->getTotal();
    }
}

void
AccessTraceForAddress::addSample(int value)
{
    assert(m_total == 0);
    if (m_histogram_ptr == NULL) {
        m_histogram_ptr = new Histogram;
    }
    m_histogram_ptr->add(value);
}

} // namespace ruby
} // namespace gem5
