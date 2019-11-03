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

#ifndef __MEM_RUBY_PROFILER_ACCESSTRACEFORADDRESS_HH__
#define __MEM_RUBY_PROFILER_ACCESSTRACEFORADDRESS_HH__

#include <iostream>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Set.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"

class Histogram;

class AccessTraceForAddress
{
  public:
    AccessTraceForAddress()
        : m_loads(0), m_stores(0), m_atomics(0), m_total(0), m_user(0),
          m_sharing(0), m_histogram_ptr(NULL)
    { }
    ~AccessTraceForAddress();

    void setAddress(Addr addr) { m_addr = addr; }
    void update(RubyRequestType type, RubyAccessMode access_mode, NodeID cpu,
                bool sharing_miss);
    int getTotal() const;
    int getSharing() const { return m_sharing; }
    int getTouchedBy() const { return m_touched_by.count(); }
    Addr getAddress() const { return m_addr; }
    void addSample(int value);

    void print(std::ostream& out) const;

    static inline bool
    less_equal(const AccessTraceForAddress* n1,
        const AccessTraceForAddress* n2)
    {
        return n1->getTotal() <= n2->getTotal();
    }

  private:
    Addr m_addr;
    uint64_t m_loads;
    uint64_t m_stores;
    uint64_t m_atomics;
    uint64_t m_total;
    uint64_t m_user;
    uint64_t m_sharing;
    Set m_touched_by;
    Histogram* m_histogram_ptr;
};

inline std::ostream&
operator<<(std::ostream& out, const AccessTraceForAddress& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_PROFILER_ACCESSTRACEFORADDRESS_HH__
