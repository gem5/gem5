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
 * A entry in the cache request record. It is aware of the ruby time
 * and can issue the request back to the cache.
 */

#ifndef __MEM_RUBY_RECORDER_TRACERECORD_HH__
#define __MEM_RUBY_RECORDER_TRACERECORD_HH__

#include <iostream>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/libruby_internal.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/system/Sequencer.hh"

class CacheMsg;

class TraceRecord
{
  public:
    TraceRecord(Sequencer* _sequencer, const Address& data_addr,
        const Address& pc_addr, RubyRequestType type, Time time);

    TraceRecord()
    {
        m_sequencer_ptr = NULL;
        m_time = 0;
        m_type = RubyRequestType_NULL;
    }

    TraceRecord(const TraceRecord& obj);
    TraceRecord& operator=(const TraceRecord& obj);

    void issueRequest() const;

    void print(std::ostream& out) const;
    void output(std::ostream& out) const;
    bool input(std::istream& in);

  private:
    friend bool operator>(const TraceRecord& n1, const TraceRecord& n2);

    Sequencer* m_sequencer_ptr;
    Time m_time;
    Address m_data_address;
    Address m_pc_address;
    RubyRequestType m_type;
};

inline bool
operator>(const TraceRecord& n1, const TraceRecord& n2)
{
    return n1.m_time > n2.m_time;
}

inline std::ostream&
operator<<(std::ostream& out, const TraceRecord& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_RECORDER_TRACERECORD_HH__
