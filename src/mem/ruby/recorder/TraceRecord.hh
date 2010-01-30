
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
 * $Id$
 *
 * Description: A entry in the cache request record. It is aware of
 *              the ruby time and can issue the request back to the
 *              cache.
 *
 */

#ifndef TRACERECORD_H
#define TRACERECORD_H

#include "mem/ruby/libruby_internal.hh"

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/system/NodeID.hh"
class CacheMsg;

class TraceRecord {
public:
  // Constructors
  TraceRecord(Sequencer* _sequencer, 
              const Address& data_addr, 
              const Address& pc_addr, 
              RubyRequestType type, 
              Time time);

  TraceRecord() { 
    m_sequencer_ptr = NULL; 
    m_time = 0; 
    m_type = RubyRequestType_NULL; 
  }

  // Destructor
  //  ~TraceRecord();

  // Public copy constructor and assignment operator
  TraceRecord(const TraceRecord& obj);
  TraceRecord& operator=(const TraceRecord& obj);

  // Public Methods
  bool node_less_then_eq(const TraceRecord& rec) const { return (this->m_time <= rec.m_time); }
  void issueRequest() const;

  void print(ostream& out) const;
  void output(ostream& out) const;
  bool input(istream& in);
private:
  // Private Methods

  // Data Members (m_ prefix)
  Sequencer* m_sequencer_ptr;
  Time m_time;
  Address m_data_address;
  Address m_pc_address;
  RubyRequestType m_type;
};

inline extern bool node_less_then_eq(const TraceRecord& n1, const TraceRecord& n2);

// Output operator declaration
ostream& operator<<(ostream& out, const TraceRecord& obj);

// ******************* Definitions *******************

inline extern
bool node_less_then_eq(const TraceRecord& n1, const TraceRecord& n2)
{
  return n1.node_less_then_eq(n2);
}

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const TraceRecord& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TRACERECORD_H
