
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
 */

#include "mem/ruby/recorder/CacheRecorder.hh"
#include "mem/ruby/recorder/TraceRecord.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/gems_common/PrioHeap.hh"
#include "gzstream.hh"

CacheRecorder::CacheRecorder()
{
  m_records_ptr = new PrioHeap<TraceRecord>;
}

CacheRecorder::~CacheRecorder()
{
  delete m_records_ptr;
}

void CacheRecorder::addRecord(const string & sequencer_name, const Address& data_addr, const Address& pc_addr, RubyRequestType type, Time time)
{
  m_records_ptr->insert(TraceRecord(sequencer_name, data_addr, pc_addr, type, time));
}

int CacheRecorder::dumpRecords(string filename)
{
  ogzstream out(filename.c_str());
  if (out.fail()) {
    cout << "Error: error opening file '" << filename << "'" << endl;
    return 0;
  }

  int counter = 0;
  while (m_records_ptr->size() != 0) {
    TraceRecord record = m_records_ptr->extractMin();
    record.output(out);
    counter++;
  }
  return counter;
}

void CacheRecorder::print(ostream& out) const
{
}
