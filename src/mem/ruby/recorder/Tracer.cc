
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

#include "mem/ruby/recorder/Tracer.hh"
#include "mem/ruby/recorder/TraceRecord.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/gems_common/PrioHeap.hh"
#include "mem/ruby/system/System.hh"

Tracer::Tracer()
{
  m_enabled = false;
}

Tracer::~Tracer()
{
}

void Tracer::startTrace(string filename)
{
  if (m_enabled) {
    stopTrace();
  }

  if (filename != "") {
    m_trace_file.open(filename.c_str());
    if (m_trace_file.fail()) {
      cout << "Error: error opening file '" << filename << "'" << endl;
      cout << "Trace not enabled." << endl;
      return;
    }
    cout << "Request trace enabled to output file '" << filename << "'" << endl;
    m_enabled = true;
  }
}

void Tracer::stopTrace()
{
  assert(m_enabled == true);
  m_trace_file.close();
  cout << "Request trace file closed." << endl;
  m_enabled = false;
}

void Tracer::traceRequest(NodeID id, const Address& data_addr, const Address& pc_addr, CacheRequestType type, Time time)
{
  assert(m_enabled == true);
  TraceRecord tr(id, data_addr, pc_addr, type, time);
  tr.output(m_trace_file);
}

// Class method
int Tracer::playbackTrace(string filename)
{
  igzstream in(filename.c_str());
  if (in.fail()) {
    cout << "Error: error opening file '" << filename << "'" << endl;
    return 0;
  }

  time_t start_time = time(NULL);

  TraceRecord record;
  int counter = 0;
  // Read in the next TraceRecord
  bool ok = record.input(in);
  while (ok) {
    // Put it in the right cache
    record.issueRequest();
    counter++;

    // Read in the next TraceRecord
    ok = record.input(in);

    // Clear the statistics after warmup
    if (counter == g_trace_warmup_length) {
      cout << "Clearing stats after warmup of length " << g_trace_warmup_length << endl;
      g_system_ptr->clearStats();
    }
  }

  // Flush the prefetches through the system
  g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 1000);  // FIXME - should be smarter

  time_t stop_time = time(NULL);
  double seconds = difftime(stop_time, start_time);
  double minutes = seconds / 60.0;
  cout << "playbackTrace: " << minutes << " minutes" << endl;

  return counter;
}

void Tracer::print(ostream& out) const
{
}
