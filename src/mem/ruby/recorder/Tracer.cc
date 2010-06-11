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

#include "base/cprintf.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/recorder/TraceRecord.hh"
#include "mem/ruby/recorder/Tracer.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

Tracer::Tracer(const Params *p)
    : SimObject(p)
{
    m_enabled = false;
    m_warmup_length = p->warmup_length;
    assert(m_warmup_length  > 0);
    RubySystem::m_tracer_ptr = this;
}

void
Tracer::startTrace(string filename)
{
    if (m_enabled)
        stopTrace();

    if (filename != "") {
        m_trace_file.open(filename.c_str());
        if (m_trace_file.fail()) {
            cprintf("Error: error opening file '%s'\n", filename);
            cprintf("Trace not enabled.\n");
            return;
        }
        cprintf("Request trace enabled to output file '%s'\n", filename);
        m_enabled = true;
    }
}

void
Tracer::stopTrace()
{
    if (m_enabled) {
        m_trace_file.close();
        cout << "Request trace file closed." << endl;
        m_enabled = false;
    }
}

void
Tracer::traceRequest(Sequencer* sequencer, const Address& data_addr,
    const Address& pc_addr, RubyRequestType type, Time time)
{
    assert(m_enabled);
    TraceRecord tr(sequencer, data_addr, pc_addr, type, time);
    tr.output(m_trace_file);
}

int
Tracer::playbackTrace(string filename)
{
    igzstream in(filename.c_str());
    if (in.fail()) {
        cprintf("Error: error opening file '%s'\n", filename);
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
        if (counter == m_warmup_length) {
            cprintf("Clearing stats after warmup of length %s\n",
                    m_warmup_length);
            g_system_ptr->clearStats();
        }
    }

    // Flush the prefetches through the system
    // FIXME - should be smarter
    g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 1000);

    time_t stop_time = time(NULL);
    double seconds = difftime(stop_time, start_time);
    double minutes = seconds / 60.0;
    cout << "playbackTrace: " << minutes << " minutes" << endl;

    return counter;
}

void
Tracer::print(ostream& out) const
{
}

Tracer *
RubyTracerParams::create()
{
    return new Tracer(this);
}
