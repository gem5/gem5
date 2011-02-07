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

#include "mem/protocol/CacheMsg.hh"
#include "mem/ruby/recorder/TraceRecord.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "sim/sim_object.hh"

using namespace std;

TraceRecord::TraceRecord(Sequencer* _sequencer, const Address& data_addr,
    const Address& pc_addr, RubyRequestType type, Time time)
{
    m_sequencer_ptr = _sequencer;
    m_data_address = data_addr;
    m_pc_address = pc_addr;
    m_time = time;
    m_type = type;

    // Don't differentiate between store misses and atomic requests in
    // the trace
    if (m_type == RubyRequestType_Load_Linked) {
        m_type = RubyRequestType_ST;
    } else if (m_type == RubyRequestType_Store_Conditional) {
        m_type = RubyRequestType_ST;
    }
}

TraceRecord::TraceRecord(const TraceRecord& obj)
{
    // Call assignment operator
    *this = obj;
}

TraceRecord&
TraceRecord::operator=(const TraceRecord& obj)
{
    m_sequencer_ptr = obj.m_sequencer_ptr;
    m_time = obj.m_time;
    m_data_address = obj.m_data_address;
    m_pc_address = obj.m_pc_address;
    m_type = obj.m_type;
    return *this;
}

void
TraceRecord::issueRequest() const
{
    assert(m_sequencer_ptr != NULL);

    RubyRequest request(m_data_address.getAddress(), NULL,
        RubySystem::getBlockSizeBytes(), m_pc_address.getAddress(),
        m_type, RubyAccessMode_User, NULL);

    // Clear out the sequencer
    while (!m_sequencer_ptr->empty()) {
        g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 100);
    }

    m_sequencer_ptr->makeRequest(request);

    // Clear out the sequencer
    while (!m_sequencer_ptr->empty()) {
        g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 100);
    }
}

void
TraceRecord::print(ostream& out) const
{
    out << "[TraceRecord: Node, " << m_sequencer_ptr->name() << ", "
        << m_data_address << ", " << m_pc_address << ", "
        << m_type << ", Time: " << m_time << "]";
}

void
TraceRecord::output(ostream& out) const
{
    out << m_sequencer_ptr->name() << " ";
    m_data_address.output(out);
    out << " ";
    m_pc_address.output(out);
    out << " ";
    out << m_type;
    out << endl;
}

bool
TraceRecord::input(istream& in)
{
    string sequencer_name;
    in >> sequencer_name;

    // The SimObject find function is slow and iterates through the
    // simObjectList to find the sequencer pointer.  Therefore, expect
    // trace playback to be slow.
    m_sequencer_ptr = (Sequencer*)SimObject::find(sequencer_name.c_str());

    m_data_address.input(in);
    m_pc_address.input(in);
    if (in.eof())
        return false;

    string type;
    in >> type;
    m_type = string_to_RubyRequestType(type);

    // Ignore the rest of the line
    char c = '\0';
    while ((!in.eof()) && (c != '\n')) {
        in.get(c);
    }

    return true;
}
