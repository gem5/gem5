
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

#include "mem/ruby/recorder/TraceRecord.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/slicc_interface/AbstractChip.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/packet.hh"

TraceRecord::TraceRecord(NodeID id, const Address& data_addr, const Address& pc_addr, CacheRequestType type, Time time)
{
  m_node_num = id;
  m_data_address = data_addr;
  m_pc_address = pc_addr;
  m_time = time;
  m_type = type;

  // Don't differentiate between store misses and atomic requests in
  // the trace
  if (m_type == CacheRequestType_ATOMIC) {
    m_type = CacheRequestType_ST;
  }
}

// Public copy constructor and assignment operator
TraceRecord::TraceRecord(const TraceRecord& obj)
{
  *this = obj;  // Call assignment operator
}

TraceRecord& TraceRecord::operator=(const TraceRecord& obj)
{
  m_node_num = obj.m_node_num;
  m_time = obj.m_time;
  m_data_address = obj.m_data_address;
  m_pc_address = obj.m_pc_address;
  m_type = obj.m_type;
  return *this;
}

void TraceRecord::issueRequest() const
{
  // Lookup sequencer pointer from system
  // Note that the chip index also needs to take into account SMT configurations
  AbstractChip* chip_ptr = g_system_ptr->getChip(m_node_num/RubyConfig::numberOfProcsPerChip()/RubyConfig::numberofSMTThreads());
  assert(chip_ptr != NULL);
  Sequencer* sequencer_ptr = chip_ptr->getSequencer((m_node_num/RubyConfig::numberofSMTThreads())%RubyConfig::numberOfProcsPerChip());
  assert(sequencer_ptr != NULL);

  Addr data_addr = m_data_address.getAddress();
  Addr pc_addr = m_pc_address.getAddress();
  Request request(0, data_addr, 0, Flags<unsigned int>(Request::PREFETCH), pc_addr, m_node_num, 0);
  MemCmd::Command command;
  if (m_type == CacheRequestType_LD || m_type == CacheRequestType_IFETCH)
    command = MemCmd::ReadReq;
  else if (m_type == CacheRequestType_ST)
    command = MemCmd::WriteReq;
  else if (m_type == CacheRequestType_ATOMIC)
    command = MemCmd::SwapReq; // TODO -- differentiate between atomic types
  else
    assert(false);

  Packet pkt(&request, command, 0); // TODO -- make dest a real NodeID

  // Clear out the sequencer
  while (!sequencer_ptr->empty()) {
    g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 100);
  }

  sequencer_ptr->makeRequest(&pkt);

  // Clear out the sequencer
  while (!sequencer_ptr->empty()) {
    g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 100);
  }
}

void TraceRecord::print(ostream& out) const
{
  out << "[TraceRecord: Node, " << m_node_num << ", " << m_data_address << ", " << m_pc_address << ", " << m_type << ", Time: " << m_time << "]";
}

void TraceRecord::output(ostream& out) const
{
  out << m_node_num << " ";
  m_data_address.output(out);
  out << " ";
  m_pc_address.output(out);
  out << " ";
  out << m_type;
  out << endl;
}

bool TraceRecord::input(istream& in)
{
  in >> m_node_num;
  m_data_address.input(in);
  m_pc_address.input(in);
  string type;
  if (!in.eof()) {
    in >> type;
    m_type = string_to_CacheRequestType(type);

    // Ignore the rest of the line
    char c = '\0';
    while ((!in.eof()) && (c != '\n')) {
      in.get(c);
    }

    return true;
  } else {
    return false;
  }
}
