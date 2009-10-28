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

#include "mem/protocol/MachineType.hh"
#include "mem/ruby/network/Network.hh"

Network::Network(const string & name)
  :  m_name(name)
{
  m_virtual_networks = 0;
  m_topology_ptr = NULL;
}

void Network::init(const vector<string> & argv)
{
  m_nodes = MachineType_base_number(MachineType_NUM); // Total nodes in network

  for (size_t i=0; i<argv.size(); i+=2) {
   if (argv[i] == "number_of_virtual_networks")
     m_virtual_networks = atoi(argv[i+1].c_str());
   else if (argv[i] == "topology")
     m_topology_ptr = RubySystem::getTopology(argv[i+1]);
   else if (argv[i] == "buffer_size")
     m_buffer_size = atoi(argv[i+1].c_str());
   else if (argv[i] == "endpoint_bandwidth")
     m_endpoint_bandwidth = atoi(argv[i+1].c_str());
   else if (argv[i] == "adaptive_routing")
     m_adaptive_routing = (argv[i+1]=="true");
   else if (argv[i] == "link_latency")
     m_link_latency = atoi(argv[i+1].c_str());
   else if (argv[i] == "control_msg_size")
     m_control_msg_size = atoi(argv[i+1].c_str());		 
  }

  m_data_msg_size = RubySystem::getBlockSizeBytes() + m_control_msg_size;

  assert(m_virtual_networks != 0);
  assert(m_topology_ptr != NULL);
}

int Network::MessageSizeType_to_int(MessageSizeType size_type)
{
  switch(size_type) {
  case MessageSizeType_Undefined:
    ERROR_MSG("Can't convert Undefined MessageSizeType to integer");
    break;
  case MessageSizeType_Control:
  case MessageSizeType_Request_Control:
  case MessageSizeType_Reissue_Control:
  case MessageSizeType_Response_Control:
  case MessageSizeType_Writeback_Control:
  case MessageSizeType_Forwarded_Control:
  case MessageSizeType_Invalidate_Control:
  case MessageSizeType_Unblock_Control:
  case MessageSizeType_Persistent_Control:
  case MessageSizeType_Completion_Control:
    return m_control_msg_size;
    break;
  case MessageSizeType_Data:
  case MessageSizeType_Response_Data:
  case MessageSizeType_ResponseLocal_Data:
  case MessageSizeType_ResponseL2hit_Data:
  case MessageSizeType_Writeback_Data:
    return m_data_msg_size;
    break;
  default:
    ERROR_MSG("Invalid range for type MessageSizeType");
    break;
  }
  return 0;
}
