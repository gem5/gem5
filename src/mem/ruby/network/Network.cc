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

#include "base/misc.hh"
#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/system/RubySystem.hh"

uint32_t Network::m_virtual_networks;
uint32_t Network::m_control_msg_size;
uint32_t Network::m_data_msg_size;

Network::Network(const Params *p)
    : ClockedObject(p)
{
    m_virtual_networks = p->number_of_virtual_networks;
    m_control_msg_size = p->control_msg_size;

    // Total nodes/controllers in network
    // Must make sure this is called after the State Machine constructors
    m_nodes = MachineType_base_number(MachineType_NUM);
    assert(m_nodes != 0);
    assert(m_virtual_networks != 0);

    m_topology_ptr = new Topology(p->routers.size(), p->ext_links,
                                  p->int_links);

    // Allocate to and from queues
    // Queues that are getting messages from protocol
    m_toNetQueues.resize(m_nodes);

    // Queues that are feeding the protocol
    m_fromNetQueues.resize(m_nodes);

    m_ordered.resize(m_virtual_networks);
    m_vnet_type_names.resize(m_virtual_networks);

    for (int i = 0; i < m_virtual_networks; i++) {
        m_ordered[i] = false;
    }

    params()->ruby_system->registerNetwork(this);

    // Initialize the controller's network pointers
    for (std::vector<BasicExtLink*>::const_iterator i = p->ext_links.begin();
         i != p->ext_links.end(); ++i) {
        BasicExtLink *ext_link = (*i);
        AbstractController *abs_cntrl = ext_link->params()->ext_node;
        abs_cntrl->initNetworkPtr(this);
    }

    // Register a callback function for combining the statistics
    Stats::registerDumpCallback(new StatsCallback(this));

    for (auto &it : dynamic_cast<Network *>(this)->params()->ext_links) {
        it->params()->ext_node->initNetQueues();
    }
}

Network::~Network()
{
    for (int node = 0; node < m_nodes; node++) {

        // Delete the Message Buffers
        for (auto& it : m_toNetQueues[node]) {
            delete it;
        }

        for (auto& it : m_fromNetQueues[node]) {
            delete it;
        }
    }

    delete m_topology_ptr;
}

void
Network::init()
{
    m_data_msg_size = RubySystem::getBlockSizeBytes() + m_control_msg_size;
}

uint32_t
Network::MessageSizeType_to_int(MessageSizeType size_type)
{
    switch(size_type) {
      case MessageSizeType_Control:
      case MessageSizeType_Request_Control:
      case MessageSizeType_Reissue_Control:
      case MessageSizeType_Response_Control:
      case MessageSizeType_Writeback_Control:
      case MessageSizeType_Broadcast_Control:
      case MessageSizeType_Multicast_Control:
      case MessageSizeType_Forwarded_Control:
      case MessageSizeType_Invalidate_Control:
      case MessageSizeType_Unblock_Control:
      case MessageSizeType_Persistent_Control:
      case MessageSizeType_Completion_Control:
        return m_control_msg_size;
      case MessageSizeType_Data:
      case MessageSizeType_Response_Data:
      case MessageSizeType_ResponseLocal_Data:
      case MessageSizeType_ResponseL2hit_Data:
      case MessageSizeType_Writeback_Data:
        return m_data_msg_size;
      default:
        panic("Invalid range for type MessageSizeType");
        break;
    }
}

void
Network::checkNetworkAllocation(NodeID id, bool ordered,
                                        int network_num,
                                        std::string vnet_type)
{
    fatal_if(id >= m_nodes, "Node ID is out of range");
    fatal_if(network_num >= m_virtual_networks, "Network id is out of range");

    if (ordered) {
        m_ordered[network_num] = true;
    }

    m_vnet_type_names[network_num] = vnet_type;
}


void
Network::setToNetQueue(NodeID id, bool ordered, int network_num,
                                 std::string vnet_type, MessageBuffer *b)
{
    checkNetworkAllocation(id, ordered, network_num, vnet_type);
    while (m_toNetQueues[id].size() <= network_num) {
        m_toNetQueues[id].push_back(nullptr);
    }
    m_toNetQueues[id][network_num] = b;
}

void
Network::setFromNetQueue(NodeID id, bool ordered, int network_num,
                                   std::string vnet_type, MessageBuffer *b)
{
    checkNetworkAllocation(id, ordered, network_num, vnet_type);
    while (m_fromNetQueues[id].size() <= network_num) {
        m_fromNetQueues[id].push_back(nullptr);
    }
    m_fromNetQueues[id][network_num] = b;
}
