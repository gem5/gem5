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

#include <cassert>

#include "debug/RubyNetwork.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/TopologyType.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/BasicLink.hh"
#include "mem/ruby/network/BasicRouter.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/Topology.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"

using namespace std;

const int INFINITE_LATENCY = 10000; // Yes, this is a big hack

class BasicRouter;

// Note: In this file, we use the first 2*m_nodes SwitchIDs to
// represent the input and output endpoint links.  These really are
// not 'switches', as they will not have a Switch object allocated for
// them. The first m_nodes SwitchIDs are the links into the network,
// the second m_nodes set of SwitchIDs represent the the output queues
// of the network.

// Helper functions based on chapter 29 of Cormen et al.
void extend_shortest_path(Matrix& current_dist, Matrix& latencies,
    Matrix& inter_switches);
Matrix shortest_path(const Matrix& weights, Matrix& latencies,
    Matrix& inter_switches);
bool link_is_shortest_path_to_node(SwitchID src, SwitchID next,
    SwitchID final, const Matrix& weights, const Matrix& dist);
NetDest shortest_path_to_node(SwitchID src, SwitchID next,
    const Matrix& weights, const Matrix& dist);

Topology::Topology(const Params *p)
    : SimObject(p)
{
    m_print_config = p->print_config;
    m_number_of_switches = p->routers.size();

    // initialize component latencies record
    m_component_latencies.resize(0);
    m_component_inter_switches.resize(0);

    // Total nodes/controllers in network
    // Must make sure this is called after the State Machine constructors
    m_nodes = MachineType_base_number(MachineType_NUM);
    assert(m_nodes > 1);

    if (m_nodes != params()->ext_links.size() &&
        m_nodes != params()->ext_links.size()) {
        fatal("m_nodes (%d) != ext_links vector length (%d)\n",
              m_nodes != params()->ext_links.size());
    }

    // analyze both the internal and external links, create data structures
    // Note that the python created links are bi-directional, but that the
    // topology and networks utilize uni-directional links.  Thus each 
    // BasicLink is converted to two calls to add link, on for each direction
    for (vector<BasicExtLink*>::const_iterator i = params()->ext_links.begin();
         i != params()->ext_links.end(); ++i) {
        BasicExtLink *ext_link = (*i);
        AbstractController *abs_cntrl = ext_link->params()->ext_node;
        BasicRouter *router = ext_link->params()->int_node;

        // Store the controller and ExtLink pointers for later
        m_controller_vector.push_back(abs_cntrl);
        m_ext_link_vector.push_back(ext_link);

        int ext_idx1 = abs_cntrl->params()->cntrl_id;
        int ext_idx2 = ext_idx1 + m_nodes;
        int int_idx = router->params()->router_id + 2*m_nodes;

        // create the internal uni-directional links in both directions
        //   the first direction is marked: In
        addLink(ext_idx1, int_idx, ext_link, LinkDirection_In);
        //   the first direction is marked: Out
        addLink(int_idx, ext_idx2, ext_link, LinkDirection_Out);
    }

    for (vector<BasicIntLink*>::const_iterator i = params()->int_links.begin();
         i != params()->int_links.end(); ++i) {
        BasicIntLink *int_link = (*i);
        BasicRouter *router_a = int_link->params()->node_a;
        BasicRouter *router_b = int_link->params()->node_b;

        // Store the IntLink pointers for later
        m_int_link_vector.push_back(int_link);

        int a = router_a->params()->router_id + 2*m_nodes;
        int b = router_b->params()->router_id + 2*m_nodes;

        // create the internal uni-directional links in both directions
        //   the first direction is marked: In
        addLink(a, b, int_link, LinkDirection_In);
        //   the second direction is marked: Out
        addLink(b, a, int_link, LinkDirection_Out);
    }
}

void
Topology::init()
{
}


void
Topology::initNetworkPtr(Network* net_ptr)
{
    for (vector<BasicExtLink*>::const_iterator i = params()->ext_links.begin();
         i != params()->ext_links.end(); ++i) {
        BasicExtLink *ext_link = (*i);
        AbstractController *abs_cntrl = ext_link->params()->ext_node;
        abs_cntrl->initNetworkPtr(net_ptr);
    }
}

void
Topology::createLinks(Network *net, bool isReconfiguration)
{
    // Find maximum switchID
    SwitchID max_switch_id = 0;
    for (LinkMap::const_iterator i = m_link_map.begin();
         i != m_link_map.end(); ++i) {
        std::pair<int, int> src_dest = (*i).first;
        max_switch_id = max(max_switch_id, src_dest.first);
        max_switch_id = max(max_switch_id, src_dest.second);        
    }

    // Initialize weight, latency, and inter switched vectors
    Matrix topology_weights;
    int num_switches = max_switch_id+1;
    topology_weights.resize(num_switches);
    m_component_latencies.resize(num_switches);
    m_component_inter_switches.resize(num_switches);

    for (int i = 0; i < topology_weights.size(); i++) {
        topology_weights[i].resize(num_switches);
        m_component_latencies[i].resize(num_switches);
        m_component_inter_switches[i].resize(num_switches);

        for (int j = 0; j < topology_weights[i].size(); j++) {
            topology_weights[i][j] = INFINITE_LATENCY;

            // initialize to invalid values
            m_component_latencies[i][j] = -1;

            // initially assume direct connections / no intermediate
            // switches between components
            m_component_inter_switches[i][j] = 0;
        }
    }

    // Set identity weights to zero
    for (int i = 0; i < topology_weights.size(); i++) {
        topology_weights[i][i] = 0;
    }

    // Fill in the topology weights and bandwidth multipliers
    for (LinkMap::const_iterator i = m_link_map.begin();
         i != m_link_map.end(); ++i) {
        std::pair<int, int> src_dest = (*i).first;
        BasicLink* link = (*i).second.link;
        int src = src_dest.first;
        int dst = src_dest.second;
        m_component_latencies[src][dst] = link->m_latency;
        topology_weights[src][dst] = link->m_weight;
    }
        
    // Walk topology and hookup the links
    Matrix dist = shortest_path(topology_weights, m_component_latencies,
        m_component_inter_switches);
    for (int i = 0; i < topology_weights.size(); i++) {
        for (int j = 0; j < topology_weights[i].size(); j++) {
            int weight = topology_weights[i][j];
            if (weight > 0 && weight != INFINITE_LATENCY) {
                NetDest destination_set = shortest_path_to_node(i, j,
                                                     topology_weights, dist);
                makeLink(net, i, j, destination_set, isReconfiguration);
            }
        }
    }
}

void
Topology::addLink(SwitchID src, SwitchID dest, BasicLink* link, 
                  LinkDirection dir)
{
    assert(src <= m_number_of_switches+m_nodes+m_nodes);
    assert(dest <= m_number_of_switches+m_nodes+m_nodes);
    
    std::pair<int, int> src_dest_pair;
    LinkEntry link_entry;

    src_dest_pair.first = src;
    src_dest_pair.second = dest;
    link_entry.direction = dir;
    link_entry.link = link;
    m_link_map[src_dest_pair] = link_entry;
}

void
Topology::makeLink(Network *net, SwitchID src, SwitchID dest,
                   const NetDest& routing_table_entry, bool isReconfiguration)
{
    // Make sure we're not trying to connect two end-point nodes
    // directly together
    assert(src >= 2 * m_nodes || dest >= 2 * m_nodes);

    std::pair<int, int> src_dest;
    LinkEntry link_entry;    

    if (src < m_nodes) {
        src_dest.first = src;
        src_dest.second = dest;
        link_entry = m_link_map[src_dest];
        net->makeInLink(src, dest - (2 * m_nodes), link_entry.link,
                        link_entry.direction, 
                        routing_table_entry,
                        isReconfiguration);
    } else if (dest < 2*m_nodes) {
        assert(dest >= m_nodes);
        NodeID node = dest - m_nodes;
        src_dest.first = src;
        src_dest.second = dest;
        link_entry = m_link_map[src_dest];
        net->makeOutLink(src - (2 * m_nodes), node, link_entry.link,
                         link_entry.direction, 
                         routing_table_entry,
                         isReconfiguration);
    } else {
        assert((src >= 2 * m_nodes) && (dest >= 2 * m_nodes));
        src_dest.first = src;
        src_dest.second = dest;
        link_entry = m_link_map[src_dest];
        net->makeInternalLink(src - (2 * m_nodes), dest - (2 * m_nodes),
                              link_entry.link, link_entry.direction,
                              routing_table_entry, isReconfiguration);
    }
}

void
Topology::printStats(std::ostream& out) const
{
    for (int cntrl = 0; cntrl < m_controller_vector.size(); cntrl++) {
        m_controller_vector[cntrl]->printStats(out);
    }
}

void
Topology::clearStats()
{
    for (int cntrl = 0; cntrl < m_controller_vector.size(); cntrl++) {
        m_controller_vector[cntrl]->clearStats();
    }
}

void
Topology::printConfig(std::ostream& out) const
{
    if (m_print_config == false)
        return;

    assert(m_component_latencies.size() > 0);

    out << "--- Begin Topology Print ---" << endl
        << endl
        << "Topology print ONLY indicates the _NETWORK_ latency between two "
        << "machines" << endl
        << "It does NOT include the latency within the machines" << endl
        << endl;

    for (int m = 0; m < MachineType_NUM; m++) {
        int i_end = MachineType_base_count((MachineType)m);
        for (int i = 0; i < i_end; i++) {
            MachineID cur_mach = {(MachineType)m, i};
            out << cur_mach << " Network Latencies" << endl;
            for (int n = 0; n < MachineType_NUM; n++) {
                int j_end = MachineType_base_count((MachineType)n);
                for (int j = 0; j < j_end; j++) {
                    MachineID dest_mach = {(MachineType)n, j};
                    if (cur_mach == dest_mach)
                        continue;

                    int src = MachineType_base_number((MachineType)m) + i;
                    int dst = MachineType_base_number(MachineType_NUM) +
                        MachineType_base_number((MachineType)n) + j;
                    int link_latency = m_component_latencies[src][dst];
                    int intermediate_switches =
                        m_component_inter_switches[src][dst];

                    // NOTE switches are assumed to have single
                    // cycle latency
                    out << "  " << cur_mach << " -> " << dest_mach
                        << " net_lat: "
                        << link_latency + intermediate_switches << endl;
                }
            }
            out << endl;
        }
    }

    out << "--- End Topology Print ---" << endl;
}

// The following all-pairs shortest path algorithm is based on the
// discussion from Cormen et al., Chapter 26.1.
void
extend_shortest_path(Matrix& current_dist, Matrix& latencies,
    Matrix& inter_switches)
{
    bool change = true;
    int nodes = current_dist.size();

    while (change) {
        change = false;
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < nodes; j++) {
                int minimum = current_dist[i][j];
                int previous_minimum = minimum;
                int intermediate_switch = -1;
                for (int k = 0; k < nodes; k++) {
                    minimum = min(minimum,
                        current_dist[i][k] + current_dist[k][j]);
                    if (previous_minimum != minimum) {
                        intermediate_switch = k;
                        inter_switches[i][j] =
                            inter_switches[i][k] +
                            inter_switches[k][j] + 1;
                    }
                    previous_minimum = minimum;
                }
                if (current_dist[i][j] != minimum) {
                    change = true;
                    current_dist[i][j] = minimum;
                    assert(intermediate_switch >= 0);
                    assert(intermediate_switch < latencies[i].size());
                    latencies[i][j] = latencies[i][intermediate_switch] +
                        latencies[intermediate_switch][j];
                }
            }
        }
    }
}

Matrix
shortest_path(const Matrix& weights, Matrix& latencies, Matrix& inter_switches)
{
    Matrix dist = weights;
    extend_shortest_path(dist, latencies, inter_switches);
    return dist;
}

bool
link_is_shortest_path_to_node(SwitchID src, SwitchID next, SwitchID final,
    const Matrix& weights, const Matrix& dist)
{
    return weights[src][next] + dist[next][final] == dist[src][final];
}

NetDest
shortest_path_to_node(SwitchID src, SwitchID next, const Matrix& weights,
    const Matrix& dist)
{
    NetDest result;
    int d = 0;
    int machines;
    int max_machines;

    machines = MachineType_NUM;
    max_machines = MachineType_base_number(MachineType_NUM);

    for (int m = 0; m < machines; m++) {
        for (int i = 0; i < MachineType_base_count((MachineType)m); i++) {
            // we use "d+max_machines" below since the "destination"
            // switches for the machines are numbered
            // [MachineType_base_number(MachineType_NUM)...
            //  2*MachineType_base_number(MachineType_NUM)-1] for the
            // component network
            if (link_is_shortest_path_to_node(src, next, d + max_machines,
                    weights, dist)) {
                MachineID mach = {(MachineType)m, i};
                result.add(mach);
            }
            d++;
        }
    }

    DPRINTF(RubyNetwork, "Returning shortest path\n"
            "(src-(2*max_machines)): %d, (next-(2*max_machines)): %d, "
            "src: %d, next: %d, result: %s\n",
            (src-(2*max_machines)), (next-(2*max_machines)),
            src, next, result);

    return result;
}

Topology *
TopologyParams::create()
{
    return new Topology(this);
}

