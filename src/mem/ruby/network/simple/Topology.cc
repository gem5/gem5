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
#include "mem/protocol/Protocol.hh"
#include "mem/protocol/TopologyType.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/simple/Topology.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

const int INFINITE_LATENCY = 10000; // Yes, this is a big hack
const int DEFAULT_BW_MULTIPLIER = 1;  // Just to be consistent with above :)

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
    m_number_of_switches = p->num_int_nodes;
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

    // First create the links between the endpoints (i.e. controllers)
    // and the network.
    for (vector<ExtLink*>::const_iterator i = params()->ext_links.begin();
         i != params()->ext_links.end(); ++i) {
        const ExtLinkParams *p = (*i)->params();
        AbstractController *c = p->ext_node;

        // Store the controller pointers for later
        m_controller_vector.push_back(c);

        int ext_idx1 =
            MachineType_base_number(c->getMachineType()) + c->getVersion();
        int ext_idx2 = ext_idx1 + m_nodes;
        int int_idx = p->int_node + 2*m_nodes;

        // create the links in both directions
        addLink(ext_idx1, int_idx, p->latency, p->bw_multiplier, p->weight);
        addLink(int_idx, ext_idx2, p->latency, p->bw_multiplier, p->weight);
    }

    for (vector<IntLink*>::const_iterator i = params()->int_links.begin();
         i != params()->int_links.end(); ++i) {
        const IntLinkParams *p = (*i)->params();
        int a = p->node_a + 2*m_nodes;
        int b = p->node_b + 2*m_nodes;

        // create the links in both directions
        addLink(a, b, p->latency, p->bw_multiplier, p->weight);
        addLink(b, a, p->latency, p->bw_multiplier, p->weight);
    }
}


void
Topology::initNetworkPtr(Network* net_ptr)
{
    for (int cntrl = 0; cntrl < m_controller_vector.size(); cntrl++) {
        m_controller_vector[cntrl]->initNetworkPtr(net_ptr);
    }
}

void
Topology::createLinks(Network *net, bool isReconfiguration)
{
    // Find maximum switchID
    SwitchID max_switch_id = 0;
    for (int i = 0; i < m_links_src_vector.size(); i++) {
        max_switch_id = max(max_switch_id, m_links_src_vector[i]);
        max_switch_id = max(max_switch_id, m_links_dest_vector[i]);
    }

    // Initialize weight vector
    Matrix topology_weights;
    Matrix topology_latency;
    Matrix topology_bw_multis;
    int num_switches = max_switch_id+1;
    topology_weights.resize(num_switches);
    topology_latency.resize(num_switches);
    topology_bw_multis.resize(num_switches);

    // FIXME setting the size of a member variable here is a HACK!
    m_component_latencies.resize(num_switches);

    // FIXME setting the size of a member variable here is a HACK!
    m_component_inter_switches.resize(num_switches);

    for (int i = 0; i < topology_weights.size(); i++) {
        topology_weights[i].resize(num_switches);
        topology_latency[i].resize(num_switches);
        topology_bw_multis[i].resize(num_switches);
        m_component_latencies[i].resize(num_switches);

        // FIXME setting the size of a member variable here is a HACK!
        m_component_inter_switches[i].resize(num_switches);

        for (int j = 0; j < topology_weights[i].size(); j++) {
            topology_weights[i][j] = INFINITE_LATENCY;

            // initialize to invalid values
            topology_latency[i][j] = -1;
            topology_bw_multis[i][j] = -1;
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
    for (int i = 0; i < m_links_src_vector.size(); i++) {
        int src = m_links_src_vector[i];
        int dst = m_links_dest_vector[i];
        topology_weights[src][dst] = m_links_weight_vector[i];
        topology_latency[src][dst] = m_links_latency_vector[i];
        m_component_latencies[src][dst] = m_links_latency_vector[i];
        topology_bw_multis[src][dst] = m_bw_multiplier_vector[i];
    }

    // Walk topology and hookup the links
    Matrix dist = shortest_path(topology_weights, m_component_latencies,
        m_component_inter_switches);
    for (int i = 0; i < topology_weights.size(); i++) {
        for (int j = 0; j < topology_weights[i].size(); j++) {
            int weight = topology_weights[i][j];
            int bw_multiplier = topology_bw_multis[i][j];
            int latency = topology_latency[i][j];
            if (weight > 0 && weight != INFINITE_LATENCY) {
                NetDest destination_set = shortest_path_to_node(i, j,
                    topology_weights, dist);
                assert(latency != -1);
                makeLink(net, i, j, destination_set, latency, weight,
                    bw_multiplier, isReconfiguration);
            }
        }
    }
}

SwitchID
Topology::newSwitchID()
{
    m_number_of_switches++;
    return m_number_of_switches-1+m_nodes+m_nodes;
}

void
Topology::addLink(SwitchID src, SwitchID dest, int link_latency)
{
    addLink(src, dest, link_latency, DEFAULT_BW_MULTIPLIER, link_latency);
}

void
Topology::addLink(SwitchID src, SwitchID dest, int link_latency,
    int bw_multiplier)
{
    addLink(src, dest, link_latency, bw_multiplier, link_latency);
}

void
Topology::addLink(SwitchID src, SwitchID dest, int link_latency,
    int bw_multiplier, int link_weight)
{
    ASSERT(src <= m_number_of_switches+m_nodes+m_nodes);
    ASSERT(dest <= m_number_of_switches+m_nodes+m_nodes);
    m_links_src_vector.push_back(src);
    m_links_dest_vector.push_back(dest);
    m_links_latency_vector.push_back(link_latency);
    m_links_weight_vector.push_back(link_weight);
    m_bw_multiplier_vector.push_back(bw_multiplier);
}

void
Topology::makeLink(Network *net, SwitchID src, SwitchID dest,
    const NetDest& routing_table_entry, int link_latency, int link_weight,
    int bw_multiplier, bool isReconfiguration)
{
    // Make sure we're not trying to connect two end-point nodes
    // directly together
    assert(src >= 2 * m_nodes || dest >= 2 * m_nodes);

    if (src < m_nodes) {
        net->makeInLink(src, dest-(2*m_nodes), routing_table_entry,
            link_latency, bw_multiplier, isReconfiguration);
    } else if (dest < 2*m_nodes) {
        assert(dest >= m_nodes);
        NodeID node = dest-m_nodes;
        net->makeOutLink(src-(2*m_nodes), node, routing_table_entry,
            link_latency, link_weight, bw_multiplier, isReconfiguration);
    } else {
        assert((src >= 2*m_nodes) && (dest >= 2*m_nodes));
        net->makeInternalLink(src-(2*m_nodes), dest-(2*m_nodes),
            routing_table_entry, link_latency, link_weight, bw_multiplier,
            isReconfiguration);
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

Link *
LinkParams::create()
{
    return new Link(this);
}

ExtLink *
ExtLinkParams::create()
{
    return new ExtLink(this);
}

IntLink *
IntLinkParams::create()
{
    return new IntLink(this);
}
