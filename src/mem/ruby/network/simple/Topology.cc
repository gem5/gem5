
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
 * Topology.cc
 *
 * Description: See Topology.hh
 *
 * $Id$
 *
 * */

#include "mem/ruby/network/simple/Topology.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/protocol/TopologyType.hh"
#include "mem/gems_common/util.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/ruby/system/System.hh"
#include <string>

static const int INFINITE_LATENCY = 10000; // Yes, this is a big hack
static const int DEFAULT_BW_MULTIPLIER = 1;  // Just to be consistent with above :)

// Note: In this file, we use the first 2*m_nodes SwitchIDs to
// represent the input and output endpoint links.  These really are
// not 'switches', as they will not have a Switch object allocated for
// them. The first m_nodes SwitchIDs are the links into the network,
// the second m_nodes set of SwitchIDs represent the the output queues
// of the network.

// Helper functions based on chapter 29 of Cormen et al.
static void extend_shortest_path(Matrix& current_dist, Matrix& latencies, Matrix& inter_switches);
static Matrix shortest_path(const Matrix& weights, Matrix& latencies, Matrix& inter_switches);
static bool link_is_shortest_path_to_node(SwitchID src, SwitchID next, SwitchID final, const Matrix& weights, const Matrix& dist);
static NetDest shortest_path_to_node(SwitchID src, SwitchID next, const Matrix& weights, const Matrix& dist);

Topology::Topology(const string & name)
  : m_name(name)
{
  m_network_ptr = NULL;
  m_nodes = MachineType_base_number(MachineType_NUM);
  m_number_of_switches = 0;
}

void Topology::init(const vector<string> & argv)
{
  for (size_t i=0; i<argv.size(); i+=2) {
    if (argv[i] == "network")
      m_network_ptr = RubySystem::getNetwork();
    else if (argv[i] == "connections")
      m_connections = argv[i+1];
    else if (argv[i] == "print_config") {
      m_print_config = string_to_bool(argv[i+1]);
    }
  }
  assert(m_network_ptr != NULL);
}

void Topology::makeTopology()
{
  /*
  if (m_nodes == 1) {
    SwitchID id = newSwitchID();
    addLink(0, id, m_network_ptr->getOffChipLinkLatency());
    addLink(id, 1, m_network_ptr->getOffChipLinkLatency());
    return;
  }
  */
  assert(m_nodes > 1);

  Vector< Vector < SwitchID > > nodePairs;  // node pairs extracted from the file
  Vector<int> latencies;  // link latencies for each link extracted
  Vector<int> bw_multis;  // bw multipliers for each link extracted
  Vector<int> weights;  // link weights used to enfore e-cube deadlock free routing
  Vector< SwitchID > int_network_switches;  // internal switches extracted from the file
  Vector<bool> endpointConnectionExist;  // used to ensure all endpoints are connected to the network

  endpointConnectionExist.setSize(m_nodes);

  // initialize endpoint check vector
  for (int k = 0; k < endpointConnectionExist.size(); k++) {
    endpointConnectionExist[k] = false;
  }

  stringstream networkFile( m_connections );

  string line = "";

  while (!networkFile.eof()) {

    Vector < SwitchID > nodes;
    nodes.setSize(2);
    int latency = -1;  // null latency
    int weight = -1;  // null weight
    int bw_multiplier = DEFAULT_BW_MULTIPLIER;  // default multiplier incase the network file doesn't define it
    int i = 0;  // node pair index
    int varsFound = 0;  // number of varsFound on the line
    int internalNodes = 0;  // used to determine if the link is between 2 internal nodes
    std::getline(networkFile, line, '\n');
    string varStr = string_split(line, ' ');

    // parse the current line in the file
    while (varStr != "") {
      string label = string_split(varStr, ':');

      // valid node labels
      if (label == "ext_node" || label == "int_node") {
        ASSERT(i < 2); // one link between 2 switches per line
        varsFound++;
        bool isNewIntSwitch = true;
        if (label == "ext_node") { // input link to node
          MachineType machine = string_to_MachineType(string_split(varStr, ':'));
          string nodeStr = string_split(varStr, ':');
          nodes[i] = MachineType_base_number(machine)
            + atoi(nodeStr.c_str());

          // in nodes should be numbered 0 to m_nodes-1
          ASSERT(nodes[i] >= 0 && nodes[i] < m_nodes);
          isNewIntSwitch = false;
          endpointConnectionExist[nodes[i]] = true;
        }
        if (label == "int_node") { // interior node
          nodes[i] = atoi((string_split(varStr, ':')).c_str())+m_nodes*2;
          // in nodes should be numbered >= m_nodes*2
          ASSERT(nodes[i] >= m_nodes*2);
          for (int k = 0; k < int_network_switches.size(); k++) {
            if (int_network_switches[k] == nodes[i]) {
              isNewIntSwitch = false;
            }
          }
          if (isNewIntSwitch) {  // if internal switch
            m_number_of_switches++;
            int_network_switches.insertAtBottom(nodes[i]);
          }
          internalNodes++;
        }
        i++;
      } else if (label == "link_latency") {
        latency = atoi((string_split(varStr, ':')).c_str());
        varsFound++;
      } else if (label == "bw_multiplier") {  // not necessary, defaults to DEFAULT_BW_MULTIPLIER
        bw_multiplier = atoi((string_split(varStr, ':')).c_str());
      } else if (label == "link_weight") {  // not necessary, defaults to link_latency
        weight = atoi((string_split(varStr, ':')).c_str());
      } else {
        cerr << "Error: Unexpected Identifier: " << label << endl;
        exit(1);
      }
      varStr = string_split(line, ' ');
    }
    if (varsFound == 3) { // all three necessary link variables where found so add the link
      nodePairs.insertAtBottom(nodes);
      latencies.insertAtBottom(latency);
      if (weight != -1) {
        weights.insertAtBottom(weight);
      } else {
        weights.insertAtBottom(latency);
      }
      bw_multis.insertAtBottom(bw_multiplier);
      Vector < SwitchID > otherDirectionNodes;
      otherDirectionNodes.setSize(2);
      otherDirectionNodes[0] = nodes[1];
      if (internalNodes == 2) {  // this is an internal link
        otherDirectionNodes[1] = nodes[0];
      } else {
        otherDirectionNodes[1] = nodes[0]+m_nodes;
      }
      nodePairs.insertAtBottom(otherDirectionNodes);
      latencies.insertAtBottom(latency);
      if (weight != -1) {
        weights.insertAtBottom(weight);
      } else {
        weights.insertAtBottom(latency);
      }
      bw_multis.insertAtBottom(bw_multiplier);
    } else {
      if (varsFound != 0) {  // if this is not a valid link, then no vars should have been found
        cerr << "Error in line: " << line << endl;
        exit(1);
      }
    }
  } // end of file

  // makes sure all enpoints are connected in the soon to be created network
  for (int k = 0; k < endpointConnectionExist.size(); k++) {
    if (endpointConnectionExist[k] == false) {
      cerr << "Error: Unconnected Endpoint: " << k << endl;
      exit(1);
    }
  }

  ASSERT(nodePairs.size() == latencies.size() && latencies.size() == bw_multis.size() && latencies.size() == weights.size())
  for (int k = 0; k < nodePairs.size(); k++) {
    ASSERT(nodePairs[k].size() == 2);
    addLink(nodePairs[k][0], nodePairs[k][1], latencies[k], bw_multis[k], weights[k]);
  }

  // initialize component latencies record
  m_component_latencies.setSize(0);
  m_component_inter_switches.setSize(0);
}


void Topology::createLinks(bool isReconfiguration)
{
  // Find maximum switchID

  SwitchID max_switch_id = 0;
  for (int i=0; i<m_links_src_vector.size(); i++) {
    max_switch_id = max(max_switch_id, m_links_src_vector[i]);
    max_switch_id = max(max_switch_id, m_links_dest_vector[i]);
  }

  // Initialize weight vector
  Matrix topology_weights;
  Matrix topology_latency;
  Matrix topology_bw_multis;
  int num_switches = max_switch_id+1;
  topology_weights.setSize(num_switches);
  topology_latency.setSize(num_switches);
  topology_bw_multis.setSize(num_switches);
  m_component_latencies.setSize(num_switches);  // FIXME setting the size of a member variable here is a HACK!
  m_component_inter_switches.setSize(num_switches);  // FIXME setting the size of a member variable here is a HACK!
  for(int i=0; i<topology_weights.size(); i++) {
    topology_weights[i].setSize(num_switches);
    topology_latency[i].setSize(num_switches);
    topology_bw_multis[i].setSize(num_switches);
    m_component_latencies[i].setSize(num_switches);
    m_component_inter_switches[i].setSize(num_switches);  // FIXME setting the size of a member variable here is a HACK!
    for(int j=0; j<topology_weights[i].size(); j++) {
      topology_weights[i][j] = INFINITE_LATENCY;
      topology_latency[i][j] = -1;  // initialize to an invalid value
      topology_bw_multis[i][j] = -1;  // initialize to an invalid value
      m_component_latencies[i][j] = -1; // initialize to an invalid value
      m_component_inter_switches[i][j] = 0;  // initially assume direct connections / no intermediate switches between components
    }
  }

  // Set identity weights to zero
  for(int i=0; i<topology_weights.size(); i++) {
    topology_weights[i][i] = 0;
  }

  // Fill in the topology weights and bandwidth multipliers
  for (int i=0; i<m_links_src_vector.size(); i++) {
    topology_weights[m_links_src_vector[i]][m_links_dest_vector[i]] = m_links_weight_vector[i];
    topology_latency[m_links_src_vector[i]][m_links_dest_vector[i]] = m_links_latency_vector[i];
    m_component_latencies[m_links_src_vector[i]][m_links_dest_vector[i]] = m_links_latency_vector[i];  // initialize to latency vector
    topology_bw_multis[m_links_src_vector[i]][m_links_dest_vector[i]] = m_bw_multiplier_vector[i];
  }

  // Walk topology and hookup the links
  Matrix dist = shortest_path(topology_weights, m_component_latencies, m_component_inter_switches);
  for(int i=0; i<topology_weights.size(); i++) {
    for(int j=0; j<topology_weights[i].size(); j++) {
      int weight = topology_weights[i][j];
      int bw_multiplier = topology_bw_multis[i][j];
      int latency = topology_latency[i][j];
      if (weight > 0 && weight != INFINITE_LATENCY) {
        NetDest destination_set = shortest_path_to_node(i, j, topology_weights, dist);
        assert(latency != -1);
        makeLink(i, j, destination_set, latency, weight, bw_multiplier, isReconfiguration);
      }
    }
  }
}

SwitchID Topology::newSwitchID()
{
  m_number_of_switches++;
  return m_number_of_switches-1+m_nodes+m_nodes;
}

void Topology::addLink(SwitchID src, SwitchID dest, int link_latency)
{
  addLink(src, dest, link_latency, DEFAULT_BW_MULTIPLIER, link_latency);
}

void Topology::addLink(SwitchID src, SwitchID dest, int link_latency, int bw_multiplier)
{
  addLink(src, dest, link_latency, bw_multiplier, link_latency);
}

void Topology::addLink(SwitchID src, SwitchID dest, int link_latency, int bw_multiplier, int link_weight)
{
  ASSERT(src <= m_number_of_switches+m_nodes+m_nodes);
  ASSERT(dest <= m_number_of_switches+m_nodes+m_nodes);
  m_links_src_vector.insertAtBottom(src);
  m_links_dest_vector.insertAtBottom(dest);
  m_links_latency_vector.insertAtBottom(link_latency);
  m_links_weight_vector.insertAtBottom(link_weight);
  m_bw_multiplier_vector.insertAtBottom(bw_multiplier);
}

void Topology::makeLink(SwitchID src, SwitchID dest, const NetDest& routing_table_entry, int link_latency, int link_weight, int bw_multiplier, bool isReconfiguration)
{
  // Make sure we're not trying to connect two end-point nodes directly together
  assert((src >= 2*m_nodes) || (dest >= 2*m_nodes));

  if (src < m_nodes) {
    m_network_ptr->makeInLink(src, dest-(2*m_nodes), routing_table_entry, link_latency, bw_multiplier, isReconfiguration);
  } else if (dest < 2*m_nodes) {
    assert(dest >= m_nodes);
    NodeID node = dest-m_nodes;
    m_network_ptr->makeOutLink(src-(2*m_nodes), node, routing_table_entry, link_latency, link_weight, bw_multiplier, isReconfiguration);
  } else {
    assert((src >= 2*m_nodes) && (dest >= 2*m_nodes));
    m_network_ptr->makeInternalLink(src-(2*m_nodes), dest-(2*m_nodes), routing_table_entry, link_latency, link_weight, bw_multiplier, isReconfiguration);
  }
}

void Topology::printConfig(ostream& out) const
{
  if (m_print_config == false) return;

  assert(m_component_latencies.size() > 0);

  out << "--- Begin Topology Print ---" << endl;
  out << endl;
  out << "Topology print ONLY indicates the _NETWORK_ latency between two machines" << endl;
  out << "It does NOT include the latency within the machines" << endl;
  out << endl;
  for (int m=0; m<MachineType_NUM; m++) {
    for (int i=0; i<MachineType_base_count((MachineType)m); i++) {
      MachineID cur_mach = {(MachineType)m, i};
      out << cur_mach << " Network Latencies" << endl;
      for (int n=0; n<MachineType_NUM; n++) {
        for (int j=0; j<MachineType_base_count((MachineType)n); j++) {
          MachineID dest_mach = {(MachineType)n, j};
          if (cur_mach != dest_mach) {
            int link_latency = m_component_latencies[MachineType_base_number((MachineType)m)+i][MachineType_base_number(MachineType_NUM)+MachineType_base_number((MachineType)n)+j];
            int intermediate_switches = m_component_inter_switches[MachineType_base_number((MachineType)m)+i][MachineType_base_number(MachineType_NUM)+MachineType_base_number((MachineType)n)+j];
            out << "  " << cur_mach << " -> " << dest_mach << " net_lat: "
                << link_latency+intermediate_switches << endl;  // NOTE switches are assumed to have single cycle latency
          }
        }
      }
      out << endl;
    }
  }

  out << "--- End Topology Print ---" << endl;
}

/**************************************************************************/

// The following all-pairs shortest path algorithm is based on the
// discussion from Cormen et al., Chapter 26.1.

static void extend_shortest_path(Matrix& current_dist, Matrix& latencies, Matrix& inter_switches)
{
  bool change = true;
  int nodes = current_dist.size();

  while (change) {
    change = false;
    for (int i=0; i<nodes; i++) {
      for (int j=0; j<nodes; j++) {
        int minimum = current_dist[i][j];
        int previous_minimum = minimum;
        int intermediate_switch = -1;
        for (int k=0; k<nodes; k++) {
          minimum = min(minimum, current_dist[i][k] + current_dist[k][j]);
          if (previous_minimum != minimum) {
            intermediate_switch = k;
            inter_switches[i][j] = inter_switches[i][k] + inter_switches[k][j] + 1;
          }
          previous_minimum = minimum;
        }
        if (current_dist[i][j] != minimum) {
          change = true;
          current_dist[i][j] = minimum;
          assert(intermediate_switch >= 0);
          assert(intermediate_switch < latencies[i].size());
          latencies[i][j] = latencies[i][intermediate_switch] + latencies[intermediate_switch][j];
        }
      }
    }
  }
}

static Matrix shortest_path(const Matrix& weights, Matrix& latencies, Matrix& inter_switches)
{
  Matrix dist = weights;
  extend_shortest_path(dist, latencies, inter_switches);
  return dist;
}

static bool link_is_shortest_path_to_node(SwitchID src, SwitchID next, SwitchID final,
                                          const Matrix& weights, const Matrix& dist)
{
  return (weights[src][next] + dist[next][final] == dist[src][final]);
}

static NetDest shortest_path_to_node(SwitchID src, SwitchID next,
                                     const Matrix& weights, const Matrix& dist)
{
  NetDest result;
  int d = 0;
  int machines;
  int max_machines;

  machines = MachineType_NUM;
  max_machines = MachineType_base_number(MachineType_NUM);

  for (int m=0; m<machines; m++) {
    for (int i=0; i<MachineType_base_count((MachineType)m); i++) {
      // we use "d+max_machines" below since the "destination" switches for the machines are numbered
      // [MachineType_base_number(MachineType_NUM)...2*MachineType_base_number(MachineType_NUM)-1]
      // for the component network
      if (link_is_shortest_path_to_node(src, next,
                                        d+max_machines,
                                        weights, dist)) {
        MachineID mach = {(MachineType)m, i};
        result.add(mach);
      }
      d++;
    }
  }

  DEBUG_MSG(NETWORK_COMP, MedPrio, "returning shortest path");
  DEBUG_EXPR(NETWORK_COMP, MedPrio, (src-(2*max_machines)));
  DEBUG_EXPR(NETWORK_COMP, MedPrio, (next-(2*max_machines)));
  DEBUG_EXPR(NETWORK_COMP, MedPrio, src);
  DEBUG_EXPR(NETWORK_COMP, MedPrio, next);
  DEBUG_EXPR(NETWORK_COMP, MedPrio, result);
  DEBUG_NEWLINE(NETWORK_COMP, MedPrio);

  return result;
}

