
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
 * Topology.C
 *
 * Description: See Topology.h
 *
 * $Id$
 *
 * */

#include "Topology.hh"
#include "NetDest.hh"
#include "Network.hh"
#include "TopologyType.hh"
#include "RubyConfig.hh"
#include "util.hh"
#include "MachineType.hh"
#include "Protocol.hh"
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
static Matrix extend_shortest_path(const Matrix& current_dist, Matrix& latencies, Matrix& inter_switches);
static Matrix shortest_path(const Matrix& weights, Matrix& latencies, Matrix& inter_switches);
static bool link_is_shortest_path_to_node(SwitchID src, SwitchID next, SwitchID final, const Matrix& weights, const Matrix& dist);
static NetDest shortest_path_to_node(SwitchID src, SwitchID next, const Matrix& weights, const Matrix& dist);


Topology::Topology(Network* network_ptr, int number_of_nodes)
{
  m_network_ptr = network_ptr;
  m_nodes = number_of_nodes;
  m_number_of_switches = 0;
  init();
}

void Topology::init()
{
  if (m_nodes == 1) {
    SwitchID id = newSwitchID();
    addLink(0, id, NETWORK_LINK_LATENCY);
    addLink(id, 1, NETWORK_LINK_LATENCY);
    return;
  }

  // topology-specific set-up
  TopologyType topology = string_to_TopologyType(g_NETWORK_TOPOLOGY);
  switch (topology) {
  case TopologyType_TORUS_2D:
    make2DTorus();
    break;
  case TopologyType_HIERARCHICAL_SWITCH:
    makeHierarchicalSwitch(FAN_OUT_DEGREE);
    break;
  case TopologyType_CROSSBAR:
    makeHierarchicalSwitch(1024);
    break;
  case TopologyType_PT_TO_PT:
    makePtToPt();
    break;
  case TopologyType_FILE_SPECIFIED:
    makeFileSpecified();
    break;
  default:
    ERROR_MSG("Unexpected typology type")
  }

  // initialize component latencies record
  m_component_latencies.setSize(0);
  m_component_inter_switches.setSize(0);
}

void Topology::makeSwitchesPerChip(Vector< Vector < SwitchID > > &nodePairs, Vector<int> &latencies, Vector<int> &bw_multis, int numberOfChipSwitches)
{

  Vector < SwitchID > nodes;  // temporary buffer
  nodes.setSize(2);

  Vector<bool> endpointConnectionExist;  // used to ensure all endpoints are connected to the network
  endpointConnectionExist.setSize(m_nodes);
  // initialize endpoint check vector
  for (int k = 0; k < endpointConnectionExist.size(); k++) {
    endpointConnectionExist[k] = false;
  }

  Vector<int> componentCount;
  componentCount.setSize(MachineType_NUM);
  for (MachineType mType = MachineType_FIRST; mType < MachineType_NUM; ++mType) {
    componentCount[mType] = 0;
  }

  // components to/from network links
  for (int chip = 0; chip < RubyConfig::numberOfChips(); chip++) {
    for (MachineType mType = MachineType_FIRST; mType < MachineType_NUM; ++mType) {
      for (int component = 0; component < MachineType_chip_count(mType, chip); component++) {

        int latency = -1;
        int bw_multiplier = -1;  // internal link bw multiplier of the global bandwidth
        if (mType != MachineType_Directory) {
          latency = ON_CHIP_LINK_LATENCY;  // internal link latency
          bw_multiplier = 10;  // internal link bw multiplier of the global bandwidth
        } else {
          latency = NETWORK_LINK_LATENCY;  // local memory latency
          bw_multiplier = 1;  // local memory link bw multiplier of the global bandwidth
        }
        nodes[0] = MachineType_base_number(mType)+componentCount[mType];
        nodes[1] = chip+m_nodes*2; // this is the chip's internal switch id #

        // insert link
        nodePairs.insertAtBottom(nodes);
        latencies.insertAtBottom(latency);
        //bw_multis.insertAtBottom(bw_multiplier);
        bw_multis.insertAtBottom(componentCount[mType]+MachineType_base_number((MachineType)mType));

        // opposite direction link
        Vector < SwitchID > otherDirectionNodes;
        otherDirectionNodes.setSize(2);
        otherDirectionNodes[0] = nodes[1];
        otherDirectionNodes[1] = nodes[0]+m_nodes;
        nodePairs.insertAtBottom(otherDirectionNodes);
        latencies.insertAtBottom(latency);
        bw_multis.insertAtBottom(bw_multiplier);

        assert(!endpointConnectionExist[nodes[0]]);
        endpointConnectionExist[nodes[0]] = true;
        componentCount[mType]++;
      }
    }
  }

  // make sure all enpoints are connected in the soon to be created network
  for (int k = 0; k < endpointConnectionExist.size(); k++) {
    if (endpointConnectionExist[k] == false) {
      cerr << "Error: Unconnected Endpoint: " << k << endl;
      exit(1);
    }
  }

  // secondary check to ensure we saw the correct machine counts
  for (MachineType mType = MachineType_FIRST; mType < MachineType_NUM; ++mType) {
    assert(componentCount[mType] == MachineType_base_count((MachineType)mType));
  }

}

// 2D torus topology

void Topology::make2DTorus()
{
  Vector< Vector < SwitchID > > nodePairs;  // node pairs extracted from the file
  Vector<int> latencies;  // link latencies for each link extracted
  Vector<int> bw_multis;  // bw multipliers for each link extracted

  Vector < SwitchID > nodes;  // temporary buffer
  nodes.setSize(2);

  // number of inter-chip switches
  int numberOfTorusSwitches = m_nodes/MachineType_base_level(MachineType_NUM);
  // one switch per machine node grouping
  Vector<SwitchID> torusSwitches;
  for(int i=0; i<numberOfTorusSwitches; i++){
    SwitchID new_switch = newSwitchID();
    torusSwitches.insertAtBottom(new_switch);
  }

  makeSwitchesPerChip(nodePairs, latencies, bw_multis, numberOfTorusSwitches);

  int lengthOfSide = (int)sqrt((double)numberOfTorusSwitches);

  // Now connect the inter-chip torus links

  int latency = NETWORK_LINK_LATENCY;  // external link latency
  int bw_multiplier = 1;  // external link bw multiplier of the global bandwidth

  for(int i=0; i<numberOfTorusSwitches; i++){
    nodes[0] = torusSwitches[i];  // current switch

    // left
    if(nodes[0]%lengthOfSide == 0){ // determine left neighbor
      nodes[1] = nodes[0] - 1 + lengthOfSide;
    } else {
      nodes[1] = nodes[0] - 1;
    }
    nodePairs.insertAtBottom(nodes);
    latencies.insertAtBottom(latency);
    bw_multis.insertAtBottom(bw_multiplier);

    // right
    if((nodes[0] + 1)%lengthOfSide == 0){ // determine right neighbor
      nodes[1] = nodes[0] + 1 - lengthOfSide;
    } else {
      nodes[1] = nodes[0] + 1;
    }
    nodePairs.insertAtBottom(nodes);
    latencies.insertAtBottom(latency);
    bw_multis.insertAtBottom(bw_multiplier);

    // top
    if(nodes[0] - lengthOfSide < 2*m_nodes){ // determine if node is on the top
      nodes[1] = nodes[0] - lengthOfSide + (lengthOfSide*lengthOfSide);
    } else {
      nodes[1] = nodes[0] - lengthOfSide;
    }
    nodePairs.insertAtBottom(nodes);
    latencies.insertAtBottom(latency);
    bw_multis.insertAtBottom(bw_multiplier);

    // bottom
    if(nodes[0] + lengthOfSide >= 2*m_nodes+numberOfTorusSwitches){ // determine if node is on the bottom
      // sorin: bad bug if this is a > instead of a >=
      nodes[1] = nodes[0] + lengthOfSide - (lengthOfSide*lengthOfSide);
    } else {
      nodes[1] = nodes[0] + lengthOfSide;
    }
    nodePairs.insertAtBottom(nodes);
    latencies.insertAtBottom(latency);
    bw_multis.insertAtBottom(bw_multiplier);

  }

  // add links
  ASSERT(nodePairs.size() == latencies.size() && latencies.size() == bw_multis.size())
  for (int k = 0; k < nodePairs.size(); k++) {
    ASSERT(nodePairs[k].size() == 2);
    addLink(nodePairs[k][0], nodePairs[k][1], latencies[k], bw_multis[k]);
  }

}

// hierarchical switch topology
void Topology::makeHierarchicalSwitch(int fan_out_degree)
{
  // Make a row of switches with only one input.  This extra row makes
  // sure the links out of the nodes have latency and limited
  // bandwidth.

  // number of inter-chip switches, i.e. the last row of switches
  Vector<SwitchID> last_level;
  for (int i=0; i<m_nodes; i++) {
    SwitchID new_switch = newSwitchID();  // internal switch id #
    addLink(i, new_switch, NETWORK_LINK_LATENCY);
    last_level.insertAtBottom(new_switch);
  }

  // Create Hierarchical Switches

  // start from the bottom level and work up to root
  Vector<SwitchID> next_level;
  while(last_level.size() > 1) {
    for (int i=0; i<last_level.size(); i++) {
      if ((i % fan_out_degree) == 0) {
        next_level.insertAtBottom(newSwitchID());
      }
      // Add this link to the last switch we created
      addLink(last_level[i], next_level[next_level.size()-1], NETWORK_LINK_LATENCY);
    }

    // Make the current level the last level to get ready for next
    // iteration
    last_level = next_level;
    next_level.clear();
  }

  SwitchID root_switch = last_level[0];

  Vector<SwitchID> out_level;
  for (int i=0; i<m_nodes; i++) {
    out_level.insertAtBottom(m_nodes+i);
  }

  // Build the down network from the endpoints to the root
  while(out_level.size() != 1) {

    // A level of switches
    for (int i=0; i<out_level.size(); i++) {
      if ((i % fan_out_degree) == 0) {
        if (out_level.size() > fan_out_degree) {
          next_level.insertAtBottom(newSwitchID());
        } else {
          next_level.insertAtBottom(root_switch);
        }
      }
      // Add this link to the last switch we created
      addLink(next_level[next_level.size()-1], out_level[i], NETWORK_LINK_LATENCY);
    }

    // Make the current level the last level to get ready for next
    // iteration
    out_level = next_level;
    next_level.clear();
  }
}

// one internal node per chip, point to point links between chips
void Topology::makePtToPt()
{
  Vector< Vector < SwitchID > > nodePairs;  // node pairs extracted from the file
  Vector<int> latencies;  // link latencies for each link extracted
  Vector<int> bw_multis;  // bw multipliers for each link extracted

  Vector < SwitchID > nodes;
  nodes.setSize(2);

  // number of inter-chip switches
  int numberOfChipSwitches = m_nodes/MachineType_base_level(MachineType_NUM);
  // two switches per machine node grouping
  // one intra-chip switch and one inter-chip switch per chip
  for(int i=0; i<numberOfChipSwitches; i++){
    SwitchID new_switch = newSwitchID();
    new_switch = newSwitchID();
  }

  makeSwitchesPerChip(nodePairs, latencies, bw_multis, numberOfChipSwitches);

  // connect intra-chip switch to inter-chip switch
  for (int chip = 0; chip < RubyConfig::numberOfChips(); chip++) {

    int latency = ON_CHIP_LINK_LATENCY;  // internal link latency
    int bw_multiplier = 10;  // external link bw multiplier of the global bandwidth

    nodes[0] = chip+m_nodes*2;
    nodes[1] = chip+m_nodes*2+RubyConfig::numberOfChips();

    // insert link
    nodePairs.insertAtBottom(nodes);
    latencies.insertAtBottom(latency);
    bw_multis.insertAtBottom(bw_multiplier);

    // opposite direction link
    Vector < SwitchID > otherDirectionNodes;
    otherDirectionNodes.setSize(2);
    otherDirectionNodes[0] = nodes[1];
    otherDirectionNodes[1] = nodes[0];
    nodePairs.insertAtBottom(otherDirectionNodes);
    latencies.insertAtBottom(latency);
    bw_multis.insertAtBottom(bw_multiplier);
  }

  // point-to-point network between chips
  for (int chip = 0; chip < RubyConfig::numberOfChips(); chip++) {
    for (int other_chip = chip+1; other_chip < RubyConfig::numberOfChips(); other_chip++) {

      int latency = NETWORK_LINK_LATENCY;  // external link latency
      int bw_multiplier = 1;  // external link bw multiplier of the global bandwidth

      nodes[0] = chip+m_nodes*2+RubyConfig::numberOfChips();
      nodes[1] = other_chip+m_nodes*2+RubyConfig::numberOfChips();

      // insert link
      nodePairs.insertAtBottom(nodes);
      latencies.insertAtBottom(latency);
      bw_multis.insertAtBottom(bw_multiplier);

      // opposite direction link
      Vector < SwitchID > otherDirectionNodes;
      otherDirectionNodes.setSize(2);
      otherDirectionNodes[0] = nodes[1];
      otherDirectionNodes[1] = nodes[0];
      nodePairs.insertAtBottom(otherDirectionNodes);
      latencies.insertAtBottom(latency);
      bw_multis.insertAtBottom(bw_multiplier);
    }
  }

  // add links
  ASSERT(nodePairs.size() == latencies.size() && latencies.size() == bw_multis.size())
  for (int k = 0; k < nodePairs.size(); k++) {
    ASSERT(nodePairs[k].size() == 2);
    addLink(nodePairs[k][0], nodePairs[k][1], latencies[k], bw_multis[k]);
  }
}

// make a network as described by the networkFile
void Topology::makeFileSpecified()
{

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

  string filename = "network/simple/Network_Files/";
  filename = filename+g_CACHE_DESIGN
    +"_Procs-"+int_to_string(RubyConfig::numberOfProcessors())
    +"_ProcsPerChip-"+int_to_string(RubyConfig::numberOfProcsPerChip())
    +"_L2Banks-"+int_to_string(RubyConfig::numberOfL2Cache())
    +"_Memories-"+int_to_string(RubyConfig::numberOfMemories())
    +".txt";

  if (g_SIMICS) {
    filename = "../../../ruby/"+filename;
  }
  ifstream networkFile( filename.c_str() , ios::in);
  if (!networkFile.is_open()) {
    cerr << "Error: Could not open network file: " << filename << endl;
    cerr << "Probably no network file exists for " << RubyConfig::numberOfProcessors()
         << " processors and " << RubyConfig::numberOfProcsPerChip() << " procs per chip " << endl;
    exit(1);
  }

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
          if (string_split(varStr, ':') == "bank") {
            nodes[i] = MachineType_base_number(machine)
              + atoi(nodeStr.c_str())
              + atoi((string_split(varStr, ':')).c_str())*RubyConfig::numberOfChips();
          } else {
            nodes[i] = MachineType_base_number(machine)
              + atoi(nodeStr.c_str());
          }
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
      } else if (label == "processors") {
        ASSERT(atoi((string_split(varStr, ':')).c_str()) == RubyConfig::numberOfProcessors());
      } else if (label == "bw_unit") {
        ASSERT(atoi((string_split(varStr, ':')).c_str()) == g_endpoint_bandwidth);
      } else if (label == "procs_per_chip") {
        ASSERT(atoi((string_split(varStr, ':')).c_str()) == RubyConfig::numberOfProcsPerChip());
      } else if (label == "L2banks") {
        ASSERT(atoi((string_split(varStr, ':')).c_str()) == RubyConfig::numberOfL2Cache());
      } else if (label == "memories") {
        ASSERT(atoi((string_split(varStr, ':')).c_str()) == RubyConfig::numberOfMemories());
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

  networkFile.close();
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

