
#include "mem/ruby/network/simple/CustomTopology.hh"
#include "mem/protocol/MachineType.hh"

static const int INFINITE_LATENCY = 10000; // Yes, this is a big hack
static const int DEFAULT_BW_MULTIPLIER = 1;  // Just to be consistent with above :)

// make a network as described by the networkFile
void CustomTopology::construct()
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

  //  networkFile.close();
}
