
// 2D torus topology

void Torus2DTopology::construct()
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

  int latency = m_network_ptr->getLinkLatency();  // external link latency
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
