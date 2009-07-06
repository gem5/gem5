
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/network/simple/PtToPtTopology.hh"

// one internal node per chip, point to point links between chips
void PtToPtTopology::construct()
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
  for (int chip = 0; chip < RubyConfig::getNumberOfChips(); chip++) {

    int latency = m_network_ptr->getOnChipLinkLatency();  // internal link latency
    int bw_multiplier = 10;  // external link bw multiplier of the global bandwidth

    nodes[0] = chip+m_nodes*2;
    nodes[1] = chip+m_nodes*2+RubyConfig::getNumberOfChips();

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
  for (int chip = 0; chip < RubyConfig::getNumberOfChips(); chip++) {
    for (int other_chip = chip+1; other_chip < RubyConfig::getNumberOfChips(); other_chip++) {

      int latency = m_network_ptr->getOffChipLinkLatency();  // external link latency
      int bw_multiplier = 1;  // external link bw multiplier of the global bandwidth

      nodes[0] = chip+m_nodes*2+RubyConfig::getNumberOfChips();
      nodes[1] = other_chip+m_nodes*2+RubyConfig::getNumberOfChips();

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
