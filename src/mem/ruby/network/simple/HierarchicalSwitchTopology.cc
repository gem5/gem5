
#include "mem/ruby/network/simple/HierarchicalSwitchTopology.hh"

// hierarchical switch topology
void Topology::construct(int fan_out_degree)
{
  // Make a row of switches with only one input.  This extra row makes
  // sure the links out of the nodes have latency and limited
  // bandwidth.

  // number of inter-chip switches, i.e. the last row of switches
  Vector<SwitchID> last_level;
  for (int i=0; i<m_nodes; i++) {
    SwitchID new_switch = newSwitchID();  // internal switch id #
    addLink(i, new_switch, m_network_ptr->getLinkLatency());
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
      addLink(last_level[i], next_level[next_level.size()-1], m_network_ptr->getLinkLatency());
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
      addLink(next_level[next_level.size()-1], out_level[i], m_network_ptr->getLinkLatency());
    }

    // Make the current level the last level to get ready for next
    // iteration
    out_level = next_level;
    next_level.clear();
  }
}
