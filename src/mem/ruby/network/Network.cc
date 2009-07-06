
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

  }
  assert(m_virtual_networks != 0);
  assert(m_topology_ptr != NULL);
//  printf ("HERE \n");
}
