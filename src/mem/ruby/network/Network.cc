
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
   else if (argv[i] == "control_msg_size")
     m_control_msg_size = atoi(argv[i+1].c_str());		 
  }

  m_data_msg_size = RubySystem::getBlockSizeBytes() + m_control_msg_size;

  assert(m_virtual_networks != 0);
  assert(m_topology_ptr != NULL);
}

int Network::MessageSizeType_to_int(MessageSizeType size_type)
{
  switch(size_type) {
  case MessageSizeType_Undefined:
    ERROR_MSG("Can't convert Undefined MessageSizeType to integer");
    break;
  case MessageSizeType_Control:
  case MessageSizeType_Request_Control:
  case MessageSizeType_Reissue_Control:
  case MessageSizeType_Response_Control:
  case MessageSizeType_Writeback_Control:
  case MessageSizeType_Forwarded_Control:
  case MessageSizeType_Invalidate_Control:
  case MessageSizeType_Unblock_Control:
  case MessageSizeType_Persistent_Control:
  case MessageSizeType_Completion_Control:
    return m_control_msg_size;
    break;
  case MessageSizeType_Data:
  case MessageSizeType_Response_Data:
  case MessageSizeType_ResponseLocal_Data:
  case MessageSizeType_ResponseL2hit_Data:
  case MessageSizeType_Writeback_Data:
    return m_data_msg_size;
    break;
  default:
    ERROR_MSG("Invalid range for type MessageSizeType");
    break;
  }
  return 0;
}
