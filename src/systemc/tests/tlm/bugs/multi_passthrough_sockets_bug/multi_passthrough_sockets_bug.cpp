/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/
#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc"
#include "tlm"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/multi_passthrough_target_socket.h"

class introspection_extension;

class initiator_module : public sc_core::sc_module
{
public:
  
  tlm_utils::simple_initiator_socket<initiator_module> initiator_socket;
  SC_HAS_PROCESS(initiator_module);
  explicit initiator_module(sc_core::sc_module_name module_name)
    : sc_core::sc_module(module_name)
    , initiator_socket("initiator_socket")
  {
    SC_THREAD(process);
  }
 
  void process()
  {
    // To verify regular TLM-2 access from initiators are OK
    tlm::tlm_generic_payload transaction;
    
    unsigned char byte = 0x42;
    
    transaction.set_command(tlm::TLM_WRITE_COMMAND);
    transaction.set_address(sc_dt::uint64(0));
    transaction.set_data_length(1);
    transaction.set_streaming_width(1);
    transaction.set_data_ptr(&byte);
    transaction.set_byte_enable_ptr(0);
    transaction.set_byte_enable_length(0);
    transaction.set_dmi_allowed(false);
    transaction.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    
    sc_core::sc_time t = sc_core::SC_ZERO_TIME;
    
    initiator_socket->b_transport(transaction, t);
  }
  
};

class target_module : public sc_core::sc_module
{
public:
  
  tlm_utils::multi_passthrough_target_socket<target_module> target_socket;
  tlm_utils::multi_passthrough_target_socket<target_module, 32, tlm::tlm_base_protocol_types,0,::sc_core::SC_ZERO_OR_MORE_BOUND> target_optional;
  
  explicit target_module(sc_core::sc_module_name module_name)
    : sc_core::sc_module(module_name)
    , target_socket("target_socket")
    , target_optional("target_optional")
  {
    target_socket.register_b_transport(this, &target_module::transport);
    target_socket.register_transport_dbg(this, &target_module::transport_dbg);
    target_socket.register_get_direct_mem_ptr(this, &target_module::get_direct_mem_ptr);

    // bind callbacks to optional socket (unbound)
    target_optional.register_b_transport(this, &target_module::transport);
    target_optional.register_transport_dbg(this, &target_module::transport_dbg);
    target_optional.register_get_direct_mem_ptr(this, &target_module::get_direct_mem_ptr);
  }
  
  virtual void transport(int port, tlm::tlm_generic_payload & transaction, sc_core::sc_time & t) {}

  virtual unsigned int transport_dbg(int port, tlm::tlm_generic_payload & transaction)
  {
    if ((transaction.get_command() == tlm::TLM_IGNORE_COMMAND) &&
        transaction.get_extension<introspection_extension>())
    {
      std::cout << "Received successfully introspection extension!" << std::endl;
    }

    return 0;
  }

  virtual bool get_direct_mem_ptr(int port, tlm::tlm_generic_payload & transaction, tlm::tlm_dmi & dmi_data) { return false; }

};

// Simple empty extension to verify the target module is receiving it
class introspection_extension : public tlm::tlm_extension<introspection_extension>
{
public:
  
  virtual tlm_extension_base * clone() const
  {
    return new introspection_extension;
  }
  
  virtual void copy_from(tlm_extension_base const & ext)
  {
  }
  
};

class introspector_module : public sc_core::sc_module
{
public:
  
  SC_HAS_PROCESS(introspector_module);
  introspector_module(sc_core::sc_module_name module_name) :
    sc_core::sc_module(module_name)
  {
    SC_THREAD(process);
  }
  
  void send_introspection_request(sc_core::sc_export<tlm::tlm_fw_transport_if<> > & target_socket)
  {
    tlm::tlm_generic_payload transaction;
    
    transaction.set_command(tlm::TLM_IGNORE_COMMAND);
    transaction.set_address(sc_dt::uint64(0));
    transaction.set_data_length(0);
    transaction.set_streaming_width(0);
    transaction.set_data_ptr(0);
    transaction.set_byte_enable_ptr(0);
    transaction.set_byte_enable_length(0);
    transaction.set_dmi_allowed(false);
    transaction.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    
    introspection_extension *ext = new introspection_extension;
    
    transaction.set_extension(ext);

    target_socket->transport_dbg(transaction);
  }
  
  void find_target_sockets(sc_core::sc_module *module)
  {
    std::vector<sc_core::sc_object*> children = module->get_child_objects();
    
    for (std::vector<sc_core::sc_object*>::iterator i = children.begin(); i != children.end(); ++i)
    {
      if (dynamic_cast<sc_core::sc_module *>(*i))
      {
        find_target_sockets(dynamic_cast<sc_core::sc_module *>(*i));
      }
      else if (dynamic_cast<sc_core::sc_export<tlm::tlm_fw_transport_if<> > *>((*i)))
      {
        sc_core::sc_export<tlm::tlm_fw_transport_if<> > * target_socket = dynamic_cast<sc_core::sc_export<tlm::tlm_fw_transport_if<> > *>(*i);
        
        send_introspection_request(*target_socket);
      }
    }
  }
    
  void process()
  {
    const std::vector<sc_core::sc_object*> & top_objs = sc_core::sc_get_top_level_objects();
    
    for (std::vector<sc_core::sc_object*>::const_iterator i = top_objs.begin(); i != top_objs.end(); ++i)
    {
      if (dynamic_cast<sc_core::sc_module *>(*i))
      {
        find_target_sockets(dynamic_cast<sc_core::sc_module *>(*i));
      }
    }
  }
  
};

int sc_main(int argc, char* argv[])
{
  initiator_module    initiator("initiator");
  target_module       target("target");
  introspector_module introspector("introspector");
  
  initiator.initiator_socket(target.target_socket);
  
  sc_core::sc_start();

  return 0;
}
