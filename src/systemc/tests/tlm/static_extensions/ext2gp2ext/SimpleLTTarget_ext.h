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

#ifndef __SIMPLE_LT_TARGET2_H__
#define __SIMPLE_LT_TARGET2_H__

#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include "my_extension.h"

//#include <systemc>
#include <cassert>
#include <vector>
//#include <iostream>

class SimpleLTTarget_ext : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload              transaction_type;
  typedef tlm::tlm_phase                        phase_type;
  typedef tlm::tlm_sync_enum                    sync_enum_type;
  typedef tlm_utils::simple_target_socket<SimpleLTTarget_ext, 32,
                             my_extended_payload_types> target_socket_type;

public:
  target_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleLTTarget_ext);
  SimpleLTTarget_ext(sc_core::sc_module_name name,
                     sc_core::sc_time invalidate_dmi_time = sc_core::sc_time(25, sc_core::SC_NS)) :
    sc_core::sc_module(name),
    socket("socket")
  {
    // register nb_transport method
    socket.register_nb_transport_fw(this, &SimpleLTTarget_ext::myNBTransport);
    socket.register_get_direct_mem_ptr(this, &SimpleLTTarget_ext::myGetDMIPtr);

    socket.register_transport_dbg(this, &SimpleLTTarget_ext::transport_dbg);
    
    SC_METHOD(invalidate_dmi_method);
    sensitive << m_invalidate_dmi_event;
    dont_initialize();
    m_invalidate_dmi_time = invalidate_dmi_time;
  }

  sync_enum_type myNBTransport(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
  {
    sc_assert(phase == tlm::BEGIN_REQ);

    my_extension* tmp_ext;
    trans.get_extension(tmp_ext);
    if (!tmp_ext)
    {
        std::cout << name() << ": ERROR, extension not present" << std::endl;
    }
    else
    {
        std::cout << name() << ": OK, extension data = "
                  << tmp_ext->m_data << std::endl;
    }
    sc_dt::uint64 address = trans.get_address();
    sc_assert(address < 400);

    unsigned int& data = *reinterpret_cast<unsigned int*>(trans.get_data_ptr());
    if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
      std::cout << name() << ": Received write request: A = 0x"
                << std::hex << (unsigned int)address
                << ", D = 0x" << data << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;

      *reinterpret_cast<unsigned int*>(&mMem[address]) = data;
      t += sc_core::sc_time(10, sc_core::SC_NS);

    } else {
      std::cout << name() << ": Received read request: A = 0x"
                << std::hex << (unsigned int)address << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;

      data = *reinterpret_cast<unsigned int*>(&mMem[address]);
      t += sc_core::sc_time(100, sc_core::SC_NS);
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);

    trans.set_dmi_allowed(true);

    // LT target
    // - always return true
    // - not necessary to update phase (if true is returned)
    return tlm::TLM_COMPLETED;
  }

  unsigned int transport_dbg(transaction_type& r)
  {
    if (r.get_address() >= 400) return 0;

    unsigned int tmp = (int)r.get_address();
    unsigned int num_bytes;
    if (tmp + r.get_data_length() >= 400) {
      num_bytes = 400 - tmp;

    } else {
      num_bytes = r.get_data_length();
    }
    if (r.is_read()) {
      for (unsigned int i = 0; i < num_bytes; ++i) {
        r.get_data_ptr()[i] = mMem[i + tmp];
      }

    } else {
      for (unsigned int i = 0; i < num_bytes; ++i) {
        mMem[i + tmp] = r.get_data_ptr()[i];
      }
    }
    return num_bytes;
  }

  bool myGetDMIPtr(transaction_type& trans,
                   tlm::tlm_dmi&  dmi_data)
  {
      // notify DMI invalidation, just to check if this reaches the
      // initiators properly
      m_invalidate_dmi_event.notify(m_invalidate_dmi_time);

      // Check for DMI extension:
      my_extension * tmp_ext;
      trans.get_extension(tmp_ext);
      if (tmp_ext)
      {
        std::cout << name() << ": get_direct_mem_ptr OK, extension data = "
                  <<tmp_ext->m_data << std::endl;
      }
      else
      {
          std::cout << name() << ", get_direct_mem_ptr ERROR: "
                    << "didn't get pointer to extension"
                    << std::endl;          
      }
      if (trans.get_address() < 400) {
        dmi_data.allow_read_write();
        dmi_data.set_start_address(0x0);
        dmi_data.set_end_address(399);
        dmi_data.set_dmi_ptr(mMem);
        dmi_data.set_read_latency(sc_core::sc_time(100, sc_core::SC_NS));
        dmi_data.set_write_latency(sc_core::sc_time(10, sc_core::SC_NS));
        return true;

      } else {
        // should not happen
        dmi_data.set_start_address(trans.get_address());
        dmi_data.set_end_address(trans.get_address());
        return false;

      }
  }

  void invalidate_dmi_method()
  {
      sc_dt::uint64 start_address = 0x0;
      sc_dt::uint64 end_address = 399;
      socket->invalidate_direct_mem_ptr(start_address, end_address);
  }
private:
  unsigned char mMem[400];
  sc_core::sc_event m_invalidate_dmi_event;
  sc_core::sc_time  m_invalidate_dmi_time;
};

#endif
