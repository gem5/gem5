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

#ifndef __SIMPLE_LT_TARGET1_H__
#define __SIMPLE_LT_TARGET1_H__

#include "tlm.h"
#include <cassert>
#include <vector>

class SimpleLTTarget1 :
  public sc_core::sc_module,
  public virtual tlm::tlm_fw_transport_if<>
{
public:
  typedef tlm::tlm_generic_payload      transaction_type;
  typedef tlm::tlm_phase                phase_type;
  typedef tlm::tlm_sync_enum            sync_enum_type;
  typedef tlm::tlm_fw_transport_if<>    fw_interface_type;
  typedef tlm::tlm_bw_transport_if<>    bw_interface_type;
  typedef tlm::tlm_target_socket<32>    target_socket_type;

public:
  target_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleLTTarget1);
  SimpleLTTarget1(sc_core::sc_module_name name, bool invalidate = false) :
      sc_core::sc_module(name),
      socket("socket"),
      m_invalidate(invalidate)
  {
    // Bind this target's interface to the target socket
    socket(*this);
    if (invalidate)
    {
        SC_METHOD(invalidate_dmi_method);
        sensitive << m_invalidate_dmi_event;
        dont_initialize();
        m_invalidate_dmi_time = sc_core::sc_time(25, sc_core::SC_NS);
    }
  }

  sync_enum_type nb_transport_fw(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
  {
    //Target never calls wait, so we can do this
    b_transport(trans, t);

    return tlm::TLM_COMPLETED;
  }

  void b_transport(transaction_type& trans, sc_core::sc_time &t)
  {
    sc_dt::uint64 address = trans.get_address();
    assert(address < 400);

    unsigned int& data = *reinterpret_cast<unsigned int*>(trans.get_data_ptr());
    if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
      std::cout << name() << ": Received write request: A = 0x"
                << std::hex << (unsigned int)address
                << ", D = 0x" << data << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;

      *reinterpret_cast<unsigned int*>(&mMem[address]) = data;
      t+=  sc_core::sc_time(10, sc_core::SC_NS);

    } else {
      std::cout << name() << ": Received read request: A = 0x"
                << std::hex << (unsigned int)address << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;

      data = *reinterpret_cast<unsigned int*>(&mMem[address]);
      t += sc_core::sc_time(100, sc_core::SC_NS);
    }

    trans.set_response_status(tlm::TLM_OK_RESPONSE);

    trans.set_dmi_allowed(true);
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

  bool get_direct_mem_ptr(transaction_type& trans,
                          tlm::tlm_dmi&  dmi_data)
  {
    sc_dt::uint64 address = trans.get_address();
    if (m_invalidate) m_invalidate_dmi_event.notify(m_invalidate_dmi_time);
    if (address < 400) {
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
  bool              m_invalidate;
  sc_core::sc_event m_invalidate_dmi_event;
  sc_core::sc_time  m_invalidate_dmi_time;
};

#endif
