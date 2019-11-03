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

#ifndef __EXPLICIT_AT_TARGET_H__
#define __EXPLICIT_AT_TARGET_H__

#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
//#include <systemc>
#include <cassert>
#include <vector>
#include <queue>
//#include <iostream>

class ExplicitATTarget : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload                 transaction_type;
  typedef tlm::tlm_phase                           phase_type;
  typedef tlm::tlm_sync_enum                       sync_enum_type;
  typedef tlm_utils::simple_target_socket<ExplicitATTarget>     target_socket_type;

public:
  target_socket_type socket;

public:
  SC_HAS_PROCESS(ExplicitATTarget);
  ExplicitATTarget(sc_core::sc_module_name name) :
    sc_core::sc_module(name),
    socket("socket"),
    mCurrentTransaction(0)
  {
    // register nb_transport method
    socket.register_nb_transport_fw(this, &ExplicitATTarget::myNBTransport);
    socket.register_transport_dbg(this, &ExplicitATTarget::transport_dbg);

    SC_THREAD(beginResponse)
  }

  sync_enum_type myNBTransport(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
  {
    if (phase == tlm::BEGIN_REQ) {
      sc_dt::uint64 address = trans.get_address();
      assert(address < 400);

      // This target only supports one transaction at a time
      // This will only work with LT initiators
      assert(mCurrentTransaction == 0);

      unsigned int& data = *reinterpret_cast<unsigned int*>(trans.get_data_ptr());
      if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
        std::cout << name() << ": Received write request: A = 0x"
                  << std::hex << (unsigned int)address << ", D = 0x" << data
                  << std::dec << " @ " << sc_core::sc_time_stamp()
                  << std::endl;

        *reinterpret_cast<unsigned int*>(&mMem[address]) = data;

        // Synchronization on demand (eg need to assert an interrupt)
        mResponseEvent.notify(t);
        mCurrentTransaction = &trans;

        // End request phase
        phase = tlm::END_REQ;
        return tlm::TLM_UPDATED;

      } else {
        std::cout << name() << ": Received read request: A = 0x"
                  << std::hex << (unsigned int)address
                  << std::dec << " @ " << sc_core::sc_time_stamp()
                  << std::endl;

        data = *reinterpret_cast<unsigned int*>(&mMem[address]);
        trans.set_response_status(tlm::TLM_OK_RESPONSE);

        // Finish transaction (use timing annotation)
        t += sc_core::sc_time(100, sc_core::SC_NS);
        return tlm::TLM_COMPLETED;
      }

    } else if (phase == tlm::END_RESP) {

      // Transaction finished
      mCurrentTransaction = 0;
      return tlm::TLM_COMPLETED;
    }

    // Not possible
    assert(0); exit(1);
//    return tlm::TLM_COMPLETED;  //unreachable code
  }

  void beginResponse()
  {
    while (true) {
      // Wait for next synchronization request
      wait(mResponseEvent);

      assert(mCurrentTransaction);
      // start response phase
      phase_type phase = tlm::BEGIN_RESP;
      sc_core::sc_time t = sc_core::SC_ZERO_TIME;

      // Set response data
      mCurrentTransaction->set_response_status(tlm::TLM_OK_RESPONSE);
      assert(mCurrentTransaction->get_command() == tlm::TLM_WRITE_COMMAND);

      sc_dt::uint64 address = mCurrentTransaction->get_address();
      assert(address < 400);
      *reinterpret_cast<unsigned int*>(mCurrentTransaction->get_data_ptr()) =
        *reinterpret_cast<unsigned int*>(&mMem[address]);

      // We are synchronized, we can read/write sc_signals, wait,...
      // Wait before sending the response
      wait(50, sc_core::SC_NS);

      if (socket->nb_transport_bw(*mCurrentTransaction, phase, t) == tlm::TLM_COMPLETED) {
        mCurrentTransaction = 0;

      } else {
        // Initiator will call nb_transport(trans, END_RESP, t)
      }
    }
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
    if (!r.is_read() && !r.is_write()) {
      return 0;
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

private:
  unsigned char mMem[400];
  sc_core::sc_event mResponseEvent;
  transaction_type* mCurrentTransaction;
};

#endif
