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

#ifndef __SIMPLE_AT_TARGET2_H__
#define __SIMPLE_AT_TARGET2_H__

#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
//#include <systemc>
#include <cassert>
#include <vector>
#include <queue>
//#include <iostream>

class SimpleATTarget2 : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload            transaction_type;
  typedef tlm::tlm_phase                      phase_type;
  typedef tlm::tlm_sync_enum                  sync_enum_type;
  typedef tlm_utils::simple_target_socket<SimpleATTarget2> target_socket_type;

public:
  target_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleATTarget2);
  SimpleATTarget2(sc_core::sc_module_name name) :
    sc_core::sc_module(name),
    socket("socket"),
    ACCEPT_DELAY(25, sc_core::SC_NS),
    RESPONSE_DELAY(100, sc_core::SC_NS)
  {
    // register nb_transport method
    socket.register_nb_transport_fw(this, &SimpleATTarget2::myNBTransport);

    SC_METHOD(beginResponse)
    sensitive << mBeginResponseEvent;
    dont_initialize();

    SC_METHOD(endResponse)
    sensitive << mEndResponseEvent;
    dont_initialize();
  }

  //
  // Simple AT-TA target
  // - Request is accepted after fixed delay (relative to end of prev request
  //   phase)
  // - Response is started after fixed delay (relative to end of prev resp
  //   phase)
  //
  sync_enum_type myNBTransport(transaction_type& trans,
                               phase_type& phase,
                               sc_core::sc_time& t)
  {
    if (phase == tlm::BEGIN_REQ) {
      // transactions may be kept in queue after the initiator has send END_REQ
      trans.acquire();

      sc_dt::uint64 address = trans.get_address();
      assert(address < 400);

      unsigned int& data = *reinterpret_cast<unsigned int*>(trans.get_data_ptr());
      if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
        std::cout << name() << ": Received write request: A = 0x"
                  << std::hex << (unsigned int)address << ", D = 0x" << data
                  << std::dec << " @ " << sc_core::sc_time_stamp()
                  << std::endl;

        *reinterpret_cast<unsigned int*>(&mMem[address]) = data;

      } else {
        std::cout << name() << ": Received read request: A = 0x"
                  << std::hex << (unsigned int)address
                  << std::dec << " @ " << sc_core::sc_time_stamp()
                  << std::endl;

        data = *reinterpret_cast<unsigned int*>(&mMem[address]);
      }

      // End request phase after accept delay
      t += ACCEPT_DELAY;
      phase = tlm::END_REQ;

      if (mResponseQueue.empty()) {
        // Start processing transaction after accept delay
        // Notify begin of response phase after accept delay + response delay
        mBeginResponseEvent.notify(t + RESPONSE_DELAY);
      }
      mResponseQueue.push(&trans);

      // AT-noTA target
      // - always return false
      // - immediately return delay to indicate end of phase
      return tlm::TLM_UPDATED;

    } else if (phase == tlm::END_RESP) {

      // response phase ends after t
      mEndResponseEvent.notify(t);

      return tlm::TLM_COMPLETED;
    }

    // Not possible
    assert(0); exit(1);
//    return tlm::TLM_COMPLETED;  //unreachable code
  }

  void beginResponse()
  {
    assert(!mResponseQueue.empty());
    // start response phase of oldest transaction
    phase_type phase = tlm::BEGIN_RESP;
    sc_core::sc_time t = sc_core::SC_ZERO_TIME;
    transaction_type* trans = mResponseQueue.front();
    assert(trans);

    // Set response data
    trans->set_response_status(tlm::TLM_OK_RESPONSE);
    if (trans->get_command() == tlm::TLM_READ_COMMAND) {
       sc_dt::uint64 address = trans->get_address();
       assert(address < 400);
      *reinterpret_cast<unsigned int*>(trans->get_data_ptr()) =
        *reinterpret_cast<unsigned int*>(&mMem[address]);
    }

    if (socket->nb_transport_bw(*trans, phase, t) == tlm::TLM_COMPLETED) {
      // response phase ends after t
      mEndResponseEvent.notify(t);

    } else {
      // initiator will call nb_transport to indicate end of response phase
    }
  }

  void endResponse()
  {
    assert(!mResponseQueue.empty());
    mResponseQueue.front()->release();
    mResponseQueue.pop();

    // Start processing next transaction when previous response is accepted.
    // Notify begin of response phase after RESPONSE delay
    if (!mResponseQueue.empty()) {
      mBeginResponseEvent.notify(RESPONSE_DELAY);
    }
  }

private:
  const sc_core::sc_time ACCEPT_DELAY;
  const sc_core::sc_time RESPONSE_DELAY;

private:
  unsigned char mMem[400];
  std::queue<transaction_type*> mResponseQueue;
  sc_core::sc_event mBeginResponseEvent;
  sc_core::sc_event mEndResponseEvent;
};

#endif
