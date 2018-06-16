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

//====================================================================
//  Nov 06, 2008
//
//  Updated by:
//    Xiaopeng Qiu, JEDA Technologies, Inc
//    Email:  qiuxp@jedatechnologies.net
//
//  To fix violations of TLM2.0 rules, which are detected by JEDA 
//  TLM2.0 checker.
//
//====================================================================

#ifndef __SIMPLE_LT_INITIATOR3_H__
#define __SIMPLE_LT_INITIATOR3_H__

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
//#include <systemc>
#include <cassert>
//#include <iostream>

class SimpleLTInitiator3 : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload                  transaction_type;
  typedef tlm::tlm_phase                            phase_type;
  typedef tlm::tlm_sync_enum                        sync_enum_type;
  typedef tlm_utils::simple_initiator_socket<SimpleLTInitiator3> initiator_socket_type;

public:
  initiator_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleLTInitiator3);
  SimpleLTInitiator3(sc_core::sc_module_name name,
                  unsigned int nrOfTransactions = 0x5,
                  unsigned int baseAddress = 0x0) :
    sc_core::sc_module(name),
    socket("socket"),
    mNrOfTransactions(nrOfTransactions),
    mBaseAddress(baseAddress),
    mTransactionCount(0)
  {
    // Initiator thread
    SC_THREAD(run);
  }

  bool initTransaction(transaction_type& trans)
  {
    if (mTransactionCount < mNrOfTransactions) {
      trans.set_address(mBaseAddress + 4*mTransactionCount);
      mData = mTransactionCount;
      trans.set_command(tlm::TLM_WRITE_COMMAND);

    } else if (mTransactionCount < 2 * mNrOfTransactions) {
      trans.set_address(mBaseAddress + 4*(mTransactionCount-mNrOfTransactions));
      mData = 0;
      trans.set_command(tlm::TLM_READ_COMMAND);

    } else {
      return false;
    }

    trans.set_data_ptr(reinterpret_cast<unsigned char*>(&mData));
    trans.set_data_length(4);
    trans.set_streaming_width(4);
    trans.set_dmi_allowed(false);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    ++mTransactionCount;
    return true;
  }

  void logStartTransation(transaction_type& trans)
  {
    if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
      std::cout << name() << ": Send write request: A = 0x"
                << std::hex << (unsigned int)trans.get_address()
                << ", D = 0x" << mData << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;
      
    } else {
      std::cout << name() << ": Send read request: A = 0x"
                << std::hex << (unsigned int)trans.get_address() << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;
    }
  }

  void logEndTransaction(transaction_type& trans)
  {
    if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
      std::cout << name() << ": Received error response @ "
                << sc_core::sc_time_stamp() << std::endl;

    } else {
      std::cout << name() <<  ": Received ok response";
      if (trans.get_command() == tlm::TLM_READ_COMMAND) {
        std::cout << ": D = 0x" << std::hex << mData << std::dec;
      }
      std::cout << " @ " << sc_core::sc_time_stamp() << std::endl;
    }
  }

  void run()
  {
    transaction_type trans;
    sc_core::sc_time t;
    
    while (initTransaction(trans)) {
      // Create transaction and initialise t
      t = sc_core::SC_ZERO_TIME;

      logStartTransation(trans);

      socket->b_transport(trans, t);
      // Transaction Finished, wait for the returned delay
      wait(t);

      logEndTransaction(trans);
    }
    wait();

  }

private:
  sc_core::sc_event mEndEvent;
  unsigned int mNrOfTransactions;
  unsigned int mBaseAddress;
  unsigned int mTransactionCount;
  unsigned int mData;
};

#endif
