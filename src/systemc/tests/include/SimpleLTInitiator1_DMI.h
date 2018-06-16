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

#ifndef __SIMPLE_LT_INITIATOR1_DMI_H__
#define __SIMPLE_LT_INITIATOR1_DMI_H__

#include "tlm.h"
#include <systemc>
#include <cassert>
#include <iostream>
#include <iomanip>

class SimpleLTInitiator1_dmi :
  public sc_core::sc_module,
  public virtual tlm::tlm_bw_transport_if<>
{
public:
  typedef tlm::tlm_generic_payload      transaction_type;
  typedef tlm::tlm_dmi                  dmi_type;
  typedef tlm::tlm_phase                phase_type;
  typedef tlm::tlm_sync_enum            sync_enum_type;
  typedef tlm::tlm_fw_transport_if<>    fw_interface_type;
  typedef tlm::tlm_bw_transport_if<>    bw_interface_type;
  typedef tlm::tlm_initiator_socket<>   initiator_socket_type;

public:
  initiator_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleLTInitiator1_dmi);
  SimpleLTInitiator1_dmi(sc_core::sc_module_name name,
                         unsigned int nrOfTransactions = 0x5,
                         unsigned int baseAddress = 0x0) :
    sc_core::sc_module(name),
    socket("socket"),
    mNrOfTransactions(nrOfTransactions),
    mBaseAddress(baseAddress),
    mTransactionCount(0)
  {
    invalidate(mDMIData);

    // Bind this initiator's interface to the initiator socket
    socket(*this);

    // Initiator thread
    SC_THREAD(run);
  }

  bool initTransaction(transaction_type& trans)
  {
    // initialize DMI hint:
    trans.set_dmi_allowed(false);

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
    phase_type phase;
    sc_core::sc_time t;
    
    while (initTransaction(trans)) {
      // Create transaction and initialise phase and t
      phase = tlm::BEGIN_REQ;
      t = sc_core::SC_ZERO_TIME;

      logStartTransation(trans);

      ///////////////////////////////////////////////////////////
      // DMI handling:
      // We use the DMI hint to check if it makes sense to ask for
      // DMI pointers. The pattern is:
      // - if the address is covered by a DMI region do a DMI access
      // - otherwise do a normal transaction
      //   -> check if we get a DMI hint and acquire the DMI pointers if it is
      //      set
      ///////////////////////////////////////////////////////////

      // Check if the address is covered by our DMI region
      if ( (trans.get_address() >= mDMIData.get_start_address()) &&
           (trans.get_address() <= mDMIData.get_end_address()) ) {
          // We can handle the data here. As the logEndTransaction is assuming
          // something to happen in the data structure, we really need to
          // do this:
          trans.set_response_status(tlm::TLM_OK_RESPONSE);
          sc_dt::uint64 tmp = trans.get_address() - mDMIData.get_start_address();
          if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
              *(unsigned int*)&mDMIData.get_dmi_ptr()[tmp] = mData;

          } else {
              mData = *(unsigned int*)&mDMIData.get_dmi_ptr()[tmp];
          }
          
          // Do the wait immediately. Note that doing the wait here eats almost
          // all the performance anyway, so we only gain something if we're
          // using temporal decoupling.
          if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
            wait(mDMIData.get_write_latency());

          } else {
            wait(mDMIData.get_read_latency());
          }
          
          logEndTransaction(trans);

      } else { // we need a full transaction
          sc_dt::uint64 addr = trans.get_address(); //Save address before it is mutated
          socket->b_transport(trans, t);
          wait(t);
          logEndTransaction(trans);
          
		  // Acquire DMI pointer on is available:
          if (trans.is_dmi_allowed())
          {
              dmi_type tmp;
              tmp.init();
              trans.set_address(addr);  //restore address, in case it was mutated.
              trans.set_write();
              if ( socket->get_direct_mem_ptr(trans, tmp)
                   && tmp.is_write_allowed() )
              {
                  mDMIData = tmp;
              }
          }
      }
    }
    wait();
  }

  sync_enum_type nb_transport_bw(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
  {
      // We should never be called
      assert(0);
      return tlm::TLM_COMPLETED;
  }

  void invalidate(dmi_type& dmiData)
  {
    dmiData.set_start_address(1);
    dmiData.set_end_address(0);
  }

  // Invalidate DMI pointer(s)
  void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                 sc_dt::uint64 end_range)
  {
      // do the invalidation if there is an address range overlap
      if (start_range <= mDMIData.get_end_address ()&&
          end_range >= mDMIData.get_start_address()) {
          std::cout <<  name() << ": got DMI pointer invalidation"
                    << " @ " << sc_core::sc_time_stamp() << std::endl;
          
          invalidate(mDMIData);
      } else {
          std::cout <<  name() << ": ignored DMI invalidation for addresses "
                    << std::hex << start_range << ", "
                    << end_range << std::dec
                    << " @ " << sc_core::sc_time_stamp() << std::endl;
      }
  }

  // Test for transport_dbg:
  // FIXME: use a configurable address
  void end_of_simulation()
  {
    std::cout <<  name() << ", <<SimpleLTInitiator1>>:" << std::endl
              << std::endl;
    unsigned char data[32];

    transaction_type trans;
    trans.set_address(mBaseAddress);
    trans.set_data_length(32);
    trans.set_data_ptr(data);
    trans.set_read();

    unsigned int n = socket->transport_dbg(trans);
        
    std::cout << "Mem @" << std::hex << mBaseAddress << std::endl;
    unsigned int j = 0;
        
    if (n > 0)
    {
        // always align endianness, so that we don't get a diff when
        // printing the raw data
        int e_start = 0;
        int e_end = 4;
        int e_increment = 1;
        if (!tlm::host_has_little_endianness())
        {
            e_start = 3;
            e_end = -1;
            e_increment = -1;
        }
        
        for (unsigned int i=0; i<n; i+=4)
        {
            for (int k=e_start; k!=e_end; k+=e_increment)
            {
                std::cout << std::setw(2) << std::setfill('0')
                          << (int)data[i+k];
                j++;
                if (j==16) {
                    j=0;
                    std::cout << std::endl;
                } else {
                    std::cout << " ";
                }
            }
        }
    }
    else
    {
        std::cout << "ERROR: debug transaction didn't give data." << std::endl;
    }
    std::cout << std::dec << std::endl;  
  }
private:
  dmi_type mDMIData;

  sc_core::sc_event mEndEvent;
  unsigned int mNrOfTransactions;
  unsigned int mBaseAddress;
  unsigned int mTransactionCount;
  unsigned int mData;
};

#endif
