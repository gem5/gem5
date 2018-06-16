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

#ifndef __SIMPLE_LT_INITIATOR2_DMI_H__
#define __SIMPLE_LT_INITIATOR2_DMI_H__

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include <systemc>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <map>

class SimpleLTInitiator2_dmi : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload                      transaction_type;
  typedef tlm::tlm_dmi                                  dmi_type;
  typedef tlm::tlm_phase                                phase_type;
  typedef tlm::tlm_sync_enum                            sync_enum_type;
  typedef tlm_utils::simple_initiator_socket<SimpleLTInitiator2_dmi> initiator_socket_type;

public:
  initiator_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleLTInitiator2_dmi);
  SimpleLTInitiator2_dmi(sc_core::sc_module_name name,
                  unsigned int nrOfTransactions = 0x5,
                  unsigned int baseAddress = 0x0) :
    sc_core::sc_module(name),
    socket("socket"),
    mNrOfTransactions(nrOfTransactions),
    mBaseAddress(baseAddress),
    mTransactionCount(0)
  {
    mDMIDataReads.first.set_start_address(1);
    mDMIDataReads.first.set_end_address(0);
    mDMIDataWrites.first.set_start_address(1);
    mDMIDataWrites.first.set_end_address(0);

    // register invalidate method
    socket.register_invalidate_direct_mem_ptr(this, &SimpleLTInitiator2_dmi::invalidate_direct_mem_ptr);

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

  std::pair<dmi_type, bool>& getDMIData(const transaction_type& trans)
  {
    if (trans.get_command() == tlm::TLM_READ_COMMAND) {
      return mDMIDataReads;

    } else { // WRITE
      return mDMIDataWrites;
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

      ///////////////////////////////////////////////////////////
      // DMI handling:
      // We do *not* use the DMI hint to check if it makes sense to ask for
      // DMI pointers. So the pattern is:
      // - if the address is not covered by a DMI region try to acquire DMI
      //   pointers
      // - if we have a DMI pointer, do the DMI "transaction"
      // - otherwise fall back to a normal transaction
      ///////////////////////////////////////////////////////////

      std::pair<dmi_type, bool>& dmi_data = getDMIData(trans);

      // Check if we need to acquire a DMI pointer
      if((trans.get_address() < dmi_data.first.get_start_address()) ||
         (trans.get_address() > dmi_data.first.get_end_address()) )
      {
          sc_dt::uint64 address = trans.get_address(); //save original address
          dmi_data.second =
            socket->get_direct_mem_ptr(trans,
                                       dmi_data.first);
          trans.set_address(address);
      }
      // Do DMI "transaction" if we have a valid region
      if (dmi_data.second &&
          (trans.get_address() >= dmi_data.first.get_start_address()) &&
          (trans.get_address() <= dmi_data.first.get_end_address()) )
      {
          // We can handle the data here. As the logEndTransaction is assuming
          // something to happen in the data structure, we really need to
          // do this:
          trans.set_response_status(tlm::TLM_OK_RESPONSE);
          sc_dt::uint64 tmp = trans.get_address() - dmi_data.first.get_start_address();
          if (trans.get_command() == tlm::TLM_WRITE_COMMAND)
          {
              *(unsigned int*)&dmi_data.first.get_dmi_ptr()[tmp] = mData;
          }
          else
          {
              mData = *(unsigned int*)&dmi_data.first.get_dmi_ptr()[tmp];
          }
          
          // Do the wait immediately. Note that doing the wait here eats almost
          // all the performance anyway, so we only gain something if we're
          // using temporal decoupling.
          if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
            wait(dmi_data.first.get_write_latency());

          } else {
            wait(dmi_data.first.get_read_latency());
          }
      }
      else // we need a full transaction
      {
          socket->b_transport(trans, t);
          wait(t);
      }
      logEndTransaction(trans);
    }
    wait();

  }

  // Invalidate DMI pointer(s)
  void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                 sc_dt::uint64 end_range)
  {
    // FIXME: probably faster to always invalidate everything?
    if (start_range <= mDMIDataReads.first.get_end_address ()&&
        end_range >= mDMIDataReads.first.get_start_address()) {
        mDMIDataReads.second = false;
    }
    if (start_range <= mDMIDataWrites.first.get_end_address ()&&
        end_range >= mDMIDataWrites.first.get_start_address()) {
      mDMIDataWrites.second = false;
    }
  }

  // Test for transport_dbg, this one should fail in bus_dmi as we address
  // a target that doesn't support transport_dbg:
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
        std::cout << "OK: debug transaction didn't give data." << std::endl;
    }
    std::cout << std::dec << std::endl;  
  }
private:
  std::pair<dmi_type, bool> mDMIDataReads;
  std::pair<dmi_type, bool> mDMIDataWrites;

  sc_core::sc_event mEndEvent;
  unsigned int mNrOfTransactions;
  unsigned int mBaseAddress;
  unsigned int mTransactionCount;
  unsigned int mData;
};

#endif
