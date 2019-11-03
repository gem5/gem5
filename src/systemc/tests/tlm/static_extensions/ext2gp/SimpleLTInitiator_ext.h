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

#ifndef __SIMPLE_LT_INITIATOR_EXT_H__
#define __SIMPLE_LT_INITIATOR_EXT_H__

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "my_extension.h"

#include <systemc>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <map>

class SimpleLTInitiator_ext : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload transaction_type;
  typedef tlm::tlm_dmi        dmi_type;
  typedef tlm::tlm_phase      phase_type;
  typedef tlm::tlm_sync_enum  sync_enum_type;
  typedef tlm_utils::simple_initiator_socket<SimpleLTInitiator_ext, 32,
                                my_extended_payload_types> initiator_socket_type;

public:
  initiator_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleLTInitiator_ext);
  SimpleLTInitiator_ext(sc_core::sc_module_name name,
                        unsigned int nrOfTransactions = 0x5,
                        unsigned int baseAddress = 0x0) :
      sc_core::sc_module(name),
      socket("socket"),
      mNrOfTransactions(nrOfTransactions),
      mBaseAddress(baseAddress),
      mTransactionCount(0)
  {
      invalidate(mDMIData);
      
      // register nb_transport method
      socket.register_nb_transport_bw(this, &SimpleLTInitiator_ext::myNBTransport);
      socket.register_invalidate_direct_mem_ptr(this, &SimpleLTInitiator_ext::invalidate_direct_mem_ptr);
      
      // Initiator thread
      SC_THREAD(run);
      
  }

  bool initTransaction(transaction_type& trans)
  {
      // initialize DMI hint:
      trans.set_dmi_allowed(false);
      
      if (mTransactionCount < mNrOfTransactions)
      {
          trans.set_address(mBaseAddress + 4*mTransactionCount);
          mData = mTransactionCount;
          trans.set_data_ptr(reinterpret_cast<unsigned char*>(&mData));
          trans.set_command(tlm::TLM_WRITE_COMMAND);
          
      }
      else if (mTransactionCount < 2 * mNrOfTransactions)
      {
          trans.set_address(mBaseAddress + 4*(mTransactionCount-mNrOfTransactions));
          mData = 0;
          trans.set_data_ptr(reinterpret_cast<unsigned char*>(&mData));
          trans.set_command(tlm::TLM_READ_COMMAND);
          
      }
      else
      {
          return false;
      }
      
      ++mTransactionCount;
      return true;
  }
  
  void logStartTransation(transaction_type& trans)
  {
      if (trans.get_command() == tlm::TLM_WRITE_COMMAND)
      {
          std::cout << name() << ": Send write request: A = 0x"
                    << std::hex << (unsigned int)trans.get_address()
                    << ", D = 0x" << mData << std::dec
                    << " @ " << sc_core::sc_time_stamp() << std::endl;
          
      }
      else
      {
          std::cout << name() << ": Send read request: A = 0x"
                    << std::hex << (unsigned int)trans.get_address()
                    << std::dec
                    << " @ " << sc_core::sc_time_stamp() << std::endl;
      }
  }

  void logEndTransaction(transaction_type& trans)
  {
      if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
          std::cout << name() << ": Received error response @ "
                    << sc_core::sc_time_stamp() << std::endl;
      }
      else
      {
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
      // make sure that our transaction has the proper extension:
      my_extension* tmp_ext = new my_extension();
      tmp_ext->m_data = 11;
      
      trans.set_extension(tmp_ext);
      
      while (initTransaction(trans))
      {
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
          //   -> check if we get a DMI hint and acquire the DMI pointers if it
          //      is set
          ///////////////////////////////////////////////////////////
          
          // Check if the address is covered by our DMI region
          if ( (trans.get_address() >= mDMIData.get_start_address()) &&
               (trans.get_address() <= mDMIData.get_end_address()) )
          {
              // We can handle the data here. As the logEndTransaction is
              // assuming something to happen in the data structure, we really
              // need to do this:
              trans.set_response_status(tlm::TLM_OK_RESPONSE);
              sc_dt::uint64 tmp = trans.get_address() - mDMIData.get_start_address();
              if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
                  *(unsigned int*)&mDMIData.get_dmi_ptr()[tmp] = mData;
                  
              } else {
                  mData = *(unsigned int*)&mDMIData.get_dmi_ptr()[tmp];
              }
              
              // Do the wait immediately. Note that doing the wait here eats
              //  almost all the performance anyway, so we only gain something
              // if we're using temporal decoupling.
              if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
                  wait(mDMIData.get_write_latency());
                  
              } else {
                  wait(mDMIData.get_read_latency());
              }

              logEndTransaction(trans);
              
          } else { // we need a full transaction
              switch (socket->nb_transport_fw(trans, phase, t)) {
              case tlm::TLM_COMPLETED:
                  // Transaction Finished, wait for the returned delay
                  wait(t);
                  break;
                  
              case tlm::TLM_ACCEPTED:
              case tlm::TLM_UPDATED:
                  // Transaction not yet finished, wait for the end of it
                  wait(mEndEvent);
                  break;
                  
              default:
                  sc_assert(0); exit(1);
              };
    
              logEndTransaction(trans);

              // Acquire DMI pointer if one is available:
              if (trans.is_dmi_allowed())
              {
                  trans.set_write();
                  dmi_type tmp;
                  if (socket->get_direct_mem_ptr(trans,
                                                 tmp))
                  {
                      // FIXME: No support for separate read/write ranges
                      sc_assert(tmp.is_read_write_allowed());
                      mDMIData = tmp;
                  }
              }
          }
      }
      delete tmp_ext;
      wait();
      
  }
  
  sync_enum_type myNBTransport(transaction_type& trans,
                               phase_type& phase,
                               sc_core::sc_time& t)
  {
      switch (phase) {
      case tlm::END_REQ:
          // Request phase ended
          return tlm::TLM_ACCEPTED;
          
      case tlm::BEGIN_RESP:
          sc_assert(t == sc_core::SC_ZERO_TIME); // FIXME: can t != 0?
          mEndEvent.notify(t);
          // Not needed to update the phase if true is returned
          return tlm::TLM_COMPLETED;
          
      case tlm::BEGIN_REQ: // fall-through
      case tlm::END_RESP: // fall-through
      default:
          // A target should never call nb_transport with these phases
          sc_assert(0); exit(1);
//          return tlm::TLM_COMPLETED;  //unreachable code
      };
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
  dmi_type mDMIData;
  
  sc_core::sc_event mEndEvent;
  unsigned int mNrOfTransactions;
  unsigned int mBaseAddress;
  unsigned int mTransactionCount;
  unsigned int mData;
};

#endif
