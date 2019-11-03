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

#ifndef __SIMPLE_AT_INITIATOR2_H__
#define __SIMPLE_AT_INITIATOR2_H__

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
//#include <systemc>
#include <cassert>
#include <queue>
//#include <iostream>

class SimpleATInitiator2 : public sc_core::sc_module
{
public:
  typedef tlm::tlm_generic_payload                   transaction_type;
  typedef tlm::tlm_phase                             phase_type;
  typedef tlm::tlm_sync_enum                         sync_enum_type;
  typedef tlm_utils::simple_initiator_socket<SimpleATInitiator2>  initiator_socket_type;

public:
  // extended transaction, holds tlm_generic_payload + data storage
  template <typename DT>
  class MyTransaction : public transaction_type
  {
  public:
    MyTransaction()
    {
      this->set_data_ptr(reinterpret_cast<unsigned char*>(&mData));
    }
    MyTransaction(tlm::tlm_mm_interface* mm) : transaction_type(mm)
    {
      this->set_data_ptr(reinterpret_cast<unsigned char*>(&mData));
    }

    void setData(DT& data) { mData = data; }
    DT getData() const { return mData; }

  private:
    DT mData;
  };
  typedef MyTransaction<unsigned int>  mytransaction_type;

  // Dummy Transaction Pool
  class SimplePool : public tlm::tlm_mm_interface
  {
  public:
    SimplePool() {}
    mytransaction_type* claim()
    { 
      mytransaction_type* t = new mytransaction_type(this);
      t->acquire();
      return t;
    }
    void release(mytransaction_type* t)
    {
      t->release();
    }
    void free(tlm::tlm_generic_payload* t)
    {
      t->reset(); 
      delete t;
    }
  };

public:
  initiator_socket_type socket;

public:
  SC_HAS_PROCESS(SimpleATInitiator2);
  SimpleATInitiator2(sc_core::sc_module_name name,
                     unsigned int nrOfTransactions = 0x5,
                     unsigned int baseAddress = 0) :
    sc_core::sc_module(name),
    socket("socket"),
    ACCEPT_DELAY(10, sc_core::SC_NS),
    mNrOfTransactions(nrOfTransactions),
    mBaseAddress(baseAddress),
    mTransactionCount(0),
    mCurrentTransaction(0)
  {
    // register nb_transport method
    socket.register_nb_transport_bw(this, &SimpleATInitiator2::myNBTransport);

    // Initiator thread
    SC_THREAD(run);
  }

  bool initTransaction(mytransaction_type*& trans)
  {
    if (mTransactionCount < mNrOfTransactions) {
      trans = transPool.claim();
      trans->set_address(mBaseAddress + 4*mTransactionCount);
      trans->setData(mTransactionCount);
      trans->set_command(tlm::TLM_WRITE_COMMAND);

    } else if (mTransactionCount < 2 * mNrOfTransactions) {
      trans = transPool.claim();
      trans->set_address(mBaseAddress + 4*(mTransactionCount - mNrOfTransactions));
      trans->set_command(tlm::TLM_READ_COMMAND);

    } else {
      return false;
    }

    trans->set_data_length(4);
    trans->set_streaming_width(4);

    ++mTransactionCount;
    return true;
  }

  void logStartTransation(mytransaction_type& trans)
  {
    if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
      std::cout << name() << ": Send write request: A = 0x"
                << std::hex << (unsigned int)trans.get_address()
                << ", D = 0x" << trans.getData() << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;
      
    } else {
      std::cout << name() << ": Send read request: A = 0x"
                << std::hex << (unsigned int)trans.get_address() << std::dec
                << " @ " << sc_core::sc_time_stamp() << std::endl;
    }
  }

  void logEndTransaction(mytransaction_type& trans)
  {
    if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
      std::cout << name() << ": Received error response @ "
                << sc_core::sc_time_stamp() << std::endl;

    } else {
      std::cout << name() <<  ": Received ok response";
      if (trans.get_command() == tlm::TLM_READ_COMMAND) {
        std::cout << ": D = 0x" << std::hex << trans.getData() << std::dec;
      }
      std::cout << " @ " << sc_core::sc_time_stamp() << std::endl;
    }
  }

  //
  // Simple AT Initiator
  // - Request must be accepted by the target before the next request can be
  //   send
  // - Responses can come out of order
  // - Responses will be accepted after fixed delay
  //
  void run()
  {
    phase_type phase;
    sc_core::sc_time t;
    
    mytransaction_type* ptrans;
    while (initTransaction(ptrans)) {
      // Create transaction and initialise phase and t
      mytransaction_type& trans = *ptrans;
      phase = tlm::BEGIN_REQ;
      t = sc_core::SC_ZERO_TIME;

      logStartTransation(trans);

      switch (socket->nb_transport_fw(trans, phase, t)) {
      case tlm::TLM_COMPLETED:
        // Transaction Finished, wait for the returned delay
        wait(t);
        logEndTransaction(trans);
        transPool.release(&trans);
        break;

      case tlm::TLM_ACCEPTED:
      case tlm::TLM_UPDATED:
        switch (phase) {
        case tlm::BEGIN_REQ:
          // Request phase not yet finished
          // Wait until end of request phase before sending new request
          
          // FIXME
          mCurrentTransaction = &trans;
          wait(mEndRequestPhase);
	  mCurrentTransaction = 0;
          break;

        case tlm::END_REQ:
          // Request phase ended
          if (t != sc_core::SC_ZERO_TIME) {
            // Wait until end of request time before sending new request
            wait(t);
          }
          break;

        case tlm::BEGIN_RESP:
          // Request phase ended and response phase already started
          if (t != sc_core::SC_ZERO_TIME) {
            // Wait until end of request time before sending new request
            wait(t);
          }
          // Notify end of response phase after ACCEPT delay
          t += ACCEPT_DELAY;
          phase = tlm::END_RESP;
          socket->nb_transport_fw(trans, phase, t);
          logEndTransaction(trans);
          transPool.release(&trans);
          break;

        case tlm::END_RESP:   // fall-through
        default:
          // A target should never return with these phases
          // If phase == END_RESP, nb_transport should have returned true
          assert(0); exit(1);
          break;
        }
        break;

      default:
        assert(0); exit(1);
      };
    }
    wait();

  }

  sync_enum_type myNBTransport(transaction_type& trans, phase_type& phase, sc_core::sc_time& t)
  {
    switch (phase) {
    case tlm::END_REQ:
      assert(t == sc_core::SC_ZERO_TIME); // FIXME: can t != 0?
      // Request phase ended
      mEndRequestPhase.notify(sc_core::SC_ZERO_TIME);
      return tlm::TLM_ACCEPTED;

    case tlm::BEGIN_RESP:
    {
      assert(t == sc_core::SC_ZERO_TIME); // FIXME: can t != 0?

      // Notify end of request phase if run thread is waiting for it
      // FIXME
      if (&trans == mCurrentTransaction) {
        mEndRequestPhase.notify(sc_core::SC_ZERO_TIME);
      }

      assert(dynamic_cast<mytransaction_type*>(&trans));
      mytransaction_type* myTrans = static_cast<mytransaction_type*>(&trans);
      assert(myTrans);

      // Notify end of response phase after ACCEPT delay
      t += ACCEPT_DELAY;
      phase = tlm::END_RESP;
      logEndTransaction(*myTrans);
      transPool.release(myTrans);

      return tlm::TLM_COMPLETED;
    }

    case tlm::BEGIN_REQ: // fall-through
    case tlm::END_RESP:  // fall-through
    default:
      // A target should never call nb_transport with these phases
      assert(0); exit(1);
//      return tlm::TLM_COMPLETED;  //unreachable code
    };
  }

private:
  const sc_core::sc_time ACCEPT_DELAY;

private:
  unsigned int mNrOfTransactions;
  unsigned int mBaseAddress;
  SimplePool transPool;
  unsigned int mTransactionCount;
  sc_core::sc_event mEndRequestPhase;
  transaction_type* mCurrentTransaction;
};

#endif
