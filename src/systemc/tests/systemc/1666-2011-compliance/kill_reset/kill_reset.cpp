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

// kill_reset.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: kill_reset.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Reset and kill a thread process, including nested kills and resets

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M3: sc_module
{
  M3(sc_module_name _name)
  {
    SC_THREAD(ticker);
      k = sc_get_current_process_handle();
      
    SC_THREAD(calling);
    
    SC_THREAD(target);
      t = sc_get_current_process_handle();
      
    SC_THREAD(victim);
      v = sc_get_current_process_handle();
      
    SC_THREAD(bystander);
      b = sc_get_current_process_handle();
      
    SC_THREAD(second_bystander);
      b2 = sc_get_current_process_handle();
      
    SC_THREAD(third_bystander);
      b3 = sc_get_current_process_handle();
      
    killing_over = false;
    third_bystander_knocked_over = false;
  }
  
  sc_process_handle t, k, v, b, b2, b3;
  sc_event ev;
  int count;
  bool killing_over;
  bool third_bystander_knocked_over;

  void ticker()
  {
    for (;;)
    {
      try {
        wait(10, SC_NS);
        ev.notify();
        sc_assert( !sc_is_unwinding() );
      }
      catch (const sc_unwind_exception& ex)
      {
        // ticker process killed by target
        cout << "sc_unwind_exception caught by ticker" << endl;
        sc_assert( !ex.is_reset() );
        sc_assert( count == 1 );
        sc_assert( !killing_over );
        sc_assert( t.is_unwinding() );
        sc_assert( sc_is_unwinding() );
       
        v.kill();
        throw ex;
      }
    }
  }
   
  void calling()
  {
    wait(15, SC_NS);
    // Target runs at time 10 NS due to notification
    sc_assert( count == 1 );
    // The victim awakes every 1ns
    sc_assert( sc_time_to_pending_activity() <= sc_time(1, SC_NS) );
 
    wait(10, SC_NS);
    // Target runs again at time 20 NS due to notification
    sc_assert( count == 2 );
    
    t.reset();
    // Target reset immediately at time 25 NS
    sc_assert( count == 0 );
 
    wait(10, SC_NS);
    // Target runs again at time 30 NS due to notification
    sc_assert( count == 1 );
    
    t.kill();
    sc_assert( !killing_over );
    killing_over = true;
    
    // Target killed immediately at time 35 NS
    if (t.valid())
      sc_assert( t.terminated() );
    if (k.valid())
      sc_assert( k.terminated() );
    if (v.valid())
      sc_assert( v.terminated() );
    sc_assert( b.valid() && !b.terminated() );
    sc_assert( b2.valid() && !b2.terminated() );
    if (b3.valid())
      sc_assert( b3.terminated() );
      
    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;
    count = 0;
    for (;;)
    {
      try {
        wait(ev);
        cout << "Target awoke at " << sc_time_stamp() << endl;
        ++count;
      }
      catch (const sc_unwind_exception& ex)
      {
        cout << "sc_unwind_exception caught by target" << endl;
        if (count == 2)
          sc_assert( ex.is_reset() );
        else if (count == 1)
        {
          sc_assert( !ex.is_reset() );
          sc_assert( !killing_over );
          k.kill();
        }
        else
          sc_assert( false );
        throw ex;
      }
    }
  }
  
  void victim()
  {
    try {
      while (true)
      {
        wait(1, SC_NS);
        sc_assert( !sc_is_unwinding() );
      }
    }
    catch (const sc_unwind_exception& ex)
    {
      cout << "sc_unwind_exception caught by victim" << endl;
      sc_assert( sc_time_stamp() == sc_time(35, SC_NS) );
      sc_assert( ex.is_reset() == false );
      sc_assert( !killing_over );
      sc_assert( v.is_unwinding() );
      sc_assert( sc_is_unwinding() );
      
      b.reset();
      throw ex;
    }
  }

  void bystander() // Gets reset by victim
  {
    for (;;)
    {
      try {
        wait(ev);
      }
      catch (const sc_unwind_exception& ex) {
        cout << "sc_unwind_exception caught by bystander" << endl;
        sc_assert( sc_time_stamp() == sc_time(35, SC_NS) );
        sc_assert( ex.is_reset() == true );
        sc_assert( !killing_over );
        sc_assert( v.is_unwinding() ); // sic
        sc_assert( sc_is_unwinding() );
        
        b2.reset();
        throw ex;
      }
    }
  }
    
  void second_bystander() // Gets reset by bystander
  {
    for (;;)
    {
      try {
        wait(ev);
      }
      catch (const sc_unwind_exception& ex) {
        cout << "sc_unwind_exception caught by second_bystander" << endl;
        sc_assert( sc_time_stamp() == sc_time(35, SC_NS) );
        sc_assert( ex.is_reset() == true );
        sc_assert( !killing_over );
        sc_assert( v.is_unwinding() ); // sic
        sc_assert( b.is_unwinding() ); // sic
        sc_assert( sc_is_unwinding() );
        
        b3.kill();
        throw ex;
      }
    }
  }
    
  void third_bystander() // Gets killed by second bystander
  {
    for (;;)
    {
      try {
        wait(ev);
      }
      catch (const sc_unwind_exception& ex) {
        cout << "sc_unwind_exception caught by third_bystander" << endl;
        sc_assert( sc_time_stamp() == sc_time(35, SC_NS) );
        sc_assert( !ex.is_reset() == true );
        sc_assert( !killing_over );
        sc_assert( v.is_unwinding() ); // sic
        sc_assert( b.is_unwinding() ); // sic
        sc_assert( b2.is_unwinding() ); // sic
        sc_assert( sc_is_unwinding() );

        third_bystander_knocked_over = true;
        throw ex;
      }
    }
  }
    
  SC_HAS_PROCESS(M3);
};

int sc_main(int argc, char* argv[])
{
  M3 m("m");
  sc_assert( sc_pending_activity() == false );
  sc_assert( sc_time_to_pending_activity() == sc_max_time() );
  
  sc_start();
  sc_assert( m.third_bystander_knocked_over );  

  cout << endl << "Success" << endl;
  return 0;
}
  
