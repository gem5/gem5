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

// sc_delta_count.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: sc_delta_count.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// sc_delta_count, including notifications across pauses

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    
    SC_THREAD(T);

    SC_METHOD(M);
      sensitive << ev << ev2 << ev3;
      dont_initialize();
      m = sc_get_current_process_handle();
      
    sc_assert( sc_delta_count() == 0 );
    count = 0;
    reached_end = false;
  }
  
  void start_of_simulation()
  {
    sc_assert( sc_delta_count() == 0 );
  }

  sc_process_handle m;  
  sc_event ev, ev2, ev3;
  int count;
  sc_time last_time;
  bool reached_end;
  
  void T()
  {
    sc_assert( sc_delta_count() == 0 );
    
    count = 1;
    ev.notify();
    wait(SC_ZERO_TIME);
    sc_assert( sc_delta_count() == 1 );
    
    count = 2;
    ev.notify();
    wait(SC_ZERO_TIME);
    sc_assert( sc_delta_count() == 2 );
    
    count = 3;
    ev.notify(SC_ZERO_TIME);
    wait(SC_ZERO_TIME);
    sc_assert( sc_delta_count() == 3 );
    
    count = 4;
    ev.notify(1, SC_NS);
    wait(SC_ZERO_TIME);
    sc_assert( sc_delta_count() == 4 );

    count = 5;
    wait(1, SC_NS);
    ev.notify(1, SC_NS);
    sc_assert( sc_delta_count() == 5 );
    
    count = 6;
    wait(2, SC_NS);
    sc_assert( sc_delta_count() == 7 );

    count = 7;
    m.disable();
    ev.notify(1, SC_NS);
    wait(2, SC_NS);
    sc_assert( sc_delta_count() == 8 );

    count = 8;
    m.enable();
    ev .notify();
    ev2.notify(1, SC_NS);
    ev3.notify(2, SC_NS);
    wait(3, SC_NS);
    sc_assert( sc_delta_count() == 11 );

    count = 9;
    last_time = sc_time_stamp();
    sc_pause(); // 1st pause
    wait(ev);
    sc_assert( sc_delta_count() == 12 );

    count = 10;
    last_time = sc_time_stamp();
    sc_pause(); // 2nd pause
    wait(ev);
    sc_assert( sc_delta_count() == 13 );

    count = 11;
    last_time = sc_time_stamp();
    sc_pause(); // 3rd pause
    wait(ev);
    sc_assert( sc_delta_count() == 14 );

    count = 12;
    last_time = sc_time_stamp();
    sc_pause(); // 4th pause
    wait(ev);
    sc_assert( sc_delta_count() == 15 );
    
    count = 13;
    last_time = sc_time_stamp();
    sc_pause(); // 5th pause
    wait(ev);
    sc_assert( sc_delta_count() == 16 );
    
     count = 14;
     wait(ev);
     sc_assert( sc_delta_count() == 17 );
 
    last_time = sc_time_stamp();
    reached_end = true;
    sc_stop();
  }
  
  void M()
  {
    cout << "M() awoke at " << sc_time_stamp() << endl;
    switch (count)
    {
      case  0: sc_assert( false ); break;
      case  1: sc_assert( sc_delta_count() == 0 ); break;
      case  2: sc_assert( sc_delta_count() == 1 ); break;
      case  3: sc_assert( sc_delta_count() == 3 ); break;
      case  4: sc_assert( false ); break;
      case  5: sc_assert( sc_delta_count() == 5 ); break;
      case  6: sc_assert( sc_delta_count() == 6 ); break;
      case  7: sc_assert( false ); break;
      case  8: sc_assert( sc_delta_count() == 8 ||
                         sc_delta_count() == 9 ||
                         sc_delta_count() == 10 ); break;
      case  9: sc_assert( sc_delta_count() == 12 ); break;
      case 10: sc_assert( sc_delta_count() == 13 ); break;
      case 11: sc_assert( sc_delta_count() == 14 ); break;
      case 12: sc_assert( sc_delta_count() == 15 ); break;
      case 13: sc_assert( sc_delta_count() == 16 ); break;
      case 14: sc_assert( sc_delta_count() == 17 ); break;
    }
  }
    
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  sc_assert( sc_delta_count() == 0 );

  Top top("top");
  
  sc_start();
  
  sc_assert( sc_delta_count() == 12 );
  sc_assert( sc_get_status() == SC_PAUSED );
  top.ev.notify();  // Wake from 1st pause on immed notification
  
  sc_start();
  sc_assert( sc_delta_count() == 13 );
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_pending_activity_at_current_time() == false );
  sc_assert( sc_pending_activity_at_future_time() == false );

  sc_start(SC_ZERO_TIME);
  sc_assert( sc_delta_count() == 13 );
  
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_start(SC_ZERO_TIME);
  sc_assert( sc_delta_count() == 13 );
  sc_assert( top.last_time == sc_time_stamp() );
  
  sc_start(1, SC_NS);
  sc_assert( sc_delta_count() == 13 );
  sc_assert( top.count == 10 );
  // sc_assert( top.last_time == sc_time_stamp() );

  top.ev.notify(SC_ZERO_TIME); // Wake from 2nd pause on delta notification
  sc_start(1, SC_NS);
  sc_assert( sc_delta_count() == 14 );
  sc_assert( top.last_time == sc_time_stamp() );
  
  top.ev.notify(1, SC_NS);   // Wake from 3rd pause on timed notification
  sc_start(2, SC_NS);
  sc_assert( sc_delta_count() == 15 );
  sc_assert( top.last_time == sc_time_stamp() );
  
  top.ev.notify(2, SC_NS);   // Future notification beyond the subsequent start
  sc_assert( sc_pending_activity_at_current_time() == false );
  sc_assert( sc_pending_activity_at_future_time() == true );
  sc_start(1, SC_NS);
  sc_assert( sc_delta_count() == 15 );

  sc_start();
  sc_assert( sc_delta_count() == 16 );
  sc_assert( top.last_time == sc_time_stamp() );

  top.ev.notify();  // Wake from 5th pause on immed notification
  sc_start(SC_ZERO_TIME);
  sc_assert( sc_delta_count() == 17 );
  sc_assert( top.last_time == sc_time_stamp() );
  
  sc_assert( sc_get_status() == SC_PAUSED );

  top.ev.notify(1, SC_NS);   // Future notification before calling sc_stop
  sc_start(2, SC_NS);

  sc_assert( sc_get_status() == SC_STOPPED );
  sc_assert( sc_end_of_simulation_invoked() );
  sc_assert( top.last_time == sc_time_stamp() );
  sc_assert( top.reached_end );

  cout << endl << "Success" << endl;
  return 0;
}
