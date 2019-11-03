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

// sc_start_starvation.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: sc_start_starvation.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// sc_start with event starvation policy
// sc_pending_activity and friends

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

SC_MODULE(Top)
{
  SC_CTOR(Top)
  {
    SC_THREAD(T);
  }
  
  sc_event ev2;
  
  void T()
  {
    sc_assert( sc_get_status() == SC_RUNNING );

    sc_assert( sc_pending_activity_at_current_time() == false );    
    sc_assert( sc_pending_activity_at_future_time() == true );
    sc_assert( sc_time_to_pending_activity() == sc_time(250, SC_NS) );
    
    ev2.notify(150, SC_NS);
    
    sc_assert( sc_pending_activity_at_current_time() == false );    
    sc_assert( sc_pending_activity_at_future_time() == true );
    sc_assert( sc_time_to_pending_activity() == sc_time(150, SC_NS) );
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");

  sc_event ev;
  ev.notify(250, SC_NS);

  sc_assert( sc_pending_activity_at_current_time() == false );    
  sc_assert( sc_pending_activity_at_future_time() == true );
  sc_assert( sc_pending_activity() == true );
  sc_assert( sc_time_to_pending_activity() == sc_time(250, SC_NS) );
    
  sc_assert( sc_get_status() == SC_ELABORATION );
  sc_assert( sc_time_stamp() == SC_ZERO_TIME );
  sc_start(100, SC_NS);
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_time_stamp() == sc_time(100, SC_NS) );

  sc_assert( sc_pending_activity_at_current_time() == false );    
  sc_assert( sc_pending_activity_at_future_time() == true );
  sc_assert( sc_time_to_pending_activity() == sc_time(50, SC_NS) );
    
  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );
  sc_assert( sc_time_to_pending_activity() == sc_time(40, SC_NS) );

  sc_start(10, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(80, SC_NS, SC_EXIT_ON_STARVATION);

  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );
  sc_assert( sc_pending_activity_at_current_time() == false );    
  sc_assert( sc_pending_activity_at_future_time() == true );
  sc_assert( sc_time_to_pending_activity() == sc_time(100, SC_NS) );

  sc_start(50, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );
  sc_assert( sc_time_to_pending_activity() == sc_time(100, SC_NS) );

  sc_start(50, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(200, SC_NS) );
  sc_assert( sc_time_to_pending_activity() == sc_time(50, SC_NS) );

  sc_start();
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_time_stamp() == sc_time(250, SC_NS) );
  
  sc_assert( sc_pending_activity() == false );    
  sc_assert( sc_time_to_pending_activity() == sc_max_time() - sc_time_stamp() );
  
  ev.notify(SC_ZERO_TIME);

  sc_assert( sc_pending_activity_at_current_time() == true );    
  sc_assert( sc_pending_activity_at_future_time() == false );
  sc_assert( sc_time_to_pending_activity() == SC_ZERO_TIME );

  sc_start();
  sc_assert( sc_time_stamp() == sc_time(250, SC_NS) );

  sc_assert( sc_pending_activity() == false );    
  sc_assert( sc_time_to_pending_activity() == sc_max_time() - sc_time_stamp() );
  
  ev.notify(10, SC_NS);

  sc_assert( sc_pending_activity() == true );    
  sc_assert( sc_time_to_pending_activity() == sc_time(10, SC_NS) );
  
  sc_start();
  sc_assert( sc_time_stamp() == sc_time(260, SC_NS) );

  ev.notify(10, SC_NS);
  sc_start(sc_time(100, SC_NS), SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(270, SC_NS) );

  ev.notify(10, SC_NS);
  sc_start(sc_time(100, SC_NS)); // SC_RUN_TO_TIME
  sc_assert( sc_time_stamp() == sc_time(370, SC_NS) );
  sc_assert( sc_get_status() == SC_PAUSED );
  
  ev.notify();
  sc_assert( sc_pending_activity() == false );    
  
  cout << endl << "Success" << endl;
  return 0;
}
