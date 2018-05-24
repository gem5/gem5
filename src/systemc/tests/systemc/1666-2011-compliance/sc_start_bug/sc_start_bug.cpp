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

// sc_start_bug.cpp
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    SC_THREAD(T1);
    SC_THREAD(T2);
  }
  
  void T1()
  {
    sc_assert( sc_time_stamp() == sc_time(0, SC_NS) );
    wait(15, SC_NS);
    sc_assert( sc_time_stamp() == sc_time(15, SC_NS) );
    sc_pause();
  }

  void T2()
  {
    sc_assert( sc_time_stamp() == sc_time(0, SC_NS) );
    wait(30, SC_NS);
    sc_assert( sc_time_stamp() == sc_time(30, SC_NS) );
    sc_pause();
  }

  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );

  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(15, SC_NS) );
  
  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(25, SC_NS) );
  
  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(30, SC_NS) );
  
  cout << endl << "Success" << endl;
  return 0;
}
