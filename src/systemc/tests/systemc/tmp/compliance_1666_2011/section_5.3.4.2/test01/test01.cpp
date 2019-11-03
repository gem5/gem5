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

// test01.cpp -- sc_start With Event Starvation Policy
//
//  Original Author: John Aynsley, Doulos
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: test01.cpp,v $
// Revision 1.1  2011/02/05 21:14:24  acg
//  Andy Goodrich: moving tests John Aynsley will replace.
//
// Revision 1.2  2011/01/25 20:54:03  acg
//  Andy Goodrich: regolden for new delta counter rules.
//
// Revision 1.1  2011/01/24 12:06:06  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//

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

  sc_event ev;

  void T()
  {
    ev.notify(150, SC_NS);
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");

  sc_assert( sc_get_status() == SC_ELABORATION );
  sc_assert( sc_time_stamp() == SC_ZERO_TIME );
  sc_start(100, SC_NS);
  sc_assert( sc_time_stamp() == sc_time(100, SC_NS) );

  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(10, SC_NS, SC_EXIT_ON_STARVATION);
  // sc_assert( sc_time_stamp() == sc_time(120, SC_NS) );
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(80, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );  

  sc_start();
  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );
  sc_assert( sc_get_status() == SC_PAUSED );

  cout << endl << "Success" << endl;
  return 0;
}
