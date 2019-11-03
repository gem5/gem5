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

// old_event_bug.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: old_event_bug.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Longstanding bug when checking for events in signals

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  : counti(0)
  , countb(0)
  , reached_the_end(false)
  {
    SC_THREAD(T);

    SC_METHOD(MI);
      sensitive << sigi;
      dont_initialize();

    SC_METHOD(MB);
      sensitive << sigb;
      dont_initialize();
    
    sigi.write(0);
    sigb.write(false);
    sigi_dummy.write(0);
    sigb_dummy.write(false);
  }
  
  int counti;
  int countb;
  bool reached_the_end;
  
  sc_signal<int>  sigi;
  sc_signal<bool> sigb;
  
  sc_signal<int>  sigi_dummy;
  sc_signal<bool> sigb_dummy;
  
  void T()
  {
    sc_assert( sigi.event() == false );
    sc_assert( sigb.event() == false );
    sc_assert( sigb.posedge() == false );
    sc_assert( sigb.negedge() == false );
    
    sigi.write(1);
    wait(sigi.value_changed_event());
    sc_assert( sigi.event() );

    sigb.write(true);
    wait(sigb.value_changed_event());
    sc_assert( sigb.event() );
    sc_assert( sigb.posedge() );
    sc_assert( sigb.negedge() == false );

    wait(1, SC_NS);

    sc_assert( sigi.event() == false );
    sc_assert( sigb.event() == false );

    sigi.write(2);
    sigb.write(false);
    
    wait(1, SC_NS);

    sc_assert( sigi.event() == false );
    sc_assert( sigb.event() == false );
    sc_assert( sigb.posedge() == false );
    sc_assert( sigb.negedge() == false );
    
    sigi_dummy.write(1);
    sigb_dummy.write(true);
   
    wait(1, SC_NS);
    
    sc_assert( sigi_dummy.event() == false );
    sc_assert( sigb_dummy.event() == false );
    sc_assert( sigb_dummy.posedge() == false );
    sc_assert( sigb_dummy.negedge() == false );
      
    reached_the_end = true;  
  }
  
  void MI()
  {
    sc_assert( sigi.event() );
    ++counti;
  }
  
  void MB()
  {
    sc_assert( sigb.event() );
    ++countb;
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();
  
  sc_assert( top.counti == 2 );
  sc_assert( top.countb == 2 );
  sc_assert( top.reached_the_end );

  cout << endl << "Success" << endl;
  return 0;
}
  
