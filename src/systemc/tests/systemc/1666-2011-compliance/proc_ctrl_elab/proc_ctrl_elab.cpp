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

// proc_ctrl_elab.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: proc_ctrl_elab.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Calling process control methods during elaboration

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include "systemc.h"

SC_MODULE(Top)
{
  SC_CTOR(Top)
  : caught_throw_it(0)
  , caught_reset(0)
  , caught_kill(0)
  
  , target_suspend_1_called(false)
  , target_suspend_2_called(false)
  , target_suspend_3_called(false)
  , target_suspend_4_called(false)
  , target_suspend_5_called(false)

  , target_disable_1_called(false)
  , target_disable_2_called(false)
  , target_disable_2_called_again(false)
  , target_disable_3_called(false)
  , target_disable_4_called(false)
  , target_disable_5_called(false)

  , target_sync_reset_1_called(false)
  , target_sync_reset_2_called(false)
  , target_sync_reset_3_called(false)
  , target_sync_reset_4_called(false)

  , reset_1_count(0)
  , reset_2_count(0)
  , reset_3_count(0)
  , reset_4_count(0)
  {
    SC_THREAD(ctrl);

    SC_THREAD(target_suspend_1);
      sensitive << ev;
      ts1 = sc_get_current_process_handle();

    SC_THREAD(target_suspend_2);
      dont_initialize();
      sensitive << ev;
      ts2 = sc_get_current_process_handle();

    SC_THREAD(target_suspend_3);
      dont_initialize();
      sensitive << ev;
      ts3 = sc_get_current_process_handle();

    SC_THREAD(target_suspend_4);
      dont_initialize();
      sensitive << ev;
      ts4 = sc_get_current_process_handle();
      ts4.suspend();
      
    SC_THREAD(target_suspend_5);
      dont_initialize();
      sensitive << ev;
      ts5 = sc_get_current_process_handle();
      ts5.suspend();
      ts5.resume();
      
    SC_THREAD(target_disable_1);
      sensitive << dummy_ev;
      td1 = sc_get_current_process_handle();

    SC_THREAD(target_disable_2);
      sensitive << ev;
      td2 = sc_get_current_process_handle();

    SC_THREAD(target_disable_3);
      td3 = sc_get_current_process_handle();
      
    SC_THREAD(target_disable_4);
      dont_initialize();
      sensitive << ev;
      td4 = sc_get_current_process_handle();
      td4.disable();

    SC_THREAD(target_disable_5);
      dont_initialize();
      sensitive << ev;
      td5 = sc_get_current_process_handle();
      td5.disable();
      td5.enable();
      td5.sync_reset_on();
      td5.sync_reset_off();

    SC_THREAD(target_sync_reset_1);
      sensitive << ev;
      tr1 = sc_get_current_process_handle();

    SC_THREAD(target_sync_reset_2);
      sensitive << ev;
      tr2 = sc_get_current_process_handle();

    SC_THREAD(target_sync_reset_3);
      dont_initialize();
      sensitive << ev;
      tr3 = sc_get_current_process_handle();

    SC_THREAD(target_sync_reset_4);
      dont_initialize();
      sensitive << ev;
      tr4 = sc_get_current_process_handle();
      tr4.sync_reset_on();
      
    try {
      ts2.throw_it(ex);
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_throw_it;
    }
      
    try {
      ts2.reset();
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_reset;
    }
      
    try {
      ts2.kill();
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_kill;
    }
  }
  
  void before_end_of_elaboration()
  {
    ts1.suspend();
    ts2.suspend();
    ts3.suspend();

    td1.disable();
    td2.disable();
    td3.disable();
    
    tr1.sync_reset_on();
    tr2.sync_reset_on();
    tr3.sync_reset_on();
      
    try {
      ts2.throw_it(ex);
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_throw_it;
    }
      
    try {
      ts2.reset();
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_reset;
    }
      
    try {
      ts2.kill();
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_kill;
    }
  }

  void start_of_simulation()
  {
    td3.enable();
    tr3.sync_reset_off();
      
    try {
      ts2.throw_it(ex);
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_throw_it;
    }
      
    try {
      ts2.reset();
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_reset;
    }
      
    try {
      ts2.kill();
      sc_assert( false );
    }
    catch (std::exception e) {
      ++caught_kill;
    }
  }
  
  sc_event ev, dummy_ev;
  
  std::exception ex;  

  sc_process_handle ts1;
  sc_process_handle ts2;  
  sc_process_handle ts3;
  sc_process_handle ts4;
  sc_process_handle ts5;

  sc_process_handle td1;
  sc_process_handle td2;
  sc_process_handle td3;
  sc_process_handle td4;
  sc_process_handle td5;

  sc_process_handle tr1;
  sc_process_handle tr2;
  sc_process_handle tr3;
  sc_process_handle tr4;

  int caught_throw_it;
  int caught_reset;
  int caught_kill;

  bool target_suspend_1_called;
  bool target_suspend_2_called;
  bool target_suspend_3_called;
  bool target_suspend_4_called;
  bool target_suspend_5_called;

  bool target_disable_1_called;
  bool target_disable_2_called;
  bool target_disable_2_called_again;
  bool target_disable_3_called;
  bool target_disable_4_called;
  bool target_disable_5_called;

  bool target_sync_reset_1_called;
  bool target_sync_reset_2_called;
  bool target_sync_reset_3_called;
  bool target_sync_reset_4_called;
  
  int reset_1_count;
  int reset_2_count;
  int reset_3_count;
  int reset_4_count;
  
  void ctrl()
  {
    ts3.resume();
    
    wait(10, SC_NS);
    ts1.resume();
    ev.notify();
    
    wait(10, SC_NS);
    ts2.resume();
    tr2.sync_reset_off();
    tr4.sync_reset_off();
    
    wait(10, SC_NS);
    td2.enable();
    
    wait(10, SC_NS);
    ev.notify();
    
    wait(10, SC_NS);
    ev.notify();
    td2.disable();
    
    wait(10, SC_NS);
    td4.enable();
    ts4.resume();
    ev.notify();
  }
   
  void target_suspend_1()
  {
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
    target_suspend_1_called = true;
  }
   
  void target_suspend_2()
  {
    sc_assert( sc_time_stamp() == sc_time(20, SC_NS) );
    target_suspend_2_called = true;
  }
   
  void target_suspend_3()
  {
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
    target_suspend_3_called = true;
  }
   
  void target_suspend_4()
  {
    sc_assert( sc_time_stamp() == sc_time(60, SC_NS) );
    target_suspend_4_called = true;
  }
   
  void target_suspend_5()
  {
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
    target_suspend_5_called = true;
    ts5.suspend();
  }
   
  void target_disable_1()
  {
    sc_assert( false );
    target_disable_1_called = true;
  }
   
  void target_disable_2()
  {
    sc_assert( sc_time_stamp() == sc_time(40, SC_NS) );
    target_disable_2_called = true;
    wait(); // on ev

    sc_assert( sc_time_stamp() == sc_time(50, SC_NS) );
    target_disable_2_called_again = true;
  }
   
  void target_disable_3()
  {
    sc_assert( sc_time_stamp() == sc_time(0, SC_NS) );
    sc_assert( sc_delta_count() == 0 );
    target_disable_3_called = true;
  }
   
  void target_disable_4()
  {
    sc_assert( sc_time_stamp() == sc_time(60, SC_NS) );
    target_disable_4_called = true;
  }
   
  void target_disable_5()
  {
    sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
    target_disable_5_called = true;
    td5.disable();
  }
   
  void target_sync_reset_1()
  {
    switch (reset_1_count)
    {
      case 0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) );
              sc_assert( sc_delta_count() == 0 );
              break;
      case 1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); 
              break;
      case 2: sc_assert( sc_time_stamp() == sc_time(40, SC_NS) ); 
              break;
      case 3: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); 
              target_sync_reset_1_called = true;
              break;
    }
    ++reset_1_count;
    
    while (true)
    {
      wait();
      sc_assert( false );
    }
  }
   
  void target_sync_reset_2()
  {
    switch (reset_2_count)
    {
      case 0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) );
              sc_assert( sc_delta_count() == 0 );
              break;
      case 1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); 
              break;
      case 2: sc_assert( false ); 
              break;
      case 3: sc_assert( false ); 
              break;
    }
    ++reset_2_count;
    
    while (true)
    {
      wait();
      
    switch (reset_2_count)
    {
      case 0: sc_assert( false );
              break;
      case 1: sc_assert( false );
              break;
      case 2: sc_assert( sc_time_stamp() == sc_time(40, SC_NS) ); 
              break;
      case 3: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); 
              target_sync_reset_2_called = true;
              break;
    }
    ++reset_2_count;
    
    }
  }
   
  void target_sync_reset_3()
  {
    switch (reset_3_count)
    {
      case 0: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
              break;
      case 1: sc_assert( false ); 
              break;
      case 2: sc_assert( false ); 
              break;
    }
    ++reset_3_count;
    
    while (true)
    {
      wait();
      
    switch (reset_3_count)
    {
      case 0: sc_assert( false );
              break;
      case 1: sc_assert( sc_time_stamp() == sc_time(40, SC_NS) ); 
              break;
      case 2: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); 
              target_sync_reset_3_called = true;
              break;
    }
    ++reset_3_count;
    
    }
  }
   
  void target_sync_reset_4()
  {
    switch (reset_4_count)
    {
      case 0: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) );
              break;
      case 1: sc_assert( false );
              break;
      case 2: sc_assert( false ); 
              break;
    }
    ++reset_4_count;
    
    while (true)
    {
      wait();
      
    switch (reset_4_count)
    {
      case 0: sc_assert( false );
              break;
      case 1: sc_assert( sc_time_stamp() == sc_time(40, SC_NS) ); 
              break;
      case 2: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); 
              target_sync_reset_4_called = true;
              break;
    }
    ++reset_4_count;
    
    }
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  sc_start();

  sc_assert( top.caught_throw_it == 3 );
  sc_assert( top.caught_reset == 3 );
  sc_assert( top.caught_kill == 3 );
  
  sc_assert( top.target_suspend_1_called );
  sc_assert( top.target_suspend_2_called );
  sc_assert( top.target_suspend_3_called );
  sc_assert( top.target_suspend_4_called );
  sc_assert( top.target_suspend_5_called );

  sc_assert( !top.target_disable_1_called );
  sc_assert( top.target_disable_2_called );
  sc_assert( top.target_disable_2_called_again );
  sc_assert( top.target_disable_3_called );
  sc_assert( top.target_disable_4_called );
  sc_assert( top.target_disable_5_called );

  sc_assert( top.target_sync_reset_1_called );
  sc_assert( top.target_sync_reset_2_called );
  sc_assert( top.target_sync_reset_3_called );
  sc_assert( top.target_sync_reset_4_called );
  
  cout << endl << "Success" << endl;
  return 0;
}
