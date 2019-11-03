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

// proc_ctrl_priority.cpp -- test for 
//
//  Original Author: John Aynsley, Doulus
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: proc_ctrl_priority.cpp,v $
// Revision 1.3  2011/09/05 21:23:35  acg
//  Philipp A. Hartmann: eliminate compiler warnings.
//
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Priority of process control methods suspend, disable, sync_reset_on, reset

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    SC_THREAD(calling);

    SC_THREAD(target);
      t = sc_get_current_process_handle();

    count = 0;
    f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
  }
  
  sc_process_handle t;
  sc_event ev;
  int count;
  bool target_awoke;
  int f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;

  void calling()
  {
    count = 0;
    wait(SC_ZERO_TIME);
    
    count = 1;
    ev.notify(5, SC_NS);
    wait(10, SC_NS);
        
    count = 2;
    ev.notify(5, SC_NS);
    t.disable();
    wait(10, SC_NS);
    
    count = 3;
    ev.notify(5, SC_NS);
    t.disable();
    t.disable(); // Dummy
    t.resume();
    t.resume();
    t.sync_reset_off(); // Dummy
    t.sync_reset_off(); // Dummy
    wait(10, SC_NS);
    
    count = 4;
    ev.notify(5, SC_NS);
    t.enable();
    wait(10, SC_NS);

    count = 5;
    ev.notify(5, SC_NS);
    t.enable(); // Dummy
    t.enable(); // Dummy
    t.sync_reset_off(); // Dummy
    t.sync_reset_off(); // Dummy
    wait(10, SC_NS);

    count = 6;
    ev.notify(5, SC_NS);
    t.reset();
    wait(SC_ZERO_TIME);
    
    count = 7;
    wait(sc_time(100, SC_NS) - sc_time_stamp());

    count = 8;
    ev.notify(5, SC_NS);
    t.disable();
    wait(10, SC_NS);
    
    count = 9;
    ev.notify(5, SC_NS);
    t.reset();
    wait(10, SC_NS);

    count = 10;
    ev.notify(5, SC_NS);
    t.reset();
    wait(10, SC_NS);

    count = 11;
    ev.notify(5, SC_NS);
    t.enable();
    wait(sc_time(200, SC_NS) - sc_time_stamp());
    
    count = 12;
    ev.notify(5, SC_NS);
    t.suspend();
    wait(10, SC_NS);
    
    count = 13;
    t.suspend(); // Dummy
    t.suspend(); // Dummy
    t.enable(); // Dummy
    t.enable(); // Dummy
    t.sync_reset_off(); // Dummy
    t.sync_reset_off(); // Dummy
    wait(10, SC_NS);
    
    count = 14;
    t.resume();
    wait(10, SC_NS);

    count = 15;
    ev.notify(5, SC_NS);
    t.resume(); // Dummy
    t.resume(); // Dummy
    t.enable(); // Dummy
    t.enable(); // Dummy
    t.sync_reset_off(); // Dummy
    t.sync_reset_off(); // Dummy
    wait(10, SC_NS);

    count = 16;
    ev.notify();
    t.resume(); // Dummy
    t.resume(); // Dummy 
    t.disable(); // Dummy
    t.disable(); // Dummy
    t.enable(); // Dummy
    t.enable(); // Dummy
    t.suspend();
    wait(10, SC_NS);

    count = 17;
    t.suspend(); // Dummy
    t.suspend(); // Dummy
    t.enable(); // Dummy
    t.enable(); // Dummy
    t.sync_reset_off(); // Dummy
    t.sync_reset_off(); // Dummy
    wait(10, SC_NS);

    count = 18;
    t.resume();
    wait(sc_time(300, SC_NS) - sc_time_stamp());

    count = 19;
    ev.notify();
    ev.notify(5, SC_NS);
    t.disable();
    t.disable(); // Dummy
    wait(5, SC_NS);
    t.disable(); // Dummy
    t.resume();
    t.resume();
    wait(5, SC_NS);

    count = 20;
    ev.notify(5, SC_NS);
    t.enable();
    t.enable(); // Dummy
    wait(SC_ZERO_TIME);
    
    count = 21;
    wait(10, SC_NS);

    count = 22;
    ev.notify(5, SC_NS);
    t.suspend();
    wait(10, SC_NS);
    
    count = 23;
    t.reset();
    ev.notify(5, SC_NS);
    wait(10, SC_NS);

    count = 24;
    t.reset();
    ev.notify(5, SC_NS);
    wait(10, SC_NS);

    count = 25;
    t.resume();
    wait(sc_time(400, SC_NS) - sc_time_stamp());
    
    count = 26;
    ev.notify();
    t.reset();
    wait(10, SC_NS);

    count = 27;
    ev.notify();
    t.suspend();
    wait(10, SC_NS);
    
    count = 28;
    t.reset();
    wait(10, SC_NS);
    t.resume();
    
    wait(sc_time(500, SC_NS) - sc_time_stamp());

    count = 29;
    t.sync_reset_on();
    t.sync_reset_on(); // Dummy
    wait(10, SC_NS);
    
    count = 30;
    ev.notify();
    wait(10, SC_NS);
    
    count = 31;
    t.resume(); // Dummy
    t.resume(); // Dummy 
    t.enable(); // Dummy
    t.enable(); // Dummy
    t.sync_reset_on(); // Dummy
    t.sync_reset_on(); // Dummy
    wait(10, SC_NS);
    
    count = 32;
    ev.notify();
    wait(10, SC_NS);
    
    count = 33;
    ev.notify();
    t.disable();
    wait(10, SC_NS);

    count = 34;
    ev.notify();
    wait(10, SC_NS);
    
    count = 35;
    t.enable();
    ev.notify();
    wait(10, SC_NS);
    
    count = 36;
    t.disable();
    t.disable(); // Dummy
    ev.notify();
    ev.notify(); // Dummy
    wait(10, SC_NS);
    
    count = 37;
    t.sync_reset_off();
    t.sync_reset_off(); // Dummy
    wait(10, SC_NS);
    
    count = 38;
    t.enable();
    wait(10, SC_NS);
    
    count = 39;
    ev.notify();
    wait(sc_time(700, SC_NS) - sc_time_stamp());
    
    sc_stop();
  }

  void target()
  {
    switch (count)
    {
        case 0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f1=1; break;
        case 6: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); f5=1; break;
        case 9: sc_assert( sc_time_stamp() == sc_time(110, SC_NS) ); f7=1; break;
        case 10: sc_assert( sc_time_stamp() == sc_time(120, SC_NS) ); f8=1; break;
        case 23: sc_assert( sc_time_stamp() == sc_time(330, SC_NS) ); f15=1; break;
        case 24: sc_assert( sc_time_stamp() == sc_time(340, SC_NS) ); f16=1; break;
        case 26: sc_assert( sc_time_stamp() == sc_time(400, SC_NS) ); f18=1; break;
        case 28: sc_assert( sc_time_stamp() == sc_time(420, SC_NS) ); f19=1; break;
        case 30: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f20=1; break;
        case 32: sc_assert( sc_time_stamp() == sc_time(530, SC_NS) ); f21=1; break;
        case 33: sc_assert( sc_time_stamp() == sc_time(540, SC_NS) ); f22=1; break;
        case 35: sc_assert( sc_time_stamp() == sc_time(560, SC_NS) ); f23=1; break;
        default: sc_assert( false ); break;
    }
    
    for (;;)
    {
      wait(ev);
      switch (count)
      {
        case 1: sc_assert( sc_time_stamp() == sc_time(5, SC_NS) ); f2=1; break;
        case 4: sc_assert( sc_time_stamp() == sc_time(35, SC_NS) ); f3=1; break;
        case 5: sc_assert( sc_time_stamp() == sc_time(45, SC_NS) ); f4=1; break;
        case 7: sc_assert( sc_time_stamp() == sc_time(55, SC_NS) ); f6=1; break;
        case 11: sc_assert( sc_time_stamp() == sc_time(135, SC_NS) ); f9=1; break;
        case 14: sc_assert( sc_time_stamp() == sc_time(220, SC_NS) ); f10=1; break;
        case 15: sc_assert( sc_time_stamp() == sc_time(235, SC_NS) ); f11=1; break;
        case 18: sc_assert( sc_time_stamp() == sc_time(260, SC_NS) ); f12=1; break;
        case 19: sc_assert( sc_time_stamp() == sc_time(300, SC_NS) ); f13=1; break;
        case 21: sc_assert( sc_time_stamp() == sc_time(315, SC_NS) ); f14=1; break;
        case 25: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f17=1; break;
        case 39: sc_assert( sc_time_stamp() == sc_time(600, SC_NS) ); f24=1; break;
        default: sc_assert( false ); break;
      }
    }
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  sc_assert( top.f1 ); 
  sc_assert( top.f2 ); 
  sc_assert( top.f3 ); 
  sc_assert( top.f4 ); 
  sc_assert( top.f5 ); 
  sc_assert( top.f6 ); 
  sc_assert( top.f7 ); 
  sc_assert( top.f8 ); 
  sc_assert( top.f9 ); 
  sc_assert( top.f10 ); 
  sc_assert( top.f11 ); 
  sc_assert( top.f12 ); 
  sc_assert( top.f13 ); 
  sc_assert( top.f14 ); 
  sc_assert( top.f15 ); 
  sc_assert( top.f16 ); 
  sc_assert( top.f17 ); 
  sc_assert( top.f18 ); 
  sc_assert( top.f19 ); 
  sc_assert( top.f20 ); 
  sc_assert( top.f21 ); 
  sc_assert( top.f22 ); 
  sc_assert( top.f23 ); 
  sc_assert( top.f24 ); 
  
  cout << endl << "Success" << endl;
  return 0;
}
  
