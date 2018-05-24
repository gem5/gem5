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

// sync_reset.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: sync_reset.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// sync_reset_on/off

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M2: sc_module
{
  M2(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_THREAD(target1);
      t1 = sc_get_current_process_handle();
    
    sc_spawn_options opt;
      opt.spawn_method();
      opt.dont_initialize();
      opt.set_sensitivity( &t1.reset_event() );
    sc_spawn(sc_bind( &M2::reset_handler, this ), "reset_handler", &opt);
    
    SC_THREAD(target2);
      t2 = sc_get_current_process_handle();
      
    SC_METHOD(target3);
      sensitive << ev;
      t3 = sc_get_current_process_handle();
      
    count = 1;
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
    f30 = f31 = f32 = f33 = f34 = f35 = f36 = f37 = f38 = f39 = 0;
    f40 = f41 = f42 = f43 = f44 = f45 = f46 = f47 = f48 = f49 = 0;
  }
  
  sc_process_handle t1, t2, t3;
  sc_event ev;
  int count;
  
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  int f30, f31, f32, f33, f34, f35, f36, f37, f38, f39;
  int f40, f41, f42, f43, f44, f45, f46, f47, f48, f49;

  void ticker()
  {
    for (;;)
    {
      wait(10, SC_NS);
      sc_assert( !sc_is_unwinding() );
      ev.notify();
    }
  }
   
  void calling()
  {
    count = 1;
    wait(15, SC_NS);
    // Target runs at 10 NS 
    
    count = 2;
    t1.sync_reset_on();
    // Target does not run at 15 NS 
    
    wait(10, SC_NS);
    // Target is reset at 20 NS  
    
    count = 3;
    wait(10, SC_NS);
    // Target is reset again at 30 NS  
    
    count = 4;
    t1.sync_reset_off();
    // Target does not run at 35 NS 
    
    wait(10, SC_NS);
    // Target runs at 40 NS 
    
    count = 5;
    t1.sync_reset_off();
    // Double sync_reset_off 

    wait(10, SC_NS);
    // Target runs at 50 NS 
    
    count = 6;
    t1.sync_reset_on();
    t1.disable();
    wait(10, SC_NS);
    // Target does not run at 60 NS
    
    count = 7;
    t1.enable();
    // Target does not run at 65 NS
    wait(10, SC_NS);
    // Target reset at 70 NS
    
    count = 8;
    t1.disable();
    wait(10, SC_NS);
    // Target does not run at 80 NS
    
    count = 9;
    t1.sync_reset_off();
    wait(10, SC_NS);
    // Target still disabled at 90 NS

    count = 10;
    t1.enable();
    wait(10, SC_NS);
    // Target runs at 100 NS 
    
    count = 11;
    t1.suspend();
    wait(10, SC_NS);
    // Target does not run at 110 NS

    count = 12;
    wait(10, SC_NS);
    // Target still suspended at 120 NS

    count = 13;
    t1.resume();
    // Target runs at 125 NS
    wait(1, SC_NS);
        
    count = 14;
    wait(9, SC_NS);
    // Target runs again at 130 NS
    
    count = 15;
    t1.sync_reset_on();
    // Double sync_reset_on
    wait(10, SC_NS);
    // Target reset at 140 NS
    
    count = 16;
    t1.sync_reset_off();
    wait(10, SC_NS);
    // Target runs at 150 NS 
    
    count = 17;
    t1.sync_reset_off();
    wait(10, SC_NS);
    // Target runs at 160 NS 
    
    count = 18;
    t1.sync_reset_on();
    wait(10, SC_NS);
    // Target reset at 170 NS
    
    count = 19;
    t1.reset();
    // Target reset at 175 NS
    wait(SC_ZERO_TIME);

    count = 20;
    wait(1, SC_NS);
    t1.reset();
    // Target reset at 176 NS
    
    count = 21;
    t1.reset();
    // Target reset at 176 NS
    wait(1, SC_NS);

    count = 22;
    wait(8, SC_NS);
    // Target reset at 180 NS
    
    count = 23;
    wait(10, SC_NS);
    // Target reset at 190 NS
    
    count = 24;
    t1.sync_reset_off();
    wait(10, SC_NS);
    // Target runs at 200 NS 

    count = 25;
    wait(10, SC_NS);
    // Target runs at 210 NS 

    count = 26;
    t1.reset();
    wait(SC_ZERO_TIME);
    // Target reset at 215

    t1.disable(); // Close it down
    wait(sc_time(300, SC_NS) - sc_time_stamp());
    
    count = 27;
    t2.resume();
    wait(SC_ZERO_TIME);
    
    count = 28;
    wait(15, SC_NS);
        
    count = 29;
    t2.sync_reset_on();
    wait(10, SC_NS);
    
    t2.sync_reset_off();
    t2.suspend();
    wait(sc_time(405, SC_NS) - sc_time_stamp());

    count = 30;
    t3.resume();
    wait(SC_ZERO_TIME);

    count = 31;
    wait(10, SC_NS);
        
    count = 32;
    t3.sync_reset_on();
    wait(10, SC_NS);
    
    sc_stop();
  }

  void target1()
  {
    //cout << "Target1 called/reset at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
        case 1: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f0=1; break;
        case 2: sc_assert( sc_time_stamp() == sc_time(20, SC_NS) ); f1=1; break;
        case 3: sc_assert( sc_time_stamp() == sc_time(30, SC_NS) ); f2=1; break;
        case 7: sc_assert( sc_time_stamp() == sc_time(70, SC_NS) ); f3=1; break;
        case 15: sc_assert( sc_time_stamp() == sc_time(140, SC_NS) ); f4=1; break;
        case 18: sc_assert( sc_time_stamp() == sc_time(170, SC_NS) ); f5=1; break;
        case 19: sc_assert( sc_time_stamp() == sc_time(175, SC_NS) ); f6=1; break;
        case 20: sc_assert( sc_time_stamp() == sc_time(176, SC_NS) ); f7=1; break;
        case 21: sc_assert( sc_time_stamp() == sc_time(176, SC_NS) ); f8=1; break;
        case 22: sc_assert( sc_time_stamp() == sc_time(180, SC_NS) ); f9=1; break;
        case 23: sc_assert( sc_time_stamp() == sc_time(190, SC_NS) ); f10=1; break;
        case 26: sc_assert( sc_time_stamp() == sc_time(215, SC_NS) ); f11=1; break;
        default: sc_assert( false ); break;
    }

    for (;;)
    {
      try {
        wait(ev);
        //cout << "Target1 awoke at " << sc_time_stamp() << " count = " << count << endl;
        sc_assert( !sc_is_unwinding() );
        switch (count)
        {
          case 1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); f12=1; break;
          case 4: sc_assert( sc_time_stamp() == sc_time(40, SC_NS) ); f13=1; break;
          case 5: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); f14=1; break;
          case 10: sc_assert( sc_time_stamp() == sc_time(100, SC_NS) ); f15=1; break;
          case 13: sc_assert( sc_time_stamp() == sc_time(125, SC_NS) ); f16=1; break;
          case 14: sc_assert( sc_time_stamp() == sc_time(130, SC_NS) ); f17=1; break;
          case 16: sc_assert( sc_time_stamp() == sc_time(150, SC_NS) ); f18=1; break;
          case 17: sc_assert( sc_time_stamp() == sc_time(160, SC_NS) ); f19=1; break;
          case 24: sc_assert( sc_time_stamp() == sc_time(200, SC_NS) ); f20=1; break;
          case 25: sc_assert( sc_time_stamp() == sc_time(210, SC_NS) ); f21=1; break;
          default: sc_assert( false ); break;
        }
      }
      catch (const sc_unwind_exception& ex) {
        sc_assert( sc_is_unwinding() );
        sc_assert( ex.is_reset() );
        throw ex;
      }
    }
  }
  
  void reset_handler()
  {
    //cout << "reset_handler awoke at " << sc_time_stamp() << " count = " << count << endl;
    sc_assert( !sc_is_unwinding() );
    switch (count)
    {
      case 2: sc_assert( sc_time_stamp() == sc_time(20, SC_NS) ); f22=1; break;
      case 3: sc_assert( sc_time_stamp() == sc_time(30, SC_NS) ); f23=1; break;
      case 7: sc_assert( sc_time_stamp() == sc_time(70, SC_NS) ); f24=1; break;
      case 15: sc_assert( sc_time_stamp() == sc_time(140, SC_NS) ); f27=1; break;;
      case 18: sc_assert( sc_time_stamp() == sc_time(170, SC_NS) ); f28=1; break;
      case 19: sc_assert( sc_time_stamp() == sc_time(175, SC_NS) ); f29=1; break;
      case 21: sc_assert( sc_time_stamp() == sc_time(176, SC_NS) ); f31=1; break;
      case 22: sc_assert( sc_time_stamp() == sc_time(180, SC_NS) ); f32=1; break;
      case 23: sc_assert( sc_time_stamp() == sc_time(190, SC_NS) ); f33=1; break;
      case 26: sc_assert( sc_time_stamp() == sc_time(215, SC_NS) ); f34=1; break;
      default: sc_assert( false ); break;
    }
  }
  
  void target2()
  {
    if (sc_delta_count() == 0)
      t2.suspend(); // Hack to work around not being able to call suspend during elab

    switch (count)
    {
        case 27: sc_assert( sc_time_stamp() == sc_time(300, SC_NS) ); f35=1; break;
        case 29: sc_assert( sc_time_stamp() == sc_time(320, SC_NS) ); f37=1; break;
        default: sc_assert( false ); break;
    }
    while(1)
    {
      try {
        wait(10, SC_NS);
      }
      catch (const sc_unwind_exception& e) {
        switch (count)
        {
        case 29: sc_assert( sc_time_stamp() == sc_time(320, SC_NS) ); f38=1; break;
        default: sc_assert( false ); break;
        }
        throw e;
      }
      switch (count)
      {
        case 28: sc_assert( sc_time_stamp() == sc_time(310, SC_NS) ); f36=1; break;
        default: sc_assert( false ); break;
      }
    }
  }
  
  void target3()
  {
    if (sc_delta_count() == 0)
      t3.suspend(); // Hack to work around not being able to call suspend during elab

    switch (count)
    {
        case  1: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); break;
        case 30: sc_assert( sc_time_stamp() == sc_time(405, SC_NS) ); f39=1; break;
        case 31: sc_assert( sc_time_stamp() == sc_time(410, SC_NS) ); f40=1; break;
        case 32: sc_assert( sc_time_stamp() == sc_time(420, SC_NS) ); f41=1; break;
        default: sc_assert( false ); break;
    }
  }

  SC_HAS_PROCESS(M2);
};

int sc_main(int argc, char* argv[])
{
  M2 m("m");
  
  sc_start();
  
  sc_assert(m.f0);
  sc_assert(m.f1);
  sc_assert(m.f2);
  sc_assert(m.f3);
  sc_assert(m.f4);
  sc_assert(m.f5);
  sc_assert(m.f6);
  sc_assert(m.f7);
  sc_assert(m.f8);
  sc_assert(m.f9);
  sc_assert(m.f10);
  sc_assert(m.f11);
  sc_assert(m.f12);
  sc_assert(m.f13);
  sc_assert(m.f14);
  sc_assert(m.f15);
  sc_assert(m.f16);
  sc_assert(m.f17);
  sc_assert(m.f18);
  sc_assert(m.f19);
  sc_assert(m.f20);
  sc_assert(m.f21);
  sc_assert(m.f22);
  sc_assert(m.f23);
  sc_assert(m.f24);
  sc_assert(m.f27);
  sc_assert(m.f28);
  sc_assert(m.f29);
  sc_assert(m.f31);
  sc_assert(m.f32);
  sc_assert(m.f33);
  sc_assert(m.f34);
  sc_assert(m.f35);
  sc_assert(m.f36);
  sc_assert(m.f37);
  sc_assert(m.f38);
  sc_assert(m.f39);
  sc_assert(m.f40);
  sc_assert(m.f41);
  
  cout << endl << "Success" << endl;
  return 0;
}
  
