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

// method_with_reset.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: method_with_reset.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Method processes with sync and async resets, reset_event, sc_event_or_list

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  : clk("clk")
  , count(0)
  {
    SC_THREAD(ctrl);
    
    SC_METHOD(method_sync_reset);
      sensitive << clk.posedge_event();
      reset_signal_is(reset, true);
      dont_initialize();
      m1 = sc_get_current_process_handle();
      
    SC_METHOD(method_async_reset);
      sensitive << clk.posedge_event();
      async_reset_signal_is(reset, true);
      dont_initialize();
      m2 = sc_get_current_process_handle();
      
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
    f30 = f31 = f32 = f33 = f34 = f35 = f36 = f37 = f38 = f39 = 0;
    f40 = f41 = f42 = f43 = f44 = f45 = f46 = f47 = f48 = f49 = 0;
    f50 = f51 = f52 = f53 = f54 = f55 = f56 = f57 = f58 = f59 = 0;
  }
  
  sc_process_handle m1, m2, m3, m4, m5, m6;
  sc_signal<bool> clk, reset, sreset, areset;
  int count;
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  int f30, f31, f32, f33, f34, f35, f36, f37, f38, f39;
  int f40, f41, f42, f43, f44, f45, f46, f47, f48, f49;
  int f50, f51, f52, f53, f54, f55, f56, f57, f58, f59;
  
  void ctrl()
  {
    count = 1;
    reset.write(false);
    sreset.write(false);
    areset.write(false);
    clk.write(false);
    wait(10, SC_NS);
    
    count = 2;
    reset.write(true);
    wait(10, SC_NS);
    
    count = 3;
    reset.write(false);
    wait(10, SC_NS);
    
    count = 4;
    reset.write(true);
    wait(10, SC_NS);
    
    count = 5;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 6;
    clk.write(false);
    wait(10, SC_NS);
    
    count = 7;
    reset.write(false);
    wait(10, SC_NS);
    
    count = 8;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 9;
    clk.write(false);
    wait(10, SC_NS);
    
    count = 10;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 11;
    reset.write(true);
    wait(10, SC_NS);
    
    count = 12;
    reset.write(false);
    wait(10, SC_NS);
    
    count = 13;
    reset.write(true);
    wait(10, SC_NS);

    count = 14;
    clk.write(false);
    wait(sc_time(200, SC_NS) - sc_time_stamp());

    count = 15;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 16;
    clk.write(false);
    wait(10, SC_NS);
    
    count = 17;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 18;
    reset.write(false);
    wait(10, SC_NS);
    
    count = 19;
    reset.write(true);
    wait(10, SC_NS);

    count = 20;
    reset.write(false);
    wait(10, SC_NS);
    
    count = 21;
    clk.write(false);
    wait(sc_time(300, SC_NS) - sc_time_stamp());
    
    count = 22;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 23;
    clk.write(false);
    m1.disable();
    m2.disable();
    
    sc_spawn_options opt3;
    opt3.spawn_method();
    opt3.set_sensitivity( &clk.posedge_event() );
    opt3.reset_signal_is(sreset, true);
    opt3.async_reset_signal_is(areset, true);
    m3 = sc_spawn(sc_bind( &Top::spawned_method, this ), "m3", &opt3);
 
    sc_spawn_options opt4; 
    opt4.spawn_method();
    opt4.set_sensitivity( &m3.reset_event() );
    opt4.dont_initialize();
    m4 = sc_spawn(sc_bind( &Top::reset_handler, this), "m4", &opt4);
    
    sc_spawn_options opt5;
    opt5.spawn_method();
    m5 = sc_spawn(sc_bind( &Top::reset_or_terminated_handler, this), "m5", &opt5);
    
    std::vector<sc_event*> vec = this->get_child_events();
    sc_assert( vec.size() == 0 );
    wait(10, SC_NS);
    
    m6 = sc_spawn(sc_bind( &Top::multiple_reset_handler, this) );
    
    count = 24;
    clk.write(true);   
    wait(10, SC_NS);
    
    count = 25;
    clk.write(false);
    wait(10, SC_NS);
    
    count = 26;
    sreset.write(true);
    wait(10, SC_NS);
    
    count = 27;
    clk.write(true);   
    wait(sc_time(500, SC_NS) - sc_time_stamp());
    
    count = 28;
    clk.write(false);   
    sreset.write(false);
    wait(10, SC_NS);
 
    count = 29;
    m3.reset();   
    wait(10, SC_NS);
 
    count = 30;
    areset.write(true);
    wait(10, SC_NS);
 
    count = 31;
    areset.write(false);
    wait(10, SC_NS);
 
    count = 32;
    areset.write(true);
    wait(10, SC_NS);
 
    count = 33;
    clk.write(true);
    wait(10, SC_NS);
    
    count = 34;
    clk.write(false);   
    wait(10, SC_NS);
    
    count = 35;
    areset.write(false);
    wait(sc_time(600, SC_NS) - sc_time_stamp());
    
    count = 36;
    m3.kill();
    wait(10, SC_NS);

    count = 37;
    m1.reset();
    wait(10, SC_NS);

    count = 38;
    m2.reset();
    wait(10, SC_NS);

    
  }
  
  void method_sync_reset()
  {
    if (reset)
      switch (count)
      {
        case  5: sc_assert( sc_time_stamp() == sc_time( 40, SC_NS) ); f0=1; break;
        case 15: sc_assert( sc_time_stamp() == sc_time(200, SC_NS) ); f3=1; break;
        case 17: sc_assert( sc_time_stamp() == sc_time(220, SC_NS) ); f4=1; break;
        default: sc_assert( false );
      }
    else
      switch (count)
      {
        case  8: sc_assert( sc_time_stamp() == sc_time( 70, SC_NS) ); f1=1; break;
        case 10: sc_assert( sc_time_stamp() == sc_time( 90, SC_NS) ); f2=1; break;
        case 22: sc_assert( sc_time_stamp() == sc_time(300, SC_NS) ); f5=1; break;
        case 37: sc_assert( sc_time_stamp() == sc_time(610, SC_NS) ); f55=1; break;
        default: sc_assert( false );
      }
  }
  
  void method_async_reset()
  {
    if (reset)
      switch (count)
      {
        case  2: sc_assert( sc_time_stamp() == sc_time( 10, SC_NS) ); f10=1; break;
        case  4: sc_assert( sc_time_stamp() == sc_time( 30, SC_NS) ); f11=1; break;
        case  5: sc_assert( sc_time_stamp() == sc_time( 40, SC_NS) ); f12=1; break;
        case 11: sc_assert( sc_time_stamp() == sc_time(100, SC_NS) ); f15=1; break;
        case 13: sc_assert( sc_time_stamp() == sc_time(120, SC_NS) ); f16=1; break;
        case 15: sc_assert( sc_time_stamp() == sc_time(200, SC_NS) ); f17=1; break;
        case 17: sc_assert( sc_time_stamp() == sc_time(220, SC_NS) ); f18=1; break;
        case 19: sc_assert( sc_time_stamp() == sc_time(240, SC_NS) ); f19=1; break;
        default: sc_assert( false );
      }
    else
      switch (count)
      {
        case  8: sc_assert( sc_time_stamp() == sc_time( 70, SC_NS) ); f13=1; break;
        case 10: sc_assert( sc_time_stamp() == sc_time( 90, SC_NS) ); f14=1; break;
        case 22: sc_assert( sc_time_stamp() == sc_time(300, SC_NS) ); f20=1; break;
        case 38: sc_assert( sc_time_stamp() == sc_time(620, SC_NS) ); f57=1; break;
        default: sc_assert( false );
      }
  }
  
  void spawned_method()
  {
    switch (count)
    {
      case 23: sc_assert( sc_time_stamp() == sc_time(310, SC_NS) ); f30=1; break;
      case 24: sc_assert( sc_time_stamp() == sc_time(320, SC_NS) ); f31=1; break;
      case 27: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f32=1; break;
      case 29: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f34=1; break;
      case 30: sc_assert( sc_time_stamp() == sc_time(520, SC_NS) ); f36=1; break;
      case 32: sc_assert( sc_time_stamp() == sc_time(540, SC_NS) ); f38=1; break;
      case 33: sc_assert( sc_time_stamp() == sc_time(550, SC_NS) ); f40=1; break;
      default: sc_assert( false );
    }
  }
  
  void reset_handler()
  {
    switch (count)
    {
      case 27: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f33=1; break;
      case 29: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f35=1; break;
      case 30: sc_assert( sc_time_stamp() == sc_time(520, SC_NS) ); f37=1; break;
      case 32: sc_assert( sc_time_stamp() == sc_time(540, SC_NS) ); f39=1; break;
      case 33: sc_assert( sc_time_stamp() == sc_time(550, SC_NS) ); f41=1; break;
      default: sc_assert( false );
    }
  }

  sc_event_or_list event_list;
  
  void reset_or_terminated_handler()
  {
    switch (count)
    {
      case 23: sc_assert( sc_time_stamp() == sc_time(310, SC_NS) ); f42=1; break;
      case 27: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f43=1; break;
      case 29: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f44=1; break;
      case 30: sc_assert( sc_time_stamp() == sc_time(520, SC_NS) ); f45=1; break;
      case 32: sc_assert( sc_time_stamp() == sc_time(540, SC_NS) ); f46=1; break;
      case 33: sc_assert( sc_time_stamp() == sc_time(550, SC_NS) ); f47=1; break;
      case 36: sc_assert( sc_time_stamp() == sc_time(600, SC_NS) ); f48=1; break;
      default: sc_assert( false );
    }
    event_list = m3.reset_event() | m3.terminated_event();
    next_trigger(event_list);
  }
  
  void multiple_reset_handler()
  {
    sc_event_or_list or_list;
    or_list |= m1.reset_event();
    or_list |= m2.reset_event();
    or_list |= m3.reset_event();
    or_list |= m4.reset_event();
    or_list |= m5.reset_event();
    
    while (true)
    {
      wait(or_list);
      switch (count)
      {
        case 27: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f50=1; break;
        case 29: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f51=1; break;
        case 30: sc_assert( sc_time_stamp() == sc_time(520, SC_NS) ); f52=1; break;
        case 32: sc_assert( sc_time_stamp() == sc_time(540, SC_NS) ); f53=1; break;
        case 33: sc_assert( sc_time_stamp() == sc_time(550, SC_NS) ); f54=1; break;
        case 37: sc_assert( sc_time_stamp() == sc_time(610, SC_NS) ); f56=1; break;
        case 38: sc_assert( sc_time_stamp() == sc_time(620, SC_NS) ); f58=1; break;
        default: sc_assert( false );
      }
    }
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  sc_allow_process_control_corners = true; // Andy's hack to switch on async_reset with method
 
  Top top("top");
  
  sc_start();
  
  sc_assert( top.f0 );
  sc_assert( top.f1 );
  sc_assert( top.f2 );
  sc_assert( top.f3 );
  sc_assert( top.f4 );
  sc_assert( top.f5 );
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
  
  sc_assert( top.f30 );
  sc_assert( top.f31 );
  sc_assert( top.f32 );
  sc_assert( top.f33 );
  sc_assert( top.f34 );
  sc_assert( top.f35 );
  sc_assert( top.f36 );
  sc_assert( top.f37 );
  sc_assert( top.f38 );
  sc_assert( top.f39 );
  sc_assert( top.f40 );
  sc_assert( top.f41 );
  sc_assert( top.f42 );
  sc_assert( top.f43 );
  sc_assert( top.f44 );
  sc_assert( top.f45 );
  sc_assert( top.f46 );
  sc_assert( top.f47 );
  sc_assert( top.f48 );

  sc_assert( top.f50 );
  sc_assert( top.f51 );
  sc_assert( top.f52 );
  sc_assert( top.f53 );
  sc_assert( top.f54 );
  sc_assert( top.f55 );
  sc_assert( top.f56 );
  sc_assert( top.f57 );
  sc_assert( top.f58 );

  cout << endl << "Success" << endl;
  return 0;
}
  
