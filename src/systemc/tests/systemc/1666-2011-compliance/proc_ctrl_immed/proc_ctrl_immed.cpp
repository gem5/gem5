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

// proc_ctrl_immed.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: proc_ctrl_immed.cpp,v $
// Revision 1.3  2011/09/01 15:47:15  acg
//  John Aynsley: correction for immediate method invocation on reset.
//
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Process control methods executed immediately in same eval phase

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  : count(0), reset_count(0)
  {
    SC_THREAD(ctrl);
    SC_THREAD(target_thread);
      t = sc_get_current_process_handle();

    SC_METHOD(target_method);
      sensitive << ev;
      dont_initialize();
      m = sc_get_current_process_handle();
      m.disable();
      
    SC_METHOD(thread_reset_handler);
      sensitive << t.reset_event();
      dont_initialize();
      
    SC_METHOD(method_reset_handler);
      sensitive << m.reset_event();
      dont_initialize();
      
    SC_METHOD(thread_terminated_handler);
      sensitive << t.terminated_event();
      dont_initialize();
      
    SC_METHOD(method_terminated_handler);
      sensitive << m.terminated_event();
      dont_initialize();
      
    SC_METHOD(yield_helper_method);
      sensitive << yield_event_1;
      dont_initialize();
 
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
    f30 = f31 = f32 = f33 = f34 = f35 = f36 = f37 = f38 = f39 = 0;
  }
  
  sc_event yield_event_1, yield_event_2, target_awoke_event;
  int count, reset_count;
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  int f30, f31, f32, f33, f34, f35, f36, f37, f38, f39;
  
  sc_event ev;
  sc_process_handle t, m;
  std::exception ex;
  
  void ctrl()
  {
    wait(SC_ZERO_TIME);
    sc_assert( sc_delta_count() == 1 );
    
    count = 1;
    ev.notify();
    wait(target_awoke_event);
    
    count = 2;
    ev.notify();
    t.suspend();
    yield();
    
    count = 2;
    t.resume();
    wait(target_awoke_event);

    count = 3;
    ev.notify();
    t.disable();
    wait(target_awoke_event);
    
    count = 4;
    ev.notify();
    yield();

    count = 5;
    t.enable();
    yield();

    count = 6;
    ev.notify();
    wait(target_awoke_event);
    
    count = 7;
    t.suspend();
    ev.notify();
    yield();
    
    count = 8;
    t.resume();
    wait(target_awoke_event);
    
    count = 9;
    reset_count = 9;
    t.sync_reset_on();
    ev.notify();
    wait(target_awoke_event);
    
    count = 10;
    reset_count = 10;
    ev.notify();
    wait(target_awoke_event);

    count = 11;
    t.sync_reset_off();
    ev.notify();
    wait(target_awoke_event);
    
    count = 12;
    t.resume();
    t.enable();
    t.sync_reset_off();
    yield();
    
    count = 13;
    ev.notify();
    wait(target_awoke_event);
     
    count = 14;
    reset_count = 14;
    t.reset();
    
    count = 15;
    ev.notify();
    wait(target_awoke_event);
    
    count = 16;
    reset_count = 16;
    t.reset();
    
    count = 17;
    t.throw_it(ex);

    count = 18;
    t.kill();
    yield();
    
    count = 19;
    m.enable();
    ev.notify();
    wait(target_awoke_event);
        
    count = 20;
    ev.notify();
    m.suspend();
    yield();
    
    count = 21;
    m.resume();
    wait(target_awoke_event);
        
    count = 22;
    m.suspend();
    ev.notify();
    yield();
    
    count = 23;
    m.resume();
    wait(target_awoke_event);

    count = 24;
    m.suspend();
    ev.notify();
    
    count = 25;
    m.resume();
    wait(target_awoke_event);

    count = 26;
    reset_count = 26;
    m.sync_reset_on();
    ev.notify();
    wait(target_awoke_event);
    
    count = 27;
    m.disable();
    ev.notify();
    yield();
    
    count = 28;
    reset_count = 28;
    m.enable();
    ev.notify();
    wait(target_awoke_event);
    
    count = 29;
    m.sync_reset_off();
    m.enable();
    m.resume();
    yield();
    
    count = 30;
    reset_count = 30;
    m.reset();
    
    count = 31;
    m.kill();
    yield();
    
    sc_assert( sc_delta_count() == 1 );
    f27 = 1;
  }
  
  void target_thread()
  {
    switch (count)
    {
      case  0: f0=1; break;
      case  9: f7=1; break;
      case 10: f9=1; break;
      case 14: f13=1; break;
      case 16: f16=1; break;
      default: sc_assert(false); break;
    }
    while (true)
    {
      try {
        target_awoke_event.notify();
        wait(ev);
       }
      catch (const std::exception& e) {
        switch (count)
        {
          case  9: sc_assert(sc_is_unwinding()); f6=1; break;
          case 10: sc_assert(sc_is_unwinding()); f8=1; break;
          case 14: sc_assert(sc_is_unwinding()); f12=1; break;
          case 16: sc_assert(sc_is_unwinding()); f15=1; break;
          case 17: sc_assert( !sc_is_unwinding() ); f17=1; break;
          case 18: sc_assert(sc_is_unwinding()); f19=1; break;
          default: sc_assert(false); break;
        }
        if ( sc_is_unwinding() )
          throw dynamic_cast<const sc_unwind_exception&>(e);
      }
      switch (count)
      {
        case  1: f1=1; break;
        case  2: f2=1; break;
        case  3: f3=1; break;
        case  6: f4=1; break;
        case  8: f5=1; break;
        case 11: f10=1; break;
        case 13: f11=1; break;
        case 15: f14=1; break;
        case 17: f18=1; break;
        default: sc_assert(false); break;
      }
    }
  }
  
  void target_method()
  {
    switch (count)
    {
      case 19: f20=1; break;
      case 21: f21=1; break;
      case 23: f22=1; break;
      case 25: f23=1; break;
      case 26: f24=1; break;
      case 28: f25=1; break;
      case 30: f26=1; break;
      default: sc_assert(false); break;
    }
    target_awoke_event.notify();
  }
  
  void thread_reset_handler()
  {
    switch (reset_count)
    {
      case  9: f30=1; break;
      case 10: f31=1; break;
      case 14: f32=1; break;
      case 16: f33=1; break;
      default: sc_assert(false); break;
    }
  }
  
  void method_reset_handler()
  {
    switch (reset_count)
    {
      case 26: f34=1; break;
      case 28: f35=1; break;
      case 30: f36=1; break;
      default: sc_assert(false); break;
    }
  }
  
  void thread_terminated_handler()
  {
    sc_assert(count == 18);
    sc_assert(sc_delta_count() == 1);
    f37 = 1;
  }
  
  void method_terminated_handler()
  {
    sc_assert(count == 31);
    sc_assert(sc_delta_count() == 1);
    f38 = 1;
  }
  
  void yield()
  {
    yield_event_1.notify();
    wait(yield_event_2);
  }
  
  void yield_helper_method()
  {
    yield_event_2.notify();
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  sc_assert( top.f0 );
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
  sc_assert( top.f25 );
  sc_assert( top.f26 );
  sc_assert( top.f27 );

  sc_assert( top.f30 );
  sc_assert( top.f31 );
  sc_assert( top.f32 );
  sc_assert( top.f33 );
  sc_assert( top.f34 );
  sc_assert( top.f35 );
  sc_assert( top.f36 );
  sc_assert( top.f37 );
  sc_assert( top.f38 );
  
  cout << endl << "Success" << endl;
  return 0;
}
  
