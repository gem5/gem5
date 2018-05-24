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

// throw_it.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: throw_it.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Process control method throw_it

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
    
    SC_THREAD(target1);
      t1 = sc_get_current_process_handle();
      
    SC_THREAD(target2);
      t2 = sc_get_current_process_handle();
      
    SC_THREAD(target3);
      t3 = sc_get_current_process_handle();
      
    SC_METHOD(target4);
      t4 = sc_get_current_process_handle();
      
    SC_THREAD(target5);
      async_reset_signal_is(areset, true);
      sensitive << ev;
      t5 = sc_get_current_process_handle();
      t5.disable();
      
    count = 0;
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
  }
  
  sc_process_handle t1, t2, t3, t4, t5;
  sc_event ev;
  sc_signal<bool> areset;
  int count;
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  
  std::exception ex;
  
  void start_of_simulation()
  {
    try {
      t1.throw_it(ex);
    }
    catch (std::exception ex) {
      f1 = 1;
      sc_assert( t1.valid() );
      sc_assert( !t1.terminated() );
    }
  }

  void calling()
  {
    wait(SC_ZERO_TIME);

    count = 1;
    ev.notify(5, SC_NS);
    wait(10, SC_NS);

    count = 2;
    t1.throw_it(ex);
    sc_assert( t1.valid() );
    sc_assert( !t1.terminated() );
    sc_assert(f4); 
    wait(10, SC_NS);

    count = 3;
    t4.throw_it(ex); // Throw exception in method process
    sc_assert( t4.valid() );
    sc_assert( !t4.terminated() );
    wait(sc_time(200, SC_NS) - sc_time_stamp());
    
    count = 4;
    t1.suspend();
    ev.notify(5, SC_NS);
    wait(10, SC_NS);
    
    count = 5;
    t1.throw_it(ex);
    wait(10, SC_NS);
    
    count = 6;
    t1.throw_it(ex);
    sc_assert( t1.valid() );
    sc_assert( !t1.terminated() );
    wait(10, SC_NS);
    
    count = 7;
    t1.resume();
    wait(sc_time(300, SC_NS) - sc_time_stamp());
    
    count = 8;
    t1.disable();
    ev.notify(5, SC_NS);
    wait(10, SC_NS);
    
    count = 9;
    t1.throw_it(ex);
    wait(10, SC_NS);
    
    count = 10;
    t1.throw_it(ex);
    wait(10, SC_NS);
    
    count = 11;
    t1.enable();
    wait(sc_time(400, SC_NS) - sc_time_stamp());
    
    count = 12;
    t1.sync_reset_on();
    ev.notify(5, SC_NS);
    wait(10, SC_NS);
    wait(sc_time(400, SC_NS) - sc_time_stamp());
    
    count = 13;
    t1.throw_it(ex);
    wait(10, SC_NS);
    
    count = 14;
    t1.throw_it(ex);
    wait(10, SC_NS);
    
    count = 15;
    t1.sync_reset_off();
    wait(sc_time(500, SC_NS) - sc_time_stamp());
    
    count = 16;
    ev.notify();
    t1.throw_it(ex);
    wait(10, SC_NS);
    
    count = 17;
    t1.reset();
    wait(sc_time(600, SC_NS) - sc_time_stamp());
    
    count = 18;
    t1.disable();
    t5.enable();
    areset.write(false);
    wait(10, SC_NS);
    
    count = 19;
    ev.notify();
    wait(10, SC_NS);

    count = 20;
    ev.notify();
    wait(10, SC_NS);

    count = 21;
    areset.write(true);
    wait(10, SC_NS);

    count = 22;
    t5.throw_it(ex);
    wait(10, SC_NS);

    count = 23;
    ev.notify();
    wait(10, SC_NS);

    count = 24;
    t5.throw_it(ex);
    wait(10, SC_NS);

    count = 25;
    areset.write(false);
    wait(sc_time(700, SC_NS) - sc_time_stamp());

    count = 26;
    ev.notify();
    wait(10, SC_NS);
    // async_reset_signal_is ?    
  }

  void target1() // Target for throw_it from calling()
  {
    switch (count)
    {
      case  0: sc_assert( sc_time_stamp() == sc_time(  0, SC_NS) ); f2=1; break;
      case 12: sc_assert( sc_time_stamp() == sc_time(405, SC_NS) ); f13=1; break;
      case 17: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f19=1; break;
      default: sc_assert( false ); break;
    }
    
    for (;;)
    {
      try {
        wait(ev);
        switch (count)
        {
          case  1: sc_assert( sc_time_stamp() == sc_time(5, SC_NS) ); f3=1; break;
          case  7: sc_assert( sc_time_stamp() == sc_time(230, SC_NS) ); f9=1; break;
          default: sc_assert( false ); break;
        }
      }
      catch (const std::exception& ex) {
        switch (count)
        {
          case  2: sc_assert( !sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time( 10, SC_NS) ); f4=1; break;
          case  5: sc_assert( sc_time_stamp() == sc_time(210, SC_NS) ); f7=1; break;
          case  6: sc_assert( sc_time_stamp() == sc_time(220, SC_NS) ); f8=1; break;
          case  9: sc_assert( sc_time_stamp() == sc_time(310, SC_NS) ); f10=1; break;
          case 10: sc_assert( sc_time_stamp() == sc_time(320, SC_NS) ); f11=1; break;
          case 12: sc_assert( sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(405, SC_NS) ); f12=1; 
                   throw dynamic_cast<const sc_unwind_exception&>(ex);
          case 13: sc_assert( !sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(410, SC_NS) ); f14=1; break;
          case 14: sc_assert( sc_time_stamp() == sc_time(420, SC_NS) ); f15=1; break;
          case 16: sc_assert( sc_time_stamp() == sc_time(500, SC_NS) ); f16=1; break;
          case 17: sc_assert( sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f18=1; 
                   throw dynamic_cast<const sc_unwind_exception&>(ex);
          default: sc_assert( false ); break;
        }
      }
    }
  }

  void target2()
  {
    wait(100, SC_NS);
    try {
      t2.throw_it(ex);  // Process throws an exception to itself
      sc_assert( false );
    }
    catch (std::exception ex) {
      sc_assert( t2.valid() );
      sc_assert( !t2.terminated() );
      f5 = 1;
      t3.throw_it(ex);
    }
  }

  void target3() // Target for throw_it from target2()
  {
    try {
      wait(1, SC_US);
    }
    catch (std::exception ex) {
      sc_assert( t3.valid() );
      sc_assert( !t3.terminated() );
      f6 = 1;
    }
  }
  
  void target4() // SC_METHOD, target for throw_it from calling()
  {
    t4.throw_it(ex); // Method process throws exception to itself
    if (count != 0)    
      sc_assert( false );
  }
  
  void target5() // Target for throw_it from calling() + async_reset_signal
  {
    switch (count)
    {
      case 19: sc_assert( sc_time_stamp() == sc_time(610, SC_NS) ); f20=1; break;
      case 21: sc_assert( sc_time_stamp() == sc_time(630, SC_NS) ); f23=1; break;
      case 23: sc_assert( sc_time_stamp() == sc_time(650, SC_NS) ); f26=1; break;
      default: sc_assert( false ); break;
    }
    
    for (;;)
    {
      try {
        wait();

        switch (count)
        {
          case 20: sc_assert( sc_time_stamp() == sc_time(620, SC_NS) ); f21=1; break;
          case 26: sc_assert( sc_time_stamp() == sc_time(700, SC_NS) ); f28=1; break;
          default: sc_assert( false ); break;
        }
      }
      catch (const std::exception& ex) {
        switch (count)
        {
          case 21: sc_assert( sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(630, SC_NS) ); f22=1; 
                   throw dynamic_cast<const sc_unwind_exception&>(ex);
          case 22: sc_assert( !sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(640, SC_NS) ); f24=1; break;
          case 23: sc_assert( sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(650, SC_NS) ); f25=1; 
                   throw dynamic_cast<const sc_unwind_exception&>(ex);
          case 24: sc_assert( !sc_is_unwinding() );
                   sc_assert( sc_time_stamp() == sc_time(660, SC_NS) ); f27=1; break;
          default: sc_assert( false ); break;
        }
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
  sc_assert( top.f28 );
  
  cout << endl << "Success" << endl;
  return 0;
}
  
