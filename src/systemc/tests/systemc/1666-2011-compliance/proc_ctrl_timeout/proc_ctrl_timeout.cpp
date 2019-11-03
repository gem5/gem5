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

// proc_ctrl_timeout.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: proc_ctrl_timeout.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Process control methods interacting with time-out and event lists

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

    SC_METHOD(target2);
      t2 = sc_get_current_process_handle();
      
    SC_THREAD(target3);
      t3 = sc_get_current_process_handle();

    SC_METHOD(target4);
      t4 = sc_get_current_process_handle();
      
    SC_METHOD(target5);
      t5 = sc_get_current_process_handle();
      t5.disable();
      sensitive << ev4;
      
    count = 0;
    f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = f10 = 0;
    f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = 0;
  }
  
  sc_process_handle t1, t2, t3, t4, t5;
  sc_event ev1, ev2, ev3, ev4;
  int count;
  int f1, f2, f3, f4, f5, f6, f7, f8, f9, f10;
  int f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22;

  void calling()
  {
    wait(SC_ZERO_TIME);
    count = 1;
    wait(15, SC_NS);
    
    count = 2;
    try {
      t1.disable();
      sc_assert(false);
    }
    catch (sc_exception ex) {
      //cout << "Exception caught at " << sc_time_stamp() << endl;
      f7 = 1;
    }

    try {
      t2.disable();
      sc_assert(false);
    }
    catch (sc_exception ex) {
      //cout << "Exception caught at " << sc_time_stamp() << endl;
      f8 = 1;
    }
    wait(SC_ZERO_TIME);
    t1.kill();
    t2.kill();
    ev1.notify();
    wait(sc_time(100, SC_NS) - sc_time_stamp());
    
    count = 6;
    t3.disable();
    t4.disable();
    wait(10, SC_NS);
    
    ev2.notify();
    wait(10, SC_NS);
    
    t3.enable();
    t4.enable();
    wait(10, SC_NS);

    ev3.notify();
    wait(10, SC_NS);
    
    ev2.notify();
    wait(sc_time(200, SC_NS) - sc_time_stamp());
    
    count = 7;
    ev1.notify();
    wait(10, SC_NS);
    
    t3.suspend();
    t4.suspend();
    wait(10, SC_NS);
    
    ev2.notify();
    wait(10, SC_NS);
    
    t3.resume();
    t4.resume();
    wait(10, SC_NS);
    
    ev3.notify();
    wait(sc_time(300, SC_NS) - sc_time_stamp());
    
    count = 8;
    ev1.notify();
    wait(10, SC_NS);
    
    ev2.notify();
    wait(10, SC_NS);
    
    t3.reset();
    count = 9;
    wait(10, SC_NS);
    
    ev3.notify();
    wait(10, SC_NS);
    
    ev1.notify();
    wait(10, SC_NS);
    
    ev2.notify();
    wait(sc_time(400, SC_NS) - sc_time_stamp());
    t3.disable();
    t4.disable();
    
    // Now target5
    count = 9;
    t5.enable();
    ev4.notify();
    wait(SC_ZERO_TIME);    

    count = 10;    
    ev3.notify(3, SC_NS);
    ev2.notify(2, SC_NS);
    ev1.notify(1, SC_NS);
    wait(10, SC_NS);
   
    count = 11;
    t5.reset(); // On reset, dynamic sensit is cleared, then target is called again
    wait(SC_ZERO_TIME);
    
    count = 12;
    ev3.notify(3, SC_NS);
    ev2.notify(2, SC_NS);
    ev1.notify(1, SC_NS);
    wait(10, SC_NS);
   
    count = 13;
    ev4.notify();
    wait(SC_ZERO_TIME);

    count = 14;
    try {
      t5.disable();  // Disabling a process waiting on a time-out
    }
    catch (sc_exception ex) {
      //cout << "Exception caught at " << sc_time_stamp() << endl;
      f21 = 1;
    }
    wait(sc_time(500, SC_NS) - sc_time_stamp());
    
    count = 15;
    t5.reset();
    wait(10, SC_NS);
    
    count = 16;
    ev4.notify();
    wait(10, SC_NS);

    sc_stop();
  }

  void target1()
  {
    //cout << "target1() called at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
        case  0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f1=1; break;
        default: sc_assert( false ); break;
    }
    
    for (;;)
    {
      wait(10, SC_NS);
      //cout << "target1() awoke at " << sc_time_stamp() << " count = " << count << endl;
      switch (count)
      {
        case  1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); f5=1; break;
        default: sc_assert( false ); break;
      }
    }
  }

  void target2()
  {
    //cout << "target2() called at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
        case  0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f2=1; break;
        case  1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); f6=1; break;
        default: sc_assert( false ); break;
    }
    next_trigger(10, SC_NS);
  }
    
  void target3()
  {
    //cout << "target3() called at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
        case  0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f3=1; break;
        case  8: sc_assert( sc_time_stamp() == sc_time(320, SC_NS) ); f13=1; break;
        default: sc_assert( false ); break;
    }
    
    for (;;)
    {
      wait(ev1 & ev2 & ev3);
      //cout << "target3() awoke at " << sc_time_stamp() << " count = " << count << endl;
      switch (count)
      {
        case  6: sc_assert( sc_time_stamp() == sc_time(140, SC_NS) ); f9=1; break;
        case  7: sc_assert( sc_time_stamp() == sc_time(240, SC_NS) ); f11=1; break;
        case  9: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f15=1; break;
        default: sc_assert( false ); break;
      }
    }
  }

  void target4()
  {
    //cout << "target4() called at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
        case  0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f4=1; break;
        case  6: sc_assert( sc_time_stamp() == sc_time(140, SC_NS) ); f10=1; break;
        case  7: sc_assert( sc_time_stamp() == sc_time(240, SC_NS) ); f12=1; break;
        case  9: sc_assert( sc_time_stamp() == sc_time(330, SC_NS) ); f14=1; break;
        default: sc_assert( false ); break;
    }
    next_trigger(ev1 & ev2 & ev3);
  }
  
  void target5()
  {
    //cout << "target5() called at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
        case  9: sc_assert( sc_time_stamp() == sc_time(400, SC_NS) ); f16=1; break;
        case 10: sc_assert( sc_time_stamp() == sc_time(403, SC_NS) ); f17=1; break;
        case 11: sc_assert( sc_time_stamp() == sc_time(410, SC_NS) ); f18=1; break;
        case 12: sc_assert( sc_time_stamp() == sc_time(413, SC_NS) ); f19=1; break;
        case 14: sc_assert( sc_time_stamp() == sc_time(424, SC_NS) );        break;
        case 15: sc_assert( sc_time_stamp() == sc_time(500, SC_NS) ); f20=1; break;
        case 16: sc_assert( sc_time_stamp() == sc_time(510, SC_NS) ); f22=1; break;
        default: sc_assert( false ); break;
    }
    if (count < 12)
      next_trigger(ev1 & ev2 & ev3);
    else if (count == 12)
      next_trigger(11, SC_NS, ev3);
    else if (count > 12)
      next_trigger();
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
  
  cout << endl << "Success" << endl;
  return 0;
}
  
