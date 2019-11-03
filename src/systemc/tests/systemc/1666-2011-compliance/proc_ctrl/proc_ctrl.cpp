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

// proc_ctrl.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: proc_ctrl.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Basic functionality of process control methods

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using sc_core::sc_process_handle;
using sc_core::sc_event;
using sc_core::sc_get_current_process_handle;
using sc_core::SC_NS;
using sc_core::sc_time;
using sc_core::sc_time_stamp;
using sc_core::sc_stop;
using sc_core::sc_unwind_exception;
using sc_core::sc_is_unwinding;
using sc_core::sc_start;
using sc_core::SC_ZERO_TIME;
using std::cout;
using std::endl;

SC_MODULE(Top)
{
  SC_CTOR(Top)
  : count(0)
  {
    SC_METHOD(gen);
      sensitive << ev;
      
    SC_THREAD(ctrl);
    
    SC_THREAD(target);
      t1 = sc_get_current_process_handle();
      
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
  }
  
  sc_process_handle t1, t2;
  sc_event ev;
  int count;
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  
  void gen()
  {
    ev.notify(10, SC_NS);
  }
 
  void ctrl()
  {
    wait(SC_ZERO_TIME);
    
    count = 1;
    wait(15, SC_NS);

    count = 2;
    t1.suspend();
    wait(20, SC_NS);

    count = 3;
    t1.reset();      // Reset takes priority over suspend
    wait(20, SC_NS);

    count = 4;
    t1.reset();      // Reset takes priority over suspend
    wait(20, SC_NS);

    count = 5;
    t1.resume();
    wait(SC_ZERO_TIME);
    
    count = 6;
    wait(10, SC_NS);

    count = 7;
    wait(10, SC_NS);

    count = 8;
    t1.reset();
    
    count = 9;
    wait(10, SC_NS);

    count = 10;
    wait(10, SC_NS);

    count = 11;
    t1.disable();
    wait(20, SC_NS);

    count = 12;
    t1.reset();      // Reset takes priority over enable

    count = 13;
    wait(20, SC_NS);

    count = 14;
    t1.reset();      // Reset takes priority over enable

    count = 15;
    wait(20, SC_NS);

    count = 16;
    t1.enable();
    wait(SC_ZERO_TIME);
    
    count = 17;
    wait(10, SC_NS);
    
    count = 18;
    wait(10, SC_NS);

    count = 19;
    t1.disable();
    wait(SC_ZERO_TIME);

    count = 20;
    wait(20, SC_NS);

    count = 21;
    t1.suspend();
    wait(20, SC_NS);

    count = 22;
    t1.enable();
    wait(SC_ZERO_TIME);
    
    count = 23;
    wait(20, SC_NS);

    count = 24;
    t1.resume();
    wait(SC_ZERO_TIME);

    count = 25;
    wait(10, SC_NS);

    count = 26;
    wait(10, SC_NS);

    count = 27;
    t1.suspend();
    
    count = 28;
    wait(20, SC_NS);

    count = 29;
    t1.kill();       // kill takes priority over suspend
    wait(20, SC_NS);

    count = 30;
    t1.resume();
    wait(20, SC_NS);
    
    count = 31;
    sc_assert( !sc_is_unwinding() );
    if (t1.valid())
      sc_assert( !t1.is_unwinding() ); 

    sc_stop();
  }
   
  void target()
  {
    sc_assert( !sc_is_unwinding() );
    
      switch(count)
      {
      case  0: sc_assert( sc_time_stamp() == sc_time(0, SC_NS) ); f1=1; break;
      case  3: sc_assert( sc_time_stamp() == sc_time(35, SC_NS) ); f3=1; break;
      case  4: sc_assert( sc_time_stamp() == sc_time(55, SC_NS) ); f4=1; break;
      case  8: sc_assert( sc_time_stamp() == sc_time(95, SC_NS) ); f8=1; break;
      case 12: sc_assert( sc_time_stamp() == sc_time(135, SC_NS) ); f11=1; break;
      case 14: sc_assert( sc_time_stamp() == sc_time(155, SC_NS) ); f12=1; break;
      default: sc_assert( false ); break;
      }
    
    for(;;)
    {
      try {
        wait(ev);

      switch(count)
      {
      case  1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); f2=1; break;
      case  5: sc_assert( sc_time_stamp() == sc_time(75, SC_NS) ); f5=1; break;
      case  6: sc_assert( sc_time_stamp() == sc_time(80, SC_NS) ); f6=1; break;
      case  7: sc_assert( sc_time_stamp() == sc_time(90, SC_NS) ); f7=1; break;
      case  9: sc_assert( sc_time_stamp() == sc_time(100, SC_NS) ); f9=1; break;
      case 10: sc_assert( sc_time_stamp() == sc_time(110, SC_NS) ); f10=1; break;
      case 17: sc_assert( sc_time_stamp() == sc_time(180, SC_NS) ); f13=1; break;
      case 18: sc_assert( sc_time_stamp() == sc_time(190, SC_NS) ); f14=1; break;
      case 24: sc_assert( sc_time_stamp() == sc_time(255, SC_NS) ); f15=1; break;
      case 25: sc_assert( sc_time_stamp() == sc_time(260, SC_NS) ); f16=1; break;
      case 26: sc_assert( sc_time_stamp() == sc_time(270, SC_NS) ); f17=1; break;
      default: sc_assert( false ); break;
      }

      }
      catch (const sc_unwind_exception& e)
      {
        sc_assert( sc_is_unwinding() );

        if (count == 29)
        {
          sc_assert( e.is_reset() == false ); f24=1;
        }       
        switch(count)
        {
        case  3: sc_assert( sc_time_stamp() == sc_time(35, SC_NS) ); f18=1; break;
        case  4: sc_assert( sc_time_stamp() == sc_time(55, SC_NS) ); f19=1; break;
        case  8: sc_assert( sc_time_stamp() == sc_time(95, SC_NS) ); f20=1; break;
        case 12: sc_assert( sc_time_stamp() == sc_time(135, SC_NS) ); f21=1; break;
        case 14: sc_assert( sc_time_stamp() == sc_time(155, SC_NS) ); f22=1; break;
        case 29: sc_assert( sc_time_stamp() == sc_time(295, SC_NS) ); f23=1; break;
        default: sc_assert( false ); break;
        }

        throw e;
      }
    }
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  sc_start();
  
  sc_assert(top.f1);
  sc_assert(top.f2);
  sc_assert(top.f3);
  sc_assert(top.f4);
  sc_assert(top.f5);
  sc_assert(top.f6);
  sc_assert(top.f7);
  sc_assert(top.f8);
  sc_assert(top.f9);
  sc_assert(top.f10);
  sc_assert(top.f11);
  sc_assert(top.f12);
  sc_assert(top.f13);
  sc_assert(top.f14);
  sc_assert(top.f15);
  sc_assert(top.f16);
  sc_assert(top.f17);
  sc_assert(top.f18);
  sc_assert(top.f19);
  sc_assert(top.f20);
  sc_assert(top.f21);
  sc_assert(top.f22);
  sc_assert(top.f23);
  sc_assert(top.f24);
  /*
  sc_assert(top.f25);
  sc_assert(top.f26);
  sc_assert(top.f27);
  sc_assert(top.f28);
  sc_assert(top.f29);
  */

  cout << endl << "Success" << endl;
  return 0;
}
