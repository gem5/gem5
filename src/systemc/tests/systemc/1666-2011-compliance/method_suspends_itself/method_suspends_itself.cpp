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

// method_suspends_itself.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: method_suspends_itself.cpp,v $
// Revision 1.3  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// Method process uses suspends, resumes, disables, and enables itself

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);

    SC_METHOD(target);
      sensitive << ev;
      dont_initialize();
      t = sc_get_current_process_handle();

    suspend_target = false;
    resume_target  = false;
    disable_target = false;
    enable_target  = false;
    dynamic_sensitivity = false;
    f0 = f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
  }
  
  sc_process_handle t;
  sc_event ev, ev2;
  bool suspend_target;
  bool resume_target;
  bool disable_target;
  bool enable_target;
  bool dynamic_sensitivity;
  int  count;
  int f0, f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;

  void ticker()
  {
    for (;;)
    {
      wait(10, SC_NS);
      ev.notify();
    }
  }
   
  void calling()
  {
    count = 1;
    wait(15, SC_NS);
    // Target runs at 10 NS
    
    count = 2;
    suspend_target = true;
    wait(10, SC_NS);
    // Target runs at 20 NS and suspends itself
    
    count = 3;
    suspend_target = false;
    wait(10, SC_NS);
    // Target does not run at 30 NS
    
    count = 4;
    t.resume();
    // Target runs at 35 NS
    
    wait(10, SC_NS);
    // Target runs at 40 NS  

    count = 6;
    suspend_target = true;
    resume_target = true;
    wait(10, SC_NS);
    // Target runs at 50 NS

    count = 7;
    resume_target = true;
    wait(10, SC_NS);
    // Double resume at 60 NS    
    
    count = 8;
    suspend_target = true;
    resume_target = false;
    wait(10, SC_NS);
    // Target runs at 70 NS
    
    count = 9;
    wait(10, SC_NS);
    // Double suspend
    // Target does not run at 80 NS
    
    count = 10;
    suspend_target = false;
    resume_target = false;
    t.resume();
    // Target runs at 85 NS
    
    wait(10, SC_NS);
    // Target runs at 90 NS
    sc_assert( count == 11 );
    
    count = 12;
    t.suspend();
    wait(10, SC_NS);
    // Target does not run at 100 NS
    
    count = 13;
    t.resume();
    // Target runs at 105 NS
    
    wait(10, SC_NS);
    // Target runs at 110 NS
    sc_assert( count == 14 );
    
    count = 15;
    t.disable();
    wait(10, SC_NS);
    // Target does not run at 120 NS
    
    count = 16;
    wait(10, SC_NS);
    // Target does not run at 130 NS
    
    count = 17;
    t.disable();
    // Double disable
    wait(10, SC_NS);
    // Target does not run at 140 NS
    
    count = 18;
    t.enable();
    wait(10, SC_NS);
    // Target runs at 150 NS
    
    count = 19;
    t.enable();
    // Double enable
    wait(10, SC_NS);
    // Target runs at 160 NS

    count = 20;
    disable_target = true;
    wait(10, SC_NS);
    // Target runs at 170 and disables itself
    
    count = 21;
    disable_target = false;
    wait(10, SC_NS);
    // Target does not run at 180
    
    count = 22;
    enable_target = true;
    wait(10, SC_NS);
    // Target does not run at 190 
    
    count = 23;
    wait(10, SC_NS);
    // Failed to enable it itself, so still does not run at 200
    
    count = 24;
    enable_target = false;
    t.enable();
    wait(10, SC_NS);
    // Target runs at 210
    
    count = 25;
    disable_target = true;
    enable_target = true;
    wait(10, SC_NS);
    // Target runs at 220 and calls disable -> enable
    
    count = 26;
    disable_target = false;
    enable_target = false;
    wait(10, SC_NS);
    // Target runs at 230
    
    count = 27;
    t.suspend();
    wait(10, SC_NS);
    // Target does not run at 240
    
    count = 28;
    t.enable();
    wait(10, SC_NS);
    // Has no effect - still suspended at 250

    count = 29;
    t.disable();
    wait(10, SC_NS);
    // Both disabled and suspended at 260

    count = 30;
    t.enable();
    wait(10, SC_NS);
    // Enabled but still suspended
    
    count = 31;
    t.resume();
    // Target resumed at 275 NS
    wait(SC_ZERO_TIME);
    
    count = 311;
    wait(10, SC_NS);
    
    count = 32;
    t.disable();
    wait(10, SC_NS);
    // Disabled at 290 NS
    
    count = 33;
    t.resume();
    wait(10, SC_NS);
    // Still disabled at 300 NS
        
    count = 34;
    t.suspend();
    wait(10, SC_NS);
    // Both disabled and suspended at 310
    
    count = 35;
    t.enable();
    wait(10, SC_NS);
    // Remains suspended at 320
    
    count = 36;
    t.resume();
    wait(10, SC_NS);
    // Resumed at 325 NS and runs at 330
    sc_assert( count == 37 );
    
    count = 38;
    suspend_target = true;
    resume_target = true;
    disable_target = true;
    enable_target = true;
    wait(10, SC_NS);
    // Runs at 340
    
    count = 39;
    suspend_target = true;
    resume_target = false;
    disable_target = true;
    enable_target = true;
    wait(10, SC_NS);
    // Runs at 350, when it suspends

    count = 40;
    suspend_target = false;
    wait(10, SC_NS);
    // Suspended at 360
    
    count = 41;
    t.resume();
    wait(10, SC_NS);
    // Runs at 365 and 370
    sc_assert( count == 42 );
    
    sc_assert( t.valid() );
    sc_assert( t.terminated() == false );
    sc_assert( t.dynamic() == false );
    sc_assert( t.get_parent_object() == this );
    sc_assert( t.get_process_object() != 0 );
    
    count = 43;
    t.reset();
    wait(SC_ZERO_TIME);

    count = 44;
    dynamic_sensitivity = true;
    wait(10, SC_NS);
    // Runs at 380
    
    count = 45;
    ev2.notify();
    wait(1, SC_NS);
        
    count = 46;
    ev2.notify();
    wait(sc_time(400, SC_NS) - sc_time_stamp());
        
    count = 47;
    dynamic_sensitivity = false;
    t.sync_reset_on(); // Still dynamically sensitive to ev2
    ev2.notify(); // Clears dynamic sensitivity, restores static sensitivity
    
    count = 48;
    wait(10, SC_NS);
    
    count = 49;
    t.kill();
 
    if (t.valid())   
      sc_assert( t.terminated() );
    
    sc_stop();
  }

  void target()
  {
    //cout << "Target called at " << sc_time_stamp() << " count = " << count << endl;
    switch (count)
    {
    case 1: sc_assert( sc_time_stamp() == sc_time(10, SC_NS) ); f0=1; break;
    case 2: sc_assert( sc_time_stamp() == sc_time(20, SC_NS) ); f1=1; break;
    case 4: sc_assert( sc_time_stamp() == sc_time(35, SC_NS) ); count = 5; f2=1; break;
    case 5: sc_assert( sc_time_stamp() == sc_time(40, SC_NS) ); f3=1; break;
    case 6: sc_assert( sc_time_stamp() == sc_time(50, SC_NS) ); f4=1; break;
    case 7: sc_assert( sc_time_stamp() == sc_time(60, SC_NS) ); f5=1; break;
    case 8: sc_assert( sc_time_stamp() == sc_time(70, SC_NS) ); f6=1; break;
    case 10: sc_assert( sc_time_stamp() == sc_time(85, SC_NS) ); count = 11; f7=1; break;
    case 11: sc_assert( sc_time_stamp() == sc_time(90, SC_NS) ); f8=1; break;
    case 13: sc_assert( sc_time_stamp() == sc_time(105, SC_NS) ); count = 14; f9=1; break;
    case 14: sc_assert( sc_time_stamp() == sc_time(110, SC_NS) ); f10=1; break;
    case 18: sc_assert( sc_time_stamp() == sc_time(150, SC_NS) ); f11=1; break;
    case 19: sc_assert( sc_time_stamp() == sc_time(160, SC_NS) ); f12=1; break;
    case 20: sc_assert( sc_time_stamp() == sc_time(170, SC_NS) ); f13=1; break;
    case 24: sc_assert( sc_time_stamp() == sc_time(210, SC_NS) ); f14=1; break;
    case 25: sc_assert( sc_time_stamp() == sc_time(220, SC_NS) ); f15=1; break;
    case 26: sc_assert( sc_time_stamp() == sc_time(230, SC_NS) ); f16=1; break;
    case 31: sc_assert( sc_time_stamp() == sc_time(275, SC_NS) ); f17=1; break;
    case 311:sc_assert( sc_time_stamp() == sc_time(280, SC_NS) ); f18=1; break;
    case 36: sc_assert( sc_time_stamp() == sc_time(325, SC_NS) ); count = 37; f19=1; break;
    case 37: sc_assert( sc_time_stamp() == sc_time(330, SC_NS) ); f20=1; break;
    case 38: sc_assert( sc_time_stamp() == sc_time(340, SC_NS) ); f21=1; break;
    case 39: sc_assert( sc_time_stamp() == sc_time(350, SC_NS) ); f22=1; break;
    case 41: sc_assert( sc_time_stamp() == sc_time(365, SC_NS) ); count = 42; f23=1; break;
    case 42: sc_assert( sc_time_stamp() == sc_time(370, SC_NS) ); f24=1; break;
    case 43: sc_assert( sc_time_stamp() == sc_time(375, SC_NS) ); f29=1; break; ////////
    case 44: sc_assert( sc_time_stamp() == sc_time(380, SC_NS) ); f25=1; break;
    case 45: sc_assert( sc_time_stamp() == sc_time(385, SC_NS) ); f26=1; break;
    case 46: sc_assert( sc_time_stamp() == sc_time(386, SC_NS) ); f27=1; break;
    case 48: sc_assert( sc_time_stamp() == sc_time(400, SC_NS) ); f28=1; break;
    default: sc_assert( false ); break;
    }
    
    if (suspend_target)
      t.suspend();
    if (resume_target)
      t.resume();
    if (disable_target)
      t.disable();
    if (enable_target)
      t.enable();
    if (dynamic_sensitivity)
      next_trigger(ev2);
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
  sc_assert( top.f28 );
  sc_assert( top.f29 );

  cout << endl << "Success" << endl;
  return 0;
}
  
