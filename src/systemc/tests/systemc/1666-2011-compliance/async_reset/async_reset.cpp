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

// async_reset.cpp
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: async_reset.cpp,v $
// Revision 1.3  2011/05/08 19:18:45  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// async_reset_signal_is

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

struct Top: sc_module
{
  Top(sc_module_name _name)
  : count(0)
  {
    clk_port.bind(clk);
    
    SC_THREAD(ctrl);
    
    SC_CTHREAD(CT, clk_port.value_changed());
      async_reset_signal_is(areset, true);
      ct = sc_get_current_process_handle();
    
    sc_spawn_options opt;
    opt.async_reset_signal_is(areset, true);
    t = sc_spawn(sc_bind(&Top::T, this), "T", &opt);
  
    opt.spawn_method();
    opt.dont_initialize();
    opt.set_sensitivity( &clk );
    m = sc_spawn(sc_bind(&Top::M, this), "M", &opt);
    
    SC_CTHREAD(CT2, clk_port.pos())
      async_reset_signal_is(areset1, true);
      async_reset_signal_is(areset2, true);
      async_reset_signal_is(areset3, true);
      reset_signal_is(sreset1, true);
      reset_signal_is(sreset2, true);
      reset_signal_is(sreset3, true);
      ct2 = sc_get_current_process_handle();
      ct2.disable();
    
    SC_THREAD(T2)
      sensitive << clk_port.pos();
      async_reset_signal_is(areset1, true);
      async_reset_signal_is(areset2, true);
      async_reset_signal_is(areset3, true);
      reset_signal_is(sreset1, true);
      reset_signal_is(sreset2, true);
      reset_signal_is(sreset3, true);
      t2 = sc_get_current_process_handle();
      t2.disable();
    
    clk.write(false);
    areset.write(false);
    
    f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
    f30 = f31 = f32 = f33 = f34 = f35 = f36 = f37 = f38 = f39 = 0;
    f40 = f41 = f42 = f43 = f44 = f45 = f46 = f47 = f48 = f49 = 0;
    f50 = f51 = f52 = f53 = f54 = f55 = f56 = f57 = f58 = f59 = 0;
    f60 = f61 = f62 = f63 = f64 = f65 = f66 = f67 = f68 = f69 = 0;
    f70 = f71 = f72 = f73 = f74 = f75 = f76 = f77 = f78 = f79 = 0;
    f80 = f81 = f82 = f83 = f84 = f85 = f86 = f87 = f88 = f89 = 0;
  }

  sc_signal<bool> clk;
  sc_in<bool> clk_port;
  sc_signal<bool> areset;
  sc_signal<bool> areset1, areset2, areset3, sreset1, sreset2, sreset3;
  
  sc_process_handle ct, t, m, ct2, t2;
  int count;
  
  int f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  int f30, f31, f32, f33, f34, f35, f36, f37, f38, f39;
  int f40, f41, f42, f43, f44, f45, f46, f47, f48, f49;
  int f50, f51, f52, f53, f54, f55, f56, f57, f58, f59;
  int f60, f61, f62, f63, f64, f65, f66, f67, f68, f69;
  int f70, f71, f72, f73, f74, f75, f76, f77, f78, f79;
  int f80, f81, f82, f83, f84, f85, f86, f87, f88, f89;
  
  void ctrl()
  {
    sc_assert( sc_delta_count() == 0 );
    clock();
    wait(SC_ZERO_TIME);
    wait(SC_ZERO_TIME);
    
    count = 1;
    clock();
    wait(1, SC_NS);
    
    count = 2;
    clock();
    wait(10, SC_NS);
    
    count = 3;
    clock();
    wait(sc_time(20, SC_NS) - sc_time_stamp());
    
    count = 4;
    areset.write(true);
    wait(1, SC_NS);

    count = 5;
    areset.write(false);
    wait(1, SC_NS);
    
    count = 6;
    clock();
    wait(sc_time(30, SC_NS) - sc_time_stamp());

    count = 7;
    areset.write(true);
    wait(1, SC_NS);

    count = 8;
    clock();      // Clocked while asynch reset is active
    wait(1, SC_NS);

    count = 9;
    clock();
    wait(1, SC_NS);

    count = 10;
    areset.write(false);
    wait(1, SC_NS);

    count = 11;
    areset.write(true);
    wait(1, SC_NS);

    count = 12;
    areset.write(false);
    wait(sc_time(40, SC_NS) - sc_time_stamp());

    count = 13;
    sync_reset_on();
    wait(1, SC_NS);

    count = 14;
    clock();
    wait(1, SC_NS);

    count = 15;
    sync_reset_off();
    wait(sc_time(50, SC_NS) - sc_time_stamp());

    count = 16;
    disable();
    wait(1, SC_NS);

    count = 17;
    clock();
    wait(1, SC_NS);

    count = 18;
    sync_reset_on();
    clock();
    wait(1, SC_NS);
    
    count = 19;
    enable();
    clock();
    wait(1, SC_NS);
    
    count = 20;
    sync_reset_off();
    clock();
    wait(sc_time(60, SC_NS) - sc_time_stamp());

    count = 21;
    disable();
    wait(1, SC_NS);

    count = 22;
    areset.write(true);
    wait(1, SC_NS);

    count = 23;
    clock();
    wait(1, SC_NS);

    count = 24;
    areset.write(false);
    wait(1, SC_NS);
    
    count = 25;
    clock();
    wait(1, SC_NS);

    count = 26;
    areset.write(true);
    wait(1, SC_NS);

    count = 27;
    enable();
    wait(1, SC_NS);

    count = 28;
    clock();
    wait(1, SC_NS);

    count = 29;
    areset.write(false);
    wait(sc_time(100, SC_NS) - sc_time_stamp());
    
    count = 30;
    ct.disable();
    t.disable();
    m.disable();
    
    // Test multiple resets
    ct2.enable();
    t2.enable();
    clock2();

    count = 31;
    clock2();

    count = 32;
    sreset1.write(1);
    clock2();
    
    count = 33;
    sreset2.write(1);
    sreset3.write(1);
    clock2();
    
    count = 34;
    sreset1.write(0);
    sreset2.write(0);
    clock2();
   
    count = 35;
    sreset3.write(0);
    clock2();
   
    count = 36;
    areset1.write(1);
    areset2.write(1);
    areset3.write(1);
    wait(SC_ZERO_TIME);
    
    count = 37;
    clock2();
    
    count = 38;
    sreset1.write(1);
    sreset2.write(1);
    sreset3.write(1);
    ct2.sync_reset_on();
    t2.sync_reset_on();
    clock2();
    
    count = 39;
    areset1.write(0);
    areset2.write(0);
    areset3.write(0);
    sreset1.write(0);
    sreset2.write(0);
    sreset3.write(0);
    clock2();
    
    count = 40;
    ct2.sync_reset_off();
    t2.sync_reset_off();
    clock2();
    
    count = 41;
    sreset2.write(1);
    ct2.sync_reset_on();
    t2.sync_reset_on();
    clock2();
    
    count = 42;
    areset2.write(1);
    wait(SC_ZERO_TIME);

    count = 43;
    clock2();
    
    count = 44;
    sreset2.write(0);
    ct2.sync_reset_off();
    t2.sync_reset_off();
    clock2();
    
    count = 45;
    areset1.write(0);
    areset2.write(0);
    areset3.write(0);
    sreset1.write(0);
    sreset2.write(0);
    sreset3.write(0);
    clock2();
    
    count = 46;
    ct2.reset();
    t2.reset();
    wait(SC_ZERO_TIME);
    
    count = 47;
    clock2();
  }
  
  void CT()
  {
    //cout << "CT() called at " << sc_time_stamp() << endl;
    switch (count)
    {
      case  0: sc_assert( sc_delta_count() == 1 ); f1=1; break;
      case  4: sc_assert( sc_time_stamp() == sc_time(20, SC_NS) ); f2=1; break;
      case  7: sc_assert( sc_time_stamp() == sc_time(30, SC_NS) ); f3=1; break;
      case  8: sc_assert( sc_time_stamp() == sc_time(31, SC_NS) ); f4=1; break;
      case  9: sc_assert( sc_time_stamp() == sc_time(32, SC_NS) ); f5=1; break;
      case 11: sc_assert( sc_time_stamp() == sc_time(34, SC_NS) ); f6=1; break;
      case 14: sc_assert( sc_time_stamp() == sc_time(41, SC_NS) ); f7=1; break;
      case 19: sc_assert( sc_time_stamp() == sc_time(53, SC_NS) ); f8=1; break;
      case 22: sc_assert( sc_time_stamp() == sc_time(61, SC_NS) ); f9=1; break;
      case 26: sc_assert( sc_time_stamp() == sc_time(65, SC_NS) ); f11=1; break;
      case 28: sc_assert( sc_time_stamp() == sc_time(67, SC_NS) ); f12=1; break;
      default: sc_assert( false ); break;
    }
    while (true)
    {
      wait();
      //cout << "CT() awoke at " << sc_time_stamp() << endl;

    switch (count)
    {
      case  1: sc_assert( sc_delta_count() == 3 ); f13=1; break;
      case  2: sc_assert( sc_time_stamp() == sc_time(1, SC_NS) ); f14=1; break;
      case  3: sc_assert( sc_time_stamp() == sc_time(11, SC_NS) ); f15=1; break;
      case  6: sc_assert( sc_time_stamp() == sc_time(22, SC_NS) ); f16=1; break;
      case 20: sc_assert( sc_time_stamp() == sc_time(54, SC_NS) ); f17=1; break;
      default: sc_assert( false ); break;
    }

    }
  }

  void T()
  {
    //cout << "T() called at " << sc_time_stamp() << endl;
    switch (count)
    {
      case  0: sc_assert( sc_delta_count() == 0 ); f18=1; break;
      case  4: sc_assert( sc_time_stamp() == sc_time(20, SC_NS) ); f19=1; break;
      case  7: sc_assert( sc_time_stamp() == sc_time(30, SC_NS) ); f20=1; break;
      case  8: sc_assert( sc_time_stamp() == sc_time(31, SC_NS) ); f21=1; break;
      case  9: sc_assert( sc_time_stamp() == sc_time(32, SC_NS) ); f22=1; break;
      case 11: sc_assert( sc_time_stamp() == sc_time(34, SC_NS) ); f23=1; break;
      case 14: sc_assert( sc_time_stamp() == sc_time(41, SC_NS) ); f24=1; break;
      case 19: sc_assert( sc_time_stamp() == sc_time(53, SC_NS) ); f25=1; break;
      case 22: sc_assert( sc_time_stamp() == sc_time(61, SC_NS) ); f26=1; break;
      case 26: sc_assert( sc_time_stamp() == sc_time(65, SC_NS) ); f28=1; break;
      case 28: sc_assert( sc_time_stamp() == sc_time(67, SC_NS) ); f29=1; break;
      default: sc_assert( false ); break;
    }
    while (true)
    {
      wait(clk.default_event());
      //cout << "T() awoke at " << sc_time_stamp() << endl;

    switch (count)
    {
      case  0: sc_assert( sc_delta_count() == 1 ); f30=1; break;
      case  1: sc_assert( sc_delta_count() == 3 ); f31=1; break;
      case  2: sc_assert( sc_time_stamp() == sc_time(1, SC_NS) ); f32=1; break;
      case  3: sc_assert( sc_time_stamp() == sc_time(11, SC_NS) ); f33=1; break;
      case  6: sc_assert( sc_time_stamp() == sc_time(22, SC_NS) ); f34=1; break;
      case 20: sc_assert( sc_time_stamp() == sc_time(54, SC_NS) ); f35=1; break;
      default: sc_assert( false ); break;
    }

    }
  }
   
  void M()
  {
    //cout << "M() called at " << sc_time_stamp() << endl;
    switch (count)
    {
      case  0: sc_assert( sc_delta_count() == 1 ); f36=1; break;
      case  1: sc_assert( sc_delta_count() == 3 ); f37=1; break;
      case  2: sc_assert( sc_time_stamp() == sc_time(1, SC_NS) ); f38=1; break;
      case  3: sc_assert( sc_time_stamp() == sc_time(11, SC_NS) ); f39=1; break;
      case  4: sc_assert( sc_time_stamp() == sc_time(20, SC_NS) ); f83=1; break;
      case  6: sc_assert( sc_time_stamp() == sc_time(22, SC_NS) ); f40=1; break;
      case  7: sc_assert( sc_time_stamp() == sc_time(30, SC_NS) ); f84=1; break;
      case  8: sc_assert( sc_time_stamp() == sc_time(31, SC_NS) ); f41=1; break;
      case  9: sc_assert( sc_time_stamp() == sc_time(32, SC_NS) ); f42=1; break; 
      case 11: sc_assert( sc_time_stamp() == sc_time(34, SC_NS) ); f85=1; break;
      case 14: sc_assert( sc_time_stamp() == sc_time(41, SC_NS) ); f43=1; break;
      case 19: sc_assert( sc_time_stamp() == sc_time(53, SC_NS) ); f44=1; break;
      case 20: sc_assert( sc_time_stamp() == sc_time(54, SC_NS) ); f45=1; break;
      case 22: sc_assert( sc_time_stamp() == sc_time(61, SC_NS) ); f26=1; break;
      case 26: sc_assert( sc_time_stamp() == sc_time(65, SC_NS) ); f86=1; break;
      case 28: sc_assert( sc_time_stamp() == sc_time(67, SC_NS) ); f46=1; break;
      default: sc_assert( false ); break;
    }
  }

  void CT2()
  {
    //cout << "CT2() called at " << sc_time_stamp() << endl;
    switch (count)
    {
        case 30: sc_assert( sc_time_stamp() == sc_time(105, SC_NS) ); f47=1; break;
        case 32: sc_assert( sc_time_stamp() == sc_time(125, SC_NS) ); f51=1; break;
        case 33: sc_assert( sc_time_stamp() == sc_time(135, SC_NS) ); f53=1; break;
        case 34: sc_assert( sc_time_stamp() == sc_time(145, SC_NS) ); f55=1; break;
        case 36: sc_assert( sc_time_stamp() == sc_time(160, SC_NS) ); f59=1; break;
        case 37: sc_assert( sc_time_stamp() == sc_time(165, SC_NS) ); f61=1; break;
        case 38: sc_assert( sc_time_stamp() == sc_time(175, SC_NS) ); f63=1; break;
        case 39: sc_assert( sc_time_stamp() == sc_time(185, SC_NS) ); f65=1; break;
        case 41: sc_assert( sc_time_stamp() == sc_time(205, SC_NS) ); f69=1; break;
        case 42: sc_assert( sc_time_stamp() == sc_time(210, SC_NS) ); f71=1; break;
        case 43: sc_assert( sc_time_stamp() == sc_time(215, SC_NS) ); f73=1; break;
        case 44: sc_assert( sc_time_stamp() == sc_time(225, SC_NS) ); f75=1; break;
        case 46: sc_assert( sc_time_stamp() == sc_time(240, SC_NS) ); f79=1; break;
        default: sc_assert( false ); break;
    }
    while (true)
    {
      wait();
      //cout << "CT2() awoke at " << sc_time_stamp() << endl;
      switch (count)
      {
        case 31: sc_assert( sc_time_stamp() == sc_time(115, SC_NS) ); f49=1; break;
        case 35: sc_assert( sc_time_stamp() == sc_time(155, SC_NS) ); f57=1; break;
        case 40: sc_assert( sc_time_stamp() == sc_time(195, SC_NS) ); f67=1; break;
        case 45: sc_assert( sc_time_stamp() == sc_time(235, SC_NS) ); f77=1; break;
        case 47: sc_assert( sc_time_stamp() == sc_time(245, SC_NS) ); f81=1; break;
        default: sc_assert( false ); break;
      }
    }
  }
  
  void T2()
  {
    //cout << "T2() called at " << sc_time_stamp() << endl;
    switch (count)
    {
        case 30: sc_assert( sc_time_stamp() == sc_time(105, SC_NS) ); f48=1; break;
        case 32: sc_assert( sc_time_stamp() == sc_time(125, SC_NS) ); f52=1; break;
        case 33: sc_assert( sc_time_stamp() == sc_time(135, SC_NS) ); f54=1; break;
        case 34: sc_assert( sc_time_stamp() == sc_time(145, SC_NS) ); f56=1; break;
        case 36: sc_assert( sc_time_stamp() == sc_time(160, SC_NS) ); f60=1; break;
        case 37: sc_assert( sc_time_stamp() == sc_time(165, SC_NS) ); f62=1; break;
        case 38: sc_assert( sc_time_stamp() == sc_time(175, SC_NS) ); f64=1; break;
        case 39: sc_assert( sc_time_stamp() == sc_time(185, SC_NS) ); f66=1; break;
        case 41: sc_assert( sc_time_stamp() == sc_time(205, SC_NS) ); f70=1; break;
        case 42: sc_assert( sc_time_stamp() == sc_time(210, SC_NS) ); f72=1; break;
        case 43: sc_assert( sc_time_stamp() == sc_time(215, SC_NS) ); f74=1; break;
        case 44: sc_assert( sc_time_stamp() == sc_time(225, SC_NS) ); f76=1; break;
        case 46: sc_assert( sc_time_stamp() == sc_time(240, SC_NS) ); f80=1; break;
        default: sc_assert( false ); break;
    }
    while (true)
    {
      wait();
      //cout << "T2() awoke at " << sc_time_stamp() << endl;
      switch (count)
      {
        case 31: sc_assert( sc_time_stamp() == sc_time(115, SC_NS) ); f50=1; break;
        case 35: sc_assert( sc_time_stamp() == sc_time(155, SC_NS) ); f58=1; break;
        case 40: sc_assert( sc_time_stamp() == sc_time(195, SC_NS) ); f68=1; break;
        case 45: sc_assert( sc_time_stamp() == sc_time(235, SC_NS) ); f78=1; break;
        case 47: sc_assert( sc_time_stamp() == sc_time(245, SC_NS) ); f82=1; break;
        default: sc_assert( false ); break;
      }
    }
  }
  
  void clock()
  {
    clk.write( !clk.read() );
  } 
  
  void clock2()
  {
    clk.write(0);
    wait(5, SC_NS);
    clk.write(1);
    wait(5, SC_NS);
  } 
  
  void suspend()
  {
    ct.suspend();
    t.suspend();
    m.suspend();
  }  
   
  void resume()
  {
    ct.resume();
    t.resume();
    m.resume();
  }  
   
  void disable()
  {
    ct.disable();
    t.disable();
    m.disable();
  }  
   
  void enable()
  {
    ct.enable();
    t.enable();
    m.enable();
  }
    
  void sync_reset_on()
  {
    ct.sync_reset_on();
    t.sync_reset_on();
    m.sync_reset_on();
  }  

  void sync_reset_off()
  {
    ct.sync_reset_off();
    t.sync_reset_off();
    m.sync_reset_off();
  }  
    
  SC_HAS_PROCESS(Top);
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
  sc_assert(top.f25);  
  sc_assert(top.f26);  
  sc_assert(top.f28);  
  sc_assert(top.f29);  
  sc_assert(top.f30);  
  sc_assert(top.f31);  
  sc_assert(top.f32);  
  sc_assert(top.f33);  
  sc_assert(top.f34);  
  sc_assert(top.f35);  
  sc_assert(top.f36);  
  sc_assert(top.f37);  
  sc_assert(top.f38);  
  sc_assert(top.f39);  
  sc_assert(top.f40);  
  sc_assert(top.f41);  
  sc_assert(top.f42);  
  sc_assert(top.f43);  
  sc_assert(top.f44);  
  sc_assert(top.f45);  
  sc_assert(top.f46);  
  sc_assert(top.f47); 
  sc_assert(top.f48);  
  sc_assert(top.f49);  
  sc_assert(top.f50);  
  sc_assert(top.f51);  
  sc_assert(top.f52);  
  sc_assert(top.f53);  
  sc_assert(top.f54);  
  sc_assert(top.f55);  
  sc_assert(top.f56);  
  sc_assert(top.f57); 
  sc_assert(top.f58);  
  sc_assert(top.f59);  
  sc_assert(top.f60);  
  sc_assert(top.f61);  
  sc_assert(top.f62);  
  sc_assert(top.f63);  
  sc_assert(top.f64);  
  sc_assert(top.f65);  
  sc_assert(top.f66);  
  sc_assert(top.f67); 
  sc_assert(top.f68); 
  sc_assert(top.f69);  
  sc_assert(top.f70);  
  sc_assert(top.f71);  
  sc_assert(top.f72);  
  sc_assert(top.f73);  
  sc_assert(top.f74);  
  sc_assert(top.f75);  
  sc_assert(top.f76);  
  sc_assert(top.f77); 
  sc_assert(top.f78); 
  sc_assert(top.f79);  
  sc_assert(top.f80);  
  sc_assert(top.f81);  
  sc_assert(top.f82);  
  sc_assert(top.f83);  
  sc_assert(top.f84);  
  sc_assert(top.f85);  
  sc_assert(top.f86);  

  cout << endl << "Success" << endl;
  return 0;
}
