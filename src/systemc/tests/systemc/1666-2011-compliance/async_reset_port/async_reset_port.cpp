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

// async_reset_port.cpp -- 
//
//  Original Author: John Aynsley, Doulos Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: async_reset_port.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// async_reset_signal_is

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

struct M: sc_module
{
  sc_in<bool>    clk;
  
  sc_in<bool>    sreset1;
  sc_inout<bool> sreset2;
  sc_out<bool>   sreset3;
  
  sc_in<bool>    areset1;
  sc_inout<bool> areset2;
  sc_out<bool>   areset3;
  
  M(sc_module_name _name)
  : count(0)
  {
    SC_CTHREAD(CT1, clk);
      reset_signal_is(sreset1, false);
      ct1 = sc_get_current_process_handle();

    SC_CTHREAD(CT2, clk);
      reset_signal_is(sreset2, false);
      ct2 = sc_get_current_process_handle();

    SC_CTHREAD(CT3, clk);
      reset_signal_is(sreset3, false);
      ct3 = sc_get_current_process_handle();

    SC_CTHREAD(CT4, clk);
      async_reset_signal_is(areset1, false);
      ct4 = sc_get_current_process_handle();

    SC_CTHREAD(CT5, clk);
      async_reset_signal_is(areset2, false);
      ct5 = sc_get_current_process_handle();

    SC_CTHREAD(CT6, clk);
      async_reset_signal_is(areset3, false);
      ct6 = sc_get_current_process_handle();

    f1 = f2 = f3 = f4 = f5 = f6 = f7 = f8 = f9 = 0;
    f10 = f11 = f12 = f13 = f14 = f15 = f16 = f17 = f18 = f19 = 0;
    f20 = f21 = f22 = f23 = f24 = f25 = f26 = f27 = f28 = f29 = 0;
    f30 = f31 = f32 = f33 = f34 = f35 = f36 = f37 = f38 = f39 = 0;
  }

  int count;
  sc_process_handle ct1, ct2, ct3, ct4, ct5, ct6;
  
  int f1, f2, f3, f4, f5, f6, f7, f8, f9;
  int f10, f11, f12, f13, f14, f15, f16, f17, f18, f19;
  int f20, f21, f22, f23, f24, f25, f26, f27, f28, f29;
  int f30, f31, f32, f33, f34, f35, f36, f37, f38, f39;
  
  void CT1()
  {
    if (count ==  2) { sc_assert(sc_time_stamp() == sc_time( 15, SC_NS));  f1 = 1; }
    if (count == 17) { sc_assert(false); }
    if (count == 18) { sc_assert(sc_time_stamp() == sc_time(135, SC_NS)); f16 = 1; }
    if (count == 19) { sc_assert(sc_time_stamp() == sc_time(145, SC_NS)); f25 = 1; }
    while (true)
    {
      wait();
      if (count == 16) { sc_assert(sc_time_stamp() == sc_time(125, SC_NS)); f10 = 1; }
      if (count == 20) { sc_assert(sc_time_stamp() == sc_time(155, SC_NS)); f31 = 1; }
    }
  }
  
  void CT2()
  {
    if (count ==  4) { sc_assert(sc_time_stamp() == sc_time( 35, SC_NS));  f2 = 1; }
    if (count == 17) { sc_assert(false); }
    if (count == 18) { sc_assert(sc_time_stamp() == sc_time(135, SC_NS)); f17 = 1; }
    if (count == 19) { sc_assert(sc_time_stamp() == sc_time(145, SC_NS)); f26 = 1; }
    while (true)
    {
      wait();
      if (count == 16) { sc_assert(sc_time_stamp() == sc_time(125, SC_NS)); f11 = 1; }
      if (count == 20) { sc_assert(sc_time_stamp() == sc_time(155, SC_NS)); f32 = 1; }
    }
  }
  
  void CT3()
  {
    if (count ==  6) { sc_assert(sc_time_stamp() == sc_time( 55, SC_NS));  f3 = 1; }
    if (count == 17) { sc_assert(false); }
    if (count == 18) { sc_assert(sc_time_stamp() == sc_time(135, SC_NS)); f18 = 1; }
    if (count == 19) { sc_assert(sc_time_stamp() == sc_time(145, SC_NS)); f27 = 1; }
    while (true)
    {
      wait();
      if (count == 16) { sc_assert(sc_time_stamp() == sc_time(125, SC_NS)); f12 = 1; }
      if (count == 20) { sc_assert(sc_time_stamp() == sc_time(155, SC_NS)); f33 = 1; }
    }
  }
  
  void CT4()
  {
    if (count ==  8) { sc_assert(sc_time_stamp() == sc_time( 70, SC_NS));  f4 = 1; }
    if (count ==  9) { sc_assert(sc_time_stamp() == sc_time( 75, SC_NS));  f5 = 1; }
    if (count == 17) { sc_assert(sc_time_stamp() == sc_time(130, SC_NS)); f19 = 1; }
    if (count == 18) { sc_assert(sc_time_stamp() == sc_time(135, SC_NS)); f22 = 1; }
    if (count == 19) { sc_assert(sc_time_stamp() == sc_time(145, SC_NS)); f28 = 1; }
    while (true)
    {
      wait();
      if (count == 16) { sc_assert(sc_time_stamp() == sc_time(125, SC_NS)); f13 = 1; }
      if (count == 20) { sc_assert(sc_time_stamp() == sc_time(155, SC_NS)); f34 = 1; }
    }
  }
  
  void CT5()
  {
    if (count == 11) { sc_assert(sc_time_stamp() == sc_time( 90, SC_NS));  f6 = 1; }
    if (count == 12) { sc_assert(sc_time_stamp() == sc_time( 95, SC_NS));  f7 = 1; }
    if (count == 17) { sc_assert(sc_time_stamp() == sc_time(130, SC_NS)); f20 = 1; }
    if (count == 18) { sc_assert(sc_time_stamp() == sc_time(135, SC_NS)); f23 = 1; }
    if (count == 19) { sc_assert(sc_time_stamp() == sc_time(145, SC_NS)); f29 = 1; }
    while (true)
    {
      wait();
      if (count == 16) { sc_assert(sc_time_stamp() == sc_time(125, SC_NS)); f14 = 1; }
      if (count == 20) { sc_assert(sc_time_stamp() == sc_time(155, SC_NS)); f35 = 1; }
    }
  }
  
  void CT6()
  {
    if (count == 14) { sc_assert(sc_time_stamp() == sc_time(110, SC_NS));  f8 = 1; }
    if (count == 15) { sc_assert(sc_time_stamp() == sc_time(115, SC_NS));  f9 = 1; }
    if (count == 17) { sc_assert(sc_time_stamp() == sc_time(130, SC_NS)); f21 = 1; }
    if (count == 18) { sc_assert(sc_time_stamp() == sc_time(135, SC_NS)); f24 = 1; }
    if (count == 19) { sc_assert(sc_time_stamp() == sc_time(145, SC_NS)); f30 = 1; }
    while (true)
    {
      try {
        wait();
      }
      catch (const sc_unwind_exception& e) {
        sc_assert( e.is_reset() );
        sc_assert( sc_is_unwinding() );
        if (count == 14) { sc_assert(sc_time_stamp() == sc_time(110, SC_NS)); f37 = 1; }
        if (count == 17) { sc_assert(sc_time_stamp() == sc_time(130, SC_NS)); f38 = 1; }
        throw e;
      }
      if (count == 16) { sc_assert(sc_time_stamp() == sc_time(125, SC_NS)); f15 = 1; }
      if (count == 20) { sc_assert(sc_time_stamp() == sc_time(155, SC_NS)); f36 = 1; }
    }
  }
  
  SC_HAS_PROCESS(M);
};

struct Top: sc_module
{
  Top(sc_module_name _name)
  {
    m = new M("m");
    m->clk(clk);
    m->sreset1(sreset1);
    m->sreset2(sreset2);
    m->sreset3(sreset3);
    m->areset1(areset1);
    m->areset2(areset2);
    m->areset3(areset3);
    
    SC_THREAD(ctrl);
    
    // Resets are all active-low
    sreset1.write(1);
    sreset2.write(1);
    sreset3.write(1);
    areset1.write(1);
    areset2.write(1);
    areset3.write(1);
  }

  M* m;
  
  sc_signal<bool> clk;
  
  sc_signal<bool> sreset1;
  sc_signal<bool> sreset2;
  sc_signal<bool> sreset3;
  
  sc_signal<bool> areset1;
  sc_signal<bool> areset2;
  sc_signal<bool> areset3;
  
  void ctrl()
  {
    m->count = 1;
    clock();

    m->count = 2;
    sreset1.write(0);
    clock();

    m->count = 3;
    sreset1.write(1);
    clock();

    m->count = 4;
    sreset2.write(0);
    clock();

    m->count = 5;
    sreset2.write(1);
    clock();

    m->count = 6;
    sreset3.write(0);
    clock();

    m->count = 7;
    sreset3.write(1);
    clock();

    m->count = 8;
    areset1.write(0);
    wait(SC_ZERO_TIME);

    m->count = 9;
    clock();

    m->count = 10;
    areset1.write(1);
    clock();

    m->count = 11;
    areset2.write(0);
    wait(SC_ZERO_TIME);

    m->count = 12;
    clock();

    m->count = 13;
    areset2.write(1);
    clock();

    m->count = 14;
    areset3.write(0);
    wait(SC_ZERO_TIME);

    m->count = 15;
    clock();

    m->count = 16;
    areset3.write(1);
    clock();

    m->count = 17;
    sreset1.write(0);
    sreset2.write(0);
    sreset3.write(0);
    areset1.write(0);
    areset2.write(0);
    areset3.write(0);
    wait(SC_ZERO_TIME);

    m->count = 18;
    clock();

    m->count = 19;
    clock();

    m->count = 20;
    sreset1.write(1);
    sreset2.write(1);
    sreset3.write(1);
    areset1.write(1);
    areset2.write(1);
    areset3.write(1);
    clock();
  }
  
  void clock()
  {
    clk.write(0);
    wait(5, SC_NS);
    clk.write(1);
    wait(5, SC_NS);
  }
  
  SC_HAS_PROCESS(Top);
};

int sc_main(int argc, char* argv[])
{
  Top top("top");
  
  sc_start();

  sc_assert(top.m->f1);  
  sc_assert(top.m->f2);  
  sc_assert(top.m->f3);  
  sc_assert(top.m->f4);  
  sc_assert(top.m->f5);  
  sc_assert(top.m->f6);  
  sc_assert(top.m->f7);  
  sc_assert(top.m->f8);  
  sc_assert(top.m->f9);  
  sc_assert(top.m->f10);  
  sc_assert(top.m->f11);  
  sc_assert(top.m->f12);  
  sc_assert(top.m->f13);  
  sc_assert(top.m->f14);  
  sc_assert(top.m->f15);  
  sc_assert(top.m->f16);  
  sc_assert(top.m->f17);  
  sc_assert(top.m->f18);  
  sc_assert(top.m->f19);  
  sc_assert(top.m->f20);  
  sc_assert(top.m->f21);  
  sc_assert(top.m->f22);  
  sc_assert(top.m->f23);  
  sc_assert(top.m->f24);  
  sc_assert(top.m->f25);  
  sc_assert(top.m->f26);  
  sc_assert(top.m->f28);  
  sc_assert(top.m->f29);  
  sc_assert(top.m->f30);  
  sc_assert(top.m->f31);  
  sc_assert(top.m->f32);  
  sc_assert(top.m->f33);  
  sc_assert(top.m->f34);  
  sc_assert(top.m->f35);  
  sc_assert(top.m->f36);  
  sc_assert(top.m->f37);  
  sc_assert(top.m->f38);  

  cout << endl << "Success" << endl;
  return 0;
}
