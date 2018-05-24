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

// sc_writer_policy.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: sc_writer_policy.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// sc_writer_policy template argument of class sc_signal

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>

using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;
using std::string;


struct M: sc_module
{
  sc_inout<bool> port;
  
  sc_time delay;
  sc_signal<int>                  one_sig;
  sc_signal<int, SC_MANY_WRITERS> many_sig;
  
  bool first_run;
  int g0, g1, g2, g3;
  
  M(sc_module_name _name, sc_time _delay)
  : port("port")
  , delay(_delay)
  , first_run(true)
  {
    sc_assert( one_sig.get_writer_policy() == SC_ONE_WRITER );
    sc_assert( many_sig.get_writer_policy() == SC_MANY_WRITERS );

    SC_THREAD(T);
    SC_METHOD(method_process_1);
    SC_METHOD(method_process_2);

    one_sig.write(-1);
    many_sig.write(-1);
    
    g0 = g1 = g2 = g3 = 0;
  }
  
  void end_of_elaboration()
  {
    sc_assert( port->get_writer_policy() == SC_MANY_WRITERS );
    one_sig.write(1);
    many_sig.write(1);
    g0 = 1;
  }
  
  void start_of_simulation()
  {
    one_sig.write(2);
    many_sig.write(2);
    g1 = 1;
  }
  
  void T()
  {
    wait(delay);
    port.write(true);
    g2 = 1;
  }
  
  void method_process_1()
  {
    one_sig.write(3);
    many_sig.write(3);
  }
  
  void method_process_2()
  {
    if (first_run)
    {
      first_run = false;
      next_trigger(SC_ZERO_TIME);
    }
    else
    {
      try {
        one_sig = 4;
      }
      catch (const std::exception& e) {
        g3 = 1;
      }
      many_sig.write(4);
    }
  }
  
  SC_HAS_PROCESS(M);
};

struct Top: sc_module
{
  M *m1;
  M *m2;
  
  sc_signal<bool,SC_MANY_WRITERS> many_sig_1;
  sc_signal<int,SC_MANY_WRITERS>  many_sig_2;
  sc_signal<int,SC_ONE_WRITER>    one_sig_1;
  sc_signal<int>                  one_sig_2;
  
  sc_buffer<sc_logic, SC_MANY_WRITERS> buffy;
  
  sc_signal_resolved              resolved;
  sc_signal_rv<2>                 rv; 
    
  Top(sc_module_name _name)
  : many_sig_1("many_sig_1")
  , many_sig_2("many_sig_2")
  , one_sig_1("one_sig_1")
  , buffy("buffy")
  , resolved("resolved")
  , rv("rv")
  {
    m1 = new M("m1", sc_time(1, SC_PS));
    m2 = new M("m2", sc_time(2, SC_PS));
    
    m1->port.bind(many_sig_1);
    m2->port.bind(many_sig_1);
    
    SC_THREAD(T1);
    SC_THREAD(T2);
    sc_spawn(sc_bind(&Top::T3, this));
    
    sc_assert( many_sig_1.get_writer_policy() == SC_MANY_WRITERS );
    sc_assert( many_sig_2.get_writer_policy() == SC_MANY_WRITERS );
    sc_assert( one_sig_1 .get_writer_policy() == SC_ONE_WRITER );
    sc_assert( one_sig_2 .get_writer_policy() == SC_ONE_WRITER );
    sc_assert( buffy     .get_writer_policy() == SC_MANY_WRITERS );
    sc_assert( resolved  .get_writer_policy() == SC_MANY_WRITERS );
    sc_assert( rv        .get_writer_policy() == SC_MANY_WRITERS );
    
    one_sig_1 = 0;
    buffy = SC_LOGIC_X;
    resolved = SC_LOGIC_Z;
    rv = sc_lv<2>("ZZ");
    
    // Writes outside of a process should not count as manyple writers
    many_sig_1.write(true);
    many_sig_2.write(0);
    one_sig_1.write(0);
    one_sig_2.write(0);
    buffy.write(SC_LOGIC_0);
    
    f0 = f1 = f2 = f3 = f4 = f5 = 0;

  }
  
  int f0, f1, f2, f3, f4, f5;

  void T1()
  { 
    resolved = SC_LOGIC_0;
    rv = sc_lv<2>("01");
    
    // Attempt to write SC_ONE_WRITER signal from >1 process should fail
    try {
      one_sig_1 = 1;
    }
    catch (const std::exception& e) {
      f3 = 1;
    }
    
    try {
      one_sig_2 = 1;
    }
    catch (const std::exception& e) {
      f4 = 1;
    }
    wait(1, SC_PS);

    // Attempt to write SC_MANY_WRITER signal from >1 process IN SAME DELTA should fail
    try {
      many_sig_2.write(3);
    }
    catch (const std::exception& e) {
      f5 = 1;
    }
    wait(3, SC_PS);

    many_sig_2.write(6);
    buffy = SC_LOGIC_0;

    wait(many_sig_2.default_event());
    f0 = 1;
  }

  void T2()
  {
    resolved = SC_LOGIC_1;
    rv = sc_lv<2>("10");
    
    try {
      one_sig_1 = 2;
    }
    catch (const std::exception& e) {
      f3 = 1;
    }

    try {
      one_sig_2 = 2;
    }
    catch (const std::exception& e) {
      f4 = 1;
    }
    wait(1, SC_PS);
    
    try {
      many_sig_2.write(4);
    }
    catch (const std::exception& e) {
      f5 = 1;
    }
    wait(4, SC_PS);

    many_sig_2.write(7);
    buffy = SC_LOGIC_1;

    wait(many_sig_2.default_event());
    f1 = 1;
  }
  
  void T3()
  {
    resolved = SC_LOGIC_Z;
    rv = sc_lv<2>("ZZ");

    try {
      one_sig_1 = 3;
    }
    catch (const std::exception& e) {
      f3 = 1;
    }

    try {
      one_sig_2 = 3;
    }
    catch (const std::exception& e) {
      f4 = 1;
    }
    wait(1, SC_PS);
    
    try {
      many_sig_2.write(5);
    }
    catch (const std::exception& e) {
      f5 = 1;
    }
    wait(5, SC_PS);

    many_sig_2.write(8);
    buffy = SC_LOGIC_0;

    wait(many_sig_2.default_event());
    f2 = 1;
    
    sc_assert( resolved.read() == SC_LOGIC_X );
    sc_assert( rv.read() == sc_lv<2>("XX") );
    sc_assert( many_sig_2.read() == 8 );
    sc_assert( buffy.read() == SC_LOGIC_0 );
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

  sc_assert( top.m1->g0 );
  sc_assert( top.m2->g0 );
  sc_assert( top.m1->g1 );
  sc_assert( top.m2->g1 );
  sc_assert( top.m1->g2 );
  sc_assert( top.m2->g2 );
  sc_assert( top.m1->g3 );
  sc_assert( top.m2->g3 );
  
  cout << endl << "Success" << endl;
  return 0;
}
  
