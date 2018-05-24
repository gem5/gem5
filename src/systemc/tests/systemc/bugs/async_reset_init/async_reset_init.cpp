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

/*****************************************************************************

  async_reset_init.cpp -- Starting a process in async reset state

  Original Author: Philipp A. Hartmann, Intel Corporation, 2017-07-23

 *****************************************************************************/

#include <systemc>
#include <iomanip>

SC_MODULE(module)
{
  sc_core::sc_in<bool> rst_in;
  sc_core::sc_event    ev;

  SC_CTOR(module)
    : rst_in("rst_in")
    , ev("ev")
  {
    SC_THREAD(thread0);
      sensitive << ev;
      async_reset_signal_is(rst_in,true);

    SC_THREAD(thread1);
      sensitive << ev;
      async_reset_signal_is(rst_in,true);
      dont_initialize();

    SC_METHOD(method0);
      sensitive << ev;
      async_reset_signal_is(rst_in,true);

    SC_METHOD(method1);
      sensitive << ev;
      async_reset_signal_is(rst_in,true);
      dont_initialize();
  }

  void thread0() { do_thread(); }
  void thread1() { do_thread(); }

  void do_thread()
  {
    print( "reset state" );
    wait();
    print( "reset done" );

    while(1) // main loop
    {
      wait();
      print( "continuing" );
    }
  }

  void method0() { do_method(); }
  void method1() { do_method(); }

  void do_method()
  {
    if( rst_in.read() ) {
      print("reset state");
    } else {
      print("running");
    }
  }

  void print(const char* msg)
  {
    using namespace sc_core;
    using namespace std;
    cout
      << setw(6) << sc_time_stamp()
      << " (" << sc_delta_count() << "): "
      << sc_get_current_process_handle().name() << ": "
      << msg
      << endl;
  }
}; // SC_MODULE(module)

int sc_main(int argc, char* argv[])
{
  using namespace sc_core;
  using namespace std;

  sc_signal<bool> rst_sig;
  rst_sig.write(true);

  module top("top");
  top.rst_in(rst_sig);

  cout << "Starting simulation ... " << endl;

  sc_start(10, SC_NS);
  top.ev.notify(SC_ZERO_TIME);
  sc_start(10, SC_NS);

  rst_sig.write(false); // releasing reset

  sc_start(10, SC_NS);
  top.ev.notify(SC_ZERO_TIME);
  sc_start(10, SC_NS);
  top.ev.notify(SC_ZERO_TIME);
  sc_start(10, SC_NS);

  rst_sig.write(true);  // entering reset

  sc_start(10, SC_NS);
  top.ev.notify(SC_ZERO_TIME);
  sc_start(10, SC_NS);

  cout << "... done." << endl;
  sc_stop();
  return 0;
}

