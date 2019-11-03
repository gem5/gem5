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

  test15.cpp -- test event/time tracing

  Original Author: Philipp A Hartmann, Intel Corporation, 2017-08-29

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc>
using namespace sc_core;

SC_MODULE(module)
{
  SC_CTOR(module)
  {
    SC_THREAD(driver);
    SC_METHOD(consumer);
      sensitive << event;
      dont_initialize();
  }

  void driver()
  {
    wait(1, SC_NS);
    event.notify(); // t == 1ns

    wait(1, SC_NS);
    event.notify(SC_ZERO_TIME); // t == 2ns

    wait(1, SC_NS);
    event.notify(1, SC_NS); // t == 4ns
    time = sc_time_stamp();

    wait(2, SC_NS);
    event.notify(); // t == 5ns

    wait(1, SC_NS);
    event.notify(SC_ZERO_TIME); // t == 6ns

    wait(1, SC_NS);
    event.notify(1, SC_NS); // t == 8ns
    time = sc_time_stamp();

    wait(2, SC_NS);
  }

  void consumer()
  {
    time = sc_time_stamp();
  }

  sc_event event;
  sc_time  time;
};

int sc_main(int argc, char* argv[])
{
  sc_trace_file* tf = sc_create_vcd_trace_file("test16");

  module m("m");

  sc_trace(tf, m.event, "event");
  sc_trace(tf, m.time,  "time");

  sc_start();
  sc_stop();

  sc_close_vcd_trace_file(tf);
  return 0;
}


