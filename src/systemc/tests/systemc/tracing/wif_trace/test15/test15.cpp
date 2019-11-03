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

  test15.cpp -- test delta cycle tracing

  Original Author: Romain I Popov, Intel Corporation, 2017-05-29

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>

SC_MODULE(trace_delta_test)
{
    sc_signal <int> sig;
    sc_inout<int > iport;

    SC_CTOR(trace_delta_test)
      : sig("sig",0)
    {
        iport.bind(sig);
        iport.initialize(1);
        SC_THREAD(test_thread);
    }

    void test_thread() {
        cout << "initial sig value is " << sig.read() << endl;
        sig = 2;
        wait(SC_ZERO_TIME);
        sig = 3;
        wait(SC_ZERO_TIME);
        sig = 4;
        wait(1, SC_PS);
        sig = 5; // This won't be shown on delta-cycle enabled trace
        wait(1, SC_PS);
        sig = 6; // This won't be shown on delta-cycle enabled trace
        wait(1, SC_NS);
        sig = 7;
        wait(1, SC_NS);
        sig = 8;
        wait(1, SC_NS);
        sig = 9;
        wait(SC_ZERO_TIME);
        sig = 10;
        wait(3, SC_PS);
        sig = 11; // Ok: 3ps > 2ps delta cycle shift
        wait(1, SC_NS);
        sig = 12;

        cout << "stop at " << sc_time_stamp() << endl;
        sc_stop();
    }

};


int sc_main(int argc, char **argv)
{
    trace_delta_test test_top("test_top");
    sc_trace_file* tf = sc_create_wif_trace_file("test15");
    sc_trace_delta_cycles(tf,true);
    sc_trace(tf, test_top.sig, test_top.sig.name());
    sc_start();
    return 0;
}
