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

  test6.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
  Corner case testing for new scheduler.
  Case 5: Checking multiple clock transitions at the same time
  */

#include "systemc.h"

SC_MODULE( triga )
{
  SC_HAS_PROCESS( triga );

  sc_in<bool> clock;
  sc_signal<int>& out;

  int i;

  triga(sc_module_name NAME,
	sc_signal_in_if<bool>& CLOCK,
	sc_signal<int>& OUT_)
    : out(OUT_)
  {
    clock(CLOCK);
    SC_METHOD( entry );
    sensitive << clock;
    i = 0;
    out = i++;
  }

  void entry()
  {
    out = i++;
  }
};

SC_MODULE( watcher )
{
  SC_HAS_PROCESS( watcher );

  sc_in<bool> clock1;
  sc_in<bool> clock2;
  const sc_signal<int>& in1;
  const sc_signal<int>& in2;
  const sc_signal<int>& in3;
  const sc_signal<int>& in4;

  watcher(sc_module_name NAME,
	  sc_signal_in_if<bool>& CLOCK1,
	  sc_signal_in_if<bool>& CLOCK2,
	  const sc_signal<int>& IN1,
	  const sc_signal<int>& IN2,
	  const sc_signal<int>& IN3,
	  const sc_signal<int>& IN4)
    : in1(IN1), in2(IN2), in3(IN3), in4(IN4)
  {
    clock1(CLOCK1); clock2(CLOCK2);
    SC_METHOD( entry );
    sensitive << clock1 << clock2 << in1 << in2 << in3 << in4;
  }

  void entry()
  {
    cout << "[ "; 
    if (clock1.posedge()) cout << "Posedge(1) - ";
    if (clock1.negedge()) cout << "Negedge(1) - ";
    if (clock2.posedge()) cout << "Posedge(2) - ";
    if (clock2.negedge()) cout << "Negedge(2) - ";
    if (in1.event()) cout << "Sync1 Out = " << in1.read() << " - ";
    if (in2.event()) cout << "ASync1 Out = " << in2.read() << " - ";
    if (in3.event()) cout << "Sync2 Out = " << in3.read() << " - ";
    if (in4.event()) cout << "ASync2 Out = " << in4.read() << " - ";
    cout << "]" << endl;
  }
};


SC_MODULE( trigp )
{
  SC_HAS_PROCESS( trigp );

  sc_in<bool> clk;

  sc_signal<int>& out;

  trigp(sc_module_name NAME,
	sc_signal_in_if<bool>& CLK,
	sc_signal<int>& OUT_)
    : out(OUT_)
  {
    clk(CLK); 
    SC_CTHREAD( entry, clk.pos() );
    out = 0;
  }

  void entry()
  {
    int i = 11;
    while (true) {
      out = i++;
      wait();
    }
  }
};

int
sc_main(int ac, char *av[])
{
  sc_clock clock1("Clock1", 20, SC_NS, 0.5);
  sc_clock clock2("Clock2", 20, SC_NS, 0.5);

  sc_signal<int> sig1, sig2, sig3, sig4;

  triga T1("T1", clock1, sig2);
  triga T2("T2", clock2, sig4);
  trigp T3("T3", clock1, sig1);
  trigp T4("T4", clock2, sig3);
  watcher W("W", clock1, clock2, sig1, sig2, sig3, sig4);

  sc_trace_file *tf = sc_create_vcd_trace_file("systemc");
  sc_trace(tf, clock1, "Clock1");
  sc_trace(tf, clock2, "Clock2");
  sc_trace(tf, sig1, "Sync1");
  sc_trace(tf, sig2, "Async1");
  sc_trace(tf, sig3, "Sync2");
  sc_trace(tf, sig4, "Async2");

  sc_start(100, SC_NS);
  return 0;
}
