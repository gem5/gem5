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

  test1.cpp -- 

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
  Case 1. Checking triggering of a sc_async/sc_aproc
  vis-a-vis a synchronous process. 
  This test checks to ensure that a synch. and async. sensitive
  to a clock are triggered correctly.
  */

#include "systemc.h"

SC_MODULE( syncproc )
{
  SC_HAS_PROCESS( syncproc );

  sc_in<bool> clk;

  const sc_signal<int>& in1;
  const sc_signal<int>& in2;
  sc_signal<int>& out;
  
  syncproc(sc_module_name NAME,
	   sc_signal_in_if<bool>& CLK,
	   const sc_signal<int>& IN1,
	   const sc_signal<int>& IN2,
	   sc_signal<int>& OUT_)
    : in1(IN1), in2(IN2), out(OUT_)
  {
    SC_CTHREAD( entry, clk.pos() );
    clk(CLK);
    out = 0;
  }

  void entry()
  {
    int i = 100;
    while (true) {
      out = i;
      wait();
      while (in1.read() != i) {
	cout << "Sync: Value written = " << i << "  value1 read = " << in1.read() << "  value2 read = " << in2.read() << endl;
	wait();
	cout << "Waited one cycle\n" << endl;
      }
      i++;
    }
  }
};

SC_MODULE( asyncproc )
{
  SC_HAS_PROCESS( asyncproc );

  const sc_signal<int>& in;
  sc_signal<int>& out;
  sc_in<bool> clock;

  asyncproc(sc_module_name NAME,
	    const sc_signal<int>& IN_,
	    sc_signal<int>& OUT_, 
	    sc_signal_in_if<bool>& CLOCK)
    : in(IN_), out(OUT_)
  {
    out = 0;
    clock(CLOCK);
    SC_THREAD( entry );
    sensitive << clock.pos();
  }

  void entry()
  {
    wait();
    while (true) {
      if (clock.posedge()) {
	out = in;
	cout << "AsyncProc: Value read = " << in.read() << endl;
      }
      else {
	cout << "Error" << endl;
      }
      wait();
    }
  }
};

SC_MODULE( asyncblock )
{
  SC_HAS_PROCESS( asyncblock );

  const sc_signal<int>& in;
  sc_signal<int>& out;
  sc_in<bool> clock;

  asyncblock(sc_module_name NAME,
	     const sc_signal<int>& IN_,
	     sc_signal<int>& OUT_, 
	     sc_signal_in_if<bool>& CLOCK)
    : in(IN_), out(OUT_)
  {
    clock(CLOCK);
    out = 0;
    SC_METHOD( entry );
    sensitive << clock;
  }

  void entry()
  {
    if (clock.posedge()) {
      out = in;
      cout << "AsyncBlock: Value read = " << in.read() << endl;
    }
    else {
      cout << "Seen other edge" << endl;
    }
  }
};
    

int
sc_main(int ac, char *av[])
{
  sc_signal<int> a, b, c;

  sc_clock clock("Clock", 20, SC_NS, 0.5);

  syncproc P1("P1", clock, a, b, c);
  asyncproc P2("P2", c, a, clock);
  asyncblock P3("P3", c, b, clock);

  sc_trace_file *tf = sc_create_vcd_trace_file("systemc");
  tf->set_time_unit(1, SC_NS);
  sc_trace(tf, a, "SYNC-IN1");
  sc_trace(tf, b, "SYNC-IN2");
  sc_trace(tf, c, "SYNC2-OUT");
  sc_trace(tf, clock, "Clock");

  sc_start(160, SC_NS);
  return 0;

}
