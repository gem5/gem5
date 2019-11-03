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

  hshake1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( proc1 )
{
  SC_HAS_PROCESS( proc1 );

  sc_in_clk clk;

  sc_fifo<int>& in;
  sc_fifo<bool>& done;
  sc_fifo<int>& out;
  sc_fifo<bool>& ready;

  // Constructor
  proc1 ( sc_module_name NAME,
	  sc_clock& CLOCK,
	  sc_fifo<int>& IN_,
	  sc_fifo<bool>& DONE,
	  sc_fifo<int>& OUT_,
	  sc_fifo<bool>& READY )
    : in(IN_), done(DONE), out(OUT_), ready(READY)
  {
    clk(CLOCK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry() {
    ready.write(1);
    bool done_ = done.read();
    cout << "Done is " << done_ << endl;
    for (int i=0; i < 100; i++) {
      out.write(i);
      int in_ = in.read();
      cout << "Input is " << in_ << endl;
    }
    ready.write(0);
  }
};

SC_MODULE( proc2 )
{
  SC_HAS_PROCESS( proc2 );

  sc_in_clk clk;

  sc_fifo<int>& in;
  sc_fifo<bool>& done;
  sc_fifo<int>& out;
  sc_fifo<bool>& ready;

  // Constructor
  proc2 ( sc_module_name NAME,
	  sc_clock& CLOCK,
	  sc_fifo<int>& IN_,
	  sc_fifo<bool>& DONE,
	  sc_fifo<int>& OUT_,
	  sc_fifo<bool>& READY )
    : in(IN_), done(DONE), out(OUT_), ready(READY)
  {
    clk(CLOCK); 
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry() {
    bool done_ = done.read();
    cout << "Proc2::Done is " << done_ << endl;
    ready.write(1);
    for (int i=0; i < 100; i++) {
      out.write(i);
      int in_ = in.read();
      cout << "Proc2::Input is " << in_ << endl;
    }
    ready.write(0);
  }
};

int sc_main(int ac, char *av[])
{
  sc_fifo<bool> a;
  sc_fifo<bool> b;
  sc_fifo<int> c("C", 10);
  // sc_fifo<int> d("D", 2);
  sc_fifo<int> d("D", 1);

  sc_clock clock("CLK", 20, SC_NS);

  proc1 p1("P1", clock, c, a, d, b);
  proc2 p2("P2", clock, d, b, c, a);

  sc_start(1000, SC_NS);
  return 0;
}
