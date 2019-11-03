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

  dataflow.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( sawtooth )
{
  SC_HAS_PROCESS( sawtooth );

  sc_in_clk clk;

  sc_fifo<int>& out1;
  sc_fifo<int>& out2;

  sawtooth( sc_module_name NAME,
	    sc_clock& CLK,
	    sc_fifo<int>& OUT1,
	    sc_fifo<int>& OUT2 )
    : out1(OUT1), out2(OUT2)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry();
};

void sawtooth::entry()
{
  int index = 0;
  while (true) {
     wait();
    out1.write(index % 17);
    out2.write(index % 17);
    index++;
  }
}

SC_MODULE( delay )
{
  SC_HAS_PROCESS( delay );

  sc_in_clk clk;

  sc_fifo<int>& in;
  sc_fifo<int>& out;

  delay( sc_module_name NAME,
	 sc_clock& CLK,
	 sc_fifo<int>& IN_,
	 sc_fifo<int>& OUT_ )
    : in(IN_), out(OUT_)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry();
};

void delay::entry()
{
  int buffer = 0;

  while (true) {
    out.write(buffer);
    buffer = in.read();
  }
}

SC_MODULE( downsample )
{
  SC_HAS_PROCESS( downsample );

  sc_in_clk clk;

  sc_fifo<int>& in;
  sc_fifo<int>& out;

  downsample( sc_module_name NAME,
	      sc_clock& CLK,
	      sc_fifo<int>& IN_,
	      sc_fifo<int>& OUT_ )
    : in(IN_), out(OUT_)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry();
};

void downsample::entry()
{
  int temp;
  while (true) {
    temp = in.read();
    temp = in.read();
    out.write(temp);
  }
}

SC_MODULE( upsample )
{
  SC_HAS_PROCESS( upsample );

  sc_in_clk clk;

  sc_fifo<int>& in;
  sc_fifo<int>& out;

  upsample( sc_module_name NAME,
	    sc_clock& CLK,
	    sc_fifo<int>& IN_,
	    sc_fifo<int>& OUT_ )
    : in(IN_), out(OUT_)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry();
};

void upsample::entry()
{
  while(true) {
    out.write(in.read());
    out.write(0);
  }
}

SC_MODULE( adder )
{
  SC_HAS_PROCESS( adder );

  sc_in_clk clk;

  sc_fifo<int>& a;
  sc_fifo<int>& b;

  adder( sc_module_name NAME,
	 sc_clock& CLK,
	 sc_fifo<int>& A,
	 sc_fifo<int>& B )
    : a(A), b(B)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry();
};

void adder::entry()
{
  while(true) {
    int tmp = a.read() + b.read();
    cout << "Sum = " << tmp << endl;
  }
}


int sc_main(int ac, char *av[])
{
  sc_fifo<int> st1("ST1", 2), st2("ST2", 2);
  sc_fifo<int> a1("A1", 2), a2("A2", 2), a3("A3", 2);
  sc_fifo<int> b1("B1", 2), b2("B2", 2), b3("B3", 2);

  sc_clock clock("CLOCK");

  sawtooth ST("TB1", clock, st1, st2);

  delay D1("D1", clock, st1, a1);
  downsample DN1("DN1", clock, a1, a2);
  upsample UP1("UP1", clock, a2, a3);

  downsample DN2("DN2", clock, st2, b1);
  upsample UP2("UP2", clock, b1, b2);
  delay D2("D2", clock, b2, b3);

  adder A ("A", clock, a3, b3);

  sc_start(100, SC_NS);

  return 0;
}
