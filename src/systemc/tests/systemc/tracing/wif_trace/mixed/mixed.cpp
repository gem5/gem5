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

  mixed.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
#include "isaac.h"

QTIsaac<8> rng;

SC_MODULE( adder )
{
    SC_HAS_PROCESS( adder );

    sc_in<bool> clk;
    sc_in<int>  a;
    sc_in<int>  b;
    sc_out<int> sum;

    adder( sc_module_name NAME,
           sc_clock& CLK,
           sc_signal<int>& A,
           sc_signal<int>& B,
           sc_signal<int>& SUM )
        : a(A), b(B), sum(SUM)
    {
        clk(CLK);
		SC_METHOD( entry );
        sensitive << clk;
        sensitive << a;
        sensitive << b;
    }
    void entry();
};

void
adder::entry()
{
    if (clk.posedge()) {
        sum = a + b;
    }
}

SC_MODULE( stim )
{
    SC_HAS_PROCESS( stim );

    sc_in_clk   clk;
    sc_out<int> a;
    sc_out<int> b;

    stim( sc_module_name NAME,
          sc_clock& CLK,
          sc_signal<int>& A,
          sc_signal<int>& B )
        : a(A), b(B)
    {
        clk(CLK);
		SC_CTHREAD( entry, clk.pos() );
    }
    void entry();
};

void
stim::entry()
{
    while (true) {
        a = rng.rand() % 32768;
        b = rng.rand() % 32768;
        wait();
    }
}

int
sc_main( int argc, char* argv[] )
{
    sc_signal<int> a("a");
    sc_signal<int> b("b");
    sc_signal<int> sum("sum");
    sc_clock clk("clk", 20, SC_NS);

    a = 0;
    b = 0;
    sum = 0;

    adder add("add", clk, a, b, sum);
    stim  sti("sti", clk, a, b);

    sc_trace_file* tf = sc_create_wif_trace_file("mixed");
    sc_trace(tf, a, "a");
    sc_trace(tf, b, "b");
    sc_trace(tf, sum, "sum");
    sc_trace(tf, clk, "clk");
    sc_start(1000, SC_NS);
    sc_close_wif_trace_file( tf );
    return 0;
}
