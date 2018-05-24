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

  async_clock.cpp -- 

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

    proc1( sc_module_name NAME,
           sc_signal_in_if<bool>& CLK )
    {
        clk( CLK );
		SC_CTHREAD( entry, clk.pos() );
    }

    void entry()
    {
        while (true) {
            wait();
	    cout << "Process 1 triggered" << endl;
        }
    }
};

SC_MODULE( proc2 )
{
    SC_HAS_PROCESS( proc2 );

    sc_in_clk clk;

    proc2( sc_module_name NAME,
           sc_signal_in_if<bool>& CLK )
    {
        clk( CLK );
	SC_CTHREAD( entry, clk.pos() );
    }

    void entry()
    {
        while (true) {
            wait();
	    cout << "Process 2 triggered" << endl;
        }
    }
}; 

SC_MODULE( proc3 )
{
    SC_HAS_PROCESS( proc3 );

    sc_in_clk clk;

    proc3( sc_module_name NAME,
           sc_signal_in_if<bool>& CLK )
    {
        clk( CLK );
	SC_CTHREAD( entry, clk.pos() );
    }

    void entry()
    {
        while (true) {
            wait();
	    cout << "Process 3 triggered" << endl;
        }
    }
};           


SC_MODULE( proc4 )
{
    SC_HAS_PROCESS( proc4 );

    sc_in<bool>  a;
    sc_in<bool>  b;
    sc_in_clk    clk;
    sc_out_clk   c;
    sc_out_clk   d;

    proc4( sc_module_name NAME,
           sc_signal<bool>& A,
           sc_signal<bool>& B,
	   sc_signal_in_if<bool>& CLK,
           sc_signal_out_if<bool>& C,
           sc_signal_out_if<bool>& D )
    {
        a(A);
	b(B);
	clk(CLK);
	c(C);
	d(D);
        SC_METHOD( entry );
        sensitive << a << b << clk;
    }
           
    void entry()
    {
      if ((bool) a == 1)
	c = clk;
      else 
	c = 0;
	
      d = clk & b; 
    }
};

#define NS * 1e-9

int
sc_main( int argc, char* argv[] )
{
    sc_signal<bool> clk1("clk1");
    sc_signal<bool> dclk1("Dclock1"); // First derived clock
    sc_signal<bool> dclk2("Dclock2"); // Second derived clock

    sc_signal<bool> p("p"), q("q");

    proc1 p1("p1", clk1);
    proc2 p2("p2", dclk1);
    proc3 p3("p3", dclk2);
    proc4 p4("p4", p, q, clk1, dclk1, dclk2 );

    sc_start(0, SC_NS);
    p = 1;
    q = 1;
    for (double t = 0; t < 5 NS; t += 1 NS) {
        clk1 = 1;
        sc_start( 1, SC_NS );
        clk1 = 0;
        sc_start( 1, SC_NS );
	cout << " ***" << endl;
    }
    q = 0;
    for (double t = 0; t < 5 NS; t += 1 NS) {
        clk1 = 1;
        sc_start( 1, SC_NS );
        clk1 = 0;
        sc_start( 1, SC_NS );
	cout << " ***" << endl;
    }
    p = 0;
    for (double t = 0; t < 5 NS; t += 1 NS) {
        clk1 = 1;
        sc_start( 1, SC_NS );
        clk1 = 0;
        sc_start( 1, SC_NS );
	cout << " ***" << endl;
    }

    return 0;
}
