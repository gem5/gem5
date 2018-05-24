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

  test12.cpp -- 

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

  sc_in<bool> clk;
  sc_signal<sc_int<10> >& bv;
  sc_signal<sc_uint<10> >& sv;

    sc_int<10> obj1;
    sc_uint<10> obj2;

    proc1( sc_module_name NAME,
	   sc_signal<bool>& CLK,
	   sc_signal<sc_int<10> >& BV,
	   sc_signal<sc_uint<10> >& SV )
	: bv(BV), sv(SV)
    {
        clk(CLK);
		SC_THREAD( entry );
	sensitive << clk;
	obj1 = 0;
	obj2 = 0;
        bv.write(0);
        sv.write(0);
    }

    void entry();
};
  
void proc1::entry() 
{
    wait();
    while(true) {
	obj1 = 3;
	obj2 = 7;
	bv = obj1;
	sv = obj2;
	wait();
	obj1 = -3;
	obj2 = 5;
	bv = obj1;
	sv = obj2;
	wait();
    }
}
  

int sc_main(int ac, char *av[])
{
  sc_trace_file *tf;
  sc_signal<bool> clock;
  sc_signal<sc_int<10> > bv;
  sc_signal<sc_uint<10> > sv;

  proc1 P1("P1", clock, bv, sv);

  tf = sc_create_vcd_trace_file("test12");
  sc_trace(tf, P1.obj1, "Signed");
  sc_trace(tf, P1.obj2, "Unsigned");
  sc_trace(tf, bv, "BV");
  sc_trace(tf, sv, "SV");

  clock.write(0);
  sc_start(0, SC_NS);
  for (int i = 0; i< 10; i++) {
cerr << sc_time_stamp() << endl;
    clock.write(1);
    sc_start(10, SC_NS);
cerr << sc_time_stamp() << endl;
    clock.write(0);
    sc_start(10, SC_NS);
cerr << sc_time_stamp() << endl;
  }

  sc_close_vcd_trace_file( tf );

  return 0;
}
