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

  test02.cpp -- 

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

  unsigned char obj1;
  unsigned short obj2;
  unsigned int obj3;
  unsigned long obj4;

  proc1( sc_module_name NAME,
	 sc_signal<bool>& CLK )
  {
    clk(CLK);
    SC_THREAD( entry );
    sensitive << clk;
    obj1 = 0;
    obj2 = 0;
    obj3 = 0;
    obj4 = 0;
  }

  void entry();
};
  
void proc1::entry() 
{
  wait();
  while(true) {
    obj1 = 7;
    obj2 = 31;
    obj3 = 1023;
    obj4 = 2047;
    wait();
    obj1 = 1;
    obj2 = 32;
    obj3 = 1024;
    obj4 = 2048;
    wait();
  }
}
  

int sc_main(int ac, char *av[])
{
  sc_trace_file *tf;
  sc_signal<bool> clock;

  proc1 P1("P1", clock);

  tf = sc_create_vcd_trace_file("test02");
  sc_trace(tf, P1.obj1, "Char");
  sc_trace(tf, P1.obj2, "Short");
  sc_trace(tf, P1.obj3, "Int");
  sc_trace(tf, P1.obj4, "Long");
  sc_trace(tf, clock, "Clock");

  clock.write(0);
  sc_start(0, SC_NS);
  for (int i = 0; i< 10; i++) {
    clock.write(1);
    sc_start(10, SC_NS);
    clock.write(0);
    sc_start(10, SC_NS);
  }
  sc_close_vcd_trace_file( tf );
  return 0;
}
