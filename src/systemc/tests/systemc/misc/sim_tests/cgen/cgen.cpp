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

  cgen.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

/* Filename seqgen.h */
/* This is the interface file for asynchronous process 'seqgen' */

SC_MODULE( seqgen )
{
  SC_HAS_PROCESS( seqgen );

  sc_in<bool> clock_i;
  sc_out<int> data_o;

  // Constructor
  seqgen( sc_module_name NAME,
	  sc_signal<bool>& CLOCK_I,
	  sc_signal<int>& DATA_O )
  {
    clock_i(CLOCK_I);  
	data_o(DATA_O);
    SC_THREAD( entry );
    sensitive << clock_i;
  }

  // Functionality of process
  void entry();
};


/* Filename seqgen.cc */
/* This is the implementation file for asynchronous process 'seqgen' */

void seqgen::entry()
{
  // Initialization code can go here
  int i = 17;
  wait();
  
  while (true) { // Infinite loop encompassing functionality
    if (clock_i.posedge()) {
      // Generate output only on positive edge of clock
      data_o.write(i);
      i = ((i * i) % 1019) - 9;
    }
    wait();
  }
}


// Interface and implementation files for
// Asynchronous block for code generation

SC_MODULE( codegen )
{
  SC_HAS_PROCESS( codegen );

  sc_in<int>  data_i;
  sc_out<int> code_o;

  int sum, num;

  // Constructor
  codegen( sc_module_name NAME,
	   sc_signal<int>& DATA_I,
	   sc_signal<int>& CODE_O )
  {
    data_i(DATA_I);
	code_o(CODE_O);
    SC_METHOD( entry );
    sensitive << data_i;

    sum = 0;
    num = 0;
  }

  // Process functionality
  void entry();
};

void codegen::entry()
{
  // Simple code generation routine
  sum += data_i.read();
  num++;
  code_o.write(sum % num);
}

void testbench(const sc_signal<int>& data, 
	       const sc_signal<int>& code,
	       sc_signal<bool>& clock)
{
  int i;

  sc_start(0, SC_NS);
  for (i=0; i<100; i++) {
    sc_start( 10, SC_NS );
    clock.write(1);
    sc_start( 10, SC_NS );
    clock.write(0);
    char buf[BUFSIZ];
    sprintf( buf, "Data = %4d\tCode = %4d", data.read(), code.read() );
    cout << buf << endl;
  }
}

int
sc_main(int ac, char *av[])
{
  sc_signal<bool> clock("Clock");
  sc_signal<int> data("Data");
  sc_signal<int> code("Code");

   clock = 0;
   data = 0;
   code = 0;

  seqgen S("S", clock, data);
  codegen C("C", data, code);

  testbench(data, code, clock);
  return 0;
}
