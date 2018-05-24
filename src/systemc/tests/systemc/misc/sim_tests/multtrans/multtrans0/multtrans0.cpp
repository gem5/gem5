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

  multtrans0.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( delay1 )
{
  SC_HAS_PROCESS( delay1 );

  sc_in<int>  data_i;
  sc_out<int> data_o;

  // Constructor
  delay1( sc_module_name NAME,
          sc_signal<int>& DATA_I,
          sc_signal<int>& DATA_O )
  {
    data_i(DATA_I);
	data_o(DATA_O);
    SC_THREAD( entry );
    sensitive << data_i;
  }

  // Functionality
  void entry();
};

void delay1::entry()
{
  wait();
  int buffer;
  wait(); // until the first real event
  buffer = data_i.read(); 
  wait(); // until you get the next sample to send data out
  while (true) {
    data_o.write(buffer);
    buffer = data_i.read();
    wait();
  }
}

SC_MODULE( state_machine )
{
  SC_HAS_PROCESS( state_machine );

  sc_in<int>  data_i1;
  sc_in<int>  data_i2;
  sc_out<int> data_o;

  int state;

  // Constructor
  state_machine( sc_module_name NAME,
		 sc_signal<int>& DATA_I1,
		 sc_signal<int>& DATA_I2,
		 sc_signal<int>& DATA_O )
  {
    data_i1(DATA_I1); data_i2(DATA_I2); data_o(DATA_O);
    SC_METHOD( entry );
    sensitive << data_i1 << data_i2;

    state = 0;
  }

  // Functionality
  void entry();
};

void state_machine::entry()
{
  switch(state) {
  case 0: // initial state
    cout << "In state 0 :: " << flush;
    if (data_i1.event()) {
      if (data_i2.event()) {
	data_o.write(data_i1.read() + data_i2.read());
	cout << "staying in state 0" << endl;
      }
      else {
	state = 1;
	cout << "going to state 1" << endl;
	// No output is written
      }
    }
    else if (data_i2.event()) {
      state = 2;
      cout << "going to state 2" << endl;
      // No output is written
    }
    break;
  case 1: // Event on First input seen only
    cout << "In state 1 :: " << flush;
    if (data_i2.event()) {
      if (data_i1.event()) {
	state = 0;
	cout << "going to state 0" << endl;
	data_o.write(data_i1.read() + data_i2.read());
      }
      else {
	cout << "going to state 2" << endl;
	state = 2;
	// No output written
      }
    }
    else {
      cout << "staying in state 1" << endl;
    }
    break;
  case 2:
    cout << "In state 2 :: " << flush;
    if (data_i1.event()) {
      if (data_i2.event()) {
	state = 0;
	cout << "going to state 0" << endl;
	data_o.write(data_i1.read() + data_i2.read());
      }
      else {
	cout << "going to state 1" << endl;
	state = 1;
	// No output written
      }
    }
    else {
      cout << "staying in state 2" << endl;
    }
    break;
  default: 
    cout << "In bad state - resetting" << endl;
    state = 0;
    break;
  }
}

void testbench(sc_signal<int>& data, const sc_signal<int>& res)
{
  int i;
  sc_start(0, SC_NS);
  for (i=0; i<10; i++) {
    data.write(i*10 + 123);
    sc_start( 10, SC_NS );
    cout << "Result = " << res.read() << endl;
    data.write(i * 11 - 34);
    sc_start( 10, SC_NS );
    cout << "Result = " << res.read() << endl;
  }
}

int
sc_main(int ac, char *av[])
{
  sc_signal<int> data_gen("Data");
  sc_signal<int> data_delayed("DelayedData");
  sc_signal<int> result("Result");

  data_gen = 0;
  data_delayed = 0;
  result = 0;

  delay1 D1("DL", data_gen, data_delayed);
  state_machine SM("SM", data_gen, data_delayed, result);

  testbench(data_gen, result);
  return 0;
}
