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

  test2.cpp -- 

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

  // Constructor
  proc1( sc_module_name NAME,
	 sc_clock& CLOCK,
	 sc_fifo<int>& IN_ )
    : in(IN_)
  {
    clk( CLOCK );
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry() {
    int val;
    cout << "Proc1:: Waiting 10 cycles before reading\n";
    wait( 10 );
    cout << "Proc1:: Wait completed\n";
    val = in.read();
    cout << "Proc1:: Read. Value = " << val << "\n";
    cout << "Proc1:: Read\n";
    val = in.read();
    cout << "Proc1:: Read completed. Value = " << val << "\n";
    val = in.read();
    cout << "Proc1:: Read. Value = " << val << "\n";
    cout << "Proc1:: Loop start\n";
    int i = 1;
    int j;
    while (true) {
      j = in;
      cout << "Proc1:: Read. Value = " << j << "\n";
      if (in > 5 && in < 7) cout << "Proc1:: Special value 6 read\n";
      wait( i );
      i += 3;
    }
  }
};

SC_MODULE( proc2 )
{
  SC_HAS_PROCESS( proc2 );

  sc_in_clk clk;

  sc_fifo<int>& out;

  // Constructor
  proc2( sc_module_name NAME,
	 sc_clock& CLOCK,
	 sc_fifo<int>& OUT_ )
    : out(OUT_)
  {
    clk( CLOCK );
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  void entry() {
    cout << "Proc2:: Write\n";
    out = 1;
    cout << "Proc2:: Write completed\n";
    cout << "Proc2:: Wait 10 cycles before write\n";
    wait( 10 );
    cout << "Proc2:: Wait completed\n";
    cout << "Proc2:: Write\n";
    out = 2;
    cout << "Proc2:: Write Completed\n";
    cout << "Proc2:: Write\n";
    out = 3;
    cout << "Proc2:: Write Completed\n";
    cout << "Proc2:: Loop start\n";
    for (int i=4; i<10; i++) {
      out = i;
      wait( i + 2 );
    }
    sc_stop();
  }
};

int sc_main(int ac, char *av[])
{
  sc_fifo<int> c("C");

  sc_clock clock("CLK", 20, SC_NS);

  proc1 p1("P1", clock, c);
  proc2 p2("P2", clock, c);

  sc_start();

  return 0;
}
