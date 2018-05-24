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

  test3.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE( p1 )
{
  SC_HAS_PROCESS( p1 );

  sc_fifo<int>& b;

  int init;

  p1(sc_module_name name,
     sc_fifo<int>& B, 
     int INIT)
    : b(B)
  {
    init = INIT;
    SC_THREAD( entry );
    // sensitive << b;
  }

  void entry() {
    // wait();
    int i = init;
    // wait();
    while (true) {
      int j = b.read();
      cout << " Value read = " << j << endl;
      // wait();
    }
  }
};

SC_MODULE( p2 )
{
  SC_HAS_PROCESS( p2 );

  sc_in<bool> clk;

  sc_fifo<int>& a;

  int init;

  p2 (sc_module_name name,
      sc_signal_in_if<bool>& CLK,
      sc_fifo<int>& A,
      int INIT)
    : a(A)
  {
    clk(CLK);
	SC_CTHREAD( entry, clk.pos() );
    init = INIT;
  }

  void entry() {
    int i = init;
    wait();
    while (true) {
      a.write(i);
      cout << "Value sent = " << i << endl;
      wait(); i++;
    }
  }
};
    
int sc_main(int ac, char *av[])
{
  sc_fifo<int> a(2), b(2);
  sc_clock clock("Clock", 20, SC_NS);

  p1 Proc1("Proc1", a, 10);
  p2 Proc2("Proc2", clock, a, 129);

  // sc_start(500, SC_NS);
  sc_start(250, SC_NS);

  return 0;
}
