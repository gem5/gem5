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

  test10.cpp -- Test sc_module::set_stack_size

  Original Author: Andy Goodrich, Forte Design Systemc, Inc. 2003-10-13

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "systemc.h"

SC_MODULE(A)
{
	SC_CTOR(A)
	{
		SC_THREAD(thread);
		sensitive << m_clk;
		set_stack_size(0x600000);
	}
	void thread()
	{
		int  x[0x100000];    // Grab a lot of stack...
		x[0x100000-1] = 42;  // ... and then modify the last location`

	    for (;;) 
		{
			cout << sc_time_stamp() << endl;
			wait();
		}
	}
	sc_in_clk m_clk;
};

int sc_main(int argc, char* argv[])
{
	sc_clock clock;
	A        a("a");
	a.m_clk(clock);

	sc_start(2, SC_NS);

	return 0;
}
