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

  test13.cpp -- Test detection of write from process and sc_main.

  Original Author: Andy Goodrich, Forte Design Systems, 02 Apr 2007

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

SC_MODULE(DUT)
{
	SC_CTOR(DUT)
	{
		SC_CTHREAD(thread,m_clk.pos());
	}
	void thread()
	{
		m_data = false;
		for (;;)
		{
			wait();
		}
	}
	sc_in<bool>  m_clk;
	sc_out<bool> m_data;
};

int sc_main(int argc, char* argv[])
{
	sc_clock        clock;
	sc_signal<bool> data;
	DUT             dut("dut");

	dut.m_clk(clock);
	dut.m_data(data);

	sc_start(1, SC_NS);
	data = true;
	sc_start(1, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}
