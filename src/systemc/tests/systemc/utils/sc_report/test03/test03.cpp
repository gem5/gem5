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

  test03.cpp -- Test of disabling of SC_FATAL

  Original Author: Andy Goodrich, Forte Design Systems, Inc., 2005-12-12

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
		reset_signal_is(m_reset, true);
	}
	void thread()
	{
		sc_report_handler::set_actions(SC_FATAL,SC_DISPLAY);
		sc_report_handler::stop_after(SC_FATAL,-1);
		for (;;)
		{
			wait();
			SC_REPORT_FATAL("Oh no!","A bad thing has happened");
		}
	}
	sc_in<bool> m_clk;
	sc_in<bool> m_reset;
};
int sc_main(int argc, char* argv[])
{
	sc_clock        clock;
	DUT             dut("dut");
	sc_signal<bool> reset;

	dut.m_clk(clock);
	dut.m_reset(reset);

	reset = true;
	sc_start(1, SC_NS);
	reset = false;
	sc_start(2, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}
