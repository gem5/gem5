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

  test05.cpp -- test of binding w/zero bound port and binding after the fact.

  Original Author: Andy Goodrich, Forte Design Systems, 2005-09-12

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of binding with zero bound port and binding after the fact.


#include "systemc.h"

SC_MODULE(DUT)
{
	SC_CTOR(DUT) : 
		m_all_bound_0("all_bound_0"),
		m_all_bound_4("all_bound_4"),
		m_one_or_more_2("one_or_more_2"),
		m_zero_or_more_2("zero_or_more_2")
	{
		SC_CTHREAD(thread,m_clk.pos());

		m_all_bound_0(m_signal_0);       // 1

		m_all_bound_4(m_all_bound_0);    // 1
		m_all_bound_4(m_one_or_more_2);  // 2 and 3
		m_all_bound_4(m_zero_or_more_2); 
		m_all_bound_4(m_signal_3);       // 4
		m_all_bound_4(m_zero_or_more_2);

		m_one_or_more_2(m_signal_1);     // 1
		m_one_or_more_2(m_signal_2);     // 2
	}
	void thread()
	{
		m_signal_0 = 0;
		m_signal_1 = 1;
		m_signal_2 = 2;
		m_signal_3 = 3;
		for (;;)
		{
			wait();
			cout << "m_all_bound_4[0] = " << m_all_bound_4[0]->read() << endl;
			cout << "m_all_bound_4[1] = " << m_all_bound_4[1]->read() << endl;
			cout << "m_all_bound_4[2] = " << m_all_bound_4[2]->read() << endl;
			cout << "m_all_bound_4[3] = " << m_all_bound_4[3]->read() << endl;
		}
	}
	sc_in<bool>                                            m_clk;

	sc_port<sc_signal_in_if<int>,0,SC_ALL_BOUND>          m_all_bound_0;
	sc_port<sc_signal_in_if<int>,4,SC_ALL_BOUND>          m_all_bound_4;
	sc_port<sc_signal_in_if<int>,2,SC_ONE_OR_MORE_BOUND>  m_one_or_more_2;
	sc_signal<int>                                        m_signal_0;
	sc_signal<int>                                        m_signal_1;
	sc_signal<int>                                        m_signal_2;
	sc_signal<int>                                        m_signal_3;
	sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> m_zero_or_more_2;
};
int sc_main(int argc, char* argv[])
{
	sc_clock        clock;
	DUT             dut("dut");

	dut.m_clk(clock);

	sc_start(2, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}
