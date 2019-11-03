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

  test01.cpp -- Test for reset_signal_is support.

  Original Author: Andy Goodrich

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
		SC_CTHREAD(test,m_clk.pos());
		reset_signal_is( m_reset, false );
	}
	void test()
	{
		{
			cout << "A: reset" << endl;
			wait();
		}
		for (;;)
		{
			wait();
		}
	}
	sc_in_clk   m_clk;
	sc_in<bool> m_reset;
};

SC_MODULE(B)
{
    B(sc_module_name name, sc_signal<bool>* reset_p ):
		sc_module(name), m_reset_p(reset_p)
	{
		SC_HAS_PROCESS(B);
		SC_CTHREAD(test,m_clk.pos());
		reset_signal_is( *m_reset_p, false );
	}
	void test()
	{
		{
			cout << "B: reset" << endl;
			wait();
		}
		for (;;)
		{
			wait();
		}
	}
	sc_in_clk        m_clk;
	sc_signal<bool>* m_reset_p;
};

int sc_main(int argc, char* argv[])
{
	sc_clock        clk;
	sc_signal<bool> reset;
	A a("a");
    B b("b",&reset);

	a.m_clk(clk);
	a.m_reset(reset);
	b.m_clk(clk);

	cout << "Before start" << endl;
	sc_start(2, SC_NS);
	reset = true;
	cout << "After reset true" << endl;
	sc_start(3, SC_NS);
	cout << "Ending" << endl;

	return 0;
}
