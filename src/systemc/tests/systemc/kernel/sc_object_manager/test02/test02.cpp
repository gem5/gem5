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
  Original Author: Andy Goodrich, Forte Design Systems

 *****************************************************************************/
/*****************************************************************************
  MODIFICATION LOG - modifiers, enter your name, affiliation, date and  
  changes you are making here.

      Name, Affiliation, Date: 
  Description of Modification: 

 *****************************************************************************/

// TEST THAT THE CORRECT PARENT POINTER IS SET FOR THE before_end_of_elaboration
// and end_of_elaboration CALLBACKS

#include "systemc.h"

class my_object : public sc_object 
{
  public:
	my_object() {}
	virtual ~my_object() {}
};

SC_MODULE(DUT)
{
	SC_CTOR(DUT)
	{
		SC_CTHREAD(thread,m_clk.pos());
		reset_signal_is(m_reset, true);
	}
	void before_end_of_elaboration()
	{
		m_before_p = new my_object;		
	}
	void end_of_elaboration()
	{
		m_end_p = new my_object;		
	}
	void thread()
	{
		for (;;)
		{
			wait();
			if ( m_before_p->get_parent_object() == 0 )
				cout << "before_end_of_elaboration parent is 0!" <<endl;
			if ( m_end_p->get_parent_object() == 0 )
				cout << "end_of_elaboration parent is 0!" <<endl;
		}
	}
	my_object*  m_before_p;
	sc_in<bool> m_clk;
	my_object*  m_end_p;
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
	sc_start(1, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}
