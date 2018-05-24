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

  test07.cpp -- Test that for all the interfaces of a port a callback occurs 
                for a dynamic process created after the beginning of simulation.

  Original Author: Andy Goodrich, Forte Design Systems

 *****************************************************************************/


/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test07.cpp,v $
// Revision 1.2  2011/02/01 17:17:40  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//


#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc.h"

SC_MODULE(DUT)
{
	SC_CTOR(DUT)
	{
		SC_CTHREAD(thread,m_clk.pos());

		// Set up interfaces for m_port.

		m_port(m_a);
		m_port(m_b);
		m_port(m_c);
	}
	void method()
	{
		cout << sc_time_stamp() << " callback" << endl;
	}
	void thread()
	{
		sc_spawn_options options;
		options.spawn_method();
		options.set_sensitivity( &m_port );
		options.dont_initialize();
		sc_spawn( sc_bind(&DUT::method,this), "method", &options );
		for ( bool value=true;; value = !value)
		{
			wait();
			cout << sc_time_stamp() << " setting m_a " << endl;
			m_a = value;
			wait();
			cout << sc_time_stamp() << " setting m_b " << endl;
			m_b = value;
			wait();
			cout << sc_time_stamp() << " setting m_c " << endl;
			m_c = value;
		}
	}
	sc_signal<bool>                     m_a;
	sc_signal<bool>                     m_b;
	sc_signal<bool>                     m_c;
	sc_in<bool>                         m_clk;
	sc_port<sc_signal_inout_if<bool>,3> m_port;
};

int sc_main(int argc, char* argv[])
{
	sc_clock clock;
    DUT      dut("dut");

	dut.m_clk(clock);

	sc_start(50, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}
