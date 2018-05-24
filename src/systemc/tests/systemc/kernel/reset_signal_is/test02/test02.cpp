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

  test02.cpp -- Test reset_signal_is() usage with SC_CTHREAD processes.

  Original Author: Andy Goodrich, Forte Design Systems, 12 August 2005
    
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
		SC_CTHREAD(cthread,m_clk.pos());
		reset_signal_is(m_reset,true);
		SC_CTHREAD(resetter,m_clk.pos());
	}
	void cthread()
	{
		cout << sc_time_stamp() << ": initializing" << endl;
		for (;;)
		{
		    wait(3);
			cout << sc_time_stamp() << ": waited 3" << endl;
		}
	}
	void resetter()
	{
		m_reset = false;
		wait(3);
		m_reset = true;
		wait(2);
		m_reset = false;
		wait(6);
		m_reset = true;
		wait(5);
		sc_stop();
	}
	sc_in<bool>    m_clk;
	sc_inout<bool> m_reset;
};

int sc_main( int argc, char* argv[] )
{
	sc_clock        clock;
	DUT             dut("dut");
	sc_signal<bool> reset;

	dut.m_clk(clock);
	dut.m_reset(reset);

	sc_start();
	return 0;
}

