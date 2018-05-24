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

  test03.cpp -- Test sc_join as gating mechanism for a process awaiting the
                demise of its child processes.

  Original Author: Andy Goodrich, Forte Design Systems, 18 April 2005
    
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:    
    
 *****************************************************************************/

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc.h"

SC_MODULE(TB)
{
	SC_CTOR(TB)
	{
		SC_THREAD(abc);
		sensitive << m_clk.pos();
	}
	void abc()
	{
		for ( int i = 0; i < 3; i++ )
		{
			cout << "Time  Spawn Start Stop " << endl;
			cout << "----- ----- ----- ----" << endl;
			int              ii = 2;
			int              spawn_i;
			int              spawn_n = 8;
			sc_spawn_options options;
			sc_join          join;
			options.set_sensitivity(&m_clk.pos());
			for ( spawn_i = 0; spawn_i < spawn_n; spawn_i++ )
			{
				int process_i = spawn_i + i * spawn_n;
				cout << sc_time_stamp() << " " << process_i << endl;
				join.add_process(sc_spawn(
				    sc_bind(&TB::process, this, sc_ref(process_i)),
					sc_gen_unique_name("pipe"), &options ) );
				sc_core::wait(ii);
			}
			cout << sc_time_stamp() << " waiting for termination of " 
			     << join.process_count() << " processes" << endl;
			join.wait();
			cout << sc_time_stamp() << " back from termination wait " << endl;
		}
	}
	void process( int& instance )
	{
	    int i = instance;
		cout << sc_time_stamp() << "        " << i << endl;
		wait(6);
		cout << sc_time_stamp() << "              " << i << endl;
	}
	sc_in<bool> m_clk;
};


int sc_main(int argc, char* argv[])
{
	sc_clock clock;
	TB		 tb("tb");

	tb.m_clk(clock);
	sc_start(100, SC_NS);
	cout << "Program completed." << endl;
	return 0;
}
