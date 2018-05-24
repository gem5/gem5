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

  test05.cpp -- Test using SC_FORK and SC_CJOIN macros.

  Original Author: Andy Goodrich, Forte Design Systems, 29 April 2004
    
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:    
    
 *****************************************************************************/

#include "systemc.h"
#include "sysc/kernel/sc_dynamic_processes.h"

SC_MODULE(X)
{
	SC_CTOR(X)
	{
		SC_CTHREAD(cwaiting,m_clk.pos());
	}
	void cwaiting()
	{
		SC_FORK
			sc_spawn( sc_bind(&X::sync, this, 3 ) ) ,
			sc_spawn( sc_bind(&X::sync, this, 4 ) ) ,
			sc_spawn( sc_bind(&X::sync, this, 5 ) ) ,
			sc_spawn( sc_bind(&X::sync, this, 5 ) ) ,
			sc_spawn( sc_bind(&X::sync, this, 7 ) ) ,
			sc_spawn( sc_bind(&X::sync, this, 11 ) ) ,
			sc_spawn( sc_bind(&X::sync, this, 21 ) ) 
		SC_CJOIN
		cout << sc_time_stamp() << ": clocked wait waking" << endl;
	}

	void sync(int context)
	{
		for ( int i = 0; i < context; i++ ) 
		{
			wait(m_clk.posedge_event());
		}
		cout << sc_time_stamp() << ": sync(" << context << ") terminating" << endl;
	}
	sc_in_clk m_clk;
};

int sc_main( int argc, char* argv[] )
{
	sc_clock clock;
	X x("x");
	x.m_clk(clock);

	sc_start(1000, SC_NS);

	cerr << "Program completed" << endl;
	return 0;
}

