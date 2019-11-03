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

  test02.cpp -- Test SC_FORK and SC_JOIN macros.

  Original Author: Andy Goodrich, Forte Design Systems, 10 October 2004
    
 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:    
    
 *****************************************************************************/

#include "systemc.h"

SC_MODULE(X)
{
	SC_CTOR(X)
	{

		SC_THREAD(waiting);
	}
	void sync(int context)
	{
		for ( int i = 0; i < context; i++ ) 
		{
			wait(m_clk.posedge_event());
		}
		cout << sc_time_stamp() << ": sync(" << context << ") terminating"<< endl;
	}
	void waiting()
	{
		SC_FORK
		    sc_spawn( sc_bind( &X::sync, this, 3 ) ),
		    sc_spawn( sc_bind( &X::sync, this, 4 ) ),
		    sc_spawn( sc_bind( &X::sync, this, 5 ) ),
		    sc_spawn( sc_bind( &X::sync, this, 5 ) ),
		    sc_spawn( sc_bind( &X::sync, this, 7 ) ),
		    sc_spawn( sc_bind( &X::sync, this, 11) ),
		    sc_spawn( sc_bind( &X::sync, this, 21) )
		SC_JOIN
		cout << sc_time_stamp() << ": waiting waking" << endl;
	}

	sc_in_clk m_clk;
	sc_join   m_join;
};

int sc_main( int argc, char* argv[] )
{
	sc_clock clock;
	X x("x");
	x.m_clk(clock);

	sc_start(1000, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}

