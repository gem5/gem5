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

  test08.cpp -- Test for sc_spawn during update phase, including after stop.

  Original Author: Andy Goodrich

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test08.cpp,v $
// Revision 1.4  2011/02/20 13:43:44  acg
//  Andy Goodrich: updates for IEEE 1666 2011.
//
// Revision 1.3  2011/02/18 21:11:07  acg
//  Philipp A. Hartmann: rename ABC class to eliminate class with wingdi.h.
//
// Revision 1.2  2011/02/01 17:17:40  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc.h"

class prim_channel : public sc_prim_channel {
  public:
  	prim_channel(const char* name = sc_gen_unique_name("prim_channel") ) 
	    : sc_prim_channel(name)
	{}
	void thread()
	{
		cout << "thread here..." << endl;
	}
	void update()
	{
		cout << "update called..." << endl;
		sc_spawn( sc_bind(&prim_channel::thread,this), 
		          sc_gen_unique_name("thread"));
	}
	void write( int i )
	{
		request_update();
	}
};

SC_MODULE(DUT)
{
	SC_CTOR(DUT)
	{
		SC_CTHREAD(thread,m_clk.pos());
	}
	void thread()
	{
		for (;;)
		{
			wait();
			m_chan.write(0);
			wait();
			m_chan.write(0);
			sc_stop();
		}
	}
	sc_in<bool>  m_clk;
	prim_channel m_chan;
};
int sc_main(int argc, char* argv[])
{
	sc_clock        clock;
	DUT             dut("dut");

	dut.m_clk(clock);

	sc_start(10, SC_NS);

	cout << "Program completed" << endl;
	return 0;
}
