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

  test1.cpp -- 

  Original Author: Andy Goodrich, Forte Design Systems 16 July 2004

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// TEST THAT sc_gen_unique_user_name() GENERATES THE CORRECT NAMING.

#include "systemc.h"

SC_MODULE(A) 
{
	SC_CTOR(A)
	{
		SC_CTHREAD(abc, m_clk.pos());
		SC_CTHREAD(abc, m_clk.pos());
		SC_METHOD(def);
		SC_METHOD(def);
		SC_METHOD(def);
		SC_THREAD(ghi);
		SC_THREAD(ghi);
	}
	void abc()
	{
		sc_curr_proc_handle cpi = sc_get_curr_simcontext()->get_curr_proc_info();
		cout << cpi->process_handle->name() << endl;
	}
	void def()
	{
		sc_curr_proc_handle cpi = sc_get_curr_simcontext()->get_curr_proc_info();
		cout << cpi->process_handle->name() << endl;
	}
	void ghi()
	{
		sc_curr_proc_handle cpi = sc_get_curr_simcontext()->get_curr_proc_info();
		cout << cpi->process_handle->name() << endl;
	}
	sc_in_clk m_clk;
};

int sc_main(int argc, char* argv[])
{
	A        a("a");
	sc_clock clock;
	a.m_clk(clock);

	sc_start(10, SC_NS);
    return 0;
}
