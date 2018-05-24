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

  test10.cpp -- Testing proper process execution order for SC_METHOD murderer.

  Original Author: Andy Goodrich

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test10.cpp,v $
// Revision 1.2  2011/02/01 20:00:37  acg
//  Andy Goodrich: better messaging for output.
//
// Revision 1.1  2011/02/01 17:16:48  acg
//  Andy Goodrich: first check-in.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc.h"

SC_MODULE(DUT)
{
    SC_CTOR(DUT)
    {
	SC_METHOD(killer);
        sensitive << m_clk.pos();
        SC_CTHREAD(stimulus,m_clk.pos());
        SC_THREAD(thread0);
        sensitive << m_clk.pos();
        m_thread0 = sc_get_current_process_handle();
        SC_THREAD(thread1);
        sensitive << m_clk.pos();
        m_thread1 = sc_get_current_process_handle();
        SC_THREAD(thread2);
        sensitive << m_clk.pos();
        m_thread2 = sc_get_current_process_handle();
	m_kill = false;
    }

    void killer()
    {
        if ( m_kill )
	{
	    cout << sc_time_stamp() << " killer: killing thread0 " << endl;
	    m_thread0.kill();
	    cout << sc_time_stamp() << " killer: after killing thread0" << endl;
	    m_thread2.kill();
	    cout << sc_time_stamp() << " killer: after killing thread2" << endl;
	}
   } 

    void thread0()
    {
        cout << sc_time_stamp() << " thread 0: initialization" << endl;
        try {
            for (;;)
            {
                wait();
            }
        } 
        catch(sc_core::sc_unwind_exception& ex)
        {
	    if ( !ex.is_reset() )
	    {
		cout << sc_time_stamp() << " thread0: received kill" << endl;
		m_thread1.kill();
		cout << sc_time_stamp() << " thread0: after killing thread1"
		     << endl;
	    }
	    throw ex;
        }
    }

    void thread1()
    {
        cout << sc_time_stamp() << " thread 1: initialization" << endl;
        try {
            for (;;)
            {
                wait();
            }
        } 
        catch(sc_core::sc_unwind_exception& ex)
        {
	    if ( !ex.is_reset() )
	    {
		cout << sc_time_stamp() << " thread1: received kill" << endl;
	    }
	    throw ex;
        }
    }

    void thread2()
    {
        cout << sc_time_stamp() << " thread 2: initialization" << endl;
        try {
            for (;;)
            {
                wait();
            }
        } 
        catch(sc_core::sc_unwind_exception& ex)
        {
	    if ( !ex.is_reset() )
	    {
		cout << sc_time_stamp() << " thread2: received kill" << endl;
	    }
	    throw ex;
        }
    }

    void stimulus()
    {
        for (;;)
        {
            wait();
            wait();
            wait();
            wait();
	    cout << sc_time_stamp() << " stimulus setting kill" << endl;
	    m_kill = true;
            wait();
	    m_kill = false;
            wait();
            wait();
	    sc_stop();
        }
    }

    sc_in<bool>       m_clk;
    bool              m_kill;
    sc_process_handle m_thread0;
    sc_process_handle m_thread1;
    sc_process_handle m_thread2;
};

int sc_main(int argc, char* argv[])
{
    sc_clock        clock;
    DUT             dut("dut");

    dut.m_clk(clock);

    sc_start();

    cout << "Program completed" << endl;
    return 0;
}
