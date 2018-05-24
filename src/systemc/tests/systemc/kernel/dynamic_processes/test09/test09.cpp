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

  test09.cpp -- Test for hierarchical reset, try/catch and individual kill 
                processes

  Original Author: Andy Goodrich

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test09.cpp,v $
// Revision 1.4  2011/02/01 17:17:40  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//
// Revision 1.3  2011/01/14 14:23:46  acg
//  Andy Goodrich: Fixes for 1666_2011
//
// Revision 1.2  2009/10/14 19:13:27  acg
//  Andy Goodrich: changes for SystemC 2.3
//
// Revision 1.1.1.1  2006/12/15 20:26:03  acg
// systemc_tests-2.3
//
// Revision 1.1  2006/04/17 20:10:04  acg
//  Andy Goodrich: first inclusion of test for expanded process support.
//


#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "systemc.h"

SC_MODULE(DUT)
{
    SC_CTOR(DUT)
    {
        SC_CTHREAD(stimulus,m_clk.pos());
        reset_signal_is(m_reset, true);
        SC_THREAD(grand_parent);
        sensitive << m_clk.pos();
    }

    void child()
    {
        sc_process_handle my_handle = sc_get_current_process_handle();
        cout << sc_time_stamp() << " " << my_handle.name() 
             << " initialization" << endl;
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
		cout << sc_time_stamp() << " " << my_handle.name() 
		     << " got kill" << endl;
	    }
	    throw ex;
        }
    }

    void grand_parent()
    {
        static bool       initialize = true;
        cout << sc_time_stamp() << " " << "dut.grand_parent initialization" 
             << endl;
        cout << endl;
        if ( initialize )
        {
            m_grand_parent_handle = sc_get_current_process_handle();
            sc_spawn( sc_bind(&DUT::parent1, this), "parent1" );
            sc_spawn( sc_bind(&DUT::parent2, this), "parent2" );
            initialize = false;
        }

        for (;;)
        {
            wait();
        }
    }

    void parent1()
    {
        static bool initialize = true;
        sc_process_handle m_child1;
        sc_process_handle m_child2;
        sc_process_handle m_child3;
        sc_process_handle my_handle = sc_get_current_process_handle();
        cout << sc_time_stamp() << " " << my_handle.name() 
             << " initialization" << endl;
        if ( initialize )
        {
            m_child1 = sc_spawn( sc_bind(&DUT::child, this), "child1" );
            m_child2 = sc_spawn( sc_bind(&DUT::child, this), "child2" );
            m_child3 = sc_spawn( sc_bind(&DUT::child, this), "child3" );
            initialize = false;
        }

        for (;;)
        {
            wait();
        }
    }

    void parent2()
    {
        sc_process_handle m_child1;
        sc_process_handle m_child2;
        sc_process_handle m_child3;
        sc_process_handle my_handle;
        for (;;)
        {
            try
            {
                my_handle = sc_get_current_process_handle();
                cout << sc_time_stamp() << " " << my_handle.name() 
                     << " initialization" << endl;
                m_child1 = sc_spawn( sc_bind(&DUT::child, this), "child1" );
                m_child2 = sc_spawn( sc_bind(&DUT::child, this), "child2" );
                m_child3 = sc_spawn( sc_bind(&DUT::child, this), "child3" );

                for (;;)
                {
                    wait();
                }
            } 
            catch ( sc_core::sc_unwind_exception& ex )
            {
		if ( ex.is_reset() )
		{
		    cout << sc_time_stamp() << " " << my_handle.name() 
			 << " removing children" << endl;
		    m_child1.kill();
		    m_child2.kill();
		    m_child3.kill();
		}
		throw ex;
            }
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
	    cout << sc_time_stamp() << " stimulus issuing reset" << endl;
            m_grand_parent_handle.reset(SC_INCLUDE_DESCENDANTS);
        }
    }

    sc_in<bool>       m_clk;
    sc_process_handle m_grand_parent_handle;
    sc_in<bool>       m_reset;
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
    sc_start(20, SC_NS);

    cout << "Program completed" << endl;
    return 0;
}
