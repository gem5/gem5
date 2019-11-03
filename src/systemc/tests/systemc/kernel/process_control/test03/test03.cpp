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

  test03.cpp -- Test of disable enable on processes

  Original Author: Andy Goodrich

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

  Revision log at end of the file to let __LINE__ give the same results 
  after a check-in.
 *****************************************************************************/


#include "systemc.h"

SC_MODULE(DUT)
{
    SC_CTOR(DUT)
    {
        SC_CTHREAD(cthread,m_clk.pos());
        m_cthread = sc_get_current_process_handle();
        SC_METHOD(dynamic_method_and_events);

	m_dynamic_method_and_events = sc_get_current_process_handle();
        SC_METHOD(dynamic_method_negedge);
	m_dynamic_method_negedge = sc_get_current_process_handle();
        SC_METHOD(dynamic_method_posedge);
	m_dynamic_method_posedge = sc_get_current_process_handle();

        SC_THREAD(dynamic_thread_and_events);
        m_dynamic_thread_and_events = sc_get_current_process_handle();
        SC_THREAD(dynamic_thread_negedge);
        m_dynamic_thread_negedge = sc_get_current_process_handle();
        SC_THREAD(dynamic_thread_posedge);
        m_dynamic_thread_posedge = sc_get_current_process_handle();

        SC_METHOD(static_method);
        sensitive << m_clk.pos();
	dont_initialize();
        m_static_method = sc_get_current_process_handle();
        SC_THREAD(static_thread);
        sensitive << m_clk.pos();
        m_static_thread = sc_get_current_process_handle();
        SC_METHOD(stimulus_method);

        SC_CTHREAD(stimulus,m_clk.pos());
        reset_signal_is(m_reset, true);
    }

    void cthread()
    {
        for (;;)
        {
            wait();
            cout << sc_time_stamp() << ": cthread awakened" << endl;
        }
    }

    // dynamic_method_and_events - dynamic method waiting on the and of 
    // two events:

    void dynamic_method_and_events()
    {
        static int state = 0;
	switch( state )
	{
	  case 0:
	    cout << sc_time_stamp() 
	         << " dynamic_method_and_events: initialization call" << endl;
	    break;
	  case 1:
	    cout << sc_time_stamp() << " dynamic_method_and_events: awakened"
		 << endl;
	    break;
	}
	next_trigger( m_event1 & m_event2 );
	state = 1;
    }

    // dynamic_method_negedge - dynamic method waiting on negedge events:

    void dynamic_method_negedge()
    {
        static int state = 0;
	switch( state )
	{
	  case 0:
	    cout << sc_time_stamp() 
		 << " dynamic_method_negedge: initialization call" << endl;
	    break;
	  case 1:
	    cout << sc_time_stamp() << " dynamic_method_negedge: awakened"
		 << endl;
	    break;
	}
	next_trigger( m_clk.negedge_event() );
	state = 1;
    }

    // dynamic_method_posedge - dynamic method waiting on posedge events:

    void dynamic_method_posedge()
    {
        static int state = 0;
	switch( state )
	{
	  case 0:
	    cout << sc_time_stamp() 
		 << " dynamic_method_posedge: initialization call" << endl;
	    break;
	  default:
	    cout << sc_time_stamp() << " dynamic_method_posedge: awakened"
		 << endl;
	}
	next_trigger( m_clk.posedge_event() );
	state = 1;
    }

    // dynamic_thread_and_events - dynamic thread waiting on the and of 
    // two events:

    void dynamic_thread_and_events()
    {
	cout << sc_time_stamp() 
	     << " dynamic_thread_and_events: initialization call" << endl;
        for (;;)
	{
	    wait( m_event1 & m_event2 );
	    cout << sc_time_stamp() << " dynamic_thread_and_events: awakened"
	         << endl;
	}
    }

    // dynamic_thread_negedge - dynamic thread waiting on negedge events:

    void dynamic_thread_negedge()
    {
	cout << sc_time_stamp() 
	     << " dynamic_thread_negedge: initialization call" << endl;
        for (;;)
	{
	    wait( m_clk.negedge_event() );
	    cout << sc_time_stamp() << " dynamic_thread_negedge: awakened"
	         << endl;
	}
    }

    // dynamic_thread_posedge - dynamic thread waiting on posedge events:

    void dynamic_thread_posedge()
    {
	cout << sc_time_stamp() 
	     << " dynamic_thread_posedge: initialization call" << endl;
        for (;;)
	{
	    wait( m_clk.posedge_event() );
	    cout << sc_time_stamp() << " dynamic_thread_posedge: awakened"
	         << endl;
	}
    }

    void static_method()
    {
        cout << sc_time_stamp() << ": static method awakened" << endl;
    }
    void static_thread()
    {
        for (;;)
        {
            wait();
            cout << sc_time_stamp() << ": static thread awakened" << endl;
        }
    }
    void stimulus_method()
    {
	cout << "Status during sc_start(1,SC_NS) = " << sc_get_status() << endl;
	cout << sc_time_stamp() << ": stimulus ("
	     << __LINE__ << ") - disabling all processes" << endl;
	m_cthread.disable();
	m_dynamic_method_and_events.disable();
	m_dynamic_method_negedge.disable();
	m_dynamic_method_posedge.disable();
	m_dynamic_thread_and_events.disable();
	m_dynamic_thread_negedge.disable();
	m_dynamic_thread_posedge.disable();
	m_static_method.disable();
	m_static_thread.disable();
    }
    void stimulus()
    {
        for (;;)
        {
	    // START OUT BY WAITING ON THE DISABLE FROM THE stimulus_method.

            wait();
            wait();
            wait();

	    // PROCEED WITH AN ENABLE ON EVERYONE - EXPECTING posedge WAKES:

            m_cthread.enable();
            m_dynamic_method_and_events.enable();
            m_dynamic_method_negedge.enable();
            m_dynamic_method_posedge.enable();
            m_dynamic_thread_and_events.enable();
            m_dynamic_thread_negedge.enable();
            m_dynamic_thread_posedge.enable();
            m_static_method.enable();
            m_static_thread.enable();
            cout << endl << sc_time_stamp() << ": stimulus (" 
                 << __LINE__ << ") - enabling all processes" << endl;
            wait();

	    // DISABLE EVERYONE AGAIN:

            cout << endl << sc_time_stamp() << ": stimulus ("
                 << __LINE__ << ") - disabling all processes" << endl;
            m_cthread.disable();
            m_dynamic_method_and_events.disable();
            m_dynamic_method_negedge.disable();
            m_dynamic_method_posedge.disable();
            m_dynamic_thread_and_events.disable();
            m_dynamic_thread_negedge.disable();
            m_dynamic_thread_posedge.disable();
            m_static_method.disable();
            m_static_thread.disable();
            wait();

	    // PROCEED WITH AN ENABLE ON EVERYONE - EXPECTING negedge WAKES:

            cout << endl << sc_time_stamp() << ": stimulus (" 
                 << __LINE__ << ") - enabling all processes" << endl;
            m_cthread.enable();
            m_dynamic_method_and_events.enable();
            m_dynamic_method_negedge.enable();
            m_dynamic_method_posedge.enable();
            m_dynamic_thread_and_events.enable();
            m_dynamic_thread_negedge.enable();
            m_dynamic_thread_posedge.enable();
            m_static_method.enable();
            m_static_thread.enable();
            wait();

            // FIRE OFF EVENT 1:
            cout << endl << sc_time_stamp() << ": stimulus ("
                 << __LINE__ << ") - firing event1 " << endl;
            m_event1.notify(SC_ZERO_TIME);
            wait();

            cout << endl << sc_time_stamp() << ": stimulus ("
                 << __LINE__ << ") - disabling all processes" << endl;
            m_cthread.disable();
            m_dynamic_method_and_events.disable();
            m_dynamic_method_negedge.disable();
            m_dynamic_method_posedge.disable();
            m_dynamic_thread_and_events.disable();
            m_dynamic_thread_negedge.disable();
            m_dynamic_thread_posedge.disable();
            m_static_method.disable();
            m_static_thread.disable();
            wait();


	    // FIRE OFF EVENT 2: WITH EVERYONE DISABLED:

            m_event2.notify(SC_ZERO_TIME);
            cout << endl << sc_time_stamp() << ": stimulus ("
                 << __LINE__ << ") - firing event2 " << endl;
            wait();
            wait();
            wait();

	    // FIRE OFF EVENT 2: WITH EVERYONE ENABLED:

            cout << endl << sc_time_stamp() << ": stimulus (" 
                 << __LINE__ << ") - enabling all processes" << endl;
            m_cthread.enable();
            m_dynamic_method_and_events.enable();
            m_dynamic_method_negedge.enable();
            m_dynamic_method_posedge.enable();
            m_dynamic_thread_and_events.enable();
            m_dynamic_thread_negedge.enable();
            m_dynamic_thread_posedge.enable();
            m_static_method.enable();
            m_static_thread.enable();
            wait();

            m_event2.notify(SC_ZERO_TIME);
            cout << endl << sc_time_stamp() << ": stimulus ("
                 << __LINE__ << ") - firing event2 " << endl;
            wait();
            wait();
            sc_stop();
        }
    }
    sc_in<bool>       m_clk;
    sc_process_handle m_cthread;
    sc_process_handle m_dynamic_method_and_events;
    sc_process_handle m_dynamic_method_negedge;
    sc_process_handle m_dynamic_method_posedge;
    sc_process_handle m_dynamic_thread_and_events;
    sc_process_handle m_dynamic_thread_negedge;
    sc_process_handle m_dynamic_thread_posedge;
    sc_event          m_event1;
    sc_event          m_event2;
    sc_event          m_event3;
    sc_event          m_event4;
    sc_in<bool>       m_reset;
    sc_process_handle m_static_method;
    sc_process_handle m_static_thread;
};

int sc_main(int argc, char* argv[])
{
    sc_clock        clock;
    DUT             dut("dut");
    sc_signal<bool> reset;

    dut.m_clk(clock);
    dut.m_reset(reset);

    reset = true;
    cout << "Status before sc_start(1,SC_NS) = " << sc_get_status() << endl;
    sc_start(1, SC_NS);
    cout << "Status after sc_start(1,SC_NS) = " << sc_get_status() << endl;
    reset = false;
    sc_start();

    cout << "Program completed" << endl;
    return 0;
}

// $Log: test03.cpp,v $
// Revision 1.5  2011/02/20 13:44:06  acg
//  Andy Goodrich: updates for IEEE 1666 2011.
//
// Revision 1.4  2011/02/14 17:00:00  acg
//  Andy Goodrich: updated copyright and added cvs logging information inline.
//
// Revision 1.3  2011/01/14 14:23:58  acg
//  Andy Goodrich: Fixes for 1666_2011
//
// Revision 1.2  2009/05/22 16:07:26  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.1.1.1  2006/12/15 20:26:03  acg
// systemc_tests-2.3
//
// Revision 1.1  2006/12/14 21:40:10  acg
//  Andy Goodrich: moving test to new directory.
//
// Revision 1.2  2006/04/20 19:43:31  acg
//  Andy Goodrich: moved CVS log to end of file so that __LINE__ does not
//  change when a checkin is done.
//
// Revision 1.1  2006/04/17 20:10:55  acg
//  Andy Goodrich: First inclusion of test for suspend and resume support.
//
