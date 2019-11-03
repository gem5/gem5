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

  test2.cpp -- Test use of sc_pending_activity_at_current_time()

  Original Author: Andy Goodrich, Forte Design Systems, 18 August 2006

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: 
  Description of Modification: - 

 *****************************************************************************/

#include "systemc.h"

SC_MODULE(DUT)
{
    SC_CTOR(DUT)
    {
        SC_THREAD(thread)
        sensitive << m_clk.pos();
        dont_initialize();
        SC_METHOD(cascade0_monitor);
        sensitive << m_cascade0;
        dont_initialize();
        SC_METHOD(cascade1_monitor);
        sensitive << m_cascade1;
        dont_initialize();
        SC_METHOD(cascade2_monitor);
        sensitive << m_cascade2;
        dont_initialize();
    }

    void cascade0_monitor()
    {
        cout << sc_time_stamp() << " " << sc_delta_count() << " cascade0"
             << endl;
        m_cascade1 = m_cascade0;
    }

    void cascade1_monitor()
    {
        cout << sc_time_stamp() << " " << sc_delta_count() << " cascade1"
             << endl;
        m_cascade2 = m_cascade1;
    }

    void cascade2_monitor()
    {
        cout << sc_time_stamp() << " " << sc_delta_count() << " cascade2"
             << endl;
    }

    void thread()
    {
        for (;;)
        {
            cout << sc_time_stamp() << " " << sc_delta_count() << " thread"
                 << endl;
            m_cascade0 = !m_cascade0.read();
            wait();
        }
    }
    sc_signal<bool> m_cascade0;
    sc_signal<bool> m_cascade1;
    sc_signal<bool> m_cascade2;
    sc_in<bool>     m_clk;
};

int sc_main(int argc, char* argv[])
{
    sc_clock        clock;
    DUT             dut("dut");

    dut.m_clk(clock);


    do { sc_start(0, SC_NS); } while (sc_pending_activity_at_current_time());
    cout << endl;
    sc_start(1, SC_NS);
    do { sc_start(0, SC_NS); } while (sc_pending_activity_at_current_time());
    cout << endl;
    sc_start(1, SC_NS);
    do { sc_start(0, SC_NS); } while (sc_pending_activity_at_current_time());

    cout << "Program completed" << endl;
    return 0;
}
