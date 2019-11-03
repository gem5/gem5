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

  test01.cpp -- 

  Original Author: Andy Goodrich, Forte Design Systems, 8 December 2005

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: 
  Description of Modification: 

 *****************************************************************************/

// Test of return values for sc_process handle instances, along with 
// comparison operators.

#include "systemc.h"

SC_MODULE(DUT)
{
    SC_CTOR(DUT)
    {
        SC_CTHREAD(process_a, m_clk.pos());
        SC_THREAD(process_b)
        sensitive << m_clk.pos();
    }
    void process_a()
    {
        m_a = sc_get_current_process_handle();
        sc_process_handle b;
        sc_process_handle c = sc_get_current_process_handle();

        // TEST COMPARISONS:

        if ( m_a == b )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << " non-null process handle == null process handle" << endl;
        }
        if ( m_a != c )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << " process handles for same process not equal" << endl;
        }
        wait(1);

        // TEST RETURN VALUES:

        const std::vector<sc_object*>& objects = m_a.get_child_objects();
        if ( objects.size() != 0 )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << "get_child_objects() returned non-null vector" << endl;
        }
        if ( m_a.get_parent_object() == 0 )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << " get_parent_object() returned null value" << endl;
        }
        if ( !strcmp( m_a.name(), "") )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << "name() returned empty string" << endl;
        }
        if ( m_a.proc_kind() != SC_CTHREAD_PROC_ )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << "proc_kind() returned " << m_a.proc_kind() 
                 << " not " << SC_CTHREAD_PROC_ << endl;
        }
        if ( m_a.terminated() )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << "terminated() returned true" << endl;
        }
        if ( !m_a.valid() )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << "valid() returned false" << endl;
        }
    }
    void process_b()
    {
        wait(1);
        sc_process_handle b = sc_get_current_process_handle();
        if ( m_a == b )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << " process handles for two different processes were equal"
                 << endl;
        }
        if ( b.get_parent_object() == 0 )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << " get_parent_object() returned null value" << endl;
        }
        if ( b.proc_kind() != SC_THREAD_PROC_ )
        {
            cout << __FILE__ << " " << __LINE__ 
                 << "proc_kind() returned " << b.proc_kind() 
                 << " not " << SC_THREAD_PROC_ << endl;
        }
        wait(2);
        if ( m_a.valid() )
        {
            if ( !m_a.terminated() )
            {
                cout << __FILE__ << " " << __LINE__ 
                     << "terminated() returned false" << endl;
            }
        }
        else
        {
            if ( m_a.terminated() )
            {
                cout << __FILE__ << " " << __LINE__ 
                     << "terminated() returned true" << endl;
            }
        }
    }

    sc_process_handle m_a;
    sc_in<bool>       m_clk;
};


int sc_main(int argc, char* argv[])
{
    sc_clock          clock;
    DUT               dut("dut");
    sc_process_handle handle;
    sc_process_handle handle2;

    dut.m_clk(clock);
    if ( handle == handle2 )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " == operator returned true" << endl;
    }
    if ( !(handle != handle2) )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " != operator returned true" << endl;
    }
    const std::vector<sc_object*>& objects = handle.get_child_objects();
    if ( objects.size() != 0 )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " get_child_objects() returned non-null vector" << endl;
    }
    if ( handle.get_parent_object() != 0 )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " get_parent_object() returned non-null value" << endl;
    }
    if ( strcmp( handle.name(), "") )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " name() returned non-empty string" << endl;
    }
    if ( handle.proc_kind() != SC_NO_PROC_ )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " proc_kind() returned " << handle.proc_kind() 
             << " not " << SC_NO_PROC_ << endl;
    }
    if ( handle.terminated() )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " terminated() returned true" << endl;
    }
    if ( handle.valid() )
    {
        cout << __FILE__ << " " << __LINE__ 
             << " valid() returned true" << endl;
    }

    sc_start(10, SC_NS);
    cout << "Program completed" << endl;
    return 0;
}
