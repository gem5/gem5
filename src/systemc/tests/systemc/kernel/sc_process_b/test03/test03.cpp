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

  test03.cpp -- 

  Original Author: Andy Goodrich, Forte Design Systems, 27 July 2005

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


// This tests that one can get process handles on static processes.

#include "systemc.h"


SC_MODULE(Test) {
    sc_in<bool> m_clk;

    void method() {
      sc_process_handle handle = sc_get_current_process_handle(); 
      cout << handle.name() << " " << handle.proc_kind() << endl; 
    }
    void thread() {
		for (;;)
		{
			wait();
			sc_process_handle handle = sc_get_current_process_handle(); 
			cout << handle.name() << " " << handle.proc_kind() << endl; 
		}
    }
    SC_CTOR(Test) {
        SC_METHOD(method);
        sensitive << m_clk.neg();
        sc_process_handle method_handle = sc_get_current_process_handle(); 
        cout << name() << ".method " << method_handle.proc_kind() << endl; 
        SC_CTHREAD(thread,m_clk.pos());
        sc_process_handle thread_handle = sc_get_current_process_handle(); 
        cout << name() << ".thread " << thread_handle.proc_kind() << endl; 
    }
};


int sc_main(int argc,char *argv[]) {
  
    Test t1("t1");
    sc_clock clk("clk",10,SC_NS);
    
    t1.m_clk(clk);
    
    sc_start(50,SC_NS);
    return 0;
}
