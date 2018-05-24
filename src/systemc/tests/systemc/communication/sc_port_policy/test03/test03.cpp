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

  test03.cpp -- Test bind policy: one required not bound.

  Original Author: Andy Goodrich, Forte Design Systems, 02 September 2005

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h> 

// #define ALL_ERROR // Force error on all bound port.
// #define BIND_NONE // Bind port with zero or more bound.
#define ONE_ERROR // Force error on one or more bound port.

SC_MODULE(TB)
{
    SC_CTOR(TB) : m_all("all"), m_none("none"), m_one("one")
    {
        SC_CTHREAD(thread, m_clk.pos());
    }
    void thread()
    {
        for (;;)
        {
            wait();
            cout << sc_time_stamp() << ":" << endl;
            cout << "    all[0] = " << m_all[0]->read() << endl;
            cout << "    all[1] = " << m_all[1]->read() << endl;
            cout << "    one    = " << m_one->read() << endl;
#           if defined(BIND_NONE)
                cout << "    none   = " << m_none->read() << endl;
#           endif
        }
    }
    sc_port<sc_signal_in_if<bool>,2,SC_ALL_BOUND>          m_all;
    sc_in<bool>                                            m_clk;
    sc_port<sc_signal_in_if<bool>,2,SC_ZERO_OR_MORE_BOUND> m_none;
    sc_port<sc_signal_in_if<bool>,2,SC_ONE_OR_MORE_BOUND>  m_one;
};

int sc_main (int argc , char *argv[]) { 
   sc_clock        clock;
   sc_signal<bool> sig1;
   sc_signal<bool> sig2;

   TB       tb("tb"); 

   tb.m_clk(clock);

#  if !defined(ONE_ERROR)
       tb.m_one(sig1);
#  endif
#  if defined(BIND_NONE)
       tb.m_none(sig2);
#  endif
   tb.m_all(sig1);
#  if !defined(ALL_ERROR)
       tb.m_all(sig2);
#  endif

   sc_start(1, SC_NS); 
   sig1 = true;
   sc_start(1, SC_NS); 
   sig2 = true;
   sc_start(1, SC_NS); 
   sig1 = false;
   sig2 = false;
   sc_start(1, SC_NS); 
   return 0; 
} 

