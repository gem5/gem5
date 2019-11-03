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

  test05.cpp -- 

  Original Author: Andy Goodrich, Forte Design Systems, 2005-09-12

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of error detection: interface being supplied twice to a multi-port.

#include "systemc.h"

SC_MODULE(TB)
{
	SC_CTOR(TB)
	{
		m_port(m_signal);
		m_multi_port(m_signal);
		m_multi_port(m_port);
	}
	sc_port<sc_signal_inout_if<int>,0> m_multi_port;
	sc_inout<int>                      m_port;
	sc_signal<int>                     m_signal;
};

int sc_main(int, char**) 
{
	sc_clock clock;
	TB       tb("tb");

	sc_start(1, SC_NS);
    return 0;
} 

