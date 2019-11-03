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

  main.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// Main routine
 
#include "systemc.h"
#include "test.h"
#include "tb.h"
#include "monitor.h"
#include "define.h"
 
int sc_main(int ac, char *av[])
{
  sc_clock clock("Clock", CLOCK_PERIOD, SC_NS, DUTY_CYCLE, 0, SC_NS);
  sc_clock tb_clock("TBClock", TB_CLOCK_PERIOD, SC_NS, DUTY_CYCLE, 0, SC_NS);
  sc_clock mon_clock("MonClock", CLOCK_PERIOD, SC_NS, DUTY_CYCLE, 75, SC_NS);
 
  sc_signal<bool> reset_sig;

  sc_signal<int> i1;
  sc_signal<int> i2;
  sc_signal<int> i3;
  sc_signal<int> i4;
  sc_signal<int> i5;
  
  sc_signal<bool> cont1;
  sc_signal<bool> cont2;
  sc_signal<bool> cont3;
  
  sc_signal<int> o1;
  sc_signal<int> o2;
  sc_signal<int> o3;
  sc_signal<int> o4;
  sc_signal<int> o5;

  test TEST ("TEST", clock, reset_sig, i1, i2, i3, i4, i5,
	 cont1, cont2, cont3, o1, o2, o3, o4, o5);
  tb TB ("TB", tb_clock, reset_sig, i1, i2, i3, i4, i5,
	 cont1, cont2, cont3, o1, o2, o3, o4, o5);
  monitor MONITOR ("MONITOR", mon_clock, reset_sig, i1, i2, i3, i4, i5,
	 cont1, cont2, cont3, o1, o2, o3, o4, o5);

  // Simulation Run Control
  sc_start();
  return 0;
}
