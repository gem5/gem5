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


/* Main file for accumulator simulation */

#include "testbench.h"
#include "accumulator.h"

int sc_main(int ac, char *av[])
{
  sc_signal<int> number;
  sc_signal<int> resulta;
  sc_signal<int> resultm;

  sc_clock clk("Clock", 20.0, SC_NS, 0.5, 0.0, SC_NS);

  testbench TBH("TB", clk, resulta,resultm, number);
  accumulator ACC("ACC", clk, number, resulta, resultm);

  sc_start(1000, SC_NS);
  return 0;
}
