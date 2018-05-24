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

  T_1_1_2_1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int sc_main(int argc, char* argv[] )
{
    sc_clock clock1("CLK1");
    sc_clock clock2("CLK2", 2, SC_NS);
    sc_clock clock3("CLK3", 3, SC_NS, 0.25);
    sc_clock clock4("CLK4", 4, SC_NS, 0.5, 0.5, SC_NS);
    sc_clock clock5("CLK5", 5, SC_NS, 0.75, 1.0, SC_NS, false);
    const char *name = clock1.name();
    // double period = clock2.period();
    sc_time period = clock2.period();
    double duty_cycle = clock3.duty_cycle();

    // clock4.pos();
    // clock5.neg();

    return 0;
}
