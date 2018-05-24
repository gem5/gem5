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

/* Main file for memory simulation */

#include "accessor.h"
#include "ram.h"

int global_mem[4000];

int sc_main(int ac, char *av[])
{
  sc_signal<bool> cs("CS");
  sc_signal<bool> we("WE");
  signal_bool_vector10 addr("Address");
  signal_bool_vector32 data1("Data1");
  signal_bool_vector32 data2("Data2");

  sc_clock clk("Clock", 20, SC_NS, 0.5, 0.0, SC_NS);

  accessor A("Accessor", clk, data1, cs, we, addr, data2);
  ram R("Ram", data2, cs, we, addr, data1);

  sc_start(560, SC_NS);
  return 0;
}
