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

  std_ulogic_tilda.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int sc_main(int ac, char *av[])
{

  sc_logic  value1 = SC_LOGIC_1;//'1';
  sc_logic  value0 = SC_LOGIC_0;//'0';

  sc_logic	a;  
  sc_logic	b;  
  sc_logic	c;  
  sc_logic	d;  
  sc_logic  e;

  a = !value1.to_bool();		
  b = !value0.to_bool();
  c = ~value1;
  d = ~value0;
  e = ~sc_logic('1');

  cout << "\n a = " << a << " (!1)"
       << "\n b = " << b << " (!0)"
       << "\n c = " << c << " (~1)"
       << "\n d = " << d << " (~0)"
       << "\n e = " << e << " (~1)"
       << endl;
  return 0;
} 
