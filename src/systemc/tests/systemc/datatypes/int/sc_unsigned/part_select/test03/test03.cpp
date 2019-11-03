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

  test03.cpp -- test sign extension in part select assignments.

  Original Author: Andy Goodrich, Forte Design Systems, 29 April 2008

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
#define TERM(EXPR) cout << #EXPR << " " << hex << (EXPR) << endl


int sc_main(int argc, char* argv[])
{
    sc_biguint<31>  a;
    sc_bigint<4>   b = 0xa;
    sc_bigint<5>   c = 0x0a;
    sc_biguint<4>  d = 0x0a;
    a = 0x12345678;
    a.range(15,8) = b;
    cout << hex << a << endl;
    a = 0x12345678;
    a.range(15,8) = c;
    cout << hex << a << endl;
    a = 0x12345678;
    a.range(15,8) = d;
    cout << hex << a << endl;

    cout << "Program completed" << endl;
    return 0;
}
