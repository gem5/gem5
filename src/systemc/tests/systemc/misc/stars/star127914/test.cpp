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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
When I do an  sc_biguint + sc_int  addition, then the
code is compiled fine but links fails because the 
operator was never defined.

This happens with SystemC 1.0.2 on Solaris with both
gcc and Sun SC compiler.
*/

#include <systemc.h>

int sc_main(int, char**)
{
  sc_int<8> i8;
  sc_biguint<8> bu8;

  i8=2;
  bu8=3;
  bu8 = bu8 + i8;
  cout << "sum is " << bu8 << "\n";

  return 0;
}

/*
Undefined			first referenced
 symbol  			    in file
operator+(sc_unsigned const &, sc_int_base const &)str.o
*/
