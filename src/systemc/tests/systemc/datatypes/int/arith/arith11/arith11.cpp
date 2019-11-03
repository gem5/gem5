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

  arith11.cpp -- test auto increment on sc_bigint<N> and sc_biguint<N>

  Original Author: Andy Goodrich, Forte Design Systems, 24 March 2008

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>
 
int sc_main(int argc, char**argv)
{
  sc_biguint<3> a = 7;
  sc_bigint<3>  b = 7;

  ++a;
  cout << "a         is " << a << endl;
  cout << "a == 0    is " << (a==0) << endl;
  cout << "raw digit: " << *a.get_raw() << endl;
  cout << endl;

  ++b;
  cout << "b         is " << b << endl;
  cout << "b == 0    is " << (b==0) << endl;
  cout << "raw digit: " << *b.get_raw() << endl;

  return 0;
}
