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

  test_small.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"


int sc_main(int argc, char* argv[])
{
  sc_int<4> a(8);
  sc_uint<4> b(8);
  
  cout << "a = " << a << " " << "b = " << b << endl;
  a += 1;
  b += 1;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a -= 1;
  b -= 1;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a *= 2;
  b *= 2;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a = a * 2;
  b = b * 2;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a /= 2;
  b /= 2;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a = a/2;
  b = b/2;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a &= 0x04;
  b &= 0x04;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a |= 0x04;
  b |= 0x04;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a ^= 0x04;
  b ^= 0x04;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  int c = -8;
  c %= 6;
  a %= 6;
  b %= 6;
  cout << "a = " << a << " " << "b = " << b << " " << "c = " << c << endl;
  a = 8;
  b = 8;
  a <<= 1;
  b <<= 1;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  a >>= 1;
  b >>= 1;
  cout << "a = " << a << " " << "b = " << b << endl;
  a = 8;
  b = 8;
  cout << "a = " << a++ << " " << "b = " << b++ << endl;
  a = 8;
  b = 8;
  cout << "a = " << ++a << " " << "b = " << ++b << endl;
  a = 8;
  b = 8;
  cout << "a = " << a-- << " " << "b = " << b-- << endl;
  a = 8;
  b = 8;
  cout << "a = " << --a << " " << "b = " << --b << endl;
  return 0;
}
