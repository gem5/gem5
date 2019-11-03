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
Please add support in form of constructors and assigment operators
such that assigments between types sc_int, sc_uint, sc_bigint, sc_biguint
work in all cases. Currently, assigning a sc_int or sc_uint does not
work in most cases.

The following test case shows were things don't work. I tried it with
g++ 2.92.5 and Sun SC6.1 on Solaris.
*/

#include <systemc.h>

int sc_main( int, char*[] )
{
  sc_int<8>      i8   =  8;
  sc_uint<9>     u9   =  9;
  sc_bigint<10>  bi10 = 10;
  sc_biguint<11> bu11 = 11;

  sc_int<8>      i8_2;
  sc_uint<9>     u9_2;
  sc_bigint<10>  bi10_2;
  sc_biguint<11> bu11_2;

  i8_2 = sc_int<8>(i8);
  cout << i8_2 << endl;
  i8_2 = sc_int<8>(u9);     // g++ 2.95.2: ambiguous
  cout << i8_2 << endl;
  i8_2 = sc_int<8>(bi10);
  cout << i8_2 << endl;
  i8_2 = sc_int<8>(bu11);
  cout << i8_2 << endl;

  cout << endl;

  u9_2 = sc_uint<9>(i8);     // g++ 2.95.2: ambiguous
  cout << u9_2 << endl;
  u9_2 = sc_uint<9>(u9);
  cout << u9_2 << endl;
  u9_2 = sc_uint<9>(bi10);
  cout << u9_2 << endl;
  u9_2 = sc_uint<9>(bu11);
  cout << u9_2 << endl;

  cout << endl;

  bi10_2 = sc_bigint<10>(i8);
  cout << bi10_2 << endl;
  bi10_2 = sc_bigint<10>(u9);
  cout << bi10_2 << endl;
  bi10_2 = sc_bigint<10>(bi10);
  cout << bi10_2 << endl;
  bi10_2 = sc_bigint<10>(bu11);
  cout << bi10_2 << endl;

  cout << endl;

  bu11_2 = sc_biguint<11>(i8);  // g++ 2.95.2: ambiguous, SC6.1: error
  cout << bu11_2 << endl;
  bu11_2 = sc_biguint<11>(u9);  // g++ 2.95.2: ambiguous, SC6.1: error
  cout << bu11_2 << endl;
  bu11_2 = sc_biguint<11>(bi10);
  cout << bu11_2 << endl;
  bu11_2 = sc_biguint<11>(bu11);
  cout << bu11_2 << endl;

  cout << endl;

  i8_2 = i8;
  cout << i8_2 << endl;
  i8_2 = u9;
  cout << i8_2 << endl;
  i8_2 = bi10;
  cout << i8_2 << endl;
  i8_2 = bu11;
  cout << i8_2 << endl;

  cout << endl;

  u9_2 = i8;
  cout << u9_2 << endl;
  u9_2 = u9;
  cout << u9_2 << endl;
  u9_2 = bi10;
  cout << u9_2 << endl;
  u9_2 = bu11;
  cout << u9_2 << endl;

  cout << endl;

  bi10_2 = i8;
  cout << bi10_2 << endl;
  bi10_2 = u9;
  cout << bi10_2 << endl;
  bi10_2 = bi10;
  cout << bi10_2 << endl;
  bi10_2 = bu11;
  cout << bi10_2 << endl;

  cout << endl;

  bu11_2 = i8;
  cout << bu11_2 << endl;
  bu11_2 = u9;
  cout << bu11_2 << endl;
  bu11_2 = bi10;
  cout << bu11_2 << endl;
  bu11_2 = bu11;
  cout << bu11_2 << endl;

  return 0;
}
