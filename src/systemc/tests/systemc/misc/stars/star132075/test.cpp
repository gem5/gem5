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
Dec/6/01 ulrich

Comparison of bit-slices of sc_int and sc_uint vectors
does not work due to ambiguity. See example below.
It does work with sc_bigint, however.
*/

#include <systemc.h>

int sc_main(int, char**)
{
  {
    // sc_biguint<32> a,b;
    sc_uint<32> a,b;
    a=15;
    b=45;

    cout << (a.range(5,2) <  b.range(5,2)) << "\n";
    cout << (a.range(5,2) <= b.range(5,2)) << "\n";
    cout << (a.range(5,2) >  b.range(5,2)) << "\n";
    cout << (a.range(5,2) >= b.range(5,2)) << "\n";
  }

  {
    // sc_bigint<32> a,b;
    sc_int<32> a,b;
    a=15;
    b=45;

    cout << (a.range(5,2) <  b.range(5,2)) << "\n";
    cout << (a.range(5,2) <= b.range(5,2)) << "\n";
    cout << (a.range(5,2) >  b.range(5,2)) << "\n";
    cout << (a.range(5,2) >= b.range(5,2)) << "\n";
  }

  return 0;
}
