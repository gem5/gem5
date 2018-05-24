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
Ulli Holtmann, Nov/17/00

with SystemC 1.0.1, on sparcOS5, SC5.0 compiler, I can't assign a sc_bv::range() bit-slice
to a sc_bv:
*/

#include "systemc.h"

int
sc_main( int, char*[] )
{
   sc_bv<4>      bv4;
   sc_bv<10>      bv10 = 5;

    bv4 = bv10.range(4,1);
    // g++: OK
    // SC5.0: Error
    //  "str.cc", line 8: Error: Cannot assign 
    //  sc_bv_ns::sc_range<sc_bv_ns::sc_bv_base> to 
    //  sc_bv_ns::sc_bv<4> without 
    //  "sc_bv_ns::sc_bv<4>::operator=(const sc_bv_ns::sc_bv<4>&)";.
    cout << bv4 << endl;

    return 0;
}

/*
It works fine with g++, though.
*/
