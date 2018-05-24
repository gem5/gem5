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
Jan/4/00 ulrich

In SystemC 1.0.1:
Conversion (via assignment or constructor) from an integer to sc_lv<32> is broken
on Sun SC5.0. I think it's an endless recursion.

It works fine with gcc. 

Example:
*/

#include "systemc.h"

int sc_main(int argc, char* arg[]) 
{
    int a;
    sc_lv<32> b;
    a=10;

    b=a;     // core dump
    cout << b << endl;
    b=sc_lv<32>(a);  // core dump
    cout << b << endl;

    return 0;
}



