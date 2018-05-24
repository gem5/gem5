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
Dec/20/00 ulrich

I am using SystemC 1.0.1

This example causes some really ugly warnings when compiled with Solaris SC5.0.
Please note that there are actually 3 time more warnings than the one below. 
Although the simulation result is correct,  all these warnings make my hair stand up
and I wonder what I did wrong in my application code.

It works fine with gcc.
*/

#include "systemc.h"

int sc_main(int argc, char* arg[]) 
{
  sc_bv<8> bv8 = 3;
  int i;

  // gcc: OK
  // SC5.0:  works but strange warnings:
  //  "/home/pumba0/systemc-1.0.1/include/sc_bit_proxies.h", line 169: 
  //  Warning: bv hides sc_bv_ns::sc_range<sc_bv_ns::sc_bv_base>::bv.
  //   "/home/pumba0/systemc-1.0.1/include/sc_proxy.h", line 487:     
  //  Where: While instantiating "sc_bv_ns::sc_range<sc_bv_ns::sc_bv_base>
  //   ::sc_range(sc_bv_ns::sc_bv_base&, unsigned, unsigned)".
  //  "/home/pumba0/systemc-1.0.1/include/sc_proxy.h", line 487:
  //  Where: Instantiated from non-template code.
  i = (bv8.range(4, 1)).to_uint();

  cout << i << "\n";

  return 0;
}
