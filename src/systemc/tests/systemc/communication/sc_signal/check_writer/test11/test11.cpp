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

  test10.cpp -- Test detection of two write ports connected to 1 signal.

  Original Author: Andy Goodrich, Forte Design Systems, 15 Oct 2003

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test register port for more than one writer

#define DEBUG_SYSTEMC
#include "systemc.h"
typedef sc_int<5> target;

SC_MODULE( mod_a )
{
    sc_in_clk clk;

    sc_out<target>      out_target;
    sc_out<target>      out_target2;

    SC_CTOR( mod_a )
    {
    }
};

int
sc_main( int, char*[] )
{
    sc_clock clk;

    sc_signal<target> sig_target;

    mod_a a("a");
    a(clk, sig_target, sig_target);

    sc_start(1, SC_NS);

    return 0;
}
