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

  T_2_5_1_1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

#define MYNAME T_2_5_1_1

const int WIDTH = 13;
typedef sc_bv<WIDTH>         my_vector;
typedef sc_signal<my_vector> signal_vector;

#include "T_2_5.h"

sc_signed
foobar1( const sc_signed& a,
        sc_signed const& b )
{
    return a + b;
}

sc_unsigned
foobar2( const sc_unsigned& a,
         sc_unsigned const& b )
{
    return a + b;
}

sc_bv_base
foobar3( const sc_bv_base& a,
         sc_bv_base const& b )
{
    return a | b;
}

sc_lv_base
foobar4( const sc_lv_base& a,
         sc_lv_base const& b )
{
    return a & b;
}

void
MYNAME::entry()
{
    my_vector a;
    my_vector b;
    sc_biguint<WIDTH> c, d;
    sc_biguint<WIDTH*2> e;

    a = x;
    b = y;
    c = a;
    c += d;
    z = a | b;
    wait();
    a = x.read() | y.read();
    b = x.read() ^ y.read();
    c = b;
    c = c - d;
    z = a & b;
    wait();
    a = x.read() & y.read();
    b = x.read() | y.read();
    z = a ^ b;
    c = a;
    c = c * d;
    wait();
    a = ~ x.read();
    b = ~ y.read();
    c = b;
    e = c * d;
    z = a | b;
    wait();

    a1 = x.read().to_int();
    a2 = a1 + a1;
    b1 = y.read().to_uint();
    b2 = b1 + b1;
    c1 = x;
    c2 = (c1, "00");
    d1 = y.read();
    d2 = ("00", d1);
    wait();
}

int sc_main(int argc, char* argv[] )
{
  return 0;
}
