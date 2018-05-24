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

  test01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of the bitwise binary operators of sc_proxy<X>

#include "systemc.h"

int
sc_main( int, char*[] )
{
    sc_bv<32> bv = 10;
    sc_bv<32> lv = 11;

    int I = 12;
    unsigned U = 12;

    sc_bv<32> bv1;
    sc_bv<32> bv2;
    sc_lv<32> lv1;
    sc_lv<32> lv2;

    // &

    bv1 = bv;
    bv2 = bv;
    bv1 &= I;
    bv2 = bv2 & I;
    sc_assert( bv1 == bv2 );

    bv1 = bv;
    bv2 = bv;
    bv1 &= U;
    bv2 = bv2 & U;
    sc_assert( bv1 == bv2 );

    lv1 = lv;
    lv2 = lv;
    lv1 &= I;
    lv2 = lv2 & I;
    sc_assert( lv1 == lv2 );

    lv1 = lv;
    lv2 = lv;
    lv1 &= U;
    lv2 = lv2 & U;
    sc_assert( lv1 == lv2 );

    // |

    bv1 = bv;
    bv2 = bv;
    bv1 |= I;
    bv2 = bv2 | I;
    sc_assert( bv1 == bv2 );

    bv1 = bv;
    bv2 = bv;
    bv1 |= U;
    bv2 = bv2 | U;
    sc_assert( bv1 == bv2 );

    lv1 = lv;
    lv2 = lv;
    lv1 |= I;
    lv2 = lv2 | I;
    sc_assert( lv1 == lv2 );

    lv1 = lv;
    lv2 = lv;
    lv1 |= U;
    lv2 = lv2 | U;
    sc_assert( lv1 == lv2 );

    // ^

    bv1 = bv;
    bv2 = bv;
    bv1 ^= I;
    bv2 = bv2 ^ I;
    sc_assert( bv1 == bv2 );

    bv1 = bv;
    bv2 = bv;
    bv1 ^= U;
    bv2 = bv2 ^ U;
    sc_assert( bv1 == bv2 );

    lv1 = lv;
    lv2 = lv;
    lv1 ^= I;
    lv2 = lv2 ^ I;
    sc_assert( lv1 == lv2 );

    lv1 = lv;
    lv2 = lv;
    lv1 ^= U;
    lv2 = lv2 ^ U;
    sc_assert( lv1 == lv2 );

    cout << "OK" << endl;

    return 0;
}
