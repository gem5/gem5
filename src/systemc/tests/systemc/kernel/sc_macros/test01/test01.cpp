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

// test of sc_abs<T> for all SystemC arithmetic datatypes

#define SC_INCLUDE_FX
#include "systemc.h"

int
sc_main( int, char*[] )
{
    cout << "\n*** fx" << endl;

    // sc_fxval
    {
        sc_fxval a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_fxval_fast
    {
        sc_fxval_fast a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_fix
    {
        sc_fix a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_fix_fast
    {
        sc_fix_fast a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_ufix
    {
        sc_ufix a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_ufix_fast
    {
        sc_ufix_fast a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_fixed
    {
        sc_fixed<32,32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_fixed_fast
    {
        sc_fixed_fast<32,32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_ufixed
    {
        sc_ufixed<32,32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_ufixed_fast
    {
        sc_ufixed_fast<32,32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    cout << "\n*** int" << endl;

    // sc_signed
    {
        sc_signed a( 32 );
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_unsigned
    {
        sc_unsigned a( 32 );
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_bigint
    {
        sc_bigint<32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_biguint
    {
        sc_biguint<32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_signed_subref
    // sc_unsigned_subref

    // sc_int_base
    {
        sc_int_base a( 32 );
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_uint_base
    {
        sc_uint_base a( 32 );
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_int
    {
        sc_int<32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_uint
    {
        sc_uint<32> a;
        cout << endl;
        a = 1;
        cout << sc_abs( a ) << endl;
        a = -1;
        cout << sc_abs( a ) << endl;
    }

    // sc_int_subref
    // sc_uint_subref
    // sc_int_concref
    // sc_uint_concref

    return 0;
}
