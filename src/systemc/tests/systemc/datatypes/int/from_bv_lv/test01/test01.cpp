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

// test of assigning sc_bv_base to the integer types

#include "systemc.h"

void
test_smaller( const sc_bv<4>& bv_one, const sc_bv<4>& bv_two )
{
    cout << "*** smaller ***" << endl;
    {
        sc_int<8> a( bv_one );
        sc_int<8> b( bv_two );
        sc_int<8> c;
        sc_int<8> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_uint<8> a( bv_one );
        sc_uint<8> b( bv_two );
        sc_uint<8> c;
        sc_uint<8> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_bigint<8> a( bv_one );
        sc_bigint<8> b( bv_two );
        sc_bigint<8> c;
        sc_bigint<8> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_biguint<8> a( bv_one );
        sc_biguint<8> b( bv_two );
        sc_biguint<8> c;
        sc_biguint<8> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
}

void
test_equal( const sc_bv<4>& bv_one, const sc_bv<4>& bv_two )
{
    cout << "*** equal ***" << endl;
    {
        sc_int<4> a( bv_one );
        sc_int<4> b( bv_two );
        sc_int<4> c;
        sc_int<4> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_uint<4> a( bv_one );
        sc_uint<4> b( bv_two );
        sc_uint<4> c;
        sc_uint<4> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_bigint<4> a( bv_one );
        sc_bigint<4> b( bv_two );
        sc_bigint<4> c;
        sc_bigint<4> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_biguint<4> a( bv_one );
        sc_biguint<4> b( bv_two );
        sc_biguint<4> c;
        sc_biguint<4> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
}

void
test_larger( const sc_bv<4>& bv_one, const sc_bv<4>& bv_two )
{
    cout << "*** larger ***" << endl;
    {
        sc_int<2> a( bv_one );
        sc_int<2> b( bv_two );
        sc_int<2> c;
        sc_int<2> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_uint<2> a( bv_one );
        sc_uint<2> b( bv_two );
        sc_uint<2> c;
        sc_uint<2> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_bigint<2> a( bv_one );
        sc_bigint<2> b( bv_two );
        sc_bigint<2> c;
        sc_bigint<2> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
    cout << endl;
    {
        sc_biguint<2> a( bv_one );
        sc_biguint<2> b( bv_two );
        sc_biguint<2> c;
        sc_biguint<2> d;
        c = bv_one;
        d = bv_two;
        cout << a << endl;
        cout << b << endl;
        cout << c << endl;
        cout << d << endl;
    }
}

int
sc_main( int, char*[] )
{
    sc_bv<4> bv_one( "0101" );
    sc_bv<4> bv_two( "1010" );

    test_smaller( bv_one, bv_two );
    test_equal( bv_one, bv_two );
    test_larger( bv_one, bv_two );

    return 0;
}
