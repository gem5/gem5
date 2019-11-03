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

  test02.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of the reduce methods in the sc_[u]int datatypes -- functional notation

#include "systemc.h"

int
sc_main( int, char*[] )
{
    // 1) check the existence of the reduce methods

    sc_int<42> a = 42;

    // sc_int_base
    cout << endl;
    cout <<  and_reduce( a ) << endl;
    cout <<   or_reduce( a ) << endl;
    cout <<  xor_reduce( a ) << endl;
    cout << nand_reduce( a ) << endl;
    cout <<  nor_reduce( a ) << endl;
    cout << xnor_reduce( a ) << endl;

    // sc_int_subref
    cout << endl;
    cout <<  and_reduce( a( 7, 0 ) ) << endl;
    cout <<   or_reduce( a( 7, 0 ) ) << endl;
    cout <<  xor_reduce( a( 7, 0 ) ) << endl;
    cout << nand_reduce( a( 7, 0 ) ) << endl;
    cout <<  nor_reduce( a( 7, 0 ) ) << endl;
    cout << xnor_reduce( a( 7, 0 ) ) << endl;

    // sc_int_concref<T1,T2>
    cout << endl;
    cout <<  and_reduce( ( a( 7, 0 ), a( 15, 8 ) ) ) << endl;
    cout <<   or_reduce( ( a( 7, 0 ), a( 15, 8 ) ) ) << endl;
    cout <<  xor_reduce( ( a( 7, 0 ), a( 15, 8 ) ) ) << endl;
    cout << nand_reduce( ( a( 7, 0 ), a( 15, 8 ) ) ) << endl;
    cout <<  nor_reduce( ( a( 7, 0 ), a( 15, 8 ) ) ) << endl;
    cout << xnor_reduce( ( a( 7, 0 ), a( 15, 8 ) ) ) << endl;

    sc_uint<42> b = 42;

    // sc_uint_base
    cout << endl;
    cout <<  and_reduce( b ) << endl;
    cout <<   or_reduce( b ) << endl;
    cout <<  xor_reduce( b ) << endl;
    cout << nand_reduce( b ) << endl;
    cout <<  nor_reduce( b ) << endl;
    cout << xnor_reduce( b ) << endl;

    // sc_uint_subref
    cout << endl;
    cout <<  and_reduce( b( 7, 0 ) ) << endl;
    cout <<   or_reduce( b( 7, 0 ) ) << endl;
    cout <<  xor_reduce( b( 7, 0 ) ) << endl;
    cout << nand_reduce( b( 7, 0 ) ) << endl;
    cout <<  nor_reduce( b( 7, 0 ) ) << endl;
    cout << xnor_reduce( b( 7, 0 ) ) << endl;

    // sc_uint_concref<T1,T2>
    cout << endl;
    cout <<  and_reduce( ( b( 7, 0 ), b( 15, 8 ) ) ) << endl;
    cout <<   or_reduce( ( b( 7, 0 ), b( 15, 8 ) ) ) << endl;
    cout <<  xor_reduce( ( b( 7, 0 ), b( 15, 8 ) ) ) << endl;
    cout << nand_reduce( ( b( 7, 0 ), b( 15, 8 ) ) ) << endl;
    cout <<  nor_reduce( ( b( 7, 0 ), b( 15, 8 ) ) ) << endl;
    cout << xnor_reduce( ( b( 7, 0 ), b( 15, 8 ) ) ) << endl;

    // 2) check the functionality of the reduce methods

    sc_int<2> c2 = -1;
    cout << endl;
    cout << and_reduce( c2 ) << endl;
    cout << xor_reduce( c2 ) << endl;

    sc_int<3> c3 = -1;
    cout << endl;
    cout << and_reduce( c3 ) << endl;
    cout << xor_reduce( c3 ) << endl;

    sc_uint<2> d2 = sc_dt::uint_type( -1 );
    cout << endl;
    cout << and_reduce( d2 ) << endl;
    cout << xor_reduce( d2 ) << endl;

    sc_uint<3> d3 = sc_dt::uint_type( -1 );
    cout << endl;
    cout << and_reduce( d3 ) << endl;
    cout << xor_reduce( d3 ) << endl;

    sc_int<6> e;
    sc_uint<6> f;
    cout << endl;
    for( int i = 0; i >= -10; -- i ) {
        e = i;
        f = i;
        cout << xor_reduce( e ) << endl;
        cout << xor_reduce( f ) << endl;
    }
    
    return 0;
}
