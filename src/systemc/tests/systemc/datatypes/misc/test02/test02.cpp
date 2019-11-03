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

// test of explicit conversions of the datatypes

#define SC_INCLUDE_FX
#include "systemc.h"


template <class T>
void
write1( const T& a )
{
    cout << endl;
    cout << a.to_int() << endl;
    cout << a.to_uint() << endl;
    cout << a.to_long() << endl;
    cout << a.to_ulong() << endl;
}

template <class T>
void
write2( const T& a )
{
    write1( a );
    cout << a.to_double() << endl;
}

template <class T>
void
write3( const T& a )
{
    write2( a );
    cout << a.to_int64() << endl;
    cout << a.to_uint64() << endl;
}


void
test_bit()
{
    cout << "\n*** test_bit()" << endl;

    sc_bv<8> a( "10101010" );
    write1( a );
    write1( a.range( 3, 0 ) );

    sc_lv<8> b( "11001100" );
    write1( b );
    write1( b.range( 3, 0 ) );

    write1( ( a.range( 3, 0 ), b.range( 3, 0 ) ) );
}

void
test_fx()
{
    cout << "\n*** test_fx()" << endl;

    sc_fixed<8,4> a( "0b1010.1010" );
    write2( a );
    write1( a.range( 3, 0 ) );

    sc_fixed_fast<8,4> b( "0b1010.1010" );
    write2( b );
    write1( b.range( 3, 0 ) );

    sc_ufixed<8,4> c( "0b1010.1010" );
    write2( c );
    write1( c.range( 3, 0 ) );

    sc_ufixed_fast<8,4> d( "0b1010.1010" );
    write2( d );
    write1( d.range( 3, 0 ) );
}

void
test_int()
{
    cout << "\n*** test_int()" << endl;

    sc_bigint<8> a( "0b10101010" );
    write3( a );
    write3( a.range( 3, 0 ) );

    sc_biguint<8> b( "0b10101010" );
    write3( b );
    write3( b.range( 3, 0 ) );

    sc_int<8> c( -86 );
    write3( c );
    write3( c.range( 3, 0 ) );
    write3( ( c.range( 7, 4 ), c.range( 3, 0 ) ) );

    sc_uint<8> d( 170 );
    write3( d );
    write3( d.range( 3, 0 ) );
    write3( ( d.range( 7, 4 ), d.range( 3, 0 ) ) );
}

int
sc_main( int, char*[] )
{
    test_bit();
    test_fx();
    test_int();

    return 0;
}
