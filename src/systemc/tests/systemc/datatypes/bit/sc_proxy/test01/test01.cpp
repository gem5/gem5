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

#include "systemc.h"

#define WRITE(a) \
    cout << a.length() << endl; \
    cout << a << endl

void
test_ctors()
{
    cout << "*** test_ctors" << endl;
    {
        cout << "sc_bv_base()" << endl;

        sc_bv_base a;
        WRITE( a );
    }
    {
        cout << "sc_bv_base( int )" << endl;

        sc_bv_base a( 3 );
        WRITE( a );
    }
    {
        cout << "sc_bv_base( bool )" << endl;

        sc_bv_base a( true );
        WRITE( a );
    }
    {
        cout << "sc_bv_base( bool, int )" << endl;

        sc_bv_base a( true, 3 );
        WRITE( a );
    }
    {
        cout << "sc_bv_base( const char* )" << endl;

        sc_bv_base a( "0101" );
        WRITE( a );
        sc_bv_base b( "1010" );
        WRITE( b );
        sc_bv_base c( "0b0101" );
        WRITE( c );
        sc_bv_base d( "0b1010" );
        WRITE( d );
    }
    {
        cout << "sc_bv_base( const char*, int )" << endl;

        sc_bv_base a3( "0101", 3 );
        WRITE( a3 );
        sc_bv_base a4( "0101", 4 );
        WRITE( a4 );
        sc_bv_base a5( "0101", 5 );
        WRITE( a5 );

        sc_bv_base b3( "1010", 3 );
        WRITE( b3 );
        sc_bv_base b4( "1010", 4 );
        WRITE( b4 );
        sc_bv_base b5( "1010", 5 );
        WRITE( b5 );

        sc_bv_base c3( "0b0101", 3 );
        WRITE( c3 );
        sc_bv_base c4( "0b0101", 4 );
        WRITE( c4 );
        sc_bv_base c5( "0b0101", 5 );
        WRITE( c5 );

        sc_bv_base d3( "0b1010", 3 );
        WRITE( d3 );
        sc_bv_base d4( "0b1010", 4 );
        WRITE( d4 );
        sc_bv_base d5( "0b1010", 5 );
        WRITE( d5 );
    }
    {
        cout << "sc_bv_base( const sc_proxy<X>& )" << endl;

        sc_lv<4> x( "01ZX" );
        sc_bv_base a( x );
        WRITE( a );
    }
    {
        cout << "sc_bv_base( const sc_bv_base& )" << endl;

        sc_bv<4> x( "0110" );
        sc_bv_base a( x );
        WRITE( a );
    }
}

void
test_bitwise_complement()
{
    cout << "*** test_bitwise_complement" << endl;
    {
        cout << "sc_bv_base::b_not()" << endl;

        sc_bv_base a( "0110", 4 );
        WRITE( a );
        a.b_not();
        WRITE( a );
    }
    {
        cout << "sc_bv_base::operator ~ () const" << endl;

        sc_bv_base a( "0110", 4 );
        WRITE( a );
        sc_bv_base b( 4 );
        b = ~a;
        WRITE( b );
    }
    {
        cout << "sc_lv_base::b_not()" << endl;

        sc_lv_base a( "01ZX", 4 );
        WRITE( a );
        a.b_not();
        WRITE( a );
    }
    {
        cout << "sc_lv_base::operator ~ () const" << endl;

        sc_lv_base a( "01ZX", 4 );
        WRITE( a );
        sc_lv_base b( 4 );
        b = ~a;
        WRITE( b );
    }
    {
        cout << "sc_proxy<X>::b_not()" << endl;

        sc_lv_base a( "01ZX", 4 );
        WRITE( a );
        (a( 3, 2 ), a( 1, 0 )).b_not();
        WRITE( a );
    }
    {
        cout << "sc_proxy<X>::operator ~ () const" << endl;

        sc_lv_base a( "01ZX", 4 );
        WRITE( a );
        sc_lv_base b( 4 );
        b = ~(a( 3, 2 ), a( 1, 0 ));
        WRITE( b );
    }
}

void
test_bitwise_and()
{
    cout << "*** test_bitwise_and" << endl;
}

void
test_bitwise_or()
{
    cout << "*** test_bitwise_or" << endl;
}

void
test_bitwise_xor()
{
    cout << "*** test_bitwise_xor" << endl;
}

void
test_bitwise_left_shift()
{
    cout << "*** test_bitwise_left_shift" << endl;
    {
        cout << "sc_bv_base::operator <<= ( int )" << endl;

        sc_bv_base a( 1, 70 );
        a[0] = 0;
        WRITE( a );
	try {
	    a <<= -1;
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a <<= 3;
        WRITE( a );
        a <<= 33;
        WRITE( a );
        a <<= 72;
        WRITE( a );
    }
    {
        cout << "sc_bv_base::operator << ( int ) const" << endl;

        sc_bv_base a( 1, 70 );
        a[0] = 0;
        WRITE( a );
        sc_bv_base b( 70 );
	try {
	    b = a << -1;
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = a << 3;
        WRITE( b );
        b = a << 33;
        WRITE( b );
        b = a << 72;
        WRITE( b );
    }
    {
        cout << "sc_lv_base::operator <<= ( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    a <<= -1;
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a <<= 3;
        WRITE( a );
        a <<= 33;
        WRITE( a );
        a <<= 72;
        WRITE( a );
    }
    {
        cout << "sc_lv_base::operator << ( int ) const" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = a << -1;
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = a << 3;
        WRITE( b );
        b = a << 33;
        WRITE( b );
        b = a << 72;
        WRITE( b );
    }
    {
        cout << "sc_proxy<X>::operator <<= ( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    (a( 69, 20 ), a( 19, 0 )) <<= -1;
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        // FIX ME: BUG, the following two operations result in 0XX..
        (a( 69, 20 ), a( 19, 0 )) <<= 3;
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )) <<= 33;
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )) <<= 72;
        WRITE( a );
    }
    {
        cout << "sc_proxy<X>::operator << ( int ) const" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = (a( 69, 20 ), a( 19, 0 )) << -1;
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = (a( 69, 20 ), a( 19, 0 )) << 3;
        WRITE( b );
        b = (a( 69, 20 ), a( 19, 0 )) << 33;
        WRITE( b );
        b = (a( 69, 20 ), a( 19, 0 )) << 72;
        WRITE( b );
    }
}

void
test_bitwise_right_shift()
{
    cout << "*** test_bitwise_right_shift" << endl;
    {
        cout << "sc_bv_base::operator >>= ( int )" << endl;

        sc_bv_base a( 1, 70 );
        a[69] = 0;
        WRITE( a );
	try {
	    a >>= -1;
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a >>= 3;
        WRITE( a );
        a >>= 33;
        WRITE( a );
	a >>= 72;
	WRITE( a );
    }
    {
        cout << "sc_bv_base::operator >> ( int ) const" << endl;

        sc_bv_base a( 1, 70 );
        a[69] = 0;
        WRITE( a );
        sc_bv_base b( 70 );
	try {
	    b = a >> -1;
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = a >> 3;
        WRITE( b );
        b = a >> 33;
        WRITE( b );
        b = a >> 72;
        WRITE( b );
    }
    {
        cout << "sc_lv_base::operator >>= ( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    a >>= -1;
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a >>= 3;
        WRITE( a );
        a >>= 33;
        WRITE( a );
        a >>= 72;
        WRITE( a );
    }
    {
        cout << "sc_lv_base::operator >> ( int ) const" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = a >> -1;
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = a >> 3;
        WRITE( b );
        b = a >> 33;
        WRITE( b );
        b = a >> 72;
        WRITE( b );
    }
    {
        cout << "sc_proxy<X>::operator >>= ( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    (a( 69, 20 ), a( 19, 0 )) >>= -1;
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        (a( 69, 20 ), a( 19, 0 )) >>= 3;
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )) >>= 33;
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )) >>= 72;
        WRITE( a );
    }
    {
        cout << "sc_proxy<X>::operator >> ( int ) const" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = (a( 69, 20 ), a( 19, 0 )) >> -1;
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = (a( 69, 20 ), a( 19, 0 )) >> 3;
        WRITE( b );
        b = (a( 69, 20 ), a( 19, 0 )) >> 33;
        WRITE( b );
        b = (a( 69, 20 ), a( 19, 0 )) >> 72;
        WRITE( b );
    }
}

void
test_bitwise_left_rotate()
{
    cout << "*** test_bitwise_left_rotate" << endl;
    {
        cout << "sc_bv_base::lrotate( int )" << endl;

        sc_bv_base a( 1, 70 );
        a[0] = 0;
        WRITE( a );
	try {
	    a.lrotate( -1 );
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a.lrotate( 3 );
        WRITE( a );
        a.lrotate( 33 );
        WRITE( a );
        a.lrotate( 72 );
        WRITE( a );
    }
    {
        cout << "lrotate( const sc_bv_base&, int )" << endl;

        sc_bv_base a( 1, 70 );
        a[0] = 0;
        WRITE( a );
        sc_bv_base b( 70 );
	try {
	    b = lrotate( a, -1 );
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = lrotate( a, 3 );
        WRITE( b );
        b = lrotate( a, 33 );
        WRITE( b );
        b = lrotate( a, 72 );
        WRITE( b );
    }
    {
        cout << "sc_lv_base::lrotate( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    a.lrotate( -1 );
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a.lrotate( 3 );
        WRITE( a );
        a.lrotate( 33 );
        WRITE( a );
        a.lrotate( 72 );
        WRITE( a );
    }
    {
        cout << "lrotate( const sc_lv_base&, int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = lrotate( a, -1 );
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = lrotate( a, 3 );
        WRITE( b );
        b = lrotate( a, 33 );
        WRITE( b );
        b = lrotate( a, 72 );
        WRITE( b );
    }
    {
        cout << "sc_proxy<X>::lrotate( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    (a( 69, 20 ), a( 19, 0 )).lrotate( -1 );
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        (a( 69, 20 ), a( 19, 0 )).lrotate( 3 );
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )).lrotate( 33 );
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )).lrotate( 72 );
        WRITE( a );
    }
    {
        cout << "lrotate( const sc_proxy<X>&,  int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[0] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = lrotate( (a( 69, 20 ), a( 19, 0 )), -1 );
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = lrotate( (a( 69, 20 ), a( 19, 0 )), 3 );
        WRITE( b );
        b = lrotate( (a( 69, 20 ), a( 19, 0 )), 33 );
        WRITE( b );
        b = lrotate( (a( 69, 20 ), a( 19, 0 )), 72 );
        WRITE( b );
    }
}

void
test_bitwise_right_rotate()
{
    cout << "*** test_bitwise_right_rotate" << endl;
    {
        cout << "sc_bv_base::rrotate( int )" << endl;

        sc_bv_base a( 1, 70 );
        a[69] = 0;
        WRITE( a );
	try {
	    a.rrotate( -1 );
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a.rrotate( 3 );
        WRITE( a );
        a.rrotate( 33 );
        WRITE( a );
        a.rrotate( 72 );
        WRITE( a );
    }
    {
        cout << "rrotate( const sc_bv_base&, int )" << endl;

        sc_bv_base a( 1, 70 );
        a[69] = 0;
        WRITE( a );
        sc_bv_base b( 70 );
	try {
	    b = rrotate( a, -1 );
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = rrotate( a, 3 );
        WRITE( b );
        b = rrotate( a, 33 );
        WRITE( b );
        b = rrotate( a, 72 );
        WRITE( b );
    }
    {
        cout << "sc_lv_base::rrotate( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    a.rrotate( -1 );
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        a.rrotate( 3 );
        WRITE( a );
        a.rrotate( 33 );
        WRITE( a );
        a.rrotate( 72 );
        WRITE( a );
    }
    {
        cout << "rrotate( const sc_lv_base&, int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = rrotate( a, -1 );
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = rrotate( a, 3 );
        WRITE( b );
        b = rrotate( a, 33 );
        WRITE( b );
        b = rrotate( a, 72 );
        WRITE( b );
    }
    {
        cout << "sc_proxy<X>::rrotate( int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
	try {
	    (a( 69, 20 ), a( 19, 0 )).rrotate( -1 );
	    WRITE( a );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        // FIX ME: BUG, the following three operations result in 0XX..
        (a( 69, 20 ), a( 19, 0 )).rrotate( 3 );
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )).rrotate( 33 );
        WRITE( a );
        (a( 69, 20 ), a( 19, 0 )).rrotate( 72 );
        WRITE( a );
    }
    {
        cout << "rrotate( const sc_proxy<X>&,  int )" << endl;

        sc_lv_base a( SC_LOGIC_X, 70 );
        a[69] = SC_LOGIC_Z;
        WRITE( a );
        sc_lv_base b( 70 );
	try {
	    b = rrotate( (a( 69, 20 ), a( 19, 0 )), -1 );
	    WRITE( b );
	}
	catch( sc_report x ) {
	    cout << "\ncaught exception" << endl;
	    cout << x.what() << endl;
	}
        b = rrotate( (a( 69, 20 ), a( 19, 0 )), 3 );
        WRITE( b );
        b = rrotate( (a( 69, 20 ), a( 19, 0 )), 33 );
        WRITE( b );
        b = rrotate( (a( 69, 20 ), a( 19, 0 )), 72 );
        WRITE( b );
    }
}

void
test_bitwise_reverse()
{
    cout << "*** test_bitwise_reverse" << endl;
    {
	cout << "sc_bv_base::reverse()" << endl;

	sc_bv_base a( "1111000", 7 );
	WRITE( a );
	a.reverse();
	WRITE( a );
	sc_bv_base b( "11110000", 8 );
	WRITE( b );
	b.reverse();
	WRITE( b );
    }
    {
	cout << "reverse( const sc_bv_base& )" << endl;

	sc_bv_base a( "1111000", 7 );
	WRITE( a );
	sc_bv_base b( 7 );
	b = reverse( a );
	WRITE( b );
	sc_bv_base c( "11110000", 8 );
	WRITE( c );
	sc_bv_base d( 8 );
	d = reverse( c );
	WRITE( d );
    }
    {
	cout << "sc_lv_base::reverse()" << endl;

	sc_lv_base a( "01ZX01Z", 7 );
	WRITE( a );
	a.reverse();
	WRITE( a );
	sc_lv_base b( "01ZX01ZX", 8 );
	WRITE( b );
	b.reverse();
	WRITE( b );
    }
    {
	cout << "reverse( const sc_lv_base& )" << endl;

	sc_lv_base a( "01ZX01Z", 7 );
	WRITE( a );
	sc_lv_base b( 7 );
	b = reverse( a );
	WRITE( b );
	sc_lv_base c( "01ZX01ZX", 8 );
	WRITE( c );
	sc_lv_base d( 8 );
	d = reverse( c );
	WRITE( d );
    }
    {
	cout << "sc_proxy<X>::reverse()" << endl;

	sc_lv_base a( "01ZX01ZX", 8 );
	WRITE( a );
	(a( 7, 4 ), a( 3, 0 )).reverse();
	WRITE( a );
	(a( 0, 3 ), a( 4, 7 )).reverse();
	WRITE( a );
    }
    {
	cout << "reverse( const sc_proxy<X>& )" << endl;

	sc_lv_base a( "01ZX01ZX", 8 );
	WRITE( a );
	sc_lv_base b( 8 );
	b = reverse( (a( 7, 4 ), a( 3, 0 )) );
	WRITE( b );
	b = reverse( (a( 0, 3 ), a( 4, 7 )) );
	WRITE( b );
    }
}

void
test_string_conversions()
{
    cout << "*** test_string_conversions" << endl;
    {
	cout << "sc_bv_base" << endl;

	sc_bv_base a( 1, 8 );
	sc_bv_base b( 8 );
	std::string s;
	s = a.to_string();
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_BIN );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_BIN_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_BIN_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_OCT );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_OCT_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_OCT_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_HEX );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_HEX_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_HEX_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_DEC );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_CSD );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
    }
    {
	cout << "sc_lv_base" << endl;

	sc_lv_base a( SC_LOGIC_1, 8 );
	sc_lv_base b( 8 );
	std::string s;
	s = a.to_string();
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_BIN );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_BIN_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_BIN_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_OCT );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_OCT_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_OCT_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_HEX );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_HEX_US );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_HEX_SM );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_DEC );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
	s = a.to_string( SC_CSD );
	cout << s << endl;
	b = s.c_str();
	sc_assert( b == a );
    }
    {
	cout << "sc_proxy<X>" << endl;

	sc_lv_base a( SC_LOGIC_1, 8 );
	sc_lv_base b( 8 );
	std::string s;
	s = (a( 7, 4 ), a( 3, 0 )).to_string();
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_BIN );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_BIN_US );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_BIN_SM );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_OCT );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_OCT_US );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_OCT_SM );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_HEX );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_HEX_US );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_HEX_SM );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_DEC );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
	s = (a( 7, 4 ), a( 3, 0 )).to_string( SC_CSD );
	cout << s << endl;
	(b( 7, 4 ), b( 3, 0 )) = s.c_str();
	sc_assert( b == a );
    }
}

int
sc_main( int, char*[] )
{
    test_ctors();
    test_bitwise_complement();
    test_bitwise_and();
    test_bitwise_or();
    test_bitwise_xor();
    test_bitwise_left_shift();
    test_bitwise_right_shift();
    test_bitwise_left_rotate();
    test_bitwise_right_rotate();
    test_bitwise_reverse();
    test_string_conversions();

    return 0;
}
