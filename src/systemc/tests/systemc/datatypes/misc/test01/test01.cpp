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

// test of i/o streaming of the datatypes

#define SC_INCLUDE_FX
#include "systemc.h"
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
#   include "sstream"
#else
#   include "strstream.h"
#endif

// THE FOLLOWING SPECIALIZATIONS FOR sc_bitref<X> AND sc_subref<X> GO AWAY
// WHEN sc_bv and sc_lv ARE INTEGRATED INTO THE NORMAL CONCATENATION SCHEME:

template <class X>
void
test( sc_dt::sc_bitref<X> a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}

template <class X>
void
test( sc_dt::sc_subref<X> a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}


void
test( sc_dt::sc_fxnum_bitref a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}

void
test( sc_dt::sc_fxnum_fast_bitref a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}


void
test( sc_dt::sc_fxnum_subref a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}

void
test( sc_dt::sc_fxnum_fast_subref a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}

template <class T>
void
test( T& a )
{
# if (defined(__GNUC__) && (__GNUC__ >= 3))|| (defined(_MSC_VER) && (_MSC_VER >= 1300))
    std::stringstream ss;
# else
	strstream ss;
# endif

    cout << a << endl;
    ss << a;
    ss >> a;
    cout << a << endl;
}

void
test_bit()
{
    cout << "\n*** test_bit ***" << endl;

    // sc_bit
    {
        cout << "\nsc_bit" << endl;
        sc_bit a( true );
        sc_bit b( false );
        test( a );
        test( b );
    }

    // sc_logic
    {
        cout << "\nsc_logic" << endl;
        sc_logic a( SC_LOGIC_0 );
        sc_logic b( SC_LOGIC_1 );
        sc_logic c( SC_LOGIC_Z );
        sc_logic d( SC_LOGIC_X );
        test( a );
        test( b );
        test( c );
        test( d );
    }

    // sc_bv
    {
        cout << "\nsc_bv" << endl;
        sc_bv<4> a( "0101" );
        sc_bv<8> b( "11110000" );
        test( a );
        test( b );
    }

    // sc_lv
    {
        cout << "\nsc_lv" << endl;
        sc_lv<4> a( "01ZX" );
        sc_lv<8> b( "XXZZ1100" );
        test( a );
        test( b );
    }

    // sc_bitref
    {
        cout << "\nsc_bitref" << endl;
        sc_bv<4> a( "0101" );
        sc_lv<4> b( "01ZX" );
        test( a[0] );
        test( b[0] );
    }

    // sc_subref
    {
        cout << "\nsc_subref" << endl;
        sc_bv<4> a( "0101" );
        sc_lv<4> b( "01ZX" );
        test( a( 1, 0 ) );
        test( b( 1, 0 ) );
    }

    // sc_concref
    {
        cout << "\nsc_concref" << endl;
        sc_bv<4> a( "0101" );
        sc_lv<4> b( "01ZX" );
#if 0 // #### re-enable when concatenation support is homogenous.
        test( ( a[1], a[0] ) );
        test( ( b[1], b[0] ) );
#endif // 0
    }
}

void
test_int()
{
    cout << "\n*** test_int ***" << endl;

    // sc_int
    {
        cout << "\nsc_int" << endl;
        sc_int<4> a = -7;
        sc_int<8> b = 15;
        test( a );
        test( b );
    }

    // sc_int_bitref
    {
        cout << "\nsc_int_bitref" << endl;
        sc_int<4> a = -7;
        sc_int<8> b = 15;
        test( a[0] );
        test( b[0] );
    }

    // sc_int_subref
    {
        cout << "\nsc_int_subref" << endl;
        sc_int<4> a = -7;
        sc_int<8> b = 15;
        test( a( 3, 0 ) );
        test( b( 3, 0 ) );
    }

    // sc_int_concref
    {
        cout << "\nsc_int_concref" << endl;
        sc_int<4> a = -7;
        sc_int<8> b = 15;
        test( (a[1], a[0]) );
        test( (b[1], b[0]) );
    }

    // sc_uint
    {
        cout << "\nsc_uint" << endl;
        sc_uint<4> a = -7;
        sc_uint<8> b = 15;
        test( a );
        test( b );
    }

    // sc_uint_bitref
    {
        cout << "\nsc_uint_bitref" << endl;
        sc_uint<4> a = -7;
        sc_uint<8> b = 15;
        test( a[0] );
        test( b[0] );
    }

    // sc_uint_subref
    {
        cout << "\nsc_uint_subref" << endl;
        sc_uint<4> a = -7;
        sc_uint<8> b = 15;
        test( a( 3, 0 ) );
        test( b( 3, 0 ) );
    }

    // sc_uint_concref
    {
        cout << "\nsc_uint_concref" << endl;
        sc_uint<4> a = -7;
        sc_uint<8> b = 15;
        test( (a[1], a[0]) );
        test( (b[1], b[0]) );
    }

    // sc_bigint
    {
        cout << "\nsc_bigint" << endl;
        sc_bigint<4> a = -7;
        sc_bigint<8> b = 15;
        test( a );
        test( b );
    }

    // sc_signed_bitref
    {
        cout << "\nsc_signed_bitref" << endl;
        sc_bigint<4> a = -7;
        sc_bigint<8> b = 15;
        test( a[0] );
        test( b[0] );
    }

    // sc_signed_subref
    {
        cout << "\nsc_signed_subref" << endl;
        sc_bigint<4> a = -7;
        sc_bigint<8> b = 15;
        test( a( 3, 0 ) );
        test( b( 3, 0 ) );
    }

    // sc_signed_concref
    {
        sc_bigint<4> a = -7;
        sc_bigint<8> b = 15;
        test( (a[1], a[0]) );
        test( (b[1], b[0]) );
    }

    // sc_biguint
    {
        cout << "\nsc_biguint" << endl;
        sc_biguint<4> a = -7;
        sc_biguint<8> b = 15;
        test( a );
        test( b );
    }

    // sc_unsigned_bitref
    {
        cout << "\nsc_unsigned_bitref" << endl;
        sc_biguint<4> a = -7;
        sc_biguint<8> b = 15;
        test( a[0] );
        test( b[0] );
    }

    // sc_unsigned_subref
    {
        cout << "\nsc_unsigned_subref" << endl;
        sc_biguint<4> a = -7;
        sc_biguint<8> b = 15;
        test( a( 3, 0 ) );
        test( b( 3, 0 ) );
    }

    // sc_unsigned_concref
    {
        sc_biguint<4> a = -7;
        sc_biguint<8> b = 15;
        test( (a[1], a[0]) );
        test( (b[1], b[0]) );
    }
}

void
test_fx()
{
    cout << "\n*** test_fx ***" << endl;

    // sc_fxnum
    {
        cout << "\nsc_fxnum" << endl;
        sc_fixed<4,4> a = -7;
        sc_fixed<8,8> b = 15;
        test( a );
        test( b );
    }

    // sc_fxnum_fast
    {
        cout << "\nsc_fxnum_fast" << endl;
        sc_fixed_fast<4,4> a = -7;
        sc_fixed_fast<8,8> b = 15;
        test( a );
        test( b );
    }

    // sc_fxnum_bitref
    {
        cout << "\nsc_fxnum_bitref" << endl;
        sc_fixed<4,4> a = -7;
        sc_fixed<8,8> b = 15;
        test( a[0] );
        test( b[0] );
    }

    // sc_fxnum_fast_bitref
    {
        cout << "\nsc_fxnum_fast_bitref" << endl;
        sc_fixed_fast<4,4> a = -7;
        sc_fixed_fast<8,8> b = 15;
        test( a[0] );
        test( b[0] );
    }

    // sc_fxnum_subref
    {
        cout << "\nsc_fxnum_subref" << endl;
        sc_fixed<4,4> a = -7;
        sc_fixed<8,8> b = 15;
        test( a( 3, 0 ) );
        test( b( 3, 0 ) );
    }

    // sc_fxnum_fast_subref
    {
        cout << "\nsc_fxnum_fast_subref" << endl;
        sc_fixed_fast<4,4> a = -7;
        sc_fixed_fast<8,8> b = 15;
        test( a( 3, 0 ) );
        test( b( 3, 0 ) );
    }

    // sc_fxval
    {
        cout << "\nsc_fxval" << endl;
        sc_fxval a(-7);
        sc_fxval b(15);
        test( a );
        test( b );
    }

    // sc_fxval_fast
    {
        cout << "\nsc_fxval_fast" << endl;
        sc_fxval_fast a(-7);
        sc_fxval_fast b(15);
        test( a );
        test( b );
    }
}

int
sc_main( int, char*[] )
{
    test_bit();
    test_int();
    test_fx();

    return 0;
}
