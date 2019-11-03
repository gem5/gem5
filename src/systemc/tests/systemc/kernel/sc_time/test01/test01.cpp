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

// test of sc_time's methods

#include "systemc.h"

void
test_print()
{
    cout << "test_print" << endl;

    sc_time t1;
    cout << t1 << endl;

    uint64 v = 1230;
    sc_time t2 = sc_time::from_value( v );
    cout << t2 << endl;

    v *= 10000;
    sc_time t3 = sc_time::from_value( v );
    cout << t3 << endl;

    v *= 100;
    sc_time t4 = sc_time::from_value( v );
    cout << t4 << endl;

    v *= 10000;
    sc_time t5 = sc_time::from_value( v );
    cout << t5 << endl;

    v *= 100;
    sc_time t6 = sc_time::from_value( v );
    cout << t6 << endl;

    v *= 10000;
    sc_time t7 = sc_time::from_value( v );
    cout << t7 << endl;
}

void
test_constructors()
{
    cout << "test_constructors" << endl;

    sc_time t1;
    cout << t1 << endl;

    sc_time t2a( 0, SC_SEC );
    cout << t2a << endl;

    sc_time t2b( 1.2345, SC_NS );
    cout << t2b << endl;
    sc_time t2c( 1.2341, SC_NS );
    cout << t2c << endl;

    sc_time t2d( 1, SC_FS );
    cout << t2d << endl;

    sc_time t2e( -1.2345, SC_NS );
    cout << t2e << endl;
    sc_time t2f( -1.2341, SC_NS );
    cout << t2f << endl;

    char v1 = 1;
    signed char v2 = 2;
    unsigned char v3 = 3;
    short v4 = 4;
    unsigned short v5 = 5;
    int v6 = 6;
    unsigned int v7 = 7;
    long v8 = 8;
    unsigned long v9 = 9;
    float v10 = 10;
    double v11 = 11;

    sc_time t2g( v1, SC_NS );
    cout << t2g << endl;
    sc_time t2h( v2, SC_NS );
    cout << t2h << endl;
    sc_time t2i( v3, SC_NS );
    cout << t2i << endl;
    sc_time t2j( v4, SC_NS );
    cout << t2j << endl;
    sc_time t2k( v5, SC_NS );
    cout << t2k << endl;
    sc_time t2l( v6, SC_NS );
    cout << t2l << endl;
    sc_time t2m( v7, SC_NS );
    cout << t2m << endl;
    sc_time t2n( v8, SC_NS );
    cout << t2n << endl;
    sc_time t2o( v9, SC_NS );
    cout << t2o << endl;
    sc_time t2p( v10, SC_NS );
    cout << t2p << endl;
    sc_time t2q( v11, SC_NS );
    cout << t2q << endl;

    sc_time t3a( 0, SC_SEC );
    cout << t3a << endl;

    sc_time t3b( 1.2341, true );
    cout << t3b << endl;
    sc_time t3c( 1.2345, true );
    cout << t3c << endl;
    sc_time t3d( -1.2341, true );
    cout << t3d << endl;
    sc_time t3e( -1.2345, true );
    cout << t3e << endl;

    sc_time t3f( 1.2345, false );
    cout << t3f << endl;
    sc_time t3g( 1.5432, false );
    cout << t3g << endl;
    sc_time t3h( -1.2345, false );
    cout << t3h << endl;
    sc_time t3i( -1.5432, false );
    cout << t3i << endl;

#if !defined( _MSC_VER )
    sc_time t4a( 0ull, true );
    cout << t4a << endl;
    sc_time t4b( 25ull, true );
    cout << t4b << endl;
    sc_time t4c( 25ull, false );
    cout << t4c << endl;
#else
    sc_time t4a( 0ui64, true );
    cout << t4a << endl;
    sc_time t4b( 25ui64, true );
    cout << t4b << endl;
    sc_time t4c( 25ui64, false );
    cout << t4c << endl;
#endif

    sc_time t5( t4c );
    cout << t5 << endl;
}

void
test_assignment()
{
    cout << "test_assignment" << endl;

    sc_time t1;

    sc_time t2;
    t1 = t2;
    cout << t1 << endl;

    sc_time t3( 1.2345, SC_NS );
    t1 = t3;
    cout << t1 << endl;

    sc_time t4( -1.5432, SC_NS );
    t1 = t4;
    cout << t1 << endl;
}

void
test_conversion()
{
    cout << "test_conversion" << endl;

    sc_time t1;
    cout << t1.value() << endl;
    cout << t1.to_double() << endl;
    cout << t1 << endl;
    cout << t1.to_seconds() << endl;

    sc_time t2( 1.2345, SC_US );
    cout << t2.value() << endl;
    cout << t2.to_double() << endl;
    cout << t2 << endl;
    cout << t2.to_seconds() << endl;

    sc_time t3( -1.5432, SC_NS );
    cout << t3.value() << endl;
    cout << t3.to_double() << endl;
    cout << t3 << endl;
    cout << t3.to_seconds() << endl;
}

void
test_relational()
{
    cout << "test_relational" << endl;

    sc_time t1;
    sc_time t2( 1, SC_FS );
    sc_time t3( 1.2345, SC_NS );
    sc_time t4( 1.2341, SC_NS );
    sc_time t5( -1.5432, SC_NS );

    cout << ( t1 == t2 ) << endl;
    cout << ( t1 != t2 ) << endl;
    cout << ( t1 <  t2 ) << endl;
    cout << ( t1 <= t2 ) << endl;
    cout << ( t1 >  t2 ) << endl;
    cout << ( t1 >= t2 ) << endl;

    cout << ( t3 == t4 ) << endl;
    cout << ( t3 != t4 ) << endl;
    cout << ( t3 <  t4 ) << endl;
    cout << ( t3 <= t4 ) << endl;
    cout << ( t3 >  t4 ) << endl;
    cout << ( t3 >= t4 ) << endl;

    cout << ( t1 == t5 ) << endl;
    cout << ( t1 != t5 ) << endl;
    cout << ( t1 <  t5 ) << endl;
    cout << ( t1 <= t5 ) << endl;
    cout << ( t1 >  t5 ) << endl;
    cout << ( t1 >= t5 ) << endl;
}

void
test_arithmetic()
{
    cout << "test_arithmetic" << endl;

    sc_time t1;
    sc_time t2( 1, SC_FS );
    sc_time t3( 1.2345, SC_NS );
    sc_time t4( 1.2341, SC_NS );
    sc_time t5( -1.5432, SC_NS );

    cout << ( t1 + t2 ) << endl;
    cout << ( t1 + t3 ) << endl;
    cout << ( t1 + t5 ) << endl;
    cout << ( t3 + t2 ) << endl;
    cout << ( t3 + t4 ) << endl;
    cout << ( t3 + t5 ) << endl;
    cout << ( t5 + t5 ) << endl;

    cout << ( t1 - t2 ) << endl;
    cout << ( t1 - t3 ) << endl;
    cout << ( t1 - t5 ) << endl;
    cout << ( t3 - t4 ) << endl;
    cout << ( t3 - t5 ) << endl;
    cout << ( t5 - t5 ) << endl;

    cout << ( t1 * 1.2345 ) << endl;
    cout << ( 1.2345 * t3 ) << endl;
    cout << ( t4 * 2 ) << endl;

    cout << ( t1 / 1.2345 ) << endl;
    cout << ( t4 / 2 ) << endl;
    cout << ( t3 / t4 ) << endl;

    cout << ( t2 += t3 ) << endl;
    cout << ( t2 -= t4 ) << endl;
    cout << ( t2 *= 1.2345 ) << endl;
    cout << ( t2 /= 2 ) << endl;
}

void
test_SC_ZERO_TIME()
{
    cout << "test_SC_ZERO_TIME" << endl;

    cout << SC_ZERO_TIME << endl;
}

int
sc_main( int, char*[] )
{
#if defined(_MSC_VER) && _MSC_VER < 1900
     _set_output_format(_TWO_DIGIT_EXPONENT);
#endif
    test_print();
    test_constructors();
    test_assignment();
    test_conversion();
    test_relational();
    test_arithmetic();
    test_SC_ZERO_TIME();

    return 0;
}
