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

// test of sc_uint's concat operators -- basic functionality, operator , ()

#include "systemc.h"

void
test_concat( const sc_uint<8>& a )
{
    sc_uint<8> b;
    sc_uint<4> c;
    sc_uint<4> d;

    cout << a << endl;

    cout << "*** sc_uint_base" << endl;

    ( c, d ) = a;
    cout << c << endl;
    cout << d << endl;

    b = ( d, c );
    cout << b << endl;

    cout << "---" << endl;

    ( c, d[1] ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = ( c, d[1] );
    cout << b << endl;

    cout << "---" << endl;

    ( c, d.range( 1, 0 ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = ( c, d.range( 1, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    ( c, ( d[0], d[1] ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = ( c, ( d[0], d[1] ) );
    cout << b << endl;

    cout << "*** sc_uint_bitref" << endl;

    ( d[1], c ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = ( d[1], c );
    cout << b << endl;

    cout << "---" << endl;

    ( d[0], d[1] ) = a.range( 1,0 );
    cout << d << endl;

    b.range( 1, 0 ) = ( d[1], d[0] );
    cout << b << endl;

    cout << "---" << endl;

    ( d[0], c.range( 3, 0 ) ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = ( d[0], c.range( 3, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    ( d[0], ( d[1], d[2], d[3] ) ) = a.range( 3, 0 );
    cout << d << endl;

    b.range( 3, 0 ) = ( d[3], ( d[2], d[1], d[0] ) );
    cout << b << endl;

    cout << "*** sc_uint_subref" << endl;

    ( c.range( 3, 0 ), d ) = a;
    cout << c << endl;
    cout << d << endl;

    b = ( c.range( 3, 0 ), d );
    cout << b << endl;

    cout << "---" << endl;

    ( c.range( 3, 0 ), d[1] ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = ( c.range( 3, 0 ), d[1] );
    cout << b << endl;

    cout << "---" << endl;

    ( c.range( 3, 0 ), d.range( 3, 0 ) ) = a;
    cout << c << endl;
    cout << d << endl;

    b = ( d.range( 3, 0 ), c.range( 3, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    ( c.range( 3, 0 ), ( d[0], d[1] ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = ( c.range( 3, 0 ), ( d[1], d[0] ) );
    cout << b << endl;

    cout << "*** sc_uint_concat" << endl;

    ( ( c[1], c[0] ), d ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = ( ( c[0], c[1] ), d );
    cout << b << endl;

    cout << "---" << endl;

    ( ( c[1], c[0] ), d[0] ) = a.range( 2, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 2, 0 ) = ( ( c[0], c[1] ), d[0] );
    cout << b << endl;

    cout << "---" << endl;

    ( ( c[1], c[0] ), d.range( 3, 0 ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = ( ( c[0], c[1] ), d.range( 3, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    ( ( c[0], c[1] ), ( c[2], c[3] ) ) = a.range( 3, 0 );
    cout << c << endl;

    b.range( 3, 0 ) = ( ( c[3], c[2] ), ( c[1], c[0] ) );
    cout << b << endl;
}

int
sc_main( int, char*[] )
{
    sc_uint<8> a( 33 );

    test_concat( a );

    return 0;
}
