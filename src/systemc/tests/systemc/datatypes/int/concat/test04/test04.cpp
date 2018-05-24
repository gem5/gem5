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

  test04.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_int's concat operators -- basic functionality, concat()

#include "systemc.h"

void
test_concat( const sc_int<8>& a )
{
    sc_int<8> b;
    sc_int<4> c;
    sc_int<4> d;

    cout << a << endl;

    cout << "*** sc_int_base" << endl;

    concat( c, d ) = a;
    cout << c << endl;
    cout << d << endl;

    b = concat( d, c );
    cout << b << endl;

    cout << "---" << endl;

    concat( c, d[1] ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = concat( c, d[1] );
    cout << b << endl;

    cout << "---" << endl;

    concat( c, d.range( 1, 0 ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = concat( c, d.range( 1, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    concat( c, ( d[0], d[1] ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = concat( c, ( d[0], d[1] ) );
    cout << b << endl;

    cout << "*** sc_int_bitref" << endl;

    concat( d[1], c ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = concat( d[1], c );
    cout << b << endl;

    cout << "---" << endl;

    concat( d[0], d[1] ) = a.range( 1,0 );
    cout << d << endl;

    b.range( 1, 0 ) = concat( d[1], d[0] );
    cout << b << endl;

    cout << "---" << endl;

    concat( d[0], c.range( 3, 0 ) ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = concat( d[0], c.range( 3, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    concat( d[0], concat( concat( d[1], d[2] ), d[3] ) ) = a.range( 3, 0 );
    cout << d << endl;

    b.range( 3, 0 ) = concat( d[3], concat( concat( d[2], d[1] ), d[0] ) );
    cout << b << endl;

    cout << "*** sc_int_subref" << endl;

    concat( c.range( 3, 0 ), d ) = a;
    cout << c << endl;
    cout << d << endl;

    b = concat( c.range( 3, 0 ), d );
    cout << b << endl;

    cout << "---" << endl;

    concat( c.range( 3, 0 ), d[1] ) = a.range( 4, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 4, 0 ) = concat( c.range( 3, 0 ), d[1] );
    cout << b << endl;

    cout << "---" << endl;

    concat( c.range( 3, 0 ), d.range( 3, 0 ) ) = a;
    cout << c << endl;
    cout << d << endl;

    b = concat( d.range( 3, 0 ), c.range( 3, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    concat( c.range( 3, 0 ), concat( d[0], d[1] ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = concat( c.range( 3, 0 ), concat( d[1], d[0] ) );
    cout << b << endl;

    cout << "*** sc_int_concat" << endl;

    concat( concat( c[1], c[0] ), d ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = concat( concat( c[0], c[1] ), d );
    cout << b << endl;

    cout << "---" << endl;

    concat( concat( c[1], c[0] ), d[0] ) = a.range( 2, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 2, 0 ) = concat( concat( c[0], c[1] ), d[0] );
    cout << b << endl;

    cout << "---" << endl;

    concat( concat( c[1], c[0] ), d.range( 3, 0 ) ) = a.range( 5, 0 );
    cout << c << endl;
    cout << d << endl;

    b.range( 5, 0 ) = concat( concat( c[0], c[1] ), d.range( 3, 0 ) );
    cout << b << endl;

    cout << "---" << endl;

    concat( concat( c[0], c[1] ), concat( c[2], c[3] ) ) = a.range( 3, 0 );
    cout << c << endl;

    b.range( 3, 0 ) = concat( concat( c[3], c[2] ), concat( c[1], c[0] ) );
    cout << b << endl;
}

int
sc_main( int, char*[] )
{
    sc_int<8> a( 33 );

    test_concat( a );

    return 0;
}
