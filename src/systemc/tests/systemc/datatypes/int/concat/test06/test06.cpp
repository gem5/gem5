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

  test06.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_[u]int concatenation with type bool, concat()

#include "systemc.h"

#define WRITE(a) \
    cout << a << endl

int
sc_main( int, char*[] )
{
    bool b;

    // )))  sc_uint  (((

    sc_uint<3> ui3 = 3;
    sc_uint<4> ui4;
    sc_uint<2> ui2;

    // sc_uint_base

    b = false;
    ui4 = concat( ui3, b );
    WRITE( ui4 );
    ui4 = concat( b, ui3 );
    WRITE( ui4 );

    b = true;
    ui4 = concat( ui3, b );
    WRITE( ui4 );
    ui4 = concat( b, ui3 );
    WRITE( ui4 );

    // sc_uint_bitref

    cout << endl;

    b = false;
    ui2 = concat( ui3[0], b );
    WRITE( ui2 );
    ui2 = concat( b, ui3[0] );
    WRITE( ui2 );

    b = true;
    ui2 = concat( ui3[0], b );
    WRITE( ui2 );
    ui2 = concat( b, ui3[0] );
    WRITE( ui2 );

    // sc_uint_subref

    cout << endl;

    b = false;
    ui2 = concat( ui3.range( 0, 0 ), b );
    WRITE( ui2 );
    ui2 = concat( b, ui3.range( 0, 0 ) );
    WRITE( ui2 );

    b = true;
    ui2 = concat( ui3.range( 0, 0 ), b );
    WRITE( ui2 );
    ui2 = concat( b, ui3.range( 0, 0 ) );
    WRITE( ui2 );

    // sc_uint_concat

    cout << endl;

    b = false;
    ui4 = concat( concat( ui3.range( 2, 1 ), ui3[0] ), b );
    WRITE( ui4 );
    ui4 = concat( b, concat( ui3.range( 2, 1 ), ui3[0] ) );
    WRITE( ui4 );

    b = true;
    ui4 = concat( concat( ui3.range( 2, 1 ), ui3[0] ), b );
    WRITE( ui4 );
    ui4 = concat( b, concat( ui3.range( 2, 1 ), ui3[0] ) );
    WRITE( ui4 );


    // )))  sc_int  (((

    sc_int<3> i3 = 3;
    sc_int<4> i4;
    sc_int<2> i2;

    // sc_int_base

    cout << endl;

    b = false;
    i4 = concat( i3, b );
    WRITE( i4 );
    i4 = concat( b, i3 );
    WRITE( i4 );

    b = true;
    i4 = concat( i3, b );
    WRITE( i4 );
    i4 = concat( b, i3 );
    WRITE( i4 );

    // sc_int_bitref

    cout << endl;

    b = false;
    i2 = concat( i3[0], b );
    WRITE( i2 );
    i2 = concat( b, i3[0] );
    WRITE( i2 );

    b = true;
    i2 = concat( i3[0], b );
    WRITE( i2 );
    i2 = concat( b, i3[0] );
    WRITE( i2 );

    // sc_int_subref

    cout << endl;

    b = false;
    i2 = concat( i3.range( 0, 0 ), b );
    WRITE( i2 );
    i2 = concat( b, i3.range( 0, 0 ) );
    WRITE( i2 );

    b = true;
    i2 = concat( i3.range( 0, 0 ), b );
    WRITE( i2 );
    i2 = concat( b, i3.range( 0, 0 ) );
    WRITE( i2 );

    // sc_int_concat

    cout << endl;

    b = false;
    i4 = concat( concat( i3.range( 2, 1 ), i3[0] ), b );
    WRITE( i4 );
    i4 = concat( b, concat( i3.range( 2, 1 ), i3[0] ) );
    WRITE( i4 );

    b = true;
    i4 = concat( concat( i3.range( 2, 1 ), i3[0] ), b );
    WRITE( i4 );
    i4 = concat( b, concat( i3.range( 2, 1 ), i3[0] ) );
    WRITE( i4 );

    return 0;
}
