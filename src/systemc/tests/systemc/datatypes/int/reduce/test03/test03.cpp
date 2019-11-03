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

  test03.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems 2006-08-17
  Description of Modification: Converted over to test sc_big types.

 *****************************************************************************/

// test of the reduce methods in the sc_big[u]int datatypes -- method notation

#include "systemc.h"

int
sc_main( int, char*[] )
{
    // 1) check the existence of the reduce methods

    sc_bigint<42> a = 42;

    // sc_signed
    cout << endl;
    cout << a.and_reduce() << endl;
    cout << a.or_reduce() << endl;
    cout << a.xor_reduce() << endl;
    cout << a.nand_reduce() << endl;
    cout << a.nor_reduce() << endl;
    cout << a.xnor_reduce() << endl;

    // sc_signed_subref
    cout << endl;
    cout << a( 7, 0 ).and_reduce() << endl;
    cout << a( 7, 0 ).or_reduce() << endl;
    cout << a( 7, 0 ).xor_reduce() << endl;
    cout << a( 7, 0 ).nand_reduce() << endl;
    cout << a( 7, 0 ).nor_reduce() << endl;
    cout << a( 7, 0 ).xnor_reduce() << endl;

    // concatenation
    cout << endl;
    cout << ( a( 7, 0 ), a( 15, 8 ) ).and_reduce() << endl;
    cout << ( a( 7, 0 ), a( 15, 8 ) ).or_reduce() << endl;
    cout << ( a( 7, 0 ), a( 15, 8 ) ).xor_reduce() << endl;
    cout << ( a( 7, 0 ), a( 15, 8 ) ).nand_reduce() << endl;
    cout << ( a( 7, 0 ), a( 15, 8 ) ).nor_reduce() << endl;
    cout << ( a( 7, 0 ), a( 15, 8 ) ).xnor_reduce() << endl;

    sc_biguint<42> b = 42;

    // sc_unsigned
    cout << endl;
    cout << b.and_reduce() << endl;
    cout << b.or_reduce() << endl;
    cout << b.xor_reduce() << endl;
    cout << b.nand_reduce() << endl;
    cout << b.nor_reduce() << endl;
    cout << b.xnor_reduce() << endl;

    // sc_unsigned_subref
    cout << endl;
    cout << b( 7, 0 ).and_reduce() << endl;
    cout << b( 7, 0 ).or_reduce() << endl;
    cout << b( 7, 0 ).xor_reduce() << endl;
    cout << b( 7, 0 ).nand_reduce() << endl;
    cout << b( 7, 0 ).nor_reduce() << endl;
    cout << b( 7, 0 ).xnor_reduce() << endl;

    // concatenation
    cout << endl;
    cout << ( b( 7, 0 ), b( 15, 8 ) ).and_reduce() << endl;
    cout << ( b( 7, 0 ), b( 15, 8 ) ).or_reduce() << endl;
    cout << ( b( 7, 0 ), b( 15, 8 ) ).xor_reduce() << endl;
    cout << ( b( 7, 0 ), b( 15, 8 ) ).nand_reduce() << endl;
    cout << ( b( 7, 0 ), b( 15, 8 ) ).nor_reduce() << endl;
    cout << ( b( 7, 0 ), b( 15, 8 ) ).xnor_reduce() << endl;

    // 2) check the functionality of the reduce methods

    sc_bigint<2> c2 = -1;
    cout << endl;
    cout << c2.and_reduce() << endl;
    cout << c2.xor_reduce() << endl;

    sc_bigint<3> c3 = -1;
    cout << endl;
    cout << c3.and_reduce() << endl;
    cout << c3.xor_reduce() << endl;

    sc_biguint<2> d2 = sc_dt::uint_type( -1 );
    cout << endl;
    cout << d2.and_reduce() << endl;
    cout << d2.xor_reduce() << endl;

    sc_biguint<3> d3 = sc_dt::uint_type( -1 );
    cout << endl;
    cout << d3.and_reduce() << endl;
    cout << d3.xor_reduce() << endl;

    sc_bigint<6> e;
    sc_biguint<6> f;
    cout << endl;
    for( int i = 0; i >= -10; -- i ) {
        e = i;
        f = i;
        cout << e.xor_reduce() << endl;
        cout << f.xor_reduce() << endl;
    }
    
    return 0;
}
