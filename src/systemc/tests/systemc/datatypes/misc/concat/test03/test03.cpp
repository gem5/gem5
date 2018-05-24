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

  Original Author: Andy Goodrich, Forte Design Systems, 7 Apr 2005

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of l-value concatenation

#include "systemc.h"

int
sc_main( int, char*[] )
{
    sc_uint<2> uint2a;
    sc_int<2> int2a;
    const sc_uint<2> uint2c( "10" );
    const sc_uint<2> uint2d( "01" );
    sc_biguint<4> biguint4a;
    sc_bigint<4> bigint4a( "1111" );

    cout << "( uint2a, int2a ) = \"0110\"" << endl;
    ( uint2a, int2a ) = "0110";
    cout << "uint2a = " << uint2a << endl;
    cout << "int2a = " << int2a << endl;

    cout << "biguint4a = ( uint2a, int2a )" << endl;
    biguint4a = ( uint2a, int2a );
    cout << "biguint4a = " << biguint4a << endl;

    cout << "biguint4a = ( uint2c, uint2d )" << endl;
    biguint4a = ( uint2c, uint2d );
    cout << "biguint4a = " << biguint4a << endl;

    cout << "( biguint4a.range( 2, 1 ), uint2a ) = \"0110\"" << endl;
    ( biguint4a.range( 2, 1 ), uint2a ) = "0110";
    cout << "biguint4a = " << biguint4a << endl;
    cout << "uint2a = " << uint2a << endl;

#if 0
    cout << "( uint2a, biguint4a.range( 1, 2 ) ) = \"0101\"" << endl;
    ( uint2a, biguint4a.range( 1, 2 ) ) = "0101";
    cout << "biguint4a = " << biguint4a << endl;
    cout << "uint2a = " << uint2a << endl;
#endif // 0

    cout << "( biguint4a.range( 2, 1 ), bigint4a.range( 2, 1 ) ) = \"1100\"" << endl;
    ( biguint4a.range( 2, 1 ), bigint4a.range( 2, 1 ) ) = "1100";
    cout << "biguint4a = " << biguint4a << endl;
    cout << "bigint4a = " << bigint4a << endl;

    cout << "( ( uint2a, int2a ), biguint4a ) = \"00110011\"" << endl;
    ( ( uint2a, int2a ), biguint4a ) = "00110011";
    cout << "uint2a = " << uint2a << endl;
    cout << "int2a = " << int2a << endl;
    cout << "biguint4a = " << biguint4a << endl;

    cout << "( biguint4a, ( uint2a, int2a ) ) = \"11001100\"" << endl;
    ( biguint4a, ( uint2a, int2a ) ) = "11001100";
    cout << "biguint4a = " << biguint4a << endl;
    cout << "uint2a = " << uint2a << endl;
    cout << "int2a = " << int2a << endl;

    cout << "( ( uint2a, int2a ), ( biguint4a, bigint4a ) ) = \"011000001111\"" << endl;
    ( ( uint2a, int2a ), ( biguint4a, bigint4a ) ) = "011000001111";
    cout << "uint2a = " << uint2a << endl;
    cout << "int2a = " << int2a << endl;
    cout << "biguint4a = " << biguint4a << endl;
    cout << "bigint4a = " << bigint4a << endl;

    cout << "( ( uint2a, int2a ), biguint4a.range( 2, 1 ) ) = \"001111\"" << endl;
    ( ( uint2a, int2a ), biguint4a.range( 2, 1 ) ) = "001111";
    cout << "uint2a = " << uint2a << endl;
    cout << "int2a = " << int2a << endl;
    cout << "biguint4a = " << biguint4a << endl;

    cout << "( biguint4a.range( 2, 1 ), ( uint2a, int2a ) ) = \"001100\"" << endl;
    ( biguint4a.range( 2, 1 ), ( uint2a, int2a ) ) = "001100";
    cout << "biguint4a = " << biguint4a << endl;
    cout << "uint2a = " << uint2a << endl;
    cout << "int2a = " << int2a << endl;

    return 0;
}
