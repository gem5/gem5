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
    sc_bv<2> bv2a;
    sc_lv<2> lv2b;
    const sc_bv<2> bv2c( "10" );
    const sc_bv<2> bv2d( "01" );
    sc_bv<4> bv4a;

    ( bv2a, lv2b ) = "0110";
    cout << "bv2a = " << bv2a << endl;
    cout << "lv2b = " << lv2b << endl;

    bv4a = ( bv2a, lv2b );
    cout << "bv4a = " << bv4a << endl;

    bv4a = ( bv2c, bv2d );
    cout << "bv4a = " << bv4a << endl;

    ( bv4a.range( 2, 1 ), bv2a ) = "0110";
    cout << "bv4a = " << bv4a << endl;
    cout << "bv2a = " << bv2a << endl;

    ( bv2a, bv4a.range( 1, 2 ) ) = "0101";
    cout << "bv4a = " << bv4a << endl;
    cout << "bv2a = " << bv2a << endl;

    ( ( bv2a, lv2b ), bv4a ) = "00110011";
    cout << "bv2a = " << bv2a << endl;
    cout << "lv2b = " << lv2b << endl;
    cout << "bv4a = " << bv4a << endl;

    ( bv4a, ( bv2a, lv2b ) ) = "11001100";
    cout << "bv4a = " << bv4a << endl;
    cout << "bv2a = " << bv2a << endl;
    cout << "lv2b = " << lv2b << endl;

    ( ( bv2a, lv2b ), bv4a.range( 2, 1 ) ) = "001111";
    cout << "bv2a = " << bv2a << endl;
    cout << "lv2b = " << lv2b << endl;
    cout << "bv4a = " << bv4a << endl;

    ( bv4a.range( 2, 1 ), ( bv2a, lv2b ) ) = "001100";
    cout << "bv4a = " << bv4a << endl;
    cout << "bv2a = " << bv2a << endl;
    cout << "lv2b = " << lv2b << endl;

    return 0;
}
