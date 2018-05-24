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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
Hi,

Im using SystemC 2.0b2 with gcc 2.95.3 (Linux). If I create an sc_lv 
assignment with range methods on every side like

lv1.range(15,8) = lv2.range(7,0);

and the lv2 vector contains "z", I get the warning message

WARNING: (E2004) sc_bv cannot contain values X and Z :
 - ../../../../../src/sysc/datatypes/bit/sc_bv.h: 340

and the assignment is not done. Further investigation revealed that the 
vector seems to get mistakenly converted to a sc_bv in between. Removing the 
range method on one side or adding a cast

lv1.range(15,8) = (sc_lv<8>)lv2.range(7,0);

solves that problem.

Regards, S. Heithecker
*/

#include "systemc.h"

int
sc_main( int, char*[] )
{
    sc_lv<16> a( "01ZX0000111101ZX" );
    sc_lv<16> b;

    b.range( 15, 8 ) = a.range( 7, 0 );
    cout << a << endl;
    cout << b << endl;

    return 0;
}
