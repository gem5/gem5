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

// test of sc_dt::sc_string_old's pos() method -- star 124010

#ifdef SC_USE_STD_STRING
#   undef SC_USE_STD_STRING
#endif
#define SC_USE_SC_STRING_OLD
#include "systemc.h"

int
sc_main( int, char*[] )
{
    sc_dt::sc_string_old a( "aap noot mies" );

    cout << a.pos( "noot" ) << endl;
    cout << a.pos( "not" ) << endl;
    cout << a.pos( "" ) << endl;

    return 0;
}
