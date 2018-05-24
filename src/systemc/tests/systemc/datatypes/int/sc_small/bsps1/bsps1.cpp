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

  bsps1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int
sc_main(int ac, char* av[])
{
    sc_int<32> x;

    x = 0;
    x[1] = 1;
    x[2] = 1;
    x[3] = 0;
    x[4] = 0;
    x[5] = 1;

    sc_assert( x == 38 );
    cout << x << endl;

    x[5] = 0;
    x[31] = 1;

#if !defined( _MSC_VER )
    sc_assert((uint64)x == 0xffffffff80000006ULL);
#else
    sc_assert((uint64)x == 0xffffffff80000006ui64);
#endif
    cout << x << endl;

    x.range(31,5) = 10;
    sc_assert(x == 326);
    cout << x << endl;

    x = 0;
    x.range(21,7) = 9;
    sc_assert(x == 1152);
    cout << x << endl;

    return 0;
}
