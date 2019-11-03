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
I found this while trying to improve coverage of the SystemC regression tests.

Compile the following program, bug.cpp, with

g++ -g -I /u/scp/src/systemc-2.0/include bug.cpp /u/scp/src/systemc-2.0/lib-gccsparcOS5/libsystemc.a
(in mountain view, elsewhere use SYSTEMC_HOME pointing to the 2.0 release)

-------------------------------------------------------------------------------
*/

#define SC_INCLUDE_FX 1
#include "systemc.h"

double bug()
{
    sc_fxval_fast fast(2);
    sc_fxval slow(1);
    fast  = slow;
    fast += slow;
    fast -= slow;
    fast *= slow;
    fast /= slow;
    return fast.to_double();
}

int sc_main(int, char*[])
{
    cout << bug() << endl;
    return 0;
}

/*
-------------------------------------------------------------------------------
It fails to link, giving the message

Undefined                       first referenced
 symbol                             in file
sc_fxval_fast::operator+=(sc_fxval const &)      /var/tmp/ccNbIiHN.o
sc_fxval_fast::operator-=(sc_fxval const &)       /var/tmp/cclXNLsO.o
sc_fxval_fast::operator/=(sc_fxval const &)      /var/tmp/cclXNLsO.o
sc_fxval_fast::operator*=(sc_fxval const &)       /var/tmp/cclXNLsO.o
*/

