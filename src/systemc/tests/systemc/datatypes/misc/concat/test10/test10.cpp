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

  test10.cpp -- Test that low order word of negative sc_signed value is
                properly masked when used in sc_signed::concat_get_data()

  Original Author: Andy Goodrich, Forte Design Systems, 29 April 2008

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int sc_main(int argc, char* argv[])
{

    sc_bigint<65> hi;
    hi[64] = 1; hi(63,32) = 0xffffe47b;
    hi[64] = 1; hi(63,32) = 0xffffe47b;
    sc_bigint<31> lo(1);
    sc_bigint<96> x;
    cout << "x = " << x.to_string(SC_BIN) << endl << endl;
    x = (hi,lo);
    cout  << hex << "hi = " << hi << endl
          << "lo = " << lo << endl
          << "x = " << x << endl;
    cout  << dec << "hi = " << hi.to_string(SC_BIN) << endl
          << "lo = " << lo.to_string(SC_BIN) << endl
          << "x = " << x.to_string(SC_BIN) << endl;
    cout << "Program completed" << endl;
    return 0;
}
