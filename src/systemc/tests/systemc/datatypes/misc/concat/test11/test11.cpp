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

  test11.cpp -- Test that sign extension and bit inversion works for the
                ~ operator when used in sc_XXsigned::concat_get_data()

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
    sc_bigint<20> w;
    sc_biguint<20> x;
    sc_biguint<30> y;

    w = 1;
    x = 1;
    y = (sc_bigint<21>)(sc_uint<1>(1),~w);
    cout << "sc_signed 1 " << hex << y << endl;
    y = (sc_bigint<21>)(sc_uint<1>(1),~x);
    cout << "sc_unsigned 1 " << hex << y << endl;

    w = -1;
    x = -1;
    y = (sc_bigint<21>)(sc_uint<1>(1),~w);
    cout << "sc_signed -1 " << hex << y << endl;
    y = (sc_bigint<21>)(sc_uint<1>(1),~x);
    cout << "sc_unsigned -1 " << hex << y << endl;

    w = 0x80000;
    x = 0x80000;
    y = (sc_bigint<21>)(sc_uint<1>(1),~w);
    cout << "sc_signed 0x800000 " << hex << y << endl;
    y = (sc_bigint<21>)(sc_uint<1>(1),~x);
    cout << "sc_unsigned 0x800000 " << hex << y << endl;

    w = 0;
    x = 0;
    y = (sc_bigint<21>)(sc_uint<1>(1),~w);
    cout << "sc_signed 0 " << hex << y << endl;
    y = (sc_bigint<21>)(sc_uint<1>(1),~x);
    cout << "sc_unsigned 0 " << hex << y << endl;

    cout << "Program completed" << endl;
    return 0;
}
