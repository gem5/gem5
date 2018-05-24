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

  reduce.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

unsigned
reduce_imp(const sc_bv<8>& x)
{
    unsigned r = 0;
    r = (r << 1) | x.and_reduce();
    r = (r << 1) | x.nand_reduce();
    r = (r << 1) | x.or_reduce();
    r = (r << 1) | x.nor_reduce();
    r = (r << 1) | x.xor_reduce();
    r = (r << 1) | x.xnor_reduce();

    r = (r << 1) | and_reduce(x);
    r = (r << 1) | nand_reduce(x);
    r = (r << 1) | or_reduce(x);
    r = (r << 1) | nor_reduce(x);
    r = (r << 1) | xor_reduce(x);
    r = (r << 1) | xnor_reduce(x);

    return r;
}

int
sc_main(int argc, char* argv[])
{
    sc_bv<8> u;
    u = "10011011";
    cout << reduce_imp(u) << endl;
    u = "11101001";
    cout << reduce_imp(u) << endl;
    u = "01101001";
    cout << reduce_imp(u) << endl;
    u = "11111111";
    cout << reduce_imp(u) << endl;
    u = "00000000";
    cout << reduce_imp(u) << endl;

    return 0;
}
