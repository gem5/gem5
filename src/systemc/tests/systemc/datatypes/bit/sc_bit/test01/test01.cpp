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

  test01.cpp -- 

  Original Author: Joe Buck, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

void cover_sc_bit()
{
    sc_bit bdef;
    sc_bit bf(false);
    sc_bit bt(true);
    sc_bit b0(0);
    sc_bit b1(1);
    try {
	sc_bit foo(2);
    }
    catch (sc_report) {
	cout << "Caught exception for sc_bit(2)\n";
    }
    sc_bit bc0('0');
    sc_bit bc1('1');
    try {
	sc_bit foo('2');
    }
    catch (sc_report) {
	cout << "Caught exception for sc_bit('2')\n";
    }
    sc_bit blc0(sc_logic('0'));
    sc_bit blc1(sc_logic('1'));
    sc_bit blcx(sc_logic('X'));
    sc_bit bcop(bt);
    cout << bdef << bf << bt << b0 << b1 << bc0 << bc1 << blc0 << blc1
	 << blcx << bcop << endl;
    sc_bit b;
    b = bt;
    sc_assert(b);
    b = 0;
    sc_assert(!b);
    b = true;
    sc_assert(b.to_bool());
    b = '0';
    sc_assert(!b.to_bool());
    b = sc_logic('1');
    sc_assert(b.to_char() == '1');
    b = bf;
    sc_assert(~b);
    b |= bt;
    sc_assert(b);
    b &= bf;
    sc_assert(!b);
    b |= 1;
    sc_assert(b);
    b &= 0;
    sc_assert(!b);
    b |= '1';
    sc_assert(b);
    b &= '0';
    sc_assert(!b);
    b |= true;
    sc_assert(b);
    b &= false;
    sc_assert(!b);
    b ^= bt;
    sc_assert(b);
    b ^= 1;
    sc_assert(!b);
    b ^= '1';
    sc_assert(b);
    b ^= true;
    sc_assert(!b);

    sc_assert(b == bf);
    sc_assert(b == 0);
    sc_assert(b == '0');
    sc_assert(b == false);
    b = 1;
    sc_assert(b == bt);
    sc_assert(b == 1);
    sc_assert(b == '1');
    sc_assert(b == true);
    sc_assert(1 == b);
    sc_assert('1' == b);
    sc_assert(true == b);
    sc_assert(equal(b, bt));
    sc_assert(equal(b, 1));
    sc_assert(equal(b, '1'));
    sc_assert(equal(b, true));
    sc_assert(equal(1, b));
    sc_assert(equal('1', b));
    sc_assert(equal(true, b));
    b = 0;
    sc_assert(b != bt);
    sc_assert(b != 1);
    sc_assert(b != '1');
    sc_assert(b != true);
    sc_assert(1 != b);
    sc_assert('1' != b);
    sc_assert(true != b);
    sc_assert(not_equal(b, bt));
    sc_assert(not_equal(b, 1));
    sc_assert(not_equal(b, '1'));
    sc_assert(not_equal(b, true));
    sc_assert(not_equal(1, b));
    sc_assert(not_equal('1', b));
    sc_assert(not_equal(true, b));

    // the following assertion is incorrect, because the b_not() method
    // is destructive, i.e., it implements something like b ~= void.
    /// sc_assert(b == b_not(b.b_not()));
    b.b_not();
    sc_assert(b);
    sc_bit bx;
    b_not(bx, b0);
    sc_assert(bx);
    b_not(bx, b1);
    sc_assert(!bx);

    cout << (b0|b0) << (b0|b1) << (b1|b0) << (b1|b1) << endl;
    cout << (b0&b0) << (b0&b1) << (b1&b0) << (b1&b1) << endl;
    cout << (b0^b0) << (b0^b1) << (b1^b0) << (b1^b1) << endl;

    cout << (b0|0) << (b0|1) << (b1|0) << (b1|1) << endl;
    cout << (b0&0) << (b0&1) << (b1&0) << (b1&1) << endl;
    cout << (b0^0) << (b0^1) << (b1^0) << (b1^1) << endl;

    cout << (b0|'0') << (b0|'1') << (b1|'0') << (b1|'1') << endl;
    cout << (b0&'0') << (b0&'1') << (b1&'0') << (b1&'1') << endl;
    cout << (b0^'0') << (b0^'1') << (b1^'0') << (b1^'1') << endl;

    cout << (b0|true) << (b0|false) << (b1|true) << (b1|false) << endl;
    cout << (b0&true) << (b0&false) << (b1&true) << (b1&false) << endl;
    cout << (b0^true) << (b0^false) << (b1^true) << (b1^false) << endl;

    cout << (0|b0) << (0|b1) << (1|b0) << (1|b1) << endl;
    cout << (0&b0) << (0&b1) << (1&b0) << (1&b1) << endl;
    cout << (0^b0) << (0^b1) << (1^b0) << (1^b1) << endl;

    cout << ('0'|b0) << ('0'|b1) << ('1'|b0) << ('1'|b1) << endl;
    cout << ('0'&b0) << ('0'&b1) << ('1'&b0) << ('1'&b1) << endl;
    cout << ('0'^b0) << ('0'^b1) << ('1'^b0) << ('1'^b1) << endl;

    cout << (false|b0) << (false|b1) << (true|b0) << (true|b1) << endl;
    cout << (false&b0) << (false&b1) << (true&b0) << (true&b1) << endl;
    cout << (false^b0) << (false^b1) << (true^b0) << (true^b1) << endl;

    cout << b_or(b0,b0) << b_or(b0,b1) << b_or(b1,b0) << b_or(b1,b1) << endl;
    cout << b_and(b0,b0) << b_and(b0,b1) << b_and(b1,b0) << b_and(b1,b1)
         << endl;
    cout << b_xor(b0,b0) << b_xor(b0,b1) << b_xor(b1,b0) << b_xor(b1,b1)
         << endl;

    cout << b_or(b0,0) << b_or(b0,1) << b_or(b1,0) << b_or(b1,1) << endl;
    cout << b_and(b0,0) << b_and(b0,1) << b_and(b1,0) << b_and(b1,1) << endl;
    cout << b_xor(b0,0) << b_xor(b0,1) << b_xor(b1,0) << b_xor(b1,1) << endl;

    cout << b_or(b0,'0') << b_or(b0,'1') << b_or(b1,'0') << b_or(b1,'1')
         << endl;
    cout << b_and(b0,'0') << b_and(b0,'1') << b_and(b1,'0') << b_and(b1,'1')
         << endl;
    cout << b_xor(b0,'0') << b_xor(b0,'1') << b_xor(b1,'0') << b_xor(b1,'1')
         << endl;

    cout << b_or(b0,false) << b_or(b0,true) << b_or(b1,false) << b_or(b1,true)
         << endl;
    cout << b_and(b0,false) << b_and(b0,true) << b_and(b1,false)
         << b_and(b1,true) << endl;
    cout << b_xor(b0,false) << b_xor(b0,true) << b_xor(b1,false)
         << b_xor(b1,true) << endl;

    cout << b_or(0,b0) << b_or(0,b1) << b_or(1,b0) << b_or(1,b1) << endl;
    cout << b_and(0,b0) << b_and(0,b1) << b_and(1,b0) << b_and(1,b1) << endl;
    cout << b_xor(0,b0) << b_xor(0,b1) << b_xor(1,b0) << b_xor(1,b1) << endl;

    cout << b_or('0',b0) << b_or('0',b1) << b_or('1',b0) << b_or('1',b1)
         << endl;
    cout << b_and('0',b0) << b_and('0',b1) << b_and('1',b0) << b_and('1',b1)
         << endl;
    cout << b_xor('0',b0) << b_xor('0',b1) << b_xor('1',b0) << b_xor('1',b1)
         << endl;

    cout << b_or(false,b0) << b_or(false,b1) << b_or(true,b0) << b_or(true,b1)
         << endl;
    cout << b_and(false,b0) << b_and(false,b1) << b_and(true,b0)
         << b_and(true,b1) << endl;
    cout << b_xor(false,b0) << b_xor(false,b1) << b_xor(true,b0)
         << b_xor(true,b1) << endl;

    b_or(b, b0, b1);
    sc_assert(b);
    b_and(b, b0, b1);
    sc_assert(!b);
    b_xor(b, b0, b1);
    sc_assert(b);
}

int
sc_main(int, char*[])
{
    cover_sc_bit();
    return 0;
}

