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

  test1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

void
one_to_four( const sc_bv<4>& x )
{
    cout << "x[3] = " << x[3] << endl;
    cout << "x[2] = " << x[2] << endl;
    cout << "x[1] = " << x[1] << endl;
    cout << "x[0] = " << x[0] << endl;
}

int
sc_main( int argc, char* argv[] )
{
    sc_bv<10> b1;
    sc_bv<8>  b2;
    sc_bv<7>  b3;

    sc_biguint<10> u1;
    sc_biguint<8>  u2;
    sc_biguint<7>  u3;

    sc_bigint<10> s1;
    sc_bigint<8>  s2;
    sc_bigint<7>  s3;

    b1 = "0010110101";
    b2 = "10010011";
    b3 = "1011011";

    cout << "b1.range(5,2) ... " << endl;
    one_to_four(b1.range(5,2));
    cout << "b2.range(4,1) ... " << endl;
    one_to_four(b2.range(4,1));
    cout << "b3.range(6,3) ... " << endl;
    one_to_four(b3.range(6,3));

    u1 = 235;
    u2 = 67;
    u3 = 44;

    s1 = -235;
    s2 = -32;
    s3 = -1;

    cout << "b1 = " << b1 << endl;
    cout << "b2 = " << b2 << endl;
    cout << "b3 = " << b3 << endl;
    
    cout << "u1 = " << u1 << endl;
    cout << "u2 = " << u2 << endl;
    cout << "u3 = " << u3 << endl;

    cout << "s1 = " << s1 << endl;
    cout << "s2 = " << s2 << endl;
    cout << "s3 = " << s3 << endl;

    cout << "b1.range(3,0) = " << b1.range(3,0) << endl;
    cout << "b1.range(0,3) = " << b1.range(0,3) << endl;
    cout << "b2.range(4,1) = " << b2.range(4,1) << endl;
    cout << "b2.range(1,4) = " << b2.range(1,4) << endl;
    cout << "b3.range(5,3) = " << b3.range(5,3) << endl;
    cout << "b3.range(3,5) = " << b3.range(3,5) << endl;

    cout << "u1.range(3,0) = " << sc_unsigned(u1.range(3,0)) << endl;
    cout << "u1.range(0,3) = " << sc_unsigned(u1.range(0,3)) << endl;
    cout << "u2.range(4,1) = " << sc_unsigned(u2.range(4,1)) << endl;
    cout << "u2.range(1,4) = " << sc_unsigned(u2.range(1,4)) << endl;
    cout << "u3.range(5,3) = " << sc_unsigned(u3.range(5,3)) << endl;
    cout << "u3.range(3,5) = " << sc_unsigned(u3.range(3,5)) << endl;
    cout << "u3.range(6,3) = " << sc_unsigned(u3.range(6,3)) << endl;
    cout << "u3 = " << u3 << endl;

    cout << "s1.range(3,0) = " << sc_signed(s1.range(3,0)) << endl;
    cout << "s1.range(0,3) = " << sc_signed(s1.range(0,3)) << endl;
    cout << "s2.range(4,1) = " << sc_signed(s2.range(4,1)) << endl;
    cout << "s2.range(1,4) = " << sc_signed(s2.range(1,4)) << endl;
    cout << "s3.range(5,3) = " << sc_signed(s3.range(5,3)) << endl;
    cout << "s3.range(3,5) = " << sc_signed(s3.range(3,5)) << endl;
    cout << "s3.range(6,3) = " << sc_signed(s3.range(6,3)) << endl;
    cout << "s3 = " << s3 << endl;

    u1 = b1;
    u2 = b2;
    u3 = b3;

    s1 = b1;
    s2 = b2;
    s3 = b3;

    cout << "u1.range(3,0) = " << sc_unsigned(u1.range(3,0)) << endl;
    cout << "u1.range(0,3) = " << sc_unsigned(u1.range(0,3)) << endl;
    cout << "u2.range(4,1) = " << sc_unsigned(u2.range(4,1)) << endl;
    cout << "u2.range(1,4) = " << sc_unsigned(u2.range(1,4)) << endl;
    cout << "u3.range(5,3) = " << sc_unsigned(u3.range(5,3)) << endl;
    cout << "u3.range(3,5) = " << sc_unsigned(u3.range(3,5)) << endl;
    cout << "u3.range(6,3) = " << sc_unsigned(u3.range(6,3)) << endl;
    cout << "u3 = " << u3 << endl;

    cout << "s1.range(3,0) = " << sc_signed(s1.range(3,0)) << endl;
    cout << "s1.range(0,3) = " << sc_signed(s1.range(0,3)) << endl;
    cout << "s2.range(4,1) = " << sc_signed(s2.range(4,1)) << endl;
    cout << "s2.range(1,4) = " << sc_signed(s2.range(1,4)) << endl;
    cout << "s3.range(5,3) = " << sc_signed(s3.range(5,3)) << endl;
    cout << "s3.range(3,5) = " << sc_signed(s3.range(3,5)) << endl;
    cout << "s3.range(6,3) = " << sc_signed(s3.range(6,3)) << endl;
    cout << "s3 = " << s3 << endl;

    return 0;
}
