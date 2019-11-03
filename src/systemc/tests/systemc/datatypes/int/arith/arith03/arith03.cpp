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

  arith03.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <stdlib.h>
#include "systemc.h"
#include "isaac.h"

QTIsaac<8> rng;		// Platform independent random number generator.

#define TEST(A,B) \
{ \
	if ( A != B ) \
		cout << #A << " (" << hex << A << ") != " << #B << "(" << hex << B << ")" << endl; \
}

#define TESTA(A,B,C) \
{ \
	if ( A != (B-C) ) \
		cout << #A << " (" << hex << A << ") != " << hex << B-C << " " << B << " " << C << endl; \
}

int
sc_main( int argc, char* argv[] )
{
    unsigned vali[5] = { 0, 1, (unsigned) -1, 7, (unsigned)-8 };
    unsigned valj[5] = { 0, 1, (unsigned) -1, 7, (unsigned)-8 };

    for (int i = 3; i < 32; ++i) {
        for (int j = 3; j < 32; ++j) {
            cout << "i = " << i << ", j = " << j << endl;

            sc_unsigned x(i);
            sc_unsigned y(j);
            sc_unsigned z(64);

            vali[3] = (1 << (i - 1)) - 1;
            vali[4] = - (1 << (i - 1));

            valj[3] = (1 << (j - 1)) - 1;
            valj[4] = - (1 << (j - 1));

            for (int ii = 0; ii < 100; ++ii) {
                for (int jj = 0; jj < 100; ++jj) {
                    unsigned qi = (ii < 5) ? vali[ii] : (rng.rand() & ((1 << i) - 1));
                    unsigned qj = (jj < 5) ? valj[jj] : (rng.rand() & ((1 << j) - 1));

                    if (qi & (1 << (i - 1))) {
                        qi = (qi << (32 - i)) >> (32 - i);
                    }
                    if (qj & (1 << (j - 1))) {
                        qj = (qj << (32 - j)) >> (32 - j);
                    }

                    x = qi;
                    TEST(x,qi);
                    y = qj;
                    TEST(y,qj);
                    z = x + y;
                    TEST((sc_unsigned)( z.range(31,0) ),(qi + qj));
                    z = x - y;
                    // TEST((sc_unsigned)( z.range(31,0) ),(qi - qj));
                    TESTA((sc_unsigned)( z.range(31,0) ),qi,qj);
                    z = x * y;
                    TEST((sc_unsigned)( z.range(31,0) ),
			    (qi * qj) );
                    if (y != 0) {
                        z = x / y;
                        TEST((sc_unsigned)( z.range(31,0) ),(qi/qj));
                        z = x % y;
                        TEST((sc_unsigned)( z.range(31,0) ),(qi%qj));
                    }
                    z = x & y;
                    TEST( (sc_unsigned)( z.range(31,0) ), (qi & qj) );
                    z = x | y;
                    TEST((sc_unsigned)( z.range(31,0) ), (qi | qj) );
                    z = x ^ y;
                    TEST((sc_unsigned)( z.range(31,0) ), (qi ^ qj) );
                    if (jj < i - 1) {
                        z = x << jj;
                        for (int r = 0; r < i; ++r) {
                            TEST((bool) z[r],!!((qi << jj) & (1 << r)) );
                        }
                        z = x >> jj;
                        for (int r = 0; r < i; ++r) {
                            TEST((bool) z[r],!!((qi >> jj) & (1 << r)) );
                        }
                    }
                }
            }
        }
    }
    return 0;
}
