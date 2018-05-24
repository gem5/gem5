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

  arith10.cpp -- 

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

int
sc_main( int argc, char* argv[] )
{
    signed int vali[5] = { 0, 1, -1, 7, -8 };
    signed int valj[5] = { 0, 1, -1, 7, -8 };

    for (int i = 3; i < 32; ++i) {
        for (int j = 3; j < 32; ++j) {
            cout << "i = " << i << ", j = " << j << endl;

            sc_signed x(i);
            sc_signed y(j);
            sc_signed z(65), q(65);

            vali[3] = (1 << (i - 1)) - 1;
            vali[4] = - (1 << (i - 1));

            valj[3] = (1 << (j - 1)) - 1;
            valj[4] = - (1 << (j - 1));

            for (int ii = 0; ii < 100; ++ii) {
                for (int jj = 0; jj < 100; ++jj) {
                    signed int qi = (ii < 5) ? vali[ii] : (rng.rand() & ((1 << i) - 1));
                    signed int qj = (jj < 5) ? valj[jj] : (rng.rand() & ((1 << j) - 1));

                    if (qi & (1 << (i - 1))) {
                        qi = (qi << (32 - i)) >> (32 - i);
                    }
                    if (qj & (1 << (j - 1))) {
                        qj = (qj << (32 - j)) >> (32 - j);
                    }

                    x = qi;
                    y = qj;
                    z = x * y;
                    sc_assert( static_cast<sc_bigint<32> >( z.range(31,0) ) ==
                            (qi * qj) );
                    bool s;
                    s = ((x < 0) != (y < 0));
                    sc_signed x2(i+1);
                    x2 = x;
                    if (x < 0) {
                        x2 = - x;
                    }
                    sc_signed y2(j+1);
                    y2 = y;
                    if (y < 0) {
                        y2 = - y;
                    }

                    sc_unsigned xhi(16), xlo(16);
                    sc_unsigned yhi(16), ylo(16);
		    sc_unsigned zero(16);
		    zero = 0;
                    xlo = i > 14 ? x2.range(15,0) : x2.range(i,0);
                    xhi = i > 15 ? x2.range(i,16) : zero;
                    ylo = j > 14 ? y2.range(15,0) : y2.range(j,0);
                    yhi = j > 15 ? y2.range(j,16) : zero;
                    q = (xlo * ylo) +
                        (xhi * ylo + xlo * yhi) * 65536 +
                        ((xhi * yhi) * 65536) * 65536;
                    if (s)
                        q = - q;
                    if (z.range(63,0) != q.range(63,0)) {
                        cout << "xlo = " << xlo << endl;
                        cout << "xhi = " << xhi << endl;
                        cout << "ylo = " << ylo << endl;
                        cout << "yhi = " << yhi << endl;
                        sc_assert(false);
                    }
                }
            }
        }
    }
    return 0;
}
