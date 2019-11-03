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

  arith01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "systemc.h"

/*

 int9 x;

 x.q = -256;

 x.q is equal to 256 in Sun's CC, and -256 in gcc. The standard leaves
 this issue as implementation dependent unless the definition is
 qualified explicitly by signed or unsigned.  

 In order to force consistent behavior, I've qualified every int
 definition accordingly in all the arith files.  

*/

typedef struct int9 {
    signed int q : 9;
} int9;

typedef struct int31 {
    signed int q : 31;
} int31;

void
crunch(sc_signed& z, int31 v31, int u, int v)
{
    for (int i = 0; i < 100000; ++i) {
        z *= u;
        z += v;
        v31.q *= u;
        v31.q += v;
        sc_assert(z == v31.q);
    }
}

// Function to fix result in int9 struct to correctly under-/overflow
// within its 9 bits range and still ensure the correct sign encoding
// over the full size of the integer variable. Otherwise, compiler
// optimization may lead to spurious assertion errors.
void
fix_int9(int9& v) {
  v.q %= 0x200;
}

int
sc_main( int argc, char* argv[] )
{
    sc_signed x(31);
    sc_signed y(9);
    sc_signed z(31);
    int9 v;

    y = -256;
    v.q = -256;
    sc_assert(y == v.q);
    cout << y << '\t' << v.q << endl;
    for (int i = 0; i < 1000; ++i) {
        y++;
        v.q++;
        fix_int9(v);
        cout << y << '\t' << v.q << endl;
        sc_assert(y == v.q);
    }
    for (int i = 0; i < 1000; ++i) {
        y--;
        v.q--;
        fix_int9(v);
        cout << y << '\t' << v.q << endl;
        sc_assert(y == v.q);
    }
    for (int i = 0; i < 1000; ++i) {
        ++y;
        ++v.q;
        fix_int9(v);
        cout << y << '\t' << v.q << endl;
        sc_assert(y == v.q);
    }
    for (int i = 0; i < 1000; ++i) {
        --y;
        --v.q;
        fix_int9(v);
        cout << y << '\t' << v.q << endl;
        sc_assert(y == v.q);
    }

    z = 129023;
    int31 v31;
    v31.q = 129023;
    crunch(z, v31, 491, 12089);

    x = -129023;
    v31.q = -129023;
    crunch(x, v31, 109, -426);

    x = -1;
    v31.q = -1;
    crunch(x, v31, 30941, -1188);
    
    return 0;
}
