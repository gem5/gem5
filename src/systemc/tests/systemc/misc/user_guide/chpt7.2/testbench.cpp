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

  testbench.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename testbench.cc */
/* This is the implementation file for synchronous process `testbench' */

#include <math.h>
#include "testbench.h"
#include "isaac.h"

QTIsaac<8> rng;

void testbench::entry()
{
  int a, b, c, d;
  int sum;
  int i;

  for (i=0; i < 10; i++) {
    a = rng.rand() & 0x0fffffff;
    b = rng.rand() & 0x0fffffff;
    c = rng.rand() & 0x0fffffff;

    Sa.write(a);
    Sb.write(b);
    Sc.write(c);
    adder_sub_ready.write(true);
    wait();
    adder_sub_ready.write(false);
    do { wait(); } while (adder_sub_done != true);
    sum = Ssum.read();
    d = Sdiff.read();
    // printf("A = %d, B = %d, C = %d, D = %d, SUM = %d\n", a, b, c, d, sum);
    char buf[BUFSIZ];
    sprintf(buf, "A = %d, B = %d, C = %d, D = %d, SUM = %d\n", a, b, c, d, sum);
    cout << buf;
  }
  sc_stop();

} // end of entry function

