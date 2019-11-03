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

  adder_sub.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename adder_sub.cc */
/* This is the implementation file for synchronous process `adder_sub' */

#include "adder_sub.h"

int add(int a, int b)
{
  return (a + b);
}

void adder_sub::entry()
{
  int sum;
  int a, b, c, d;

  while (true) {
    // Wait until you get signal to go
    do { wait(); } while (adder_sub_ready != true);
    // Read inputs
    a = Sa.read();
    b = Sb.read();
    c = Sc.read();
    
    // Perform the computation.
    sum = add(a, b);
    sum = add(sum, c);
    d = a - b;

    // Write outputs
    adder_sub_done.write(true);
    Ssum.write(sum);
    Sd.write(d);
    wait();
    adder_sub_done.write(false);
    // Loop back to do { wait(); } while . 
  }

} // end of entry function

