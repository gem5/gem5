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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "test.h"

/* From Test Case 57.sc */
void test::entry() 
{
  while (true) {

  do { wait(); } while  (cont1 != 1);
  wait ();
  o1 = 9;
 
  switch (i5) {
  case 1: 
        if (i2 > 4) {
                o2 = 80;
                do { wait(); } while  (cont2 != 1);
        } else {
                o2 = 81;
                wait();
        }
        break;
  case 2: 
        o2 = 9; 
        if (i4 == 5) {
                wait(); wait(); break;
        }
        else 
                wait(); 
        break;
  case 3: 
        o2 = 10;
        wait();
        while (i3 < 5) {
                wait();
                o3 = i4 + 1;
                if (i4 > 7) break;
                wait();
        }
        wait ();
        break;
   default: o2 = 11; wait(); break;
  }
  if (i3 == 3) {
        o1 = 4;
        wait();
  } else {
        o1 = 5;
        wait();
  }

  }
}

