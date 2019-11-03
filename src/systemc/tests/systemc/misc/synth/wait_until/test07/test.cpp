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

/* From Test Case 51.sc */
void test::entry() 
{
  while (true) {

  do { wait(); } while  (cont1 == 0);
  wait();
  o1 = 0;
  o2 = 0;
  o3 = 0;
  o4 = 0;
  o5 = 0;
  wait ();
  if (i1 == 25) {
        if (i2 == 15) {
                  if (i3 == 5) {
                        wait ();
                  } else {
                        if (i4 == 1) {
                                wait ();
                                o5 = 2;
                        } else {
                                do { wait(); } while  (cont2 != 8);
                        }
                  }
                o1 = 9;
                o2 = 10;
        } else {
                wait();
                wait();
        }
        o3 = 5;
        o4 = 20;
        wait();
        wait();
  } else {
        wait();
  }
  o5 = 6;
  wait ();
  wait ();
  wait ();


  }
}

