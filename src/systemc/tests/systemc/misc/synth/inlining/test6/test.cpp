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


//
//      Verifies function inlining
//
//      Author: PRP
//      Date Created: 26 Feb 99
//
 
#include "systemc.h"
#include "test.h"

void test::entry() 
{
  int i,j,h, temp;
 
  wait ();
  j = i1.read();
  for (i = 0; i < 4; i = i + 1)
  {  // Default: no unrolling
    j = j + 1;
    wait();
  }
  wait ();
  temp = 4+j;
  modify (o1, temp); 
  h = (i1 > i2) ? i3 : i4;
  o2 = h;
  wait ();
  i= 9;
  noModify (i);
  wait();
}
 

void test::modify (sc_signal<int>& i, int& j)
{
  i = i1 + j;
}
 
void test::noModify (int i)
{
  o3 = i;
}
