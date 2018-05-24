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
//	Verifies loop unrolling
//
//      Test Plan: 5.2
//
//	Author: PRP
//	Date Created: 19 Feb 99
//


#include "test.h"
 
void test::entry() 
{
  int i,j;
  int a[10],b[10];
   

  do { wait(); } while  (cont1 == 0);
  wait ();

  i = 0;
  while (i < 4) {
        a[i] = 0;
        i = i + 1;
  }
 
  i = 0;
  while (i <= 4) {
        b[i] = 10;
        i = i + 1;
  }

  i = 9;
  while (i > 4) {
        a[i] = 20;
        i = i - 1;
  }
 
  i = 9;
  while (i >= 4) {
        b[i] = 30;
        i = i - 1;
  }
 
  i = -4;
  while (i < 0) {
        a[i+4] = 40;
        i = i + 1;
  }
 
 
  i = -4;
  while (i < 0) {
        a[i+4] = 50;
        i = i + 2;
  }
 
  i = -4;
  while (i <= 0) {
        a[i+4] = 60;
        i = i + 2;
  }
 
  i = -4;
  while (i <= 0) {
        a[i+4] = 70;
        i = i + 3;
  }
  i = -4;
  while (i <= 0) {
        a[i+4] = 80;
        i = i + 4;
  }
 
  i = -6;
  while (i <= 0) {
        a[i+6] = 90;
        i = i + 5;
  }
 
  i = 8;
  if (i) {
        j = 9;
  }
 
  while (i <= 10) {
        a[i] = 80;
        i = i + 5;
  }
 
  for (i = 0; i < 2; i++)
        a[i] = 8;
 
  i = 0;
  for (; i < 3; i++)
        a[i] = 10;
 
  i = 0;
  wait();
  for (; i < 3; ++i) {
        a[i] = 11;
        //i = i + 1;
  wait();
  }

   i = 0;
  for (; i < 3; i++)
        a[i] = 12;
 
  wait();

}
 
