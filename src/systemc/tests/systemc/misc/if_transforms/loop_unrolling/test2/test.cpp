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
//      From 37.sc
//
//	Author: PRP
//	Date Created: 05 APR 99
//


#include "test.h"
 
void test::entry() 
{
  int i,j;
  int a[10],b[10];
   

  do { wait(); } while  (cont1 == 0);
  wait ();

  i = 0;
  wait ();
  for (; i < 3; /*i++*/) {
        a[i] = 11;
        i += 2;
	wait ();
  }
 
  for (i=0; i < 3; i ++) {
        a[i] = 1;
  }
 
 for (i=0; i < 3;) {
        a[i] = 2;
        ++i;
  }
 
  i = 0;
  for (; i < 3; ) {
        a[i] = 3;
        i += 2;
  }
 
  if (i) 
        j = 1;
  else 
        j = 3;
  
  wait();
  for (; i < 5;++i) {
        a[i] = 8;
	wait();
  }
  if (i) 
        i = 1;
  else 
        j = 3;
  i = 0;
  for (; i < 3;++i) {
        a[i] = 9;
  }
 
  i = j;
  if (j) {
        i = 0;
        for (; i < 3;++i) {
                a[i] = 10;
        }
  }
 
  for (j = 0; j < 2; j++) {
        i = 0;
        for (; i < 3;++i) {
                a[i] = 11;
        }
        b[j] = i;
  }

  wait();

}
 
