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

  array_range.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

                /*******************************************/
                /* Implementation Filename:  array_range.cc  */
                /*******************************************/
 
#include "array_range.h"
 
void
array_range::entry()
{
  bool_vector8	a;
  bool_vector4	b;
  bool_vector4	c;
  bool_vector8	d;
  bool_vector8	e;
  bool_vector8	f;
  // bool_vector0 	nullbv;	// Null vector to make scalar concat work

  wait();

  a = in1.read();

  b = a.range(7,4);	c = a.range(3,0);			// sub vectors
 
  d = a.range(0,7);						// bit reverse

  // e = (nullbv, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);	
  e = (a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);	
       						// bit reverse concat

  f = (a.range(3,1), a.range(7,6), a[0], a.range(4,5));		// shuffle

  o1.write(b);
  o2.write(c);
  o3.write(d);
  o4.write(e);
  o5.write(f);

  wait();

}
