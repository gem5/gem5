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
//	Verifies the functionality of concanetation operation.
//	Operands form the rvalue of an assignment
//
//	Author: PRP
//	Date Created: 19 Feb 99
//


#include "systemc.h"
#include "test.h"
 
void test::entry() 
{
  sc_lv<8> a;
  sc_lv<8> b;
  sc_lv<8> c;
  sc_lv<8> d;
  sc_lv<24> e;
  sc_lv<24> f;

  // sc_logic k;
  // sc_logic n;
  // sc_logic m, o, p, q, r, s, t, u;
  sc_lv<1> k, n, m, o, p, q, r, s, t, u;

  sc_lv<32> x;
  sc_lv<32> y;
  sc_lv<32> z;

  int i,j;
 
  while (true) {

  wait ();


  b = "00000001";	// 1
  c = "00000011";	// 3
  d = "00001111";	// 15
  
  // =============== Array + Array ====================================

  (a, f) = "00000000000000000000000000000011"; // a = 0, f = 3

  // =============== Cascading ====================================
  // cascading  array variables
  (a, f) = (b, c, d, a);
	// a = 00000001  f = "00000011 00001111 00000000"
  z = (a, f);	

  // composing cascaded  array variables
  x = ( sc_lv_base( "00000011" ), "00000011", "00000011", "00000101");	
  (a, (b, c, d)) = x;
  z = z | x;	// z = 00000011 00000011 00001111 00000101

  // =============== Array (variable) + Scalar ==============================

  (b, c, d, a.range (7, 2), k, n) = (a, b, c, d);
	// b = 00000011  c = 00000011  d = 00000011 a = 00000111
	// k = 0  n = 1
  z = z | (a, b, c, d); // z = 00000111 00000011 00001111 00000111

  ( k, m, a.range (5, 0)) = z.range (15, 8);
	// k = 0  m = 0  a = 00001111
  z = z | ( sc_lv_base( n ), m, k, n, "0000", a, "0000000000000000");
	// z = 10010111 00001111 00001111 00000111

  // =============== Null Vector ====================================

  ( m, n, o, p, q, r, s, t ) = "11011010";
  a = ( sc_lv_base( s ), t, q, r, o, m, n, p);	// a = "10100111"
  z = (z.range (31, 8), a); 	// z = 10010111 00001111 00001111 10100111

  b = "00000000";

  z =  z | (b, b, b, z.range (7, 0));

  // =============== LHS/RHS of different widths ==============================

  (x.range (15, 0), x.range (31, 16)) = ("1001011100001111000011111010011101");
  // RHS is (z, "01")
  // x = 00111110 10011101 01011100 00111100

  (z.range (31, 16), z.range (15, 0)) = (x.range (7, 0), x.range (23, 16),
					 x.range (15, 8), x.range (31, 24));
	// z = 00111100 10011101 01011100 00111110 
	
  o1 = z.to_int(); 
  wait();

  }
}
 
