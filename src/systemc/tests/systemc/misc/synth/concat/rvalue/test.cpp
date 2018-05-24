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

  sc_logic k;
  sc_logic n;
  sc_logic m;

  sc_lv<32> x;
  sc_lv<32> y;
  sc_lv<32> z;

  sc_lv<2> kk;

  int i,j;
 
  while (true) {

  wait ();

  // ------- rvalue ---------------------------------------------

  a = "00000000";  	// 0
  b = "00000001";	// 1
  c = "00000011";	// 3
  d = "00001111";	// 15
  e = "000000000000000000000001"; // 1
  f = "000000000000000000001010"; // 10
  
  // =============== Array + Array ====================================
  // array constant +  array constant
  x = ( sc_lv_base( "000000000000000000000000" ), "00010000");	// x = 32

  //  array constant +  array variable
  y = ("000000000000000000000000", b);	// y = 1
  z = x | y;	// z = 00000000 00000000 00000000 00010001

  //  array variable +  array constant
  x = (a, "000000000000000000000011");	// x = 3
  z = z & x;	// z = 00000000 00000000 00000000 00000001

  //  array variable +  array variable
  x = (a, f);	// x = 10
  z = z | x;	// z = 00000000 00000000 00000000 00001011

  // =============== Cascading ====================================
  // cascading  array variables
  x = (a, b, c, d);	// x = 00000000 00000001 00000011 00001111
  z = z & x;		// z = 00000000 00000000 00000000 00001011
		
  // cascading  array constants
  x = ( sc_lv_base( "00000011" ), "00000011", "00000011", "00000011");	
  		// x = 00000011 00000011 00000011 00000011
  z = z | x;	// z = 00000011 00000011 00000011 00001011

  // composing  array concats
  x = ( sc_lv_base( "00000011" ), ( sc_lv_base( "11111111" ), "00000011", "00000011"));
  		// x = 00000011 11111111 00000011 00000011
  z = z | x;	// z = 00000011 11111111 00000011 00001011

  // =============== Array (variable) + Scalar ==============================
  // array variable + scalar constant
  m = '0';
  n = '1';
  x = (a, b, c, d.range (6, 0), m);
		// x = 00000000 00000001 00000011 00011110
  z = z | x;	// z = 00000011 11111111 00000011 00011111

  k = '1';
  // array variable + scalar variable
  x = (a, b, k, c.range (6, 0), x.range (7, 0));
		// x = 00000000 00000001 10000001 00011110
  z = z & x;	// z = 00000000 00000001 00000001 00011110

  // =============== Null Vector ====================================
  // null vector - variable
  kk = ~( sc_lv_base( k ), k); // "00" 
  z = (z.range (31, 2), kk);	// z = 00000000 00000001 00000001 00011100

  // null vector - constant
  kk = ( sc_lv_base( n ), n); // "11"
  z = (kk, z.range (29, 0));	// z = 11000000 00000001 00000001 00011100

  // =============== Array (constant) + Scalar ==============================
  // scalar constant + array constant
  x = ( sc_lv_base( n ), "1111111000000000000000000000011");
		// x = 01111111 00000000 00000000 00000011
  z = z | x;	// z = 11111111 00000001 00000001 00011111

  // array constant + scalar variable
  x = ( sc_lv_base( "1111111000000000000000000000011" ), k);
		// x = 11111110 00000000 00000000 00000111
  z = z & x;	// z = 11111110 00000000 00000000 00000111

  // =============== LHS/RHS of different widths ==============================
  // lhs and rhs of different widths
  x = "100001111000000000000000000001111"; // warning should be issued
  z = z & x;
	// z = 00001110 00000000 00000000 00000111

  o1 = z.to_int();
  wait();

  }
}
 
