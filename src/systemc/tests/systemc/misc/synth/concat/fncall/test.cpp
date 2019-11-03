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
//	Operands are arguments to a function
//
//	Author: PRP
//	Date Created: 19 Feb 99
//


#include "systemc.h"
#include "test.h"
 
sc_lv_base AND_fn (const sc_lv_base &a, const sc_lv_base &b)
{
  return a | b;
}
 
sc_lv_base OR_fn (const sc_lv_base &a, const sc_lv_base &b)
{
  return a & b;
}
 
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
  sc_logic m, o, p, q, r, s, t, u;

  sc_lv<32> x;
  sc_lv<32> y;
  sc_lv<32> z;
  sc_lv<32> z1;


  while (true) {

  wait ();

  a = "00000000";  	// 0
  b = "00000001";	// 1
  c = "00000011";	// 3
  d = "00001111";	// 15
  e = "000000000000000000000001"; // 1
  f = "000000000000000000001010"; // 10
  
  // =============== Array + Array ====================================
  // variable + variable, cascading
  x = OR_fn ((a, b, c, d), (d, c, b, a));
  // x = 00001111 00000011 00000011 00001111

  // variable + constant, cascading, composition
  y = AND_fn  (((a, b, c), "00000000"), (d, "00000000", b, a));
  // y = 00000000 00000000 00000001 00000000

  // constant + constant
  z = OR_fn (( sc_lv_base( "0000000000000000" ), "1000000000000000"), x);
  // z = 00001111 00000011 10000011 00001111

  z = z & (~y);
  // z = 00001111 00000011 10000010 00001111

  // =============== Array (variable) + Scalar ==============================

  n = '1';
  o = '0';
  z = OR_fn (z, ( sc_lv_base( n ), "000000000000000", "0000000000000000"));
  // z = 10001111 00000011 10000010 00001111

  z =  OR_fn (z, ( sc_lv_base( "00000000" ), o, n, d, "00000000000000"));
  // z = 10001111 01000011 11000010 00001111

  k = '1';
  z =  OR_fn (z, ( sc_lv_base( "00000000" ), "00", k, "00000", "0000000000000000"));
  // z = 10001111 01100011 11000010 00001111

  // =============== Null Vector ====================================

  z = AND_fn (z, (( sc_lv_base( o ), sc_lv_base( k ), "111111"), "111111111111111111111111"));
  // z = 00001111 01100011 11000010 00001111

  // =============== LHS/RHS of different widths ==============================

  z1 = OR_fn (z, sc_lv_base( "00000000000000000000000000000000" )); // length of string = 32

  o1 = z.to_int();  // o1 = 00001111 01100011 11000010 00001111
  wait();

  }
}
 
