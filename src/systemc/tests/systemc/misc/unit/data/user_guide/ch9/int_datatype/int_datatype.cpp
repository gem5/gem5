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

  int_datatype.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Main file for "int" data type */ 

#include "systemc.h"

int sc_main(int ac, char *av[])
{

// 1. DECLARATION SYNTAX
  int	a;  
  int	b;  
  
// 2. TYPE CONVERSION

  // int <- int
  a = 5;
  b = -12;

  cout 	<< "int \t<=\t int"
	<< "\n---------------------"
	<< "\nA = " << a << "\t\t 5"
	<< "\nB = " << b << "\t\t -12"
	<< "\n" << endl;

  // **** ADD MORE TYPE CONVERSIONS *****

// 3. OPERATORS
//    Supported operators:      ! && || ~ & ^ | + - * / % << >>
//                              &= ^= |= += -= *= /= %= <<= >>=
//                              = == != < <= > >= << >> 
//                              ()++ ++() ()-- --() ?: ,

#define VAL1	13
#define VAL2	 3

  unsigned int 	op1 = VAL1;
  unsigned int  op2 = VAL2;
  unsigned int 	r1, r2, r3, r4, r5, r6, r7, r8, r9; 
  unsigned int 	r10, r11, r12, r13, r14, r15, r16, r17, r18, r19;  
  unsigned int 	r20, r21, r22, r23, r24, r25, r26, r27, r28, r29;  
  unsigned int 	r30, r31, r32, r33, r34, r35;

  r1 = op1 * op2;			// Multiplication

  r2 = op1 / op2;			// Division

  r3 = op1 % op2;			// Modulus

  r4 = op1 + op2;			// Addition

  r5 = op1 - op2;			// Subtraction

  r6 = !op1;				// Logical NOT

  r7 = op1 && op2;			// Logical AND

  r8 = op1 || op2;			// Logical OR

  r9 = op1 < op2;			// Less than

  r10 = op1 <= op2;			// Less than or equal

  r11 = op1 > op2;			// Greater than

  r12 = op1 >= op2;			// Greater than or equal

  r13 = op1 += op2;			// Compound addition
    op1 = VAL1; op2 = VAL2;

  r14 = op1 -= op2;			// Compound subtraction
    op1 = VAL1; op2 = VAL2;

  r15 = op1 *= op2;			// Compound multiplication
    op1 = VAL1; op2 = VAL2;

  r16 = op1 /= op2;			// Compound division
    op1 = VAL1; op2 = VAL2;

  r17 = op1 %= op2;			// Compound modulus
    op1 = VAL1; op2 = VAL2;

  r18 = op1 <<= op2;			// Compound shift left 
    op1 = VAL1; op2 = VAL2;

  r19 = op1 >>= op2;			// Compound shift right 
    op1 = VAL1; op2 = VAL2;

  r20 = op1 &= op2;			// Compound bitwise AND 
    op1 = VAL1; op2 = VAL2;

  r21 = op1 ^= op2;			// Compound bitwise XOR 
    op1 = VAL1; op2 = VAL2;

  r22 = op1 |= op2;			// Compound bitwise OR 
    op1 = VAL1; op2 = VAL2;

  r23 = op2++;				// Postfix increment 
    op1 = VAL1; op2 = VAL2;

  r24 = ++op2;				// Prefix increment 
    op1 = VAL1; op2 = VAL2;

  r25 = op2--;				// Postfix decrement 
    op1 = VAL1; op2 = VAL2;

  r26 = --op2;				// Prefix decrement 
    op1 = VAL1; op2 = VAL2;

  r27 = (op1 > op2) ? true : false;	// Arithmetic if
  r28 = (op1 < op2) ? true : false;	// Arithmetic if

  r29 = op1, r29 = op2;		 	// Comma  

  r30 = ~op1;				// Bitwise NOT

  r31 = op1 << op2;			// Left shift 
    op1 = VAL1; op2 = VAL2;

  r32 = op1 >> op2;			// Right shift 
    op1 = VAL1; op2 = VAL2;

  r33 = op1 & op2;			// Bitwise AND 

  r34 = op1 ^ op2;			// Bitwise XOR 

  r35 = op1 | op2;			// Bitwise OR 

  cout 	<< "op1 \t operator \t op2 \t result  [All operands are int]"
	<< "\n----------------------------------------------------------------"
  	<< "\n" << op1 << "\t    * \t\t " << op2 << "\t = " << r1
  	<< "\n" << op1 << "\t    / \t\t " << op2 << "\t = " << r2
  	<< "\n" << op1 << "\t    % \t\t " << op2 << "\t = " << r3
  	<< "\n" << op1 << "\t    + \t\t " << op2 << "\t = " << r4
  	<< "\n" << op1 << "\t    - \t\t " << op2 << "\t = " << r5
  	<< "\n!(" << op1 << ") \t\t\t\t = " << r6 
  	<< "\n" << op1 << "\t    && \t\t " << op2 << "\t = " << r7
  	<< "\n" << op1 << "\t    || \t\t " << op2 << "\t = " << r8
  	<< "\n" << op1 << "\t    < \t\t "  << op2 << "\t = " << r9
  	<< "\n" << op1 << "\t    <= \t\t " << op2 << "\t = " << r10
  	<< "\n" << op1 << "\t    > \t\t "  << op2 << "\t = " << r11
  	<< "\n" << op1 << "\t    >= \t\t " << op2 << "\t = " << r12
  	<< "\n" << op1 << "\t    += \t\t " << op2 << "\t = " << r13 
  	<< "\n" << op1 << "\t    -= \t\t " << op2 << "\t = " << r14 
  	<< "\n" << op1 << "\t    *= \t\t " << op2 << "\t = " << r15 
  	<< "\n" << op1 << "\t    /= \t\t " << op2 << "\t = " << r16 
  	<< "\n" << op1 << "\t    %= \t\t " << op2 << "\t = " << r17 
  	<< "\n" << op1 << "\t    <<=\t\t " << op2 << "\t = " << r18 
  	<< "\n" << op1 << "\t    >>=\t\t " << op2 << "\t = " << r19 
  	<< "\n" << op1 << "\t    &= \t\t " << op2 << "\t = " << r20 
  	<< "\n" << op1 << "\t    ^= \t\t " << op2 << "\t = " << r21 
  	<< "\n" << op1 << "\t    |= \t\t " << op2 << "\t = " << r22 
  	<< "\n" << "\t    ()++ \t " << op2 << "\t = " << r23 
  	<< "\n" << "\t    ++() \t " << op2 << "\t = " << r24 
  	<< "\n" << "\t    ()-- \t " << op2 << "\t = " << r25 
  	<< "\n" << "\t    --() \t " << op2 << "\t = " << r26 
  	<< "\n" << op1 << "\t    > ?: \t " << op2 << "\t = " << r27 
  	<< "\n" << op1 << "\t    < ?: \t " << op2 << "\t = " << r28 
  	<< "\n" << op1 << "\t    , \t\t " << op2 << "\t = " << r29 
  	<< "\n~(" << op1 << ") \t\t\t\t = " << r30 
  	<< "\n" << op1 << "\t    << \t\t " << op2 << "\t = " << r31 
  	<< "\n" << op1 << "\t    >> \t\t " << op2 << "\t = " << r32 
  	<< "\n" << op1 << "\t    & \t\t " << op2 << "\t = " << r33 
  	<< "\n" << op1 << "\t    ^ \t\t " << op2 << "\t = " << r34 
  	<< "\n" << op1 << "\t    | \t\t " << op2 << "\t = " << r35 
 	<< endl;

  if (op1 == op2)			// Equality
   cout << op1 << "\t    == \t\t " << op2 << "\t -> true" << endl;
  else
   cout << op1 << "\t    == \t\t " << op2 << "\t -> false" << endl;

  if (op1 != op2)			// Inequality
   cout << op1 << "\t    != \t\t " << op2 << "\t -> true" << endl;
  else
   cout << op1 << "\t    != \t\t " << op2 << "\t -> false" << endl;

  op1 = op2 = 0;			// Assignment operator concatenation
   cout << op1 << "\t    = \t\t " << op2 << endl;
  sc_start(0, SC_NS);
  return 0;
}
